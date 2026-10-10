// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! CANalyst-II (Waveshare USB-CAN-B) device: a full NMEA 2000 or J1939
//! node on the adapter's CAN channel, like [`super::socketcan`] — it
//! claims an address, answers ISO Requests, sends the Heartbeat and
//! Product Information, and frames what the application sends. The
//! node itself is [`super::iso11783_node`]; this module moves its frames
//! over the 64-byte packets of [`crate::engine::codec::canalyst`].
//!
//! A reader thread turns packets into CAN frames; a worker thread owns
//! the node and the writer, and takes both the received frames and the
//! application's commands from one queue, so it sleeps until there is
//! something to do or a timer is due.

use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::Duration;

use smallvec::SmallVec;

use crate::engine::codec::canalyst::{Message, PACKET_LEN, Packets, encode_packets, messages};
use crate::engine::frame::RawFrame;
use crate::engine::{ADDR_GLOBAL, Reassembler};
use crate::io::address_claim::ClaimState;

use super::iso11783_node::{
    Bus, Config, LinkStats, LoadSample, MAX_POLL_MS, NmeaDevice, TxBuffer, dispatch_cmd,
    format_iso, handle_frame, now_ms,
};
use super::socketcan::CLAIM_UNCLAIMED;
use super::{DeviceHandle, WriterCmd, from_parts};

/// Packets written in one USB transfer at most.
const PACKETS_PER_WRITE: usize = 16;

/// One CAN frame off the bus.
struct RxFrame {
    id: u32,
    extended: bool,
    data: SmallVec<[u8; 8]>,
}

/// What wakes the worker.
enum Input {
    /// Frames from one read, received at `when` (Unix ms).
    Rx(Vec<RxFrame>, u64),
    Cmd(WriterCmd),
    /// Every command sender is gone.
    CmdClosed,
    /// The adapter went away.
    Gone(io::Error),
}

/// Frames and data bytes each way, counted as they pass, for the load
/// figure of the network-status PGN.
#[derive(Default)]
struct Counters {
    rx_bytes: AtomicU64,
    rx_packets: AtomicU64,
    tx_bytes: AtomicU64,
    tx_packets: AtomicU64,
}

impl Counters {
    fn count(bytes: &AtomicU64, packets: &AtomicU64, data: usize) {
        bytes.fetch_add(data as u64, Ordering::Relaxed);
        packets.fetch_add(1, Ordering::Relaxed);
    }
}

struct CountedStats {
    counters: Arc<Counters>,
    bitrate: u32,
}

impl LinkStats for CountedStats {
    fn bitrate(&self) -> Option<u32> {
        Some(self.bitrate)
    }

    fn load_sample(&self, now_ms: u64) -> Option<LoadSample> {
        let c = &self.counters;
        Some(LoadSample {
            rx_bytes: c.rx_bytes.load(Ordering::Relaxed),
            tx_bytes: c.tx_bytes.load(Ordering::Relaxed),
            rx_packets: c.rx_packets.load(Ordering::Relaxed),
            tx_packets: c.tx_packets.load(Ordering::Relaxed),
            at_ms: now_ms,
        })
    }
}

/// Run the node over an open channel's packet streams (see
/// `io::canalyst::open_rw`). `claim_addr` follows the address the node
/// holds, [`CLAIM_UNCLAIMED`] while it has none, as for SocketCAN.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
    claim_addr: Arc<AtomicU8>,
) -> io::Result<DeviceHandle> {
    let (frames_tx, frames_rx) = mpsc::channel::<RawFrame>();
    let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd>();
    let (input_tx, input_rx) = mpsc::channel::<Input>();
    let stop = Arc::new(AtomicBool::new(false));
    let progress = Arc::new(AtomicU64::new(0));

    let reader_join = {
        let (input_tx, stop) = (input_tx.clone(), stop.clone());
        thread::Builder::new()
            .name("canalyst-reader".into())
            .spawn(move || read_packets(reader, &input_tx, &stop))?
    };
    // Not joined: it ends when the last command sender goes, or with the
    // worker at the next command.
    thread::Builder::new()
        .name("canalyst-commands".into())
        .spawn(move || {
            for cmd in cmd_rx {
                if input_tx.send(Input::Cmd(cmd)).is_err() {
                    return;
                }
            }
            let _ = input_tx.send(Input::CmdClosed);
        })?;
    let worker_join = {
        let progress = progress.clone();
        thread::Builder::new()
            .name("canalyst-worker".into())
            .spawn(move || {
                let mut worker = Worker::new(writer, &config, frames_tx, progress);
                worker.run(&config, &input_rx, &claim_addr);
                stop.store(true, Ordering::Relaxed);
            })?
    };
    Ok(from_parts(
        frames_rx,
        cmd_tx,
        vec![worker_join, reader_join],
        progress,
    ))
}

/// The reader thread: packets in, frames out to the worker, until the
/// adapter goes, the worker stops, or `stop` is set (checked at every
/// read timeout, so a quiet bus still lets go of the adapter).
fn read_packets(mut reader: Box<dyn Read + Send>, input: &mpsc::Sender<Input>, stop: &AtomicBool) {
    let mut packets = Packets::default();
    let mut buf = [0u8; 16 * PACKET_LEN];
    while !stop.load(Ordering::Relaxed) {
        let n = match reader.read(&mut buf) {
            Ok(0) => {
                let _ = input.send(Input::Gone(io::ErrorKind::UnexpectedEof.into()));
                return;
            }
            Ok(n) => n,
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut
                        | io::ErrorKind::WouldBlock
                        | io::ErrorKind::Interrupted
                ) =>
            {
                continue;
            }
            Err(e) => {
                let _ = input.send(Input::Gone(e));
                return;
            }
        };
        let when = now_ms();
        let mut frames = Vec::new();
        packets.push(&buf[..n], |packet| match messages(packet) {
            Ok(ms) => frames.extend(ms.iter().map(|m| RxFrame {
                id: m.id,
                extended: m.extended,
                data: SmallVec::from_slice(m.data),
            })),
            Err(count) => log::warn!("canalyst: a packet claims {count} messages"),
        });
        if !frames.is_empty() && input.send(Input::Rx(frames, when)).is_err() {
            return;
        }
    }
}

struct Worker {
    writer: Box<dyn Write + Send>,
    node: NmeaDevice,
    tx_buf: TxBuffer,
    reasm: Reassembler,
    frames_tx: mpsc::Sender<RawFrame>,
    counters: Arc<Counters>,
    progress: Arc<AtomicU64>,
}

impl Worker {
    fn new(
        writer: Box<dyn Write + Send>,
        config: &Config,
        frames_tx: mpsc::Sender<RawFrame>,
        progress: Arc<AtomicU64>,
    ) -> Self {
        let counters = Arc::new(Counters::default());
        let stats = CountedStats {
            counters: counters.clone(),
            bitrate: config.bitrate,
        };
        Self {
            writer,
            node: NmeaDevice::new(config, Box::new(stats)),
            tx_buf: TxBuffer::with_protocol(config.protocol),
            reasm: Reassembler::new(),
            frames_tx,
            counters,
            progress,
        }
    }

    fn bus(&mut self) -> (Bus<'_>, &mut NmeaDevice) {
        (
            Bus {
                tx_buf: &mut self.tx_buf,
                frames_tx: &self.frames_tx,
            },
            &mut self.node,
        )
    }

    fn run(&mut self, config: &Config, input: &mpsc::Receiver<Input>, claim_addr: &AtomicU8) {
        claim_addr.store(CLAIM_UNCLAIMED, Ordering::Relaxed);
        let mut published = CLAIM_UNCLAIMED;
        let timeout_ms = config.timeout_secs.saturating_mul(1000);
        let mut last_frame = now_ms();
        if self.node.claim.state() != ClaimState::Disabled {
            let (mut bus, node) = self.bus();
            node.start(&mut bus);
        }
        loop {
            let now = now_ms();
            self.tx_buf.poll_tp(now);
            if let Err(e) = self.write_queued() {
                log::error!("canalyst: writing to the adapter: {e}");
                return;
            }
            let claim = &self.node.claim;
            let mut wait = if claim.is_timing() && claim.deadline() > now {
                claim.deadline() - now
            } else if claim.is_claimed() && self.node.heartbeat_interval > 0 {
                self.node.next_heartbeat.saturating_sub(now)
            } else {
                MAX_POLL_MS
            };
            if let Some(at) = self.tx_buf.tp.next_deadline() {
                wait = wait.min(at.saturating_sub(now));
            }
            match input.recv_timeout(Duration::from_millis(wait.min(MAX_POLL_MS))) {
                Ok(Input::Rx(frames, when)) => {
                    last_frame = now_ms();
                    self.receive(config, frames, when);
                }
                Ok(Input::Cmd(WriterCmd::Shutdown(done))) => {
                    let flushed = self.shutdown();
                    if let Some(done) = done {
                        let _ = done.send(flushed);
                    }
                    return;
                }
                Ok(Input::Cmd(cmd)) => {
                    let (mut bus, node) = self.bus();
                    dispatch_cmd(&mut bus, node, cmd);
                }
                Ok(Input::Gone(e)) => {
                    log::error!("canalyst: the adapter went away: {e}");
                    return;
                }
                Ok(Input::CmdClosed) | Err(mpsc::RecvTimeoutError::Disconnected) => return,
                Err(mpsc::RecvTimeoutError::Timeout) => {}
            }
            if timeout_ms > 0 && now_ms().saturating_sub(last_frame) >= timeout_ms {
                log::error!("Timeout {} seconds; no data received", config.timeout_secs);
                return;
            }
            let (mut bus, node) = self.bus();
            node.tick(&mut bus, now_ms());
            let live = self.node.claim.send_address().unwrap_or(CLAIM_UNCLAIMED);
            if live != published {
                claim_addr.store(live, Ordering::Relaxed);
                published = live;
            }
        }
    }

    fn receive(&mut self, config: &Config, frames: Vec<RxFrame>, when: u64) {
        for f in frames {
            Counters::count(
                &self.counters.rx_bytes,
                &self.counters.rx_packets,
                f.data.len(),
            );
            // NMEA 2000 and J1939 are 29-bit, Quick 11-bit: skip the
            // other kind rather than reinterpret its identifier.
            if f.extended == config.protocol.standard_frames() {
                continue;
            }
            if !f.extended {
                let _ = self.frames_tx.send(RawFrame::new(
                    Some(format_iso(when)),
                    0,
                    f.id,
                    0,
                    ADDR_GLOBAL,
                    f.data.iter().copied(),
                ));
                continue;
            }
            let mut bus = Bus {
                tx_buf: &mut self.tx_buf,
                frames_tx: &self.frames_tx,
            };
            handle_frame(
                &mut bus,
                &mut self.node,
                &mut self.reasm,
                f.id,
                &f.data,
                when,
            );
        }
    }

    /// Write everything queued, [`PACKETS_PER_WRITE`] packets at a time.
    fn write_queued(&mut self) -> io::Result<()> {
        let mut out = Vec::with_capacity(PACKETS_PER_WRITE * PACKET_LEN);
        while !self.tx_buf.is_empty() {
            let n = self.tx_buf.queue.len().min(PACKETS_PER_WRITE * 3);
            let batch: Vec<Message> = self
                .tx_buf
                .queue
                .iter()
                .take(n)
                .map(|f| Message {
                    id: f.id,
                    extended: !f.standard,
                    data: &f.data,
                })
                .collect();
            out.clear();
            encode_packets(&batch, &mut out);
            self.writer.write_all(&out)?;
            for _ in 0..n {
                let len = self.tx_buf.queue.front().map_or(0, |f| f.data.len());
                Counters::count(&self.counters.tx_bytes, &self.counters.tx_packets, len);
                self.tx_buf.pop_sent();
            }
            self.progress.fetch_add(n as u64, Ordering::Relaxed);
        }
        Ok(())
    }

    /// Send what is queued and leave the bus (dropping the writer stops
    /// the channel). Whether nothing was left behind.
    fn shutdown(&mut self) -> bool {
        // An ISO TP transfer still running is not waited for; abort the
        // RTS/CTS ones so their receivers know.
        let tp_unfinished = !self.tx_buf.tp.is_idle();
        if tp_unfinished {
            log::warn!("canalyst: leaving with an ISO TP transfer unfinished");
            for f in self.tx_buf.tp.abort_all() {
                self.tx_buf.push_frame(&f);
            }
        }
        self.write_queued().is_ok() && !tp_unfinished
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    use crate::engine::format::{iso11783_compose, iso11783_decompose};

    /// The adapter's side of the link: packets the test feeds the node,
    /// and the frames the node writes.
    #[derive(Clone, Default)]
    struct Adapter {
        written: Arc<Mutex<Vec<u8>>>,
    }

    impl Write for Adapter {
        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            self.written.lock().unwrap().extend_from_slice(data);
            Ok(data.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Adapter {
        /// Every frame written so far, as `(prio, pgn, src, dst, data)`.
        fn frames(&self) -> Vec<(u8, u32, u8, u8, Vec<u8>)> {
            let written = self.written.lock().unwrap();
            let (packets, rest) = written.as_chunks::<PACKET_LEN>();
            assert!(rest.is_empty());
            packets
                .iter()
                .flat_map(|p| {
                    messages(p)
                        .unwrap()
                        .into_iter()
                        .map(|m| {
                            let (prio, pgn, src, dst) = iso11783_decompose(m.id);
                            (prio, pgn, src, dst, m.data.to_vec())
                        })
                        .collect::<Vec<_>>()
                })
                .collect()
        }
    }

    /// A reader that hands out `packets`, then reports timeouts until
    /// the node stops reading.
    struct Feed(std::collections::VecDeque<Vec<u8>>);

    impl Read for Feed {
        fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
            match self.0.pop_front() {
                Some(p) => {
                    out[..p.len()].copy_from_slice(&p);
                    Ok(p.len())
                }
                None => {
                    thread::sleep(Duration::from_millis(5));
                    Err(io::ErrorKind::TimedOut.into())
                }
            }
        }
    }

    fn packet(id: u32, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        encode_packets(
            &[Message {
                id,
                extended: true,
                data,
            }],
            &mut out,
        );
        out
    }

    #[test]
    fn received_frames_reach_the_application() {
        let adapter = Adapter::default();
        let feed = Feed(
            [packet(iso11783_compose(2, 127250, 0x21, 255), &[1; 8])]
                .into_iter()
                .collect(),
        );
        let config = Config {
            no_claim: true,
            ..Config::default()
        };
        let handle = run(
            Box::new(feed),
            Box::new(adapter),
            config,
            Arc::new(AtomicU8::new(CLAIM_UNCLAIMED)),
        )
        .unwrap();
        let f = handle
            .frames_rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(
            (f.pgn, f.src, f.data.as_slice()),
            (127250, 0x21, &[1; 8][..])
        );
        handle.join();
    }

    /// The node claims its address on the adapter and, once it holds it,
    /// sends what the application sends from it, fast-packets split.
    #[test]
    fn the_node_claims_and_then_sends() {
        let adapter = Adapter::default();
        let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
        let config = Config {
            address: 77,
            unique: 1234,
            heartbeat_ms: 0,
            ..Config::default()
        };
        let handle = run(
            Box::new(Feed(Default::default())),
            Box::new(adapter.clone()),
            config,
            claim.clone(),
        )
        .unwrap();
        // Scan, then the claim's 250 ms to settle.
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while claim.load(Ordering::Relaxed) != 77 {
            assert!(std::time::Instant::now() < deadline, "never claimed");
            thread::sleep(Duration::from_millis(20));
        }
        let claims: Vec<_> = adapter
            .frames()
            .into_iter()
            .filter(|f| f.1 == 60928)
            .collect();
        assert!(claims.iter().any(|f| f.2 == 77), "{claims:?}");

        let data: Vec<u8> = (0..20).collect();
        handle
            .send_frame(RawFrame::new(None, 3, 129029, 0, 255, data.iter().copied()))
            .unwrap();
        assert_eq!(handle.close(), super::super::Closed::Confirmed);
        let sent: Vec<_> = adapter
            .frames()
            .into_iter()
            .filter(|f| f.1 == 129029)
            .collect();
        assert_eq!(sent.len(), 3, "20 bytes: 3 fast-packet frames");
        assert!(sent.iter().all(|f| f.0 == 3 && f.2 == 77));
        assert_eq!(&sent[0].4[2..], &data[..6]);
    }
}
