// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! A full NMEA 2000 or J1939 node on a CAN adapter that passes single CAN
//! frames, like [`super::socketcan`]: it claims an address, answers ISO
//! Requests, sends the Heartbeat and Product Information, and frames what
//! the application sends. The node itself is [`super::iso11783_node`];
//! this module moves its frames over the adapter's [`Link`]: the CANalyst-II
//! ([`super::canalyst`]) and the Waveshare USB-CAN-A ([`super::usb_can_a`]).
//!
//! A reader thread turns packets into CAN frames; a worker thread owns
//! the node and the writer, and takes both the received frames and the
//! application's commands from one queue, so it sleeps until there is
//! something to do or a timer is due.

use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use crate::engine::codec::raw_can::CanFrame;
use crate::engine::frame::RawFrame;
use crate::engine::{ADDR_GLOBAL, Reassembler};
use crate::io::address_claim::ClaimState;

use super::iso11783_node::{
    Bus, Config, LinkStats, LoadSample, MAX_POLL_MS, NmeaDevice, TxBuffer, dispatch_cmd,
    format_iso, handle_frame, now_ms,
};
use super::socketcan::CLAIM_UNCLAIMED;
use super::{DeviceFailure, DeviceHandle, WriterCmd, from_parts};

/// How an adapter carries CAN frames over its byte streams.
pub(crate) trait Link: Send + 'static {
    /// For log lines and thread names.
    const NAME: &'static str;
    /// Frames written at once at most.
    const FRAMES_PER_WRITE: usize;
    /// The least time from one write to the next on a bus at `bitrate`, for
    /// an adapter that loses frames written faster than it sends them.
    fn write_gap(_bitrate: u32) -> Duration {
        Duration::ZERO
    }
    /// Bytes read at once.
    const READ_LEN: usize;

    /// The state of the reading side between reads.
    type Decoder: Default + Send + 'static;

    /// Add the frames `bytes` complete to `frames`.
    fn decode(decoder: &mut Self::Decoder, bytes: &[u8], frames: &mut Vec<CanFrame>);

    /// Append the bytes that carry `frames`.
    fn encode(frames: &[CanFrame], out: &mut Vec<u8>);
}

/// What wakes the worker.
enum Input {
    /// Frames from one read, received at `when` (Unix ms).
    Rx(Vec<CanFrame>, u64),
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

/// Run the node over an open adapter's byte streams. `claim_addr` follows
/// the address the node holds, [`CLAIM_UNCLAIMED`] while it has none, as
/// for SocketCAN.
pub(crate) fn run<L: Link>(
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
    let lost = DeviceFailure::default();

    let reader_join = {
        let (input_tx, stop) = (input_tx.clone(), stop.clone());
        thread::Builder::new()
            .name(format!("{}-reader", L::NAME))
            .spawn(move || read_frames::<L>(reader, &input_tx, &stop))?
    };
    // Not joined: it ends when the last command sender goes, or with the
    // worker at the next command.
    thread::Builder::new()
        .name(format!("{}-commands", L::NAME))
        .spawn(move || {
            for cmd in cmd_rx {
                if input_tx.send(Input::Cmd(cmd)).is_err() {
                    return;
                }
            }
            let _ = input_tx.send(Input::CmdClosed);
        })?;
    let worker_join = {
        let (progress, lost) = (progress.clone(), lost.clone());
        thread::Builder::new()
            .name(format!("{}-worker", L::NAME))
            .spawn(move || {
                let mut worker = Worker::<L>::new(writer, &config, frames_tx, progress);
                if let Err(why) = worker.run(&config, &input_rx, &claim_addr) {
                    log::error!("{why}");
                    lost.set(&why);
                }
                stop.store(true, Ordering::Relaxed);
            })?
    };
    Ok(from_parts(
        frames_rx,
        cmd_tx,
        vec![worker_join, reader_join],
        progress,
        lost,
    ))
}

/// The reader thread: bytes in, frames out to the worker, until the
/// adapter goes, the worker stops, or `stop` is set (checked at every
/// read timeout, so a quiet bus still lets go of the adapter).
fn read_frames<L: Link>(
    mut reader: Box<dyn Read + Send>,
    input: &mpsc::Sender<Input>,
    stop: &AtomicBool,
) {
    let mut decoder = L::Decoder::default();
    let mut buf = vec![0u8; L::READ_LEN];
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
        L::decode(&mut decoder, &buf[..n], &mut frames);
        if !frames.is_empty() && input.send(Input::Rx(frames, when)).is_err() {
            return;
        }
    }
}

struct Worker<L> {
    writer: Box<dyn Write + Send>,
    /// When the last write was, and the least time to the next one
    /// ([`Link::write_gap`]).
    last_write: Option<Instant>,
    write_gap: Duration,
    node: NmeaDevice,
    tx_buf: TxBuffer,
    reasm: Reassembler,
    frames_tx: mpsc::Sender<RawFrame>,
    counters: Arc<Counters>,
    progress: Arc<AtomicU64>,
    link: std::marker::PhantomData<L>,
}

impl<L: Link> Worker<L> {
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
            last_write: None,
            write_gap: L::write_gap(config.bitrate),
            node: NmeaDevice::new(config, Box::new(stats)),
            tx_buf: TxBuffer::with_protocol(config.protocol),
            reasm: Reassembler::new(),
            frames_tx,
            counters,
            progress,
            link: std::marker::PhantomData,
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

    /// Run until closed or until every sender is gone; `Err` with why when
    /// the adapter went away instead.
    fn run(
        &mut self,
        config: &Config,
        input: &mpsc::Receiver<Input>,
        claim_addr: &AtomicU8,
    ) -> Result<(), String> {
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
                return Err(format!("{}: writing to the adapter: {e}", L::NAME));
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
                    return Ok(());
                }
                Ok(Input::Cmd(cmd)) => {
                    let (mut bus, node) = self.bus();
                    dispatch_cmd(&mut bus, node, cmd);
                }
                Ok(Input::Gone(e)) => {
                    return Err(format!("{}: the adapter went away: {e}", L::NAME));
                }
                Ok(Input::CmdClosed) | Err(mpsc::RecvTimeoutError::Disconnected) => return Ok(()),
                Err(mpsc::RecvTimeoutError::Timeout) => {}
            }
            if timeout_ms > 0 && now_ms().saturating_sub(last_frame) >= timeout_ms {
                return Err(format!(
                    "no data received for {} seconds",
                    config.timeout_secs
                ));
            }
            let (mut bus, node) = self.bus();
            node.tick(&mut bus, now_ms());
            self.reasm.tick(now_ms());
            let live = self.node.claim.send_address().unwrap_or(CLAIM_UNCLAIMED);
            if live != published {
                claim_addr.store(live, Ordering::Relaxed);
                published = live;
            }
        }
    }

    fn receive(&mut self, config: &Config, frames: Vec<CanFrame>, when: u64) {
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

    /// Write everything queued, [`Link::FRAMES_PER_WRITE`] frames at a
    /// time, [`Link::write_gap`] apart.
    fn write_queued(&mut self) -> io::Result<()> {
        let mut out = Vec::new();
        while !self.tx_buf.is_empty() {
            let n = self.tx_buf.queue.len().min(L::FRAMES_PER_WRITE);
            let batch: Vec<CanFrame> = self
                .tx_buf
                .queue
                .iter()
                .take(n)
                .map(|f| CanFrame {
                    id: f.id,
                    extended: !f.standard,
                    data: f.data.iter().copied().collect(),
                })
                .collect();
            out.clear();
            L::encode(&batch, &mut out);
            if let Some(last) = self.last_write {
                let next = last + self.write_gap;
                let now = Instant::now();
                if next > now {
                    thread::sleep(next - now);
                }
            }
            self.writer.write_all(&out)?;
            self.last_write = Some(Instant::now());
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
            log::warn!("{}: leaving with an ISO TP transfer unfinished", L::NAME);
            for f in self.tx_buf.tp.abort_all() {
                self.tx_buf.push_frame(&f);
            }
        }
        self.write_queued().is_ok() && !tp_unfinished
    }
}
