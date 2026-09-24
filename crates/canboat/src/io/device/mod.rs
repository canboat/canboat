// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Device-level read/write abstraction shared by the standalone
//! reader binaries (actisense-serial, ikonvert-serial, maretron-ipg)
//! and the integrated `n2kd-pipeline` binary.
//!
//! Each supported device speaks its own wire protocol (NGT-1 binary,
//! iKonvert ASCII, Maretron 0xA5 TCP), but the threading shape is
//! identical: a reader thread drains device bytes through a sans-I/O
//! decoder, and a writer thread drains a command channel back out to
//! the device. Both threads are owned by the runner here; the
//! per-device codec lives behind two small traits ([`DeviceDecoder`]
//! and [`DeviceEncoder`]) so the runner stays oblivious to the
//! protocol.
//!
//! A typical use, through the public `bus::open_*` wrappers (which open
//! the serial port and call the runner, e.g. `ngt1::run(reader, writer)`):
//!
//! ```no_run
//! let handle = canboat::bus::open_ngt1("/dev/ttyUSB0", 115_200)?;
//! while let Ok(frame) = handle.frames_rx.recv() {
//!     println!("got pgn {}", frame.pgn);
//! }
//! # Ok::<(), std::io::Error>(())
//! ```

pub mod canboat_csv;
pub mod ikonvert;
pub mod line_gateway;
pub mod maretron;
pub mod ngt1;
pub mod ngt1_tx_list;
pub mod socketcan;
pub mod supervisor;

pub use supervisor::{DeviceFactory, Supervisor};

use std::io::{self, Read, Write};
use std::sync::mpsc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::engine::RawFrame;

/// Events emitted by a [`DeviceDecoder`] as bytes are pumped in.
#[derive(Debug)]
pub enum DeviceEvent {
    /// A complete N2K frame received from the bus.
    Frame(RawFrame),
    /// The codec needs the runner to write these bytes back to the
    /// device. Used by codecs whose handshake response depends on
    /// device output (e.g. Maretron `SET_MODE BINARY` after seeing
    /// `CONNECTED`).
    SendBytes(Vec<u8>),
    /// Decoded a framing or protocol error. The runner logs it.
    Error(String),
}

/// Decoder half of a device codec: pushes received bytes and emits
/// [`DeviceEvent`]s.
pub trait DeviceDecoder: Send + 'static {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>);

    /// Called when a read times out with nothing received (a serial port
    /// does every 250 ms), so a decoder's own deadlines advance on a quiet
    /// line too. Nothing by default.
    fn tick(&mut self, _events: &mut Vec<DeviceEvent>) {}
}

/// Encoder half: turns app-side [`RawFrame`]s into device bytes, plus
/// optional init / keepalive sequences.
pub trait DeviceEncoder: Send + Sync + 'static {
    /// Bytes to write as soon as the connection opens (NGT-1 startup
    /// ping, iKonvert init handshake, Maretron CONNECT). Empty when
    /// the device needs no handshake.
    fn init_bytes(&self) -> Vec<u8> {
        Vec::new()
    }

    /// Optional periodic keepalive: `(interval, payload)`. `None`
    /// means the writer thread never wakes on a timer.
    fn keepalive(&self) -> Option<(Duration, Vec<u8>)> {
        None
    }

    /// Encode a [`RawFrame`] for transmission. `None` means "silently
    /// drop" — used to skip synthetic PGNs that never hit the bus.
    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>>;

    /// Bytes to write when the application closes the device on purpose
    /// ([`DeviceHandle::close`]), e.g. to take a gateway off the bus.
    /// Empty when the device needs nothing.
    fn shutdown_bytes(&self) -> Vec<u8> {
        Vec::new()
    }
}

/// Commands accepted by the writer thread.
pub(crate) enum WriterCmd {
    Bytes(Vec<u8>),
    Frame(RawFrame),
    /// Write the encoder's [`DeviceEncoder::shutdown_bytes`] and stop,
    /// then tell the sender, if any, whether that succeeded.
    Shutdown(Option<mpsc::Sender<bool>>),
}

/// Returned by [`run`]. Owns the reader/writer threads' join handles
/// and the channels used to talk to them.
pub struct DeviceHandle {
    /// Frames received from the device. The reader thread drops the
    /// sender on EOF / fatal error, which closes this channel.
    pub frames_rx: mpsc::Receiver<RawFrame>,
    cmd_tx: mpsc::Sender<WriterCmd>,
    joins: Vec<JoinHandle<()>>,
    /// The writer thread, joined apart from the others by [`Self::close`].
    /// `None` for a codec that has no separate writer (SocketCAN).
    writer: Option<JoinHandle<()>>,
}

/// Returned by [`DeviceHandle::send_frame`] / [`FrameSender::send_frame`]
/// when the writer thread has gone away (either the device closed or
/// the runner shut down).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceWriterGone;

impl std::fmt::Display for DeviceWriterGone {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("device writer is no longer accepting frames")
    }
}

impl std::error::Error for DeviceWriterGone {}

impl DeviceHandle {
    /// Queue a frame to be encoded and written to the device.
    pub fn send_frame(&self, frame: RawFrame) -> Result<(), DeviceWriterGone> {
        self.cmd_tx
            .send(WriterCmd::Frame(frame))
            .map_err(|_| DeviceWriterGone)
    }

    /// Clone-able sender, for handing off to a producer thread.
    pub fn frame_sender(&self) -> FrameSender {
        FrameSender {
            cmd_tx: self.cmd_tx.clone(),
        }
    }

    /// Signal shutdown to the writer thread and wait for both threads
    /// to exit.
    pub fn join(self) {
        drop(self.cmd_tx);
        for j in self.joins.into_iter().chain(self.writer) {
            let _ = j.join();
        }
    }

    /// Close the device on purpose: the writer sends the encoder's
    /// [`DeviceEncoder::shutdown_bytes`] (an iKonvert goes off the bus)
    /// and stops. Waits up to [`CLOSE_TIMEOUT`] for it — a writer stuck
    /// behind a full socket is left behind rather than hanging shutdown —
    /// and returns whether the device confirmed. The reader thread is not
    /// joined: it may be blocked reading the device, and ends with it.
    pub fn close(self) -> bool {
        let closed = self.closer().close(CLOSE_TIMEOUT);
        if closed {
            if let Some(writer) = self.writer {
                let _ = writer.join();
            }
        } else {
            log::warn!("device did not confirm closing within {CLOSE_TIMEOUT:?}; leaving it");
        }
        closed
    }

    /// A handle another thread can [`close`](DeviceCloser::close) the
    /// device with — for a signal handler while this thread is busy
    /// reading.
    pub fn closer(&self) -> DeviceCloser {
        DeviceCloser {
            cmd_tx: self.cmd_tx.clone(),
        }
    }
}

/// How long [`DeviceHandle::close`] waits for the device to confirm.
pub const CLOSE_TIMEOUT: Duration = Duration::from_secs(2);

/// Closes a device from another thread; see [`DeviceHandle::closer`].
#[derive(Clone)]
pub struct DeviceCloser {
    cmd_tx: mpsc::Sender<WriterCmd>,
}

impl DeviceCloser {
    /// [`DeviceHandle::close`] from elsewhere: have the writer send the
    /// encoder's shutdown bytes and stop, waiting up to `timeout` for it.
    /// Returns whether the device confirmed the close: the goodbye written
    /// and flushed (or none needed).
    pub fn close(&self, timeout: Duration) -> bool {
        let (done_tx, done_rx) = mpsc::channel();
        if self
            .cmd_tx
            .send(WriterCmd::Shutdown(Some(done_tx)))
            .is_err()
        {
            return false;
        }
        matches!(done_rx.recv_timeout(timeout), Ok(true))
    }
}

/// Clone-able sender for [`DeviceHandle::send_frame`].
#[derive(Clone)]
pub struct FrameSender {
    cmd_tx: mpsc::Sender<WriterCmd>,
}

impl FrameSender {
    pub fn send_frame(&self, frame: RawFrame) -> Result<(), DeviceWriterGone> {
        self.cmd_tx
            .send(WriterCmd::Frame(frame))
            .map_err(|_| DeviceWriterGone)
    }

    /// Crate-internal constructor used by [`supervisor::Supervisor`]
    /// so it can hand out a stable `FrameSender` backed by its own
    /// outer cmd channel.
    pub(crate) fn from_cmd_tx(cmd_tx: mpsc::Sender<WriterCmd>) -> Self {
        Self { cmd_tx }
    }
}

/// Build a [`DeviceHandle`] from already-wired channels and thread
/// handles. Used by codecs whose I/O model isn't byte-stream-shaped
/// (e.g. `socketcan`) and can't use the generic [`run`] runner below.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
pub(crate) fn from_parts(
    frames_rx: mpsc::Receiver<RawFrame>,
    cmd_tx: mpsc::Sender<WriterCmd>,
    joins: Vec<JoinHandle<()>>,
) -> DeviceHandle {
    DeviceHandle {
        frames_rx,
        cmd_tx,
        joins,
        writer: None,
    }
}

/// Generic two-thread runner. Each codec module wraps this with its
/// own [`DeviceDecoder`] / [`DeviceEncoder`] impls.
pub fn run<D: DeviceDecoder, E: DeviceEncoder>(
    decoder: D,
    encoder: E,
    mut reader: Box<dyn Read + Send>,
    mut writer: Box<dyn Write + Send>,
) -> DeviceHandle {
    let (frames_tx, frames_rx) = mpsc::channel::<RawFrame>();
    let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd>();
    let cmd_tx_for_reader = cmd_tx.clone();

    let init = encoder.init_bytes();
    let keepalive = encoder.keepalive();

    let writer_join = thread::Builder::new()
        .name("device-writer".into())
        .spawn(move || writer_thread(&mut *writer, encoder, init, keepalive, cmd_rx))
        .expect("spawn device writer");

    let reader_join = thread::Builder::new()
        .name("device-reader".into())
        .spawn(move || reader_thread(&mut *reader, decoder, frames_tx, cmd_tx_for_reader))
        .expect("spawn device reader");

    DeviceHandle {
        frames_rx,
        cmd_tx,
        joins: vec![reader_join],
        writer: Some(writer_join),
    }
}

fn reader_thread<D: DeviceDecoder>(
    reader: &mut dyn Read,
    mut decoder: D,
    frames_tx: mpsc::Sender<RawFrame>,
    cmd_tx: mpsc::Sender<WriterCmd>,
) {
    let mut buf = [0u8; 4096];
    let mut events: Vec<DeviceEvent> = Vec::with_capacity(8);
    // Hand the decoder's events on; `false` once a channel is gone.
    let deliver = |events: &mut Vec<DeviceEvent>| -> bool {
        for ev in events.drain(..) {
            match ev {
                DeviceEvent::Frame(f) => {
                    if frames_tx.send(f).is_err() {
                        return false;
                    }
                }
                DeviceEvent::SendBytes(b) => {
                    if cmd_tx.send(WriterCmd::Bytes(b)).is_err() {
                        return false;
                    }
                }
                DeviceEvent::Error(e) => {
                    log::warn!("device decode error: {e}");
                }
            }
        }
        true
    };
    loop {
        match reader.read(&mut buf) {
            Ok(0) => return,
            Ok(n) => {
                decoder.decode(&buf[..n], &mut events);
                if !deliver(&mut events) {
                    return;
                }
            }
            Err(e) if e.kind() == io::ErrorKind::TimedOut => {
                decoder.tick(&mut events);
                if !deliver(&mut events) {
                    return;
                }
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => {
                log::error!("device read error: {e}");
                return;
            }
        }
    }
}

fn writer_thread<E: DeviceEncoder>(
    writer: &mut dyn Write,
    encoder: E,
    init: Vec<u8>,
    keepalive: Option<(Duration, Vec<u8>)>,
    rx: mpsc::Receiver<WriterCmd>,
) {
    if !init.is_empty() {
        if let Err(e) = writer.write_all(&init) {
            log::error!("device init write failed: {e}");
            return;
        }
        let _ = writer.flush();
    }

    let mut next_keepalive = keepalive.as_ref().map(|(d, _)| Instant::now() + *d);

    loop {
        let now = Instant::now();
        let cmd = match (next_keepalive, &keepalive) {
            (Some(deadline), Some((interval, payload))) => {
                let timeout = deadline.saturating_duration_since(now);
                match rx.recv_timeout(timeout) {
                    Ok(cmd) => cmd,
                    Err(mpsc::RecvTimeoutError::Timeout) => {
                        if let Err(e) = writer.write_all(payload) {
                            log::warn!("device keepalive failed: {e}");
                            return;
                        }
                        let _ = writer.flush();
                        next_keepalive = Some(Instant::now() + *interval);
                        continue;
                    }
                    Err(mpsc::RecvTimeoutError::Disconnected) => return,
                }
            }
            _ => match rx.recv() {
                Ok(cmd) => cmd,
                Err(_) => return,
            },
        };

        let bytes = match cmd {
            WriterCmd::Bytes(b) => b,
            WriterCmd::Frame(f) => match encoder.encode_frame(&f) {
                Some(b) => b,
                None => continue,
            },
            WriterCmd::Shutdown(done) => {
                let bytes = encoder.shutdown_bytes();
                let sent = bytes.is_empty()
                    || match writer.write_all(&bytes).and_then(|()| writer.flush()) {
                        Ok(()) => true,
                        Err(e) => {
                            log::warn!("device shutdown write failed: {e}");
                            false
                        }
                    };
                if let Some(done) = done {
                    let _ = done.send(sent);
                }
                return;
            }
        };
        if let Err(e) = writer.write_all(&bytes) {
            log::error!("device write failed: {e}");
            return;
        }
        let _ = writer.flush();
        // Push the keepalive deadline forward — every real write
        // counts as activity.
        if let Some((interval, _)) = keepalive.as_ref() {
            next_keepalive = Some(Instant::now() + *interval);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A line that stays quiet: every read times out, then it closes.
    struct QuietThenEof(u32);
    impl Read for QuietThenEof {
        fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
            if self.0 == 0 {
                return Ok(0);
            }
            self.0 -= 1;
            Err(io::ErrorKind::TimedOut.into())
        }
    }

    /// Emits one frame per tick.
    struct Ticker;
    impl DeviceDecoder for Ticker {
        fn decode(&mut self, _bytes: &[u8], _events: &mut Vec<DeviceEvent>) {}
        fn tick(&mut self, events: &mut Vec<DeviceEvent>) {
            events.push(DeviceEvent::Frame(RawFrame::new(
                None,
                0,
                1,
                0,
                0,
                std::iter::empty(),
            )));
        }
    }

    struct NoEncoder;
    impl DeviceEncoder for NoEncoder {
        fn encode_frame(&self, _frame: &RawFrame) -> Option<Vec<u8>> {
            None
        }
    }

    /// A decoder's deadlines advance on a quiet line: each read timeout
    /// ticks it, and what the tick produces is delivered.
    #[test]
    fn read_timeouts_tick_the_decoder() {
        let handle = run(
            Ticker,
            NoEncoder,
            Box::new(QuietThenEof(3)),
            Box::new(io::sink()),
        );
        let ticks = handle.frames_rx.iter().count();
        assert_eq!(ticks, 3);
    }
}
