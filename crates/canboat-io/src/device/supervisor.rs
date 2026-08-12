// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Reconnect supervisor for [`super::DeviceHandle`].
//!
//! A bare `DeviceHandle` lasts exactly one device session: when the
//! serial port disconnects or the TCP socket dies, the reader thread
//! returns and the `frames_rx` channel closes. That's fine for a
//! one-shot command-line tool but unacceptable for a long-running
//! service like `n2kd-pipeline`.
//!
//! `Supervisor` wraps a [`DeviceFactory`] (a thunk that knows how to
//! open the device) and provides a stable `frames_rx` /
//! [`FrameSender`] pair that survives any number of disconnects. On
//! failure it logs, backs off exponentially, and reopens the device.
//! Frames produced while the device is down get dropped on the floor
//! — the writer's command channel buffers a small queue, and any
//! commands that don't drain within one session are discarded when
//! that session ends. This matches what canboat C's reader binaries
//! do on EOF (exit and let systemd restart) while keeping the
//! pipeline alive.

use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use canboat_core::RawFrame;

use super::{DeviceHandle, FrameSender, WriterCmd};

/// Factory thunk handed to [`Supervisor::new`]. Each call to `open`
/// must produce a *fresh* `DeviceHandle` — the supervisor never reuses
/// one across sessions.
pub trait DeviceFactory: Send + 'static {
    fn open(&mut self) -> io::Result<DeviceHandle>;

    /// Short label for log output (e.g. `"ngt1"` / `"maretron"`).
    fn name(&self) -> &str {
        "device"
    }
}

/// Convenient blanket impl so callers can pass an `FnMut() -> io::Result<DeviceHandle>`.
impl<F> DeviceFactory for F
where
    F: FnMut() -> io::Result<DeviceHandle> + Send + 'static,
{
    fn open(&mut self) -> io::Result<DeviceHandle> {
        (self)()
    }
}

/// Initial backoff after the first failure. Doubles each retry up to
/// [`MAX_BACKOFF`].
pub const INITIAL_BACKOFF: Duration = Duration::from_millis(500);
/// Cap on the exponential backoff so a sustained outage doesn't
/// stretch reconnect attempts past 30 s.
pub const MAX_BACKOFF: Duration = Duration::from_secs(30);

/// A supervised device connection.
///
/// Owns a manager thread that loops `open → run session → wait →
/// reconnect`. The `frames_rx` channel is stable across reconnects:
/// clients of the supervisor see one long-lived stream of `RawFrame`s
/// interleaved with reconnect log messages. The [`FrameSender`]
/// returned by [`Supervisor::frame_sender`] is similarly stable.
pub struct Supervisor {
    /// Stable receiver: frames from whichever device session is
    /// currently active.
    pub frames_rx: mpsc::Receiver<RawFrame>,
    sender: FrameSender,
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

impl Supervisor {
    /// Spawn the manager. Returns immediately; the first device-open
    /// attempt happens on the manager thread.
    pub fn new<F: DeviceFactory>(factory: F) -> Self {
        let (frames_tx, frames_rx) = mpsc::channel::<RawFrame>();
        let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd>();
        let stop = Arc::new(AtomicBool::new(false));
        let stop_for_thread = stop.clone();

        let join = thread::Builder::new()
            .name("device-supervisor".into())
            .spawn(move || supervisor_loop(factory, frames_tx, cmd_rx, stop_for_thread))
            .expect("spawn device supervisor");

        Self {
            frames_rx,
            sender: FrameSender::from_cmd_tx(cmd_tx),
            stop,
            join: Some(join),
        }
    }

    /// Clone of the stable frame sender. Use this to queue outbound
    /// frames; it routes through the active session, or drops on the
    /// floor while the device is disconnected.
    pub fn frame_sender(&self) -> FrameSender {
        self.sender.clone()
    }

    /// Signal the manager to stop reconnecting and tear down the
    /// current session.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Drop for Supervisor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(j) = self.join.take() {
            // Best-effort wait; if the manager is stuck in a long
            // recv_timeout we accept a brief tear-down delay.
            let _ = j.join();
        }
    }
}

fn supervisor_loop<F: DeviceFactory>(
    mut factory: F,
    frames_tx: mpsc::Sender<RawFrame>,
    cmd_rx: mpsc::Receiver<WriterCmd>,
    stop: Arc<AtomicBool>,
) {
    let mut backoff = INITIAL_BACKOFF;
    // Hold cmd_rx in an Option so we can move it into the per-session
    // forwarder and get it back via a one-shot return channel.
    let mut held_cmd_rx = Some(cmd_rx);

    while !stop.load(Ordering::Relaxed) {
        let handle = match factory.open() {
            Ok(h) => {
                log::info!("{}: device session opened", factory.name());
                backoff = INITIAL_BACKOFF;
                h
            }
            Err(e) => {
                log::warn!(
                    "{}: open failed: {e}; retrying in {:?}",
                    factory.name(),
                    backoff
                );
                sleep_interruptible(backoff, &stop);
                backoff = (backoff * 2).min(MAX_BACKOFF);
                continue;
            }
        };

        let cmd_rx = held_cmd_rx
            .take()
            .expect("held_cmd_rx is always Some at session start");
        held_cmd_rx = Some(run_session(
            factory.name(),
            handle,
            &frames_tx,
            cmd_rx,
            &stop,
        ));

        if stop.load(Ordering::Relaxed) {
            break;
        }
        log::warn!("{}: session ended; will reconnect", factory.name());
        sleep_interruptible(Duration::from_millis(500), &stop);
    }
    log::info!("{}: supervisor stopped", factory.name());
}

/// Sleep for up to `duration`, waking early if `stop` is set.
fn sleep_interruptible(duration: Duration, stop: &Arc<AtomicBool>) {
    // Poll every ~50ms so we stay responsive to shutdown without
    // burning CPU.
    let step = Duration::from_millis(50);
    let mut remaining = duration;
    while remaining > Duration::ZERO {
        if stop.load(Ordering::Relaxed) {
            return;
        }
        let s = step.min(remaining);
        thread::sleep(s);
        remaining = remaining.saturating_sub(s);
    }
}

/// Run one device session. Returns the (still-live) `cmd_rx` so the
/// caller can use it for the next session.
fn run_session(
    name: &str,
    handle: DeviceHandle,
    frames_tx: &mpsc::Sender<RawFrame>,
    cmd_rx: mpsc::Receiver<WriterCmd>,
    stop: &Arc<AtomicBool>,
) -> mpsc::Receiver<WriterCmd> {
    let device_sender = handle.frame_sender();
    let stop_for_fwd = stop.clone();
    // Per-session shutdown flag. Set by the main thread once the
    // device read side dies; tells the cmd forwarder to drop its
    // device_sender clone and exit, which lets the writer thread
    // close and the next session start cleanly.
    let session_done = Arc::new(AtomicBool::new(false));
    let session_done_for_fwd = session_done.clone();
    let (return_tx, return_rx) = mpsc::channel::<mpsc::Receiver<WriterCmd>>();
    let fwd_name = format!("{name}-cmd-fwd");

    // Forwarder: stable cmd_rx -> current session's device sender.
    // Owns cmd_rx for the duration of the session; returns it via
    // return_tx when it exits so the supervisor can reuse it.
    let cmd_join = thread::Builder::new()
        .name(fwd_name)
        .spawn(move || {
            loop {
                if stop_for_fwd.load(Ordering::Relaxed)
                    || session_done_for_fwd.load(Ordering::Relaxed)
                {
                    break;
                }
                // recv_timeout so we can periodically observe `stop`
                // / `session_done` even when no commands are flowing.
                match cmd_rx.recv_timeout(Duration::from_millis(250)) {
                    Ok(WriterCmd::Frame(f)) => {
                        if device_sender.send_frame(f).is_err() {
                            // Device writer is gone — session over.
                            break;
                        }
                    }
                    Ok(WriterCmd::Bytes(_)) => {
                        // SendBytes only comes from the device-side
                        // decoder back to the device writer; should
                        // never reach the supervisor's stable channel.
                    }
                    Err(mpsc::RecvTimeoutError::Timeout) => continue,
                    Err(mpsc::RecvTimeoutError::Disconnected) => break,
                }
            }
            // Drop our device_sender clone here so the device's
            // writer thread can see cmd_rx disconnect and exit. The
            // local `device_sender` goes out of scope at the closing
            // brace below, which is sufficient.
            let _ = return_tx.send(cmd_rx);
        })
        .expect("spawn supervisor cmd forwarder");

    // Main: pump frames from the active session into the stable
    // outer channel. Exits when the device's frames_rx closes
    // (device died) or when `stop` is set.
    //
    // Greedy drain: block once, then forward everything already queued
    // via `try_recv` before parking again. The device worker reads the
    // bus in `recvmmsg` batches and hands us the frames back-to-back, so
    // this parks (and wakes the outer consumer) roughly once per batch
    // instead of once per frame — the bulk of the RX-path context-switch
    // and futex churn on a busy bus.
    'pump: while let Ok(frame) = handle.frames_rx.recv() {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        if frames_tx.send(frame).is_err() {
            // Outer consumer is gone — supervisor will exit too.
            break;
        }
        loop {
            match handle.frames_rx.try_recv() {
                Ok(frame) => {
                    if frames_tx.send(frame).is_err() {
                        break 'pump;
                    }
                }
                // Nothing more queued right now — go back to blocking.
                Err(mpsc::TryRecvError::Empty) => break,
                // Device session ended — let the outer `recv` observe it.
                Err(mpsc::TryRecvError::Disconnected) => break 'pump,
            }
        }
    }

    // Signal the forwarder to exit. It will drop its device_sender,
    // which (together with our `drop(handle)` below) closes the
    // device's writer cmd_rx and lets the writer thread exit.
    session_done.store(true, Ordering::Relaxed);
    drop(handle);

    // Wait for the forwarder to finish so we can recover cmd_rx.
    let _ = cmd_join.join();
    return_rx
        .recv()
        .expect("cmd forwarder always returns cmd_rx")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::{DeviceDecoder, DeviceEncoder, DeviceEvent, run as run_device};
    use std::io::{self, Read, Write};
    use std::sync::Mutex;
    use std::sync::atomic::AtomicU32;

    /// Test decoder: every byte becomes a fake `RawFrame` whose `pgn`
    /// equals that byte. Lets the test feed deterministic byte
    /// streams in and verify the same bytes pop out as frames.
    struct TestDecoder;
    impl DeviceDecoder for TestDecoder {
        fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
            for &b in bytes {
                events.push(DeviceEvent::Frame(RawFrame::new(
                    None,
                    0,
                    b as u32,
                    0,
                    0,
                    std::iter::empty(),
                )));
            }
        }
    }

    struct TestEncoder;
    impl DeviceEncoder for TestEncoder {
        fn encode_frame(&self, _frame: &RawFrame) -> Option<Vec<u8>> {
            None
        }
    }

    /// `Read` impl that emits a fixed byte stream once, then EOFs on
    /// the next call. Simulates a device that disconnects after
    /// delivering its payload.
    struct OneShotReader(Option<Vec<u8>>);
    impl Read for OneShotReader {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match self.0.take() {
                Some(data) => {
                    let n = data.len().min(buf.len());
                    buf[..n].copy_from_slice(&data[..n]);
                    Ok(n)
                }
                None => Ok(0),
            }
        }
    }

    struct SinkWriter;
    impl Write for SinkWriter {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// Factory that opens a fresh `OneShotReader` for each call,
    /// emitting a different byte each session so we can tell sessions
    /// apart. Tracks how many sessions have been opened.
    struct ReconnectFactory {
        sessions: Arc<AtomicU32>,
        bytes_per_session: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl DeviceFactory for ReconnectFactory {
        fn open(&mut self) -> io::Result<DeviceHandle> {
            let n = self.sessions.fetch_add(1, Ordering::Relaxed) as usize;
            let payload = {
                let table = self.bytes_per_session.lock().unwrap();
                table.get(n).cloned().unwrap_or_default()
            };
            if payload.is_empty() {
                // Simulate "device not ready yet" until we exhaust the
                // table; the supervisor should back off and retry.
                return Err(io::Error::other("no payload for session"));
            }
            let reader: Box<dyn Read + Send> = Box::new(OneShotReader(Some(payload)));
            let writer: Box<dyn Write + Send> = Box::new(SinkWriter);
            Ok(run_device(TestDecoder, TestEncoder, reader, writer))
        }
    }

    #[test]
    fn supervisor_reconnects_across_eof() {
        let sessions = Arc::new(AtomicU32::new(0));
        let payloads = Arc::new(Mutex::new(vec![
            vec![1u8, 2, 3],
            vec![], // simulate transient open failure
            vec![10u8, 20],
            vec![100u8, 101],
        ]));
        let factory = ReconnectFactory {
            sessions: sessions.clone(),
            bytes_per_session: payloads,
        };
        let sup = Supervisor::new(factory);

        // Collect frames until we've seen all the expected payloads
        // or we time out.
        //
        // The happy path spends ~2s in real sleeps — 500ms after each of
        // the two mid-stream EOFs plus one INITIAL_BACKOFF for the
        // simulated open failure — and the loop breaks the moment all 7
        // frames land, so a generous ceiling costs nothing when the test
        // passes. It only bounds a genuine hang. Keep it far above the
        // expected time: a loaded CI runner that stretches those sleeps
        // must not be mistaken for a dropped frame (a 5s deadline flaked
        // on macOS, timing out one frame short of the last session).
        let mut got: Vec<u32> = Vec::new();
        let deadline = std::time::Instant::now() + Duration::from_secs(60);
        while got.len() < 7 && std::time::Instant::now() < deadline {
            match sup.frames_rx.recv_timeout(Duration::from_millis(500)) {
                Ok(f) => got.push(f.pgn),
                Err(_) => continue,
            }
        }
        sup.shutdown();

        assert_eq!(
            got,
            vec![1, 2, 3, 10, 20, 100, 101],
            "expected all 3 sessions' bytes in order"
        );
        // Sessions: opened=4 (3 successful + 1 failed open), plus
        // maybe more before shutdown lands. Just check ≥ 4.
        assert!(
            sessions.load(Ordering::Relaxed) >= 4,
            "expected ≥ 4 open attempts (3 successful + 1 transient failure)"
        );
    }
}
