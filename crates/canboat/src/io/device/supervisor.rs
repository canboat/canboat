// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Reconnect supervisor for [`super::DeviceHandle`].
//!
//! A bare `DeviceHandle` lasts exactly one device session: when the
//! serial port disconnects or the TCP socket dies, the reader thread
//! returns and the `frames_rx` channel closes. That's fine for a
//! one-shot command-line tool but unacceptable for a long-running
//! service like `canboat server`.
//!
//! [`supervise`] wraps a [`DeviceFactory`] (a thunk that knows how to
//! open the device) in a `DeviceHandle` whose `frames_rx` and
//! [`FrameSender`](super::FrameSender) survive any number of disconnects. On failure it logs,
//! backs off exponentially, and reopens the device. Frames sent while the
//! device is down wait in the handle's command channel, and go to the
//! next session's writer once it is up.

use std::io;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::Duration;

use crate::engine::RawFrame;

use super::{Closed, DeviceFailure, DeviceHandle, StopOnDrop, WriterCmd};

/// Factory thunk handed to [`supervise`]. Each call to `open` must
/// produce a *fresh* `DeviceHandle` — the supervisor never reuses one
/// across sessions.
pub trait DeviceFactory<F = RawFrame>: Send + 'static {
    fn open(&mut self) -> io::Result<DeviceHandle<F>>;

    /// Short label for log output (e.g. `"ngt1"` / `"maretron"`).
    fn name(&self) -> &str {
        "device"
    }
}

/// Convenient blanket impl so callers can pass an `FnMut() -> io::Result<DeviceHandle>`.
impl<F, T> DeviceFactory<F> for T
where
    T: FnMut() -> io::Result<DeviceHandle<F>> + Send + 'static,
{
    fn open(&mut self) -> io::Result<DeviceHandle<F>> {
        (self)()
    }
}

/// A [`DeviceFactory`] with a name for the log lines.
pub(crate) struct Named<T> {
    pub(crate) name: String,
    pub(crate) open: T,
}

impl<F, T> DeviceFactory<F> for Named<T>
where
    T: FnMut() -> io::Result<DeviceHandle<F>> + Send + 'static,
{
    fn open(&mut self) -> io::Result<DeviceHandle<F>> {
        (self.open)()
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Initial backoff after the first failure. Doubles each retry up to
/// [`MAX_BACKOFF`].
pub const INITIAL_BACKOFF: Duration = Duration::from_millis(500);
/// Cap on the exponential backoff so a sustained outage doesn't
/// stretch reconnect attempts past 30 s.
pub const MAX_BACKOFF: Duration = Duration::from_secs(30);

/// The [`DeviceCloser`](super::DeviceCloser) of a supervised handle
/// waiting for the session to close: told whether it did.
type Done = Arc<Mutex<Option<mpsc::Sender<bool>>>>;

/// Run `factory`'s device sessions one after another, reconnecting
/// whenever one ends, behind one long-lived [`DeviceHandle`]: its
/// `frames_rx` and [`frame_sender`](DeviceHandle::frame_sender) carry on
/// across reconnects, its [`failure`](DeviceHandle::failure) is set when
/// a session found the wrong device (and the frames end), and
/// [`close`](DeviceHandle::close) closes the current session and stops.
/// So does letting go of the handle.
pub(crate) fn supervise<F: Send + 'static>(factory: impl DeviceFactory<F>) -> DeviceHandle<F> {
    let (frames_tx, frames_rx) = mpsc::channel::<F>();
    let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd<F>>();
    let stop = Arc::new(AtomicBool::new(false));
    let failure = DeviceFailure::default();
    let progress = Arc::new(AtomicU64::new(0));
    let join = {
        let (stop, failure, progress) = (stop.clone(), failure.clone(), progress.clone());
        thread::Builder::new()
            .name("device-supervisor".into())
            .spawn(move || supervisor_loop(factory, frames_tx, cmd_rx, &stop, &failure, &progress))
            .expect("spawn device supervisor")
    };
    DeviceHandle {
        frames_rx,
        cmd_tx,
        joins: vec![join],
        writer: None,
        progress,
        failure,
        lost: DeviceFailure::default(),
        stop: Some(StopOnDrop(stop)),
        claim: None,
    }
}

fn supervisor_loop<F: Send + 'static>(
    mut factory: impl DeviceFactory<F>,
    frames_tx: mpsc::Sender<F>,
    cmd_rx: mpsc::Receiver<WriterCmd<F>>,
    stop: &Arc<AtomicBool>,
    failure: &DeviceFailure,
    progress: &AtomicU64,
) {
    let mut backoff = INITIAL_BACKOFF;
    // Hold cmd_rx in an Option so we can move it into the per-session
    // forwarder and get it back via a one-shot return channel.
    let mut held_cmd_rx = Some(cmd_rx);
    let done: Done = Arc::default();

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
                sleep_interruptible(backoff, stop);
                backoff = (backoff * 2).min(MAX_BACKOFF);
                continue;
            }
        };

        let cmd_rx = held_cmd_rx
            .take()
            .expect("held_cmd_rx is always Some at session start");
        let session_failure = handle.failure();
        let session_lost = handle.lost();
        held_cmd_rx = Some(run_session(
            factory.name(),
            handle,
            &frames_tx,
            cmd_rx,
            stop,
            &done,
            progress,
        ));

        if stop.load(Ordering::Relaxed) {
            break;
        }
        // The wrong device stays the wrong device: stop, and say why.
        if let Some(why) = session_failure.get() {
            log::error!("{}: giving up: {why}", factory.name());
            failure.set(why);
            break;
        }
        match session_lost.get() {
            Some(why) => log::warn!("{}: session ended: {why}; will reconnect", factory.name()),
            None => log::warn!("{}: session ended; will reconnect", factory.name()),
        }
        sleep_interruptible(Duration::from_millis(500), stop);
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
fn run_session<F: Send + 'static>(
    name: &str,
    handle: DeviceHandle<F>,
    frames_tx: &mpsc::Sender<F>,
    cmd_rx: mpsc::Receiver<WriterCmd<F>>,
    stop: &Arc<AtomicBool>,
    done: &Done,
    progress: &AtomicU64,
) -> mpsc::Receiver<WriterCmd<F>> {
    let device_sender = handle.frame_sender();
    let stop_for_fwd = stop.clone();
    let done_for_fwd = done.clone();
    // Per-session shutdown flag. Set by the main thread once the
    // device read side dies; tells the cmd forwarder to drop its
    // device_sender clone and exit, which lets the writer thread
    // close and the next session start cleanly.
    let session_done = Arc::new(AtomicBool::new(false));
    let session_done_for_fwd = session_done.clone();
    let (return_tx, return_rx) = mpsc::channel::<mpsc::Receiver<WriterCmd<F>>>();
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
                    // The handle is being closed (its closer set `stop`
                    // too): the session closes below, and says so.
                    Ok(WriterCmd::Shutdown(Some(tx))) => {
                        *super::lock(&done_for_fwd) = Some(tx);
                        stop_for_fwd.store(true, Ordering::Relaxed);
                        break;
                    }
                    Ok(WriterCmd::Bytes(_) | WriterCmd::Shutdown(None)) => {
                        // SendBytes only comes from the device-side
                        // decoder back to the device writer; neither
                        // reaches the supervisor's stable channel.
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
    //
    // `recv_timeout` rather than `recv`, so a stop is noticed on a quiet
    // bus too, not only when the next frame arrives.
    'pump: loop {
        let frame = match handle.frames_rx.recv_timeout(Duration::from_millis(250)) {
            Ok(frame) => frame,
            Err(mpsc::RecvTimeoutError::Timeout) if stop.load(Ordering::Relaxed) => break,
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        if stop.load(Ordering::Relaxed) {
            break;
        }
        if frames_tx.send(frame).is_err() {
            // Outer consumer is gone: nobody wants another session.
            stop.store(true, Ordering::Relaxed);
            break;
        }
        loop {
            match handle.frames_rx.try_recv() {
                Ok(frame) => {
                    if frames_tx.send(frame).is_err() {
                        stop.store(true, Ordering::Relaxed);
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
    // Wait for the forwarder to finish so we can recover cmd_rx, and
    // learn whether a close is waiting to hear how it went.
    let _ = cmd_join.join();
    if stop.load(Ordering::Relaxed) {
        // Closing on purpose: let the device say goodbye (an iKonvert
        // goes off the bus) and wait until it has. A device that just
        // died, or one about to be reconnected, is left alone.
        log::info!("{name}: closing the device");
        // The close has begun: the handle's closer waits on.
        progress.fetch_add(1, Ordering::Relaxed);
        let closed = handle.close();
        if let Some(tx) = super::lock(done).take() {
            match closed {
                Closed::Confirmed => drop(tx.send(true)),
                Closed::Unconfirmed => drop(tx.send(false)),
                // Dropped: the closer hears the device had gone.
                Closed::Gone => {}
            }
        }
    } else {
        drop(handle);
    }
    return_rx
        .recv()
        .expect("cmd forwarder always returns cmd_rx")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::device::{Closed, DeviceDecoder, DeviceEncoder, DeviceEvent, run as run_device};
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

    /// A session that ends because the device is the wrong one is not
    /// reopened: the supervisor stops, its frames end, and it says why.
    #[test]
    fn a_wrong_device_is_not_reopened() {
        use crate::engine::codec::{Codec, Event, Refused};
        struct Wrong;
        impl Codec for Wrong {
            fn receive(&mut self, _bytes: &[u8], _now_ms: u64, _events: &mut Vec<Event>) {}
            fn send(&mut self, _frame: &RawFrame) -> Result<Vec<u8>, Refused> {
                Ok(Vec::new())
            }
            fn failure(&self) -> Option<&str> {
                Some("not the device")
            }
        }
        let opened = Arc::new(AtomicU32::new(0));
        let count = opened.clone();
        let sup = supervise(move || {
            count.fetch_add(1, Ordering::Relaxed);
            Ok(super::super::run_codec(
                Wrong,
                None,
                Box::new(OneShotReader(Some(vec![1]))),
                Box::new(SinkWriter),
            ))
        });
        let failure = sup.failure();
        assert!(matches!(
            sup.frames_rx.recv_timeout(Duration::from_secs(5)),
            Err(mpsc::RecvTimeoutError::Disconnected)
        ));
        assert_eq!(failure.get(), Some("not the device"));
        assert_eq!(opened.load(Ordering::Relaxed), 1);
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
        let sup = supervise(factory);

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
        drop(sup);

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

    #[test]
    fn a_supervised_handle_carries_on_across_sessions() {
        let factory = ReconnectFactory {
            sessions: Arc::default(),
            bytes_per_session: Arc::new(Mutex::new(vec![vec![1u8, 2], vec![3u8, 4]])),
        };
        let handle = supervise(factory);
        let mut got = Vec::new();
        let deadline = std::time::Instant::now() + Duration::from_secs(60);
        while got.len() < 4 && std::time::Instant::now() < deadline {
            if let Ok(f) = handle.frames_rx.recv_timeout(Duration::from_millis(500)) {
                got.push(f.pgn);
            }
        }
        assert_eq!(got, [1, 2, 3, 4]);
        assert!(handle.failure().get().is_none());
    }

    /// Letting go of a supervised handle stops the reconnecting.
    #[test]
    fn dropping_a_supervised_handle_stops_it() {
        let sessions = Arc::new(AtomicU32::new(0));
        let count = sessions.clone();
        let handle = supervise(move || {
            count.fetch_add(1, Ordering::Relaxed);
            Ok(run_device(
                TestDecoder,
                TestEncoder,
                Box::new(OneShotReader(Some(vec![1]))),
                Box::new(SinkWriter),
            ))
        });
        assert!(
            handle
                .frames_rx
                .recv_timeout(Duration::from_secs(10))
                .is_ok()
        );
        drop(handle);
        thread::sleep(Duration::from_millis(1500));
        let after = sessions.load(Ordering::Relaxed);
        thread::sleep(Duration::from_millis(1500));
        assert_eq!(sessions.load(Ordering::Relaxed), after);
    }

    /// A device that stays connected but quiet: reads time out forever.
    struct QuietReader;
    impl Read for QuietReader {
        fn read(&mut self, _buf: &mut [u8]) -> io::Result<usize> {
            thread::sleep(Duration::from_millis(20));
            Err(io::ErrorKind::TimedOut.into())
        }
    }

    /// A writer that keeps what it is given.
    #[derive(Clone, Default)]
    struct RecordingWriter(Arc<Mutex<Vec<u8>>>);
    impl Write for RecordingWriter {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// An encoder that says goodbye.
    struct ByeEncoder;
    impl DeviceEncoder for ByeEncoder {
        fn encode_frame(&self, _frame: &RawFrame) -> Option<Vec<u8>> {
            None
        }
        fn shutdown_bytes(&self) -> Vec<u8> {
            b"BYE".to_vec()
        }
    }

    fn quiet_device(written: &RecordingWriter) -> DeviceHandle {
        run_device(
            TestDecoder,
            ByeEncoder,
            Box::new(QuietReader),
            Box::new(written.clone()),
        )
    }

    #[test]
    fn closing_a_device_writes_its_goodbye() {
        let written = RecordingWriter::default();
        assert_eq!(quiet_device(&written).close(), Closed::Confirmed);
        assert_eq!(*written.0.lock().unwrap(), b"BYE");
    }

    #[test]
    fn a_closer_closes_from_another_thread() {
        let written = RecordingWriter::default();
        let handle = quiet_device(&written);
        let closer = handle.closer();
        assert_eq!(
            thread::spawn(move || closer.close(Duration::from_secs(5)))
                .join()
                .unwrap(),
            Closed::Confirmed
        );
        assert_eq!(*written.0.lock().unwrap(), b"BYE");
    }

    /// A writer that fails every write.
    struct BrokenWriter;
    impl Write for BrokenWriter {
        fn write(&mut self, _buf: &[u8]) -> io::Result<usize> {
            Err(io::ErrorKind::BrokenPipe.into())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// A goodbye that could not be written is not confirmed.
    #[test]
    fn a_failed_goodbye_is_not_confirmed() {
        let handle = run_device(
            TestDecoder,
            ByeEncoder,
            Box::new(QuietReader),
            Box::new(BrokenWriter),
        );
        assert_eq!(handle.close(), Closed::Unconfirmed);
    }

    /// Closing a supervised handle closes the session on purpose, and
    /// says how that went.
    #[test]
    fn closing_a_supervised_handle_says_goodbye() {
        let written = RecordingWriter::default();
        let for_factory = written.clone();
        let handle = supervise(move || Ok(quiet_device(&for_factory)));
        thread::sleep(Duration::from_millis(300)); // let the session open
        assert_eq!(handle.close(), Closed::Confirmed);
        assert_eq!(*written.0.lock().unwrap(), b"BYE");
    }

    /// Dropping a supervised handle closes the device on purpose, even on a
    /// quiet bus; a device that merely went away is not told goodbye.
    #[test]
    fn stopping_the_supervisor_says_goodbye() {
        let written = RecordingWriter::default();
        let for_factory = written.clone();
        let sup = supervise(move || Ok(quiet_device(&for_factory)));
        thread::sleep(Duration::from_millis(300)); // let the session open
        drop(sup); // stop: the manager closes the device
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        while written.0.lock().unwrap().is_empty() && std::time::Instant::now() < deadline {
            thread::sleep(Duration::from_millis(50));
        }
        assert_eq!(*written.0.lock().unwrap(), b"BYE");

        let written = RecordingWriter::default();
        let for_factory = written.clone();
        let sup = supervise(move || {
            Ok(run_device(
                TestDecoder,
                ByeEncoder,
                Box::new(OneShotReader(Some(vec![1]))),
                Box::new(for_factory.clone()),
            ))
        });
        let _ = sup.frames_rx.recv_timeout(Duration::from_secs(5));
        thread::sleep(Duration::from_millis(200)); // the device goes away
        assert!(written.0.lock().unwrap().is_empty(), "no goodbye on EOF");
        drop(sup);
    }

    /// A writer that takes `delay` per write, or never returns (`None`).
    struct SlowWriter(Option<Duration>, Arc<Mutex<Vec<u8>>>);
    impl Write for SlowWriter {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            match self.0 {
                Some(d) => thread::sleep(d),
                None => loop {
                    thread::sleep(Duration::from_secs(3600));
                },
            }
            self.1.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// Encodes every frame as one byte.
    struct ByteEncoder;
    impl DeviceEncoder for ByteEncoder {
        fn encode_frame(&self, _frame: &RawFrame) -> Option<Vec<u8>> {
            Some(vec![1])
        }
        fn shutdown_bytes(&self) -> Vec<u8> {
            b"BYE".to_vec()
        }
    }

    fn frame() -> RawFrame {
        RawFrame::new(None, 0, 1, 0, 0, std::iter::empty())
    }

    /// A backlog longer than the timeout still drains, goodbye included,
    /// because the writer keeps making progress.
    ///
    /// The backlog (30 writes of 20 ms, ~600 ms) outlasts the timeout, while
    /// a single write would have to stall for ~480 ms to look stuck. With
    /// 60 ms writes against 150 ms, one oversleep on a busy macOS runner
    /// failed it (#928).
    #[test]
    fn a_long_backlog_drains_before_the_goodbye() {
        let written = Arc::new(Mutex::new(Vec::new()));
        let handle = run_device(
            TestDecoder,
            ByteEncoder,
            Box::new(QuietReader),
            Box::new(SlowWriter(Some(Duration::from_millis(20)), written.clone())),
        );
        for _ in 0..30 {
            handle.send_frame(frame()).unwrap();
        }
        let closer = handle.closer();
        assert_eq!(closer.close(Duration::from_millis(500)), Closed::Confirmed);
        assert!(written.lock().unwrap().ends_with(b"BYE"));
    }

    /// A writer that stops making progress is given up on.
    #[test]
    fn a_stuck_writer_is_unconfirmed() {
        let handle = run_device(
            TestDecoder,
            ByteEncoder,
            Box::new(QuietReader),
            Box::new(SlowWriter(None, Arc::default())),
        );
        handle.send_frame(frame()).unwrap();
        assert_eq!(
            handle.closer().close(Duration::from_millis(150)),
            Closed::Unconfirmed
        );
    }

    /// A device whose writer has already stopped (a failed write) has
    /// nothing left to close: that is not a failed close.
    #[test]
    fn a_device_that_went_away_is_gone() {
        let handle = run_device(
            TestDecoder,
            ByteEncoder,
            Box::new(QuietReader),
            Box::new(BrokenWriter),
        );
        handle.send_frame(frame()).unwrap(); // the writer fails and stops
        thread::sleep(Duration::from_millis(100));
        assert_eq!(handle.close(), Closed::Gone);
    }
}
