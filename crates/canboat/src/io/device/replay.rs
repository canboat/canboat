// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Replay device: plays a capture file as if it were a live gateway.
//!
//! The reader thread reads the capture line by line in any format canboat
//! reads (PLAIN, FAST, candump, Actisense ASCII, YDWG-02, …), waits between
//! frames as long as their timestamps were apart (see [`Pacer`]), and hands
//! each frame on stamped with the current time, so a consumer sees live data
//! rather than the capture's old dates. At the end of the file it starts
//! over, so the bus never goes quiet.
//!
//! A frame without a timestamp is sent [`UNTIMED_GAP`] after the previous
//! one, so a capture without timestamps still plays at a finite rate.
//!
//! Frames sent to the device are written as canboat PLAIN/FAST lines to
//! the `sent` file when one is given, stamped with the current time when
//! they carry none, and dropped otherwise. As on a real
//! bus they are not echoed back into the frames read, so a consumer that
//! both reads and writes does not see its own output come back.

use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::engine::format::timestamp::{normalize_timestamp, to_unix_ms};
use crate::engine::format::{
    InputFormat, detect, header_implies_coalesced, parse_for, parse_format_header,
};
use crate::engine::{BusProtocol, RawFrame, format_iso_ms};

use super::{DeviceEncoder, DeviceHandle, WriterCmd, canboat_csv, writer_thread};

/// How long a frame without a timestamp waits after the previous one.
pub const UNTIMED_GAP: Duration = Duration::from_millis(10);

/// Frames [`scan`] reads at most before deciding the capture is not
/// coalesced.
const SCAN_FRAMES: usize = 10_000;

#[derive(Debug, Clone)]
pub struct Config {
    /// The capture to play.
    pub path: PathBuf,
    /// What the bus carries; picks how candump and PLAIN lines are read.
    pub protocol: BusProtocol,
    /// Where frames sent to the device are written, as PLAIN/FAST lines
    /// appended to the file. `None` drops them.
    pub sent: Option<PathBuf>,
    /// Gaps between timestamps of this length or more are not waited for.
    pub max_gap: Duration,
}

impl Config {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self {
            path: path.into(),
            protocol: BusProtocol::Nmea2000,
            sent: None,
            max_gap: Duration::from_secs(10),
        }
    }
}

/// What [`scan`] learned about a capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Scan {
    /// Its frames are whole PGNs already (FAST, Actisense, …), so the
    /// pipeline must not reassemble them.
    pub coalesced: bool,
}

/// Check that `path` holds frames canboat can read, and whether they are
/// coalesced: a `# format=` header says so, or else a frame of more than
/// 8 bytes among the first [`SCAN_FRAMES`].
pub fn scan(path: &Path, protocol: BusProtocol) -> io::Result<Scan> {
    let reader = BufReader::new(File::open(path)?);
    let mut format: Option<InputFormat> = None;
    let mut header_coalesced: Option<bool> = None;
    let mut frames = 0;
    let mut long_frame = false;
    for line in reader.lines() {
        let line = line?;
        let trimmed = line.trim_end_matches('\r');
        if trimmed.starts_with('#') {
            // The header names the format, as play_once reads it: a
            // Garmin CSV data row is not one `detect` recognises.
            if format.is_none()
                && let Some(fmt) = parse_format_header(trimmed)
            {
                format = Some(fmt);
                header_coalesced = Some(header_implies_coalesced(trimmed));
            }
            continue;
        }
        if trimmed.is_empty() {
            continue;
        }
        let fmt = *format.get_or_insert_with(|| detect(trimmed).unwrap_or(InputFormat::Plain));
        if let Ok(Some(frame)) = parse_for(fmt, protocol, trimmed) {
            frames += 1;
            long_frame |= frame.data.len() > 8;
            if header_coalesced.is_some() || long_frame || frames >= SCAN_FRAMES {
                break;
            }
        }
    }
    if frames == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("{}: no frames canboat can read", path.display()),
        ));
    }
    Ok(Scan {
        coalesced: header_coalesced.unwrap_or(long_frame),
    })
}

/// Start playing the capture. Fails when the capture or the `sent` file
/// cannot be opened.
pub fn run(config: Config) -> io::Result<DeviceHandle> {
    // Frames written to the capture itself would come back on the next
    // pass as bus traffic.
    if let Some(sent) = &config.sent
        && same_file(sent, &config.path)
    {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "{}: the replay capture and the sent file must differ",
                sent.display()
            ),
        ));
    }
    // Open now so a bad path fails the open rather than the thread.
    let file = File::open(&config.path)?;
    let mut writer: Box<dyn Write + Send> = match &config.sent {
        Some(path) => Box::new(OpenOptions::new().create(true).append(true).open(path)?),
        None => Box::new(io::sink()),
    };

    let (frames_tx, frames_rx) = mpsc::channel::<RawFrame>();
    let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd>();
    let progress = Arc::new(AtomicU64::new(0));
    let writer_progress = progress.clone();

    // The writer stops when it cannot write the sent file (it logs why)
    // or when the device closes. Either way the reader stops too, so the
    // closed frames channel tells the supervisor, which reconnects, rather
    // than the capture playing on while what is sent goes nowhere.
    let writing = Arc::new(AtomicBool::new(true));
    let reader_writing = writing.clone();
    let encoder = SentEncoder;
    let init = encoder.init_bytes();
    let writer_join = thread::Builder::new()
        .name("replay-writer".into())
        .spawn(move || {
            writer_thread(&mut *writer, encoder, init, None, cmd_rx, &writer_progress);
            writing.store(false, Ordering::Release);
        })?;
    let reader_join = thread::Builder::new()
        .name("replay-reader".into())
        .spawn(move || play(file, &config, &frames_tx, &reader_writing))?;

    Ok(DeviceHandle {
        frames_rx,
        cmd_tx,
        joins: vec![reader_join],
        writer: Some(writer_join),
        progress,
    })
}

/// Whether two paths name the same file, also through a link or `..`.
fn same_file(a: &Path, b: &Path) -> bool {
    match (a.canonicalize(), b.canonicalize()) {
        (Ok(a), Ok(b)) => a == b,
        _ => a == b,
    }
}

/// Play the capture over and over until the receiver goes away.
fn play(mut file: File, config: &Config, frames_tx: &mpsc::Sender<RawFrame>, writing: &AtomicBool) {
    let mut pass = 0u64;
    loop {
        pass += 1;
        match play_once(file, config, frames_tx, writing) {
            Ok(Played::Done(0)) => {
                log::error!("replay: {} holds no frames", config.path.display());
                return;
            }
            Ok(Played::Done(frames)) => {
                log::info!(
                    "replay: pass {pass} of {} sent {frames} frames; starting over",
                    config.path.display()
                );
            }
            Ok(Played::Stopped) => return,
            Err(e) => {
                log::error!("replay: reading {}: {e}", config.path.display());
                return;
            }
        }
        file = match File::open(&config.path) {
            Ok(f) => f,
            Err(e) => {
                log::error!("replay: reopening {}: {e}", config.path.display());
                return;
            }
        };
    }
}

enum Played {
    /// Reached the end of the file after sending this many frames.
    Done(u64),
    /// The receiver went away.
    Stopped,
}

fn play_once(
    file: File,
    config: &Config,
    frames_tx: &mpsc::Sender<RawFrame>,
    writing: &AtomicBool,
) -> io::Result<Played> {
    let mut reader = BufReader::new(file);
    let mut line = String::with_capacity(512);
    let mut format: Option<InputFormat> = None;
    let mut pacer = Pacer::new(config.max_gap);
    let mut frames = 0u64;
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            return Ok(Played::Done(frames));
        }
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.is_empty() {
            continue;
        }
        if trimmed.starts_with('#') {
            if format.is_none() {
                format = parse_format_header(trimmed);
            }
            continue;
        }
        let fmt = *format.get_or_insert_with(|| detect(trimmed).unwrap_or(InputFormat::Plain));
        let Ok(Some(mut frame)) = parse_for(fmt, config.protocol, trimmed) else {
            continue;
        };
        let at_ms = frame.timestamp.as_deref().and_then(timestamp_ms);
        thread::sleep(pacer.wait(at_ms));
        frame.timestamp = Some(now_iso());
        if !writing.load(Ordering::Acquire) || frames_tx.send(frame).is_err() {
            return Ok(Played::Stopped);
        }
        frames += 1;
    }
}

/// Spaces frames out as far as their timestamps were apart.
///
/// It paces against the latest timestamp seen, not the previous frame's: a
/// capture can interleave frames stamped by two clocks (a host clock with
/// milliseconds and a device clock seconds behind it), and pacing frame to
/// frame would wait out the difference again every time the stamps switch
/// back. A frame stamped before the latest is sent at once, without
/// lowering it. A step forward of `max_gap` or more is not waited for, as
/// in `canboat replay`. A step back of that much starts the pacing over
/// only once [`RESYNC_FRAMES`] frames in a row are that far behind: that is
/// a time of day passing midnight or a second capture appended to the
/// first, where a lone frame that far behind is a clock that drifted.
struct Pacer {
    max_gap: i64,
    latest_ms: Option<i64>,
    /// Frames in a row stamped `max_gap` or more before `latest_ms`.
    behind: u32,
}

/// Frames in a row stamped `max_gap` or more before the latest that make
/// [`Pacer`] start over.
const RESYNC_FRAMES: u32 = 50;

impl Pacer {
    fn new(max_gap: Duration) -> Self {
        Self {
            max_gap: max_gap.as_millis().try_into().unwrap_or(i64::MAX),
            latest_ms: None,
            behind: 0,
        }
    }

    /// How long to wait before sending a frame stamped `at_ms`.
    fn wait(&mut self, at_ms: Option<i64>) -> Duration {
        let Some(at) = at_ms else {
            return UNTIMED_GAP;
        };
        let Some(latest) = self.latest_ms else {
            self.latest_ms = Some(at);
            return Duration::ZERO;
        };
        let ahead = at - latest;
        if ahead <= -self.max_gap {
            self.behind += 1;
            if self.behind >= RESYNC_FRAMES {
                self.behind = 0;
                self.latest_ms = Some(at);
            }
            return Duration::ZERO;
        }
        self.behind = 0;
        if ahead <= 0 {
            return Duration::ZERO;
        }
        self.latest_ms = Some(at);
        if ahead < self.max_gap {
            Duration::from_millis(ahead as u64)
        } else {
            Duration::ZERO
        }
    }
}

/// The canboat-CSV encoder, stamping a frame that carries no timestamp (an
/// ISO Request the server makes itself) with the current time, so every
/// line in the `sent` file says when it was sent.
struct SentEncoder;

impl DeviceEncoder for SentEncoder {
    fn init_bytes(&self) -> Vec<u8> {
        canboat_csv::Encoder.init_bytes()
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.timestamp.is_some() {
            return canboat_csv::Encoder.encode_frame(frame);
        }
        let mut stamped = frame.clone();
        stamped.timestamp = Some(now_iso());
        canboat_csv::Encoder.encode_frame(&stamped)
    }
}

/// A frame timestamp in milliseconds: since the epoch when it has a date,
/// since midnight when it is a time of day only (Actisense ASCII, YDWG-02).
fn timestamp_ms(ts: &str) -> Option<i64> {
    to_unix_ms(ts).or_else(|| time_of_day_ms(&normalize_timestamp(ts)))
}

/// `HH:MM:SS[.mmm]` in milliseconds since midnight.
fn time_of_day_ms(ts: &str) -> Option<i64> {
    let b = ts.as_bytes();
    if b.len() < 8 || b[2] != b':' || b[5] != b':' {
        return None;
    }
    let h: i64 = ts.get(0..2)?.parse().ok()?;
    let m: i64 = ts.get(3..5)?.parse().ok()?;
    let s: i64 = ts.get(6..8)?.parse().ok()?;
    let ms: i64 = match ts.get(8..9) {
        Some(".") => ts.get(9..12).and_then(|f| f.parse().ok()).unwrap_or(0),
        _ => 0,
    };
    Some(((h * 60 + m) * 60 + s) * 1000 + ms)
}

fn now_iso() -> String {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    format_iso_ms(ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LINES: &str = "# format=FAST\n\
        2016-02-28T19:57:02.364Z,7,65280,7,255,8,3f,9f,12,00,00,00,ff,ff\n\
        2016-02-28T19:57:02.414Z,2,127250,1,255,8,ff,c7,63,ff,7f,ff,7f,fd\n";

    fn capture(name: &str, text: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("canboat-replay-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("capture.raw");
        std::fs::write(&path, text).unwrap();
        path
    }

    #[test]
    fn waits_as_long_as_the_timestamps_are_apart() {
        let mut p = Pacer::new(Duration::from_secs(10));
        assert_eq!(p.wait(Some(1_000)), Duration::ZERO);
        assert_eq!(p.wait(Some(1_050)), Duration::from_millis(50));
        assert_eq!(p.wait(Some(1_050)), Duration::ZERO);
        assert_eq!(p.wait(None), UNTIMED_GAP);
        // A long step forward is not waited for, but moves the pacing on.
        assert_eq!(p.wait(Some(20_000)), Duration::ZERO);
        assert_eq!(p.wait(Some(20_100)), Duration::from_millis(100));
    }

    /// Frames from a second clock a few seconds behind are sent at once, and
    /// the next frame of the first clock is not held back by them.
    #[test]
    fn paces_against_the_latest_timestamp() {
        let mut p = Pacer::new(Duration::from_secs(10));
        p.wait(Some(5_000));
        assert_eq!(p.wait(Some(2_000)), Duration::ZERO);
        assert_eq!(p.wait(Some(5_020)), Duration::from_millis(20));
        // A clock drifted further behind than max_gap does not move it either.
        assert_eq!(p.wait(Some(-20_000)), Duration::ZERO);
        assert_eq!(p.wait(Some(5_040)), Duration::from_millis(20));
    }

    /// Frames that stay far behind — a time of day passing midnight, or a
    /// second capture appended — start the pacing over.
    #[test]
    fn starts_over_when_the_frames_stay_far_behind() {
        let mut p = Pacer::new(Duration::from_secs(10));
        p.wait(Some(86_399_000));
        for i in 0..RESYNC_FRAMES {
            assert_eq!(p.wait(Some(500 + i as i64)), Duration::ZERO);
        }
        let last = 500 + RESYNC_FRAMES as i64 - 1;
        assert_eq!(p.wait(Some(last + 20)), Duration::from_millis(20));
    }

    #[test]
    fn reads_dated_and_time_of_day_timestamps() {
        assert_eq!(timestamp_ms("1970-01-01T00:00:01.500Z"), Some(1_500));
        assert_eq!(timestamp_ms("1970-01-01-00:00:01,500"), Some(1_500));
        assert_eq!(timestamp_ms("01:00:00.250"), Some(3_600_250));
        assert_eq!(timestamp_ms("garbage"), None);
    }

    #[test]
    fn scan_reads_the_format_header() {
        let path = capture("header", LINES);
        assert_eq!(
            scan(&path, BusProtocol::Nmea2000).unwrap(),
            Scan { coalesced: true }
        );
        let plain = capture(
            "plain",
            "2016-02-28T19:57:02.364Z,7,65280,7,255,8,3f,9f,12,00,00,00,ff,ff\n",
        );
        assert_eq!(
            scan(&plain, BusProtocol::Nmea2000).unwrap(),
            Scan { coalesced: false }
        );
        let bytes = vec!["00"; 43].join(",");
        let long = capture(
            "long",
            &format!("2016-02-28T19:57:02.364Z,6,129029,3,255,43,{bytes}\n"),
        );
        assert_eq!(
            scan(&long, BusProtocol::Nmea2000).unwrap(),
            Scan { coalesced: true }
        );
    }

    /// A Garmin CSV data row is not a format `detect` recognises; the
    /// header names it, as play_once reads it.
    #[test]
    fn scan_takes_the_format_from_the_header() {
        let path = capture(
            "garmin",
            "# format=GARMIN_CSV1\n\
             0,486942,127508,Battery Status,Garmin,6,255,2,1,8,0x017505FF7FFFFFFF\n",
        );
        assert_eq!(
            scan(&path, BusProtocol::Nmea2000).unwrap(),
            Scan { coalesced: true }
        );
    }

    /// Sent frames written into the capture would come back as bus traffic.
    #[test]
    fn refuses_to_send_into_the_capture_it_plays() {
        let path = capture("same", LINES);
        let mut config = Config::new(&path);
        config.sent = Some(path.parent().unwrap().join(".").join("capture.raw"));
        assert_eq!(
            run(config).err().map(|e| e.kind()),
            Some(io::ErrorKind::InvalidInput)
        );
    }

    /// Joining stops a replay nobody reads from: its reader only ends
    /// when a send fails.
    #[test]
    fn join_stops_a_replay_nobody_reads() {
        let handle = run(Config::new(capture("join", LINES))).unwrap();
        let (done_tx, done_rx) = mpsc::channel();
        thread::spawn(move || {
            handle.join();
            let _ = done_tx.send(());
        });
        assert!(
            done_rx.recv_timeout(Duration::from_secs(5)).is_ok(),
            "join did not return"
        );
    }

    /// Once the writer has stopped (it could not write the sent file), the
    /// capture is not played on into the void.
    #[test]
    fn stops_playing_when_the_writer_has_stopped() {
        let path = capture("writer-gone", LINES);
        let (frames_tx, frames_rx) = mpsc::channel();
        let played = play_once(
            File::open(&path).unwrap(),
            &Config::new(&path),
            &frames_tx,
            &AtomicBool::new(false),
        )
        .unwrap();
        assert!(matches!(played, Played::Stopped));
        assert!(frames_rx.try_recv().is_err());
    }

    #[test]
    fn scan_rejects_a_capture_without_frames() {
        let path = capture("empty", "# nothing here\n\n");
        assert_eq!(
            scan(&path, BusProtocol::Nmea2000).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }

    /// Frames come out restamped with the current time, the capture starts
    /// over at its end, and frames sent go to the `sent` file, not back.
    #[test]
    fn plays_in_a_loop_restamped_and_logs_what_is_sent() {
        let path = capture("loop", LINES);
        let sent = path.with_file_name("sent.raw");
        let _ = std::fs::remove_file(&sent);
        let handle = run(Config {
            sent: Some(sent.clone()),
            ..Config::new(&path)
        })
        .unwrap();

        let pgns: Vec<u32> = (0..5)
            .map(|_| handle.frames_rx.recv().unwrap().pgn)
            .collect();
        assert_eq!(pgns, [65280, 127250, 65280, 127250, 65280]);
        let frame = handle.frames_rx.recv().unwrap();
        let ts = frame.timestamp.expect("restamped");
        assert!(ts.starts_with(&now_iso()[..10]), "{ts} is not today");

        let out = RawFrame::new(None, 2, 127508, 0, 255, [1u8, 2, 3, 4, 5, 6, 7, 8]);
        handle.send_frame(out).unwrap();
        let stamped = RawFrame::new(
            Some("2026-10-03T12:00:00.000Z".into()),
            2,
            127508,
            0,
            255,
            [1u8; 8],
        );
        handle.send_frame(stamped).unwrap();
        assert_eq!(handle.close(), super::super::Closed::Confirmed);
        let written = std::fs::read_to_string(&sent).unwrap();
        assert!(written.starts_with("# format=FAST\r\n"), "{written:?}");
        let today = &now_iso()[..10];
        assert!(
            written.contains(&format!("\r\n{today}"))
                && written.contains(",2,127508,0,255,8,01,02,03,04,05,06,07,08"),
            "an unstamped frame is stamped now: {written:?}"
        );
        assert!(
            written.contains("2026-10-03T12:00:00.000Z,2,127508,0,255,8,01,01"),
            "a stamped frame keeps its time: {written:?}"
        );
    }
}
