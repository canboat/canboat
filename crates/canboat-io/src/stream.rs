// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `RawFrame` streams: a sync, pull-based [`FrameReader`] /
//! [`FrameWriter`] pair over the canonical [`RawFrame`].
//!
//! Every canboat-rs tool that isn't a pure byte-shunt has the same
//! spine: pull [`RawFrame`]s from some source, optionally transform
//! them, push them to some sink. Historically each binary re-wired
//! that spine by hand — a line loop here, an `mpsc::Receiver` drain
//! there. These two traits name the spine so the `convert` and
//! `interface` subcommands can wire an arbitrary source to an
//! arbitrary sink without caring which concrete pair they hold.
//!
//! The traits are deliberately blocking: nothing in the workspace is
//! async (the device bridges get their concurrency from
//! `std::thread` + `std::sync::mpsc`), so a `read_frame` that blocks
//! until the next frame — or `Ok(None)` at end-of-stream — matches
//! every existing source, including a live CAN interface drained
//! through [`crate::device::DeviceHandle`].

use std::collections::VecDeque;
use std::io::{self, BufRead, Read, Write};

use canboat_core::RawFrame;
use canboat_core::format::actisense_ascii::write_line as write_actisense;
use canboat_core::format::ebl::encode_frame as encode_ebl;
use canboat_core::format::ngt1::{EblHeader, Ngt1Decoder, NgtEvent};
use canboat_core::format::plain::write_line as write_plain;
use canboat_core::format::ydwg02::write_line as write_ydwg02;
use canboat_core::format::{
    InputFormat, detect, header_implies_coalesced, parse_format_header, parse_with,
};

/// Pull-based source of [`RawFrame`]s — the canboat-core
/// [`FrameSource`](canboat_core::FrameSource) trait, re-exported here
/// under its historical name. Concrete readers in this crate
/// ([`LineFrameReader`], [`EblReader`]) implement it; an
/// [`mpsc::Receiver<RawFrame>`](std::sync::mpsc::Receiver) does too (the
/// impl lives in canboat-core).
pub use canboat_core::FrameSource as FrameReader;

/// Sink for [`RawFrame`]s.
pub trait FrameWriter {
    fn write_frame(&mut self, frame: &RawFrame) -> io::Result<()>;

    /// Flush any buffered output. The default is a no-op for sinks
    /// that don't buffer (e.g. a device channel).
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Drain every frame from `reader` into `writer`, then flush.
/// Returns the number of frames copied. This is the whole body of a
/// format→format `convert` once the endpoints are chosen.
pub fn copy<R: FrameReader + ?Sized, W: FrameWriter + ?Sized>(
    reader: &mut R,
    writer: &mut W,
) -> io::Result<u64> {
    let mut n = 0;
    while let Some(frame) = reader.read_frame()? {
        writer.write_frame(&frame)?;
        n += 1;
    }
    writer.flush()?;
    Ok(n)
}

/// A [`FrameReader`] over any canboat ASCII line format.
///
/// Owns the format-detection state machine that used to live inline
/// in the analyzer's replay loop: it honours `# format=<NAME>`
/// headers, auto-detects the format from the first content line when
/// none is forced, and parses each subsequent line via
/// [`parse_with`]. Blank lines, comment/header lines and
/// non-frame control sentences (iKonvert `$PDGY` status, Garmin CSV
/// headers) are swallowed internally so callers only ever see frames.
///
/// The wrapped source is any [`BufRead`], which includes a file, a
/// stdin lock, or the PLAIN [`BufRead`](crate::container::plain_reader)
/// that unwraps a `.nif` / `.pcap` capture container.
pub struct LineFrameReader<R: BufRead> {
    lines: crate::LineReader<R>,
    /// Forced format (`--format`) or, once detected, the active one.
    active: Option<InputFormat>,
    /// True once a `# format=<NAME>` header declared an
    /// already-coalesced format (FAST/ACTISENSE/…). Consumers that
    /// run reassembly downstream consult this to decide whether to
    /// bypass the reassembler.
    header_coalesced: bool,
}

impl<R: BufRead> LineFrameReader<R> {
    /// Wrap `reader`, auto-detecting the format from its first content
    /// line.
    pub fn new(reader: R) -> Self {
        Self {
            lines: crate::LineReader::new(reader),
            active: None,
            header_coalesced: false,
        }
    }

    /// Wrap `reader`, forcing `format` instead of auto-detecting
    /// (analyzer's `--format`).
    pub fn with_format(reader: R, format: InputFormat) -> Self {
        Self {
            lines: crate::LineReader::new(reader),
            active: Some(format),
            header_coalesced: false,
        }
    }

    /// The format currently in effect: the forced one, or whatever
    /// was detected once the first frame has been read.
    pub fn active_format(&self) -> Option<InputFormat> {
        self.active
    }

    /// True when a `# format=<NAME>` header has declared an
    /// already-coalesced format, so a downstream reassembler should
    /// be bypassed. Mirrors canboat's `header_implies_coalesced`.
    pub fn header_coalesced(&self) -> bool {
        self.header_coalesced
    }
}

impl<R: BufRead> FrameReader for LineFrameReader<R> {
    fn read_frame(&mut self) -> io::Result<Option<RawFrame>> {
        loop {
            let Some(line) = self.lines.next_line()? else {
                return Ok(None);
            };
            if line.is_empty() {
                continue;
            }
            // `# format=<NAME>` headers from the canboat reader
            // binaries: set the format if not already known and note
            // whether it implies coalesced frames.
            if line.starts_with('#') {
                if let Some(fmt) = parse_format_header(line) {
                    if self.active.is_none() {
                        self.active = Some(fmt);
                        log::info!("input format set by header: {fmt:?}");
                    }
                    if header_implies_coalesced(line) {
                        self.header_coalesced = true;
                        log::debug!("header declares coalesced format; skipping reassembly");
                    }
                }
                continue;
            }
            // Auto-detect on the first content line if not forced.
            // Canboat falls back to PLAIN when nothing else matches.
            if self.active.is_none() {
                self.active = detect(line).or(Some(InputFormat::Plain));
                log::debug!("input format: {:?}", self.active);
            }
            let format = self.active.expect("active format set above");
            match parse_with(format, line) {
                Ok(Some(frame)) => return Ok(Some(frame)),
                // iKonvert control sentences, Garmin CSV headers, etc.
                Ok(None) => continue,
                Err(canboat_core::format::PlainError::Empty) => continue,
                Err(e) => {
                    log::warn!("skipping malformed input line: {e}");
                    continue;
                }
            }
        }
    }
}

/// A [`FrameWriter`] that serialises each frame as a canboat
/// PLAIN/FAST line to an underlying [`Write`].
pub struct PlainWriter<W: Write> {
    inner: W,
    buf: String,
}

impl<W: Write> PlainWriter<W> {
    pub fn new(inner: W) -> Self {
        Self {
            inner,
            buf: String::with_capacity(256),
        }
    }

    /// Recover the wrapped writer.
    pub fn into_inner(self) -> W {
        self.inner
    }
}

impl<W: Write> FrameWriter for PlainWriter<W> {
    fn write_frame(&mut self, frame: &RawFrame) -> io::Result<()> {
        self.buf.clear();
        // Writing to a `String` is infallible; the only error path is
        // the real I/O below.
        write_plain(&mut self.buf, frame).expect("PLAIN write to String");
        self.buf.push('\n');
        self.inner.write_all(self.buf.as_bytes())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

/// A [`FrameWriter`] emitting one line per frame via a core text
/// encoder (`fn(&mut String, &RawFrame)`), used for the YDWG02 and
/// Actisense-ASCII output formats. Mirrors [`PlainWriter`].
pub struct TextLineWriter<W: Write> {
    inner: W,
    buf: String,
    encode: fn(&mut String, &RawFrame) -> std::fmt::Result,
}

impl<W: Write> TextLineWriter<W> {
    /// YDWG02 / YDEN received lines (`<time> R <canid> <hex…>`).
    pub fn ydwg02(inner: W) -> Self {
        Self::new(inner, write_ydwg02)
    }

    /// Actisense N2K-ASCII lines (`A<time> <sdp> <pgn> <hex>`).
    pub fn actisense(inner: W) -> Self {
        Self::new(inner, write_actisense)
    }

    fn new(inner: W, encode: fn(&mut String, &RawFrame) -> std::fmt::Result) -> Self {
        Self {
            inner,
            buf: String::with_capacity(256),
            encode,
        }
    }
}

impl<W: Write> FrameWriter for TextLineWriter<W> {
    fn write_frame(&mut self, frame: &RawFrame) -> io::Result<()> {
        self.buf.clear();
        (self.encode)(&mut self.buf, frame).expect("text write to String");
        self.buf.push('\n');
        self.inner.write_all(self.buf.as_bytes())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

/// A [`FrameWriter`] emitting each frame as an Actisense `.ebl` binary
/// record pair (timestamp header + `N2K_MSG_RECEIVED`). No line
/// terminator — the framing is self-delimiting.
pub struct EblWriter<W: Write> {
    inner: W,
    buf: Vec<u8>,
}

impl<W: Write> EblWriter<W> {
    pub fn new(inner: W) -> Self {
        Self {
            inner,
            buf: Vec::with_capacity(256),
        }
    }
}

impl<W: Write> FrameWriter for EblWriter<W> {
    fn write_frame(&mut self, frame: &RawFrame) -> io::Result<()> {
        self.buf.clear();
        encode_ebl(frame, &mut self.buf);
        self.inner.write_all(&self.buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

/// A [`FrameReader`] over an Actisense `.ebl` byte stream — the inverse
/// of [`EblWriter`]. Drives [`Ngt1Decoder::with_ebl`] (EBL framing is a
/// superset of NGT-1 framing) and hands back one [`RawFrame`] per
/// `N2K_MSG_RECEIVED` record. The per-record `ESC SOH` timestamp header
/// is folded onto the following frame as an ISO-8601 UTC timestamp.
///
/// Binary, not line-based, so it implements [`FrameReader`] directly
/// rather than going through [`LineFrameReader`].
pub struct EblReader<R: Read> {
    inner: R,
    decoder: Ngt1Decoder,
    queue: VecDeque<RawFrame>,
    /// Most recent `.ebl` header timestamp (Unix ms) to stamp onto the
    /// next message frame.
    pending_ts: Option<u64>,
    buf: Box<[u8; 8192]>,
    eof: bool,
}

impl<R: Read> EblReader<R> {
    pub fn new(inner: R) -> Self {
        Self {
            inner,
            decoder: Ngt1Decoder::with_ebl(),
            queue: VecDeque::new(),
            pending_ts: None,
            buf: Box::new([0u8; 8192]),
            eof: false,
        }
    }
}

impl<R: Read> FrameReader for EblReader<R> {
    fn read_frame(&mut self) -> io::Result<Option<RawFrame>> {
        loop {
            if let Some(frame) = self.queue.pop_front() {
                return Ok(Some(frame));
            }
            if self.eof {
                return Ok(None);
            }
            let n = self.inner.read(&mut self.buf[..])?;
            if n == 0 {
                self.eof = true;
                continue;
            }
            for ev in self.decoder.push_bytes(&self.buf[..n]) {
                match ev {
                    NgtEvent::Header(EblHeader::Timestamp(ms)) => self.pending_ts = Some(ms),
                    NgtEvent::Message(m) => {
                        if let Some(mut frame) = m.to_raw_frame() {
                            // The NGT message's own timestamp is 0 in an
                            // `.ebl`; the real instant rides in the header.
                            if let Some(ms) = self.pending_ts.take() {
                                frame.timestamp = Some(canboat_core::format_iso_ms(ms));
                            }
                            self.queue.push_back(frame);
                        }
                    }
                    // Skip an unknown header record or a framing error —
                    // resync on the next valid `ESC SOH` / `DLE STX`.
                    NgtEvent::Header(_) | NgtEvent::Error(_) => {}
                }
            }
        }
    }
}

/// The receiving end of a device's frame channel is a blocking
/// A device's [`FrameSender`](crate::device::FrameSender) is a
/// [`FrameWriter`]: each frame is queued to the device's writer
/// thread, which encodes it for the wire. A gone writer thread
/// (device closed) surfaces as a broken-pipe I/O error.
impl FrameWriter for crate::device::FrameSender {
    fn write_frame(&mut self, frame: &RawFrame) -> io::Result<()> {
        self.send_frame(frame.clone())
            .map_err(|e| io::Error::new(io::ErrorKind::BrokenPipe, e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn line_reader_yields_frames_and_skips_noise() {
        // A comment/header, a blank line, then two PLAIN frames.
        let input = "\
# format=PLAIN

2022-09-10T12:10:16.614Z,6,60928,5,255,8,fb,9b,70,22,00,9b,50,c0
2022-09-10T12:10:16.700Z,6,60928,6,255,8,01,02,03,04,05,06,07,08
";
        let mut r = LineFrameReader::new(Cursor::new(input));
        let f1 = r.read_frame().unwrap().expect("first frame");
        assert_eq!(f1.pgn, 60928);
        assert_eq!(f1.src, 5);
        let f2 = r.read_frame().unwrap().expect("second frame");
        assert_eq!(f2.src, 6);
        assert!(r.read_frame().unwrap().is_none());
        assert_eq!(r.active_format(), Some(InputFormat::Plain));
    }

    #[test]
    fn plain_round_trips_through_copy() {
        let input = "2022-09-10T12:10:16.614Z,6,60928,5,255,8,fb,9b,70,22,00,9b,50,c0\n";
        let mut reader = LineFrameReader::new(Cursor::new(input));
        let mut out: Vec<u8> = Vec::new();
        let n = copy(&mut reader, &mut PlainWriter::new(&mut out)).unwrap();
        assert_eq!(n, 1);
        // Re-parsing the emitted line yields an identical frame.
        let mut back = LineFrameReader::new(Cursor::new(out));
        let f = back.read_frame().unwrap().unwrap();
        assert_eq!(f.pgn, 60928);
        assert_eq!(f.src, 5);
        assert_eq!(f.dst, 255);
        assert_eq!(
            &f.data[..],
            &[0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0]
        );
    }

    #[test]
    fn header_coalesced_is_reported() {
        let input = "# format=FAST\n";
        let mut r = LineFrameReader::new(Cursor::new(input));
        assert!(r.read_frame().unwrap().is_none());
        assert!(r.header_coalesced());
    }
}
