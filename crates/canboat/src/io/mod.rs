// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Sync `std::io` adapters for the [`engine`](crate::engine).
//!
//! These types are thin wrappers around `BufRead` / `Read` /
//! `serialport`. They exist so the CLI can read from stdin, files, TCP
//! sockets, or serial ports without each command re-implementing
//! buffering and EOF handling.
//!
//! No tokio. No async. The sans-I/O decoder in the engine is driven
//! directly from these synchronous reads; only the [`device`] runners
//! spawn threads.
//!
//! Feature gating: the `io` feature enables all of this; the `node`
//! feature alone compiles just the transport-free ISO 11783-5 node
//! pieces ([`address_claim`], [`name`], [`nmea_responder`]).

pub mod address_claim;
#[cfg(feature = "io")]
pub mod analyze;
#[cfg(feature = "io")]
pub mod container;
#[cfg(feature = "io")]
pub mod device;
#[cfg(feature = "io")]
pub mod fastpacket;
pub mod name;
pub mod nmea_responder;
#[cfg(feature = "io")]
pub mod stream;

#[cfg(feature = "io")]
pub use stream::{
    EblReader, EblWriter, FrameReader, FrameWriter, LineFrameReader, PlainWriter, TextLineWriter,
    copy,
};

#[cfg(feature = "io")]
use std::io::{self, BufRead, Read, Write};
#[cfg(feature = "io")]
use std::time::Duration;

/// Read lines one at a time from a [`BufRead`] source.
///
/// Mirrors what the analyzer binary needs from stdin (or a `.in` test
/// file): a line-by-line iterator that hands out borrowed `&str`s and
/// reuses its internal buffer so per-line allocation stays out of the
/// hot path.
///
/// Lines are returned with any trailing `\r` / `\n` stripped.
///
/// ```text
/// let stdin = std::io::stdin();
/// let mut lr = LineReader::new(stdin.lock());
/// while let Some(line) = lr.next_line().expect("read") {
///     println!("got: {}", line);
/// }
/// ```
#[cfg(feature = "io")]
pub struct LineReader<R: BufRead> {
    reader: R,
    buf: String,
}

#[cfg(feature = "io")]
impl<R: BufRead> LineReader<R> {
    pub fn new(reader: R) -> Self {
        Self {
            reader,
            buf: String::with_capacity(1024),
        }
    }

    /// Read the next line. Returns `Ok(None)` on EOF.
    pub fn next_line(&mut self) -> io::Result<Option<&str>> {
        self.buf.clear();
        let n = self.reader.read_line(&mut self.buf)?;
        if n == 0 {
            return Ok(None);
        }
        // Strip trailing newline / CRLF.
        let trimmed = self.buf.trim_end_matches(['\r', '\n']).len();
        self.buf.truncate(trimmed);
        Ok(Some(&self.buf))
    }
}

/// Open a serial port with canboat-typical defaults (8N1, no flow
/// control, a small read timeout so callers can poll without blocking
/// forever).
///
/// `baud` is in bits per second. The Actisense NGT-1 uses 115200;
/// iKonvert uses 230400.
#[cfg(feature = "io")]
pub fn open_serial(path: &str, baud: u32) -> io::Result<Box<dyn serialport::SerialPort>> {
    serialport::new(path, baud)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(250))
        .open()
        .map_err(|e| io::Error::other(e.to_string()))
}

/// Open a serial port and return an independent `(reader, writer)`
/// pair as `Read`/`Write` trait objects. Mirrors what the
/// `actisense-serial` / `ikonvert-serial` binaries do: open the port
/// once, then `try_clone` the handle so the read and write threads
/// each own an fd-sharing handle without a mutex. Hides
/// `serialport::SerialPort` from callers that just want byte streams.
#[cfg(feature = "io")]
pub fn open_serial_rw(
    path: &str,
    baud: u32,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    let read_port = open_serial(path, baud)?;
    let write_port = read_port
        .try_clone()
        .map_err(|e| io::Error::other(format!("cloning serial handle for {path}: {e}")))?;
    Ok((
        Box::new(SerialIo(read_port)),
        Box::new(SerialIo(write_port)),
    ))
}

/// Internal wrapper that adapts `Box<dyn SerialPort>` to a concrete
/// type implementing both `Read` and `Write`.
#[cfg(feature = "io")]
struct SerialIo(Box<dyn serialport::SerialPort>);

#[cfg(feature = "io")]
impl Read for SerialIo {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.read(buf)
    }
}

#[cfg(feature = "io")]
impl Write for SerialIo {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

#[cfg(all(test, feature = "io"))]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn line_reader_handles_crlf_and_eof() {
        let data = b"first\r\nsecond\nthird".to_vec();
        let mut lr = LineReader::new(Cursor::new(data));
        assert_eq!(lr.next_line().unwrap(), Some("first"));
        assert_eq!(lr.next_line().unwrap(), Some("second"));
        assert_eq!(lr.next_line().unwrap(), Some("third"));
        assert_eq!(lr.next_line().unwrap(), None);
    }
}
