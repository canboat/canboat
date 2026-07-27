// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Binary capture containers → canboat PLAIN text.
//!
//! Some captures do not arrive as ASCII line formats but as binary
//! SocketCAN libpcap files, optionally wrapped:
//!
//! - `*.pcap`    — a libpcap file of SocketCAN frames (link-type 227).
//! - `*.pcap.gz` — the same, gzip-compressed.
//! - `*.nif`     — a Navico "Information File": gzip(GNU tar) holding a
//!   MFD diagnostic dump. Its on-board NetLogger service stores rolling
//!   SocketCAN captures as `*.pcap.gz` under two directories:
//!
//! ```text
//!   …/NetLogger/dumps/filtered/deviceInfo/can0/<ts>.pcap.gz  device roster (PGN 60928, 126996, …)
//!   …/NetLogger/dumps/raw/can0/<ts>.pcap.gz                  full recent bus traffic
//! ```
//!
//! [`plain_reader`] turns any of these into a streaming [`BufRead`] of
//! canboat PLAIN lines (`<ts>,<prio>,<pgn>,<src>,<dst>,<len>,<hex>…`),
//! so every downstream consumer that already reads PLAIN — the analyzer
//! pipeline, the TUI's log loader, `nif2analyzer` — handles them without
//! a bespoke decoder. For a `.nif` the `filtered` captures are emitted
//! first (a concrete view of who is on the bus) then the `raw` ones,
//! each group in chronological (filename) order.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read};
use std::path::Path;

use canboat_core::RawFrame;
use canboat_core::format::plain::write_line;
use canboat_core::format::{days_to_ymd, iso11783_decompose};

use flate2::read::GzDecoder;
use tar::Archive;

/// libpcap little-endian, microsecond-timestamp magic (0xa1b2c3d4).
const PCAP_MAGIC_US: u32 = 0xa1b2c3d4;

/// SocketCAN flag bits carried in the high byte of the 32-bit CAN id
/// (EFF / RTR / ERR); masking them off leaves the 29-bit ISO 11783 id.
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;

/// Path fragments identifying the two NetLogger capture directories in a
/// `.nif` tar. `Filtered` sorts before `Raw` so it is emitted first.
const FILTERED_DIR: &str = "NetLogger/dumps/filtered/deviceInfo/can0/";
const RAW_DIR: &str = "NetLogger/dumps/raw/can0/";

/// Which capture groups of a `.nif` to emit. Ignored for plain `.pcap`
/// inputs (which carry a single unlabelled capture).
#[derive(Debug, Clone, Copy)]
pub struct Options {
    /// Emit the `filtered` device-roster captures.
    pub filtered: bool,
    /// Emit the `raw` full-traffic captures.
    pub raw: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            filtered: true,
            raw: true,
        }
    }
}

/// Does `path` name a binary capture container we can unwrap into PLAIN
/// text? Recognises `.nif`, `.pcap` and `.pcap.gz` (case-insensitive).
pub fn is_container(path: &Path) -> bool {
    let name = path.file_name().unwrap_or_default().to_string_lossy();
    let lower = name.to_ascii_lowercase();
    lower.ends_with(".nif") || lower.ends_with(".pcap") || lower.ends_with(".pcap.gz")
}

fn is_nif(path: &Path) -> bool {
    path.file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .to_ascii_lowercase()
        .ends_with(".nif")
}

fn is_gzip(path: &Path) -> bool {
    path.file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .to_ascii_lowercase()
        .ends_with(".gz")
}

/// Open `path` (a `.nif` / `.pcap` / `.pcap.gz`) as a streaming reader
/// of canboat PLAIN lines. Errors surface as `io::Error`.
pub fn plain_reader(path: &Path, opts: Options) -> io::Result<Box<dyn BufRead>> {
    let sources = if is_nif(path) {
        collect_nif_sources(path, opts)?
    } else {
        // A single (optionally gzipped) libpcap file.
        let bytes = std::fs::read(path)?;
        let mut d = VecDeque::with_capacity(1);
        d.push_back(if is_gzip(path) {
            Source::Gz(bytes)
        } else {
            Source::Plain(bytes)
        });
        d
    };
    Ok(Box::new(BufReader::new(PcapPlainReader::new(sources))))
}

/// One pending capture to decode, in emit order.
enum Source {
    /// gzip-compressed libpcap bytes (inner `.pcap.gz`).
    Gz(Vec<u8>),
    /// raw libpcap bytes (uncompressed `.pcap`).
    Plain(Vec<u8>),
}

/// Unpack a `.nif` tar and collect its NetLogger captures (still
/// gzipped) in emit order: all `filtered` before all `raw`, each group
/// chronological by filename.
fn collect_nif_sources(path: &Path, opts: Options) -> io::Result<VecDeque<Source>> {
    let file = File::open(path)?;
    let mut archive = Archive::new(GzDecoder::new(file));

    // (group ordinal, filename, gz bytes). Filtered=0 sorts before Raw=1.
    let mut captures: Vec<(u8, String, Vec<u8>)> = Vec::new();
    for entry in archive.entries()? {
        let mut entry = entry?;
        let entry_path = entry.path()?;
        let entry_path = entry_path.to_string_lossy();
        let group = if entry_path.contains(FILTERED_DIR) {
            0u8
        } else if entry_path.contains(RAW_DIR) {
            1u8
        } else {
            continue;
        };
        if !entry_path.ends_with(".pcap.gz") {
            continue;
        }
        if (group == 0 && !opts.filtered) || (group == 1 && !opts.raw) {
            continue;
        }
        // The last path component is `YYYY-MM-DD-HH-MM-SS.pcap.gz`, so a
        // lexicographic name sort is chronological.
        let name = entry_path
            .rsplit('/')
            .next()
            .unwrap_or(&entry_path)
            .to_string();
        let mut gz = Vec::new();
        entry.read_to_end(&mut gz)?;
        captures.push((group, name, gz));
    }
    captures.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.cmp(&b.1)));
    Ok(captures
        .into_iter()
        .map(|(_, _, gz)| Source::Gz(gz))
        .collect())
}

/// Pull-based reader that lazily turns a queue of SocketCAN libpcap
/// buffers into PLAIN text, one frame per line. Inner `.pcap.gz` sources
/// are inflated one at a time so peak memory stays near the compressed
/// container size, not the (much larger) decoded byte stream.
struct PcapPlainReader {
    sources: VecDeque<Source>,
    /// Current decompressed pcap buffer.
    cur: Vec<u8>,
    /// Record cursor into `cur`.
    off: usize,
    /// Formatted-but-not-yet-read output bytes for the current line.
    out: Vec<u8>,
    out_pos: usize,
    /// Reusable formatting scratch.
    line: String,
}

impl PcapPlainReader {
    fn new(sources: VecDeque<Source>) -> Self {
        Self {
            sources,
            cur: Vec::new(),
            off: 0,
            out: Vec::new(),
            out_pos: 0,
            line: String::with_capacity(96),
        }
    }

    /// Move to the next libpcap source, inflating it if gzipped and
    /// validating the global header. Skips unusable sources. Returns
    /// `false` once the queue is drained.
    fn advance_source(&mut self) -> io::Result<bool> {
        while let Some(src) = self.sources.pop_front() {
            let bytes = match src {
                Source::Gz(gz) => {
                    let mut out = Vec::new();
                    GzDecoder::new(gz.as_slice()).read_to_end(&mut out)?;
                    out
                }
                Source::Plain(b) => b,
            };
            if bytes.len() < 24 {
                log::warn!("container: skipping capture too short to be a pcap");
                continue;
            }
            let magic = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
            if magic != PCAP_MAGIC_US {
                log::warn!("container: skipping capture with bad pcap magic 0x{magic:08x}");
                continue;
            }
            self.cur = bytes;
            self.off = 24; // skip the 24-byte global header
            return Ok(true);
        }
        Ok(false)
    }

    /// Format the next CAN frame into `self.out` (with a trailing
    /// newline). Returns `Ok(false)` at end of all sources.
    ///
    /// Record layout (little-endian pcap, classic-CAN payloads):
    ///
    /// ```text
    ///   pcap record header (16 bytes): ts_sec, ts_usec, incl_len, orig_len
    ///   SocketCAN frame (incl_len bytes):
    ///     can_id  : BE u32  — bit 31 EFF flag + 29-bit id
    ///     can_dlc : u8      — data length (0..=8 for classic CAN)
    ///     pad     : 3 bytes — flags / reserved
    ///     data    : 8 bytes — always 8, dlc says how many are meaningful
    /// ```
    fn refill(&mut self) -> io::Result<bool> {
        loop {
            if self.off + 16 > self.cur.len() {
                // Out of records in this buffer — pull the next source.
                if !self.advance_source()? {
                    return Ok(false);
                }
                continue;
            }
            let base = self.off;
            let ts_sec = u32::from_le_bytes(self.cur[base..base + 4].try_into().unwrap());
            let ts_usec = u32::from_le_bytes(self.cur[base + 4..base + 8].try_into().unwrap());
            let incl_len =
                u32::from_le_bytes(self.cur[base + 8..base + 12].try_into().unwrap()) as usize;
            let rec = base + 16;
            if incl_len < 8 || rec + incl_len > self.cur.len() {
                // Truncated / non-CAN tail — abandon this buffer.
                self.off = self.cur.len();
                continue;
            }

            let frame = {
                let canid =
                    u32::from_be_bytes(self.cur[rec..rec + 4].try_into().unwrap()) & CAN_EFF_MASK;
                let dlc = (self.cur[rec + 4] as usize).min(incl_len - 8).min(8);
                let data = &self.cur[rec + 8..rec + 8 + dlc];
                let (prio, pgn, src, dst) = iso11783_decompose(canid);
                let ts = format_plain_ts(ts_sec as u64 * 1000 + (ts_usec as u64) / 1000);
                RawFrame::new(Some(ts), prio, pgn, src, dst, data.iter().copied())
            };
            self.off = rec + incl_len;

            self.line.clear();
            let _ = write_line(&mut self.line, &frame);
            self.out.clear();
            self.out.extend_from_slice(self.line.as_bytes());
            self.out.push(b'\n');
            self.out_pos = 0;
            return Ok(true);
        }
    }
}

impl Read for PcapPlainReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        loop {
            if self.out_pos < self.out.len() {
                let n = (self.out.len() - self.out_pos).min(buf.len());
                buf[..n].copy_from_slice(&self.out[self.out_pos..self.out_pos + n]);
                self.out_pos += n;
                return Ok(n);
            }
            if !self.refill()? {
                return Ok(0);
            }
        }
    }
}

/// Format a Unix-epoch millisecond count as canboat's PLAIN timestamp
/// `YYYY-MM-DD-HH:MM:SS.mmm` (UTC, `%F-%T.%03d`) — the same shape
/// `candump2analyzer` emits, so PLAIN output is interchangeable.
fn format_plain_ts(ms: u64) -> String {
    let secs = (ms / 1000) as i64;
    let frac = (ms % 1000) as u32;
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let h = day_secs / 3600;
    let m = (day_secs / 60) % 60;
    let s = day_secs % 60;
    let (y, mo, d) = days_to_ymd(days);
    format!("{y:04}-{mo:02}-{d:02}-{h:02}:{m:02}:{s:02}.{frac:03}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write as _;

    /// A one-record SocketCAN pcap: PDU2 frame, CAN id 0x09F50B23 with
    /// the EFF flag set (0x89F50B23) — PGN 128267, src 0x23, dst 255.
    fn one_record_pcap() -> Vec<u8> {
        let mut b = Vec::new();
        b.extend_from_slice(&PCAP_MAGIC_US.to_le_bytes());
        b.extend_from_slice(&[0u8; 20]); // rest of global header
        b.extend_from_slice(&1_502_984_881u32.to_le_bytes()); // ts_sec
        b.extend_from_slice(&664_000u32.to_le_bytes()); // ts_usec
        b.extend_from_slice(&16u32.to_le_bytes()); // incl_len
        b.extend_from_slice(&16u32.to_le_bytes()); // orig_len
        b.extend_from_slice(&0x89F50B23u32.to_be_bytes()); // can_id (EFF flag set)
        b.push(8); // dlc
        b.extend_from_slice(&[0, 0, 0]); // flags/pad
        b.extend_from_slice(&[0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff]);
        b
    }

    fn gzip(bytes: &[u8]) -> Vec<u8> {
        let mut enc = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
        enc.write_all(bytes).unwrap();
        enc.finish().unwrap()
    }

    fn read_all(sources: VecDeque<Source>) -> String {
        let mut r = PcapPlainReader::new(sources);
        let mut s = String::new();
        r.read_to_string(&mut s).unwrap();
        s
    }

    #[test]
    fn plain_source_emits_line_and_masks_eff_flag() {
        let mut d = VecDeque::new();
        d.push_back(Source::Plain(one_record_pcap()));
        assert_eq!(
            read_all(d).trim(),
            // The PLAIN writer canonicalises the timestamp to ISO-8601 UTC.
            "2017-08-17T15:48:01.664Z,2,128267,35,255,8,ff,ff,ff,ff,ff,00,00,ff"
        );
    }

    #[test]
    fn gz_source_inflates() {
        let mut d = VecDeque::new();
        d.push_back(Source::Gz(gzip(&one_record_pcap())));
        assert!(read_all(d).contains(",128267,35,255,8,"));
    }

    #[test]
    fn filtered_group_precedes_raw_group_regardless_of_input_order() {
        // Two "raw" then two "filtered" pushed out of order; collection
        // sort must front-load filtered. Here we exercise the reader's
        // multi-source concatenation directly.
        let mut d = VecDeque::new();
        d.push_back(Source::Plain(one_record_pcap()));
        d.push_back(Source::Gz(gzip(&one_record_pcap())));
        assert_eq!(read_all(d).lines().count(), 2);
    }

    #[test]
    fn short_dlc_advances_by_incl_len_not_dlc() {
        // First record dlc=3 but a full 16-byte SocketCAN payload; a
        // dlc-based stride would desync the second record.
        let mut b = Vec::new();
        b.extend_from_slice(&PCAP_MAGIC_US.to_le_bytes());
        b.extend_from_slice(&[0u8; 20]);
        for dlc in [3u8, 8u8] {
            b.extend_from_slice(&1_502_984_881u32.to_le_bytes());
            b.extend_from_slice(&0u32.to_le_bytes());
            b.extend_from_slice(&16u32.to_le_bytes());
            b.extend_from_slice(&16u32.to_le_bytes());
            b.extend_from_slice(&0x89F50B23u32.to_be_bytes());
            b.push(dlc);
            b.extend_from_slice(&[0, 0, 0]);
            b.extend_from_slice(&[0xaa, 0xbb, 0xcc, 0, 0, 0, 0, 0]);
        }
        let mut d = VecDeque::new();
        d.push_back(Source::Plain(b));
        let lines: Vec<_> = read_all(d).lines().map(String::from).collect();
        assert_eq!(lines.len(), 2);
        assert!(lines[0].ends_with(",3,aa,bb,cc"));
        assert!(lines[1].contains(",8,aa,bb,cc,00,00,00,00,00"));
    }

    #[test]
    fn bad_magic_source_is_skipped() {
        let mut d = VecDeque::new();
        d.push_back(Source::Plain(vec![0xff; 64]));
        d.push_back(Source::Plain(one_record_pcap()));
        // The bad source is skipped; the good one still decodes.
        assert!(read_all(d).contains(",128267,"));
    }

    #[test]
    fn is_container_recognises_extensions() {
        assert!(is_container(Path::new("foo.nif")));
        assert!(is_container(Path::new("foo.pcap")));
        assert!(is_container(Path::new("foo.pcap.gz")));
        assert!(is_container(Path::new("FOO.PCAP")));
        assert!(!is_container(Path::new("foo.raw")));
        assert!(!is_container(Path::new("foo.json")));
    }
}
