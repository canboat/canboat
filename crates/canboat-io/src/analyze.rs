// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The analyzer decode pipeline: line/container input →
//! reassembly → schema decode → [`DecodedPgn`].
//!
//! This is the exact parse / reassemble / decode path the `analyzer`
//! binary drives, lifted into the library so any in-process consumer
//! (the `canboat` binary's `convert`, `canboat-tui`'s log-replay mode)
//! runs the same behaviour without shelling out to a subprocess.
//!
//! The front half — format auto-detection, `# format=<NAME>` headers,
//! per-line parsing — lives in [`LineFrameReader`](crate::LineFrameReader).
//! This module owns only the analyzer-specific back half: source-based
//! filtering, fast-packet / ISO-TP reassembly, and the schema decode.

use std::fs::File;
use std::io::{self, BufRead, BufReader};
use std::path::Path;

use canboat_core::format::InputFormat;
use canboat_core::{
    CANBOAT_BEM, DecodedPgn, FramePacketType, PacketType, PgnDatabase, Reassembled, Reassembler,
};

use crate::{FrameReader, LineFrameReader};

/// Per-call options for [`decode_stream`] / [`decode_file`].
#[derive(Debug, Default, Clone, Copy)]
pub struct Config<'a> {
    /// Force a specific input format instead of auto-detecting from
    /// the first content line. Equivalent to the `--format` flag on
    /// the analyzer binary.
    pub forced_format: Option<InputFormat>,
    /// When set, drop frames whose PGN doesn't match (analyzer's
    /// positional `[PGN]` filter).
    pub pgn_filter: Option<u32>,
    /// When set, drop frames whose src doesn't match (analyzer's
    /// `--src` filter).
    pub src_filter: Option<u8>,
    /// When set, drop frames whose dst doesn't match (analyzer's
    /// `--dst` filter).
    pub dst_filter: Option<u8>,
    /// Drop the producer's `CANBOAT_BEM` startup record (`PGN
    /// 0x40010`). Set this when running under `--fixtime` so the
    /// producer's build version doesn't leak into version-agnostic
    /// output.
    pub suppress_startup_record: bool,
    /// Unit system to decode into — `Si` (the default: rad/K/Pa,
    /// `--units si`) or `Metric` (deg/°C/bar, `--units metric`, which
    /// is what canboat C prints without `-si`).
    pub units: canboat_core::Units,
    /// Decode against the J1939 schema flavor instead of NMEA 2000 —
    /// the Rust counterpart of running `analyzer-j1939`. Table choice
    /// is exclusive (see `PgnDatabase::embedded_j1939`).
    pub j1939: bool,
    /// Stamp for frames whose input format carries no timestamp at all
    /// (e.g. candump's pretty shape). `None` stamps the wall clock —
    /// what `candump2analyzer` does; a fixed string keeps golden
    /// outputs deterministic (the analyzer's `--fixtime`). Frames that
    /// arrive with a timestamp keep it either way, matching canboat C.
    pub fixed_time: Option<&'a str>,
}

/// Open `path` and stream-decode it via [`decode_stream`]. Binary
/// capture containers (`.pcap`, `.pcap.gz`, `.nif`) are unwrapped into
/// PLAIN on the fly. Parse / reassembly / decode errors are logged via
/// the `log` crate and the stream continues; only a hard I/O error
/// stops the loop.
pub fn decode_file<F: FnMut(&DecodedPgn)>(
    path: &Path,
    cfg: &Config<'_>,
    sink: F,
) -> io::Result<()> {
    if crate::container::is_container(path) {
        let reader = crate::container::plain_reader(path, Default::default())
            .map_err(|e| open_error(path, e))?;
        return decode_stream(reader, cfg, sink);
    }
    let file = File::open(path).map_err(|e| open_error(path, e))?;
    decode_stream(BufReader::new(file), cfg, sink)
}

/// Wrap an open failure with the offending path, preserving the
/// original [`io::ErrorKind`].
fn open_error(path: &Path, e: io::Error) -> io::Error {
    io::Error::new(e.kind(), format!("opening {}: {e}", path.display()))
}

/// Wall-clock receive timestamp in the analyzer's ISO shape.
#[cfg(not(target_arch = "wasm32"))]
fn now_iso_timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let (secs, millis) = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| (d.as_secs() as i64, d.subsec_millis()))
        .unwrap_or((0, 0));
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let (y, mo, d) = canboat_core::format::days_to_ymd(days);
    let (h, m, s) = (day_secs / 3600, (day_secs / 60) % 60, day_secs % 60);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02}.{millis:03}Z")
}

/// wasm32-unknown-unknown has no host clock — `SystemTime::now()`
/// PANICS there rather than erroring. Wasm hosts stamp receive time
/// themselves; the epoch keeps the ISO shape without touching a clock.
#[cfg(target_arch = "wasm32")]
fn now_iso_timestamp() -> String {
    "1970-01-01T00:00:00.000Z".to_string()
}

/// Drive the analyzer pipeline over `source`. For each decoded
/// record, invoke `sink` with the resulting [`DecodedPgn`]. The
/// `DecodedPgn` borrows static schema data so callers can pass it
/// straight to `canboat_core::output::{write_json, write_text}`
/// without re-decoding.
///
/// The line parsing / format detection is delegated to
/// [`LineFrameReader`]; this function owns only the analyzer-specific
/// back half: filtering, fast-packet/TP reassembly, and schema decode.
pub fn decode_stream<R: BufRead, F: FnMut(&DecodedPgn)>(
    source: R,
    cfg: &Config<'_>,
    mut sink: F,
) -> io::Result<()> {
    let db = if cfg.j1939 {
        PgnDatabase::embedded_j1939(cfg.units)
    } else {
        PgnDatabase::embedded(cfg.units)
    };
    let mut reader = match cfg.forced_format {
        Some(fmt) => LineFrameReader::with_format(source, fmt),
        None => LineFrameReader::new(source),
    };
    // canboat's PLAIN_OR_FAST mode locks into "coalesced" once any
    // line carries more than 8 payload bytes — from then on every
    // frame is assumed to be pre-assembled and the reassembler is
    // skipped. Mirror the analyzer binary 1:1.
    let mut coalesced_mode = false;
    let mut warned_mislabeled = false;
    let mut reasm = Reassembler::new();

    while let Some(mut frame) = reader.read_frame()? {
        // A `# format=<NAME>` header (consumed inside `read_frame`)
        // may have declared an already-coalesced format.
        coalesced_mode |= reader.header_coalesced();

        // Formats without any time information (candump's pretty
        // shape) arrive with no timestamp; stamp receive time — the
        // same thing `candump2analyzer` does when it converts for the
        // C analyzer — or the caller's fixed string in test mode.
        if frame.timestamp.is_none() {
            frame.timestamp = Some(match cfg.fixed_time {
                Some(s) => s.to_string(),
                None => now_iso_timestamp(),
            });
        }

        // Defensive: a stream that declared itself coalesced
        // (`# format=FAST`) but carries an 8-byte frame of a fast-packet
        // PGN *that cannot fit in 8 bytes* is almost certainly single
        // frames mislabeled as coalesced. In coalesced mode we skip
        // reassembly, so we would decode only the first fragment and
        // silently drop most fields. Warn once so the producer's
        // mislabel is visible rather than corrupting output in silence.
        //
        // Only fire when *every* registered variant of the PGN needs
        // more than 8 bytes: many proprietary fast-packet PGNs (e.g. B&G
        // 130824, `MinLength` 2) legitimately fit a complete message in
        // ≤ 8 bytes, and one PGN number can carry several manufacturer
        // variants of differing lengths. If any variant could be
        // complete at ≤ 8 bytes, an 8-byte frame is plausibly whole.
        if coalesced_mode && !warned_mislabeled && frame.data.len() <= 8 {
            let mut variants = db.pgn_variants(frame.pgn).peekable();
            let all_fast_over_8 = variants.peek().is_some()
                && variants.all(|p| {
                    matches!(p.packet_type, PacketType::Fast)
                        // Smallest complete message: `min_length` when the
                        // PGN publishes one (a variable-length PGN always
                        // does; a fixed one does when a shorter payload is
                        // known to decode), else the fixed `length`. Unknown
                        // (both None) → assume it may be short, so this
                        // variant does not justify a warn.
                        && p.min_length.or(p.length).is_some_and(|n| n > 8)
                });
            if all_fast_over_8 {
                log::warn!(
                    "input declared a coalesced stream (# format=FAST) but carries an \
                     8-byte frame of fast-packet PGN {} (whose shortest complete message \
                     exceeds 8 bytes); this looks like single frames mislabeled as \
                     coalesced — fast-packet PGNs will decode incompletely. Re-emit the \
                     source with `# format=FRAMES` so the reassembler runs.",
                    frame.pgn
                );
                warned_mislabeled = true;
            }
        }

        if cfg.suppress_startup_record && frame.pgn == CANBOAT_BEM {
            continue;
        }
        if let Some(want) = cfg.pgn_filter
            && frame.pgn != want
        {
            continue;
        }
        if let Some(want) = cfg.src_filter
            && frame.src != want
        {
            continue;
        }
        if let Some(want) = cfg.dst_filter
            && frame.dst != want
        {
            continue;
        }

        // Once any line carries > 8 payload bytes, assume FAST is
        // pre-coalesced and skip reassembly for the remainder of the
        // stream. PlainMixFast intentionally interleaves so opt out.
        if frame.data.len() > 8 && reader.active_format() != Some(InputFormat::PlainMixFast) {
            coalesced_mode = true;
        }
        let packet_type = if coalesced_mode {
            FramePacketType::Other
        } else {
            db.first_pgn(frame.pgn)
                .or_else(|| db.fallback_pgn(frame.pgn))
                .map(|p| match p.packet_type {
                    PacketType::Fast => FramePacketType::Fast,
                    PacketType::Single => FramePacketType::Single,
                    _ => FramePacketType::Other,
                })
                .unwrap_or(FramePacketType::Other)
        };
        let assembled = match reasm.push(frame, packet_type) {
            Reassembled::PassThrough(f) | Reassembled::Complete(f) => f,
            Reassembled::Partial => continue,
            Reassembled::Error(e) => {
                log::warn!("reassembly error: {e}");
                continue;
            }
        };
        let decoded = match db.decode(&assembled) {
            Ok(d) => d,
            Err(e) => {
                log::warn!("decode error: {e}");
                continue;
            }
        };
        sink(&decoded);
    }
    Ok(())
}
