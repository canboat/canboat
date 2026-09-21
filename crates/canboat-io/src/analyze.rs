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

#[cfg(test)]
mod tests {
    use super::*;

    /// Four PLAIN lines from three sources, two PGNs, so every filter
    /// has something to keep and something to drop.
    const MIXED: &str = "\
2026-05-29T19:16:04.826Z,2,127251,14,255,8,ff,5e,7d,00,00,ff,ff,ff
2026-05-29T19:16:04.926Z,2,127250,35,255,8,ff,5e,7d,00,00,ff,ff,ff
2026-05-29T19:16:05.026Z,2,127251,35,255,8,ff,60,7d,00,00,ff,ff,ff
2026-05-29T19:16:05.126Z,3,127245,14,12,8,ff,f8,ff,7f,ff,7f,ff,ff
";

    /// Collect `(pgn, src, dst)` for everything the pipeline emits.
    fn run(input: &str, cfg: &Config<'_>) -> Vec<(u32, u8, u8)> {
        let mut out = Vec::new();
        decode_stream(input.as_bytes(), cfg, |d| out.push((d.pgn, d.src, d.dst)))
            .expect("stream decodes");
        out
    }

    #[test]
    fn no_filters_decodes_every_line() {
        let got = run(MIXED, &Config::default());
        assert_eq!(
            got,
            vec![
                (127251, 14, 255),
                (127250, 35, 255),
                (127251, 35, 255),
                (127245, 14, 12),
            ]
        );
    }

    #[test]
    fn pgn_filter_keeps_only_that_pgn() {
        let cfg = Config {
            pgn_filter: Some(127251),
            ..Default::default()
        };
        let got = run(MIXED, &cfg);
        assert_eq!(got, vec![(127251, 14, 255), (127251, 35, 255)]);
    }

    #[test]
    fn src_filter_keeps_only_that_source() {
        let cfg = Config {
            src_filter: Some(35),
            ..Default::default()
        };
        let got = run(MIXED, &cfg);
        assert_eq!(got, vec![(127250, 35, 255), (127251, 35, 255)]);
    }

    #[test]
    fn dst_filter_keeps_only_that_destination() {
        let cfg = Config {
            dst_filter: Some(12),
            ..Default::default()
        };
        let got = run(MIXED, &cfg);
        assert_eq!(got, vec![(127245, 14, 12)]);
    }

    /// The filters are independent predicates, so combining them
    /// intersects rather than replacing one another.
    #[test]
    fn filters_combine() {
        let cfg = Config {
            pgn_filter: Some(127251),
            src_filter: Some(35),
            ..Default::default()
        };
        let got = run(MIXED, &cfg);
        assert_eq!(got, vec![(127251, 35, 255)]);
    }

    /// A filter that matches nothing yields no records rather than
    /// falling back to pass-through.
    #[test]
    fn filter_matching_nothing_yields_nothing() {
        let cfg = Config {
            src_filter: Some(200),
            ..Default::default()
        };
        assert!(run(MIXED, &cfg).is_empty());
    }

    /// Render a frame as a PLAIN line, so a synthetic record built by
    /// the library can be fed back through the text pipeline.
    fn plain_line(f: &canboat_core::RawFrame) -> String {
        let hex: Vec<String> = f.data.iter().map(|b| format!("{b:02x}")).collect();
        format!(
            "{},{},{},{},{},{},{}\n",
            f.timestamp.as_deref().unwrap_or(""),
            f.prio,
            f.pgn,
            f.src,
            f.dst,
            f.data.len(),
            hex.join(",")
        )
    }

    /// `suppress_startup_record` drops the producer's CANBOAT_BEM
    /// record so a `--fixtime` run doesn't leak the build version.
    #[test]
    fn startup_record_is_suppressed_on_request() {
        let rec = canboat_core::startup::startup_record_at(
            "8.1.0",
            "canboat-test",
            "/dev/null",
            "2026-05-29T19:16:04.800Z".to_string(),
        );
        let input = format!("{}{MIXED}", plain_line(&rec));

        let kept = run(&input, &Config::default());
        assert_eq!(
            kept.iter().filter(|(p, ..)| *p == CANBOAT_BEM).count(),
            1,
            "kept by default"
        );

        let cfg = Config {
            suppress_startup_record: true,
            ..Default::default()
        };
        let dropped = run(&input, &cfg);
        assert!(
            !dropped.iter().any(|(p, ..)| *p == CANBOAT_BEM),
            "suppressed on request"
        );
        assert_eq!(dropped.len(), kept.len() - 1, "only that record goes");
    }

    /// candump's pretty shape carries no time, so the pipeline stamps
    /// one. `fixed_time` makes that deterministic.
    #[test]
    fn timeless_input_gets_the_fixed_stamp() {
        let cfg = Config {
            fixed_time: Some("2026-01-01T00:00:00.000Z"),
            ..Default::default()
        };
        let mut stamps = Vec::new();
        decode_stream(
            b"  can0  09F80115   [8]  FF 5E 7D 00 00 FF FF FF\n".as_slice(),
            &cfg,
            |d| stamps.push(d.timestamp.clone()),
        )
        .expect("stream decodes");
        assert_eq!(stamps, vec![Some("2026-01-01T00:00:00.000Z".to_string())]);
    }

    /// A frame that arrives with its own timestamp keeps it — the
    /// fixed stamp is a fallback, not an override.
    #[test]
    fn fixed_stamp_does_not_override_a_real_one() {
        let cfg = Config {
            fixed_time: Some("2026-01-01T00:00:00.000Z"),
            ..Default::default()
        };
        let mut stamps = Vec::new();
        decode_stream(MIXED.as_bytes(), &cfg, |d| stamps.push(d.timestamp.clone())).expect("ok");
        assert_eq!(stamps[0].as_deref(), Some("2026-05-29T19:16:04.826Z"));
    }

    /// Without a fixed stamp the host clock fills in, in the analyzer's
    /// ISO shape.
    #[test]
    fn timeless_input_falls_back_to_the_host_clock() {
        let mut stamps = Vec::new();
        decode_stream(
            b"  can0  09F80115   [8]  FF 5E 7D 00 00 FF FF FF\n".as_slice(),
            &Config::default(),
            |d| stamps.push(d.timestamp.clone()),
        )
        .expect("stream decodes");
        let ts = stamps[0].clone().expect("stamped");
        assert_eq!(ts.len(), 24, "YYYY-MM-DDTHH:MM:SS.mmmZ, got {ts}");
        assert!(ts.ends_with('Z'));
    }

    /// Single frames of a fast-packet PGN reassemble into one record
    /// rather than decoding per fragment.
    #[test]
    fn fast_packet_fragments_reassemble_into_one_record() {
        // PGN 129029 (GNSS Position Data), 43 bytes over 7 frames.
        let input = "\
2026-05-29T19:16:04.000Z,3,129029,14,255,8,40,2b,f9,5b,f4,4c,90,42
2026-05-29T19:16:04.010Z,3,129029,14,255,8,41,32,00,00,00,00,00,00
2026-05-29T19:16:04.020Z,3,129029,14,255,8,42,00,00,00,00,00,00,00
2026-05-29T19:16:04.030Z,3,129029,14,255,8,43,00,00,00,00,00,00,00
2026-05-29T19:16:04.040Z,3,129029,14,255,8,44,00,00,00,00,00,00,00
2026-05-29T19:16:04.050Z,3,129029,14,255,8,45,00,00,00,00,00,00,00
2026-05-29T19:16:04.060Z,3,129029,14,255,8,46,00,00,ff,ff,ff,ff,ff
";
        let mut lens = Vec::new();
        decode_stream(input.as_bytes(), &Config::default(), |d| {
            lens.push(d.data.len())
        })
        .expect("stream decodes");
        assert_eq!(lens.len(), 1, "seven fragments make one record");
        assert_eq!(lens[0], 43, "the whole fast-packet payload");
    }

    /// An incomplete fast-packet sequence emits nothing: the
    /// reassembler holds the partial message rather than decoding a
    /// fragment as if it were whole.
    #[test]
    fn truncated_fast_packet_emits_nothing() {
        let input = "\
2026-05-29T19:16:04.000Z,3,129029,14,255,8,40,2b,f9,5b,f4,4c,90,42
2026-05-29T19:16:04.010Z,3,129029,14,255,8,41,32,00,00,00,00,00,00
";
        assert!(run(input, &Config::default()).is_empty());
    }

    /// Once a line carries more than 8 payload bytes the stream is
    /// treated as pre-coalesced, so the payload decodes as one record
    /// without going through the reassembler.
    #[test]
    fn oversized_line_switches_to_coalesced_mode() {
        let input = "2026-05-29T19:16:04.000Z,3,129029,14,255,43,2b,f9,5b,f4,4c,90,42,32,\
00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,\
00,00,00,ff,ff\n";
        let mut lens = Vec::new();
        decode_stream(input.as_bytes(), &Config::default(), |d| {
            lens.push(d.data.len())
        })
        .expect("stream decodes");
        assert_eq!(lens, vec![43]);
    }

    /// `forced_format` skips auto-detection. Forcing the wrong format
    /// on PLAIN input yields no records — proof the flag is honoured
    /// rather than quietly re-detected.
    #[test]
    fn forced_format_overrides_detection() {
        let cfg = Config {
            forced_format: Some(InputFormat::Ydwg02),
            ..Default::default()
        };
        assert!(
            run(MIXED, &cfg).is_empty(),
            "PLAIN lines are not valid YDWG-02"
        );
    }

    /// The J1939 flag selects the other schema table, so the same
    /// frame decodes under a different description.
    #[test]
    fn j1939_flag_selects_the_other_table() {
        let line = "2026-05-29T19:16:04.826Z,3,127251,14,255,8,ff,5e,7d,00,00,ff,ff,ff\n";
        let mut n2k = None;
        decode_stream(line.as_bytes(), &Config::default(), |d| {
            n2k = Some(d.description)
        })
        .expect("ok");

        let cfg = Config {
            j1939: true,
            ..Default::default()
        };
        let mut j1939 = None;
        decode_stream(line.as_bytes(), &cfg, |d| j1939 = Some(d.description)).expect("ok");

        assert_eq!(n2k, Some("Rate of Turn"));
        // Without this, the comparison below would also pass if the J1939
        // table stopped decoding the frame at all.
        assert!(j1939.is_some(), "the J1939 table must decode the frame");
        assert_ne!(n2k, j1939, "the J1939 table is a different schema");
    }

    /// A malformed line is logged and skipped; the frames around it
    /// still decode.
    #[test]
    fn a_bad_line_does_not_stop_the_stream() {
        let input = "\
2026-05-29T19:16:04.826Z,2,127251,14,255,8,ff,5e,7d,00,00,ff,ff,ff
this is not a canboat line at all
2026-05-29T19:16:05.026Z,2,127251,35,255,8,ff,60,7d,00,00,ff,ff,ff
";
        let got = run(input, &Config::default());
        assert_eq!(got, vec![(127251, 14, 255), (127251, 35, 255)]);
    }

    /// An empty stream is not an error.
    #[test]
    fn empty_input_is_clean() {
        assert!(run("", &Config::default()).is_empty());
    }

    /// `decode_file` reports the offending path, preserving the kind so
    /// callers can still match on `NotFound`.
    #[test]
    fn missing_file_error_names_the_path() {
        let err = decode_file(
            Path::new("/nonexistent/nope.log"),
            &Config::default(),
            |_| {},
        )
        .expect_err("must fail");
        assert_eq!(err.kind(), io::ErrorKind::NotFound);
        assert!(err.to_string().contains("nope.log"), "got {err}");
    }
}
