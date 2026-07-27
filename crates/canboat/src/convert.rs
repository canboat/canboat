// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat convert` — translate a capture between formats.
//!
//! Every supported input (the canboat ASCII line formats, plus the
//! binary `.pcap`/`.pcap.gz`/`.nif` capture containers) is normalised
//! to a [`RawFrame`](canboat_core::RawFrame) stream, then emitted as:
//!
//! - `plain` — canboat PLAIN/FAST lines (a pure frame→frame shunt via
//!   [`canboat_io::copy`]); subsumes `nif2analyzer` and format
//!   normalisation.
//! - `json` / `text` — fully decoded records via the analyzer decode
//!   pipeline; this is what the `analyzer` binary does.

use std::fs::File;
use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

use canboat_core::RawFrame;
use canboat_core::format::InputFormat;
use canboat_core::output::{GeoFormat, JsonOptions, TextOptions, write_json, write_text};
use canboat_io::{
    EblReader, EblWriter, FrameReader, FrameWriter, LineFrameReader, PlainWriter, TextLineWriter,
    analyze, container, copy,
};

/// Output format for `convert --to`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum OutFormat {
    /// canboat PLAIN/FAST lines (raw frames, no decode).
    Plain,
    /// One decoded JSON object per record.
    Json,
    /// canboat human-readable text, one line per record.
    Text,
    /// Yacht Devices YDWG-02 / YDEN received lines (raw frames).
    Ydwg02,
    /// Actisense N2K-ASCII lines (`A…`, raw frames).
    Actisense,
    /// Actisense `.ebl` binary log (raw frames).
    #[value(name = "actisense-ebl")]
    ActisenseEbl,
}

impl OutFormat {
    /// Formats that re-encode raw frames (no PGN decode). Everything
    /// else (`json`/`text`) runs the analyzer decode pipeline.
    fn is_frame_level(self) -> bool {
        matches!(
            self,
            OutFormat::Plain | OutFormat::Ydwg02 | OutFormat::Actisense | OutFormat::ActisenseEbl
        )
    }
}

/// Input line format for `convert --from`. When omitted, the format is
/// auto-detected from the first line. Mirrors the ASCII line formats
/// canboat itself recognises.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum FromFormat {
    /// canboat PLAIN / FAST — `<ts>,<prio>,<pgn>,<src>,<dst>,<len>,<hex>…`.
    #[value(name = "plain", alias = "fast")]
    Plain,
    /// PLAIN with single frames interleaved among coalesced fast-packets.
    #[value(name = "plain-mix-fast")]
    PlainMixFast,
    /// Actisense NGT-1 ASCII (`A<time> …`).
    #[value(name = "actisense", alias = "actisense-ascii")]
    Actisense,
    /// Yacht Devices YDWG-02 / YDEN (`<time> R/T <canid> <hex>…`).
    #[value(name = "ydwg02", alias = "yden")]
    Ydwg02,
    /// Digital Yacht iKonvert (`!PDGY,<pgn>,…`).
    #[value(name = "ikonvert")]
    Ikonvert,
    /// Airmar (`<ts> - <pgn> <canid> <hex>…`).
    #[value(name = "airmar")]
    Airmar,
    /// Chetco sea-gauge (`$PCDIN,…`).
    #[value(name = "chetco")]
    Chetco,
    /// Garmin CSV (relative ms-since-boot timestamps).
    #[value(name = "garmin", alias = "garmin-csv")]
    Garmin,
    /// Garmin CSV2 (absolute timestamps + `Processed PGN` column).
    #[value(name = "garmin-csv2")]
    GarminCsv2,
    /// Actisense `.ebl` binary log. Not a line format — decoded via the
    /// EBL frame reader, so it has no [`InputFormat`] mapping.
    #[value(name = "actisense-ebl")]
    ActisenseEbl,
}

impl FromFormat {
    /// Map to the canboat-core line-parser selector. `None` for the
    /// binary `.ebl` format, which uses its own frame reader.
    fn to_input_format(self) -> Option<InputFormat> {
        Some(match self {
            FromFormat::Plain => InputFormat::Plain,
            FromFormat::PlainMixFast => InputFormat::PlainMixFast,
            FromFormat::Actisense => InputFormat::ActisenseAscii,
            FromFormat::Ydwg02 => InputFormat::Ydwg02,
            FromFormat::Ikonvert => InputFormat::Ikonvert,
            FromFormat::Airmar => InputFormat::Airmar,
            FromFormat::Chetco => InputFormat::Chetco,
            FromFormat::Garmin => InputFormat::GarminCsv,
            FromFormat::GarminCsv2 => InputFormat::GarminCsv2,
            FromFormat::ActisenseEbl => return None,
        })
    }
}

/// Long help for `canboat convert --help`. Kept next to the parser and
/// wired into the subcommand from `main.rs` (clap takes the subcommand's
/// short `about` from the enum variant's doc comment, so `long_about`
/// has to be attached there).
pub const LONG_ABOUT: &str = "\
Convert a capture between formats: any supported input → PLAIN, JSON, or text.

Input line formats (auto-detected from the first line, or forced with --from):
  plain, plain-mix-fast, actisense, ydwg02, ikonvert, airmar, chetco,
  garmin, garmin-csv2.

Container files (unwrapped automatically by file extension):
  .pcap / .pcap.gz   libpcap SocketCAN capture (link-type 227)
  .nif               Navico Information File (filtered + raw capture groups)

Output formats (--to, default json):
  plain          canboat PLAIN/FAST lines (raw frames, no decode)
  json           one decoded JSON object per record
  text           canboat human-readable text, one line per record
  ydwg02         Yacht Devices YDWG-02 / YDEN received lines (raw frames)
  actisense      Actisense N2K-ASCII lines (raw frames)
  actisense-ebl  Actisense .ebl binary log (raw frames)";

#[derive(Debug, clap::Args)]
pub struct Args {
    /// Input file. A `.pcap` / `.pcap.gz` / `.nif` container is
    /// unwrapped automatically. Omit (or pass `-`) to read stdin.
    #[arg(long, value_name = "PATH")]
    file: Option<PathBuf>,

    /// Input file as a positional argument — an alternative spelling of
    /// `--file` (e.g. `canboat convert capture.nif`).
    #[arg(value_name = "FILE", conflicts_with = "file")]
    file_pos: Option<PathBuf>,

    /// Force the input line format instead of auto-detecting it from
    /// the first line. Ignored for `.pcap`/`.nif` containers.
    #[arg(long, value_enum, value_name = "FORMAT")]
    from: Option<FromFormat>,

    /// Output format.
    #[arg(long, value_enum, default_value_t = OutFormat::Json)]
    to: OutFormat,

    /// Filter: only emit frames with this source address.
    #[arg(long, value_name = "N")]
    src: Option<u8>,

    /// Filter: only emit frames with this destination address.
    #[arg(long, value_name = "N")]
    dst: Option<u8>,

    /// Filter: only emit frames with this PGN.
    #[arg(long, value_name = "PGN")]
    pgn: Option<u32>,

    /// `.nif` containers only: emit just the `raw` full-traffic
    /// captures, skipping the filtered device roster.
    #[arg(long, conflicts_with = "filtered_only")]
    raw_only: bool,

    /// `.nif` containers only: emit just the `filtered` device-roster
    /// captures.
    #[arg(long)]
    filtered_only: bool,
}

impl Args {
    /// The input path: `--file` or the positional form, whichever was
    /// given. `None` means stdin.
    fn input(&self) -> Option<&Path> {
        self.file.as_deref().or(self.file_pos.as_deref())
    }

    /// Which capture groups of a `.nif` to unwrap.
    fn container_opts(&self) -> container::Options {
        container::Options {
            filtered: !self.raw_only,
            raw: !self.filtered_only,
        }
    }
}

pub fn run(args: Args) -> Result<()> {
    // Surface decode-path diagnostics (e.g. a stream mislabeled
    // `# format=FAST` but carrying single frames) on stderr without
    // polluting the converted stdout. Warnings show by default; set
    // `RUST_LOG` for more. Ignore a double-init if a shim already set one.
    let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .try_init();
    let forced = args.from.and_then(FromFormat::to_input_format);
    let ebl = ebl_input(&args);
    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());

    if args.to.is_frame_level() {
        convert_raw(&args, forced, ebl, &mut out)
    } else {
        convert_decoded(&args, forced, ebl, &mut out)
    }
}

/// True when the input is an Actisense `.ebl` binary log: `--from
/// actisense-ebl`, or (unforced) a `.ebl` filename.
fn ebl_input(args: &Args) -> bool {
    matches!(args.from, Some(FromFormat::ActisenseEbl))
        || (args.from.is_none()
            && args
                .input()
                .is_some_and(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("ebl"))))
}

/// Raw path: input frames → the selected frame-level format, no decode.
/// The no-filter case is exactly [`canboat_io::copy`]; with filters we
/// run the same pull loop with a per-frame predicate. The writer is
/// chosen by `--to` (PLAIN / YDWG02 / Actisense ASCII / Actisense EBL).
fn convert_raw<W: Write>(
    args: &Args,
    forced: Option<InputFormat>,
    ebl: bool,
    out: &mut W,
) -> Result<()> {
    let source = open_source(args.input(), args.container_opts())?;
    let mut reader: Box<dyn FrameReader> = if ebl {
        Box::new(EblReader::new(source))
    } else {
        match forced {
            Some(fmt) => Box::new(LineFrameReader::with_format(source, fmt)),
            None => Box::new(LineFrameReader::new(source)),
        }
    };
    let mut writer: Box<dyn FrameWriter + '_> = match args.to {
        OutFormat::Plain => Box::new(PlainWriter::new(out)),
        OutFormat::Ydwg02 => Box::new(TextLineWriter::ydwg02(out)),
        OutFormat::Actisense => Box::new(TextLineWriter::actisense(out)),
        OutFormat::ActisenseEbl => Box::new(EblWriter::new(out)),
        OutFormat::Json | OutFormat::Text => unreachable!("decoded formats routed elsewhere"),
    };

    if !args.has_filter() {
        copy(&mut *reader, &mut *writer).context("converting frames")?;
        return Ok(());
    }
    while let Some(frame) = reader.read_frame().context("reading input")? {
        if args.keep(&frame) {
            writer.write_frame(&frame).context("writing frame")?;
        }
    }
    writer.flush().context("flushing output")?;
    Ok(())
}

/// Decoded path: drive the analyzer pipeline and render each record as
/// JSON or text. Filters are pushed into the pipeline's `Config`.
fn convert_decoded<W: Write>(
    args: &Args,
    forced: Option<InputFormat>,
    ebl: bool,
    out: &mut W,
) -> Result<()> {
    let json_opts = JsonOptions::default();
    let text_opts = TextOptions {
        show_unavailable: false,
        debug: false,
        geo: GeoFormat::Dd,
    };
    let as_json = args.to == OutFormat::Json;
    let cfg = analyze::Config {
        forced_format: forced,
        pgn_filter: args.pgn,
        src_filter: args.src,
        dst_filter: args.dst,
        suppress_startup_record: false,
        units: canboat_core::Units::Metric,
    };

    let mut line = String::with_capacity(512);
    let mut sink_err: Option<io::Error> = None;
    let sink = |decoded: &canboat_core::DecodedPgn| {
        if sink_err.is_some() {
            return;
        }
        line.clear();
        if as_json {
            write_json(&mut line, decoded, &json_opts).expect("write to String");
        } else {
            write_text(&mut line, decoded, &text_opts).expect("write to String");
        }
        if let Err(e) = out
            .write_all(line.as_bytes())
            .and_then(|()| out.write_all(b"\n"))
        {
            sink_err = Some(e);
        }
    };

    let source = open_source(args.input(), args.container_opts())?;
    if ebl {
        // `.ebl` records are already complete N2K messages, so skip the
        // line-reader / reassembly / coalesced-mode machinery entirely:
        // pull each frame and decode it directly, honouring the filters.
        let db = canboat_core::PgnDatabase::embedded(canboat_core::Units::Metric);
        let mut reader = EblReader::new(source);
        let mut sink = sink;
        while let Some(frame) = reader.read_frame().context("reading .ebl input")? {
            if cfg.pgn_filter.is_some_and(|w| frame.pgn != w)
                || cfg.src_filter.is_some_and(|w| frame.src != w)
                || cfg.dst_filter.is_some_and(|w| frame.dst != w)
            {
                continue;
            }
            if let Ok(decoded) = db.decode(&frame) {
                sink(&decoded);
            }
        }
    } else {
        analyze::decode_stream(source, &cfg, sink).context("decoding input")?;
    }
    if let Some(e) = sink_err {
        return Err(e).context("writing output");
    }
    Ok(())
}

/// Open the input as a canboat PLAIN-capable [`BufRead`]. A `.nif` /
/// `.pcap` container is unwrapped on the fly; `None` or `-` is stdin.
fn open_source(file: Option<&Path>, opts: container::Options) -> Result<Box<dyn BufRead>> {
    match file {
        Some(path) if path.as_os_str() != "-" => {
            if container::is_container(path) {
                container::plain_reader(path, opts)
                    .with_context(|| format!("opening {}", path.display()))
            } else {
                let f = File::open(path).with_context(|| format!("opening {}", path.display()))?;
                Ok(Box::new(BufReader::new(f)))
            }
        }
        // `Stdin::lock` yields a `'static` guard, so this is owned.
        _ => Ok(Box::new(io::stdin().lock())),
    }
}

impl Args {
    fn has_filter(&self) -> bool {
        self.src.is_some() || self.dst.is_some() || self.pgn.is_some()
    }

    /// True when `frame` passes the src/dst/pgn filters.
    fn keep(&self, frame: &RawFrame) -> bool {
        self.src.is_none_or(|s| frame.src == s)
            && self.dst.is_none_or(|d| frame.dst == d)
            && self.pgn.is_none_or(|p| frame.pgn == p)
    }
}
