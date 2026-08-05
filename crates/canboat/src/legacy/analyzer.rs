// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `analyzer` legacy alias-module.
//!
//! `analyzer` has a rich, golden-tested CLI (JSON banner, `--fixtime`
//! startup-record suppression, `--nv/--empty/--geo/--id/--debug`)
//! that doesn't map onto a `convert` flag prefix, so it is preserved
//! verbatim here and dispatched by argv[0] rather than translated. The
//! heavy lifting still lives in [`canboat_io::analyze`]; this is only
//! the CLI + output glue.
//!
//! This is a copy of the standalone `analyzer` binary's `main.rs`; the
//! standalone crate is kept as a fallback until the shims are verified
//! against the n2kd parity harness, then retired — at which point this
//! becomes the single source of truth.

use std::ffi::OsString;
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

use canboat_core::{
    format::InputFormat,
    output::{GeoFormat, JsonOptions, TextOptions, write_json, write_text},
};
use canboat_io::analyze::{self, Config};

#[derive(Debug, Parser)]
#[command(
    name = "analyzer",
    about = "Decode canboat PLAIN/FAST lines from stdin into text or JSON",
    version,
    after_help = canboat_cli::help_footer()
)]
struct Cli {
    /// Read input from this file instead of stdin. Besides the ASCII
    /// line formats, a `.pcap` / `.pcap.gz` SocketCAN capture or a
    /// Navico `.nif` export is unwrapped automatically.
    #[arg(long, value_name = "PATH")]
    file: Option<PathBuf>,

    /// Emit JSON instead of canboat text.
    #[arg(long)]
    json: bool,

    /// JSON: include `null` for unavailable fields (`-empty`).
    #[arg(long)]
    empty: bool,

    /// JSON: emit lookup values as `{"value":N,"name":"..."}` (`-nv`).
    #[arg(long)]
    nv: bool,

    /// JSON: wrap every field with byte/bit diagnostics (`-debug`).
    #[arg(long)]
    debug: bool,

    /// Lat/lon display format — `dd` (default), `dm`, `dms`.
    #[arg(long, value_name = "FMT", default_value = "dd")]
    geo: String,

    /// Identifier spelling (`--id`) and unit system (`--units`), plus
    /// their deprecated canboat C spellings. Defaults: camelCase + SI.
    /// canboat C's historical shape is `--id spaces --units metric`.
    #[command(flatten)]
    shape: crate::output_opts::ShapeArgs,

    /// Use the given string in place of any analyzer-generated
    /// timestamps (matches canboat's `-fixtime`).
    #[arg(long, value_name = "STRING")]
    fixtime: Option<String>,

    /// Filter: only process frames with this source address.
    #[arg(long, value_name = "N")]
    src: Option<u8>,

    /// Filter: only process frames with this destination address.
    #[arg(long, value_name = "N")]
    dst: Option<u8>,

    /// Force a specific input format instead of auto-detecting.
    #[arg(long, value_name = "NAME")]
    format: Option<String>,

    /// Filter: only process frames with this PGN number.
    #[arg(value_name = "PGN")]
    pgn: Option<u32>,
}

fn parse_format_flag(name: &str) -> Result<InputFormat> {
    Ok(match name.to_ascii_lowercase().as_str() {
        "plain" | "fast" | "plain_or_fast" => InputFormat::Plain,
        "plain_mix_fast" | "plain-mix-fast" => InputFormat::PlainMixFast,
        "actisense" | "actisense-ascii" | "actisense_n2k_ascii" => InputFormat::ActisenseAscii,
        "ydwg02" | "yden" => InputFormat::Ydwg02,
        "ikonvert" => InputFormat::Ikonvert,
        "airmar" => InputFormat::Airmar,
        "chetco" => InputFormat::Chetco,
        "garmin" | "garmin-csv" | "garmin_csv1" => InputFormat::GarminCsv,
        "garmin-csv2" | "garmin_csv2" => InputFormat::GarminCsv2,
        other => anyhow::bail!("unknown --format {other:?}"),
    })
}

/// Entry point for the `analyzer` argv[0] alias.
pub fn run(argv: Vec<OsString>) -> Result<()> {
    run_cli(Cli::parse_from(argv))
}

fn run_cli(cli: Cli) -> Result<()> {
    cli.shape.warn_deprecated();
    let units = cli.shape.units();

    let json_opts = JsonOptions {
        include_empty: cli.empty,
        name_value: cli.nv,
        debug: cli.debug,
        camel_case: cli.shape.camel_case(),
        wrap: cli.shape.wrap(),
    };
    let geo = match cli.geo.as_str() {
        "dd" => GeoFormat::Dd,
        "dm" => GeoFormat::Dm,
        "dms" => GeoFormat::Dms,
        other => anyhow::bail!("--geo must be one of dd, dm, dms (got {other:?})"),
    };
    let text_opts = TextOptions {
        show_unavailable: cli.empty,
        debug: cli.debug,
        geo,
    };

    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut line_buf = String::with_capacity(512);

    // canboat's analyzer leads with a one-line JSON banner. `-fixtime`
    // suppresses it — unless the fixed timestamp string contains
    // "n2kd", in which case n2kd still wants the banner.
    let suppress_banner = cli.fixtime.as_deref().is_some_and(|s| !s.contains("n2kd"));
    if cli.json && !suppress_banner {
        writeln!(
            out,
            "{}",
            canboat_bridge::build_info::version_banner(cli.shape.is_si(), cli.nv)
        )
        .context("writing JSON banner")?;
    }

    let forced_format = cli.format.as_deref().map(parse_format_flag).transpose()?;

    let cfg = Config {
        forced_format,
        pgn_filter: cli.pgn,
        src_filter: cli.src,
        dst_filter: cli.dst,
        suppress_startup_record: cli.fixtime.as_deref().is_some_and(|s| !s.contains("n2kd")),
        units,
    };

    let mut sink_err: Option<anyhow::Error> = None;
    let sink = |decoded: &canboat_core::DecodedPgn| {
        if sink_err.is_some() {
            return;
        }
        line_buf.clear();
        if cli.json {
            write_json(&mut line_buf, decoded, &json_opts).expect("write to String");
        } else {
            write_text(&mut line_buf, decoded, &text_opts).expect("write to String");
        }
        if let Err(e) = out
            .write_all(line_buf.as_bytes())
            .and_then(|()| out.write_all(b"\n"))
        {
            sink_err = Some(anyhow::Error::new(e).context("writing line"));
        }
    };
    let result = if let Some(path) = cli.file.as_deref() {
        analyze::decode_file(path, &cfg, sink)
    } else {
        let stdin = io::stdin();
        analyze::decode_stream(stdin.lock(), &cfg, sink)
    };
    if let Some(e) = sink_err {
        return Err(e);
    }
    result.map_err(Into::into)
}
