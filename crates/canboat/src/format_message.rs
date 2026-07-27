// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat format-message` — assemble a single NMEA 2000 frame from
//! field values given on the command line and print it in a wire format.
//!
//! This is the Rust counterpart of canboat C's `util/format-message`,
//! but it works for **every** PGN in the database rather than a
//! hand-maintained subset — because the whole schema is embedded.
//!
//! The library equivalent (and the thing this is a thin wrapper over) is
//! [`canboat_core::PgnDatabase::encode`] →
//! [`canboat_core::PgnBuilder`]. Anything you can do here you can do
//! from a program:
//!
//! ```no_run
//! use canboat_core::{PgnDatabase, Units, field::iso_request};
//! let db = PgnDatabase::embedded(Units::Metric);
//! let frame = db.encode("isoRequest").unwrap()
//!     .destination(0)
//!     .push(iso_request::PGN, 126996u32).unwrap() // O(1), compile-checked
//!     .build().unwrap();
//! ```
//!
//! On the CLI (`FIELD=VALUE`) the field is chosen at runtime, so the
//! wrapper uses [`PgnBuilder::push_arg`] (name-or-id) instead.

use std::io::Write as _;

use anyhow::bail;
use canboat_core::format::{actisense_ascii, ebl, plain, ydwg02};
use canboat_core::{PgnBuilder, PgnDatabase, PgnInfo, Units};

#[derive(Debug, clap::Args)]
#[command(disable_help_flag = true)]
pub struct Args {
    /// PGN to build: schema id (e.g. `isoRequest`) or number (`126996`).
    pgn: Option<String>,

    /// Field assignments as `FIELD=VALUE` (field by name or id). Fields
    /// you don't set default to "not available", or — for a variant's
    /// selector fields — to the value that selects this variant.
    #[arg(value_name = "FIELD=VALUE")]
    assignments: Vec<String>,

    /// Source address (default 0).
    #[arg(long)]
    src: Option<u8>,
    /// Destination address (default 255 = broadcast).
    #[arg(long)]
    dst: Option<u8>,
    /// CAN priority (default: the PGN's schema priority, else 6).
    #[arg(long)]
    prio: Option<u8>,
    /// Interpret values in SI units (`rad`, `K`, `Pa`) instead of the
    /// default practical units (`deg`, `°C`, `bar`). Mirrors `analyzer -si`.
    #[arg(long)]
    si: bool,
    /// Output wire format.
    #[arg(long, value_enum, default_value_t = OutFmt::Plain)]
    to: OutFmt,
    /// List every known PGN (number, id, description) and exit.
    #[arg(long)]
    list: bool,
    /// Show usage; with a PGN, list that PGN's settable fields instead.
    #[arg(short = 'h', long)]
    help: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum OutFmt {
    /// canboat PLAIN: `<ts>,<prio>,<pgn>,<src>,<dst>,<len>,<hex…>`.
    #[value(name = "plain", alias = "fast")]
    Plain,
    /// Actisense ASCII (`A…`).
    #[value(name = "actisense", alias = "actisense-ascii")]
    Actisense,
    /// Yacht Devices YDWG-02 / YDEN text.
    #[value(name = "ydwg02", alias = "yden")]
    Ydwg02,
    /// Actisense `.ebl` binary (written raw to stdout).
    #[value(name = "actisense-ebl")]
    ActisenseEbl,
}

pub fn run(args: Args) -> anyhow::Result<()> {
    let db = PgnDatabase::embedded(if args.si { Units::Si } else { Units::Metric });

    if args.list {
        list_pgns(db);
        return Ok(());
    }

    let Some(pgn) = args.pgn.as_deref() else {
        if args.help {
            print_usage();
            return Ok(());
        }
        bail!("specify a PGN (id or number), or --list. See --help.");
    };

    let mut builder = resolve(db, pgn)?;

    // `--help` with a resolved PGN prints its field table, not usage.
    if args.help {
        describe(builder.pgn_info(), args.si);
        return Ok(());
    }

    if let Some(p) = args.prio {
        builder = builder.priority(p);
    }
    if let Some(s) = args.src {
        builder = builder.source(s);
    }
    if let Some(d) = args.dst {
        builder = builder.destination(d);
    }

    for a in &args.assignments {
        let Some((field, value)) = a.split_once('=') else {
            bail!("bad assignment '{a}' (expected FIELD=VALUE)");
        };
        builder.push_arg(field.trim(), value.trim())?;
    }

    let frame = builder.build()?;

    match args.to {
        OutFmt::Plain => print_text(|s| plain::write_line(s, &frame)),
        OutFmt::Actisense => print_text(|s| actisense_ascii::write_line(s, &frame)),
        OutFmt::Ydwg02 => print_text(|s| ydwg02::write_line(s, &frame)),
        OutFmt::ActisenseEbl => {
            let mut bytes = Vec::new();
            ebl::encode_frame(&frame, &mut bytes);
            std::io::stdout().write_all(&bytes)?;
        }
    }
    Ok(())
}

/// Resolve a PGN argument (number if all-digits, else schema id) into a
/// builder against `db`.
fn resolve(db: &'static PgnDatabase, pgn: &str) -> anyhow::Result<PgnBuilder> {
    match pgn.parse::<u32>() {
        Ok(num) => Ok(db.encode_by_pgn(num)?),
        Err(_) => Ok(db.encode(pgn)?),
    }
}

/// Format a text line into a `String`, trim the trailing newline the
/// writer may or may not add, and print exactly one.
fn print_text(f: impl FnOnce(&mut String) -> std::fmt::Result) {
    let mut s = String::new();
    if f(&mut s).is_ok() {
        println!("{}", s.trim_end_matches(['\r', '\n']));
    }
}

fn list_pgns(db: &'static PgnDatabase) {
    for p in db.pgns() {
        println!("{:>6}  {:34}  {}", p.pgn, p.id, p.description);
    }
}

/// Print the settable fields of a PGN — the per-PGN `--help`.
fn describe(pgn: &'static PgnInfo, si: bool) {
    println!("PGN {} — {} [{}]", pgn.pgn, pgn.description, pgn.id);
    if let Some(e) = pgn.explanation {
        println!("  {e}");
    }
    println!(
        "\nFields (set as FIELD=VALUE, by name or id; units are {}):",
        if si { "SI" } else { "practical/Metric" }
    );
    for f in pgn.fields {
        let kind = match f.lookup_enumeration {
            Some(name) => format!("lookup {name}"),
            None => match f.field_type {
                Some(ft) => format!("{ft:?}"),
                None => "Number".to_string(),
            },
        };
        let unit = f.unit.map(|u| format!(" [{u}]")).unwrap_or_default();
        let note = if f.match_value.is_some() {
            "  (auto: variant selector)"
        } else {
            ""
        };
        println!(
            "  {:>2}. {:26} id={:26} {}{}{}",
            f.order, f.name, f.id, kind, unit, note
        );
    }
    println!("\nUnset fields default to \"not available\". Example:");
    let example = pgn
        .fields
        .iter()
        .find(|f| f.match_value.is_none())
        .map(|f| format!("{}=<value>", f.id))
        .unwrap_or_default();
    println!("  canboat format-message {} {example}", pgn.id);
}

fn print_usage() {
    println!(
        "Usage: canboat format-message <PGN> [FIELD=VALUE ...] [--src N] [--dst N] \
         [--prio N] [--si] [--to FORMAT]\n\
         \n\
         <PGN> is a schema id (isoRequest) or number (126996).\n\
         \n\
         canboat format-message --list              list all PGNs\n\
         canboat format-message <PGN> --help        list a PGN's settable fields\n\
         \n\
         Example:\n\
         \x20 canboat format-message isoRequest --dst 0 PGN=126996"
    );
}
