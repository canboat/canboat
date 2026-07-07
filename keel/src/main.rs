//! keel - the CANboat PGN database tool. See DESIGN.md.
//!
//! Durable commands live here (Rust); the one-shot bootstrap converter
//! (canboat.xml -> database/) lives in keel/bootstrap/ (Python) and dies at
//! migration switchover.

mod cformat;
mod check;
mod derive;
mod emit_c;
mod emit_text;
mod emit_xml;
mod model;
mod yamlio;

use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use emit_xml::FloatStyle;

// Die silently on SIGPIPE (keel explain | head) instead of a Rust panic.
unsafe extern "C" {
    fn signal(signum: i32, handler: usize) -> usize;
}

fn find_repo_root(start: &Path) -> Result<PathBuf, String> {
    let mut path = start
        .canonicalize()
        .map_err(|e| format!("{}: {e}", start.display()))?;
    loop {
        if path.join("analyzer").is_dir() && path.join("docs").is_dir() {
            return Ok(path);
        }
        if !path.pop() {
            return Err(
                "cannot find canboat repository root (looked for analyzer/ and docs/)".into(),
            );
        }
    }
}

fn read_versions(root: &Path) -> Result<(String, String), String> {
    let text = fs::read_to_string(root.join("common/version.h"))
        .map_err(|e| format!("common/version.h: {e}"))?;
    let grab = |key: &str| -> Result<String, String> {
        text.lines()
            .find(|l| l.contains(&format!("#define {key} ")))
            .and_then(|l| l.split('"').nth(1))
            .map(String::from)
            .ok_or_else(|| format!("common/version.h: no #define {key}"))
    };
    Ok((grab("VERSION")?, grab("SCHEMA_VERSION")?))
}

struct Args {
    command: String,
    check: bool,
    diff: Option<String>,
    float_style: FloatStyle,
    which: String,
    root: PathBuf,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        command: String::new(),
        check: false,
        diff: None,
        float_style: FloatStyle::C,
        which: "normal".into(),
        root: PathBuf::from("."),
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--check" => args.check = true,
            "--diff" => args.diff = Some(it.next().ok_or("--diff needs a path")?),
            "--root" => args.root = PathBuf::from(it.next().ok_or("--root needs a path")?),
            "--which" => args.which = it.next().ok_or("--which needs normal|actisense|ikonvert")?,
            "--float-style" => {
                args.float_style = match it.next().as_deref() {
                    Some("c") => FloatStyle::C,
                    Some("rust") => FloatStyle::Rust,
                    _ => return Err("--float-style needs c|rust".into()),
                }
            }
            cmd if args.command.is_empty() && !cmd.starts_with('-') => args.command = cmd.into(),
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    if args.command.is_empty() {
        return Err("usage: keel <check|generate|emit|explain> [--check] [--diff FILE] [--which normal|actisense|ikonvert] [--float-style c|rust] [--root DIR]".into());
    }
    Ok(args)
}

fn write_diff(original: &str, emitted: &str, path: &str) -> std::io::Result<()> {
    // Minimal unified-ish diff: first divergent line with context counts.
    let o: Vec<&str> = original.lines().collect();
    let e: Vec<&str> = emitted.lines().collect();
    let mut out = String::from("--- docs/canboat.xml\n+++ emitted\n");
    let mut i = 0;
    while i < o.len() && i < e.len() && o[i] == e[i] {
        i += 1;
    }
    let end = |v: &Vec<&str>| v.len().min(i + 40);
    out.push_str(&format!("@@ first divergence at line {} @@\n", i + 1));
    for line in &o[i..end(&o)] {
        out.push_str(&format!("-{line}\n"));
    }
    for line in &e[i..end(&e)] {
        out.push_str(&format!("+{line}\n"));
    }
    fs::write(path, out)
}

fn run() -> Result<i32, String> {
    let args = parse_args()?;
    let root = find_repo_root(&args.root)?;
    let (version, schema) = read_versions(&root)?;
    let db_dir = root.join("database");
    if !db_dir.is_dir() {
        return Err(format!(
            "no database/ tree at {} (run the bootstrap converter first)",
            db_dir.display()
        ));
    }

    let mut db = yamlio::load_database(&db_dir, &version, &schema)?;
    // fieldtype-data.h is emitted from the authored (pre-percolation) state
    let authored_fieldtypes = db.fieldtypes.clone();
    derive::fill(&mut db)?;

    match args.command.as_str() {
        "check" => {
            let violations = check::check(&db);
            let errors = violations.iter().filter(|v| v.error).count();
            let warnings = violations.len() - errors;
            for v in &violations {
                println!(
                    "{} {} {}: {}",
                    v.rule,
                    if v.error { "ERROR  " } else { "warning" },
                    v.location,
                    v.message
                );
            }
            println!(
                "keel check: {} pgns, {} lookups, {} fieldtypes: {errors} error(s), {warnings} warning(s)",
                db.pgns.len(),
                db.lookups.len(),
                db.fieldtypes.len()
            );
            Ok(if errors > 0 { 1 } else { 0 })
        }
        "generate" => {
            let artifacts: Vec<(PathBuf, String)> = vec![
                (
                    root.join("docs/canboat.xml"),
                    emit_xml::emit_xml(&db, "normal", args.float_style),
                ),
                (root.join("analyzer/lookup.h"), emit_c::emit_lookup_h(&db)),
                (
                    root.join("analyzer/physicalquantity-data.h"),
                    emit_c::emit_physicalquantity_data_h(&db),
                ),
                (
                    root.join("analyzer/fieldtype-data.h"),
                    emit_c::emit_fieldtype_data_h(&authored_fieldtypes),
                ),
                (
                    root.join("analyzer/pgn-data.h"),
                    emit_c::emit_pgn_data_h(&db, false),
                ),
                (
                    root.join("analyzer/pgn-j1939-data.h"),
                    emit_c::emit_pgn_data_h(&db, true),
                ),
            ];
            let mut stale = 0;
            for (path, emitted) in &artifacts {
                if args.check {
                    let original = fs::read_to_string(path).unwrap_or_default();
                    if emitted != &original {
                        stale += 1;
                        eprintln!(
                            "keel generate --check: {} is NOT up to date with database/",
                            path.display()
                        );
                        if let Some(diff) = &args.diff {
                            let dpath =
                                format!("{diff}.{}", path.file_name().unwrap().to_string_lossy());
                            write_diff(&original, emitted, &dpath).map_err(|e| e.to_string())?;
                            eprintln!("keel generate --check: diff written to {dpath}");
                        }
                    }
                } else {
                    fs::write(path, emitted).map_err(|e| e.to_string())?;
                    println!("keel generate: wrote {}", path.display());
                }
            }
            if args.check {
                if stale > 0 {
                    return Ok(1);
                }
                println!(
                    "keel generate --check: all {} artifacts up to date",
                    artifacts.len()
                );
            }
            Ok(0)
        }
        "explain" => {
            print!("{}", emit_text::emit_text(&db, args.which == "j1939"));
            Ok(0)
        }
        "emit" => {
            // Emit any document to stdout (dev tool; also the BEM documents)
            print!("{}", emit_xml::emit_xml(&db, &args.which, args.float_style));
            Ok(0)
        }
        other => Err(format!("unknown command '{other}'")),
    }
}

fn main() -> ExitCode {
    unsafe {
        signal(13, 0); // SIGPIPE, SIG_DFL
    }
    match run() {
        Ok(code) => ExitCode::from(code as u8),
        Err(e) => {
            eprintln!("keel: {e}");
            ExitCode::from(2)
        }
    }
}
