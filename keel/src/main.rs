//! keel - the CANboat PGN database tool. See DESIGN.md.
//!
//! Durable commands live here (Rust); the one-shot bootstrap converter
//! (canboat.xml -> database/) lives in keel/bootstrap/ (Python) and dies at
//! migration switchover.

mod cformat;
mod derive;
mod emit_xml;
mod model;
mod yamlio;

use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use emit_xml::FloatStyle;

fn find_repo_root(start: &Path) -> Result<PathBuf, String> {
    let mut path = start
        .canonicalize()
        .map_err(|e| format!("{}: {e}", start.display()))?;
    loop {
        if path.join("analyzer").is_dir() && path.join("docs").is_dir() {
            return Ok(path);
        }
        if !path.pop() {
            return Err("cannot find canboat repository root (looked for analyzer/ and docs/)".into());
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
        return Err("usage: keel <generate|emit> [--check] [--diff FILE] [--which normal|actisense|ikonvert] [--float-style c|rust] [--root DIR]".into());
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
    derive::fill(&mut db)?;

    match args.command.as_str() {
        "generate" => {
            let emitted = emit_xml::emit_xml(&db, "normal", args.float_style);
            let xml_path = root.join("docs/canboat.xml");
            if args.check {
                let original = fs::read_to_string(&xml_path).map_err(|e| e.to_string())?;
                if emitted != original {
                    eprintln!("keel generate --check: docs/canboat.xml is NOT up to date with database/");
                    if let Some(diff) = &args.diff {
                        write_diff(&original, &emitted, diff).map_err(|e| e.to_string())?;
                        eprintln!("keel generate --check: diff written to {diff}");
                    }
                    return Ok(1);
                }
                println!("keel generate --check: docs/canboat.xml is up to date");
            } else {
                fs::write(&xml_path, emitted).map_err(|e| e.to_string())?;
                println!("keel generate: wrote {}", xml_path.display());
            }
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
    match run() {
        Ok(code) => ExitCode::from(code as u8),
        Err(e) => {
            eprintln!("keel: {e}");
            ExitCode::from(2)
        }
    }
}
