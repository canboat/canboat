//! keel - the CANboat PGN database tool. See DESIGN.md.
//!
//! The database under database/ is the source of truth; keel validates it,
//! generates the analyzer's C tables and canboat.xml, decodes sample frames
//! and serves the editor.

use keel::model::Protocol;
use keel::{check, decode, derive, edit, emit_text, emit_xml, harvest, rules, samples, yamlio};
use keel::{find_repo_root, generate, read_versions};

use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

// Die silently on SIGPIPE (keel explain | head) instead of a Rust panic.
// Unix only: Windows has no SIGPIPE and no libc `signal` to link against.
#[cfg(unix)]
unsafe extern "C" {
    fn signal(signum: i32, handler: usize) -> usize;
}

struct Args {
    command: String,
    help: bool,
    version: bool,
    check: bool,
    port: Option<u16>,
    diff: Option<String>,
    protocol: Protocol,
    root: PathBuf,
    per_pgn: usize,
    rest: Vec<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut args = Args {
        command: String::new(),
        help: false,
        version: false,
        check: false,
        port: None,
        diff: None,
        protocol: Protocol::Nmea2000,
        root: PathBuf::from("."),
        per_pgn: 3,
        rest: Vec::new(),
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--help" | "-h" => args.help = true,
            "--version" | "-V" => args.version = true,
            "--check" => args.check = true,
            "--port" => {
                args.port = Some(
                    it.next()
                        .ok_or("--port needs a number")?
                        .parse()
                        .map_err(|e| format!("--port: {e}"))?,
                )
            }
            "--diff" => args.diff = Some(it.next().ok_or("--diff needs a path")?),
            "--per-pgn" => {
                args.per_pgn = it
                    .next()
                    .ok_or("--per-pgn needs a number")?
                    .parse()
                    .map_err(|e| format!("--per-pgn: {e}"))?
            }
            "--root" => args.root = PathBuf::from(it.next().ok_or("--root needs a path")?),
            "--protocol" => {
                args.protocol = it
                    .next()
                    .as_deref()
                    .and_then(Protocol::parse)
                    .ok_or("--protocol needs nmea2000|j1939|quick")?;
            }
            cmd if args.command.is_empty() && !cmd.starts_with('-') => args.command = cmd.into(),
            pos if !pos.starts_with('-') => args.rest.push(pos.to_string()),
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    if args.command == "help" {
        args.help = true;
    }
    if args.command.is_empty() && !args.help && !args.version {
        return Err(format!("{USAGE}\n(keel --help for more)"));
    }
    Ok(args)
}

const USAGE: &str = "usage: keel <command> [options] [files...]";

const HELP: &str = "\
keel - the CANboat PGN database tool

The YAML under database/ is the source of truth; keel checks it, generates
the documents, the analyzer's C tables and the Rust schema from it, decodes
sample frames against it, and serves the web editor.

usage: keel <command> [options] [files...]

Commands:
  check              check the database against every rule (R01..), and
                     its samples against their expected decodes
  generate           write every generated artifact, for every protocol:
                     docs/canboat*.xml, analyzer/*-generated-data.h (not
                     for Quick), crates/canboat/src/engine/schema_generated*.rs.
                     `make generated` also runs this, then builds the
                     JSON, HTML and DBC documents from the XML
  emit               print a protocol's XML document (docs/canboat.xml, or
                     docs/canboat-<protocol>.xml) on stdout
  explain            print the database as readable text
  decode             decode sample lines from stdin (PLAIN, candump, YDWG
                     RAW) with keel's own decoder
  edit               start the web editor on localhost and open it
  harvest FILE...    decode capture files and add a few samples per PGN
                     variant to the database
  rules [md]         print the rule inventory (as Markdown with `md`)
  help               this text

Options:
  --protocol nmea2000|j1939|quick
                     the protocol for explain, emit and decode (default
                     nmea2000)
  --check            generate: write nothing, exit 1 when an artifact is
                     out of date
  --diff FILE        generate --check: write where it first differs to FILE
  --port N           edit: the port to serve on (default 8020)
  --per-pgn N        harvest: samples to keep per variant (default 3)
  --root DIR         look for the repository from DIR (default: .)
  -h, --help         this text
  -V, --version      print keel's version, and the database's when run
                     inside the repository
";

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
    if args.help {
        print!("{HELP}");
        return Ok(0);
    }
    if args.version {
        // keel's own version, and the database's when there is one to read.
        let db = find_repo_root(&args.root)
            .and_then(|root| read_versions(&root))
            .map(|(version, schema)| format!(" (CANboat {version}, schema {schema})"))
            .unwrap_or_default();
        println!("keel {}{db}", env!("CARGO_PKG_VERSION"));
        return Ok(0);
    }
    // `rules` is pure documentation — needs neither the repo nor the database,
    // so it works anywhere (e.g. regenerating docs).
    if args.command == "rules" {
        let md = args.rest.iter().any(|a| a == "md" || a == "markdown");
        print!(
            "{}",
            if md {
                rules::render_md()
            } else {
                rules::render_text()
            }
        );
        return Ok(0);
    }
    let root = find_repo_root(&args.root)?;
    let (version, schema) = read_versions(&root)?;
    let db_dir = root.join("database");
    if !db_dir.is_dir() {
        return Err(format!(
            "no database/ tree at {} (wrong --root?)",
            db_dir.display()
        ));
    }

    let mut db = yamlio::load_database(&db_dir, &version, &schema)?;
    // fieldtype-generated-data.h is emitted from the authored (pre-percolation) state
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
            let pgns: Vec<String> = Protocol::ALL
                .into_iter()
                .map(|p| format!("{} {}", db.pgns_of(p).len(), p.name()))
                .collect();
            println!(
                "keel check: {} pgns, {} lookups, {} fieldtypes: {errors} error(s), {warnings} warning(s)",
                pgns.join(" + "),
                db.lookups.len(),
                db.fieldtypes.len()
            );
            Ok(if errors > 0 { 1 } else { 0 })
        }
        "generate" => {
            let artifacts = generate::emit_artifacts(&root, &db, &authored_fieldtypes);
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
                } else if fs::read_to_string(path).ok().as_deref() == Some(emitted.as_str()) {
                    // Byte-identical: leave the file alone. Rewriting it would
                    // bump its mtime and make every downstream `make` rule fire
                    // — the C analyzer rebuilds off these headers, so an
                    // unconditional write turns `make rust` into a full C
                    // rebuild for no reason.
                    println!("keel generate: {} is up to date", path.display());
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
        "decode" => {
            // Read sample lines from stdin, reassemble, decode, print.
            let protocol = args.protocol;
            let mut fast: std::collections::HashSet<u32> = Default::default();
            for p in db.pgns_of(protocol) {
                if p.type_ == "Fast" {
                    fast.insert(p.pgn);
                }
            }
            let mut frames = Vec::new();
            for (n, line) in std::io::read_to_string(std::io::stdin())
                .map_err(|e| e.to_string())?
                .lines()
                .enumerate()
            {
                if line.trim().is_empty() || line.trim_start().starts_with('#') {
                    continue;
                }
                frames.push(samples::parse_line(line).map_err(|e| format!("line {}: {e}", n + 1))?);
            }
            let (assembled, warnings) =
                samples::reassemble_lenient(&frames, |pgn| fast.contains(&pgn))?;
            for w in &warnings {
                eprintln!("keel decode: warning: {w}");
            }
            for a in &assembled {
                match decode::select_variant(&db, a.pgn, &a.data, protocol) {
                    None => println!("PGN {}: unknown", a.pgn),
                    Some(p) => {
                        println!("PGN {} {} (src {}):", a.pgn, p.id, a.src);
                        for d in decode::decode(&db, p, &a.data)? {
                            let inst = if d.instance > 1 {
                                format!(".{}", d.instance)
                            } else {
                                String::new()
                            };
                            let unit = d
                                .unit
                                .as_deref()
                                .map(|u| format!(" {u}"))
                                .unwrap_or_default();
                            println!("  {}{} = {}{}", d.id, inst, d.value, unit);
                        }
                    }
                }
            }
            Ok(0)
        }
        "harvest" => {
            if args.rest.is_empty() {
                return Err("usage: keel harvest [--per-pgn N] <capture-file>...".into());
            }
            let summary = harvest::harvest(&db, &args.rest, args.per_pgn, &root)?;
            println!("{summary}");
            Ok(0)
        }
        "edit" => {
            let port = args.port.unwrap_or(8020);
            edit::serve(
                edit::EditServer {
                    root: root.clone(),
                    version: version.clone(),
                    schema_version: schema.clone(),
                    last_save: std::sync::Mutex::new(None),
                },
                port,
            )?;
            Ok(0)
        }
        "explain" => {
            print!("{}", emit_text::emit_text(&db, args.protocol));
            Ok(0)
        }
        "emit" => {
            // One of the two documents, on stdout.
            print!("{}", emit_xml::emit_xml(&db, args.protocol));
            Ok(0)
        }
        other => Err(format!("unknown command '{other}'")),
    }
}

fn main() -> ExitCode {
    #[cfg(unix)]
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
