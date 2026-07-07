//! `keel edit` - the local web editor (DESIGN.md §7).
//!
//! Deliberately dependency-free: a minimal hand-rolled HTTP/1.1 server on
//! localhost (GET + POST with Content-Length, thread per connection) and
//! hand-emitted JSON responses. Candidate PGN definitions travel as YAML -
//! the same text that gets saved - so the existing loader is the parser and
//! no JSON parser is needed.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::sync::Arc;

use crate::model::Database;
use crate::{check, decode, derive, samples, yamlio};

pub struct EditServer {
    pub root: PathBuf,
    pub version: String,
    pub schema_version: String,
}

const INDEX_HTML: &str = include_str!("../web/index.html");

pub fn serve(server: EditServer, port: u16) -> Result<(), String> {
    let listener =
        TcpListener::bind(("127.0.0.1", port)).map_err(|e| format!("bind 127.0.0.1:{port}: {e}"))?;
    let addr = listener.local_addr().map_err(|e| e.to_string())?;
    println!("keel edit: http://{addr}/  (Ctrl-C to stop)");
    let _ = open_browser(&format!("http://{addr}/"));

    let server = Arc::new(server);
    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        let server = Arc::clone(&server);
        std::thread::spawn(move || {
            let _ = handle(&server, stream);
        });
    }
    Ok(())
}

fn open_browser(url: &str) -> std::io::Result<()> {
    #[cfg(target_os = "macos")]
    let cmd = "open";
    #[cfg(not(target_os = "macos"))]
    let cmd = "xdg-open";
    std::process::Command::new(cmd).arg(url).spawn().map(|_| ())
}

fn handle(server: &EditServer, mut stream: TcpStream) -> std::io::Result<()> {
    let mut buf = Vec::new();
    let mut tmp = [0u8; 4096];
    // read until end of headers
    let header_end = loop {
        let n = stream.read(&mut tmp)?;
        if n == 0 {
            return Ok(());
        }
        buf.extend_from_slice(&tmp[..n]);
        if let Some(pos) = find_subslice(&buf, b"\r\n\r\n") {
            break pos + 4;
        }
        if buf.len() > 1 << 20 {
            return Ok(());
        }
    };
    let head = String::from_utf8_lossy(&buf[..header_end]).into_owned();
    let mut lines = head.lines();
    let request = lines.next().unwrap_or_default().to_string();
    let mut content_length = 0usize;
    for l in lines {
        if let Some(v) = l.to_ascii_lowercase().strip_prefix("content-length:").map(str::trim) {
            content_length = v.parse().unwrap_or(0);
        }
    }
    let mut body = buf[header_end..].to_vec();
    while body.len() < content_length {
        let n = stream.read(&mut tmp)?;
        if n == 0 {
            break;
        }
        body.extend_from_slice(&tmp[..n]);
    }
    let body = String::from_utf8_lossy(&body).into_owned();

    let mut parts = request.split_whitespace();
    let method = parts.next().unwrap_or_default();
    let target = parts.next().unwrap_or_default();
    let (path, query) = match target.split_once('?') {
        Some((p, q)) => (p, q),
        None => (target, ""),
    };

    let (status, ctype, payload) = route(server, method, path, query, &body);
    let response = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
        payload.len()
    );
    stream.write_all(response.as_bytes())?;
    stream.write_all(payload.as_bytes())?;
    Ok(())
}

fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

fn route(
    server: &EditServer,
    method: &str,
    path: &str,
    query: &str,
    body: &str,
) -> (&'static str, &'static str, String) {
    let json = "application/json; charset=utf-8";
    match (method, path) {
        ("GET", "/") => {
            // Prefer the on-disk file during development, fall back to the
            // compiled-in copy.
            let disk = server.root.join("keel/web/index.html");
            let html = std::fs::read_to_string(disk).unwrap_or_else(|_| INDEX_HTML.to_string());
            ("200 OK", "text/html; charset=utf-8", html)
        }
        ("GET", "/api/model") => match api_model(server) {
            Ok(j) => ("200 OK", json, j),
            Err(e) => ("500 Internal Server Error", json, err_json(&e)),
        },
        ("POST", "/api/analyze") => match api_analyze(server, body) {
            Ok(j) => ("200 OK", json, j),
            Err(e) => ("200 OK", json, err_json(&e)),
        },
        ("POST", "/api/decode") => match api_decode(server, query, body) {
            Ok(j) => ("200 OK", json, j),
            Err(e) => ("200 OK", json, err_json(&e)),
        },
        ("POST", "/api/save") => match api_save(server, query, body) {
            Ok(j) => ("200 OK", json, j),
            Err(e) => ("200 OK", json, err_json(&e)),
        },
        ("GET", "/api/pgn") => match api_pgn(server, query) {
            Ok(j) => ("200 OK", json, j),
            Err(e) => ("200 OK", json, err_json(&e)),
        },
        ("GET", "/api/lookup") => match api_lookup(server, query) {
            Ok(j) => ("200 OK", json, j),
            Err(e) => ("200 OK", json, err_json(&e)),
        },
        _ => ("404 Not Found", "text/plain", "not found".into()),
    }
}

fn load_db(server: &EditServer) -> Result<Database, String> {
    let mut db = yamlio::load_database(
        &server.root.join("database"),
        &server.version,
        &server.schema_version,
    )?;
    derive::fill(&mut db)?;
    Ok(db)
}

// ---- JSON emission helpers (no serde) -------------------------------------

fn js(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn err_json(e: &str) -> String {
    format!("{{\"error\":{}}}", js(e))
}

// ---- endpoints -------------------------------------------------------------

fn api_model(server: &EditServer) -> Result<String, String> {
    let db = load_db(server)?;
    let mut pgns: Vec<String> = db
        .pgns
        .iter()
        .map(|p| {
            format!(
                "{{\"pgn\":{},\"id\":{},\"description\":{},\"fallback\":{}}}",
                p.pgn,
                js(&p.id),
                js(&p.description),
                p.fallback
            )
        })
        .collect();
    pgns.sort();
    let mut lookups: Vec<String> = db
        .lookups
        .values()
        .map(|l| {
            format!(
                "{{\"name\":{},\"kind\":{},\"bits\":{}}}",
                js(&l.name),
                js(&l.kind),
                l.bits
            )
        })
        .collect();
    lookups.sort();
    let fieldtypes: Vec<String> = db
        .fieldtypes
        .iter()
        .map(|ft| {
            format!(
                "{{\"name\":{},\"root\":{},\"bits\":{},\"unit\":{},\"resolution\":{}}}",
                js(&ft.name),
                js(&ft.root_name),
                ft.size,
                ft.unit.as_deref().map(js).unwrap_or_else(|| "null".into()),
                if ft.resolution != 0.0 { format!("{:?}", ft.resolution) } else { "null".into() },
            )
        })
        .collect();
    Ok(format!(
        "{{\"pgns\":[{}],\"lookups\":[{}],\"fieldtypes\":[{}]}}",
        pgns.join(","),
        lookups.join(","),
        fieldtypes.join(",")
    ))
}

fn api_pgn(server: &EditServer, query: &str) -> Result<String, String> {
    let id = query
        .split('&')
        .find_map(|kv| kv.strip_prefix("id="))
        .ok_or("missing id=")?;
    let path = find_pgn_file(server, id)?;
    let yaml = std::fs::read_to_string(&path).map_err(|e| e.to_string())?;
    let def = yamlio::parse_pgn_str(&yaml, id)?;
    let fields: Vec<String> = def
        .fields
        .iter()
        .map(|f| {
            let opt_s = |v: &Option<String>| v.as_deref().map(js).unwrap_or_else(|| "null".into());
            format!(
                "{{\"id\":{},\"name\":{},\"type\":{},\"bits\":{},\"lookup\":{},\"lookupBits\":{},\"lookupFieldtype\":{},\"match\":{},\"unit\":{},\"resolution\":{},\"primaryKey\":{}}}",
                js(&f.id),
                js(&f.name),
                js(&f.type_),
                f.bits.map(|b| b.to_string()).unwrap_or_else(|| "null".into()),
                opt_s(&f.lookup),
                opt_s(&f.lookup_bits),
                opt_s(&f.lookup_fieldtype),
                opt_s(&f.match_),
                opt_s(&f.unit),
                f.resolution.map(|r| format!("{r:?}")).unwrap_or_else(|| "null".into()),
                f.primary_key,
            )
        })
        .collect();
    Ok(format!(
        "{{\"path\":{},\"yaml\":{},\"def\":{{\"pgn\":{},\"id\":{},\"description\":{},\"type\":{},\"priority\":{},\"fields\":[{}]}}}}",
        js(&path.display().to_string()),
        js(&yaml),
        def.pgn,
        js(&def.id),
        js(&def.description),
        js(&def.type_),
        def.priority,
        fields.join(",")
    ))
}

/// GET /api/lookup?name=NAME -> the lookup file (yaml + structured values).
fn api_lookup(server: &EditServer, query: &str) -> Result<String, String> {
    let name = query
        .split('&')
        .find_map(|kv| kv.strip_prefix("name="))
        .ok_or("missing name=")?;
    if !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err("bad lookup name".into());
    }
    let path = server.root.join(format!("database/lookups/{name}.yaml"));
    let yaml = std::fs::read_to_string(&path).map_err(|e| format!("{name}: {e}"))?;
    let lk = yamlio::parse_lookup_str(&yaml, name)?;
    let values: Vec<String> = match lk.kind.as_str() {
        "pair" | "bit" => lk
            .pairs
            .iter()
            .map(|(v, n)| format!("{{\"value\":{v},\"name\":{}}}", js(n)))
            .collect(),
        "triplet" => lk
            .triplets
            .iter()
            .map(|(a, b, n)| format!("{{\"value\":\"{a},{b}\",\"name\":{}}}", js(n)))
            .collect(),
        _ => lk
            .fieldtypes
            .iter()
            .map(|e| {
                format!(
                    "{{\"value\":{},\"name\":{},\"type\":{}}}",
                    e.value,
                    js(&e.name),
                    js(&e.fieldtype)
                )
            })
            .collect(),
    };
    Ok(format!(
        "{{\"name\":{},\"kind\":{},\"bits\":{},\"yaml\":{},\"values\":[{}]}}",
        js(&lk.name),
        js(&lk.kind),
        lk.bits,
        js(&yaml),
        values.join(",")
    ))
}

/// Keep database/lookups.order.yaml in step when the editor adds a lookup.
fn sync_lookup_order(server: &EditServer, db: &Database) -> Result<(), String> {
    let mut out = String::new();
    for kind in ["pair", "triplet", "bit", "fieldtype"] {
        out.push_str(kind);
        out.push_str(":\n");
        for name in db.lookup_order.get(kind).into_iter().flatten() {
            out.push_str("- ");
            out.push_str(name);
            out.push('\n');
        }
    }
    std::fs::write(server.root.join("database/lookups.order.yaml"), out).map_err(|e| e.to_string())
}

fn find_pgn_file(server: &EditServer, id: &str) -> Result<PathBuf, String> {
    let dir = server.root.join("database/pgns");
    for entry in std::fs::read_dir(&dir).map_err(|e| e.to_string())?.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.ends_with(&format!("-{id}.yaml")) {
            return Ok(entry.path());
        }
    }
    Err(format!("no file for pgn id '{id}'"))
}

/// POST body: raw sample lines. Response: assembled messages + warnings.
fn api_analyze(server: &EditServer, body: &str) -> Result<String, String> {
    let db = load_db(server)?;
    let mut frames = Vec::new();
    for (n, line) in body.lines().enumerate() {
        if line.trim().is_empty() || line.trim_start().starts_with('#') {
            continue;
        }
        frames.push(samples::parse_line(line).map_err(|e| format!("line {}: {e}", n + 1))?);
    }
    let (assembled, warnings) = samples::reassemble_lenient(&frames, |pgn| {
        db.pgns.iter().any(|q| q.pgn == pgn && q.type_ == "Fast")
    })?;
    let msgs: Vec<String> = assembled
        .iter()
        .map(|a| {
            let variant = decode::select_variant(&db, a.pgn, &a.data, false);
            format!(
                "{{\"pgn\":{},\"prio\":{},\"src\":{},\"dst\":{},\"hex\":{},\"variant\":{}}}",
                a.pgn,
                a.prio,
                a.src,
                a.dst,
                js(&a.data.iter().map(|b| format!("{b:02x}")).collect::<String>()),
                variant.map(|p| js(&p.id)).unwrap_or_else(|| "null".into()),
            )
        })
        .collect();
    let warns: Vec<String> = warnings.iter().map(|w| js(w)).collect();
    Ok(format!(
        "{{\"messages\":[{}],\"warnings\":[{}]}}",
        msgs.join(","),
        warns.join(",")
    ))
}

/// POST /api/decode?data=<hex>[&data=<hex>...] - body: candidate PGN YAML.
/// Decodes every payload against the candidate; also runs the rule engine
/// with the candidate swapped in and returns its violations.
fn api_decode(server: &EditServer, query: &str, body: &str) -> Result<String, String> {
    let payloads: Vec<Vec<u8>> = query
        .split('&')
        .filter_map(|kv| kv.strip_prefix("data="))
        .map(samples::parse_hex)
        .collect::<Result<_, _>>()?;

    let mut db = load_db(server)?;
    let candidate = yamlio::parse_pgn_str(body, "candidate")?;
    let cand_id = candidate.id.clone();
    db.pgns.retain(|p| p.id != cand_id);
    db.pgns.push(candidate);
    db.pgns.sort_by_key(|p| (p.pgn, p.variant_order));
    derive::fill(&mut db)?;

    let violations: Vec<String> = check::check(&db)
        .iter()
        .filter(|v| v.location.contains(&format!("-{cand_id}.yaml")) || v.location.contains("candidate"))
        .map(|v| {
            format!(
                "{{\"rule\":{},\"error\":{},\"message\":{}}}",
                js(v.rule),
                v.error,
                js(&v.message)
            )
        })
        .collect();

    let pgn = db.pgns.iter().find(|p| p.id == cand_id).unwrap();
    let mut sample_results = Vec::new();
    for data in &payloads {
        let fields = match decode::decode(&db, pgn, data) {
            Ok(d) => d,
            Err(e) => {
                sample_results.push(err_json(&e));
                continue;
            }
        };
        let items: Vec<String> = fields
            .iter()
            .map(|d| {
                format!(
                    "{{\"id\":{},\"name\":{},\"instance\":{},\"value\":{}}}",
                    js(&d.id),
                    js(&d.name),
                    d.instance,
                    js(&d.value.to_string())
                )
            })
            .collect();
        sample_results.push(format!("{{\"fields\":[{}]}}", items.join(",")));
    }
    Ok(format!(
        "{{\"violations\":[{}],\"samples\":[{}]}}",
        violations.join(","),
        sample_results.join(",")
    ))
}

/// POST /api/save?file=<name> - body: full YAML document. Validates with the
/// candidate in place; refuses to write when any error-level violation
/// remains anywhere (the edit may affect other files' rules).
fn api_save(server: &EditServer, query: &str, body: &str) -> Result<String, String> {
    let file = query
        .split('&')
        .find_map(|kv| kv.strip_prefix("file="))
        .ok_or("missing file=")?;
    let file = file.replace("%2F", "/");
    if !file.ends_with(".yaml") || file.contains("..") || !file.starts_with("database/") {
        return Err("file must be database/**.yaml".into());
    }
    let mut db = yamlio::load_database(
        &server.root.join("database"),
        &server.version,
        &server.schema_version,
    )?;
    if file.starts_with("database/lookups/") {
        let candidate = yamlio::parse_lookup_str(body, &file)?;
        let expect = format!("database/lookups/{}.yaml", candidate.name);
        if file != expect {
            return Err(format!("lookup '{}' must be saved as {expect}", candidate.name));
        }
        if !db.lookups.contains_key(&candidate.name) {
            // new lookup: emission order manifest must list it
            db.lookup_order
                .entry(candidate.kind.clone())
                .or_default()
                .push(candidate.name.clone());
        }
        db.lookups.insert(candidate.name.clone(), candidate);
    } else {
        let candidate = yamlio::parse_pgn_str(body, &file)?;
        let cand_id = candidate.id.clone();
        db.pgns.retain(|p| p.id != cand_id);
        db.pgns.push(candidate);
        db.pgns.sort_by_key(|p| (p.pgn, p.variant_order));
    }
    derive::fill(&mut db)?;
    let violations = check::check(&db);
    let errors: Vec<&check::Violation> = violations.iter().filter(|v| v.error).collect();
    if !errors.is_empty() {
        let list: Vec<String> = errors
            .iter()
            .map(|v| format!("{} {}: {}", v.rule, v.location, v.message))
            .collect();
        return Err(format!("not saved - {} error(s):\n{}", errors.len(), list.join("\n")));
    }

    let path = server.root.join(&file);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(&path, body).map_err(|e| e.to_string())?;
    if file.starts_with("database/lookups/") {
        sync_lookup_order(server, &db)?;
    }
    Ok(format!("{{\"saved\":{}}}", js(&file)))
}
