// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Device-pipeline-specific TCP listeners.
//!
//! The read-only broadcast + snapshot ports are shared with `n2kd` and
//! live in [`crate::n2kd::serving::tcp`]. What stays here needs the device
//! writer and the wire protocol, so only the `server` pipeline uses it:
//!
//! * **Write server** (WO) — accepts PLAIN/FAST lines and hands them to
//!   the device writer. Nothing is echoed back into the pipeline: the
//!   writer already knows what it sent, and a consumer that both reads
//!   the bus and writes to it (Signal K) would otherwise see its own
//!   output come back as bus data and loop. This matches the SocketCAN
//!   adapter's own policy for user-initiated sends.
//!
//! * **Filter control server** (RW) — the one bidirectional port: it
//!   carries the PGN 262657 NMEA-0183-filter control channel between the
//!   TUI and the pipeline's shared [`NmeaFilter`], request/response and
//!   private per connection (see [`spawn_filter_control_server`]).

use std::io::{BufRead, BufReader, Write};
use std::net::{Ipv4Addr, Shutdown, SocketAddrV4, TcpListener, TcpStream};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};

use crate::engine::PgnDatabase;
use crate::engine::format::{PlainError, parse_plain};
use crate::engine::output::{JsonOptions, write_json};
use crate::io::device::FrameSender;

use crate::n2kd::nmea_filter::NmeaFilter;
use crate::n2kd::overrides::OverrideEngine;

use crate::n2kd::serving::tcp::accept_until;

// Nagle's algorithm is deliberately left ENABLED on every client
// socket (no `set_nodelay`). These are telemetry streams, not
// request/response protocols: at bus rates (hundreds of lines/s)
// the kernel coalescing consecutive small writes into full segments
// is pure win — fewer packets, fewer syscall-sized wakeups on the
// receiver — and the added latency is bounded by the ACK round-trip
// (sub-millisecond on loopback / LAN). The write loops below batch
// at the application level too; Nagle mops up whatever still goes
// out small.

/// Bind a **write-only** TCP input server: clients connect and write
/// PLAIN/FAST lines that are parsed and handed to `device`. Nothing
/// is streamed back — this is the canonical `SERVER_INPUT_STREAM`
/// slot (canboat C n2kd `port+3`). Unlike the broadcast ports it
/// never subscribes to a hub, so it adds no serialization pressure;
/// that's what makes it the cheap write path for a consumer that
/// reads its data elsewhere (e.g. the binary port). Injected frames
/// are not looped back into the pipeline either: they reach the
/// output ports only if the bus (or the gateway) returns them, which
/// SocketCAN, the NGT-1 and the iKonvert do not. `device: None`
/// (stdin mode, no device) still accepts and drains client writes,
/// logging them at debug.
pub fn spawn_input_server(
    name: &'static str,
    bind: Ipv4Addr,
    port: u16,
    device: Option<FrameSender>,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding {name} TCP port {}:{}", bind, port))?;
    let mode = if device.is_some() {
        "write-only"
    } else {
        "write-only (writes dropped)"
    };
    log::info!("{name} server listening on {}:{} ({mode})", bind, port);
    Ok(thread::Builder::new()
        .name(format!("{name}-accept"))
        .spawn(move || input_accept(name, listener, device, stop))
        .expect("spawn input accept"))
}

fn input_accept(
    name: &'static str,
    listener: TcpListener,
    device: Option<FrameSender>,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until(name, listener, stop, |stream, peer| {
        log::info!("{name} client connected: {peer}");
        let dev = device.clone();
        thread::Builder::new()
            .name(format!("{name}-client"))
            .spawn(move || {
                // We never broadcast to this client; FIN our write
                // direction so a client that tries to read sees EOF.
                if let Err(e) = stream.shutdown(Shutdown::Write) {
                    log::debug!("{name}: shutdown(write) failed: {e}");
                }
                run_inbound_reader(name, stream, dev);
            })
            .ok();
    });
}

fn run_inbound_reader(name: &'static str, stream: TcpStream, device: Option<FrameSender>) {
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => return,
        };
        match &device {
            // Device wired up: forward the line. Stop when the writer
            // has gone away.
            Some(d) => {
                if !forward_plain_line(&line, d) {
                    return;
                }
            }
            // Stdin-mode pipeline: no device to forward to. Drain the
            // line so the client keeps making progress, but log at
            // debug so a stray write doesn't disappear without a
            // trace.
            None => {
                let trimmed = line.trim_end_matches(['\r', '\n']);
                if !trimmed.is_empty() {
                    log::debug!("{name}: no device wired up, dropping client line: {trimmed}");
                }
            }
        }
    }
}

/// Parse one PLAIN/FAST line and send it to the device. A `src` of 0
/// or 255 is left for the device adapter to stamp with its live claim
/// address (SocketCAN does; the serial gateways send it as given).
/// Returns `false` when the device writer has gone away (caller
/// should stop).
fn forward_plain_line(line: &str, device: &FrameSender) -> bool {
    let trimmed = line.trim_end_matches(['\r', '\n']);
    if trimmed.is_empty() || trimmed.starts_with('#') {
        return true;
    }
    // Silently skip iKonvert-style control / data sentences that
    // sometimes leak in from clients that were originally talking
    // to a Digital Yacht iKonvert (`$PDGY,N2NET_OFFLINE`,
    // `!PDGY,…`). They're not PLAIN/FAST. We could route them
    // through to an iKonvert-typed device but that's a feature for
    // another day; for now they're not "malformed", just not for
    // us.
    if trimmed.starts_with('$') || trimmed.starts_with('!') {
        log::debug!("skipping non-PLAIN/FAST line: {trimmed}");
        return true;
    }
    match parse_plain(trimmed) {
        Ok(frame) => {
            // The NMEA 0183 filter control PGN is never a bus frame and
            // no longer travels this port: it has its own dedicated
            // bidirectional control port (see
            // [`spawn_filter_control_server`]). One arriving here is
            // stale / misrouted — drop it silently rather than inject a
            // synthetic PGN onto the N2K bus.
            if frame.pgn == crate::n2kd::nmea_filter::PGN_NMEA0183_FILTER
                || frame.pgn == crate::n2kd::overrides::PGN_PGN_OVERRIDE
            {
                log::debug!(
                    "dropping stray control PGN {} on input port; control channels have dedicated ports",
                    frame.pgn
                );
                return true;
            }
            device.send_frame(frame).is_ok()
        }
        Err(PlainError::Empty) => true,
        Err(e) => {
            log_bad_plain_line(trimmed, &e);
            true
        }
    }
}

/// Render the malformed-line warning with the offending line and a
/// caret pointing at the byte offset where parsing failed. Helps
/// the user see *what* came in over the wire that the parser
/// didn't like.
fn log_bad_plain_line(line: &str, err: &PlainError) {
    match err.byte_offset() {
        Some(offset) => {
            // Cap the offset at the line length so we always render
            // a sane pointer even if the source counted past EOL.
            let pointer_col = offset.min(line.len());
            let pointer = format!("{:width$}^", "", width = pointer_col);
            log::warn!(
                "ignoring malformed PLAIN/FAST line: {err}\n  line:    {line}\n  pointer: {pointer}"
            );
        }
        None => {
            log::warn!("ignoring malformed PLAIN/FAST line: {err}\n  line: {line}");
        }
    }
}

/// Bind the **bidirectional** NMEA 0183 filter control port
/// (`--nmea0183-filter-port`).
///
/// This is the only write-capable *output* port, and it carries exactly
/// one thing: the PGN 262657 control channel between the TUI and the
/// pipeline's shared [`NmeaFilter`]. The exchange is request/response
/// and private to each connection — no filter frame ever reaches the
/// read-only analyzer stream or the N2K bus (that's the "confusing for
/// other consumers" problem the old broadcast had). On connect the
/// server pushes the current state once; thereafter each client `Set`
/// (mutate a rule) or `Request` (re-poll) is answered on the same
/// socket with a fresh batch of `Report` lines. Reports are analyzer
/// JSON — the exact shape the TUI already parses off the stream port —
/// and each carries a real timestamp.
pub fn spawn_filter_control_server(
    bind: Ipv4Addr,
    port: u16,
    filter: Arc<Mutex<NmeaFilter>>,
    json_opts: JsonOptions,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding filter-control TCP port {}:{}", bind, port))?;
    log::info!(
        "nmea0183-filter control server listening on {}:{} (RW)",
        bind,
        port
    );
    Ok(thread::Builder::new()
        .name("filter-control-accept".into())
        .spawn(move || filter_control_accept(listener, filter, json_opts, stop))
        .expect("spawn filter-control accept"))
}

fn filter_control_accept(
    listener: TcpListener,
    filter: Arc<Mutex<NmeaFilter>>,
    json_opts: JsonOptions,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until("filter-control", listener, stop, |stream, peer| {
        log::info!("filter-control client connected: {peer}");
        let f = filter.clone();
        let opts = json_opts.clone();
        thread::Builder::new()
            .name("filter-control-client".into())
            .spawn(move || run_filter_control_client(stream, f, opts))
            .ok();
    });
}

fn run_filter_control_client(
    stream: TcpStream,
    filter: Arc<Mutex<NmeaFilter>>,
    json_opts: JsonOptions,
) {
    // Single-frame synthetic PGN 262657 decodes against the embedded
    // schema with no reassembly; Metric vs SI is irrelevant (its fields
    // are plain integers / a fixed string).
    let db = PgnDatabase::embedded(crate::engine::Units::Metric);
    let mut write_stream = match stream.try_clone() {
        Ok(s) => s,
        Err(e) => {
            log::debug!("filter-control: try_clone failed: {e}");
            return;
        }
    };
    // Unprompted initial report so a freshly-connected TUI paints the
    // current state without having to ask for it.
    if !send_filter_report(&mut write_stream, &filter, db, &json_opts) {
        return;
    }
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => return,
        };
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.is_empty() {
            continue;
        }
        let frame = match parse_plain(trimmed) {
            Ok(f) => f,
            Err(PlainError::Empty) => continue,
            Err(e) => {
                log::debug!("filter-control: ignoring non-PLAIN line: {e}");
                continue;
            }
        };
        if crate::n2kd::nmea_filter::is_set_frame(&frame) {
            filter.lock().unwrap().apply_set_frame(&frame.data);
        } else if !crate::n2kd::nmea_filter::is_request_frame(&frame) {
            // Neither Set nor Request. This port never injects onto the
            // bus, so anything else is simply ignored.
            log::debug!("filter-control: ignoring frame pgn {}", frame.pgn);
            continue;
        }
        // Both Set and Request are answered with the full current state.
        if !send_filter_report(&mut write_stream, &filter, db, &json_opts) {
            return;
        }
    }
}

/// Serialize the current filter state as analyzer-JSON `Report` lines
/// (one per `(src, sentence)` row) and write them to `stream`. Returns
/// `false` only on a socket write error, so the caller drops the
/// client. An empty state (nothing observed yet) writes nothing and
/// keeps the connection open — the client re-`Request`s periodically.
fn send_filter_report(
    stream: &mut TcpStream,
    filter: &Mutex<NmeaFilter>,
    db: &PgnDatabase,
    json_opts: &JsonOptions,
) -> bool {
    let out = filter_report_lines(filter, db, json_opts);
    if out.is_empty() {
        return true;
    }
    stream.write_all(out.as_bytes()).is_ok()
}

/// Build the newline-terminated analyzer-JSON `Report` lines for the
/// current filter state (one per `(src, sentence)` row). Each row is the
/// 262657 frame decoded against `db` and serialized with `json_opts`, so
/// the shape is byte-identical to what the TUI parses off the stream
/// port. Empty when nothing has been observed yet.
fn filter_report_lines(
    filter: &Mutex<NmeaFilter>,
    db: &PgnDatabase,
    json_opts: &JsonOptions,
) -> String {
    let frames = filter.lock().unwrap().report_frames(Some(now_iso()));
    let mut out = String::with_capacity(frames.len() * 96);
    for frame in &frames {
        match db.decode(frame) {
            Ok(decoded) => {
                let start = out.len();
                if write_json(&mut out, &decoded, json_opts).is_ok() {
                    out.push('\n');
                } else {
                    out.truncate(start);
                }
            }
            Err(e) => log::debug!("filter-control: decoding a 262657 report failed: {e}"),
        }
    }
    out
}

/// Current wall-clock as an ISO-8601 millisecond timestamp, matching
/// the `timestamp` field on every other analyzer record. Falls back to
/// the epoch if the clock is somehow before 1970.
fn now_iso() -> String {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    crate::engine::format_iso_ms(ms)
}

/// Bind the bidirectional PGN 262658 override control port. Mirrors
/// [`spawn_filter_control_server`]: a connected client is sent the
/// current override state, then writes `Set` / `Delete` / `Request`
/// frames and reads `Report` frames on the same socket. A `Set` is
/// applied immediately by injecting the resulting PGN 126208 Request
/// through `device_sender` (the wire path shared with the pipeline).
pub fn spawn_overrides_control_server(
    bind: Ipv4Addr,
    port: u16,
    engine: Arc<Mutex<OverrideEngine>>,
    device_sender: Option<FrameSender>,
    json_opts: JsonOptions,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding overrides-control TCP port {}:{}", bind, port))?;
    log::info!(
        "overrides control server listening on {}:{} (RW)",
        bind,
        port
    );
    Ok(thread::Builder::new()
        .name("overrides-control-accept".into())
        .spawn(move || overrides_control_accept(listener, engine, device_sender, json_opts, stop))
        .expect("spawn overrides-control accept"))
}

fn overrides_control_accept(
    listener: TcpListener,
    engine: Arc<Mutex<OverrideEngine>>,
    device_sender: Option<FrameSender>,
    json_opts: JsonOptions,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until("overrides-control", listener, stop, |stream, peer| {
        log::info!("overrides-control client connected: {peer}");
        let e = engine.clone();
        let sender = device_sender.clone();
        let opts = json_opts.clone();
        thread::Builder::new()
            .name("overrides-control-client".into())
            .spawn(move || run_overrides_control_client(stream, e, sender, opts))
            .ok();
    });
}

fn run_overrides_control_client(
    stream: TcpStream,
    engine: Arc<Mutex<OverrideEngine>>,
    device_sender: Option<FrameSender>,
    json_opts: JsonOptions,
) {
    let db = PgnDatabase::embedded(crate::engine::Units::Metric);
    let mut write_stream = match stream.try_clone() {
        Ok(s) => s,
        Err(e) => {
            log::debug!("overrides-control: try_clone failed: {e}");
            return;
        }
    };
    if !send_override_report(&mut write_stream, &engine, db, &json_opts) {
        return;
    }
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => return,
        };
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.is_empty() {
            continue;
        }
        let frame = match parse_plain(trimmed) {
            Ok(f) => f,
            Err(PlainError::Empty) => continue,
            Err(e) => {
                log::debug!("overrides-control: ignoring non-PLAIN line: {e}");
                continue;
            }
        };
        if crate::n2kd::overrides::is_set_frame(&frame) {
            // Persist + apply now: inject the PGN 126208 Request onto the
            // bus (when there's a device writer).
            let request = engine.lock().unwrap().apply_set_frame(&frame.data);
            if let (Some(req), Some(sender)) = (request, device_sender.as_ref()) {
                let _ = sender.send_frame(req);
            }
        } else if crate::n2kd::overrides::is_delete_frame(&frame) {
            engine.lock().unwrap().apply_delete_frame(&frame.data);
        } else if !crate::n2kd::overrides::is_request_frame(&frame) {
            log::debug!("overrides-control: ignoring frame pgn {}", frame.pgn);
            continue;
        }
        if !send_override_report(&mut write_stream, &engine, db, &json_opts) {
            return;
        }
    }
}

/// Serialize the current override state as analyzer-JSON `Report` lines
/// and write them. `false` only on a socket write error.
fn send_override_report(
    stream: &mut TcpStream,
    engine: &Mutex<OverrideEngine>,
    db: &PgnDatabase,
    json_opts: &JsonOptions,
) -> bool {
    let frames = engine.lock().unwrap().report_frames(Some(now_iso()));
    let mut out = String::with_capacity(frames.len() * 96);
    for frame in &frames {
        match db.decode(frame) {
            Ok(decoded) => {
                let start = out.len();
                if write_json(&mut out, &decoded, json_opts).is_ok() {
                    out.push('\n');
                } else {
                    out.truncate(start);
                }
            }
            Err(e) => log::debug!("overrides-control: decoding a 262658 report failed: {e}"),
        }
    }
    if out.is_empty() {
        return true;
    }
    stream.write_all(out.as_bytes()).is_ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::output::CamelCase;

    #[test]
    fn filter_report_lines_render_nv_json_with_timestamp() {
        // Build a filter, teach it a device NAME, observe a VHW sentence,
        // then mute it via a Set. The lines the control port sends must
        // decode to the -nv analyzer JSON the TUI parses, each carrying a
        // timestamp (the fix for the old timestamp-less broadcast).
        let path = std::env::temp_dir().join("canboat-filter-control-test.json");
        let mut filter = NmeaFilter::load(&path).unwrap();
        filter.note_address_claim(35, 0x0004_0000_0761_9208);
        let mut buf = "$CDVHW,,T,,M,4.6,N,8.6,K*5E\r\n".to_string();
        filter.apply(35, &mut buf);
        assert!(filter.set(35, "VHW", true));

        let db = PgnDatabase::embedded(crate::engine::Units::Metric);
        let json_opts = JsonOptions {
            include_empty: false,
            name_value: true,
            debug: false,
            camel_case: CamelCase::Off,
            wrap: false,
        };
        let filter = Mutex::new(filter);
        let out = filter_report_lines(&filter, db, &json_opts);

        assert!(out.contains("262657"), "out: {out}");
        assert!(
            out.contains("\"timestamp\""),
            "every report line must carry a timestamp: {out}"
        );
        assert!(
            out.lines()
                .any(|l| l.contains("\"Sentence\":\"VHW\"") && l.contains("\"Muted\":1")),
            "expected a muted VHW row: {out}"
        );
        assert!(
            out.contains("\"Sentence\":\"ALL\""),
            "expected a whole-source ALL row: {out}"
        );
        let _ = std::fs::remove_file(&path);
    }
}
