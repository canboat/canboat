// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Device-pipeline-specific TCP listeners.
//!
//! The read-only broadcast + snapshot ports are shared with `n2kd` and
//! live in [`crate::n2kd::serving::tcp`]. What stays here needs the device
//! writer and the wire protocol, so only the `server` pipeline uses it:
//!
//! * **Write server** (WO) — accepts PLAIN/FAST lines and injects them
//!   onto the bus via an [`InjectPoint`] (device writer + pipeline
//!   loopback).
//!
//! * **Filter control server** (RW) — the one bidirectional port: it
//!   carries the PGN 262657 NMEA-0183-filter control channel between the
//!   TUI and the pipeline's shared [`NmeaFilter`], request/response and
//!   private per connection (see [`spawn_filter_control_server`]).
//!
//! * **Binary analyzer server** (RO) — the `canboat_wire` handshake
//!   plus length-prefixed postcard `WirePgn` frames from a [`BinHub`].

use std::io::{BufRead, BufReader, Write};
use std::net::{Ipv4Addr, Shutdown, SocketAddrV4, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};

use canboat_core::format::{PlainError, parse_plain};
use canboat_core::output::{JsonOptions, write_json};
use canboat_core::{PgnDatabase, RawFrame};
use canboat_io::device::FrameSender;

use crate::n2kd::nmea_filter::NmeaFilter;
use crate::n2kd::overrides::OverrideEngine;

/// Where a client-written frame goes. In device mode, we forward to
/// the device *and* loop it back into the pipeline source so it
/// appears in NMEA 0183 / snapshot / read-side TCP output, mirroring
/// the stdin-pump behaviour. In stdin mode there's no device and no
/// loopback channel, so writes are dropped on the floor.
#[derive(Clone)]
pub struct InjectPoint {
    pub device: FrameSender,
    pub loopback: mpsc::Sender<RawFrame>,
    /// Live claim address from the underlying device adapter, when
    /// known (today only `--socketcan` exposes it). Used to rewrite a
    /// client-supplied `src == 0` (PC-gateway default) or `src == 255`
    /// (broadcast — never a valid source) to the gateway's address on
    /// the loopback side, so the in-process pipeline sees the same
    /// `src` the rewritten frame will reach the bus with. `None`
    /// disables the rewrite (other device backends; stdin mode).
    pub claim_addr: Option<Arc<AtomicU8>>,
}

use crate::n2kd::serving::BinHub;
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
/// PLAIN/FAST lines that are parsed and injected onto the bus via
/// `inject`. Nothing is streamed back — this is the canonical
/// `SERVER_INPUT_STREAM` slot (canboat C n2kd `port+3`). Unlike the
/// broadcast ports it never subscribes to a hub, so it adds no
/// serialization pressure; that's what makes it the cheap write path
/// for a consumer that reads its data elsewhere (e.g. the binary
/// port). `inject: None` (stdin mode, no device) still accepts and
/// drains client writes, logging them at debug.
pub fn spawn_input_server(
    name: &'static str,
    bind: Ipv4Addr,
    port: u16,
    inject: Option<InjectPoint>,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding {name} TCP port {}:{}", bind, port))?;
    let mode = if inject.is_some() {
        "write-only"
    } else {
        "write-only (writes dropped)"
    };
    log::info!("{name} server listening on {}:{} ({mode})", bind, port);
    Ok(thread::Builder::new()
        .name(format!("{name}-accept"))
        .spawn(move || input_accept(name, listener, inject, stop))
        .expect("spawn input accept"))
}

fn input_accept(
    name: &'static str,
    listener: TcpListener,
    inject: Option<InjectPoint>,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until(name, listener, stop, |stream, peer| {
        log::info!("{name} client connected: {peer}");
        let inj = inject.clone();
        thread::Builder::new()
            .name(format!("{name}-client"))
            .spawn(move || {
                // We never broadcast to this client; FIN our write
                // direction so a client that tries to read sees EOF.
                if let Err(e) = stream.shutdown(Shutdown::Write) {
                    log::debug!("{name}: shutdown(write) failed: {e}");
                }
                run_inbound_reader(name, stream, inj);
            })
            .ok();
    });
}

fn run_inbound_reader(name: &'static str, stream: TcpStream, inject: Option<InjectPoint>) {
    let reader = BufReader::new(stream);
    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => return,
        };
        match &inject {
            // Device wired up: forward the line. Stop when the writer
            // (or pipeline) has gone away.
            Some(i) => {
                if !forward_plain_line(&line, i) {
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

/// Parse one PLAIN/FAST line, send to the device, AND loop it back
/// into the pipeline source so it appears in the pipeline's output
/// streams. Returns `false` when either the device writer or the
/// pipeline has gone away (caller should stop).
fn forward_plain_line(line: &str, inject: &InjectPoint) -> bool {
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
        Ok(mut frame) => {
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
            // Rewrite a default / broadcast `src` to our gateway's
            // live claim address on BOTH paths. The device adapter
            // does the same rewrite internally for the bus side, but
            // the loopback bypasses that — so without this, the
            // in-process pipeline (n2kd snapshot / analyzer-port JSON
            // / NMEA 0183) shows the original `src=0` or `src=255`
            // even though the on-wire frame has the gateway's src.
            // `CLAIM_UNCLAIMED` (254) means we haven't claimed yet —
            // leave src as-is rather than guess.
            if matches!(frame.src, 0 | 255)
                && let Some(claim) = inject.claim_addr.as_deref()
            {
                let live = claim.load(Ordering::Relaxed);
                if live != canboat_io::device::socketcan::CLAIM_UNCLAIMED {
                    frame.src = live;
                }
            }
            if inject.device.send_frame(frame.clone()).is_err() {
                return false;
            }
            inject.loopback.send(frame).is_ok()
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
    let db = PgnDatabase::embedded(canboat_core::Units::Metric);
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
    canboat_core::format_iso_ms(ms)
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
    let db = PgnDatabase::embedded(canboat_core::Units::Metric);
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

/// Bind the binary analyzer server (`--analyzer-binary-port`).
///
/// Strictly read-only. On connect it writes the canboat-wire
/// [`canboat_wire::Hello`] handshake — magic, wire-protocol version, and
/// the [`canboat_core::SCHEMA_HASH`] — then streams length-prefixed
/// postcard `WirePgn` frames from `hub` for the life of the connection.
/// A client MUST read and verify the `Hello` first: the schema hash is
/// what proves both ends share byte-identical field indices, without
/// which the per-field `order` numbers on the wire are meaningless.
pub fn spawn_binary_stream(
    bind: Ipv4Addr,
    port: u16,
    hub: Arc<BinHub>,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding binary TCP port {}:{}", bind, port))?;
    log::info!("binary analyzer server listening on {}:{} (RO)", bind, port);
    Ok(thread::Builder::new()
        .name("binary-accept".into())
        .spawn(move || binary_accept(listener, hub, stop))
        .expect("spawn binary accept"))
}

fn binary_accept(listener: TcpListener, hub: Arc<BinHub>, stop: Option<Arc<AtomicBool>>) {
    accept_until("binary", listener, stop, |stream, peer| {
        log::info!("binary client connected: {peer}");
        let h = hub.clone();
        thread::Builder::new()
            .name("binary-client".into())
            .spawn(move || run_binary_client(stream, h))
            .ok();
    });
}

fn run_binary_client(mut stream: TcpStream, hub: Arc<BinHub>) {
    // Read-only: FIN the read direction so client writes get
    // ECONNRESET / EPIPE rather than piling up unread.
    if let Err(e) = stream.shutdown(Shutdown::Read) {
        log::debug!("binary: shutdown(read) failed: {e}");
    }
    // Handshake first — a client can reject a schema/version mismatch
    // before decoding a single record.
    let mut hello = Vec::with_capacity(64);
    if canboat_wire::append_frame(&mut hello, &canboat_wire::Hello::current()).is_err() {
        return;
    }
    if stream.write_all(&hello).is_err() {
        return;
    }
    // Each `Arc<[u8]>` chunk from the hub is already a run of whole
    // frames (the pipeline's BinBatcher coalesces them), so forward it
    // verbatim. Nagle — left enabled like the text ports — mops up the
    // small writes into full segments.
    let rx = hub.subscribe();
    while let Ok(chunk) = rx.recv() {
        if stream.write_all(&chunk).is_err() {
            break;
        }
    }
    drop(stream);
}

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::output::CamelCase;

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

        let db = PgnDatabase::embedded(canboat_core::Units::Metric);
        let json_opts = JsonOptions {
            include_empty: false,
            name_value: true,
            debug: false,
            camel_case: CamelCase::Off,
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
