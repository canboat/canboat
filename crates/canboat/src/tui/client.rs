// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! TCP client to either `n2kd` or `canboat-pipeline`.
//!
//! On startup the client opens the snapshot port (default 2597),
//! drains the one-shot JSON blob, and seeds [`crate::tui::state::AppState`]
//! with everything currently in the cache. Then it opens the
//! analyzer/stream port (default 2598) and stays subscribed for the
//! lifetime of the process — each incoming line goes through
//! [`canboat_core::snapshot::classify_json_line`] so the keys we
//! produce are byte-identical to the ones the snapshot port stores.
//!
//! The stream port is read-only. Outgoing traffic goes to three other
//! ports, all derived from the stream port's canonical offsets (see
//! `super::write_ports`) and fed by the same [`Writer`], which routes
//! each PLAIN line by PGN:
//!
//! * the **write-only input port** (`base+3`, default 2600) — ISO
//!   Requests, injected onto the N2K bus;
//! * the **bidirectional filter control port** (`base+8`, default 2605)
//!   — the PGN 262657 NMEA-0183-filter channel: `Set` / `Request` out,
//!   `Report` JSON back on the same socket;
//! * the **bidirectional overrides control port** (`base+9`, default
//!   2606) — the PGN 262658 transmission-interval-override channel:
//!   `Set` / `Delete` / `Request` out, `Report` JSON back. The server
//!   owns the overrides (persists them, replays them, forgets NAKed
//!   ones) and injects the actual PGN 126208 Requests onto the bus.
//!
//! Against an endpoint that doesn't offer those ports (e.g. a plain
//! n2kd), the respective connection simply fails quietly and that
//! feature is inert — the read-only stream still works.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, Result};
use serde_json::Value;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::net::tcp::OwnedWriteHalf;
use tokio::sync::{Mutex, mpsc};
use tokio::task::JoinHandle;
use tokio::time::timeout;

use crate::tui::state::{AppState, Progress};

/// Per-connection timeout for the initial TCP connect. Without this
/// the TUI sits black forever when the endpoint isn't reachable (the
/// OS default connect timeout is ~75 s on Linux, longer on macOS).
/// 10 s is a tight upper bound that still survives a slow link and a
/// busy peer.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);

/// Cap on the snapshot read. canboat-pipeline / n2kd both write the
/// whole dump and close, so this only fires when the peer is
/// pathological (open socket, no FIN). Generous so a large cache
/// fits.
const SNAPSHOT_READ_TIMEOUT: Duration = Duration::from_secs(15);

/// Handle to the TUI's two outgoing connections. The UI pushes canboat
/// PLAIN lines (without the trailing newline) via [`Writer::send`],
/// which routes each by PGN to the right socket:
///
/// * the **filter control PGN (262657)** → the bidirectional control
///   port (`--nmea0183-filter-port`), where the server answers with
///   `Report` frames on the same socket;
/// * the **override control PGN (262658)** → the bidirectional override
///   control port (`--overrides-port`), likewise request/response;
/// * **everything else** (ISO requests) → the write-only **input port**
///   (`--input-port`), which injects them onto the N2K bus.
///
/// All channels are created upfront — sends queue here until the
/// respective connection task comes up, so the UI is usable from the
/// first frame. In log / idle mode no receiver is drained.
#[derive(Clone)]
pub struct Writer {
    /// Bus injection (ISO requests) → the input port.
    input_tx: mpsc::UnboundedSender<String>,
    /// PGN 262657 filter control → the bidirectional control port.
    filter_tx: mpsc::UnboundedSender<String>,
    /// PGN 262658 override control → the bidirectional control port.
    overrides_tx: mpsc::UnboundedSender<String>,
}

impl Writer {
    /// Queue a PLAIN line for transmission, routed by PGN. Returns
    /// `false` if the destination channel's connection task has gone.
    pub fn send(&self, line: String) -> bool {
        if line_is_filter_control(&line) {
            self.filter_tx.send(line).is_ok()
        } else if line_is_overrides_control(&line) {
            self.overrides_tx.send(line).is_ok()
        } else {
            self.input_tx.send(line).is_ok()
        }
    }
}

/// The receivers a live-mode setup wires to its connection tasks:
/// bus-injection, filter-control, and override-control lines.
pub struct WriterRx {
    pub input: mpsc::UnboundedReceiver<String>,
    pub filter: mpsc::UnboundedReceiver<String>,
    pub overrides: mpsc::UnboundedReceiver<String>,
}

/// The PGN of a canboat PLAIN line — its third comma-separated field
/// (`<ts>,<prio>,<pgn>,…`). `None` when the line can't be parsed.
fn line_pgn(line: &str) -> Option<u32> {
    line.split(',')
        .nth(2)
        .and_then(|s| s.trim().parse::<u32>().ok())
}

/// `true` when a PLAIN line targets the NMEA 0183 filter control PGN
/// (262657). A line we can't parse falls through to the bus path — the
/// safe default, since the input port simply ignores a non-frame.
fn line_is_filter_control(line: &str) -> bool {
    line_pgn(line) == Some(canboat_bridge::n2kd::nmea_filter::PGN_NMEA0183_FILTER)
}

/// `true` when a PLAIN line targets the override control PGN (262658).
fn line_is_overrides_control(line: &str) -> bool {
    line_pgn(line) == Some(canboat_bridge::n2kd::overrides::PGN_PGN_OVERRIDE)
}

/// Build the writer channels and return the shared [`Writer`] plus the
/// receivers for the connection tasks to drain once they connect.
pub fn make_writer() -> (Writer, WriterRx) {
    let (input_tx, input) = mpsc::unbounded_channel();
    let (filter_tx, filter) = mpsc::unbounded_channel();
    let (overrides_tx, overrides) = mpsc::unbounded_channel();
    (
        Writer {
            input_tx,
            filter_tx,
            overrides_tx,
        },
        WriterRx {
            input,
            filter,
            overrides,
        },
    )
}

/// One message from the log-decode thread to the apply task. A `Frame`
/// carries the coalesced PLAIN/FAST line for a whole decoded frame
/// (once, for the Raw-export buffer); the following `Record`(s) are the
/// per-classifier splits of that same frame applied to the model.
enum LoadItem {
    Frame(String),
    Record(canboat_core::snapshot::SnapshotInput),
}

/// Does the file look like analyzer JSON output (our own "Analysed"
/// export) rather than a raw wire capture? Peeks the first non-blank,
/// non-`#` line and checks for a leading `{`.
fn looks_like_json_capture(path: &Path) -> bool {
    use std::io::BufRead;
    let Ok(file) = std::fs::File::open(path) else {
        return false;
    };
    for line in std::io::BufReader::new(file).lines().map_while(Result::ok) {
        let t = line.trim_start();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        return t.starts_with('{');
    }
    false
}

/// Ingest a raw wire capture (PLAIN/FAST/Actisense/…) via the analyzer
/// decode pipeline, emitting a `Frame` (coalesced bytes, for Raw
/// re-export) plus the classifier `Record`s for each decoded message.
fn ingest_raw_capture(path: &Path, tx: &mpsc::UnboundedSender<LoadItem>) -> anyhow::Result<()> {
    use canboat_core::format::write_plain;
    use canboat_core::frame::RawFrame;
    use canboat_core::output::{JsonOptions, write_json};
    use canboat_core::snapshot::classify_json_line;
    // The TUI is the human-facing surface: it shows spaced field names
    // and humanized units, so it pins both rather than following the
    // machine-facing library defaults (camelCase + SI).
    let cfg = canboat_io::analyze::Config {
        units: canboat_core::Units::Metric,
        ..Default::default()
    };
    let json_opts = JsonOptions {
        camel_case: canboat_core::output::CamelCase::Off,
        ..Default::default()
    };
    let mut buf = String::with_capacity(512);
    let mut raw = String::with_capacity(128);
    canboat_io::analyze::decode_file(path, &cfg, |decoded| {
        // Re-emit the wire bytes as one coalesced PLAIN/FAST line so a
        // later "Save ▸ Raw" can round-trip the capture. Sent once per
        // frame, ahead of its classifier splits.
        raw.clear();
        let frame = RawFrame::new(
            decoded.timestamp.clone(),
            decoded.prio,
            decoded.pgn,
            decoded.src,
            decoded.dst,
            decoded.data.iter().copied(),
        );
        if write_plain(&mut raw, &frame).is_ok() {
            let _ = tx.send(LoadItem::Frame(raw.clone()));
        }
        buf.clear();
        // Re-render to JSON so the per-line classifier path here is
        // identical to the live-stream one — same composite keys, same
        // per-iteration splits for repeating-PK PGNs.
        if write_json(&mut buf, decoded, &json_opts).is_err() {
            return;
        }
        classify_json_line(&buf, |input| {
            let _ = tx.send(LoadItem::Record(input));
        });
    })?;
    Ok(())
}

/// Ingest an analyzer-JSON capture (our "Analysed" export): one JSON
/// record per line, applied through the same `classify_json_line` path
/// as the live stream. There are no wire bytes to re-emit, so no
/// `Frame`s are sent (a later Raw save of a JSON-loaded capture is
/// empty, same as live mode).
fn ingest_json_capture(path: &Path, tx: &mpsc::UnboundedSender<LoadItem>) -> anyhow::Result<()> {
    use canboat_core::snapshot::classify_json_line;
    use std::io::BufRead;
    let file = std::fs::File::open(path).with_context(|| format!("opening {}", path.display()))?;
    for line in std::io::BufReader::new(file).lines() {
        let line = line.context("reading JSON capture")?;
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        classify_json_line(t, |input| {
            let _ = tx.send(LoadItem::Record(input));
        });
    }
    Ok(())
}

/// Spawn the background log-replay task. Sniffs `path` for analyzer
/// JSON (our own "Analysed" export) vs a raw wire capture and ingests
/// it accordingly in a blocking thread, shipping records through an
/// unbounded channel to a sibling async apply task. Errors land in
/// `AppState::status.last_error` so the fatal-error modal surfaces them.
pub fn spawn_log_load(path: PathBuf, state: Arc<Mutex<AppState>>) -> Vec<JoinHandle<()>> {
    let (tx, mut rx) = mpsc::unbounded_channel::<LoadItem>();

    // CPU-bound: run the decode in a blocking thread so the tokio
    // reactor isn't starved on big captures.
    let path_for_blocker = path.clone();
    let state_for_err = state.clone();
    let decode = tokio::task::spawn_blocking(move || {
        let result = if canboat_io::container::is_container(&path_for_blocker) {
            // Binary `.pcap` / `.pcap.gz` / `.nif` — unwrapped to PLAIN
            // by the analyzer's container-aware `decode_file`.
            ingest_raw_capture(&path_for_blocker, &tx)
        } else if looks_like_json_capture(&path_for_blocker) {
            ingest_json_capture(&path_for_blocker, &tx)
        } else {
            ingest_raw_capture(&path_for_blocker, &tx)
        };
        if let Err(e) = result {
            // Lock briefly to surface the read / parse failure to the UI.
            tokio::runtime::Handle::current().block_on(async {
                let mut s = state_for_err.lock().await;
                s.status.last_error = Some(format!("log: {e:#}"));
            });
        }
    });

    // Drain the channel on the tokio runtime — applies one record at
    // a time, so a long-running ingest doesn't lock the UI out.
    let name = path
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string());
    let apply = tokio::spawn(async move {
        let mut count = 0usize;
        while let Some(item) = rx.recv().await {
            match item {
                LoadItem::Frame(line) => {
                    let mut s = state.lock().await;
                    s.raw_lines.push(line);
                }
                LoadItem::Record(input) => {
                    let value: Value = match serde_json::from_str(&input.line) {
                        Ok(v) => v,
                        Err(_) => continue,
                    };
                    let mut s = state.lock().await;
                    s.upsert(
                        input.pgn,
                        input.src,
                        input.secondary,
                        input.pgn_description,
                        value,
                    );
                    count += 1;
                }
            }
        }
        // Channel closed → analyzer pipeline finished. Confirm to the
        // user that the (asynchronous) load is done.
        let mut s = state.lock().await;
        s.status.snapshot_loaded = true;
        s.notice = Some(format!("✓ Loaded {count} records — {name}"));
    });
    vec![decode, apply]
}

/// Output format for a capture save.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SaveFormat {
    /// Decoded canboat analyzer JSON, one object per line (`.json`).
    Analysed,
    /// Coalesced canboat PLAIN/FAST text with a `# format=FAST` header
    /// (`.raw`) — re-loadable via File ▸ Load. Only carries frames
    /// captured in log-replay mode; the live JSON stream has no wire
    /// bytes to re-emit.
    Raw,
}

impl SaveFormat {
    /// File extension for this format (no dot).
    pub fn extension(self) -> &'static str {
        match self {
            SaveFormat::Analysed => "json",
            SaveFormat::Raw => "raw",
        }
    }
}

/// Write the current capture to `path` in `format`. Returns the number
/// of records / frames written.
///
/// * [`SaveFormat::Analysed`] serialises the observation history as one
///   JSON object per line.
/// * [`SaveFormat::Raw`] writes a `# format=FAST` header followed by the
///   coalesced PLAIN/FAST line for every captured frame (empty in live
///   mode, where no wire bytes are retained).
///
/// The interactive path uses [`spawn_save`] (chunked, off the UI lock);
/// this single-shot writer backs the format-correctness tests, which
/// need a synchronous result to assert against.
#[cfg(test)]
pub fn save_capture(path: &Path, state: &AppState, format: SaveFormat) -> Result<usize> {
    use std::fs::File;
    use std::io::{BufWriter, Write};
    let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
    let mut w = BufWriter::new(file);
    let mut n = 0usize;
    match format {
        SaveFormat::Analysed => {
            for rec in &state.history {
                serde_json::to_writer(&mut w, &rec.line).context("serialising record")?;
                w.write_all(b"\n").context("writing capture")?;
                n += 1;
            }
        }
        SaveFormat::Raw => {
            // The coalesced-payload header canboat's reader recognises.
            w.write_all(b"# format=FAST\n").context("writing header")?;
            for line in &state.raw_lines {
                w.write_all(line.as_bytes()).context("writing frame")?;
                w.write_all(b"\n").context("writing frame")?;
                n += 1;
            }
        }
    }
    w.flush().context("flushing capture")?;
    Ok(n)
}

/// Spawn a background capture save. A big (700 MB+) export must not run
/// under the state lock — that would freeze the UI for the whole write —
/// so this writes in chunks, releasing the lock between them and
/// reporting progress via [`AppState::save_progress`]. `history` /
/// `raw_lines` are append-only, so snapshotting the length up front and
/// only writing indices `0..total` is race-free even in live mode; the
/// bounds are re-checked with `get` in case Load/Connect replaced the
/// state mid-write (both are disabled while a save runs).
pub fn spawn_save(
    path: PathBuf,
    format: SaveFormat,
    state: Arc<Mutex<AppState>>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        if let Err(e) = run_save(&path, format, &state).await {
            let mut s = state.lock().await;
            s.save_progress = None;
            s.status.last_error = Some(format!("save: {e:#}"));
        }
    })
}

async fn run_save(path: &Path, format: SaveFormat, state: &Arc<Mutex<AppState>>) -> Result<()> {
    use tokio::io::BufWriter;

    let name = path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("capture")
        .to_string();
    let total = {
        let mut s = state.lock().await;
        let total = match format {
            SaveFormat::Analysed => s.history.len(),
            SaveFormat::Raw => s.raw_lines.len(),
        };
        s.save_progress = Some(Progress {
            label: format!("Saving {name}"),
            done: 0,
            total,
        });
        total
    };

    let file = tokio::fs::File::create(path)
        .await
        .with_context(|| format!("creating {}", path.display()))?;
    let mut w = BufWriter::new(file);
    if matches!(format, SaveFormat::Raw) {
        w.write_all(b"# format=FAST\n")
            .await
            .context("writing header")?;
    }

    // Serialise a slice of records under the lock, then release it while
    // the (potentially slow) disk write runs.
    const CHUNK: usize = 2000;
    let mut done = 0usize;
    let mut buf = String::with_capacity(64 * 1024);
    while done < total {
        let end = (done + CHUNK).min(total);
        buf.clear();
        {
            let s = state.lock().await;
            match format {
                SaveFormat::Analysed => {
                    for i in done..end {
                        let Some(rec) = s.history.get(i) else { break };
                        buf.push_str(&serde_json::to_string(&rec.line).unwrap_or_default());
                        buf.push('\n');
                    }
                }
                SaveFormat::Raw => {
                    for i in done..end {
                        let Some(line) = s.raw_lines.get(i) else {
                            break;
                        };
                        buf.push_str(line);
                        buf.push('\n');
                    }
                }
            }
        }
        w.write_all(buf.as_bytes())
            .await
            .context("writing capture")?;
        done = end;
        {
            let mut s = state.lock().await;
            if let Some(p) = &mut s.save_progress {
                p.done = done;
            }
        }
        // Yield so the UI can grab the lock and repaint the bar.
        tokio::task::yield_now().await;
    }
    w.flush().await.context("flushing capture")?;

    let mut s = state.lock().await;
    s.save_progress = None;
    s.notice = Some(if done == 0 && format == SaveFormat::Raw {
        "Saved 0 raw frames — live JSON has no wire bytes; load a capture to export raw".to_string()
    } else {
        format!("✓ Saved {done} records → {}", path.display())
    });
    Ok(())
}

/// Spawn the background snapshot-load task. Failures are surfaced
/// via [`AppState::status`] (`snapshot_loaded` + `last_error`); the
/// caller is not blocked.
pub fn spawn_snapshot_load(host: String, port: u16, state: Arc<Mutex<AppState>>) -> JoinHandle<()> {
    tokio::spawn(async move {
        if let Err(e) = load_snapshot(&host, port, state.clone()).await {
            let mut s = state.lock().await;
            s.status.last_error = Some(format!("snapshot: {e:#}"));
        }
    })
}

/// Spawn the background stream-connection task — connects to the
/// live-JSON port, then runs the reader (decoding into `state`) and
/// writer (draining `rx` to the socket) until the connection
/// drops. Failures are surfaced via [`AppState::status`].
pub fn spawn_stream_connection(
    host: String,
    port: u16,
    state: Arc<Mutex<AppState>>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        if let Err(e) = connect_stream(&host, port, state.clone()).await {
            let mut s = state.lock().await;
            s.status.last_error = Some(format!("stream: {e:#}"));
        }
    })
}

/// Spawn the write-only input-port connection: drains bus-injection
/// lines (ISO requests / PGN 126208 rate overrides) to the socket,
/// which the server injects onto the N2K bus. The input port streams
/// nothing back, so the task lives until a write fails (server gone) or
/// the channel closes.
pub fn spawn_input_connection(
    host: String,
    port: u16,
    state: Arc<Mutex<AppState>>,
    rx: mpsc::UnboundedReceiver<String>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        if let Err(e) = connect_input(&host, port, rx, state.clone()).await {
            let mut s = state.lock().await;
            s.status.last_error = Some(format!("input: {e:#}"));
        }
    })
}

/// Spawn the bidirectional filter control-port connection: reads PGN
/// 262657 `Report` JSON into `state` (the same `upsert` path the bus
/// stream uses) and writes the UI's `Set` frames plus a periodic
/// `Request` poll so newly-observed sources / sentences keep appearing.
pub fn spawn_filter_connection(
    host: String,
    port: u16,
    state: Arc<Mutex<AppState>>,
    rx: mpsc::UnboundedReceiver<String>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        if let Err(e) = connect_filter(&host, port, state.clone(), rx).await {
            // The filter control port only exists when the server was
            // started with `--nmea0183-filter`. A refused/timed-out
            // connect there is the normal "no filter configured" case,
            // not something to splash across the status bar — log it and
            // leave the Nmea0183 screen empty.
            log::debug!("filter control connection to {host}:{port} ended: {e:#}");
        }
    })
}

/// Spawn the bidirectional override control-port connection: reads PGN
/// 262658 `Report` JSON into `state` (same `upsert` path as the bus
/// stream) and writes the UI's `Set` / `Delete` frames plus a periodic
/// `Request` poll so removals (here or NAK-forgotten server-side) and new
/// overrides keep the view current.
pub fn spawn_overrides_connection(
    host: String,
    port: u16,
    state: Arc<Mutex<AppState>>,
    rx: mpsc::UnboundedReceiver<String>,
) -> JoinHandle<()> {
    tokio::spawn(async move {
        if let Err(e) = connect_overrides(&host, port, state.clone(), rx).await {
            // Like the filter port, an unreachable override port is the
            // normal "server doesn't have it" case, not a UI error.
            log::debug!("override control connection to {host}:{port} ended: {e:#}");
        }
    })
}

/// Pull the entire snapshot blob from `host:port` and seed `state`
/// with it. Connection is closed by the server when the dump
/// completes; both the connect and the read are bounded by
/// [`CONNECT_TIMEOUT`] / [`SNAPSHOT_READ_TIMEOUT`] so a black-hole
/// peer can't hang the UI.
async fn load_snapshot(host: &str, port: u16, state: Arc<Mutex<AppState>>) -> Result<()> {
    let stream = timeout(CONNECT_TIMEOUT, TcpStream::connect((host, port)))
        .await
        .with_context(|| format!("connect snapshot port {host}:{port} timed out"))?
        .with_context(|| format!("connecting snapshot port {host}:{port}"))?;
    let mut reader = BufReader::new(stream);
    let mut buf = String::with_capacity(64 * 1024);
    timeout(SNAPSHOT_READ_TIMEOUT, reader.read_to_string(&mut buf))
        .await
        .context("reading snapshot blob timed out (peer did not close)")?
        .context("reading snapshot blob")?;

    // Strip the optional leading banner from the raw payload (before any
    // trim — an empty cache arrives as `banner\n\n`, and trimming first
    // would fuse the banner and the empty doc).
    let doc = strip_snapshot_banner(&buf);
    if doc.is_empty() {
        // Empty payload, or banner-only (empty cache) — nothing to load.
        let mut s = state.lock().await;
        s.status.snapshot_loaded = true;
        return Ok(());
    }
    let mut guard = state.lock().await;
    seed_state_from_snapshot(&mut guard, doc)?;
    guard.status.snapshot_loaded = true;
    Ok(())
}

/// Parse a snapshot document (banner already stripped) and seed
/// `state` with every `(pgn, src, secondary)` record it holds.
///
/// Handles both snapshot layouts: the default canboat-C layout keys
/// each group by the numeric PGN and stores bare `-json` records, while
/// `server --camel` keys each group by the pgn's camelCase id and stores
/// the records unwrapped with camelCase field keys. For the camel layout
/// we re-wrap each record as `{"<id>":{…}}` so `AppState::upsert`'s
/// normalizer rekeys the fields back to human names — the same path the
/// live stream uses — which is what keeps the device-info cache
/// (PGN 60928 / 126996 / 126998) populated under `--camel`.
fn seed_state_from_snapshot(state: &mut AppState, doc: &str) -> Result<()> {
    let root: Value = serde_json::from_str(doc).context("parsing snapshot JSON")?;
    let Some(by_pgn) = root.as_object() else {
        anyhow::bail!("snapshot top-level is not a JSON object");
    };
    for (group_key, group) in by_pgn {
        let Some(group_obj) = group.as_object() else {
            continue;
        };
        // A non-numeric group key means the camel layout: the key is the
        // pgn's camelCase id and each record is stored unwrapped.
        let camel_id: Option<&str> = match group_key.parse::<u32>() {
            Ok(_) => None,
            Err(_) => Some(group_key.as_str()),
        };
        for (key, line_val) in group_obj {
            if key == "description" {
                continue;
            }
            // `<src>` or `<src>_<secondary>` — split on the first `_`.
            let (src_str, secondary) = match key.split_once('_') {
                Some((s, sec)) => (s, Some(sec.to_string())),
                None => (key.as_str(), None),
            };
            let Ok(src) = src_str.parse::<u8>() else {
                continue;
            };
            // The numeric PGN lives inside the record in both layouts;
            // fall back to the group key for the bare (numeric) layout.
            let pgn = line_val
                .pointer("/pgn")
                .and_then(Value::as_u64)
                .map(|n| n as u32)
                .or_else(|| group_key.parse::<u32>().ok())
                .unwrap_or(0);
            let description = line_val
                .pointer("/description")
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string();
            let record = match camel_id {
                Some(id) => serde_json::json!({ id: line_val.clone() }),
                None => line_val.clone(),
            };
            state.upsert(pgn, src, secondary, description, record);
        }
    }
    Ok(())
}

/// Open the live stream socket and run reader + writer concurrently
/// on the current task. Returns when either half ends (peer closed,
/// I/O error). The Writer channel's `rx` is owned by this task — any
/// queued sends drain to the socket as soon as the connection is up.
async fn connect_stream(host: &str, port: u16, state: Arc<Mutex<AppState>>) -> Result<()> {
    let stream = timeout(CONNECT_TIMEOUT, TcpStream::connect((host, port)))
        .await
        .with_context(|| format!("connect stream port {host}:{port} timed out"))?
        .with_context(|| format!("connecting stream port {host}:{port}"))?;
    stream.set_nodelay(true).ok();
    // Read-only: the bus JSON stream never accepts client writes (the
    // server FINs its read side). Bus injection goes to the input port
    // and the filter control channel to its own port — see `Writer`.
    let (read_half, _write_half) = stream.into_split();

    {
        let mut s = state.lock().await;
        s.status.stream_connected = true;
    }
    reader_task(read_half, state.clone()).await;
    let mut s = state.lock().await;
    s.status.stream_connected = false;
    Ok(())
}

async fn connect_input(
    host: &str,
    port: u16,
    rx: mpsc::UnboundedReceiver<String>,
    state: Arc<Mutex<AppState>>,
) -> Result<()> {
    let stream = timeout(CONNECT_TIMEOUT, TcpStream::connect((host, port)))
        .await
        .with_context(|| format!("connect input port {host}:{port} timed out"))?
        .with_context(|| format!("connecting input port {host}:{port}"))?;
    stream.set_nodelay(true).ok();
    // Write-only. The input port FINs its own write side, so we never
    // read; dropping our read half leaves the write half open to keep
    // injecting. The task ends when a write fails or `rx` closes.
    let (_read_half, write_half) = stream.into_split();
    writer_task(write_half, rx, state).await;
    Ok(())
}

async fn connect_filter(
    host: &str,
    port: u16,
    state: Arc<Mutex<AppState>>,
    rx: mpsc::UnboundedReceiver<String>,
) -> Result<()> {
    let stream = timeout(CONNECT_TIMEOUT, TcpStream::connect((host, port)))
        .await
        .with_context(|| format!("connect filter port {host}:{port} timed out"))?
        .with_context(|| format!("connecting filter port {host}:{port}"))?;
    stream.set_nodelay(true).ok();
    let (read_half, write_half) = stream.into_split();
    // Reader ingests `Report` JSON; writer drains `Set`s and polls with
    // `Request`. First half to finish tears down the other.
    tokio::select! {
        _ = reader_task(read_half, state.clone()) => {}
        _ = filter_writer_task(write_half, rx, state.clone()) => {}
    }
    Ok(())
}

async fn connect_overrides(
    host: &str,
    port: u16,
    state: Arc<Mutex<AppState>>,
    rx: mpsc::UnboundedReceiver<String>,
) -> Result<()> {
    let stream = timeout(CONNECT_TIMEOUT, TcpStream::connect((host, port)))
        .await
        .with_context(|| format!("connect overrides port {host}:{port} timed out"))?
        .with_context(|| format!("connecting overrides port {host}:{port}"))?;
    stream.set_nodelay(true).ok();
    let (read_half, write_half) = stream.into_split();
    tokio::select! {
        _ = reader_task(read_half, state.clone()) => {}
        _ = overrides_writer_task(write_half, rx, state.clone()) => {}
    }
    Ok(())
}

/// Writer half of the override control connection: drains `Set` / `Delete`
/// lines from the UI and periodically polls with `Request` so the view
/// picks up server-side removals (NAK-forgotten overrides) and refreshes.
async fn overrides_writer_task(
    mut writer: OwnedWriteHalf,
    mut rx: mpsc::UnboundedReceiver<String>,
    state: Arc<Mutex<AppState>>,
) {
    let mut poll = tokio::time::interval(canboat_bridge::n2kd::nmea_filter::FILTER_REPORT_INTERVAL);
    poll.tick().await;
    loop {
        let mut line = tokio::select! {
            maybe = rx.recv() => match maybe {
                Some(l) => l,
                None => break,
            },
            _ = poll.tick() => crate::tui::iso::override_request(),
        };
        if !line.ends_with('\n') {
            line.push('\n');
        }
        if let Err(e) = writer.write_all(line.as_bytes()).await {
            let mut s = state.lock().await;
            s.status.last_error = Some(format!("overrides write: {e}"));
            break;
        }
    }
}

/// Writer half of the filter control connection: drains `Set` lines from
/// the UI and, on every [`canboat_bridge::n2kd::nmea_filter::FILTER_REPORT_INTERVAL`]
/// tick, sends a `Request` so the server re-reports — catching sources /
/// sentences observed since its last answer. The server also reports
/// unprompted on connect and after each `Set`, so the poll only exists
/// to surface newly-appearing rows.
async fn filter_writer_task(
    mut writer: OwnedWriteHalf,
    mut rx: mpsc::UnboundedReceiver<String>,
    state: Arc<Mutex<AppState>>,
) {
    let mut poll = tokio::time::interval(canboat_bridge::n2kd::nmea_filter::FILTER_REPORT_INTERVAL);
    // The first tick is immediate and redundant with the server's
    // connect-time report — consume it so the first real poll is one
    // full interval out.
    poll.tick().await;
    loop {
        let mut line = tokio::select! {
            maybe = rx.recv() => match maybe {
                Some(l) => l,
                None => break,
            },
            _ = poll.tick() => crate::tui::iso::nmea0183_filter_request(),
        };
        if !line.ends_with('\n') {
            line.push('\n');
        }
        if let Err(e) = writer.write_all(line.as_bytes()).await {
            let mut s = state.lock().await;
            s.status.last_error = Some(format!("filter write: {e}"));
            break;
        }
    }
}

async fn reader_task(reader: tokio::net::tcp::OwnedReadHalf, state: Arc<Mutex<AppState>>) {
    let mut br = BufReader::new(reader);
    let mut line = String::with_capacity(4096);
    loop {
        line.clear();
        match br.read_line(&mut line).await {
            Ok(0) => break,
            Ok(_) => {}
            Err(e) => {
                let mut s = state.lock().await;
                s.status.last_error = Some(format!("stream read: {e}"));
                break;
            }
        }
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.is_empty() {
            continue;
        }
        // Banner from analyzer-style emitters — skip without poisoning
        // the cache.
        if trimmed.starts_with("{\"version\"") {
            continue;
        }
        // canboat C / our n2kd require `}}` to filter out fieldless
        // records; mirror that so we never insert empty entries.
        if !trimmed.ends_with("}}") {
            continue;
        }
        let mut inputs = Vec::new();
        canboat_core::snapshot::classify_json_line(trimmed, |i| inputs.push(i));
        if inputs.is_empty() {
            continue;
        }
        let mut s = state.lock().await;
        for input in inputs {
            // Re-parse the (possibly spliced) line so the entry carries a
            // structured Value the UI can pointer-walk. `upsert`
            // canonicalizes `server --camel` records itself.
            let value = match serde_json::from_str::<Value>(&input.line) {
                Ok(v) => v,
                Err(_) => continue,
            };
            s.upsert(
                input.pgn,
                input.src,
                input.secondary,
                input.pgn_description,
                value,
            );
        }
    }
    let mut s = state.lock().await;
    s.status.stream_connected = false;
}

/// Drop a `canboat server` snapshot's leading one-line version/units
/// banner (`{"version":…}`) so only the cache document is parsed. A
/// server without the banner (older, or n2kd) sends the document as the
/// whole payload, which passes through unchanged.
fn strip_snapshot_banner(payload: &str) -> &str {
    match payload.split_once('\n') {
        Some((first, rest)) if first.trim_start().starts_with("{\"version\"") => rest.trim(),
        _ => payload.trim(),
    }
}

async fn writer_task(
    mut writer: OwnedWriteHalf,
    mut rx: mpsc::UnboundedReceiver<String>,
    state: Arc<Mutex<AppState>>,
) {
    while let Some(mut line) = rx.recv().await {
        if !line.ends_with('\n') {
            line.push('\n');
        }
        if let Err(e) = writer.write_all(line.as_bytes()).await {
            let mut s = state.lock().await;
            s.status.last_error = Some(format!("stream write: {e}"));
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tui::state::Status;
    use serde_json::json;
    use std::collections::HashMap;

    #[test]
    fn snapshot_banner_is_stripped() {
        // Banner-led snapshot (canboat server): only the doc remains.
        let with = "{\"version\":\"0.5.0\",\"units\":\"si\"}\n{\"130306\":{\"7\":{}}}";
        assert_eq!(strip_snapshot_banner(with), "{\"130306\":{\"7\":{}}}");
        // No banner (older server / n2kd): whole payload is the doc.
        let without = "{\"130306\":{\"7\":{}}}";
        assert_eq!(strip_snapshot_banner(without), without);
        // A doc whose first line merely starts with `{` but isn't a
        // version banner is left intact.
        let pretty = "{\n  \"130306\": {}\n}";
        assert_eq!(strip_snapshot_banner(pretty), pretty);
        // Banner-only (empty cache): `banner\n\n` → empty doc.
        assert_eq!(strip_snapshot_banner("{\"version\":\"x\"}\n\n"), "");
    }

    /// End-to-end regression for the `--camel` device-info cache: a
    /// server in `--camel` reshapes its snapshot (id-keyed groups,
    /// unwrapped records), and the TUI must still learn each device's
    /// NAME (PGN 60928) and product info (PGN 126996) from it. Drive the
    /// real `SnapshotStore` reshaping into `seed_state_from_snapshot` and
    /// assert the persistent cache is populated.
    #[test]
    fn camel_snapshot_seeds_device_cache() {
        use crate::tui::device_cache::NameKey;
        use canboat_core::snapshot::{SnapshotInput, SnapshotStore};

        let store = SnapshotStore::new();
        // ISO Address Claim first so the src→NAME mapping exists when the
        // Product Information for the same src arrives.
        store.store(SnapshotInput {
            pgn: 60928,
            src: 51,
            secondary: None,
            is_ais: false,
            pgn_description: "ISO Address Claim".into(),
            line: r#"{"isoAddressClaim":{"prio":6,"src":51,"dst":255,"pgn":60928,"description":"isoAddressClaim","fields":{"uniqueNumber":1605586,"manufacturerCode":{"value":419,"name":"Fusion Electronics"},"deviceFunction":{"value":130,"name":"Multimedia Player"},"deviceClass":{"value":125,"name":"Entertainment"},"industryGroup":{"value":4,"name":"Marine Industry"}}}}"#.into(),
        });
        store.store(SnapshotInput {
            pgn: 126996,
            src: 51,
            secondary: None,
            is_ais: false,
            pgn_description: "Product Information".into(),
            line: r#"{"productInformation":{"prio":6,"src":51,"dst":255,"pgn":126996,"description":"productInformation","fields":{"nmea2000Version":2100,"productCode":419,"modelId":"FUSION-MS","certificationLevel":2,"loadEquivalency":1}}}"#.into(),
        });
        let doc = store.snapshot();

        let mut s = AppState::new(Status::new_log("x".into()), HashMap::new());
        seed_state_from_snapshot(&mut s, &doc).expect("seed camel snapshot");

        let key = NameKey {
            manufacturer_code: 419,
            unique_number: 1605586,
        };
        let cached = s.device_cache.get(&key).expect("device cached under NAME");
        assert_eq!(
            cached.manufacturer_name.as_deref(),
            Some("Fusion Electronics")
        );
        assert_eq!(cached.product_code, Some(419));
        assert_eq!(cached.model_id.as_deref(), Some("FUSION-MS"));
    }

    #[test]
    fn filter_control_lines_are_recognised_by_pgn() {
        assert!(line_is_filter_control(
            "2026-01-01T00:00:00.000Z,7,262657,0,255,8,01,21,41,4c,4c,01,ff,ff"
        ));
        assert!(!line_is_filter_control(
            "2026-01-01T00:00:00.000Z,6,59904,0,35,3,00,ee,01"
        ));
        assert!(!line_is_filter_control("garbage"));
    }

    #[test]
    fn overrides_control_lines_are_recognised_by_pgn() {
        assert!(line_is_overrides_control(
            "2026-01-01T00:00:00.000Z,7,262658,0,255,12,01,34,12,fe,01,e8,03,00,00,ff,ff,ff"
        ));
        assert!(!line_is_overrides_control(
            "2026-01-01T00:00:00.000Z,7,262657,0,255,8,01,21,41,4c,4c,01,ff,ff"
        ));
        assert!(!line_is_overrides_control("garbage"));
    }

    #[test]
    fn writer_routes_by_pgn() {
        let (writer, mut rx) = make_writer();
        // 262657 → filter channel; 262658 → overrides channel;
        // ISO request (59904) → input channel.
        assert!(writer.send(crate::tui::iso::nmea0183_filter_request()));
        assert!(writer.send(crate::tui::iso::override_request()));
        assert!(writer.send(crate::tui::iso::iso_request(35, 126464)));
        assert!(rx.filter.try_recv().is_ok());
        assert!(rx.overrides.try_recv().is_ok());
        assert!(rx.input.try_recv().is_ok());
        // Each landed in exactly one channel.
        assert!(rx.filter.try_recv().is_err());
        assert!(rx.overrides.try_recv().is_err());
        assert!(rx.input.try_recv().is_err());
    }

    fn seeded() -> AppState {
        let mut s = AppState::new(Status::new_log("x".into()), HashMap::new());
        s.upsert(
            129025,
            10,
            None,
            "Position".into(),
            json!({"pgn": 129025, "fields": {"Latitude": 1.0}}),
        );
        s.upsert(127250, 10, None, "Heading".into(), json!({"pgn": 127250}));
        s
    }

    #[test]
    fn save_analysed_writes_one_json_line_per_record() {
        let s = seeded();
        let path =
            std::env::temp_dir().join(format!("canboat-tui-json-{}.json", std::process::id()));
        let n = save_capture(&path, &s, SaveFormat::Analysed).unwrap();
        assert_eq!(n, 2);
        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents.lines().count(), 2);
        assert!(contents.contains("129025"));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn save_raw_writes_header_and_captured_frames() {
        let mut s = seeded();
        // Simulate what the log path captures.
        s.raw_lines
            .push("2024-01-01T00:00:00.000,2,129025,10,255,8,01,02,03,04,05,06,07,08".into());
        let path = std::env::temp_dir().join(format!("canboat-tui-raw-{}.raw", std::process::id()));
        let n = save_capture(&path, &s, SaveFormat::Raw).unwrap();
        assert_eq!(n, 1);
        let contents = std::fs::read_to_string(&path).unwrap();
        let mut lines = contents.lines();
        assert_eq!(lines.next(), Some("# format=FAST"));
        assert!(lines.next().unwrap().contains(",129025,"));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn analysed_export_is_detected_and_reingestible() {
        // Two field-bearing records so the classifier yields inputs.
        let mut s = AppState::new(Status::new_log("x".into()), HashMap::new());
        s.upsert(
            129025,
            10,
            None,
            "Position".into(),
            json!({"pgn": 129025, "src": 10, "fields": {"Latitude": 1.0}}),
        );
        s.upsert(
            127250,
            10,
            None,
            "Heading".into(),
            json!({"pgn": 127250, "src": 10, "fields": {"Heading": 2.0}}),
        );
        let path = std::env::temp_dir().join(format!("canboat-tui-rt-{}.json", std::process::id()));
        save_capture(&path, &s, SaveFormat::Analysed).unwrap();

        // Our export sniffs as JSON, a raw header does not.
        assert!(looks_like_json_capture(&path));

        // Re-ingesting yields the records back through the channel.
        let (tx, mut rx) = mpsc::unbounded_channel::<LoadItem>();
        ingest_json_capture(&path, &tx).unwrap();
        drop(tx);
        let mut records = 0;
        while let Ok(item) = rx.try_recv() {
            if matches!(item, LoadItem::Record(_)) {
                records += 1;
            }
        }
        assert_eq!(records, 2);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn raw_capture_is_not_mistaken_for_json() {
        let path =
            std::env::temp_dir().join(format!("canboat-tui-rawsniff-{}.raw", std::process::id()));
        std::fs::write(
            &path,
            "# format=FAST\n2024-01-01T00:00:00.000,2,129025,10,255,8,01,02,03,04,05,06,07,08\n",
        )
        .unwrap();
        assert!(!looks_like_json_capture(&path));
        let _ = std::fs::remove_file(&path);
    }

    #[tokio::test]
    async fn spawn_save_writes_records_and_clears_progress() {
        let state = Arc::new(Mutex::new(seeded()));
        let path =
            std::env::temp_dir().join(format!("canboat-tui-async-{}.json", std::process::id()));
        spawn_save(path.clone(), SaveFormat::Analysed, state.clone())
            .await
            .unwrap();

        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents.lines().count(), 2);
        assert!(contents.contains("129025"));

        let s = state.lock().await;
        assert!(s.save_progress.is_none(), "progress cleared when done");
        assert!(
            s.notice.as_deref().unwrap().contains("Saved 2 records"),
            "completion notice raised"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn save_raw_empty_in_live_mode_still_writes_header() {
        let s = AppState::new(Status::new_live("h".into(), 1, 2), HashMap::new());
        let path =
            std::env::temp_dir().join(format!("canboat-tui-raw-empty-{}.raw", std::process::id()));
        let n = save_capture(&path, &s, SaveFormat::Raw).unwrap();
        assert_eq!(n, 0);
        let contents = std::fs::read_to_string(&path).unwrap();
        assert_eq!(contents, "# format=FAST\n");
        let _ = std::fs::remove_file(&path);
    }
}
