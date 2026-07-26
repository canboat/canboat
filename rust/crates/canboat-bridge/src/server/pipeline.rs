// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Drives the analyzer → n2kd conversion stage.
//!
//! Consumes `RawFrame`s off an `mpsc::Receiver`, runs them through
//! the reassembler + PGN decoder, dispatches to the struct-based
//! converters in `crate::n2kd::decoded` / `crate::n2kd::ais_decoded` (with a JSON
//! fallback for the long tail of PGNs the struct path hasn't covered
//! yet), and writes NMEA 0183 sentences to stdout. Three side
//! branches feed the optional TCP servers:
//!
//! * `csv_hub` — every `RawFrame` rendered as a PLAIN/FAST line.
//! * `nmea_hub` — every NMEA 0183 sentence (one or more per record).
//! * `analyzer_hub` — every decoded record rendered as analyzer JSON.
//! * `snapshot` — analyzer JSON stashed per `(pgn, src, secondary)`
//!   for the n2kd-compatible full-state-on-connect port.
//!
//! Each side branch is gated. The hub-broadcast paths skip the
//! formatter when no one is subscribed (atomic load). The snapshot
//! cache, when present, always wants its JSON line, so JSON
//! serialization runs whenever either the analyzer hub has
//! subscribers OR the snapshot store is configured.

use std::collections::VecDeque;
use std::io::{self, LineWriter, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::n2kd::request_engine::RequestEngine;
use canboat_core::format::write_plain;
use canboat_core::output::{JsonOptions, write_json};
use canboat_core::{FramePacketType, PgnDatabase, RawFrame, Reassembled, Reassembler};
use canboat_io::device::FrameSender;

use canboat_wire::WirePgn;

use crate::n2kd::serving::{BinHub, Hub};
use crate::server::quirks::Quirks;
use crate::server::snapshot::SnapshotStore;

/// Source-side output batcher for one broadcast hub.
///
/// Broadcasting per line costs a channel send, a futex wake, and a
/// `write` syscall *per subscriber per line* — at bus rates that
/// dwarfs the formatting work. Instead, lines accumulate here and go
/// out as one multi-line chunk when the batch is old enough or big
/// enough. Subscribers are line-oriented streams, so a chunk of
/// whole lines is indistinguishable from the same lines sent singly.
///
/// Flushing is driven by the pipeline's own frame loop (one
/// [`OutputBatcher::flush_due`] call per received frame), which at
/// bus rates checks far more often than `MAX_AGE` — and a quiet bus
/// has nothing pending to flush, so no timer thread is needed.
struct OutputBatcher {
    hub: Arc<Hub>,
    pending: String,
    oldest: Instant,
}

/// Flush when the oldest pending line has waited this long…
const BATCH_MAX_AGE: Duration = Duration::from_millis(25);
/// …or when this much output has accumulated, whichever comes first.
const BATCH_MAX_BYTES: usize = 16 * 1024;

impl OutputBatcher {
    fn new(hub: Arc<Hub>) -> Self {
        Self {
            hub,
            pending: String::with_capacity(2048),
            oldest: Instant::now(),
        }
    }

    #[inline]
    fn has_subscribers(&self) -> bool {
        self.hub.has_subscribers()
    }

    /// Queue one line (trailing `\n` included by the caller).
    fn push(&mut self, line: &str, now: Instant) {
        if self.pending.is_empty() {
            self.oldest = now;
        }
        self.pending.push_str(line);
        if self.pending.len() >= BATCH_MAX_BYTES {
            self.flush();
        }
    }

    #[inline]
    fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }

    /// Flush if the oldest pending line has exceeded [`BATCH_MAX_AGE`].
    #[inline]
    fn flush_due(&mut self, now: Instant) {
        if !self.pending.is_empty() && now.duration_since(self.oldest) >= BATCH_MAX_AGE {
            self.flush();
        }
    }

    fn flush(&mut self) {
        self.hub.broadcast(&self.pending);
        self.pending.clear();
    }
}

/// Byte-oriented twin of [`OutputBatcher`] for the [`BinHub`] stream.
/// Accumulates whole length-prefixed frames and flushes them as one
/// chunk on the same age/size cadence, so a fast-packet flurry becomes
/// one `broadcast` (one channel send + write per subscriber) instead of
/// one per record.
struct BinBatcher {
    hub: Arc<BinHub>,
    pending: Vec<u8>,
    oldest: Instant,
}

impl BinBatcher {
    fn new(hub: Arc<BinHub>) -> Self {
        Self {
            hub,
            pending: Vec::with_capacity(4096),
            oldest: Instant::now(),
        }
    }

    #[inline]
    fn has_subscribers(&self) -> bool {
        self.hub.has_subscribers()
    }

    /// Append one already-framed record (its `u32` length prefix + body).
    fn push(&mut self, frame: &[u8], now: Instant) {
        if self.pending.is_empty() {
            self.oldest = now;
        }
        self.pending.extend_from_slice(frame);
        if self.pending.len() >= BATCH_MAX_BYTES {
            self.flush();
        }
    }

    #[inline]
    fn has_pending(&self) -> bool {
        !self.pending.is_empty()
    }

    #[inline]
    fn flush_due(&mut self, now: Instant) {
        if !self.pending.is_empty() && now.duration_since(self.oldest) >= BATCH_MAX_AGE {
            self.flush();
        }
    }

    fn flush(&mut self) {
        self.hub.broadcast(&self.pending);
        self.pending.clear();
    }
}

fn is_ais_pgn(pgn: u32) -> bool {
    matches!(
        pgn,
        129038
            | 129039
            | 129040
            | 129041
            | 129793
            | 129794
            | 129798
            | 129801
            | 129802
            | 129809
            | 129810
    )
}

/// Bundle of broadcast hubs the pipeline writes into.
pub struct Hubs {
    /// Raw N2K input/output: every coalesced frame goes out as a
    /// `# format=FAST` PLAIN line; clients can write PLAIN/FAST back
    /// to inject onto the bus. Previously named `csv`.
    pub raw: Arc<Hub>,
    pub nmea: Arc<Hub>,
    pub analyzer: Arc<Hub>,
    /// Binary analyzer stream: each decoded record as a length-prefixed
    /// postcard [`WirePgn`]. Lazy like the others — the encode only runs
    /// when a client is subscribed (see `--analyzer-binary-port`).
    pub bin: Arc<BinHub>,
    /// Optional cache for the snapshot port. When `Some`, every
    /// decoded record's analyzer JSON line lands in the cache; the
    /// snapshot TCP listener dumps the live entries on each connect.
    pub snapshot: Option<Arc<SnapshotStore>>,
    /// Per-device tracker for the periodic ISO claim / product-info
    /// auto-request engine. The pipeline updates it from every
    /// decoded frame; `main.rs` separately spawns the request loop
    /// when there's a device writer to send the resulting PGN 59904
    /// requests to.
    pub engine: Arc<RequestEngine>,
    /// Device-quirk workarounds. Inspects every inbound frame and
    /// optionally emits synthetic responses (e.g. PGN 126996 on
    /// behalf of an SCX-20 that's "forgotten" how to answer). Empty
    /// kinds = no-op; the per-frame call short-circuits.
    pub quirks: Quirks,
    /// Outbound frame sender into the device writer. Used by quirk
    /// synthesisers to land their impersonation on the wire so
    /// external consumers (e.g. an NGT-1 on the same bus) see it.
    /// `None` in stdin-only mode where there's no device writer.
    pub device_sender: Option<FrameSender>,
    /// canboat's live claimed source address (SocketCAN exposes it; other
    /// backends leave it `None`). Used to stamp a quirk's "own-address"
    /// emission (src `ADDR_GLOBAL`) with the real address, so it reaches
    /// the bus and the local streams as canboat's own node.
    pub claim_addr: Option<Arc<std::sync::atomic::AtomicU8>>,
    /// Server-owned PGN-rate overrides. Shared behind a mutex with the
    /// dedicated control-port server; the pipeline replays a device's
    /// overrides on first sight (PGN 60928) and forgets one the bus NAKs
    /// (PGN 126208 Acknowledge). `None` when the config dir is unwritable.
    pub overrides: Option<Arc<Mutex<crate::n2kd::overrides::OverrideEngine>>>,
    /// Optional in-process tap. When `Some`, every decoded record is
    /// forwarded here (an `Arc` refcount bump, no deep clone) for an
    /// embedding consumer — this is what backs [`crate::server::Bridge::decoded`].
    /// `None`, the CLI default, costs one `Option::is_some` per record.
    pub decoded_tx: Option<std::sync::mpsc::Sender<Arc<canboat_core::DecodedPgn>>>,
}

/// NMEA 0183 output options for [`run`], bundled to keep the entry
/// point's signature small.
pub struct Nmea0183Options {
    /// Mirrors canboat C `n2kd`'s `--nmea0183` flag: also write
    /// sentences to stdout, not just the TCP port.
    pub emit_stdout: bool,
    /// Enable the 1 Hz per-`(src, quantity)` rate limit.
    pub rate_limit: bool,
    /// Per-device NAME filter; `None` disables it. Shared behind a
    /// mutex with the dedicated control-port server (see
    /// [`crate::server::tcp::spawn_filter_control_server`]), which owns
    /// the `Set`/`Request`/`Report` exchange with the TUI. The pipeline
    /// only *reads through* it here: learning `src → NAME` from address
    /// claims and muting converted sentences. Both are ~1 Hz per source,
    /// so the lock is uncontended on the hot path.
    pub filter: Option<Arc<Mutex<crate::n2kd::nmea_filter::NmeaFilter>>>,
}

/// Pipeline entry point. Returns when `frames_rx` is closed.
///
/// * `emit_nmea_stdout` mirrors canboat C `n2kd`'s `--nmea0183` flag.
///   When `true`, NMEA 0183 sentences are also written to stdout (in
///   addition to the optional TCP NMEA-0183 broadcast). Off by
///   default — long-running deployments use the TCP port and don't
///   want their service log spammed.
/// * `pre_coalesced` is the shared "are frames already coalesced
///   PGN payloads?" flag. Set up-front to `true` for sources known
///   to coalesce on the wire (NGT-1, iKonvert, Maretron); the
///   pipeline will additionally flip it to `true` the first time it
///   sees a frame with `data.len() > 8` (matching canboat's
///   `RAWFORMAT_PLAIN_OR_FAST` → `FAST` lock-in). Once true it
///   stays true. The stdin pump also flips it when it sees a
///   `# format=<NAME>` header declaring a coalesced format.
/// * `nmea0183_rate_limit` gates the 1 Hz per-`(src, quantity)` NMEA
///   0183 limiter (see [`crate::n2kd::nmea0183::RateLimiter`]). `true` (the
///   deployed default) drops repeat sentences within 1 s so several
///   devices reporting the same measurement don't flood downstream
///   0183 consumers; AIS is emitted on a separate path and never
///   limited. `false` emits every converted sentence unthrottled.
/// * `nmea_filter`, when `Some`, mutes NMEA 0183 sentences per device
///   NAME (see [`crate::n2kd::nmea_filter`]): it learns `src → NAME` from
///   PGN 60928 and drops a converted sentence block when the source is
///   unknown or muted. `None` disables it (every converted sentence is
///   emitted). AIS is never routed through it.
/// * `json_opts` configures the analyzer JSON / snapshot serializer
///   — `camel_case` selects field-key + PGN-description style
///   (`Off` / `Lower` matches canboat C `-camel`, `Upper` matches
///   `-upper-camel`), and the same options drive per-iteration
///   snapshot lines for PGNs with PK fields in a repeating set.
pub fn run(
    db: &'static PgnDatabase,
    frames_rx: Receiver<RawFrame>,
    mut hubs: Hubs,
    pre_coalesced: Arc<AtomicBool>,
    json_opts: JsonOptions,
    nmea: Nmea0183Options,
) {
    let Nmea0183Options {
        emit_stdout: emit_nmea_stdout,
        rate_limit: nmea0183_rate_limit,
        filter: nmea_filter,
    } = nmea;
    // LineWriter (rather than BufWriter) so each NMEA 0183 sentence
    // is flushed as soon as its trailing newline arrives. Long-
    // running deployments observe stdout for the latest state, not
    // a batched dump; canboat C n2kd flushes per line too.
    let stdout = io::stdout();
    let mut out = LineWriter::new(stdout.lock());

    let mut reasm = Reassembler::new();
    let mut nmea_buf = String::with_capacity(256);
    let mut raw_line = String::with_capacity(256);
    let mut json_line = String::with_capacity(1024);
    let mut rl = crate::n2kd::nmea0183::RateLimiter::new(nmea0183_rate_limit);
    let handles = crate::n2kd::decoded::Handles::new();

    // Quirk synthesisers can produce extra `RawFrame`s in response to
    // an inbound bus frame. We re-feed them through this same loop so
    // they pass through reassembly, decode and broadcast just like a
    // real bus frame would. A synthetic frame can't re-trigger a
    // quirk (its PGN is always a *response*, never the trigger PGN),
    // so there's no risk of an infinite synthesis loop.
    let mut pending_synth: VecDeque<RawFrame> = VecDeque::new();

    let mut raw_batch = OutputBatcher::new(hubs.raw.clone());
    let mut nmea_batch = OutputBatcher::new(hubs.nmea.clone());
    let mut analyzer_batch = OutputBatcher::new(hubs.analyzer.clone());
    let mut bin_batch = BinBatcher::new(hubs.bin.clone());
    // Reused encode buffer for the binary stream: one framed WirePgn is
    // built here then appended to `bin_batch`'s pending chunk.
    let mut wire_frame: Vec<u8> = Vec::with_capacity(1024);

    loop {
        let any_pending = raw_batch.has_pending()
            || nmea_batch.has_pending()
            || analyzer_batch.has_pending()
            || bin_batch.has_pending();
        let frame = if let Some(synth) = pending_synth.pop_front() {
            synth
        } else if any_pending {
            // Output is waiting on a batch deadline: don't block
            // indefinitely on a quiet bus. On timeout, flush what's
            // pending and go back to waiting.
            match frames_rx.recv_timeout(BATCH_MAX_AGE) {
                Ok(f) => f,
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    let now = Instant::now();
                    raw_batch.flush_due(now);
                    nmea_batch.flush_due(now);
                    analyzer_batch.flush_due(now);
                    bin_batch.flush_due(now);
                    continue;
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
            }
        } else {
            match frames_rx.recv() {
                Ok(f) => f,
                Err(_) => break,
            }
        };
        let now = Instant::now();

        // The NMEA 0183 filter control channel (PGN 262657) no longer
        // touches this loop: its `Set`/`Request`/`Report` exchange runs
        // entirely on the dedicated bidirectional control port (see
        // `crate::server::tcp::spawn_filter_control_server`), so a filter
        // frame is never broadcast onto the read-only analyzer stream and
        // never reaches the bus. The pipeline only reads through the
        // shared filter below (learn NAME, mute sentences).

        // When the source already coalesces fast-packets (NGT-1,
        // iKonvert, Maretron, FAST-format stdin), the reassembler
        // must be skipped entirely. A coalesced fast-packet whose
        // payload is ≤8 bytes would otherwise have its first byte
        // misread as a sequence / frame-index header.
        //
        // Sticky lock-in: once we see ANY frame with more than 8
        // payload bytes, the upstream is definitely emitting
        // coalesced PGNs, so flip the shared flag for all
        // subsequent frames (and any other subscriber that reads
        // it). Matches canboat's `RAWFORMAT_PLAIN_OR_FAST` → `FAST`
        // promotion in analyzer.c.
        if !pre_coalesced.load(Ordering::Relaxed) && frame.data.len() > 8 {
            log::debug!(
                "frame with {} payload bytes seen; locking pipeline into coalesced mode",
                frame.data.len()
            );
            pre_coalesced.store(true, Ordering::Relaxed);
        }
        let assembled = if pre_coalesced.load(Ordering::Relaxed) {
            frame
        } else {
            let packet_type = db
                .first_pgn(frame.pgn)
                .or_else(|| db.fallback_pgn(frame.pgn))
                .map(|p| match p.packet_type {
                    canboat_core::PacketType::Fast => FramePacketType::Fast,
                    canboat_core::PacketType::Single => FramePacketType::Single,
                    _ => FramePacketType::Other,
                })
                .unwrap_or(FramePacketType::Other);
            match reasm.push(frame, packet_type) {
                Reassembled::PassThrough(f) | Reassembled::Complete(f) => f,
                _ => continue,
            }
        };

        // Lazy raw broadcast — one PLAIN line per *coalesced message*,
        // i.e. the post-reassembly frame. This stream advertises
        // `# format=FAST` (coalesced), so it must emit whole messages:
        // broadcasting the pre-reassembly fragments here would label
        // single CAN frames as coalesced and make downstream readers
        // skip reassembly, so fast-packet PGNs would decode from only
        // their first 8 bytes. Fragments (`Reassembled::Partial`) never
        // reach this point — the `continue` above drops them.
        if raw_batch.has_subscribers() {
            raw_line.clear();
            if write_plain(&mut raw_line, &assembled).is_ok() {
                raw_line.push('\n');
                raw_batch.push(&raw_line, now);
            }
        }

        let Ok(decoded) = db.decode(&assembled) else {
            continue;
        };
        // Share one decoded record across every consumer (snapshot,
        // JSON, NMEA, binary) via `Arc`. The snapshot's lazy cache can
        // then hold a refcount bump instead of a deep clone.
        let decoded = Arc::new(decoded);

        // In-process tap for an embedding consumer (`Bridge::decoded`).
        // A closed or lagging receiver must never stall the bus loop, so
        // the send is best-effort. Placed before quirk synthesis so a
        // synthetic frame is tapped once, when it loops back through here
        // as a normal decode — every record is seen exactly once.
        if let Some(tx) = hubs.decoded_tx.as_ref() {
            let _ = tx.send(Arc::clone(&decoded));
        }

        // Quirk shim: inspect the decoded PGN, maybe synthesise. Working
        // from the decoded record (rather than a raw pre-reassembly frame)
        // lets quirks read typed, named fields and act on fast-packet PGNs
        // too. Each synthetic is written to the bus (so external consumers
        // see it, with the impersonated src where applicable) and queued
        // back into this loop (so the local pipeline indexes / broadcasts
        // it too).
        if hubs.quirks.is_enabled() {
            // canboat's live claimed address, if it holds a valid unicast
            // one — used to resolve a quirk's "own-address" emission.
            let own_addr = hubs
                .claim_addr
                .as_ref()
                .map(|a| a.load(Ordering::Relaxed))
                .filter(|&a| a != canboat_core::ADDR_GLOBAL && a != canboat_core::ADDR_NULL);
            for mut synth in hubs.quirks.process_decoded(&decoded) {
                // A quirk that emits from src 0 / ADDR_GLOBAL ("send as my
                // own node", e.g. the WMM quirk) gets canboat's real
                // claimed address stamped here — the one place that knows
                // the server's identity — so the bus write and the local
                // feedback below carry the same real src. An explicit src
                // (the SCX-20 impersonation) passes through untouched.
                if (synth.src == 0 || synth.src == canboat_core::ADDR_GLOBAL)
                    && let Some(addr) = own_addr
                {
                    synth.src = addr;
                }
                if let Some(sender) = hubs.device_sender.as_ref() {
                    let _ = sender.send_frame(synth.clone());
                }
                pending_synth.push_back(synth);
            }
        }

        // Feed the periodic claim/product-info request engine.
        // Updates "last received" stamps for PGN 60928 and 126996.
        hubs.engine.note_device_seen(decoded.pgn, decoded.src);

        // Learn `src → NAME` for the per-device 0183 filter. The NAME
        // is the 64-bit ISO Address Claim identity, so even a device
        // with an "unavailable" manufacturer code is uniquely
        // identified. `iso_name()` re-packs it from the decoded fields
        // (returning `None` for any non-claim PGN) — the same helper
        // n2kd uses, which has no raw payload to read. A source that
        // never claims stays unmapped and produces no 0183 — the "no
        // NAME, no output" rule.
        if let Some(f) = nmea_filter.as_ref()
            && let Some(name) = decoded.iso_name()
        {
            f.lock().unwrap().note_address_claim(decoded.src, name);
        }

        // Server-owned PGN-rate overrides. On an ISO Address Claim, replay
        // this device's overrides (first sight this session, or after it
        // moved to a new source) by injecting the PGN 126208 Requests. On
        // a PGN 126208 Acknowledge that NAKs (non-zero error), forget the
        // override the bus rejected so it isn't replayed forever. Both
        // resolve the device by src → NAME inside the engine.
        if let Some(ov) = hubs.overrides.as_ref() {
            if let Some(name) = decoded.iso_name() {
                let replay = ov.lock().unwrap().note_address_claim(decoded.src, name);
                if let Some(sender) = hubs.device_sender.as_ref() {
                    for frame in replay {
                        let _ = sender.send_frame(frame);
                    }
                }
            } else if decoded.pgn == 126208 {
                use canboat_core::field::nmea_acknowledge_group_function as ack;
                let field_int = |f| decoded.field(f).and_then(|d| d.value.as_i64());
                if field_int(ack::FUNCTION_CODE) == Some(2) {
                    let pgn_err = field_int(ack::PGN_ERROR_CODE).unwrap_or(0);
                    let iv_err =
                        field_int(ack::TRANSMISSION_INTERVAL_PRIORITY_ERROR_CODE).unwrap_or(0);
                    if (pgn_err != 0 || iv_err != 0)
                        && let Some(acked) = field_int(ack::PGN)
                    {
                        ov.lock().unwrap().note_nak(decoded.src, acked as u32);
                    }
                }
            }
        }

        // Snapshot cache: hand the store the decoded record and let it
        // serialize lazily, only when a snapshot client reads. So the
        // snapshot no longer forces per-record JSON — that cost is now
        // paid solely for the analyzer port's live subscribers.
        if let Some(snap) = hubs.snapshot.as_ref() {
            snap.store(&decoded, now);
        }

        // Lazy analyzer JSON — serialized at most ONCE per decoded
        // record and shared by the analyzer port broadcast and the
        // NMEA 0183 fallback converter below. The serializer walks
        // every decoded field, so skipping it when no one is subscribed
        // actually buys something on a high-rate input stream.
        let want_json = analyzer_batch.has_subscribers();
        let mut json_ok = false;
        if want_json {
            json_line.clear();
            json_ok = write_json(&mut json_line, &decoded, &json_opts).is_ok();
        }

        // NMEA 0183 conversion only runs when someone will see the
        // result — a connected TCP subscriber or `--nmea0183-stdout`.
        // With neither, the per-record cost is two atomic loads.
        nmea_buf.clear();
        let pgn = decoded.pgn;
        // AIS output (`!AI…`) is exempt from the per-device 0183 filter.
        let ais_branch = is_ais_pgn(pgn);
        // Convert when someone will see the 0183, OR when the filter is
        // active: the filter learns each device's producible sentences
        // by observing conversions, and the TUI needs that inventory
        // (via the 262657 Report) even with no 0183 client attached.
        let want_nmea = emit_nmea_stdout || nmea_batch.has_subscribers() || nmea_filter.is_some();
        let converted = if !want_nmea {
            false
        } else {
            // Struct path only — `convert_nmea0183` dispatches on the
            // decoded variant id, folding in AIS `!AIVDM`, and yields
            // nothing for non-0183 PGNs. There is deliberately no JSON
            // round-trip here, so the analyzer JSON options (`--camel`,
            // SI) never reach the 0183 output.
            crate::n2kd::decoded::convert_nmea0183(&mut nmea_buf, &decoded, &mut rl, &handles) > 0
        };
        // Per-device NMEA 0183 filter mutes redundant devices' `$`
        // sentences; AIS (`!AI…`) is exempt and passes straight through.
        if converted
            && !ais_branch
            && let Some(f) = nmea_filter.as_ref()
        {
            f.lock().unwrap().apply(decoded.src, &mut nmea_buf);
        }
        if !nmea_buf.is_empty() {
            if emit_nmea_stdout {
                let _ = out.write_all(nmea_buf.as_bytes());
            }
            if nmea_batch.has_subscribers() {
                nmea_batch.push(&nmea_buf, now);
            }
        }

        // Analyzer port broadcast last, so the newline appended for
        // its one-record-per-line framing doesn't leak into the
        // shared bare-JSON uses above.
        if json_ok && analyzer_batch.has_subscribers() {
            json_line.push('\n');
            analyzer_batch.push(&json_line, now);
        }

        // Binary analyzer stream — the reference-stripped `WirePgn`
        // built straight from `decoded` (no JSON involved) and appended
        // as one length-prefixed postcard frame. Gated on subscribers so
        // the encode is skipped entirely when the port has no clients.
        if bin_batch.has_subscribers() {
            wire_frame.clear();
            if canboat_wire::append_frame(&mut wire_frame, &WirePgn::from(decoded.as_ref())).is_ok()
            {
                bin_batch.push(&wire_frame, now);
            }
        }

        raw_batch.flush_due(now);
        nmea_batch.flush_due(now);
        analyzer_batch.flush_due(now);
        bin_batch.flush_due(now);
    }
    // Shutdown: push out whatever the batchers still hold.
    raw_batch.flush();
    nmea_batch.flush();
    analyzer_batch.flush();
    bin_batch.flush();
    out.flush().ok();
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    /// Build a `Hubs` with all broadcast hubs idle and the given decoded
    /// tap — the minimal shape for driving the pipeline in a test.
    fn tapped_hubs(decoded_tx: mpsc::Sender<Arc<canboat_core::DecodedPgn>>) -> Hubs {
        Hubs {
            raw: Arc::new(Hub::new()),
            nmea: Arc::new(Hub::new()),
            analyzer: Arc::new(Hub::new()),
            bin: Arc::new(BinHub::new()),
            snapshot: None,
            engine: Arc::new(RequestEngine::new()),
            quirks: crate::server::quirks::Quirks::new(Vec::new()),
            device_sender: None,
            claim_addr: None,
            overrides: None,
            decoded_tx: Some(decoded_tx),
        }
    }

    /// The `decoded_tx` tap (backing `Bridge::decoded`) forwards every
    /// decoded record exactly once, and stays silent — costing nothing —
    /// when it is `None`.
    #[test]
    fn decoded_tap_forwards_each_record() {
        let db = canboat_core::PgnDatabase::embedded(canboat_core::Units::Metric);
        let (frames_tx, frames_rx) = mpsc::channel();
        let (tap_tx, tap_rx) = mpsc::channel();

        // One 8-byte ISO Address Claim (PGN 60928, single-frame) — decodes
        // without reassembly, so drive the loop in pre-coalesced mode.
        let claim =
            canboat_core::RawFrame::new(None, 6, 60928, 0x21, 0xFF, [1, 2, 3, 4, 5, 6, 7, 8]);
        frames_tx.send(claim).unwrap();
        drop(frames_tx); // close the source so `run` returns

        let pre_coalesced = Arc::new(AtomicBool::new(true));
        let handle = std::thread::spawn(move || {
            run(
                db,
                frames_rx,
                tapped_hubs(tap_tx),
                pre_coalesced,
                JsonOptions {
                    include_empty: false,
                    name_value: true,
                    debug: false,
                    camel_case: canboat_core::output::CamelCase::Off,
                },
                Nmea0183Options {
                    emit_stdout: false,
                    rate_limit: true,
                    filter: None,
                },
            )
        });

        let decoded = tap_rx
            .recv()
            .expect("the tap should receive the decoded record");
        assert_eq!(decoded.pgn, 60928);
        assert_eq!(decoded.src, 0x21);
        // Exactly one record, then end-of-stream when the pipeline drains.
        assert!(tap_rx.recv().is_err(), "tap should see only the one record");
        handle.join().unwrap();
    }
}
