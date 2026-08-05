// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! In-memory model of the bus, fed from the snapshot + live stream.
//!
//! Two structures matter:
//!
//! * [`Entry`] — one entry per `(pgn, src, secondary)` triple. The
//!   key shape exactly matches what canboat-pipeline / n2kd write
//!   into their TCP snapshot (port 2597) and stream (port 2598). On
//!   the snapshot port the key arrives encoded as the JSON field name
//!   `"<src>[_<secondary>]"`; on the stream port we recompute it via
//!   [`canboat_core::snapshot::classify_json_line`] so the two paths
//!   produce identical keys.
//!
//! * [`AppState`] — owns the entry map plus a derived view of which
//!   source addresses currently exist on the bus, with their
//!   manufacturer / product / model info pulled out of the cached
//!   ISO Address Claim (60928), Product Information (126996), and
//!   Configuration Information (126998) entries. The
//!   PGN-List-Transmit/Receive (126464) entry, when present, is
//!   exposed the same way.
//!
//! `AppState` is intentionally small and `tokio::sync::Mutex`-friendly
//! — the UI grabs it for one frame, the network task grabs it to
//! apply each incoming line. No background derivation runs; views are
//! computed on demand from the cached JSON values.

use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::time::Instant;

use canboat_core::{FieldRef, PgnDatabase, PgnInfo, Units, field};
use indexmap::IndexMap;
use serde_json::{Map, Value};

use crate::tui::device_cache::{CachedInfo, NameKey};

/// `(pgn, src, secondary)` — same shape as
/// [`canboat_core::snapshot`]'s internal cache key.
pub type EntryKey = (u32, u8, Option<String>);

/// One snapshot entry: a single decoded record indexed by its
/// composite primary key. `line` caches the latest observation so
/// the UI can pull fields without indirecting through
/// [`AppState::history`] on every frame; `history_indices` indexes
/// every observation we've seen for this key in chronological order
/// so the user can step through past instances with left / right
/// (and the [`Screen::TimeView`] flattens them all into one
/// timeline).
#[derive(Debug, Clone)]
pub struct Entry {
    pub pgn: u32,
    pub src: u8,
    pub secondary: Option<String>,
    /// PGN description from the record itself (full form, including
    /// the manufacturer prefix). Empty when the analyzer didn't emit
    /// one.
    pub description: String,
    pub line: Value,
    /// Wall-clock-ish moment we processed the most recent record for
    /// this key — drives both the "age" column and the interval
    /// estimate.
    pub last_update: Instant,
    /// Total records seen for this key since the entry was first
    /// inserted. Combined with [`Entry::first_seen`] and
    /// [`Entry::last_update`] this gives the average measured
    /// inter-arrival time (see [`Entry::interval`]).
    pub count: u64,
    /// First time we saw a record for this key. Anchors the
    /// interval average so the displayed cadence is `(last − first)
    /// / (count − 1)` and not perturbed by a single late frame.
    pub first_seen: Instant,
    /// Wire timestamp from the *first* observation, parsed to Unix
    /// milliseconds. `None` when the analyzer JSON line didn't
    /// carry a `"timestamp"` field. When both this and
    /// `last_stamp_ms` are present, [`Entry::interval`] uses them
    /// instead of the wall-clock `first_seen` / `last_update`
    /// pair — the whole point being that in log-replay mode the
    /// `Instant` pair reflects ingest speed, not real bus cadence.
    pub first_stamp_ms: Option<i64>,
    /// Wire timestamp from the *latest* observation, same units as
    /// [`Entry::first_stamp_ms`].
    pub last_stamp_ms: Option<i64>,
    /// Indices into [`AppState::history`], one per observation of
    /// this key, in chronological insertion order. The last index
    /// names the same record that `line` caches. Empty on
    /// synthetic silenced-override placeholder rows.
    pub history_indices: Vec<usize>,
}

impl Entry {
    /// Average measured transmission interval. Returns `None` until
    /// the second record arrives (one observation isn't enough to
    /// measure a cadence). Prefers wire timestamps
    /// (`first_stamp_ms`/`last_stamp_ms`) when both are present so
    /// the number reflects the real bus cadence even in log-replay
    /// mode; falls back to the wall-clock `first_seen`/`last_update`
    /// `Instant` pair when the analyzer JSON didn't carry a
    /// `"timestamp"` field (some formats don't).
    pub fn interval(&self) -> Option<std::time::Duration> {
        if self.count < 2 {
            return None;
        }
        let denom = self.count.saturating_sub(1) as u32;
        if let (Some(first), Some(last)) = (self.first_stamp_ms, self.last_stamp_ms) {
            let span_ms = (last - first).max(0) as u64;
            return Some(std::time::Duration::from_millis(span_ms) / denom);
        }
        let span = self.last_update.saturating_duration_since(self.first_seen);
        Some(span / denom)
    }
}

/// One row in the device list.
#[derive(Debug, Clone, Default)]
pub struct DeviceInfo {
    pub src: u8,
    /// Manufacturer name from PGN 60928 (ISO Address Claim) or
    /// 126996 (Product Information). Empty when neither has been
    /// seen.
    pub manufacturer: String,
    /// Product model from PGN 126996 `"Model ID"`. Empty when 126996
    /// hasn't been seen for this src.
    pub model: String,
    /// Software version (PGN 126996 `"Software Version Code"`).
    pub software: String,
    /// Configuration installation description (PGN 126998
    /// `"Installation Description #1"`).
    pub installation: String,
    /// Distinct PGN numbers we've seen from this source.
    pub pgn_count: usize,
}

/// Per-source NMEA 0183 filter state, learned from the pipeline's PGN
/// 262657 Report frames. `muted` is the whole-source state; `sentences`
/// maps each 3-letter formatter the device can produce to its current
/// (effective) mute state. See [`AppState::apply_nmea0183_report`].
#[derive(Debug, Clone, Default)]
pub struct Nmea0183Device {
    pub muted: bool,
    pub sentences: BTreeMap<String, bool>,
}

/// One navigable row in the NMEA 0183 filter screen: either a
/// whole-source header (`sentence == None`) or one of its sentences.
#[derive(Debug, Clone)]
pub struct Nmea0183Row {
    pub src: u8,
    pub sentence: Option<String>,
    pub muted: bool,
}

/// How long a PGN-rate override stays in the Overrides view after its
/// last server Report. The TUI re-`Request`s every 2 s, so a row that
/// stops being reported (deleted here or NAK-forgotten by the server)
/// ages out after a few missed polls.
pub const OVERRIDE_TTL: std::time::Duration = std::time::Duration::from_secs(6);

/// One PGN-rate override as reported by the server's PGN 262658 control
/// channel, addressed by the device's current source. Carries the fields
/// needed to render, edit, or delete it. Overrides are owned + persisted
/// by the server now; this is just the view model.
#[derive(Debug, Clone)]
pub struct OverrideRow {
    pub src: u8,
    pub pgn: u32,
    /// Requested transmission interval in ms; `0` = stopped.
    pub interval_ms: u32,
    pub manufacturer_code: Option<u16>,
    pub industry_code: Option<u8>,
    /// PGN description from the schema, for display.
    pub description: String,
    /// When the server last reported this override; drives [`OVERRIDE_TTL`].
    pub last_seen: Instant,
}

/// Progress of a long-running background operation (currently the
/// capture save). Rendered as a bar; `done`/`total` are record counts.
#[derive(Debug, Clone)]
pub struct Progress {
    pub label: String,
    pub done: usize,
    pub total: usize,
}

/// One row in the `top`-style PGN-load view: a single PGN aggregated
/// across every source producing it, with the total observation count
/// and the summed message rate (messages/second). Highest-rate PGNs
/// sort first, so the busiest traffic floats to the top the way
/// `top(1)` shows the hungriest processes.
#[derive(Debug, Clone)]
pub struct PgnLoadRow {
    pub pgn: u32,
    /// Description from any contributing entry (they share a PGN, so
    /// the wording is the same modulo per-source manufacturer prefix).
    pub description: String,
    /// Distinct source addresses transmitting this PGN.
    pub sources: usize,
    /// Total records observed across all sources.
    pub count: u64,
    /// Average bus rate in messages/second: total observations divided
    /// by the capture duration. `0.0` when the duration can't be
    /// measured (a single instant, or no timestamps at all).
    pub rate: f32,
}

/// The `ALL` sentence token in PGN 262657 (whole-source mute). Mirrors
/// `canboat_pipeline::nmea_filter::ALL_SENTENCES`.
pub const NMEA0183_ALL: &str = "ALL";

/// What's feeding the cache — a live bus / pipeline endpoint or a
/// captured log file. Drives several UI affordances:
///
/// * `Mode::Log` hides the `i` (ISO Request) and `o` (override)
///   bindings; both rely on writing back to a live bus and would
///   silently do nothing against a file.
/// * The status bar's "endpoint" label switches between
///   `host:snap/stream` and `log: <path>` shapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Live,
    Log,
}

/// Snapshot of source / connection state shown in the status bar.
#[derive(Debug, Clone)]
pub struct Status {
    pub mode: Mode,
    /// Live mode: hostname. Log mode: log file path (display form).
    pub host: String,
    /// Live mode: snapshot port; ignored / `0` in Log mode.
    pub snapshot_port: u16,
    /// Live mode: stream port; ignored / `0` in Log mode.
    pub stream_port: u16,
    /// Live: snapshot blob fully drained. Log: log file fully
    /// decoded.
    pub snapshot_loaded: bool,
    /// Live: stream socket up. Log: always `false`.
    pub stream_connected: bool,
    pub messages_seen: u64,
    pub last_error: Option<String>,
}

impl Status {
    pub fn new_live(host: String, snapshot_port: u16, stream_port: u16) -> Self {
        Self {
            mode: Mode::Live,
            host,
            snapshot_port,
            stream_port,
            snapshot_loaded: false,
            stream_connected: false,
            messages_seen: 0,
            last_error: None,
        }
    }

    pub fn new_log(path_display: String) -> Self {
        Self {
            mode: Mode::Log,
            host: path_display,
            snapshot_port: 0,
            stream_port: 0,
            snapshot_loaded: false,
            stream_connected: false,
            messages_seen: 0,
            last_error: None,
        }
    }

    /// Idle startup state (no `--host` / `--log`): a Log-shaped status
    /// with no source. `snapshot_loaded` is set so the status bar reads
    /// settled rather than a perpetual "loading…"; the UI opens the
    /// File ▸ Load dialog to get the user to a real source.
    pub fn new_idle() -> Self {
        Self {
            mode: Mode::Log,
            host: "(no source — File ▸ Load / Connect)".to_string(),
            snapshot_port: 0,
            stream_port: 0,
            snapshot_loaded: true,
            stream_connected: false,
            messages_seen: 0,
            last_error: None,
        }
    }
}

/// One observation of any PGN, in arrival order. The TUI keeps the
/// full history in [`AppState::history`] so each `Entry` can index
/// back into it for left/right navigation and the (forthcoming)
/// `TimeView` can iterate all records chronologically without
/// touching the per-key `Entry` map.
#[derive(Debug, Clone)]
pub struct HistoryRecord {
    /// ISO timestamp string from the analyzer JSON line, if any.
    pub timestamp: Option<String>,
    /// Local-clock arrival time — used as the chronological sort
    /// key when the wire `timestamp` is absent.
    pub seen_at: Instant,
    pub pgn: u32,
    pub src: u8,
    pub secondary: Option<String>,
    pub description: String,
    pub line: Value,
}

/// Shared state guarded by a [`tokio::sync::Mutex`] in the binary.
pub struct AppState {
    /// Insertion-ordered to keep the UI stable across renders. Insertions
    /// happen on first sight of a `(pgn, src, secondary)`; updates keep
    /// the slot in place.
    pub entries: IndexMap<EntryKey, Entry>,
    /// Every observation of every key, in chronological insertion
    /// order. `Entry::history_indices` points back into this. Grows
    /// unbounded; in `Mode::Log` it's capped by the file size, in
    /// `Mode::Live` we'll add a cap once long-running sessions need
    /// one.
    pub history: Vec<HistoryRecord>,
    pub status: Status,
    /// A one-shot positive notice (e.g. "✓ Loaded N records", "✓ Saved
    /// …") raised by a background task. The UI drains it into its toast
    /// slot so the user gets confirmation an async load / save finished.
    pub notice: Option<String>,
    /// Progress of an in-flight capture save, `None` when idle. While
    /// `Some`, the UI shows a progress bar and Save / Load / Connect are
    /// disabled (the save task holds append-only indices into `history` /
    /// `raw_lines` that a reset would invalidate).
    pub save_progress: Option<Progress>,
    /// Persistent NAME → device-info cache, loaded at startup and
    /// saved at exit. Enriches `device_list()` with fields the
    /// current session hasn't observed (typical case: a log capture
    /// that includes ISO Address Claim but not the request-only
    /// Product Information PGN 126996).
    pub device_cache: HashMap<NameKey, CachedInfo>,
    /// Last-seen NAME per source address, populated from PGN 60928
    /// as records arrive. Used to route PGN 126996 / 126998 updates
    /// to the correct NAME in the cache (those PGNs don't carry a
    /// NAME themselves — they arrive addressed to the same `src` as
    /// the recent Address Claim).
    pub src_to_name: HashMap<u8, NameKey>,
    /// Per-source NMEA 0183 filter state, fed by the pipeline's PGN
    /// 262657 Report frames. Drives the `Screen::Nmea0183` view.
    pub nmea0183: BTreeMap<u8, Nmea0183Device>,
    /// Coalesced canboat PLAIN/FAST lines, one per decoded frame, in
    /// arrival order — the source for a "Save ▸ Raw" export. Only
    /// populated in log-replay mode, where the wire bytes are still
    /// available (`DecodedPgn::data`); the live JSON stream carries no
    /// raw bytes, so this stays empty there.
    pub raw_lines: Vec<String>,
    /// PGN-rate overrides the server currently holds, keyed by
    /// `(src, pgn)` and fed by its PGN 262658 Report frames. Ages out via
    /// [`OVERRIDE_TTL`]. Drives the `Screen::Overrides` view.
    pub overrides: BTreeMap<(u8, u32), OverrideRow>,
}

impl AppState {
    pub fn new(status: Status, device_cache: HashMap<NameKey, CachedInfo>) -> Self {
        Self {
            entries: IndexMap::new(),
            history: Vec::new(),
            status,
            notice: None,
            save_progress: None,
            device_cache,
            src_to_name: HashMap::new(),
            nmea0183: BTreeMap::new(),
            raw_lines: Vec::new(),
            overrides: BTreeMap::new(),
        }
    }

    /// Insert or refresh one record. `line` has already been parsed
    /// to a `Value` by the caller.
    pub fn upsert(
        &mut self,
        pgn: u32,
        src: u8,
        secondary: Option<String>,
        description: String,
        line: Value,
    ) {
        // Canonicalize `server --camel` records to bare `-json` shape so
        // every reader below (all keyed by human field name) works
        // unchanged. No-op on the default non-camel stream.
        let line = normalize_camel(line);
        // Prefer the canonical human description the normalizer wrote (the
        // camel wrapper's `description` is the pgn id); fall back to the
        // caller's for hand-built / bare records.
        let description = line
            .pointer("/description")
            .and_then(Value::as_str)
            .map(str::to_string)
            .unwrap_or(description);
        // PGN 262657 is the pipeline's NMEA 0183 filter control channel,
        // not a bus record — route its Report frames into the filter
        // view instead of the entry/device model.
        if pgn == 262657 {
            self.apply_nmea0183_report(&line);
            self.status.messages_seen = self.status.messages_seen.saturating_add(1);
            return;
        }
        // PGN 262658 is the server's PGN-rate-override control channel —
        // route its Report frames into the Overrides view.
        if pgn == 262658 {
            self.apply_override_report(&line);
            self.status.messages_seen = self.status.messages_seen.saturating_add(1);
            return;
        }
        let key = (pgn, src, secondary.clone());
        let now = Instant::now();
        let timestamp = line
            .pointer("/timestamp")
            .and_then(Value::as_str)
            .map(str::to_string);
        // Parse the wire timestamp to Unix ms so `Entry::interval`
        // can compute real bus cadence in log mode (where the
        // `Instant`-based diff is meaningless — it just measures
        // how fast we ingested lines).
        let stamp_ms = timestamp.as_deref().and_then(canboat_core::parse_iso_ms);

        // Append to the global chronological history first so the
        // entry's freshly-pushed index points at the right row.
        let history_idx = self.history.len();
        self.history.push(HistoryRecord {
            timestamp,
            seen_at: now,
            pgn,
            src,
            secondary: secondary.clone(),
            description: description.clone(),
            line: line.clone(),
        });

        match self.entries.get_mut(&key) {
            Some(e) => {
                e.line = line;
                e.description = description;
                e.last_update = now;
                e.count += 1;
                e.history_indices.push(history_idx);
                // Only overwrite `last_stamp_ms` when this record
                // carried a valid timestamp; a single record with
                // no `timestamp` field shouldn't null out the
                // interval for a whole burst.
                if let Some(ms) = stamp_ms {
                    e.last_stamp_ms = Some(ms);
                    if e.first_stamp_ms.is_none() {
                        e.first_stamp_ms = Some(ms);
                    }
                }
            }
            None => {
                self.entries.insert(
                    key,
                    Entry {
                        pgn,
                        src,
                        secondary,
                        description,
                        line,
                        last_update: now,
                        count: 1,
                        first_seen: now,
                        first_stamp_ms: stamp_ms,
                        last_stamp_ms: stamp_ms,
                        history_indices: vec![history_idx],
                    },
                );
            }
        }
        // Every 60928 / 126996 / 126998 record enriches the
        // persistent NAME → info cache. See
        // `AppState::observe_for_cache` for the extraction logic.
        self.observe_for_cache(pgn, src, self.history.last().map(|h| h.line.clone()));
        self.status.messages_seen = self.status.messages_seen.saturating_add(1);
    }

    /// Update the persistent [`AppState::device_cache`] from a
    /// freshly-observed record. PGN 60928 (ISO Address Claim)
    /// carries the NAME components; PGNs 126996 and 126998 carry
    /// product / configuration info that we key off the NAME learned
    /// from the last 60928 for the same source.
    fn observe_for_cache(&mut self, pgn: u32, src: u8, line: Option<Value>) {
        let Some(line) = line else { return };
        let now_stamp = line
            .pointer("/timestamp")
            .and_then(Value::as_str)
            .map(str::to_string);
        match pgn {
            60928 => {
                if let Some(key) = name_key_from_claim(&line) {
                    self.src_to_name.insert(src, key);
                    let info = CachedInfo {
                        manufacturer_name: manufacturer_name_from_claim(&line),
                        device_function: field_number(
                            &line,
                            field::iso_address_claim::DEVICE_FUNCTION,
                        )
                        .map(|v| v as u32),
                        device_class: field_number(&line, field::iso_address_claim::DEVICE_CLASS)
                            .map(|v| v as u32),
                        industry_group: field_number(
                            &line,
                            field::iso_address_claim::INDUSTRY_GROUP,
                        )
                        .map(|v| v as u32),
                        first_seen: now_stamp.clone(),
                        last_seen: now_stamp,
                        ..Default::default()
                    };
                    self.device_cache.entry(key).or_default().merge_from(info);
                }
            }
            126996 => {
                let Some(key) = self.src_to_name.get(&src).copied() else {
                    return;
                };
                use field::product_information as pi;
                let info = CachedInfo {
                    product_code: field_number(&line, pi::PRODUCT_CODE).map(|v| v as u32),
                    model_id: field_text(&line, pi::MODEL_ID),
                    software_version: field_text(&line, pi::SOFTWARE_VERSION_CODE),
                    model_version: field_text(&line, pi::MODEL_VERSION),
                    model_serial_code: field_text(&line, pi::MODEL_SERIAL_CODE),
                    nmea_db_version: field_number(&line, pi::NMEA2000_VERSION).map(|v| v as u32),
                    // Was `"NMEA 2000 Certification Level"` — no such field
                    // exists (it's just `certificationLevel` / "Certification
                    // Level"), so this silently read `None` until the constant
                    // migration surfaced it.
                    certification_level: field_number(&line, pi::CERTIFICATION_LEVEL)
                        .map(|v| v as u32),
                    load_equivalency: field_number(&line, pi::LOAD_EQUIVALENCY).map(|v| v as u32),
                    last_seen: now_stamp,
                    ..Default::default()
                };
                self.device_cache.entry(key).or_default().merge_from(info);
            }
            126998 => {
                let Some(key) = self.src_to_name.get(&src).copied() else {
                    return;
                };
                use field::configuration_information as ci;
                let info = CachedInfo {
                    installation_description_1: field_text(&line, ci::INSTALLATION_DESCRIPTION1),
                    installation_description_2: field_text(&line, ci::INSTALLATION_DESCRIPTION2),
                    manufacturer_information: field_text(&line, ci::MANUFACTURER_INFORMATION),
                    last_seen: now_stamp,
                    ..Default::default()
                };
                self.device_cache.entry(key).or_default().merge_from(info);
            }
            _ => {}
        }
    }

    /// Walk the cache and produce a stable, src-ordered device list,
    /// pulling identity fields out of the device-info PGNs. Cheap
    /// enough to run per frame (a typical bus has tens of devices and
    /// hundreds of entries).
    ///
    /// Anything not yet observed this session (typical case: a log
    /// capture missing the request-only PGN 126996) is backfilled
    /// from [`AppState::device_cache`] whenever we have a NAME for
    /// this source, so the DeviceDetail screen shows the same
    /// manufacturer / model / installation labels the user has seen
    /// on this device before.
    pub fn device_list(&self) -> Vec<DeviceInfo> {
        let mut by_src: BTreeMap<u8, DeviceInfo> = BTreeMap::new();
        let mut pgns_per_src: BTreeMap<u8, std::collections::BTreeSet<u32>> = BTreeMap::new();
        for entry in self.entries.values() {
            pgns_per_src.entry(entry.src).or_default().insert(entry.pgn);
            let dev = by_src.entry(entry.src).or_insert_with(|| DeviceInfo {
                src: entry.src,
                ..Default::default()
            });
            match entry.pgn {
                60928 => fill_from_claim(dev, &entry.line),
                126996 => fill_from_product_info(dev, &entry.line),
                126998 => fill_from_config_info(dev, &entry.line),
                _ => {}
            }
        }
        for (src, set) in pgns_per_src {
            if let Some(dev) = by_src.get_mut(&src) {
                dev.pgn_count = set.len();
            }
        }
        // Enrichment pass: anywhere a field is still empty, try the
        // persistent cache. Cache lookups are anchored on the NAME
        // we learned from this session's 60928 (if any) — a session
        // with only cached info and no fresh ISO claim can't be
        // enriched because we don't know which NAME owns which src.
        for dev in by_src.values_mut() {
            let Some(key) = self.src_to_name.get(&dev.src) else {
                continue;
            };
            let Some(cached) = self.device_cache.get(key) else {
                continue;
            };
            if dev.manufacturer.is_empty()
                && let Some(m) = &cached.manufacturer_name
            {
                dev.manufacturer = m.clone();
            }
            if dev.model.is_empty()
                && let Some(m) = &cached.model_id
            {
                dev.model = m.clone();
            }
            if dev.software.is_empty()
                && let Some(s) = &cached.software_version
            {
                dev.software = s.clone();
            }
            if dev.installation.is_empty()
                && let Some(i) = &cached.installation_description_1
            {
                dev.installation = i.clone();
            }
        }
        by_src.into_values().collect()
    }

    /// Fold one PGN 262657 Report frame into [`AppState::nmea0183`].
    /// Only Report frames (Function 0) carry state; a stray Set
    /// (Function 1) echoed back is ignored.
    fn apply_nmea0183_report(&mut self, line: &Value) {
        use field::canboat_nmea0183_filter as fl;
        if field_number(line, fl::FUNCTION) != Some(0) {
            return;
        }
        let Some(src) = field_number(line, fl::SOURCE) else {
            return;
        };
        let Some(sentence) = field_text(line, fl::SENTENCE) else {
            return;
        };
        let muted = field_number(line, fl::MUTED).unwrap_or(0) != 0;
        let dev = self.nmea0183.entry(src as u8).or_default();
        if sentence == NMEA0183_ALL {
            dev.muted = muted;
        } else {
            dev.sentences.insert(sentence, muted);
        }
    }

    /// Fold a PGN 262658 override `Report` into [`AppState::overrides`].
    /// Addressed by the device's current source; refreshes `last_seen`
    /// so the row survives the next few [`OVERRIDE_TTL`] windows.
    fn apply_override_report(&mut self, line: &Value) {
        use field::canboat_pgn_override as ov;
        if field_number(line, ov::FUNCTION) != Some(0) {
            return; // Reports only (function 0)
        }
        let Some(src) = field_number(line, ov::SOURCE) else {
            return;
        };
        let Some(pgn) = field_number(line, ov::PGN) else {
            return;
        };
        let src = src as u8;
        let pgn = pgn as u32;
        let interval_ms = field_number(line, ov::INTERVAL_MS).unwrap_or(0).max(0) as u32;
        let manufacturer_code = field_number(line, ov::MANUFACTURER_CODE)
            .filter(|v| *v != 0xffff)
            .map(|v| v as u16);
        let industry_code = field_number(line, ov::INDUSTRY_CODE)
            .filter(|v| *v != 0xff)
            .map(|v| v as u8);
        // Resolve the description via the manufacturer the override itself
        // carries, so a proprietary PGN shared across manufacturers is
        // labelled with *this* device's variant (e.g. Furuno, not whichever
        // manufacturer's definition happens to be listed first).
        let description = variant_for(pgn, manufacturer_code)
            .map(|p| p.description.to_string())
            .unwrap_or_default();
        self.overrides.insert(
            (src, pgn),
            OverrideRow {
                src,
                pgn,
                interval_ms,
                manufacturer_code,
                industry_code,
                description,
                last_seen: Instant::now(),
            },
        );
    }

    /// Live override rows (dropping any that have aged past
    /// [`OVERRIDE_TTL`]), sorted by source then PGN for the Overrides view.
    pub fn override_rows(&self) -> Vec<OverrideRow> {
        self.overrides
            .values()
            .filter(|r| r.last_seen.elapsed() < OVERRIDE_TTL)
            .cloned()
            .collect()
    }

    /// Optimistically drop an override the user just deleted, so the row
    /// disappears immediately instead of waiting for it to age out.
    pub fn forget_override(&mut self, src: u8, pgn: u32) {
        self.overrides.remove(&(src, pgn));
    }

    /// Drop overrides the server has stopped reporting (deleted here or
    /// NAK-forgotten). Called each UI tick to bound the map.
    pub fn prune_overrides(&mut self) {
        self.overrides
            .retain(|_, r| r.last_seen.elapsed() < OVERRIDE_TTL);
    }

    /// Flatten [`AppState::nmea0183`] into navigable rows: one
    /// whole-source header per device followed by its sentences,
    /// source-ordered.
    pub fn nmea0183_rows(&self) -> Vec<Nmea0183Row> {
        let mut rows = Vec::new();
        for (src, dev) in &self.nmea0183 {
            rows.push(Nmea0183Row {
                src: *src,
                sentence: None,
                muted: dev.muted,
            });
            for (sentence, muted) in &dev.sentences {
                rows.push(Nmea0183Row {
                    src: *src,
                    sentence: Some(sentence.clone()),
                    muted: *muted,
                });
            }
        }
        rows
    }

    /// Aggregate every entry by PGN for the `top`-style load view.
    /// Sums observation counts across all sources of each PGN and
    /// divides by the capture duration to get the average bus rate
    /// (messages/second), then sorts busiest-first (rate desc, then
    /// count desc, then PGN asc for a stable tie-break).
    ///
    /// The rate is `count / duration`, NOT the sum of per-source
    /// instantaneous cadences: a bursty request-only PGN (e.g. 126996
    /// Product Information, a couple of frames from each of 48 sources
    /// at startup) would otherwise report a wildly inflated rate despite
    /// contributing almost nothing to the bus over the whole capture.
    pub fn pgn_load_rows(&self) -> Vec<PgnLoadRow> {
        let dur = self.capture_duration_secs();
        let mut agg: HashMap<u32, (String, HashSet<u8>, u64)> = HashMap::new();
        for e in self.entries.values() {
            // `count == 0` is the synthetic silenced-override sentinel —
            // no real traffic, so it doesn't belong in a load view.
            if e.count == 0 {
                continue;
            }
            let slot = agg.entry(e.pgn).or_default();
            if slot.0.is_empty() && !e.description.is_empty() {
                slot.0 = e.description.clone();
            }
            slot.1.insert(e.src);
            slot.2 += e.count;
        }
        let mut rows: Vec<PgnLoadRow> = agg
            .into_iter()
            .map(|(pgn, (description, srcs, count))| PgnLoadRow {
                pgn,
                description,
                sources: srcs.len(),
                count,
                rate: if dur > 0.0 { count as f32 / dur } else { 0.0 },
            })
            .collect();
        rows.sort_by(|a, b| {
            b.rate
                .partial_cmp(&a.rate)
                .unwrap_or(Ordering::Equal)
                .then_with(|| b.count.cmp(&a.count))
                .then_with(|| a.pgn.cmp(&b.pgn))
        });
        rows
    }

    /// Wall-time span of the capture in seconds, from the wire
    /// timestamps (earliest first-seen to latest last-seen across all
    /// entries). Falls back to the monotonic arrival span when the log
    /// carried no timestamps — the best we can do there, though in
    /// log-replay mode that reflects ingest time, not the real capture.
    /// `0.0` when there's nothing (or a single instant) to measure.
    pub fn capture_duration_secs(&self) -> f32 {
        let (mut min_ms, mut max_ms) = (i64::MAX, i64::MIN);
        for e in self.entries.values() {
            if let Some(f) = e.first_stamp_ms {
                min_ms = min_ms.min(f);
            }
            if let Some(l) = e.last_stamp_ms {
                max_ms = max_ms.max(l);
            }
        }
        if max_ms > min_ms {
            return (max_ms - min_ms) as f32 / 1000.0;
        }
        // No usable wire timestamps: fall back to the arrival-clock span.
        let (mut earliest, mut latest): (Option<Instant>, Option<Instant>) = (None, None);
        for e in self.entries.values() {
            earliest = Some(earliest.map_or(e.first_seen, |x: Instant| x.min(e.first_seen)));
            latest = Some(latest.map_or(e.last_update, |x: Instant| x.max(e.last_update)));
        }
        match (earliest, latest) {
            (Some(a), Some(b)) if b > a => b.duration_since(a).as_secs_f32(),
            _ => 0.0,
        }
    }

    /// All entries for `src`, sorted by PGN then secondary so the UI
    /// shows a stable list.
    pub fn entries_for_src(&self, src: u8) -> Vec<&Entry> {
        let mut v: Vec<&Entry> = self.entries.values().filter(|e| e.src == src).collect();
        v.sort_by(|a, b| {
            a.pgn
                .cmp(&b.pgn)
                .then_with(|| a.secondary.cmp(&b.secondary))
        });
        v
    }

    /// Latest cached PGN 126464 entries (PGN List — Transmit /
    /// Receive) for `src`, split into the two lists. Either Vec is
    /// empty when the corresponding direction hasn't been observed
    /// (yet).
    pub fn pgn_lists_for_src(&self, src: u8) -> PgnLists {
        let mut tx = Vec::new();
        let mut rx = Vec::new();
        for entry in self.entries.values() {
            if entry.pgn != 126464 || entry.src != src {
                continue;
            }
            let direction = field_number(
                &entry.line,
                field::pgn_list_transmit_and_receive::FUNCTION_CODE,
            );
            let list = collect_pgn_list(&entry.line);
            match direction {
                Some(0) => tx.extend(list),
                Some(1) => rx.extend(list),
                _ => {}
            }
        }
        tx.sort_unstable();
        tx.dedup();
        rx.sort_unstable();
        rx.dedup();
        PgnLists { tx, rx }
    }
}

/// TX / RX PGN lists pulled out of cached PGN 126464 records.
#[derive(Debug, Clone, Default)]
pub struct PgnLists {
    pub tx: Vec<u32>,
    pub rx: Vec<u32>,
}

impl PgnLists {
    pub fn is_empty(&self) -> bool {
        self.tx.is_empty() && self.rx.is_empty()
    }
}

/// Extract a `NameKey` (`(manufacturer_code, unique_number)`) from
/// an analyzer-JSON PGN 60928 record. Returns `None` when either
/// field is absent — an incomplete Address Claim isn't useful as a
/// cache key.
fn name_key_from_claim(line: &Value) -> Option<NameKey> {
    let mfr = field_number(line, field::iso_address_claim::MANUFACTURER_CODE)?;
    let uid = field_number(line, field::iso_address_claim::UNIQUE_NUMBER)?;
    Some(NameKey {
        manufacturer_code: u16::try_from(mfr).ok()?,
        unique_number: u32::try_from(uid).ok()?,
    })
}

/// Pull the `-nv` display name of the Manufacturer Code lookup out
/// of a PGN 60928 record. Falls back to the bare-integer path so
/// non-`-nv` producers still populate the cache with *something*.
fn manufacturer_name_from_claim(line: &Value) -> Option<String> {
    let v = field_value(line, field::iso_address_claim::MANUFACTURER_CODE)?;
    v.pointer("/name")
        .and_then(Value::as_str)
        .map(str::to_string)
}

/// The JSON `Value` at `line.fields.<field>` for a build-time field
/// constant (e.g. `field::product_information::MODEL_ID`). This is the
/// single place a field's JSON key is derived — currently the human
/// `name` (bare `-json`), so camel support later is a one-line change
/// here (`f.field.name` → `f.field.id`).
pub(crate) fn field_value(line: &Value, f: FieldRef) -> Option<&Value> {
    line.pointer(&format!("/fields/{}", json_pointer_escape(f.field.name)))
}

/// Canonicalize a camelCase analyzer record to the spaced-name shape
/// every reader below expects, in both flavours the servers emit:
///
/// * **flat** (`--id camel`, the default) — field keys are camelCase
///   `id`s on an unwrapped record; rename them to their human names.
/// * **wrapped** (`--wrap`, canboat C's `-camel`) — additionally strip
///   the `{"<pgnId>":{…}}` envelope, whose id names the exact PGN
///   variant, and restore the human `description` the wrapper replaced.
///
/// Renaming recurses into `list`/`list2` repeat sets. A spaced-name
/// record passes through untouched (no key matches a schema `id`), so
/// every reader downstream stays name-keyed and camel-oblivious.
pub(crate) fn normalize_camel(line: Value) -> Value {
    // The wrapper is the only shape that presents as a one-key object
    // whose value is itself a record (`{"windData":{"pgn":…}}`); any
    // unwrapped record has several top-level keys.
    let is_wrapper = matches!(&line, Value::Object(o)
        if o.len() == 1 && o.values().next().is_some_and(|v| v.get("pgn").is_some()));
    let Value::Object(top) = line else {
        return line;
    };
    let db = PgnDatabase::embedded(Units::Metric);
    if !is_wrapper {
        // Flat record: `description` still carries the human string in
        // every `--id` mode, so it picks the variant; fall back to the
        // PGN number.
        let mut record = Value::Object(top);
        let pgn = record.get("pgn").and_then(Value::as_u64).unwrap_or(0) as u32;
        let info = record
            .get("description")
            .and_then(Value::as_str)
            .and_then(|d| {
                db.pgn_variants(pgn)
                    .find(|p| p.description == d || p.id == d)
            })
            .or_else(|| db.first_pgn(pgn));
        if let Some(info) = info
            && let Some(Value::Object(fields)) = record.get_mut("fields")
        {
            rekey_fields(fields, info);
        }
        return record;
    }
    let (wrapper_id, mut record) = top.into_iter().next().expect("one entry");
    let pgn = record.get("pgn").and_then(Value::as_u64).unwrap_or(0) as u32;
    if let Some(info) = db.pgn_by_id(&wrapper_id).or_else(|| db.first_pgn(pgn))
        && let Value::Object(obj) = &mut record
    {
        // The camel `description` is just the pgn id — restore the human one.
        obj.insert(
            "description".to_string(),
            Value::String(info.description.to_string()),
        );
        if let Some(Value::Object(fields)) = obj.get_mut("fields") {
            rekey_fields(fields, info);
        }
    }
    record
}

/// Rename each `id`-keyed field back to its human name, recursing into
/// `list`/`list2` repeat-set element objects (whose keys are the same
/// repeating fields' ids). Keys that aren't a schema field id (`list`,
/// the `-nv` `{value,name}` sub-keys) are left untouched.
fn rekey_fields(fields: &mut Map<String, Value>, info: &PgnInfo) {
    let renames: Vec<(String, String)> = fields
        .keys()
        .filter_map(|k| {
            info.fields
                .iter()
                .find(|f| f.id == k.as_str() && f.name != k.as_str())
                .map(|f| (k.clone(), f.name.to_string()))
        })
        .collect();
    for (old, new) in renames {
        if let Some(v) = fields.remove(&old) {
            fields.insert(new, v);
        }
    }
    for list_key in ["list", "list2"] {
        if let Some(Value::Array(arr)) = fields.get_mut(list_key) {
            for elem in arr.iter_mut() {
                if let Value::Object(elem_obj) = elem {
                    rekey_fields(elem_obj, info);
                }
            }
        }
    }
}

/// Pull an integer out of `line.fields.<field>` (handles both the
/// bare-integer JSON shape and the `-nv` `{value, name}` object).
fn field_number(line: &Value, f: FieldRef) -> Option<i64> {
    field_as_int(field_value(line, f)?)
}

fn fill_from_claim(dev: &mut DeviceInfo, line: &Value) {
    if dev.manufacturer.is_empty()
        && let Some(name) = field_text(line, field::iso_address_claim::MANUFACTURER_CODE)
    {
        dev.manufacturer = name;
    }
}

fn fill_from_product_info(dev: &mut DeviceInfo, line: &Value) {
    if let Some(s) = field_text(line, field::product_information::MODEL_ID) {
        dev.model = s;
    }
    if let Some(s) = field_text(line, field::product_information::SOFTWARE_VERSION_CODE) {
        dev.software = s;
    }
    // (Product Information has no manufacturer field — the old
    // `field_text(line, "Manufacturer")` here always read `None`, so it's
    // dropped rather than migrated. Manufacturer comes from the ISO
    // Address Claim via `fill_from_claim`.)
}

fn fill_from_config_info(dev: &mut DeviceInfo, line: &Value) {
    if let Some(s) = field_text(
        line,
        field::configuration_information::INSTALLATION_DESCRIPTION1,
    ) {
        dev.installation = s;
    }
}

/// Pull a field's display text out of a parsed analyzer JSON line.
/// Handles both bare values and the `-nv` `{value, name}` lookup
/// object shape.
fn field_text(line: &Value, f: FieldRef) -> Option<String> {
    let v = field_value(line, f)?;
    if let Some(s) = v.as_str() {
        return Some(s.trim().to_string());
    }
    if let Some(obj) = v.as_object() {
        if let Some(s) = obj.get("name").and_then(Value::as_str) {
            return Some(s.trim().to_string());
        }
        if let Some(n) = obj.get("value") {
            return Some(n.to_string());
        }
    }
    if v.is_number() || v.is_boolean() {
        return Some(v.to_string());
    }
    None
}

fn field_as_int(v: &Value) -> Option<i64> {
    if let Some(n) = v.as_i64() {
        return Some(n);
    }
    if let Some(obj) = v.as_object()
        && let Some(n) = obj.get("value").and_then(Value::as_i64)
    {
        return Some(n);
    }
    if let Some(s) = v.as_str() {
        return s.parse().ok();
    }
    None
}

/// Pick the schema variant of `pgn` whose proprietary manufacturer `Match`
/// equals `mfr`. Proprietary PGN *numbers* are shared across manufacturers
/// (e.g. 130845 is Maretron **and** Furuno **and** Simnet), and only the
/// manufacturer — from the device's ISO Address Claim NAME, or an override's
/// own proprietary header — says which definition actually applies. The
/// manufacturer is a `Match` on the PGN's first field. Falls back to the
/// first definition when `mfr` is absent or unmatched.
pub fn variant_for(pgn: u32, mfr: Option<u16>) -> Option<&'static PgnInfo> {
    let db = PgnDatabase::embedded(Units::Metric);
    if let Some(m) = mfr
        && let Some(v) = db
            .pgn_variants(pgn)
            .find(|v| v.fields.first().and_then(|f| f.match_value) == Some(m as i64))
    {
        return Some(v);
    }
    db.first_pgn(pgn)
}

/// Walk the `"PGN"` list inside a PGN 126464 record — canboat
/// renders it as `fields.list: [{ "PGN": <n> }, ...]`.
fn collect_pgn_list(line: &Value) -> Vec<u32> {
    let Some(arr) = line.pointer("/fields/list").and_then(Value::as_array) else {
        return Vec::new();
    };
    arr.iter()
        .filter_map(|elem| elem.pointer("/PGN").and_then(field_as_int))
        .filter_map(|n| u32::try_from(n).ok())
        .collect()
}

/// Escape a single JSON Pointer reference token per RFC 6901: `~` →
/// `~0`, `/` → `~1`. canboat field names are mostly ASCII without
/// either, but a few like `"Installation Description #1"` survive
/// unchanged; do this anyway so we never silently miss a field.
fn json_pointer_escape(s: &str) -> String {
    s.replace('~', "~0").replace('/', "~1")
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn state() -> AppState {
        AppState::new(Status::new_live("t".into(), 0, 0), HashMap::new())
    }

    fn report(src: u8, sentence: &str, muted: u8) -> Value {
        json!({
            "pgn": 262657,
            "fields": {"Function": 0, "Source": src, "Sentence": sentence, "Muted": muted}
        })
    }

    #[test]
    fn camel_record_normalizes_to_bare_and_reads() {
        // A `server --camel` Product Information record: wrapped under the
        // pgn id, fields keyed by camelCase id. The TUI must read it the
        // same as a bare `-json` record.
        let mut s = state();
        let camel = json!({
            "productInformation": {
                "prio": 6, "src": 10, "pgn": 126996,
                "description": "productInformation",
                "fields": {
                    "modelId": "ACME Radar",
                    "softwareVersionCode": "1.2.3",
                    "certificationLevel": 2
                }
            }
        });
        s.upsert(126996, 10, None, "productInformation".into(), camel);

        // Device view pulls model/software out via the human-name readers.
        let devs = s.device_list();
        let dev = devs.iter().find(|d| d.src == 10).expect("device src 10");
        assert_eq!(dev.model, "ACME Radar");
        assert_eq!(dev.software, "1.2.3");

        // Stored line is unwrapped, name-keyed, and description restored.
        let entry = s.entries.values().next().expect("one entry");
        assert_eq!(entry.description, "Product Information");
        assert_eq!(
            entry
                .line
                .pointer("/fields/Model ID")
                .and_then(Value::as_str),
            Some("ACME Radar")
        );
        assert!(
            entry.line.get("productInformation").is_none(),
            "wrapper stripped"
        );
    }

    #[test]
    fn flat_camel_record_normalizes_to_bare_and_reads() {
        // The default `server` shape: camelCase field keys on an
        // *unwrapped* record. There is no wrapper to key off, so the
        // normalizer has to rekey from the field ids — before it did,
        // every name-keyed reader in the TUI came up empty.
        let mut s = state();
        let flat = json!({
            "prio": 6, "src": 11, "pgn": 126996,
            "description": "Product Information",
            "fields": {
                "modelId": "ACME Radar",
                "softwareVersionCode": "1.2.3",
                "certificationLevel": 2
            }
        });
        s.upsert(126996, 11, None, "Product Information".into(), flat);

        let devs = s.device_list();
        let dev = devs.iter().find(|d| d.src == 11).expect("device src 11");
        assert_eq!(dev.model, "ACME Radar");
        assert_eq!(dev.software, "1.2.3");

        let entry = s.entries.values().next().expect("one entry");
        assert_eq!(entry.description, "Product Information");
        assert_eq!(
            entry
                .line
                .pointer("/fields/Model ID")
                .and_then(Value::as_str),
            Some("ACME Radar")
        );
    }

    #[test]
    fn spaced_name_record_passes_through_untouched() {
        // `--id spaces` output must not be disturbed by the rekeying.
        let line = json!({
            "prio": 6, "src": 12, "pgn": 126996,
            "description": "Product Information",
            "fields": {"Model ID": "ACME Radar", "Certification Level": 2}
        });
        assert_eq!(normalize_camel(line.clone()), line);
    }

    #[test]
    fn variant_for_resolves_by_manufacturer() {
        // 130845 is shared: Maretron (137) / Furuno (1855) / Simnet (1857).
        // The manufacturer selects the definition — not first-listed.
        assert!(
            variant_for(130845, Some(1855))
                .unwrap()
                .description
                .contains("Furuno")
        );
        assert!(
            variant_for(130845, Some(137))
                .unwrap()
                .description
                .contains("Maretron")
        );
        // Unknown / unmatched manufacturer falls back to the first definition.
        assert!(variant_for(130845, None).is_some());
        assert!(variant_for(130845, Some(9999)).is_some());
    }

    #[test]
    fn camel_repeat_set_element_keys_are_rekeyed() {
        // PGN 126464 (pgnListTransmitAndReceive) with a camel `list`
        // repeat: each element's `pgn` id must be renamed to `PGN` so the
        // PGN-list reader finds it.
        let mut s = state();
        let camel = json!({
            "pgnListTransmitAndReceive": {
                "src": 9, "pgn": 126464, "description": "pgnListTransmitAndReceive",
                "fields": {"functionCode": 0, "list": [{"pgn": 127250}, {"pgn": 128267}]}
            }
        });
        s.upsert(126464, 9, None, "pgnListTransmitAndReceive".into(), camel);
        let lists = s.pgn_lists_for_src(9);
        assert_eq!(lists.tx, vec![127250, 128267]);
    }

    #[test]
    fn bare_record_passes_through_untouched() {
        // A normal `-json` record has no wrapper — normalization is a no-op.
        let mut s = state();
        let bare = json!({
            "prio": 6, "src": 7, "pgn": 126996, "description": "Product Information",
            "fields": {"Model ID": "Bare Model"}
        });
        s.upsert(126996, 7, None, "Product Information".into(), bare);
        let dev = s.device_list().into_iter().find(|d| d.src == 7).unwrap();
        assert_eq!(dev.model, "Bare Model");
    }

    #[test]
    fn nmea0183_report_builds_rows_and_stays_out_of_entries() {
        let mut s = state();
        s.upsert(262657, 0, None, String::new(), report(35, NMEA0183_ALL, 0));
        s.upsert(262657, 0, None, String::new(), report(35, "VHW", 0));
        s.upsert(262657, 0, None, String::new(), report(35, "VLW", 1));
        // Control frames don't pollute the bus/device model.
        assert!(s.entries.is_empty());
        assert!(s.device_list().is_empty());
        let rows = s.nmea0183_rows();
        // header (ALL) + VHW + VLW
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].src, 35);
        assert!(rows[0].sentence.is_none() && !rows[0].muted);
        assert_eq!(rows[1].sentence.as_deref(), Some("VHW"));
        assert!(!rows[1].muted);
        assert_eq!(rows[2].sentence.as_deref(), Some("VLW"));
        assert!(rows[2].muted);
    }

    #[test]
    fn whole_source_mute_updates_header() {
        let mut s = state();
        s.upsert(262657, 0, None, String::new(), report(53, "MWV", 0));
        s.upsert(262657, 0, None, String::new(), report(53, NMEA0183_ALL, 1));
        let rows = s.nmea0183_rows();
        assert!(rows[0].sentence.is_none() && rows[0].muted);
    }

    fn override_report(src: u8, pgn: u32, interval: u32, mfr: i64, industry: i64) -> Value {
        json!({
            "pgn": 262658,
            "fields": {
                "Function": 0, "Source": src, "PGN": pgn, "Interval": interval,
                "Manufacturer Code": mfr, "Industry Code": industry
            }
        })
    }

    #[test]
    fn override_report_populates_overrides_and_stays_out_of_entries() {
        let mut s = state();
        // Standard PGN: mfr/industry carry the "n/a" sentinels.
        s.upsert(
            262658,
            0,
            None,
            String::new(),
            override_report(52, 127250, 100, 0xffff, 0xff),
        );
        // Proprietary PGN silenced (interval 0), real mfr/industry.
        s.upsert(
            262658,
            0,
            None,
            String::new(),
            override_report(52, 130842, 0, 1855, 4),
        );
        // Control frames don't pollute the bus/device model.
        assert!(s.entries.is_empty());
        assert!(s.device_list().is_empty());
        let rows = s.override_rows();
        assert_eq!(rows.len(), 2);
        let heading = rows.iter().find(|r| r.pgn == 127250).unwrap();
        assert_eq!(heading.interval_ms, 100);
        assert_eq!(heading.manufacturer_code, None);
        assert_eq!(heading.industry_code, None);
        assert!(!heading.description.is_empty(), "schema description filled");
        let prop = rows.iter().find(|r| r.pgn == 130842).unwrap();
        assert_eq!(prop.interval_ms, 0);
        assert_eq!(prop.manufacturer_code, Some(1855));
        assert_eq!(prop.industry_code, Some(4));
    }

    #[test]
    fn override_report_updates_in_place_and_forget_removes() {
        let mut s = state();
        s.upsert(
            262658,
            0,
            None,
            String::new(),
            override_report(52, 127250, 100, 0xffff, 0xff),
        );
        assert_eq!(s.override_rows().len(), 1);
        // A fresh report for the same (src, pgn) replaces, not appends.
        s.upsert(
            262658,
            0,
            None,
            String::new(),
            override_report(52, 127250, 250, 0xffff, 0xff),
        );
        let rows = s.override_rows();
        assert_eq!(rows.len(), 1);
        assert_eq!(rows[0].interval_ms, 250);
        // Optimistic delete drops it immediately.
        s.forget_override(52, 127250);
        assert!(s.override_rows().is_empty());
    }

    #[test]
    fn pgn_load_rows_aggregate_across_sources_and_sort_busiest_first() {
        let mut s = state();
        // 129025 seen from two sources (two observations from one of
        // them), 127250 seen once.
        s.upsert(129025, 10, None, "Position".into(), json!({"pgn": 129025}));
        s.upsert(129025, 10, None, "Position".into(), json!({"pgn": 129025}));
        s.upsert(129025, 11, None, "Position".into(), json!({"pgn": 129025}));
        s.upsert(127250, 10, None, "Heading".into(), json!({"pgn": 127250}));
        let rows = s.pgn_load_rows();
        assert_eq!(rows.len(), 2);
        let pos = rows.iter().find(|r| r.pgn == 129025).unwrap();
        assert_eq!(pos.sources, 2);
        assert_eq!(pos.count, 3);
        assert_eq!(pos.description, "Position");
        // Ties on rate (all zero until a cadence is measured) break by
        // count desc, so the 3-count PGN sorts ahead of the 1-count one.
        assert_eq!(rows[0].pgn, 129025);
    }

    #[test]
    fn pgn_load_rate_is_count_over_capture_duration() {
        // A bursty low-count PGN must not out-rate a steady high-count
        // one just because its few frames clustered in time.
        let mut s = state();
        let ts = |ms: i64| json!({"pgn": 1, "timestamp": canboat_core::format_iso_ms(ms as u64)});
        // 100-second capture window: t=0 .. t=100_000ms.
        // Bursty PGN 126996: two frames 1 s apart from each of two srcs.
        s.upsert(126996, 10, None, "Product".into(), ts(0));
        s.upsert(126996, 10, None, "Product".into(), ts(1_000));
        s.upsert(126996, 11, None, "Product".into(), ts(0));
        s.upsert(126996, 11, None, "Product".into(), ts(1_000));
        // Steady PGN 127250: 5 frames spread across the whole window.
        for k in 0..5 {
            s.upsert(127250, 10, None, "Heading".into(), ts(k * 25_000));
        }
        assert_eq!(s.capture_duration_secs(), 100.0);
        let rows = s.pgn_load_rows();
        let bursty = rows.iter().find(|r| r.pgn == 126996).unwrap();
        let steady = rows.iter().find(|r| r.pgn == 127250).unwrap();
        // count / 100 s — not the old summed instantaneous cadence.
        assert!((bursty.rate - 4.0 / 100.0).abs() < 1e-4, "{}", bursty.rate);
        assert!((steady.rate - 5.0 / 100.0).abs() < 1e-4, "{}", steady.rate);
        // Steady (higher count) sorts ahead of bursty.
        assert!(steady.rate > bursty.rate);
    }
}
