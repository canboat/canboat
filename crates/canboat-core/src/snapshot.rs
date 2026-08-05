// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Shared snapshot cache for the canboat C-compatible base-port output.
//!
//! Both the standalone `n2kd` binary and the combined `canboat-pipeline`
//! expose a "full state on connect" TCP port: on a new connection they
//! dump the latest analyzer-JSON line per `(pgn, src, secondary)` tuple
//! and close. canboat C does this in `n2kd/main.c` and the dump shape
//! is:
//!
//! ```text
//! {"<pgn>":
//!   {"description":"<short-desc>"
//!   ,"<src>[_<secondary>]":<analyzer-json-line>
//!   ...
//!   }
//! ,"<pgn>":
//!   ...
//! }
//! ```
//!
//! - `<short-desc>` is the PGN's full description truncated at the
//!   first `:` (so `"Simnet: Device Status"` → `"Simnet"`).
//! - `<secondary>` is the readable value of the first discriminating
//!   field on the record (Instance / Reference / User ID / Message ID
//!   / Proprietary ID) — omitted when none is present.
//! - Lines that have expired (per-PGN-class TTL, see [`ttl_for_pgn`])
//!   are pruned before each snapshot.
//!
//! This module owns the cache, TTL policy, and the nested-JSON
//! emitter. The two binaries differ only in how they classify their
//! input — n2kd scans the analyzer-JSON line as text; canboat-pipeline
//! walks the `DecodedPgn` struct — and they each build a
//! [`SnapshotInput`] from it before calling [`SnapshotStore::store`].

use std::borrow::Cow;
use std::fmt::Write;
use std::sync::Mutex;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use indexmap::IndexMap;

use crate::startup::format_iso_ms;

/// TTL for ordinary sensor PGNs — matches canboat C's `SENSOR_TIMEOUT`.
pub const SENSOR_TTL: Duration = Duration::from_secs(120);

/// TTL for AIS-shaped PGNs — matches canboat C's `AIS_TIMEOUT`. AIS
/// targets seen recently are still considered "near" for an hour.
pub const AIS_TTL: Duration = Duration::from_secs(3600);

/// TTL for ISO Address Claim (PGN 60928). Renamed from canboat C's
/// `CLAIM_TIMEOUT` and bumped from 120 s to 3600 s — once a device has
/// claimed an address it should stay in the snapshot for the same
/// duration we trust AIS targets, even on a quiet bus.
pub const DEVICE_CLAIM_TTL: Duration = Duration::from_secs(3600);

/// PGNs that describe a device's identity and never need to expire
/// from the snapshot: Product Information (126996) and Configuration
/// Information (126998). Entries for these stay live until the
/// process restarts. The corresponding `DEVICE_INFO_TIMEOUT` is
/// represented by [`ttl_for_pgn`] returning `None`.
pub const DEVICE_INFO_PGNS: &[u32] = &[126996, 126998];

/// PGN that triggers [`DEVICE_CLAIM_TTL`].
pub const ISO_ADDRESS_CLAIM_PGN: u32 = 60928;

/// Pick the snapshot TTL for `pgn`. Returns `None` for PGNs in
/// [`DEVICE_INFO_PGNS`] — they never expire.
pub fn ttl_for_pgn(pgn: u32, is_ais: bool) -> Option<Duration> {
    if DEVICE_INFO_PGNS.contains(&pgn) {
        None
    } else if pgn == ISO_ADDRESS_CLAIM_PGN {
        Some(DEVICE_CLAIM_TTL)
    } else if is_ais {
        Some(AIS_TTL)
    } else {
        Some(SENSOR_TTL)
    }
}

/// PGN numbers that identify AIS messages (NMEA 2000 PGNs 129038
/// through 129810 — both the position-report family and the safety /
/// management messages). Used by [`is_ais_pgn`] to assign
/// [`AIS_TTL`] regardless of which discriminator field carries the
/// MMSI. Sorted ascending for binary-search lookup.
///
/// This list replaces the canboat C n2kd "any field named `User ID`
/// or `Message ID` is AIS" proxy — `Message ID` in particular is
/// declared as a primary-key field on dozens of non-AIS PGNs in
/// canboat 7.1.0, so the proxy over-fires today.
pub const AIS_PGNS: &[u32] = &[
    129038, 129039, 129040, 129041, 129793, 129794, 129795, 129796, 129797, 129798, 129801, 129802,
    129803, 129804, 129805, 129806, 129807, 129808, 129809, 129810,
];

/// `true` iff `pgn` identifies an AIS message that should get
/// [`AIS_TTL`] in the snapshot store.
#[inline]
pub fn is_ais_pgn(pgn: u32) -> bool {
    AIS_PGNS.binary_search(&pgn).is_ok()
}

/// Local overrides for PGNs whose canboat-schema definition is
/// missing a `PartOfPrimaryKey` flag they need for the snapshot
/// cache to distinguish records that the spec treats as distinct.
///
/// Entries are `(pgn, field_id)` pairs. The `field_id` is canboat's
/// camelCase identifier matched against [`FieldInfo::id`].
/// [`is_part_of_primary_key`] returns `true` for any (pgn, field)
/// listed here, on top of what the schema already marks.
///
/// Currently empty — every PK fix we've needed has been pushed
/// upstream into `data/canboat.json` (PGN 126464 `functionCode`,
/// the five PGN 65305 Simnet variants' `report` field, …) and is
/// flagged by the schema directly. This table stays in place as
/// the escape hatch for the next "the schema is missing a PK" bug
/// we find in the field.
const PRIMARY_KEY_OVERRIDES: &[(u32, &str)] = &[];

/// `true` iff `field` should be treated as part of `pgn`'s primary
/// key — i.e. the schema says so OR [`PRIMARY_KEY_OVERRIDES`] lists
/// the pair. The composite-key / repeating-PK helpers below all go
/// through this so a single override entry covers every snapshot
/// path.
#[inline]
pub fn is_part_of_primary_key(pgn: u32, field: &crate::FieldInfo) -> bool {
    if field.part_of_primary_key == Some(true) {
        return true;
    }
    PRIMARY_KEY_OVERRIDES
        .iter()
        .any(|&(p, id)| p == pgn && id == field.id)
}

/// Compose the schema-driven secondary discriminator for a record by
/// walking every top-level field marked `PartOfPrimaryKey` on the
/// PGN's [`PgnInfo`], looking each one up via `get_text`, and joining
/// the resolved values with `_`. Returns `None` when the PGN has no
/// top-level primary key, or when none of its top-level PK fields
/// produced a textual value.
///
/// This is canboat-rs's deliberate divergence from canboat C n2kd's
/// "first-name-from-hardcoded-list wins" key extraction (#2). The
/// canboat 7.1.0 schema declares primary keys on 115 PGNs, 26 of
/// which are *composite* (multiple PK fields). The hand-rolled list
/// picked one field, joined nothing, and collided whenever the
/// remaining PK fields varied — e.g. PGN 127509's `Instance + AC
/// Instance + DC Instance`, where two charger inverters on the same
/// nominal Instance would overwrite each other in the cache. The
/// schema-driven composite key gives them separate entries.
///
/// Fields inside a repeating set are intentionally skipped here.
/// PGNs whose PK lives inside a repeating set (PGNs 129796, 129816,
/// 130824 — `Destination ID`, `Key`, …) yield one cache entry per
/// iteration; callers should detect that case via
/// [`repeating_pk_set`] and use [`per_iteration_secondary`] to build
/// each iteration's key.
pub fn composite_secondary<F>(info: &crate::PgnInfo, mut get_text: F) -> Option<String>
where
    F: FnMut(&crate::FieldInfo) -> Option<String>,
{
    let mut parts: Vec<String> = Vec::new();
    for f in info.fields {
        if !is_part_of_primary_key(info.pgn, f) {
            continue;
        }
        if field_in_any_repeat_set(info, f).is_some() {
            continue;
        }
        if let Some(s) = get_text(f) {
            parts.push(s);
        }
    }
    if parts.is_empty() {
        None
    } else {
        Some(parts.join("_"))
    }
}

/// `Some(N)` when the PGN has at least one `PartOfPrimaryKey` field
/// inside repeating field set `N` (1 or 2). `None` when the PGN's
/// PK is purely top-level (the common case — only 4 PGNs in canboat
/// 7.1.0 carry PK fields inside a repeating set: 129796 / 129816 /
/// 130824 [B&G] / 130824 [Mercury]).
///
/// Callers use this to decide between a single cache entry
/// ([`composite_secondary`]) and one cache entry per iteration
/// ([`per_iteration_secondary`]).
pub fn repeating_pk_set(info: &crate::PgnInfo) -> Option<u8> {
    for f in info.fields {
        if !is_part_of_primary_key(info.pgn, f) {
            continue;
        }
        if let Some(set) = field_in_any_repeat_set(info, f) {
            return Some(set);
        }
    }
    None
}

/// Build the secondary discriminator for iteration `iter` of
/// `repeat_set` on a PGN whose PK includes per-iteration fields.
/// Top-level PK values are joined first (in field-`order`), then
/// each iteration-level PK value — exactly the same `_` join as
/// [`composite_secondary`] for a non-repeating PGN.
///
/// `get_top(field)` resolves a top-level field's text. `get_iter
/// (field, iter)` resolves a field's text for iteration `iter` of
/// the repeating set. Returns `None` when every PK field came back
/// `None` for this iteration.
pub fn per_iteration_secondary<F, G>(
    info: &crate::PgnInfo,
    repeat_set: u8,
    iter: u32,
    mut get_top: F,
    mut get_iter: G,
) -> Option<String>
where
    F: FnMut(&crate::FieldInfo) -> Option<String>,
    G: FnMut(&crate::FieldInfo, u32) -> Option<String>,
{
    let mut parts: Vec<String> = Vec::new();
    for f in info.fields {
        if !is_part_of_primary_key(info.pgn, f) {
            continue;
        }
        let resolved = match field_in_any_repeat_set(info, f) {
            None => get_top(f),
            Some(set) if set == repeat_set => get_iter(f, iter),
            Some(_) => None,
        };
        if let Some(s) = resolved {
            parts.push(s);
        }
    }
    if parts.is_empty() {
        None
    } else {
        Some(parts.join("_"))
    }
}

/// `Some(set)` (1 or 2) when `field`'s `order` falls inside one of
/// `pgn`'s declared repeating field sets; `None` for top-level
/// fields. Repeating-set bounds are half-open: `[start, start +
/// size)`.
fn field_in_any_repeat_set(pgn: &crate::PgnInfo, field: &crate::FieldInfo) -> Option<u8> {
    let order = field.order as u32;
    if let (Some(start), Some(size)) = (
        pgn.repeating_field_set1_start_field,
        pgn.repeating_field_set1_size,
    ) && order >= start
        && order < start + size
    {
        return Some(1);
    }
    if let (Some(start), Some(size)) = (
        pgn.repeating_field_set2_start_field,
        pgn.repeating_field_set2_size,
    ) && order >= start
        && order < start + size
    {
        return Some(2);
    }
    None
}

/// Classify one analyzer-JSON line into one or more
/// [`SnapshotInput`]s and pass each to `sink`. Returns the number of
/// records emitted (0 when the line is missing `pgn` or `src`).
///
/// This is the shared ingest path used by:
///
/// * `n2kd::Hub::store` — feeds the TCP snapshot port.
/// * `canboat-tui` — keeps the on-screen device → PGN map current.
///
/// Splitting / keying logic matches the pipeline snapshot 1:1:
///
/// * PGNs with a top-level `PartOfPrimaryKey` set get one
///   [`SnapshotInput`] per line, with `secondary` joined via
///   [`composite_secondary`].
/// * PGNs whose primary key lives inside a repeating set
///   ([`repeating_pk_set`] returns `Some`) get one
///   [`SnapshotInput`] per iteration of the `"list"` (set 1) or
///   `"list2"` (set 2) array, with the iteration spliced into the
///   stored line so each cache entry shows only its own iteration's
///   fields. PGNs 130824 (B&G + Mercury), 129796, 129816 are the
///   canboat 7.1.0 members of this set.
/// * Lines whose repeating set is empty fall back to the
///   single-record path with the top-level PK only.
pub fn classify_json_line<F>(line: &str, mut sink: F) -> usize
where
    F: FnMut(SnapshotInput),
{
    let Some(pgn) = crate::analyzer_json::int(line, "pgn") else {
        return 0;
    };
    let pgn = pgn as u32;
    let Some(src) = crate::analyzer_json::int(line, "src") else {
        return 0;
    };
    let src = src as u8;
    let description = crate::analyzer_json::value(line, "description")
        .unwrap_or("")
        .to_string();
    let is_ais = is_ais_pgn(pgn);
    let info = crate::PgnDatabase::embedded(crate::Units::Metric).first_pgn(pgn);
    // Records key fields by camelCase `id` or by human `name`. A
    // wrapper (`--wrap` / canboat C's `-camel`) always means camel;
    // otherwise sniff, since camelCase unwrapped is the default. Pick
    // the matching secondary-key field key so the snapshot stays
    // correctly split per instance / function code.
    let use_camel = crate::analyzer_json::camel_wrapper_id(line).is_some()
        || info.is_some_and(|i| crate::analyzer_json::uses_camel_keys(line, i.fields));
    let field_key = move |f: &crate::FieldInfo| if use_camel { f.id } else { f.name };
    let repeat_set = info.and_then(repeating_pk_set);

    let Some(rs) = repeat_set else {
        let secondary = info.and_then(|info| {
            composite_secondary(info, |f| {
                crate::analyzer_json::lookup_text(line, field_key(f)).map(|s| s.to_string())
            })
        });
        sink(SnapshotInput {
            pgn,
            src,
            secondary,
            is_ais,
            pgn_description: description,
            line: line.to_string(),
        });
        return 1;
    };

    let list_key = if rs == 1 { "list" } else { "list2" };
    let Some((arr_start, arr_end)) = crate::analyzer_json::array_span(line, list_key) else {
        // Repeating set declared zero iterations — store the line as
        // one record under the top-level PK alone.
        let secondary = info.and_then(|info| {
            composite_secondary(info, |f| {
                crate::analyzer_json::lookup_text(line, field_key(f)).map(|s| s.to_string())
            })
        });
        sink(SnapshotInput {
            pgn,
            src,
            secondary,
            is_ais,
            pgn_description: description,
            line: line.to_string(),
        });
        return 1;
    };

    let arr = &line[arr_start..arr_end];
    let elements = crate::analyzer_json::split_object_array(arr);
    let pgn_info = info.expect("repeat_set source implies PgnInfo present");
    let mut emitted = 0usize;
    for (iter, elem) in elements.iter().enumerate() {
        let secondary = per_iteration_secondary(
            pgn_info,
            rs,
            iter as u32,
            |f| crate::analyzer_json::lookup_text(line, field_key(f)).map(|s| s.to_string()),
            |f, _| crate::analyzer_json::lookup_text(elem, field_key(f)).map(|s| s.to_string()),
        );
        let mut spliced = String::with_capacity(line.len() - arr.len() + elem.len() + 2);
        spliced.push_str(&line[..arr_start]);
        spliced.push('[');
        spliced.push_str(elem);
        spliced.push(']');
        spliced.push_str(&line[arr_end..]);
        sink(SnapshotInput {
            pgn,
            src,
            secondary,
            is_ais,
            pgn_description: description.clone(),
            line: spliced,
        });
        emitted += 1;
    }
    emitted
}

/// One record to be stashed in the snapshot cache. The caller is
/// responsible for classifying its input (analyzer-JSON line or
/// `DecodedPgn`) and producing this struct.
#[derive(Debug, Clone)]
pub struct SnapshotInput {
    /// PGN number.
    pub pgn: u32,
    /// Source address on the N2K bus.
    pub src: u8,
    /// Schema-driven secondary discriminator from
    /// [`composite_secondary`] — `_`-joined across every
    /// `PartOfPrimaryKey` field declared on the PGN, or `None` when
    /// the PGN has no primary key.
    pub secondary: Option<String>,
    /// `true` when [`is_ais_pgn`] flags this PGN. Drives the longer
    /// [`AIS_TTL`] and routes the record to the AIS snapshot port.
    pub is_ais: bool,
    /// The PGN's full description, e.g. `"Simnet: Device Status"`.
    /// The store truncates at the first `:` for the snapshot wrapper.
    pub pgn_description: String,
    /// The analyzer-JSON line for this record (no trailing newline).
    pub line: String,
}

/// The analyzer-JSON value stored per cache key. Either a string that
/// was already serialized by the caller (the n2kd path, which reads
/// JSON lines off stdin), or a closure that serializes on demand (the
/// pipeline path, which holds a `DecodedPgn` and defers `write_json`
/// until a snapshot client actually connects — so a busy bus with no
/// snapshot consumer never pays the serialization tax).
enum LinePayload {
    Ready(String),
    Lazy(Box<dyn Fn() -> String + Send>),
}

impl LinePayload {
    fn render(&self) -> Cow<'_, str> {
        match self {
            LinePayload::Ready(s) => Cow::Borrowed(s),
            LinePayload::Lazy(f) => Cow::Owned(f()),
        }
    }
}

struct CacheEntry {
    payload: LinePayload,
    /// `pgn_description` truncated at the first `:`.
    pgn_short_description: String,
    /// `None` means "never expires" — set for PGNs in
    /// [`DEVICE_INFO_PGNS`].
    expires_at: Option<Instant>,
    /// Most recent record's `"timestamp":` field — emitted verbatim
    /// as `"last":` in the status port output.
    /// Wall-clock millis of the most recent store under this key,
    /// formatted to ISO only when the status port actually reads.
    last_timestamp: Option<u64>,
    /// Number of records stashed under this key so far.
    count: u64,
    /// Monotonic `Instant` of the previous [`SnapshotStore::store`]
    /// call for this key. Used to compute [`Self::interval_ms`] when
    /// the next record arrives.
    previous_store_at: Option<Instant>,
    /// Milliseconds between the two most recent records under this
    /// key (i.e. `now - previous_store_at` evaluated at the last
    /// `store` call). `0` until the second record arrives — matches
    /// canboat C n2kd, where `m_interval` starts at 0 and updates on
    /// each subsequent store.
    interval_ms: u64,
}

/// `(pgn, src, secondary_text)`.
type CacheKey = (u32, u8, Option<String>);

/// Shared snapshot cache. Thread-safe (interior `Mutex`).
///
/// Uses [`IndexMap`] so the snapshot emitter walks entries in
/// insertion order — matching canboat C n2kd's `pgnList[]` first-seen
/// ordering. The first record for a given PGN fixes its position;
/// subsequent records under that key update the value in place.
pub struct SnapshotStore {
    cache: Mutex<IndexMap<CacheKey, CacheEntry>>,
    /// Wall-clock reference captured once at construction. Per-record
    /// `last_timestamp` is derived as `epoch_wall_ms + (now -
    /// epoch_instant)` from the caller-supplied monotonic `now`, so the
    /// hot `store` path never issues its own `SystemTime::now()` syscall
    /// (a real trap under musl). A few ms of drift vs. wall clock over a
    /// session is immaterial for a "last seen" display value.
    epoch_instant: Instant,
    epoch_wall_ms: u64,
}

impl SnapshotStore {
    pub fn new() -> Self {
        let epoch_wall_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0);
        Self {
            cache: Mutex::new(IndexMap::new()),
            epoch_instant: Instant::now(),
            epoch_wall_ms,
        }
    }

    /// Stash one classified record whose analyzer-JSON line is already
    /// serialized (the n2kd path). Overwrites any prior entry with the
    /// same `(pgn, src, secondary)` key in place — insertion order is
    /// preserved. Status fields (`count`, `interval_ms`) accumulate
    /// across stores for the same key.
    pub fn store(&self, input: SnapshotInput) {
        // n2kd path (low rate, one JSON line per store): capture `now`
        // here. The hot pipeline path uses [`Self::store_lazy`], which
        // takes a `now` threaded from its record loop instead.
        self.store_entry(
            input.pgn,
            input.src,
            input.secondary,
            input.is_ais,
            &input.pgn_description,
            LinePayload::Ready(input.line),
            Instant::now(),
        );
    }

    /// Stash one record whose analyzer-JSON line is produced lazily by
    /// `render` — called only when a snapshot client actually reads the
    /// cache. The pipeline uses this to keep the cache current without
    /// serializing every decoded record on a busy bus. `now` is the
    /// monotonic timestamp the caller already computed for this record,
    /// reused here so the store issues no clock syscall of its own. Same
    /// keying and status accounting as [`Self::store`].
    #[allow(clippy::too_many_arguments)]
    pub fn store_lazy(
        &self,
        pgn: u32,
        src: u8,
        secondary: Option<String>,
        is_ais: bool,
        pgn_description: &str,
        render: Box<dyn Fn() -> String + Send>,
        now: Instant,
    ) {
        self.store_entry(
            pgn,
            src,
            secondary,
            is_ais,
            pgn_description,
            LinePayload::Lazy(render),
            now,
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn store_entry(
        &self,
        pgn: u32,
        src: u8,
        secondary: Option<String>,
        is_ais: bool,
        pgn_description: &str,
        payload: LinePayload,
        now: Instant,
    ) {
        let expires_at = ttl_for_pgn(pgn, is_ais).map(|ttl| now + ttl);
        let key = (pgn, src, secondary);
        let mut guard = self.cache.lock().expect("snapshot cache poisoned");

        // Carry count + previous_store_at forward across overwrites
        // so the status port's count/interval reflect the full history
        // for this (pgn, src, secondary) — not just the latest record.
        let prev_count = guard.get(&key).map(|e| e.count).unwrap_or(0);
        let prev_store_at = guard.get(&key).and_then(|e| e.previous_store_at);
        let interval_ms = match prev_store_at {
            Some(t) => now.saturating_duration_since(t).as_millis() as u64,
            None => 0,
        };

        // canboat C uses the wall-clock time of store as the status
        // port's `"last":` field (`m->m_last = now`). Derive it from the
        // construction-time wall/monotonic epoch + this record's `now`,
        // avoiding a per-record `SystemTime::now()` syscall. Formatting
        // to ISO is deferred to the (rare) status read.
        let wall_ms = self.epoch_wall_ms.saturating_add(
            now.saturating_duration_since(self.epoch_instant)
                .as_millis() as u64,
        );

        let entry = CacheEntry {
            payload,
            pgn_short_description: short_description(pgn_description),
            expires_at,
            last_timestamp: Some(wall_ms),
            count: prev_count + 1,
            previous_store_at: Some(now),
            interval_ms,
        };
        guard.insert(key, entry);
    }

    /// Build the canboat-C-compatible nested JSON dump.
    ///
    /// canboat C's JSON snapshot port (CLIENT_JSON) is the complement
    /// of its AIS port: emit records whose description does NOT start
    /// with `"AIS"`, plus the always-on special cases PGN 129026
    /// (COG/SOG, Rapid Update) and PGN 129029 (Position, Rapid
    /// Update). The comment in `n2kd/main.c:456` reads "AIS data only
    /// goes to AIS clients, non-AIS data to non-AIS clients, but PRNs
    /// 129026 and 129029 go to both."
    ///
    /// Expired entries are pruned in-place under the same lock.
    /// Returns the document as one big `String` ending in `}\n` (or
    /// `\n` if the cache is empty).
    pub fn snapshot(&self) -> String {
        self.filtered_snapshot(|pgn, entry| {
            *pgn == 129026 || *pgn == 129029 || !entry.pgn_short_description.starts_with("AIS")
        })
    }

    /// Build the canboat-C-compatible AIS snapshot dump: same shape
    /// as [`Self::snapshot`] but filtered to AIS records.
    ///
    /// canboat C n2kd routes a PGN to the AIS port when:
    ///
    /// * its description starts with `"AIS"`, OR
    /// * the PGN is 129026 (COG & SOG, Rapid Update), OR
    /// * the PGN is 129029 (Position, Rapid Update).
    ///
    /// (See the `(stream == CLIENT_AIS) == (strncmp(desc,"AIS",3)==0)
    /// || pgn == 129026 || pgn == 129029` predicate in
    /// `n2kd/main.c:458`.)
    pub fn ais_snapshot(&self) -> String {
        self.filtered_snapshot(|pgn, entry| {
            *pgn == 129026 || *pgn == 129029 || entry.pgn_short_description.starts_with("AIS")
        })
    }

    /// Internal helper: nested-JSON dump like [`Self::snapshot`] but
    /// only emits entries whose `(pgn, entry)` passes `keep`.
    fn filtered_snapshot<F>(&self, keep: F) -> String
    where
        F: Fn(&u32, &CacheEntry) -> bool,
    {
        let now = Instant::now();
        let mut guard = self.cache.lock().expect("snapshot cache poisoned");
        guard.retain(|_, v| v.expires_at.is_none_or(|t| t > now));

        type GroupKey<'a> = (u8, &'a Option<String>);
        let mut by_pgn: IndexMap<u32, Vec<(GroupKey<'_>, &CacheEntry)>> = IndexMap::new();
        for ((pgn, src, sec), entry) in guard.iter() {
            if !keep(pgn, entry) {
                continue;
            }
            by_pgn.entry(*pgn).or_default().push(((*src, sec), entry));
        }

        let db = crate::PgnDatabase::embedded(crate::Units::Metric);
        let mut out = String::with_capacity(2048);
        let mut first_pgn = true;
        for (pgn, entries) in by_pgn.iter() {
            let desc = &entries[0].1.pgn_short_description;
            // Render each stored line once; `-camel` records are wrapped
            // `{"<pgnId>":{…}}`, bare `-json` is not. When they're camel,
            // key the group by the pgn's camelCase id (which we already
            // have — no need to repeat the numeric PGN as we descend) and
            // unwrap each record; otherwise keep canboat C's numeric-key
            // layout verbatim.
            let rendered: Vec<Cow<'_, str>> =
                entries.iter().map(|(_, e)| e.payload.render()).collect();
            let camel_id = crate::analyzer_json::camel_wrapper_id(&rendered[0]);
            let full_desc = camel_id
                .and_then(|id| db.pgn_by_id(id))
                .map_or(desc.as_str(), |info| info.description);
            if first_pgn {
                out.push_str("{\"");
                first_pgn = false;
            } else {
                out.push_str("\n,\"");
            }
            match camel_id {
                Some(id) => out.push_str(id),
                None => {
                    let _ = write!(out, "{pgn}");
                }
            }
            out.push_str("\":\n  {\"description\":");
            write_json_string(&mut out, desc);
            for (((src, sec), _), payload) in entries.iter().zip(rendered.iter()) {
                out.push_str("\n  ,\"");
                let _ = write!(out, "{src}");
                if let Some(s) = sec.as_deref() {
                    out.push('_');
                    out.push_str(s);
                }
                out.push_str("\":");
                match camel_id {
                    Some(id) => out.push_str(&unwrap_camel_record(payload, id, full_desc)),
                    None => out.push_str(payload),
                }
            }
            out.push_str("\n  }");
        }
        if first_pgn {
            out.push('\n');
        } else {
            out.push_str("\n}\n");
        }
        out
    }

    /// Build the canboat-C-compatible status dump: same per-PGN
    /// nested wrapper as [`Self::snapshot`], but each `<src>[_<sec>]`
    /// value is `{"last":..,"interval":..,"count":..}` describing the
    /// receive cadence rather than the latest line. Mirrors canboat
    /// C n2kd's `CLIENT_STATUS_STREAM` path in
    /// `n2kd/main.c:424-447`.
    ///
    /// Like [`Self::snapshot`], expired entries are pruned in-place
    /// and the document ends in `}\n` (or `\n` when the cache is
    /// empty).
    pub fn status_snapshot(&self) -> String {
        let now = Instant::now();
        let mut guard = self.cache.lock().expect("snapshot cache poisoned");
        guard.retain(|_, v| v.expires_at.is_none_or(|t| t > now));

        type GroupKey<'a> = (u8, &'a Option<String>);
        let mut by_pgn: IndexMap<u32, Vec<(GroupKey<'_>, &CacheEntry)>> = IndexMap::new();
        for ((pgn, src, sec), entry) in guard.iter() {
            by_pgn.entry(*pgn).or_default().push(((*src, sec), entry));
        }

        let mut out = String::with_capacity(4096);
        let mut first_pgn = true;
        for (pgn, entries) in by_pgn.iter() {
            let desc = &entries[0].1.pgn_short_description;
            if first_pgn {
                out.push_str("{\"");
                first_pgn = false;
            } else {
                out.push_str("\n,\"");
            }
            let _ = write!(out, "{pgn}\":\n  {{\"description\":");
            write_json_string(&mut out, desc);
            for ((src, sec), entry) in entries {
                out.push_str("\n  ,\"");
                let _ = write!(out, "{src}");
                if let Some(s) = sec.as_deref() {
                    out.push('_');
                    out.push_str(s);
                }
                out.push_str("\":{\"last\":");
                // canboat C always emits "last" as a quoted string,
                // even when the source line had no timestamp — it
                // falls back to "" in that case (a stored record
                // would always have one in practice). The wall-clock
                // millis are formatted to ISO here, on read, rather
                // than per-record at store time.
                let last = entry.last_timestamp.map(format_iso_ms).unwrap_or_default();
                write_json_string(&mut out, &last);
                let _ = write!(
                    out,
                    ",\"interval\":{},\"count\":{}}}",
                    entry.interval_ms, entry.count
                );
            }
            out.push_str("\n  }");
        }
        if first_pgn {
            out.push('\n');
        } else {
            out.push_str("\n}\n");
        }
        out
    }

    /// Number of live entries (no pruning).
    pub fn len(&self) -> usize {
        self.cache.lock().expect("snapshot cache poisoned").len()
    }

    /// `true` when the cache contains no live entries (no pruning).
    pub fn is_empty(&self) -> bool {
        self.cache
            .lock()
            .expect("snapshot cache poisoned")
            .is_empty()
    }
}

impl Default for SnapshotStore {
    fn default() -> Self {
        Self::new()
    }
}

/// Truncate `desc` at the first `:` to produce the PGN-family name
/// canboat C uses for the snapshot's `"description"` field.
pub fn short_description(desc: &str) -> String {
    match desc.find(':') {
        Some(idx) => desc[..idx].to_string(),
        None => desc.to_string(),
    }
}

/// Turn a `-camel` analyzer payload (`{"<id>":{…}}`) into the logical
/// snapshot record shape: strip the redundant `{"<id>":…}` wrapper (the
/// id is already the snapshot's group key) and replace the record's
/// `"description":"<id>"` — which `-camel` sets to the pgn id — with the
/// human-readable `full_desc`. Field keys stay camelCase.
///
/// Falls back to the payload verbatim if it doesn't have the expected
/// wrapper (defensive; every camel line the store holds does).
fn unwrap_camel_record(payload: &str, camel_id: &str, full_desc: &str) -> String {
    let prefix = format!("{{\"{camel_id}\":");
    let Some(inner) = payload.trim().strip_prefix(&prefix) else {
        return payload.to_string();
    };
    // Drop the wrapper's balancing `}` to expose the bare record object.
    let inner = inner.strip_suffix('}').unwrap_or(inner);
    let needle = format!("\"description\":\"{camel_id}\"");
    let mut desc_json = String::new();
    write_json_string(&mut desc_json, full_desc);
    inner.replacen(&needle, &format!("\"description\":{desc_json}"), 1)
}

/// Write `s` as a JSON string literal (`"`, `\`, and control chars
/// escaped; the rest goes verbatim).
fn write_json_string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            _ => out.push(c),
        }
    }
    out.push('"');
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeating_pk_set_detects_pgn_130824() {
        // canboat 7.1.0 lists exactly four PGNs with PartOfPrimaryKey
        // inside a repeating set — the B&G and Mercury 130824 variants
        // plus 129796 / 129816. PGN 130824 is the one driving this
        // feature (one cache entry per Key/Value pair instead of all
        // pairs collapsing into a single src-keyed entry).
        let info = crate::PgnDatabase::embedded(crate::Units::Metric)
            .first_pgn(130824)
            .expect("PGN 130824 present in embedded db");
        assert_eq!(repeating_pk_set(info), Some(1));
    }

    #[test]
    fn repeating_pk_set_returns_none_for_top_level_pk() {
        // PGN 127251 (Rate of Turn) declares no primary key at all.
        let info = crate::PgnDatabase::embedded(crate::Units::Metric)
            .first_pgn(127251)
            .expect("PGN 127251 present in embedded db");
        assert_eq!(repeating_pk_set(info), None);
        // PGN 127509 (Inverter Status): composite top-level PK
        // (`Instance`, `AC Instance`, `DC Instance`), no repeating
        // set involved.
        let info = crate::PgnDatabase::embedded(crate::Units::Metric)
            .first_pgn(127509)
            .expect("PGN 127509 present in embedded db");
        assert_eq!(repeating_pk_set(info), None);
    }

    #[test]
    fn per_iteration_secondary_joins_top_and_iter_values() {
        // PGN 129796 (AIS Acknowledge): `Source ID` is top-level PK,
        // `Destination ID` lives in repeating set 1 — composite key
        // is `<Source ID>_<Destination ID>`.
        let info = crate::PgnDatabase::embedded(crate::Units::Metric)
            .first_pgn(129796)
            .expect("PGN 129796 present in embedded db");
        let set = repeating_pk_set(info).expect("129796 has repeating PK");
        let key = per_iteration_secondary(
            info,
            set,
            2,
            |f| (f.name == "Source ID").then(|| "111".to_string()),
            |f, iter| (f.name == "Destination ID").then(|| format!("dst{iter}")),
        );
        assert_eq!(key.as_deref(), Some("111_dst2"));
    }

    #[test]
    fn classify_json_line_emits_single_record_with_composite_secondary() {
        // PGN 127501 (Binary Switch Bank Status) — top-level PK is
        // `Instance`. composite_secondary must resolve it to "0".
        let line = r#"{"pgn":127501,"src":17,"description":"Binary Switch Bank Status","fields":{"Instance":0}}"#;
        let mut got = Vec::new();
        let n = classify_json_line(line, |input| got.push(input));
        assert_eq!(n, 1);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].pgn, 127501);
        assert_eq!(got[0].src, 17);
        assert_eq!(got[0].secondary.as_deref(), Some("0"));
        assert!(!got[0].is_ais);
    }

    #[test]
    fn classify_json_line_keys_camel_secondary_by_field_id() {
        // Same PGN 127501 from `analyzer -camel`: wrapped under the pgn
        // id, `Instance` keyed as `instance`. The secondary must still
        // resolve — reading `f.name` ("Instance") would miss the camel
        // key and collapse every instance into one cache slot.
        let line = r#"{"binarySwitchBankStatus":{"pgn":127501,"src":17,"description":"binarySwitchBankStatus","fields":{"instance":3}}}"#;
        let mut got = Vec::new();
        let n = classify_json_line(line, |input| got.push(input));
        assert_eq!(n, 1);
        assert_eq!(got[0].pgn, 127501);
        assert_eq!(got[0].src, 17);
        assert_eq!(got[0].secondary.as_deref(), Some("3"));
    }

    #[test]
    fn classify_json_line_splits_repeating_pk_pgn_into_per_iteration_records() {
        // PGN 130824 (B&G key/value pairs) — each iteration of the
        // repeating set must produce its own record with its own
        // secondary, and the spliced line must contain only that
        // iteration's element.
        let line = r#"{"pgn":130824,"src":21,"description":"Manufacturer Proprietary fast-packet addressed","fields":{"Manufacturer Code":381,"Industry Code":4,"list":[{"Key":1,"Value":42},{"Key":2,"Value":99}]}}"#;
        let mut got = Vec::new();
        let n = classify_json_line(line, |input| got.push(input));
        assert_eq!(n, 2, "expected one record per iteration, got {got:?}");
        assert!(got[0].secondary.is_some());
        assert!(got[1].secondary.is_some());
        assert_ne!(
            got[0].secondary, got[1].secondary,
            "per-iteration keys must differ"
        );
        // Spliced line for record 0 must contain only its element.
        assert!(got[0].line.contains("\"Key\":1"));
        assert!(!got[0].line.contains("\"Key\":2"));
        assert!(got[1].line.contains("\"Key\":2"));
        assert!(!got[1].line.contains("\"Key\":1"));
    }

    #[test]
    fn classify_json_line_keys_pgn_126464_on_function_code() {
        // PGN 126464 carries two records per source — Function Code 0
        // (Transmit PGN list) and Function Code 1 (Receive PGN list).
        // The canboat 7.1.0 schema doesn't mark Function Code as
        // PartOfPrimaryKey, so without PRIMARY_KEY_OVERRIDES both
        // records collide on `(126464, src, None)` and only the
        // second received survives in the snapshot — a real-world
        // problem for the TUI's "show what this device transmits"
        // pane.
        let tx = r#"{"pgn":126464,"src":17,"description":"PGN List","fields":{"Function Code":0,"list":[{"PGN":127251}]}}"#;
        let rx = r#"{"pgn":126464,"src":17,"description":"PGN List","fields":{"Function Code":1,"list":[{"PGN":59904}]}}"#;
        let mut tx_inputs = Vec::new();
        classify_json_line(tx, |i| tx_inputs.push(i));
        let mut rx_inputs = Vec::new();
        classify_json_line(rx, |i| rx_inputs.push(i));
        assert_eq!(tx_inputs.len(), 1);
        assert_eq!(rx_inputs.len(), 1);
        // Override produces distinct secondaries — "0" vs "1".
        assert_eq!(tx_inputs[0].secondary.as_deref(), Some("0"));
        assert_eq!(rx_inputs[0].secondary.as_deref(), Some("1"));
        // And the cache stores both side-by-side instead of
        // overwriting.
        let store = SnapshotStore::new();
        store.store(tx_inputs.remove(0));
        store.store(rx_inputs.remove(0));
        assert_eq!(store.len(), 2);
        let dump = store.snapshot();
        assert!(dump.contains("\"17_0\":"), "TX entry missing:\n{dump}");
        assert!(dump.contains("\"17_1\":"), "RX entry missing:\n{dump}");
    }

    #[test]
    fn classify_json_line_returns_zero_when_pgn_or_src_missing() {
        assert_eq!(classify_json_line("{}", |_| {}), 0);
        assert_eq!(classify_json_line(r#"{"pgn":127251}"#, |_| {}), 0);
        assert_eq!(classify_json_line(r#"{"src":7}"#, |_| {}), 0);
    }

    #[test]
    fn ttl_for_device_info_pgns_is_none() {
        assert!(ttl_for_pgn(126996, false).is_none());
        assert!(ttl_for_pgn(126998, false).is_none());
        // is_ais flag is ignored for device-info PGNs.
        assert!(ttl_for_pgn(126996, true).is_none());
    }

    #[test]
    fn ttl_for_iso_address_claim_is_device_claim() {
        assert_eq!(ttl_for_pgn(60928, false), Some(DEVICE_CLAIM_TTL));
        // The device-info override beats DEVICE_CLAIM but isn't
        // reachable for PGN 60928 — sanity-check the dispatch order
        // stays deterministic.
        assert_eq!(ttl_for_pgn(60928, true), Some(DEVICE_CLAIM_TTL));
    }

    #[test]
    fn ttl_for_ais_record_is_ais_ttl() {
        assert_eq!(ttl_for_pgn(129038, true), Some(AIS_TTL));
        assert_eq!(ttl_for_pgn(129039, true), Some(AIS_TTL));
    }

    #[test]
    fn ttl_for_anything_else_is_sensor_ttl() {
        assert_eq!(ttl_for_pgn(127251, false), Some(SENSOR_TTL));
        assert_eq!(ttl_for_pgn(127257, false), Some(SENSOR_TTL));
    }

    #[test]
    fn device_claim_ttl_is_3600s() {
        assert_eq!(DEVICE_CLAIM_TTL.as_secs(), 3600);
    }

    #[test]
    fn short_description_trims_at_colon() {
        assert_eq!(short_description("Simnet: Device Status"), "Simnet");
        assert_eq!(short_description("Navico: Unknown 1"), "Navico");
        assert_eq!(short_description("Rudder"), "Rudder");
        assert_eq!(short_description("ISO Address Claim"), "ISO Address Claim");
    }

    #[test]
    fn snapshot_empty_returns_blank_line() {
        let s = SnapshotStore::new();
        assert_eq!(s.snapshot(), "\n");
        assert_eq!(s.len(), 0);
    }

    #[test]
    fn snapshot_wraps_one_entry_in_nested_object() {
        let s = SnapshotStore::new();
        s.store(SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: r#"{"pgn":127251,"src":7,"fields":{"Rate":0}}"#.to_string(),
        });
        let dump = s.snapshot();
        assert!(dump.starts_with("{\"127251\":\n  {\"description\":\"Rate of Turn\""));
        assert!(dump.contains("\"7\":{\"pgn\":127251,\"src\":7,\"fields\":{\"Rate\":0}}"));
        assert!(dump.ends_with("}\n"));
    }

    /// `server --camel` stores `{"<pgnId>":{…}}` lines. The snapshot
    /// keys the group by that camelCase id (not the numeric PGN), unwraps
    /// each record, and restores the human `description` (which `-camel`
    /// otherwise renders as the id). Field keys stay camelCase.
    #[test]
    fn snapshot_camel_keys_by_id_and_unwraps_record() {
        let s = SnapshotStore::new();
        s.store(SnapshotInput {
            pgn: 60928,
            src: 51,
            secondary: None,
            is_ais: false,
            pgn_description: "ISO Address Claim".to_string(),
            line: r#"{"isoAddressClaim":{"prio":6,"src":51,"dst":255,"pgn":60928,"description":"isoAddressClaim","fields":{"uniqueNumber":1605586}}}"#.to_string(),
        });
        let expected = r#"{"isoAddressClaim":
  {"description":"ISO Address Claim"
  ,"51":{"prio":6,"src":51,"dst":255,"pgn":60928,"description":"ISO Address Claim","fields":{"uniqueNumber":1605586}}
  }
}
"#;
        assert_eq!(s.snapshot(), expected);
    }

    /// The two producer models the daemons feed one store with must
    /// coexist and render together: n2kd hands over an already-
    /// serialized line via [`SnapshotStore::store`] (`Ready`), while the
    /// pipeline defers `write_json` of a `DecodedPgn` via
    /// [`SnapshotStore::store_lazy`] (`Lazy`). This is the contract the
    /// serving convergence stands on — so it is worth pinning:
    ///
    /// * the `Ready` line is served **byte-identically** (n2kd's
    ///   JSON-port parity depends on the input line passing through
    ///   untouched — not a re-serialization);
    /// * the `Lazy` closure runs **only when a reader asks**, and then
    ///   exactly once per read;
    /// * both keys appear in the same `snapshot()` dump.
    #[test]
    fn ready_and_lazy_producers_share_one_store() {
        use std::sync::Arc;
        use std::sync::atomic::{AtomicUsize, Ordering};

        let s = SnapshotStore::new();

        // n2kd path: the exact JSON line, stored verbatim.
        let ready_line = r#"{"pgn":127251,"src":7,"fields":{"Rate":0}}"#;
        s.store(SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: ready_line.to_string(),
        });

        // pipeline path: serialization deferred to read time.
        let renders = Arc::new(AtomicUsize::new(0));
        let lazy_line = r#"{"pgn":127245,"src":9,"fields":{"Position":1.5}}"#;
        let r = Arc::clone(&renders);
        s.store_lazy(
            127245,
            9,
            None,
            false,
            "Rudder",
            Box::new(move || {
                r.fetch_add(1, Ordering::Relaxed);
                lazy_line.to_string()
            }),
            Instant::now(),
        );

        assert_eq!(
            renders.load(Ordering::Relaxed),
            0,
            "Lazy payload must not serialize before a reader connects"
        );

        let dump = s.snapshot();

        assert!(
            dump.contains(ready_line),
            "Ready line must be served byte-for-byte:\n{dump}"
        );
        assert!(
            dump.contains(lazy_line),
            "Lazy record must render into the same dump:\n{dump}"
        );
        assert_eq!(
            renders.load(Ordering::Relaxed),
            1,
            "Lazy payload renders exactly once, on read"
        );
    }

    #[test]
    fn snapshot_keys_src_with_secondary_when_present() {
        // Non-AIS PGN with an Instance secondary so the entry isn't
        // filtered out of the JSON snapshot dump.
        let s = SnapshotStore::new();
        s.store(SnapshotInput {
            pgn: 127501,
            src: 17,
            secondary: Some("0".to_string()),
            is_ais: false,
            pgn_description: "Binary Switch Bank Status".to_string(),
            line: r#"{"pgn":127501,"src":17}"#.to_string(),
        });
        let dump = s.snapshot();
        assert!(
            dump.contains("\"17_0\":"),
            "expected src_secondary key, got:\n{dump}"
        );
        assert!(dump.contains("\"description\":\"Binary Switch Bank Status\""));
    }

    #[test]
    fn snapshot_filters_ais_described_pgns() {
        // canboat C's JSON snapshot port excludes AIS-described PGNs
        // (n2kd/main.c:458). 129026/129029 are special-cased to
        // appear on both the snapshot and AIS ports.
        let s = SnapshotStore::new();
        s.store(SnapshotInput {
            pgn: 129039,
            src: 23,
            secondary: Some("244180106".to_string()),
            is_ais: true,
            pgn_description: "AIS Class B Position Report".to_string(),
            line: r#"{"pgn":129039}"#.to_string(),
        });
        s.store(SnapshotInput {
            pgn: 129026,
            src: 52,
            secondary: None,
            is_ais: false,
            pgn_description: "COG & SOG, Rapid Update".to_string(),
            line: r#"{"pgn":129026}"#.to_string(),
        });
        s.store(SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: r#"{"pgn":127251}"#.to_string(),
        });
        let dump = s.snapshot();
        assert!(
            !dump.contains("\"129039\""),
            "AIS-described PGN leaked into snapshot:\n{dump}"
        );
        assert!(
            dump.contains("\"129026\""),
            "special-cased PGN 129026 missing from snapshot:\n{dump}"
        );
        assert!(
            dump.contains("\"127251\""),
            "non-AIS sensor PGN dropped from snapshot:\n{dump}"
        );
    }

    #[test]
    fn snapshot_walks_pgns_in_insertion_order() {
        let s = SnapshotStore::new();
        let inputs = [
            (65305, 21, "Simnet"),
            (60928, 7, "ISO Address Claim"),
            (127251, 27, "Rate of Turn"),
        ];
        for (pgn, src, desc) in inputs {
            s.store(SnapshotInput {
                pgn,
                src,
                secondary: None,
                is_ais: false,
                pgn_description: desc.to_string(),
                line: format!(r#"{{"pgn":{pgn},"src":{src}}}"#),
            });
        }
        let dump = s.snapshot();
        // First PGN inserted appears first; subsequent PGNs follow in
        // first-seen order. Matches canboat C's pgnList[] ordering.
        let pos_first = dump.find("65305").expect("first pgn present");
        let pos_second = dump.find("60928").expect("second pgn present");
        let pos_third = dump.find("127251").expect("third pgn present");
        assert!(
            pos_first < pos_second && pos_second < pos_third,
            "expected insertion order 65305 < 60928 < 127251, got dump:\n{dump}"
        );
    }

    #[test]
    fn snapshot_preserves_position_when_existing_key_updated() {
        let s = SnapshotStore::new();
        // Insert two PGNs, then update the first with a new line —
        // its position must not move to the end.
        for (pgn, line) in [(65305u32, "first-line"), (127251, "second-line")] {
            s.store(SnapshotInput {
                pgn,
                src: 7,
                secondary: None,
                is_ais: false,
                pgn_description: "x".to_string(),
                line: line.to_string(),
            });
        }
        s.store(SnapshotInput {
            pgn: 65305,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "x".to_string(),
            line: "first-line-updated".to_string(),
        });
        let dump = s.snapshot();
        let pos_first = dump.find("65305").expect("first pgn present");
        let pos_second = dump.find("127251").expect("second pgn present");
        assert!(
            pos_first < pos_second,
            "65305 must still precede 127251 after update; dump:\n{dump}"
        );
        assert!(dump.contains("first-line-updated"));
        assert!(!dump.contains("\"first-line\""));
    }

    #[test]
    fn store_overwrites_same_key() {
        let s = SnapshotStore::new();
        let mut input = SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: "old".to_string(),
        };
        s.store(input.clone());
        input.line = "new".to_string();
        s.store(input);
        assert_eq!(s.len(), 1);
        assert!(s.snapshot().contains("new"));
        assert!(!s.snapshot().contains("old"));
    }

    #[test]
    fn status_snapshot_emits_canboat_c_shape() {
        let s = SnapshotStore::new();
        s.store(SnapshotInput {
            pgn: 65305,
            src: 21,
            secondary: None,
            is_ais: false,
            pgn_description: "Simnet: Device Mode Request".to_string(),
            line: r#"{"pgn":65305}"#.to_string(),
        });
        let status = s.status_snapshot();
        // PGN-family header.
        assert!(status.starts_with("{\"65305\":\n  {\"description\":\"Simnet\""));
        // Per-(src,secondary) entry carries last/interval/count
        // exactly in canboat C's order. `last` is a wall-clock ISO
        // timestamp emitted at store time so we don't assert its
        // exact value; the interval/count fields are deterministic
        // for a first record (0, 1).
        assert!(
            status.contains("\"21\":{\"last\":\""),
            "missing src key with last field: {status}"
        );
        assert!(
            status.contains(",\"interval\":0,\"count\":1}"),
            "missing interval/count tail: {status}"
        );
        assert!(status.ends_with("}\n"));
    }

    #[test]
    fn status_snapshot_count_increments_across_stores() {
        let s = SnapshotStore::new();
        let template = SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: "ignored".to_string(),
        };
        for _ in 0..5 {
            s.store(template.clone());
        }
        assert!(s.status_snapshot().contains("\"count\":5"));
    }

    #[test]
    fn status_snapshot_uses_secondary_in_key() {
        let s = SnapshotStore::new();
        s.store(SnapshotInput {
            pgn: 129039,
            src: 23,
            secondary: Some("244180106".to_string()),
            is_ais: true,
            pgn_description: "AIS Class B Position Report".to_string(),
            line: "{}".to_string(),
        });
        let status = s.status_snapshot();
        assert!(status.contains("\"23_244180106\":{\"last\":"));
    }

    #[test]
    fn ais_snapshot_includes_ais_described_pgns_and_special_pgns() {
        let s = SnapshotStore::new();
        // Non-AIS sensor record — must not appear in the AIS dump.
        s.store(SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: r#"{"pgn":127251}"#.to_string(),
        });
        // AIS-described record (description starts with "AIS").
        s.store(SnapshotInput {
            pgn: 129039,
            src: 23,
            secondary: Some("244180106".to_string()),
            is_ais: true,
            pgn_description: "AIS Class B Position Report".to_string(),
            line: r#"{"pgn":129039}"#.to_string(),
        });
        // PGN 129026 (COG & SOG) — description doesn't start with
        // "AIS" but C still routes it to the AIS port.
        s.store(SnapshotInput {
            pgn: 129026,
            src: 52,
            secondary: None,
            is_ais: false,
            pgn_description: "COG & SOG, Rapid Update".to_string(),
            line: r#"{"pgn":129026}"#.to_string(),
        });
        // PGN 129029 (Position) — same special case.
        s.store(SnapshotInput {
            pgn: 129029,
            src: 52,
            secondary: None,
            is_ais: false,
            pgn_description: "Position, Rapid Update".to_string(),
            line: r#"{"pgn":129029}"#.to_string(),
        });

        let dump = s.ais_snapshot();
        assert!(
            dump.contains("\"129039\""),
            "AIS-described PGN missing:\n{dump}"
        );
        assert!(
            dump.contains("\"129026\""),
            "special-cased PGN 129026 missing:\n{dump}"
        );
        assert!(
            dump.contains("\"129029\""),
            "special-cased PGN 129029 missing:\n{dump}"
        );
        assert!(
            !dump.contains("\"127251\""),
            "non-AIS sensor PGN leaked into ais dump:\n{dump}"
        );
    }

    #[test]
    fn ais_snapshot_empty_returns_blank_line() {
        let s = SnapshotStore::new();
        // Empty cache.
        assert_eq!(s.ais_snapshot(), "\n");
        // Cache has only non-AIS entries — still empty after filter.
        s.store(SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: "{}".to_string(),
        });
        assert_eq!(s.ais_snapshot(), "\n");
    }

    #[test]
    fn status_snapshot_empty_is_blank_line() {
        assert_eq!(SnapshotStore::new().status_snapshot(), "\n");
    }

    /// Helper: realistic 4-record corpus covering the three port
    /// emitters' interesting shapes — multi-PGN, `<src>_<sec>` keys,
    /// AIS-described and special-cased PGNs, mixed timestamps.
    fn fill_canon_corpus(s: &SnapshotStore) {
        s.store(SnapshotInput {
            pgn: 127251,
            src: 7,
            secondary: None,
            is_ais: false,
            pgn_description: "Rate of Turn".to_string(),
            line: r#"{"pgn":127251,"src":7,"fields":{"Rate":0}}"#.to_string(),
        });
        s.store(SnapshotInput {
            pgn: 65305,
            src: 17,
            secondary: None,
            is_ais: false,
            pgn_description: "Simnet: Device Status".to_string(),
            line: r#"{"pgn":65305,"src":17,"fields":{"x":1}}"#.to_string(),
        });
        s.store(SnapshotInput {
            pgn: 129039,
            src: 23,
            secondary: Some("244180106".to_string()),
            is_ais: true,
            pgn_description: "AIS Class B Position Report".to_string(),
            line: r#"{"pgn":129039,"src":23,"fields":{"User ID":"244180106"}}"#.to_string(),
        });
        s.store(SnapshotInput {
            pgn: 129026,
            src: 52,
            secondary: None,
            is_ais: false,
            pgn_description: "COG & SOG, Rapid Update".to_string(),
            line: r#"{"pgn":129026,"src":52,"fields":{"COG":239.6}}"#.to_string(),
        });
    }

    /// Byte-exact lock against canboat C n2kd's JSON snapshot port
    /// output. Any change to the wrapper format, ordering, separators,
    /// key shape, or the AIS-filter predicate will fail loudly here.
    /// PGN 129039 (in `fill_canon_corpus`, description starts with
    /// "AIS") is filtered out — it goes to the AIS port instead.
    #[test]
    fn snapshot_byte_exact_canboat_c_layout() {
        let s = SnapshotStore::new();
        fill_canon_corpus(&s);
        let expected = r#"{"127251":
  {"description":"Rate of Turn"
  ,"7":{"pgn":127251,"src":7,"fields":{"Rate":0}}
  }
,"65305":
  {"description":"Simnet"
  ,"17":{"pgn":65305,"src":17,"fields":{"x":1}}
  }
,"129026":
  {"description":"COG & SOG, Rapid Update"
  ,"52":{"pgn":129026,"src":52,"fields":{"COG":239.6}}
  }
}
"#;
        assert_eq!(s.snapshot(), expected);
    }

    /// Byte-exact lock against canboat C n2kd's AIS port output:
    /// AIS-described PGNs + 129026/129029 only, otherwise identical
    /// to the snapshot wrapper.
    #[test]
    fn ais_snapshot_byte_exact_canboat_c_layout() {
        let s = SnapshotStore::new();
        fill_canon_corpus(&s);
        // 127251 and 65305 must be filtered out; 129039 (AIS-prefix)
        // and 129026 (special-case) stay, in first-seen order.
        let expected = r#"{"129039":
  {"description":"AIS Class B Position Report"
  ,"23_244180106":{"pgn":129039,"src":23,"fields":{"User ID":"244180106"}}
  }
,"129026":
  {"description":"COG & SOG, Rapid Update"
  ,"52":{"pgn":129026,"src":52,"fields":{"COG":239.6}}
  }
}
"#;
        assert_eq!(s.ais_snapshot(), expected);
    }

    /// Byte-exact lock for the status port. `last` is wall-clock at
    /// store time (matches canboat C); redact every `"last":"<iso>"`
    /// to a stable placeholder before comparing so the test stays
    /// deterministic. `interval` is 0 for first-record entries —
    /// each key here is stored exactly once.
    #[test]
    fn status_snapshot_byte_exact_canboat_c_layout() {
        let s = SnapshotStore::new();
        fill_canon_corpus(&s);
        let actual = redact_iso_last(&s.status_snapshot());
        let expected = r#"{"127251":
  {"description":"Rate of Turn"
  ,"7":{"last":"<ts>","interval":0,"count":1}
  }
,"65305":
  {"description":"Simnet"
  ,"17":{"last":"<ts>","interval":0,"count":1}
  }
,"129039":
  {"description":"AIS Class B Position Report"
  ,"23_244180106":{"last":"<ts>","interval":0,"count":1}
  }
,"129026":
  {"description":"COG & SOG, Rapid Update"
  ,"52":{"last":"<ts>","interval":0,"count":1}
  }
}
"#;
        assert_eq!(actual, expected);
    }

    /// Replace every `"last":"<24-char-ISO-timestamp>"` with
    /// `"last":"<ts>"` so the byte-exact status test stays stable.
    fn redact_iso_last(s: &str) -> String {
        // Cheap state machine — find `"last":"`, skip to the next
        // `"`, replace the body. No regex dep.
        let mut out = String::with_capacity(s.len());
        let needle = "\"last\":\"";
        let mut rest = s;
        while let Some(idx) = rest.find(needle) {
            out.push_str(&rest[..idx + needle.len()]);
            let after = &rest[idx + needle.len()..];
            if let Some(end) = after.find('"') {
                out.push_str("<ts>");
                rest = &after[end..];
            } else {
                out.push_str(after);
                return out;
            }
        }
        out.push_str(rest);
        out
    }
}
