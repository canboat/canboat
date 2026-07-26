// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Pipeline-side snapshot adapter.
//!
//! The cache, TTL policy, and nested-JSON emitter all live in
//! [`canboat_core::snapshot`]. This module is the pipeline-flavoured
//! input shim: it walks a [`DecodedPgn`] to extract the discriminator
//! text and AIS flag, then hands a [`SnapshotInput`] to the shared
//! [`canboat_core::snapshot::SnapshotStore`].

use std::sync::Arc;
use std::time::Instant;

use canboat_core::output::{JsonOptions, write_json};
use canboat_core::snapshot::{
    composite_secondary, is_ais_pgn, per_iteration_secondary, repeating_pk_set,
};
use canboat_core::{DecodedField, DecodedPgn, FieldValue, PgnDatabase};

/// Thin wrapper that keeps the existing `SnapshotStore::store(decoded,
/// line)` API used by pipeline callers, while delegating the cache /
/// TTL / emit responsibilities to [`canboat_core::snapshot`].
///
/// Holds the [`JsonOptions`] used to (re-)serialize per-iteration
/// payloads for PGNs whose primary key lives inside a repeating set
/// (PGN 130824 and friends). Those PGNs get one cache entry per
/// iteration, each carrying a freshly synthesized single-iteration
/// JSON line so that subsequent records overwrite the right entry
/// instead of replacing every value with the latest record's bulk
/// payload.
pub struct SnapshotStore {
    inner: Arc<canboat_core::snapshot::SnapshotStore>,
    json_opts: JsonOptions,
}

impl SnapshotStore {
    pub fn new(json_opts: JsonOptions) -> Self {
        Self {
            inner: Arc::new(canboat_core::snapshot::SnapshotStore::new()),
            json_opts,
        }
    }

    /// The shared core store this wraps. Handed to the read-only TCP
    /// listeners in [`crate::n2kd::serving::tcp`], which serve
    /// `snapshot()` / `ais_snapshot()` straight off it — the same store
    /// n2kd feeds directly.
    pub fn core(&self) -> Arc<canboat_core::snapshot::SnapshotStore> {
        Arc::clone(&self.inner)
    }

    /// Stash this decoded record in the snapshot cache. The analyzer
    /// JSON is **not** produced here — the store holds the decoded
    /// record and serializes lazily, only when a snapshot client reads
    /// the cache. On a busy bus with no snapshot consumer connected,
    /// that means the pipeline never serializes JSON for the snapshot
    /// at all; the expensive `write_json` walk is deferred to connect
    /// time (rare).
    ///
    /// When the PGN's primary key lives inside a repeating set
    /// ([`repeating_pk_set`]) the record is split into one cache entry
    /// per iteration, each lazily re-serializing only its own
    /// iteration's fields.
    pub fn store(&self, decoded: &Arc<DecodedPgn>, now: Instant) {
        let is_ais = is_ais_pgn(decoded.pgn);
        let Some(info) = PgnDatabase::embedded(canboat_core::Units::Metric).first_pgn(decoded.pgn)
        else {
            self.store_whole(decoded, None, is_ais, now);
            return;
        };

        if let Some(rs) = repeating_pk_set(info) {
            self.store_per_iteration(decoded, info, rs, is_ais, now);
            return;
        }

        let secondary = composite_secondary(info, |f| top_field_text(decoded, f));
        self.store_whole(decoded, secondary, is_ais, now);
    }

    /// Store the whole record under one key, deferring serialization.
    /// The decoded record is shared with the pipeline via `Arc` — the
    /// closure captures a refcount bump, not a deep clone.
    fn store_whole(
        &self,
        decoded: &Arc<DecodedPgn>,
        secondary: Option<String>,
        is_ais: bool,
        now: Instant,
    ) {
        let d = Arc::clone(decoded);
        let opts = self.json_opts.clone();
        self.inner.store_lazy(
            decoded.pgn,
            decoded.src,
            secondary,
            is_ais,
            decoded.description,
            Box::new(move || {
                let mut s = String::with_capacity(256);
                let _ = write_json(&mut s, &d, &opts);
                s
            }),
            now,
        );
    }

    /// Per-iteration storage path for PGNs whose PK lives in
    /// `repeat_set`. One cache entry is written per iteration; each
    /// entry lazily re-serializes a JSON document containing only that
    /// iteration's repeating-set fields plus all top-level fields, so a
    /// subsequent record refreshes exactly the iteration whose PK
    /// matches and leaves the rest in place. The decoded record is
    /// cloned once and shared (Arc) across the per-iteration closures.
    fn store_per_iteration(
        &self,
        decoded: &Arc<DecodedPgn>,
        info: &canboat_core::PgnInfo,
        rs: u8,
        is_ais: bool,
        now: Instant,
    ) {
        let iters: Vec<u32> = collect_iter_indexes(decoded, rs);
        if iters.is_empty() {
            return;
        }
        for iter in iters {
            let secondary = per_iteration_secondary(
                info,
                rs,
                iter,
                |f| top_field_text(decoded, f),
                |f, ii| iter_field_text(decoded, rs, ii, f),
            );
            let d = Arc::clone(decoded);
            let opts = self.json_opts.clone();
            self.inner.store_lazy(
                decoded.pgn,
                decoded.src,
                secondary,
                is_ais,
                decoded.description,
                Box::new(move || synthesize_iter_line(&d, rs, iter, &opts)),
                now,
            );
        }
    }

    /// Number of live entries (no pruning). For tests / future
    /// `--status` endpoint.
    #[allow(dead_code)]
    pub fn len(&self) -> usize {
        self.inner.len()
    }
}

/// Render `f` (a top-level field of `decoded`) as discriminator text
/// for the snapshot key. Returns `None` when the field wasn't
/// decoded or its variant carries no useful text.
fn top_field_text(decoded: &DecodedPgn, f: &canboat_core::FieldInfo) -> Option<String> {
    decoded
        .fields
        .iter()
        .find(|d| d.repeat_set == 0 && d.info.order == f.order)
        .and_then(field_value_text)
}

/// Render `f` (a field inside `decoded`'s `repeat_set` at iteration
/// `iter`) as discriminator text. Returns `None` when no decoded
/// field matches or its variant is not meaningful for a key.
fn iter_field_text(
    decoded: &DecodedPgn,
    repeat_set: u8,
    iter: u32,
    f: &canboat_core::FieldInfo,
) -> Option<String> {
    decoded
        .fields
        .iter()
        .find(|d| {
            d.repeat_set == repeat_set && d.repeat_index == Some(iter) && d.info.order == f.order
        })
        .and_then(field_value_text)
}

/// Sorted, deduplicated iteration indexes present in `decoded` for
/// `repeat_set`. Drives the per-iteration storage loop so we walk
/// each iteration exactly once even if fields arrived interleaved.
fn collect_iter_indexes(decoded: &DecodedPgn, repeat_set: u8) -> Vec<u32> {
    let mut seen: Vec<u32> = Vec::new();
    for f in &decoded.fields {
        if f.repeat_set != repeat_set {
            continue;
        }
        let Some(i) = f.repeat_index else { continue };
        if !seen.contains(&i) {
            seen.push(i);
        }
    }
    seen.sort_unstable();
    seen
}

/// Re-serialize `decoded` to JSON keeping only iteration `iter` of
/// `repeat_set` (plus all top-level fields). Used by the per-PGN-
/// 130824 storage path so each cache entry's line shows only that
/// iteration's key/value.
fn synthesize_iter_line(
    decoded: &DecodedPgn,
    repeat_set: u8,
    iter: u32,
    opts: &JsonOptions,
) -> String {
    let mut sub = decoded.clone();
    sub.fields.retain(|f| {
        f.repeat_set == 0 || (f.repeat_set == repeat_set && f.repeat_index == Some(iter))
    });
    // Collapse the surviving iteration to index 0 so the JSON emitter
    // sees a single iteration cleanly (no "},{" boundary triggered by
    // a non-zero starting `repeat_index`).
    for f in &mut sub.fields {
        if f.repeat_set == repeat_set {
            f.repeat_index = Some(0);
        }
    }
    let mut buf = String::with_capacity(256);
    let _ = write_json(&mut buf, &sub, opts);
    buf
}

/// Render the discriminating value as the bare text used in the
/// composite snapshot key. For LOOKUPs we emit the raw numeric
/// `value` rather than the schema's display `name`: the key is
/// shorter and stays stable when canboat renames an enum label
/// (e.g. tweaking `"True"` → `"True (ground referenced to North)"`
/// would otherwise reshuffle cache entries). Plain integers
/// stringify; non-discriminator variants return `None`.
fn field_value_text(field: &DecodedField) -> Option<String> {
    match &field.value {
        FieldValue::Lookup { value, .. } => Some(value.to_string()),
        FieldValue::Integer(v) => Some(v.to_string()),
        FieldValue::Number(v) | FieldValue::Float(v) => Some(format!("{v}")),
        FieldValue::String(s) => Some(s.clone()),
        FieldValue::Mmsi(v) => Some(v.to_string()),
        FieldValue::Pgn { value, .. } => Some(value.to_string()),
        FieldValue::Date(d) => Some(d.to_string()),
        FieldValue::Time { raw, .. } => Some(raw.to_string()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `collect_iter_indexes` drives the per-iteration store loop;
    /// it must walk every iteration present in `repeat_set` exactly
    /// once, in ascending order, even when fields arrive interleaved
    /// or duplicated.
    #[test]
    fn collect_iter_indexes_is_sorted_and_deduplicated() {
        // Manufactured DecodedPgn fragment — we only inspect the
        // `repeat_set` / `repeat_index` shape, so the rest of the
        // struct stays at its defaults.
        use canboat_core::{DecodedField, DecodedPgn, FieldValue};
        // Borrow any existing FieldInfo from the schema; the test
        // doesn't care which one — only `repeat_set` / `repeat_index`
        // are inspected.
        let info = canboat_core::PgnDatabase::embedded(canboat_core::Units::Metric)
            .first_pgn(130824)
            .expect("PGN 130824 in schema");
        let info_field = &info.fields[0];
        let mk = |set: u8, idx: Option<u32>| DecodedField {
            info: info_field,
            value: FieldValue::Integer(0),
            bit_offset: None,
            bit_length: None,
            repeat_index: idx,
            repeat_set: set,
            overrides: None,
        };
        let pgn = DecodedPgn {
            timestamp: None,
            prio: 0,
            pgn: 130824,
            src: 0,
            dst: 0,
            description: "",
            id: "",
            id_is_pinned: false,
            data: Vec::new(),
            fields: vec![
                mk(1, Some(2)),
                mk(0, None),
                mk(1, Some(0)),
                mk(1, Some(2)),
                mk(2, Some(7)),
                mk(1, Some(1)),
            ],
            has_repeating_set: [true, true],
            index_by_order: [i8::MIN; 32],
        };
        assert_eq!(collect_iter_indexes(&pgn, 1), vec![0, 1, 2]);
        assert_eq!(collect_iter_indexes(&pgn, 2), vec![7]);
    }
}
