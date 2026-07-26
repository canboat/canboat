// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Binary wire form of a decoded canboat PGN.
//!
//! [`canboat_core::DecodedPgn`] can't cross a process boundary as-is:
//! it's stitched together from `&'static` references into the compiled
//! schema (`FieldInfo`, lookup names, PGN descriptions). You can't
//! deserialize a `&'static`, and it derives no serde.
//!
//! This crate defines the reference-*stripped* mirror that CAN:
//!
//! * Static identity is sent as a small index, not a string. The PGN
//!   variant is a [`pgn_id_hash`] of its schema `Id`; each field is its
//!   1-based schema `order`. The receiver rehydrates the real
//!   `&'static FieldInfo` by indexing its own copy of the schema.
//! * Everything derivable from `(field, raw value)` is dropped and
//!   re-resolved on the far side: `Lookup` / `BitField` / `Pgn` labels,
//!   the PGN description, the "unsupported field type" string. The wire
//!   carries only the raw `u64`/`u32`.
//! * Only genuinely-runtime data is owned on the wire: scaled numbers,
//!   decoded strings, raw binary, repeating-set structure.
//!
//! The scheme is only sound if both peers link a **byte-identical
//! schema** — otherwise an index points at the wrong field. That's what
//! [`Hello`] guarantees: the producer sends its [`canboat_core::SCHEMA_HASH`]
//! (an FNV-1a hash of `canboat.json` + `synthetic-pgns.json`, baked in
//! at build time) as the first frame, and the consumer refuses the
//! stream on any mismatch.
//!
//! ## Framing
//!
//! Every message — the [`Hello`] and each [`WirePgn`] — is a little-
//! endian `u32` length prefix followed by that many postcard bytes.
//! [`append_frame`] writes one; [`try_read_frame`] parses one back.

use std::collections::HashMap;

use serde::Serialize;
use serde::de::DeserializeOwned;

use canboat_core::decode::FieldOverrides;
use canboat_core::{DecodedField, DecodedPgn, FieldInfo, FieldValue, PgnDatabase, PgnInfo};

/// Magic prefix on the [`Hello`] handshake — `b"CBW1"`. Bumped only on
/// an incompatible framing change (not a schema change; that's caught
/// by [`Hello::schema_hash`]).
pub const MAGIC: [u8; 4] = *b"CBW1";

/// Version of *this* wire protocol (the shape of [`WirePgn`] /
/// [`Hello`] and the framing), independent of the schema version.
pub const WIRE_VERSION: u16 = 2;

/// Encode a [`canboat_core::Units`] as the byte carried in [`Hello`].
/// A dedicated byte (rather than serializing `Units`) keeps the core
/// enum free of a serde dependency.
fn units_code(u: canboat_core::Units) -> u8 {
    match u {
        canboat_core::Units::Si => 1,
        _ => 0, // Metric — also the fallback for any future default.
    }
}

/// Decode the [`Hello::units`] byte. Unknown values fall back to Metric.
fn units_from_code(c: u8) -> canboat_core::Units {
    match c {
        1 => canboat_core::Units::Si,
        _ => canboat_core::Units::Metric,
    }
}

/// Hard cap on a single frame's postcard payload. A decoded PGN is at
/// most a few kB; anything larger is a desync or a hostile peer, so we
/// refuse rather than allocate. Fast-packet PGNs top out well under 2 kB.
pub const MAX_FRAME_LEN: usize = 1 << 20;

/// djb2 hash of a schema `Id` string — the on-wire identity of a PGN
/// variant. Both peers hash their own schema's ids with this function,
/// so the value only has to be *stable*, not canonical.
#[inline]
pub fn pgn_id_hash(id: &str) -> u64 {
    let mut h: u64 = 5381;
    for b in id.bytes() {
        h = h.wrapping_shl(5).wrapping_add(h).wrapping_add(b as u64);
    }
    h
}

/// First frame on every stream: proves the producer and consumer share
/// a framing protocol and a byte-identical schema before any
/// index-bearing [`WirePgn`] is trusted.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, serde::Deserialize)]
pub struct Hello {
    pub magic: [u8; 4],
    pub wire_version: u16,
    /// [`canboat_core::SCHEMA_HASH`] of the producer's schema source.
    pub schema_hash: u64,
    /// canboat.json `SchemaVersion`, for human-readable diagnostics.
    pub schema_version: String,
    /// canboat.json `Version`, for human-readable diagnostics.
    pub canboat_version: String,
    /// Unit system the producer decoded values in (`0` = Metric, `1` =
    /// SI). The schema hash is identical for both, so this is the only
    /// thing distinguishing a `deg` stream from a `rad` one; the consumer
    /// reads it via [`Self::producer_units`] and passes it to
    /// [`WirePgn::rehydrate`] to convert into its own units.
    pub units: u8,
}

impl Hello {
    /// The handshake describing *this* build, for a producer decoding in
    /// `units`. Use [`Self::current`] for the Metric default.
    pub fn for_units(units: canboat_core::Units) -> Self {
        let db = canboat_core::PgnDatabase::embedded(units);
        Hello {
            magic: MAGIC,
            wire_version: WIRE_VERSION,
            schema_hash: canboat_core::SCHEMA_HASH,
            schema_version: db.schema_version.to_string(),
            canboat_version: db.version.to_string(),
            units: units_code(units),
        }
    }

    /// The handshake for a Metric-units producer (the default).
    pub fn current() -> Self {
        Self::for_units(canboat_core::Units::Metric)
    }

    /// The unit system the peer producer declared it decoded in.
    pub fn producer_units(&self) -> canboat_core::Units {
        units_from_code(self.units)
    }

    /// Verify a `Hello` received from a peer against this build. `Ok`
    /// means every index on the stream can be trusted against the local
    /// schema.
    pub fn verify(&self) -> Result<(), HelloError> {
        if self.magic != MAGIC {
            return Err(HelloError::BadMagic(self.magic));
        }
        if self.wire_version != WIRE_VERSION {
            return Err(HelloError::WireVersion {
                theirs: self.wire_version,
                ours: WIRE_VERSION,
            });
        }
        if self.schema_hash != canboat_core::SCHEMA_HASH {
            return Err(HelloError::SchemaHash {
                theirs: self.schema_hash,
                ours: canboat_core::SCHEMA_HASH,
            });
        }
        // Reject a units code we don't understand rather than silently
        // treating it as Metric — scaled numbers on the wire would
        // otherwise be mis-scaled. Known codes (0 Metric, 1 SI) are
        // converted at rehydrate; see [`WirePgn::rehydrate`].
        if self.units > 1 {
            return Err(HelloError::UnknownUnits(self.units));
        }
        Ok(())
    }
}

/// Why a peer's [`Hello`] was rejected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HelloError {
    BadMagic([u8; 4]),
    WireVersion {
        theirs: u16,
        ours: u16,
    },
    SchemaHash {
        theirs: u64,
        ours: u64,
    },
    /// The peer declared a [`Hello::units`] code this build can't map to
    /// a known unit system, so its scaled numbers can't be trusted.
    UnknownUnits(u8),
}

impl std::fmt::Display for HelloError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HelloError::BadMagic(m) => {
                write!(
                    f,
                    "not a canboat-wire stream (magic {m:?}, expected {MAGIC:?})"
                )
            }
            HelloError::WireVersion { theirs, ours } => write!(
                f,
                "wire-protocol mismatch: peer speaks v{theirs}, we speak v{ours}"
            ),
            HelloError::SchemaHash { theirs, ours } => write!(
                f,
                "schema mismatch: peer schema hash {theirs:#018x} != ours {ours:#018x} \
                 — the two builds were compiled from different canboat.json/synthetic-pgns.json; \
                 field indices cannot be trusted"
            ),
            HelloError::UnknownUnits(code) => write!(
                f,
                "peer declared unit-system code {code}, which this build does not understand; \
                 its scaled values cannot be converted safely"
            ),
        }
    }
}

impl std::error::Error for HelloError {}

/// One decoded PGN, reference-stripped for transport. Mirrors
/// [`canboat_core::DecodedPgn`] minus everything the receiver can
/// re-resolve from the shared schema.
#[derive(Debug, Clone, PartialEq, Serialize, serde::Deserialize)]
pub struct WirePgn {
    /// Original analyzer timestamp string, if the producer stamped one.
    pub timestamp: Option<String>,
    pub prio: u8,
    pub pgn: u32,
    pub src: u8,
    pub dst: u8,
    /// [`pgn_id_hash`] of the resolved variant's schema `Id`. Selects
    /// *which* definition of `pgn` this record was decoded against.
    pub pgn_id_hash: u64,
    /// `[set1, set2]` — mirror of [`canboat_core::DecodedPgn::has_repeating_set`].
    pub has_repeating_set: [bool; 2],
    pub fields: Vec<WireField>,
}

/// One decoded field, identified by its schema `order` rather than a
/// `&'static FieldInfo`.
#[derive(Debug, Clone, PartialEq, Serialize, serde::Deserialize)]
pub struct WireField {
    /// 1-based schema position — index into the resolved `PgnInfo::fields`.
    pub order: u8,
    /// Iteration index within a repeating set; `None` outside a set.
    pub repeat_index: Option<u32>,
    /// Which repeating set (1 → `list`, 2 → `list2`); `0` if not repeating.
    pub repeat_set: u8,
    pub value: WireValue,
    /// VARIABLE / DYNAMIC_FIELD_VALUE decode-time metadata override.
    /// Rare (98 % of fields carry `None`), so it's fine to own the unit
    /// string when present rather than index it.
    pub overrides: Option<WireOverrides>,
}

/// Owned form of [`canboat_core::decode::FieldOverrides`].
#[derive(Debug, Clone, PartialEq, Serialize, serde::Deserialize)]
pub struct WireOverrides {
    pub unit: Option<String>,
    pub resolution: Option<f64>,
    pub precision: u8,
}

/// Reference-stripped mirror of [`canboat_core::FieldValue`]. Variants
/// that carried a `&'static str` in the original (`Lookup`, `BitField`,
/// `Pgn`, `Unsupported`) keep only the raw value; the far side
/// re-resolves the label from the field's schema tables.
#[derive(Debug, Clone, PartialEq, Serialize, serde::Deserialize)]
pub enum WireValue {
    Number(f64),
    Integer(i64),
    Float(f64),
    Binary(Vec<u8>),
    /// LOOKUP / INDIRECT_LOOKUP raw value; name re-resolved by receiver.
    Lookup(u64),
    /// BITLOOKUP raw value; set-bit names re-resolved by receiver.
    BitField(u64),
    String(String),
    Date(u16),
    Time {
        raw: i64,
        seconds: f64,
    },
    Mmsi(u32),
    /// Target PGN number; description re-resolved by receiver.
    Pgn(u32),
    IsoName {
        value: u64,
        subfields: Vec<WireField>,
    },
    Reserved {
        value: u64,
        bytes: Vec<u8>,
        bit_length: u32,
    },
    Spare {
        value: u64,
        bytes: Vec<u8>,
        bit_length: u32,
    },
    NotAvailable,
    OutOfRange(u64),
    ReservedValue(u64),
    /// FieldType not yet decodable; the type name is re-resolved from
    /// the field's schema entry on the receiver.
    Unsupported,
}

impl From<&DecodedPgn> for WirePgn {
    fn from(d: &DecodedPgn) -> Self {
        WirePgn {
            timestamp: d.timestamp.clone(),
            prio: d.prio,
            pgn: d.pgn,
            src: d.src,
            dst: d.dst,
            pgn_id_hash: pgn_id_hash(d.id),
            has_repeating_set: d.has_repeating_set,
            fields: d.fields.iter().map(WireField::from).collect(),
        }
    }
}

impl From<&DecodedField> for WireField {
    fn from(f: &DecodedField) -> Self {
        WireField {
            order: f.info.order,
            repeat_index: f.repeat_index,
            repeat_set: f.repeat_set,
            value: WireValue::from(&f.value),
            overrides: f.overrides.as_deref().map(WireOverrides::from),
        }
    }
}

impl From<&FieldOverrides> for WireOverrides {
    fn from(o: &FieldOverrides) -> Self {
        WireOverrides {
            unit: o.unit.map(str::to_string),
            resolution: o.resolution,
            precision: o.precision,
        }
    }
}

impl From<&FieldValue> for WireValue {
    fn from(v: &FieldValue) -> Self {
        match v {
            FieldValue::Number(x) => WireValue::Number(*x),
            FieldValue::Integer(x) => WireValue::Integer(*x),
            FieldValue::Float(x) => WireValue::Float(*x),
            FieldValue::Binary(b) => WireValue::Binary(b.clone()),
            FieldValue::Lookup { value, .. } => WireValue::Lookup(*value),
            FieldValue::BitField { value, .. } => WireValue::BitField(*value),
            FieldValue::String(s) => WireValue::String(s.clone()),
            FieldValue::Date(d) => WireValue::Date(*d),
            FieldValue::Time { raw, seconds } => WireValue::Time {
                raw: *raw,
                seconds: *seconds,
            },
            FieldValue::Mmsi(m) => WireValue::Mmsi(*m),
            FieldValue::Pgn { value, .. } => WireValue::Pgn(*value),
            FieldValue::IsoName { value, subfields } => WireValue::IsoName {
                value: *value,
                subfields: subfields.iter().map(WireField::from).collect(),
            },
            FieldValue::Reserved {
                value,
                bytes,
                bit_length,
            } => WireValue::Reserved {
                value: *value,
                bytes: bytes.clone(),
                bit_length: *bit_length,
            },
            FieldValue::Spare {
                value,
                bytes,
                bit_length,
            } => WireValue::Spare {
                value: *value,
                bytes: bytes.clone(),
                bit_length: *bit_length,
            },
            FieldValue::NotAvailable => WireValue::NotAvailable,
            FieldValue::OutOfRange { value } => WireValue::OutOfRange(*value),
            FieldValue::ReservedValue { value } => WireValue::ReservedValue(*value),
            FieldValue::Unsupported { .. } => WireValue::Unsupported,
        }
    }
}

/// Startup-built map from [`pgn_id_hash`] to the schema `PgnInfo` it
/// identifies, so a stream of [`WirePgn`]s can be rehydrated at O(1) per
/// record. Build once; it borrows the process-static schema.
pub struct PgnIndex {
    by_hash: HashMap<u64, &'static PgnInfo>,
}

impl PgnIndex {
    /// Index every definition in `db` by the hash of its `Id`. Two
    /// distinct ids that collide under [`pgn_id_hash`] would resolve
    /// ambiguously, but the map is built from the *same* schema both
    /// peers link, so any collision is consistent across the wire (and
    /// vanishingly unlikely for ~1300 short camelCase ids).
    pub fn build(db: &'static PgnDatabase) -> Self {
        let mut by_hash = HashMap::new();
        for p in db.pgns() {
            by_hash.insert(pgn_id_hash(p.id), p);
        }
        Self { by_hash }
    }

    /// The `PgnInfo` a [`WirePgn::pgn_id_hash`] refers to, or `None` if
    /// the peer's schema knows a PGN variant this build doesn't (which
    /// [`Hello::verify`] should already have ruled out).
    #[inline]
    pub fn get(&self, hash: u64) -> Option<&'static PgnInfo> {
        self.by_hash.get(&hash).copied()
    }
}

impl WirePgn {
    /// Reconstruct a full [`canboat_core::DecodedPgn`] against the local
    /// schema — the inverse of [`WirePgn::from`]. Static references
    /// (`FieldInfo`, PGN description, lookup labels) are re-sourced from
    /// this process's schema, and dropped labels (`Lookup` / `BitField`
    /// / `Pgn`) are re-resolved from the raw value, so the result is
    /// byte-for-byte the same decoded record the producer had — provided
    /// both linked an identical schema (the [`Hello`] handshake's job).
    ///
    /// Returns `None` if the variant hash isn't in `idx`. The raw
    /// payload (`DecodedPgn::data`) is not carried on the wire, so it
    /// comes back empty; consumers that read field values never touch it.
    pub fn rehydrate(
        &self,
        db: &'static PgnDatabase,
        idx: &PgnIndex,
        peer_units: canboat_core::Units,
    ) -> Option<DecodedPgn> {
        let info = idx.get(self.pgn_id_hash)?;
        // When the producer decoded in different units than `db`, the
        // scaled `Number`s on the wire are in the producer's units. Look
        // up the same variant in the producer's schema so each field's
        // source unit is known; `rehydrate_value` then converts to `db`'s
        // units. Same-units peers skip this entirely (the common path).
        let peer_pgn: Option<&'static PgnInfo> = (peer_units != db.units())
            .then(|| canboat_core::PgnDatabase::embedded(peer_units).pgn_by_id(info.id))
            .flatten();
        let fields: Vec<DecodedField> = self
            .fields
            .iter()
            .map(|wf| wf.rehydrate(info, peer_pgn, db))
            .collect();
        // Rebuild the O(1) order→position accelerator the decoder emits.
        let mut index_by_order = [i8::MIN; 32];
        for (pos, f) in fields.iter().enumerate() {
            if f.repeat_set == 0
                && let Some(slot) = (f.info.order as usize).checked_sub(1)
                && slot < index_by_order.len()
                && pos <= i8::MAX as usize
            {
                index_by_order[slot] = pos as i8;
            }
        }
        Some(DecodedPgn {
            timestamp: self.timestamp.clone(),
            prio: self.prio,
            pgn: self.pgn,
            src: self.src,
            dst: self.dst,
            description: info.description,
            id: info.id,
            id_is_pinned: info.id_is_pinned,
            data: Vec::new(),
            fields,
            has_repeating_set: self.has_repeating_set,
            index_by_order,
        })
    }
}

impl WireField {
    fn rehydrate(
        &self,
        pgn_info: &'static PgnInfo,
        peer_pgn: Option<&'static PgnInfo>,
        db: &'static PgnDatabase,
    ) -> DecodedField {
        // The schema `order` is 1-based; the field array is 0-based.
        // Fall back to the first field if a peer sends an out-of-range
        // order (schema mismatch that slipped past the handshake) so we
        // never panic on hostile input.
        let info: &'static FieldInfo = pgn_info
            .fields
            .get(self.order.wrapping_sub(1) as usize)
            .unwrap_or(&pgn_info.fields[0]);
        // The same field in the producer's schema (for cross-units
        // conversion); `None` when the peer decoded in `db`'s units.
        let peer_field = peer_pgn.and_then(|pi| pi.fields.get(self.order.wrapping_sub(1) as usize));
        DecodedField {
            info,
            value: rehydrate_value(&self.value, info, peer_field, pgn_info, db),
            bit_offset: None,
            bit_length: None,
            repeat_index: self.repeat_index,
            repeat_set: self.repeat_set,
            overrides: self.overrides.as_ref().map(|o| {
                Box::new(FieldOverrides {
                    unit: o.unit.as_deref().and_then(|u| static_unit(u, info)),
                    resolution: o.resolution,
                    precision: o.precision,
                })
            }),
        }
    }
}

/// Recover a `&'static str` unit for a rehydrated override. The unit on
/// the wire is owned; we can only hand back a `&'static` if it matches
/// the field's own schema unit (the overwhelmingly common case for the
/// rare VARIABLE/DYNAMIC fields that carry an override at all).
/// Otherwise `None` — the value is still correct; only the display unit
/// annotation is dropped.
fn static_unit(wire_unit: &str, info: &'static FieldInfo) -> Option<&'static str> {
    match info.unit {
        Some(u) if u == wire_unit => Some(u),
        _ => None,
    }
}

fn rehydrate_value(
    v: &WireValue,
    info: &'static FieldInfo,
    peer_field: Option<&'static FieldInfo>,
    pgn_info: &'static PgnInfo,
    db: &'static PgnDatabase,
) -> FieldValue {
    match v {
        // A scaled number is in the producer's units. Convert from the
        // producer field's unit to this field's unit; a no-op when they
        // match, and `peer_field` is `None` for same-units peers.
        WireValue::Number(x) => {
            let converted = match (peer_field.and_then(|pf| pf.unit), info.unit) {
                (Some(from), Some(to)) => {
                    canboat_core::units::convert_unit(*x, from, to).unwrap_or(*x)
                }
                _ => *x,
            };
            FieldValue::Number(converted)
        }
        WireValue::Integer(x) => FieldValue::Integer(*x),
        WireValue::Float(x) => FieldValue::Float(*x),
        WireValue::Binary(b) => FieldValue::Binary(b.clone()),
        WireValue::Lookup(value) => {
            let name = info
                .lookup_enumeration
                .and_then(|t| db.lookup(t))
                .and_then(|t| t.get(*value))
                .map(|lv| lv.name)
                // DYNAMIC_FIELD_KEY fields (e.g. B&G PGN 130824 "Key")
                // carry their label in the field-type enumeration, not the
                // plain lookup table. Mirror decode_dynamic_field_key so the
                // rehydrated key keeps its name — receivers that match on it
                // (the 130824 key-value handler) otherwise see a nameless
                // number and silently drop the record.
                .or_else(|| {
                    info.lookup_field_type_enumeration
                        .and_then(|n| db.field_type_lookup(n, *value))
                        .map(|e| e.name)
                });
            FieldValue::Lookup {
                value: *value,
                name,
            }
        }
        WireValue::BitField(value) => {
            let mut bits = Vec::new();
            if let Some(table) = info.lookup_bit_enumeration.and_then(|t| db.bit_lookup(t)) {
                for bit in 0..64u8 {
                    if value & (1u64 << bit) != 0
                        && let Some(v) = table.get(bit)
                    {
                        bits.push((1u64 << bit, v.name));
                    }
                }
            }
            FieldValue::BitField {
                value: *value,
                bits,
            }
        }
        WireValue::String(s) => FieldValue::String(s.clone()),
        WireValue::Date(d) => FieldValue::Date(*d),
        WireValue::Time { raw, seconds } => FieldValue::Time {
            raw: *raw,
            seconds: *seconds,
        },
        WireValue::Mmsi(m) => FieldValue::Mmsi(*m),
        WireValue::Pgn(value) => FieldValue::Pgn {
            value: *value,
            description: db.first_pgn(*value).map(|p| p.description),
        },
        WireValue::IsoName { value, subfields } => FieldValue::IsoName {
            value: *value,
            // ISO_NAME subfields belong to PGN 60928's schema, not this
            // record's; re-resolving them fully would need that PgnInfo.
            // They ride along by raw `order` so a consumer that cares can
            // resolve them, but the common path reads only the top-level
            // `value`.
            subfields: subfields
                .iter()
                // ISO_NAME subfields are raw bit fields, not unit-scaled,
                // so no cross-units conversion applies.
                .map(|wf| wf.rehydrate(pgn_info, None, db))
                .collect(),
        },
        WireValue::Reserved {
            value,
            bytes,
            bit_length,
        } => FieldValue::Reserved {
            value: *value,
            bytes: bytes.clone(),
            bit_length: *bit_length,
        },
        WireValue::Spare {
            value,
            bytes,
            bit_length,
        } => FieldValue::Spare {
            value: *value,
            bytes: bytes.clone(),
            bit_length: *bit_length,
        },
        WireValue::NotAvailable => FieldValue::NotAvailable,
        WireValue::OutOfRange(value) => FieldValue::OutOfRange { value: *value },
        WireValue::ReservedValue(value) => FieldValue::ReservedValue { value: *value },
        // The original carried the offending FieldType's canboat name
        // purely for diagnostics; no consumer reads it and re-deriving
        // the exact string isn't worth a 23-arm match. A stable marker
        // preserves the "this field wasn't decodable" signal.
        WireValue::Unsupported => FieldValue::Unsupported {
            field_type: "UNSUPPORTED",
        },
    }
}

/// Serialize `value` to postcard and append it as one length-prefixed
/// frame to `out`. Used for both the [`Hello`] and each [`WirePgn`].
pub fn append_frame<T: Serialize>(out: &mut Vec<u8>, value: &T) -> postcard::Result<()> {
    let body = postcard::to_allocvec(value)?;
    out.extend_from_slice(&(body.len() as u32).to_le_bytes());
    out.extend_from_slice(&body);
    Ok(())
}

/// Error parsing an inbound frame.
#[derive(Debug)]
pub enum FrameError {
    /// Frame length prefix exceeds [`MAX_FRAME_LEN`] — treat as desync.
    TooLong(usize),
    /// postcard failed to decode the payload.
    Decode(postcard::Error),
}

impl std::fmt::Display for FrameError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FrameError::TooLong(n) => {
                write!(f, "frame length {n} exceeds max {MAX_FRAME_LEN}")
            }
            FrameError::Decode(e) => write!(f, "postcard decode: {e}"),
        }
    }
}

impl std::error::Error for FrameError {}

/// Decode a bare postcard frame body — the length prefix already
/// stripped by an external framer (e.g. tokio-util's
/// `LengthDelimitedCodec`). The sibling of [`try_read_frame`] for
/// callers that don't manage the prefix themselves.
pub fn decode_frame<T: DeserializeOwned>(body: &[u8]) -> Result<T, FrameError> {
    postcard::from_bytes(body).map_err(FrameError::Decode)
}

/// Try to parse one complete frame from the front of `buf`.
///
/// * `Ok(Some((value, n)))` — a frame decoded; the caller should drain
///   the first `n` bytes of `buf`.
/// * `Ok(None)` — `buf` doesn't yet hold a whole frame; read more.
/// * `Err(_)` — oversized or corrupt frame; the stream is desynced and
///   should be dropped.
pub fn try_read_frame<T: DeserializeOwned>(buf: &[u8]) -> Result<Option<(T, usize)>, FrameError> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if len > MAX_FRAME_LEN {
        return Err(FrameError::TooLong(len));
    }
    let total = 4 + len;
    if buf.len() < total {
        return Ok(None);
    }
    let value = postcard::from_bytes(&buf[4..total]).map_err(FrameError::Decode)?;
    Ok(Some((value, total)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_roundtrips_and_self_verifies() {
        let hello = Hello::current();
        let mut buf = Vec::new();
        append_frame(&mut buf, &hello).unwrap();
        let (got, n): (Hello, usize) = try_read_frame(&buf).unwrap().unwrap();
        assert_eq!(n, buf.len());
        assert_eq!(got, hello);
        got.verify().expect("current build must verify itself");
    }

    #[test]
    fn schema_mismatch_is_rejected() {
        let mut hello = Hello::current();
        hello.schema_hash ^= 0xdead_beef;
        assert!(matches!(hello.verify(), Err(HelloError::SchemaHash { .. })));
    }

    #[test]
    fn partial_buffer_yields_none() {
        let hello = Hello::current();
        let mut buf = Vec::new();
        append_frame(&mut buf, &hello).unwrap();
        // One byte short of a full frame.
        let short = &buf[..buf.len() - 1];
        let got: Option<(Hello, usize)> = try_read_frame(short).unwrap();
        assert!(got.is_none());
    }

    #[test]
    fn rehydrate_converts_scaled_numbers_across_units() {
        use canboat_core::RawFrame;

        let metric = PgnDatabase::embedded(canboat_core::Units::Metric);
        let si = PgnDatabase::embedded(canboat_core::Units::Si);

        // Vessel Heading — "Heading" (order 2) is an angle field, so its
        // scaled value differs between schemas (deg vs rad).
        let frame = RawFrame::new(
            Some("2026-06-30T23:30:33.151Z".into()),
            2,
            127250,
            17,
            255,
            [0x00, 0xb8, 0x22, 0xff, 0x7f, 0xff, 0x7f, 0xfd],
        );
        // Producer decodes in Metric (degrees) → that's what's on the wire.
        let decoded = metric.decode(&frame).expect("decode");
        let heading_deg = decoded
            .field_by_name("Heading")
            .and_then(|f| f.value.as_f64())
            .expect("heading");
        let wire = WirePgn::from(&decoded);

        // A consumer rehydrating against the SI schema, told the peer was
        // Metric, must convert the degree value to radians.
        let rehy = wire
            .rehydrate(si, &PgnIndex::build(si), canboat_core::Units::Metric)
            .expect("rehydrate");
        let heading_rad = rehy
            .field_by_name("Heading")
            .and_then(|f| f.value.as_f64())
            .expect("heading");
        assert!(
            (heading_rad - heading_deg.to_radians()).abs() < 1e-6,
            "{heading_deg} deg should rehydrate as {} rad, got {heading_rad}",
            heading_deg.to_radians()
        );

        // Same-units rehydrate leaves the value untouched (the fast path).
        let same = wire
            .rehydrate(
                metric,
                &PgnIndex::build(metric),
                canboat_core::Units::Metric,
            )
            .expect("rehydrate");
        let h = same
            .field_by_name("Heading")
            .and_then(|f| f.value.as_f64())
            .unwrap();
        assert!((h - heading_deg).abs() < 1e-9);
    }

    #[test]
    fn wirepgn_rehydrates_to_equivalent_decoded() {
        use canboat_core::{FieldValue as FV, RawFrame};

        let db = PgnDatabase::embedded(canboat_core::Units::Metric);
        let idx = PgnIndex::build(db);

        // PGN 127250 Vessel Heading: a SID, scaled Number fields, and a
        // `Reference` LOOKUP — exercises number + lookup-name resolution.
        let frame = RawFrame::new(
            Some("2026-06-30T23:30:33.151Z".into()),
            2,
            127250,
            17,
            255,
            [0x00, 0xb8, 0x22, 0xff, 0x7f, 0xff, 0x7f, 0xfd],
        );
        let decoded = db.decode(&frame).expect("decode 127250");
        let wire = WirePgn::from(&decoded);

        // The wire form survives a full postcard round-trip.
        let mut buf = Vec::new();
        append_frame(&mut buf, &wire).unwrap();
        let (wire2, _n): (WirePgn, usize) = try_read_frame(&buf).unwrap().unwrap();
        assert_eq!(wire2, wire);

        // Rehydrating reproduces the decoded record field-for-field.
        let rehy = wire.rehydrate(db, &idx, db.units()).expect("rehydrate");
        assert_eq!(rehy.pgn, decoded.pgn);
        assert_eq!(rehy.id, decoded.id);
        assert_eq!(rehy.src, decoded.src);
        assert_eq!(rehy.fields.len(), decoded.fields.len());

        let mut saw_lookup_name = false;
        for (a, b) in decoded.fields.iter().zip(&rehy.fields) {
            assert_eq!(a.name(), b.name(), "field name");
            assert_eq!(a.value.as_f64(), b.value.as_f64(), "numeric: {}", a.name());
            assert_eq!(a.value.as_str(), b.value.as_str(), "text: {}", a.name());
            if matches!(a.value, FV::Lookup { name: Some(_), .. }) {
                saw_lookup_name = true;
            }
        }
        assert!(
            saw_lookup_name,
            "the Reference LOOKUP should re-resolve a name on the rehydrated side"
        );

        // merrimac's access path — field-by-name — works on the result.
        assert_eq!(
            rehy.field_by_name("Reference")
                .and_then(|f| f.value.as_str()),
            decoded
                .field_by_name("Reference")
                .and_then(|f| f.value.as_str()),
        );
    }

    /// Regression: a DYNAMIC_FIELD_KEY (B&G PGN 130824 "Key") carries its
    /// label in a `LookupFieldTypeEnumeration`, not the plain lookup table.
    /// The receiver must re-resolve it — otherwise the key comes back a
    /// nameless number and merrimac's key-value handler (which matches on
    /// the key name, e.g. "Polar Performance" = 124) silently drops it.
    #[test]
    fn dynamic_field_key_lookup_reresolves_name() {
        use canboat_core::FieldValue as FV;

        let db = PgnDatabase::embedded(canboat_core::Units::Metric);
        let info = db.first_pgn(130824).expect("PGN 130824 in schema");
        let key = info
            .fields
            .iter()
            .find(|f| f.lookup_field_type_enumeration.is_some())
            .expect("130824 declares a DYNAMIC_FIELD_KEY field");

        let wf = WireField {
            order: key.order,
            repeat_index: Some(0),
            repeat_set: 1,
            value: WireValue::Lookup(124),
            overrides: None,
        };
        let rehy = wf.rehydrate(info, None, db);
        assert!(
            matches!(&rehy.value, FV::Lookup { name: Some(n), value: 124 } if *n == "Polar Performance"),
            "fieldtype-key 124 must re-resolve to \"Polar Performance\", got {:?}",
            rehy.value
        );
        assert_eq!(rehy.value.as_str(), Some("Polar Performance"));
    }

    // The load-bearing guarantee for merrimac's "C-mode" shim: the JSON
    // produced from a rehydrated record is byte-for-byte what the
    // analyzer port would have emitted, so `process_message` cannot tell
    // the binary transport from the JSON one.
    #[test]
    fn rehydrated_json_matches_analyzer_json() {
        use canboat_core::RawFrame;
        use canboat_core::output::{CamelCase, JsonOptions, write_json};

        let db = PgnDatabase::embedded(canboat_core::Units::Metric);
        let idx = PgnIndex::build(db);
        // Same options canboat-pipeline's analyzer port uses.
        let opts = JsonOptions {
            include_empty: false,
            name_value: true,
            debug: false,
            camel_case: CamelCase::Off,
        };

        let frames = [
            // 127250 Vessel Heading — lookup + scaled numbers + a timestamp.
            RawFrame::new(
                Some("2026-06-30T23:30:33.151Z".into()),
                2,
                127250,
                17,
                255,
                [0x00, 0xb8, 0x22, 0xff, 0x7f, 0xff, 0x7f, 0xfd],
            ),
            // 128267 Water Depth — plain numbers, no timestamp.
            RawFrame::new(
                None,
                3,
                128267,
                35,
                255,
                [0x00, 0x10, 0x27, 0x00, 0x00, 0xff, 0xff, 0xff],
            ),
            // 127245 Rudder — instance + signed angle.
            RawFrame::new(
                None,
                2,
                127245,
                22,
                255,
                [0x00, 0xf8, 0xff, 0xff, 0x00, 0x20, 0xff, 0xff],
            ),
        ];

        for frame in frames {
            let decoded = db.decode(&frame).expect("decode");
            let wire = WirePgn::from(&decoded);
            let rehy = wire.rehydrate(db, &idx, db.units()).expect("rehydrate");

            let mut json_direct = String::new();
            write_json(&mut json_direct, &decoded, &opts).unwrap();
            let mut json_via_wire = String::new();
            write_json(&mut json_via_wire, &rehy, &opts).unwrap();

            assert_eq!(
                json_direct, json_via_wire,
                "C-mode JSON must equal analyzer JSON for PGN {}",
                frame.pgn
            );
        }
    }
}
