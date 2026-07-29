// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Decode a `RawFrame` into a structured `DecodedPgn`.
//!
//! The decoder looks the frame's PGN up in the database, picks the
//! matching variant (manufacturer-specific PGNs disambiguated by
//! `Match` field values), and walks each field, extracting raw bits
//! and applying scaling, units, and lookup-table resolution.
//!
//! No I/O. No formatting — the result is a structured event ready for
//! the output formatter or for direct consumption by merrimac-rs.

use crate::bits::{Extracted, Sentinel, classify_sentinel, extract_bits};
use crate::db::PgnDatabase;
use crate::frame::RawFrame;
use crate::types::{FieldInfo, FieldType, PgnInfo};

/// Run the schema-2.5.0 sentinel classifier against a numeric
/// extraction using the field's own `UnknownValue` / `OutOfRangeValue`
/// / `ReservedValue` hints, and map any hit to the matching
/// [`FieldValue`] sentinel variant. Returns `None` when the extracted
/// value is a legitimate reading (no hint matched).
///
/// `raw` is read from [`Extracted::raw`] — the pre-sign-extension /
/// pre-offset unsigned bit pattern that the schema hints are stated
/// against. f64 conversion is never involved, so >53-bit fields and
/// signed-field cases round-trip bit-exactly.
fn sentinel_field_value(f: &FieldInfo, ex: Extracted) -> Option<FieldValue> {
    match classify_sentinel(ex, f.unknown_value, f.out_of_range_value, f.reserved_value) {
        Sentinel::Unknown => Some(FieldValue::NotAvailable),
        Sentinel::OutOfRange => Some(FieldValue::OutOfRange { value: ex.raw }),
        Sentinel::Reserved => Some(FieldValue::ReservedValue { value: ex.raw }),
        Sentinel::None => None,
    }
}

/// The same test, for a field whose FieldType declares
/// `Sentinels: None` and so carries none of the hints above, but which
/// still has a `RangeMax` below what its bit width can hold.
///
/// The C never consults the FieldType's `Sentinels` when it works out
/// how many top-of-range values a field reserves — `reservedCount` is
/// derived from `rangeMax` against the raw bit-width maximum, capped by
/// `reservedCountForSize` (fieldtype.c). So an MMSI, whose range stops
/// at 999999999 in a 32-bit field, still reserves its top three values,
/// and `extractNumberNotEmpty` hands anything above the threshold to
/// `printEmpty`. `Sentinels: None` only stops the hints being written
/// into the document; it does not make every bit pattern valid.
fn range_max_sentinel(f: &FieldInfo, ex: Extracted, bits: u32) -> Option<FieldValue> {
    let range_max = f.range_max?;
    let resolution = f.resolution.unwrap_or(1.0);
    if bits == 0 || bits >= 64 || resolution <= 0.0 || !range_max.is_finite() {
        return None;
    }
    let raw_max = (1u64 << bits) - 1;
    let raw_range_max = (range_max / resolution + 0.5) as u64;
    if raw_range_max >= raw_max {
        return None;
    }
    // reservedCountForSize(): 3 from 8 bits up, 2 from 4, 1 from 2.
    let by_size: u64 = match bits {
        8.. => 3,
        4..=7 => 2,
        2..=3 => 1,
        _ => 0,
    };
    let reserved = (raw_max - raw_range_max).min(by_size);
    if reserved == 0 || ex.raw <= raw_max - reserved {
        return None;
    }
    Some(match raw_max - ex.raw {
        0 => FieldValue::NotAvailable,
        1 => FieldValue::OutOfRange { value: ex.raw },
        _ => FieldValue::ReservedValue { value: ex.raw },
    })
}

/// One decoded field.
///
/// Schema metadata (`id`, `name`, `order`, `unit`, `resolution`,
/// `precision`, `part_of_primary_key`) is read through accessor methods
/// that delegate to [`Self::info`] — the corresponding
/// [`FieldInfo`] entry in the static schema. Runtime values that the
/// decoder owns (`bit_offset`, `bit_length`, `repeat_*`, `value`) stay
/// as direct fields because they can legitimately differ from the
/// schema's declared values (repeating-set iterations shift the offset;
/// VARIABLE fields can be byte-aligned wider than the schema declares).
///
/// VARIABLE and DYNAMIC_FIELD_VALUE fields override unit / resolution /
/// precision with metadata drawn from a target FieldInfo or
/// LookupFieldTypeValue; those overrides live in a boxed
/// [`FieldOverrides`] sidecar — `None` in the common path, so the
/// 98 % of fields that don't override only carry an 8-byte null
/// pointer in the slot.
#[derive(Debug, Clone)]
pub struct DecodedField {
    /// Pointer back into the static [`FieldInfo`] this field was
    /// decoded against.
    pub info: &'static FieldInfo,
    pub value: FieldValue,
    /// Bit offset of this field within the parent payload. May differ
    /// from `info.bit_offset` for fields inside repeating sets (each
    /// iteration shifts) and for variable-length fields. `None` for
    /// synthetic fields.
    pub bit_offset: Option<u32>,
    /// Effective bit length on the wire. May differ from
    /// `info.bit_length` for STRING_LAU / VARIABLE / DYNAMIC_FIELD_VALUE
    /// fields (resolved at decode time).
    pub bit_length: Option<u32>,
    /// Zero-based iteration index for fields inside a repeating set;
    /// `None` for non-repeating fields.
    pub repeat_index: Option<u32>,
    /// Which RepeatingFieldSet this field belongs to: 1 → emitted
    /// under JSON `"list"`, 2 → under `"list2"`. `0` for non-repeating.
    pub repeat_set: u8,
    /// VARIABLE / DYNAMIC_FIELD_VALUE override metadata. `None` for
    /// the common path.
    pub overrides: Option<Box<FieldOverrides>>,
}

/// Resolved-at-decode-time metadata override carried by VARIABLE and
/// DYNAMIC_FIELD_VALUE fields. The accessors on [`DecodedField`]
/// consult these first, falling back to [`FieldInfo`].
///
/// `None` on a field means "no override at this level — use the value
/// from `info`".
#[derive(Debug, Clone)]
pub struct FieldOverrides {
    pub unit: Option<&'static str>,
    pub resolution: Option<f64>,
    /// `0` means "no override" (use `info.precision`); non-zero
    /// overrides.
    pub precision: u8,
}

impl DecodedField {
    /// canboat.json field `Order` (1-based schema position).
    #[inline]
    pub fn order(&self) -> u8 {
        self.info.order
    }

    /// canboat.json field `Id` (camelCase identifier).
    #[inline]
    pub fn id(&self) -> &'static str {
        self.info.id
    }

    /// canboat.json field `Name` (human label).
    #[inline]
    pub fn name(&self) -> &'static str {
        self.info.name
    }

    /// Display unit. Overridden by VARIABLE / DYNAMIC_FIELD_VALUE
    /// resolution; otherwise from [`FieldInfo::unit`].
    #[inline]
    pub fn unit(&self) -> Option<&'static str> {
        if let Some(o) = self.overrides.as_deref()
            && o.unit.is_some()
        {
            return o.unit;
        }
        self.info.unit
    }

    /// This field's numeric value expressed in `target` unit, converting
    /// from the field's own [`unit`](Self::unit) when they differ (see
    /// [`crate::units::convert_unit`]). A unitless field is returned
    /// as-is; a numeric field whose unit has no known bridge to `target`
    /// yields `None`.
    ///
    /// This lets a consumer that needs a fixed unit stay correct no
    /// matter which schema ([`crate::Units`]) produced the field — e.g.
    /// NMEA 0183 always asks angles for `"deg"` and temperatures for
    /// `"C"`, and gets them whether the stream was SI or Metric.
    pub fn as_f64_in(&self, target: &str) -> Option<f64> {
        let v = self.value.as_f64()?;
        match self.unit() {
            Some(from) => crate::units::convert_unit(v, from, target),
            None => Some(v),
        }
    }

    /// Display resolution.
    #[inline]
    pub fn resolution(&self) -> Option<f64> {
        if let Some(o) = self.overrides.as_deref()
            && o.resolution.is_some()
        {
            return o.resolution;
        }
        self.info.resolution
    }

    /// Decimal precision override (`0` = derive from resolution).
    #[inline]
    pub fn precision(&self) -> u8 {
        if let Some(o) = self.overrides.as_deref()
            && o.precision > 0
        {
            return o.precision;
        }
        self.info.precision
    }

    /// `true` if this field participates in the PGN's primary key.
    #[inline]
    pub fn part_of_primary_key(&self) -> bool {
        self.info.part_of_primary_key.unwrap_or(false)
    }
}

/// The decoded value of one field.
#[derive(Debug, Clone)]
pub enum FieldValue {
    /// Scaled numeric value (`resolution * raw + offset`).
    Number(f64),
    /// Unscaled integer (used for fields with `resolution == 1` and no
    /// unit — counts, instances, etc.).
    Integer(i64),
    /// IEEE 754 float decoded from 32 raw bits.
    Float(f64),
    /// Raw bytes (BINARY) — uninterpreted.
    Binary(Vec<u8>),
    /// LOOKUP / INDIRECT_LOOKUP result.
    Lookup {
        value: u64,
        name: Option<&'static str>,
    },
    /// BITLOOKUP result — list of set bits with the bit-flag value
    /// (1 << bit) and resolved name for each.
    BitField {
        value: u64,
        /// One entry per *set bit*, in bit order: its value (`1 << bit`)
        /// and its name, or `None` when the enumeration does not name
        /// that bit. canboat emits unnamed set bits too — as
        /// `{"value":N,"name":null}` under `-nv` and as a bare `N`
        /// otherwise — so they must not be dropped.
        bits: Vec<(u64, Option<&'static str>)>,
    },
    /// Decoded text (STRING_FIX, STRING_LZ, STRING_LAU).
    String(String),
    /// 16-bit days since 1970-01-01.
    Date(u16),
    /// Seconds since midnight (post-resolution scaling) plus the raw
    /// integer the decoder extracted. The raw is needed for `-nv`
    /// output `{"value":raw,"name":"HH:MM:SS.SSSS"}`.
    Time { raw: i64, seconds: f64 },
    /// MMSI as a 9-digit identifier.
    Mmsi(u32),
    /// 24-bit PGN number. `description` is the target PGN's
    /// human-readable name from the database, if known.
    Pgn {
        value: u32,
        description: Option<&'static str>,
    },
    /// ISO_NAME — a 64-bit packed identifier that is also a valid
    /// PGN 60928 (ISO Address Claim) payload. We carry the raw value
    /// and the recursively-decoded subfields side by side so the
    /// formatter can choose either form.
    IsoName {
        value: u64,
        subfields: Vec<DecodedField>,
    },
    /// RESERVED / SPARE — value preserved but the field is meaningless.
    /// The raw bytes (in field order, no resolution) ride along so the
    /// JSON formatter can emit them as hex strings, matching canboat.
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
    /// Field exists but raw value is the canboat "not-available"
    /// sentinel (`UnknownValue` for schema-2.4.0 fields; the top-of-
    /// range heuristic value for older databases).
    NotAvailable,
    /// Raw value hit the field's `OutOfRangeValue` sentinel (schema
    /// 2.4.0). The formatter renders this as `"Out Of Range"` to
    /// match canboat C `analyzer/print.c`. The raw value rides along
    /// as `u64` so callers that want the wire pattern bit-exactly —
    /// including for >53-bit fields where `f64` would lose precision
    /// — can read it without going back to `RawFrame`.
    OutOfRange { value: u64 },
    /// Raw value hit the field's `ReservedValue` sentinel (schema
    /// 2.4.0 "reserved for future use"). Distinct from the
    /// [`FieldValue::Reserved`] variant above which represents an
    /// explicit `RESERVED` FieldType field (padding / structural).
    /// Formatter renders as `"Reserved"`.
    ReservedValue { value: u64 },
    /// Field decoding not yet implemented for this `FieldType`.
    Unsupported { field_type: &'static str },
}

impl FieldValue {
    /// Extract a `f64` for numeric variants. `Integer` widens; `Time`
    /// returns the post-resolution `seconds` value (matching what
    /// formatters show). Non-numeric variants return `None`.
    #[inline]
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            FieldValue::Number(v) | FieldValue::Float(v) => Some(*v),
            FieldValue::Integer(v) => Some(*v as f64),
            FieldValue::Time { seconds, .. } => Some(*seconds),
            FieldValue::Date(d) => Some(*d as f64),
            FieldValue::Mmsi(v) => Some(*v as f64),
            _ => None,
        }
    }

    /// Extract a `i64` from integer-shaped variants. `Lookup` returns
    /// the raw enum integer; `Number` truncates; `Time` returns the
    /// raw scaled-out integer the decoder kept around.
    #[inline]
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            FieldValue::Integer(v) => Some(*v),
            FieldValue::Number(v) => Some(*v as i64),
            FieldValue::Lookup { value, .. } => Some(*value as i64),
            FieldValue::BitField { value, .. } => Some(*value as i64),
            FieldValue::Time { raw, .. } => Some(*raw),
            FieldValue::Date(d) => Some(*d as i64),
            FieldValue::Mmsi(v) => Some(*v as i64),
            FieldValue::Pgn { value, .. } => Some(*value as i64),
            _ => None,
        }
    }

    /// Raw `u64` for `Lookup` / `BitField`. Use when you specifically
    /// want the enum tag without worrying about sign.
    #[inline]
    pub fn lookup_value(&self) -> Option<u64> {
        match self {
            FieldValue::Lookup { value, .. } => Some(*value),
            FieldValue::BitField { value, .. } => Some(*value),
            _ => None,
        }
    }

    /// `&str` for `String` / for the resolved `Lookup`/`BitField`
    /// label when one was found. Non-text variants return `None`.
    #[inline]
    pub fn as_str(&self) -> Option<&str> {
        match self {
            FieldValue::String(s) => Some(s.as_str()),
            FieldValue::Lookup { name: Some(n), .. } => Some(*n),
            _ => None,
        }
    }

    /// `true` when the field decoded to the canboat "not-available"
    /// sentinel — useful for callers that want to fall back to a
    /// default rather than emit nothing.
    #[inline]
    pub fn is_not_available(&self) -> bool {
        matches!(self, FieldValue::NotAvailable)
    }

    /// `true` when the field decoded to ANY of the three schema-2.4.0
    /// sentinels — Unknown, Out Of Range, or Reserved-value. Callers
    /// that just want "skip this field, the wire didn't carry a real
    /// reading" can check this in one call.
    #[inline]
    pub fn is_sentinel(&self) -> bool {
        matches!(
            self,
            FieldValue::NotAvailable
                | FieldValue::OutOfRange { .. }
                | FieldValue::ReservedValue { .. }
        )
    }
}

/// A fully decoded PGN event.
#[derive(Debug, Clone)]
pub struct DecodedPgn {
    pub timestamp: Option<String>,
    pub prio: u8,
    pub pgn: u32,
    pub src: u8,
    pub dst: u8,
    pub description: &'static str,
    /// canboat.json `Id` — stable camelCase identifier.
    pub id: &'static str,
    /// Mirror of [`crate::types::PgnInfo::id_is_pinned`]: `true` when
    /// the schema's canboat C source explicitly pinned this PGN's
    /// `Id`. The JSON formatter wraps such records in `{"<id>":{…}}`
    /// to flag the stability contract to consumers.
    pub id_is_pinned: bool,
    /// The raw payload bytes the fields were decoded from. Kept on
    /// the DecodedPgn so the `-debug` JSON formatter can extract
    /// per-field `bytes` / `bits` annotations without holding the
    /// original `RawFrame`.
    pub data: Vec<u8>,
    pub fields: Vec<DecodedField>,
    /// `[set1, set2]` — true when the decoder reached the start
    /// field of the corresponding repeating set during this decode.
    /// False when the payload ran out before the set's start field
    /// or the PGN doesn't define a set at that index. The JSON
    /// formatter uses these to emit canboat's empty-`"list":[{}]`
    /// placeholder when the count is zero — matching the
    /// `"list":[{` opener that analyzer/analyzer.c:1276 emits the
    /// moment it sees the start field.
    pub has_repeating_set: [bool; 2],
    /// `index_by_order[order - 1]` = position in `self.fields` of
    /// the top-level (non-repeating) field whose schema `order` is
    /// `order`, or `i8::MIN` when the field wasn't decoded into
    /// `self.fields` (e.g. truncated payload, filtered out, or part
    /// of a repeating set). Populated by `decode_fields`. Used by
    /// [`Self::field`] to deliver `O(1)` handle lookups.
    ///
    /// 32 slots is canboat's hard cap on per-PGN field count. The
    /// array sits inline so the lookup is a single bounds-checked
    /// load with no allocation.
    pub index_by_order: [i8; 32],
}

impl DecodedPgn {
    /// Look up a non-repeating field by [`crate::FieldRef`] — a generated
    /// constant (`canboat_core::field::wind_data::WIND_ANGLE`) or one
    /// resolved at runtime via [`crate::PgnDatabase::field`]. Returns
    /// `None` when the field wasn't decoded in this payload (truncated,
    /// NotAvailable filtered by the decoder, etc.).
    ///
    /// `O(1)`: index by the field's schema `order`, one array load plus a
    /// bounds check. The `FieldRef` carries its own PGN, so a debug-only
    /// assert catches a constant used against the wrong record; it's a
    /// `&str` compare, free in release builds — no hashing on this path.
    #[inline]
    pub fn field(&self, f: crate::FieldRef) -> Option<&DecodedField> {
        debug_assert_eq!(
            f.pgn.id, self.id,
            "FieldRef/PGN mismatch: a `{}` field ref was used on a `{}` record",
            f.pgn.id, self.id,
        );
        let idx_slot = (f.field.order as usize).checked_sub(1)?;
        let idx = *self.index_by_order.get(idx_slot)?;
        if idx < 0 {
            return None;
        }
        self.fields.get(idx as usize)
    }

    /// Look up a top-level field by name. `O(n)` linear scan over
    /// the decoded fields — fine for callers that don't have a
    /// [`crate::FieldRef`] constant (e.g. AIS encoders that
    /// would otherwise need dozens of handles), since records
    /// typically have well under 20 fields. Faster than the
    /// JSON-substring-search alternative by a wide margin.
    #[inline]
    pub fn field_by_name(&self, name: &str) -> Option<&DecodedField> {
        self.fields
            .iter()
            .find(|f| f.repeat_set == 0 && f.name() == name)
    }

    /// The 64-bit ISO NAME of a PGN 60928 (ISO Address Claim) record —
    /// the ISO 11783-5 bit-packing of the claim's fields, identical to
    /// `u64::from_le_bytes(payload[..8])` for a spec-compliant frame.
    /// Returns `None` for any other PGN.
    ///
    /// Packs from the decoded *fields* (LSB-first, in schema order)
    /// rather than the raw bytes, so it works both for a record decoded
    /// from a frame and one rebuilt from analyzer JSON (which carries no
    /// payload). That makes it the canonical device key for the
    /// per-NAME NMEA 0183 filter, computed identically by every daemon.
    /// The reserved `Spare` bit is taken as its spec value 0; every
    /// consumer computing the NAME the same way still gets a stable key.
    pub fn iso_name(&self) -> Option<u64> {
        if self.pgn != 60928 {
            return None;
        }
        let f = |name: &str, bits: u32| -> u64 {
            let v = self
                .field_by_name(name)
                .and_then(|df| df.value.as_i64())
                .unwrap_or(0) as u64;
            v & ((1u64 << bits) - 1)
        };
        Some(
            f("Unique Number", 21)
                | f("Manufacturer Code", 11) << 21
                | f("Device Instance Lower", 3) << 32
                | f("Device Instance Upper", 5) << 35
                | f("Device Function", 8) << 40
                // bit 48: Spare (reserved, 0)
                | f("Device Class", 7) << 49
                | f("System Instance", 4) << 56
                | f("Industry Group", 3) << 60
                | f("Arbitrary address capable", 1) << 63,
        )
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DecodeError {
    #[error("PGN {pgn} not found in database")]
    UnknownPgn { pgn: u32 },
    #[error("PGN {pgn} payload too short for field {field:?}")]
    ShortPayload { pgn: u32, field: String },
}

impl PgnDatabase {
    /// Decode one frame.
    ///
    /// Picks the matching PGN variant (via `Match` field values) and
    /// walks every field. For unknown PGNs returns
    /// `DecodeError::UnknownPgn`.
    pub fn decode(&self, frame: &RawFrame) -> Result<DecodedPgn, DecodeError> {
        let info = self
            .pick_variant(frame)
            .ok_or(DecodeError::UnknownPgn { pgn: frame.pgn })?;

        let (fields, has_repeating_set) = decode_fields(info, &frame.data, self)?;
        let index_by_order = build_index_by_order(&fields);

        Ok(DecodedPgn {
            timestamp: frame.timestamp.clone(),
            prio: frame.prio,
            pgn: frame.pgn,
            src: frame.src,
            dst: frame.dst,
            description: info.description,
            id: info.id,
            id_is_pinned: info.id_is_pinned,
            data: frame.data.to_vec(),
            fields,
            has_repeating_set,
            index_by_order,
        })
    }

    /// Choose the best PGN definition for a frame.
    ///
    /// Priority — strict canboat semantics:
    ///
    ///   1. **JSON-order scan of variants for this PGN number.** Return
    ///      the first variant whose every `Match` field equals the raw
    ///      bits at that field's offset. JSON insertion order encodes
    ///      author-defined priority; an earlier matching specific
    ///      variant always beats a later one, and a matching specific
    ///      variant always beats the per-PGN-number fallback even when
    ///      the fallback is listed first (as in PGN 126208 etc).
    ///
    ///   2. **In-PGN fallback.** If no specific variant matched, return
    ///      the `Fallback: true` variant for this PGN number, if any.
    ///      Falls back further to any no-`Match` variant — though in
    ///      canboat.json those two are the same entry.
    ///
    ///   3. **Inter-PGN fallback.** If the PGN number itself is not in
    ///      the database, return the largest `Fallback: true` entry
    ///      whose PGN number is `<= frame.pgn`. This is the catch-all
    ///      that covers the range — e.g. unknown PGNs in
    ///      `[0x1ef00, 0x1ef00]` get
    ///      `0x1ef00ManufacturerProprietaryFastPacketAddressed`.
    pub fn pick_variant(&self, frame: &RawFrame) -> Option<&'static PgnInfo> {
        // Generated by `build.rs`: a `match` jumps to a per-PGN dispatch
        // function that extracts each unique Match-field bit-range
        // exactly once and returns the right variant in JSON order
        // (or the no-Match fallback for that PGN). Falls back to the
        // sparse cross-PGN `find_catchall` when no entry for this PGN
        // number exists at all.
        // Dispatch returns a schema index (identical across the SI and
        // Metric PGN arrays); map it through this db's own `pgns` so the
        // returned PgnInfo carries the right units.
        let idx = crate::schema_data::dispatch(frame.pgn, &frame.data)
            .or_else(|| crate::schema_data::find_catchall(frame.pgn))?;
        Some(&self.pgn_slice()[idx])
    }
}

/// Walk a PGN's fields, decoding each in turn.
///
/// Handles repeating field sets (`RepeatingFieldSet1`) — when the
/// walker reaches `repeating_field_set1_start_field`, it pulls the
/// count from the already-decoded count field, then re-runs the set
/// of `size` repeating fields N times. Each iteration's fields share
/// the layout of the first iteration shifted by `iteration_bits`
/// (the sum of `BitLength` over the set).
///
/// `Condition`-gated alternative fields are skipped (full evaluation
/// is deferred to a later commit).
/// State the decoder threads through fields so VARIABLE-typed fields
/// can find their target metadata.
///
/// - `target_pgn` is updated whenever a [`FieldValue::Pgn`] is emitted
///   (PGN 126208 group functions carry the target PGN in field 2).
/// - `current_param_idx` is updated whenever a `FIELD_INDEX` field is
///   emitted, so the next VARIABLE field knows which target field's
///   metadata to use.
#[derive(Default, Debug, Clone)]
struct DecodeContext {
    target_pgn: Option<u32>,
    current_param_idx: Option<u32>,
    /// Resolved DYNAMIC_FIELD_KEY entry — set when a key field is
    /// decoded, read by the matching DYNAMIC_FIELD_VALUE, cleared
    /// after. Stored as cloned data so we don't have to thread a
    /// lifetime through every decoder.
    dynamic_field_type: Option<crate::types::LookupFieldTypeValue>,
    /// Byte length from a DYNAMIC_FIELD_LENGTH field, net of the
    /// record overhead. Cleared with `dynamic_field_type` after the
    /// value is decoded. Signed, and legitimately negative when a
    /// truncated record declares a length below its own header — the
    /// C's `g_length` is `int64_t` for the same reason.
    dynamic_length_bytes: Option<i64>,
    /// Set by a decoder that wants its field omitted from the output
    /// while the walk carries on past it — canboat's `g_skip`
    /// (print.c). Read and cleared by the field walker.
    skip_field: bool,
    /// Which definition of `target_pgn` the parameters address, and
    /// whether it had to be borrowed from the catch-all. Resolved at
    /// the first VARIABLE field and reused for the rest of the record,
    /// mirroring the C's `g_refPgn` (analyzer.c).
    resolved_target: Option<(&'static PgnInfo, bool)>,
}

fn decode_fields(
    info: &'static PgnInfo,
    data: &[u8],
    db: &PgnDatabase,
) -> Result<(Vec<DecodedField>, [bool; 2]), DecodeError> {
    let mut out = Vec::with_capacity(info.fields.len());
    let mut ctx = DecodeContext::default();
    let start1 = info.repeating_field_set1_start_field;
    let size1 = info.repeating_field_set1_size;
    let count_field1 = info.repeating_field_set1_count_field;
    let start2 = info.repeating_field_set2_start_field;
    let size2 = info.repeating_field_set2_size;
    let count_field2 = info.repeating_field_set2_count_field;
    let mut entered = [false, false];

    let mut i = 0usize;
    // Running bit cursor — used both for variable-length fields that
    // lack an explicit BitOffset and as the entry point for repeating
    // sets that follow other variable-length content.
    let mut cursor_bits: u32 = 0;
    while i < info.fields.len() {
        let f = &info.fields[i];

        // Repeating set 1 starts here? `count_field1` can be absent
        // (e.g. PGN 126464 PGN List) — in that case canboat repeats
        // until the payload runs out.
        if let (Some(start), Some(size)) = (start1, size1)
            && (f.order as u32) == start
        {
            // Only mark as "entered" if the start field is
            // actually within the payload — canboat C's main
            // loop stops once `startBit >> 3 >= length`, so a
            // truncated PGN payload never reaches the start
            // field and never emits the list opener.
            let bit_offset = f.bit_offset.unwrap_or(cursor_bits);
            if (bit_offset as usize) < data.len() * 8 {
                entered[0] = true;
            }
            cursor_bits = decode_repeating(
                info,
                data,
                db,
                &mut out,
                &mut ctx,
                i,
                size as usize,
                count_field1,
                cursor_bits,
                1,
            );
            i += size as usize;
            continue;
        }
        // Repeating set 2?
        if let (Some(start), Some(size)) = (start2, size2)
            && (f.order as u32) == start
        {
            let bit_offset = f.bit_offset.unwrap_or(cursor_bits);
            if (bit_offset as usize) < data.len() * 8 {
                entered[1] = true;
            }
            cursor_bits = decode_repeating(
                info,
                data,
                db,
                &mut out,
                &mut ctx,
                i,
                size as usize,
                count_field2,
                cursor_bits,
                2,
            );
            i += size as usize;
            continue;
        }

        if f.condition.is_some() {
            i += 1;
            continue;
        }
        // Variable-length fields (STRING_LAU and friends) after the
        // first one have no `BitOffset` in canboat.json — they sit at
        // the byte after the previous variable field ends. Use a
        // running `cursor_bits` for those; honor an explicit BitOffset
        // when one is given.
        let effective_offset = f.bit_offset.unwrap_or(cursor_bits);
        if let Some((decoded, bits_consumed)) =
            decode_one_field_at(f, info, data, db, effective_offset, &mut ctx)
        {
            cursor_bits = effective_offset + bits_consumed;
            // `skip_field` omits the field but keeps the walk going,
            // and the cursor has already advanced past it.
            if core::mem::take(&mut ctx.skip_field) {
                i += 1;
                continue;
            }
            out.push(decoded);
        }
        i += 1;
    }
    Ok((out, entered))
}

/// Compute `DecodedPgn::index_by_order` from a freshly-decoded
/// `fields` Vec. For every top-level field (`repeat_set == 0`),
/// record its `Vec` position at slot `field.order - 1`. Slots with
/// no decoded field — truncated payloads, NotAvailable filtered
/// out, or schema positions belonging to a repeating set — keep
/// the `i8::MIN` sentinel.
pub(crate) fn build_index_by_order(fields: &[DecodedField]) -> [i8; 32] {
    let mut idx = [i8::MIN; 32];
    for (pos, f) in fields.iter().enumerate() {
        if f.repeat_set != 0 {
            continue;
        }
        if let Some(slot) = (f.order() as usize).checked_sub(1)
            && slot < idx.len()
            && pos < i8::MAX as usize
        {
            idx[slot] = pos as i8;
        }
    }
    idx
}

#[allow(clippy::too_many_arguments)]
fn decode_repeating(
    info: &'static PgnInfo,
    data: &[u8],
    db: &PgnDatabase,
    out: &mut Vec<DecodedField>,
    ctx: &mut DecodeContext,
    start_idx: usize,
    set_size: usize,
    count_field_order: Option<u32>,
    base_cursor: u32,
    set_number: u8,
) -> u32 {
    // Look up the count from the already-decoded fields when a count
    // field is set; otherwise repeat until the payload runs out
    // (matches canboat's default g_variableFieldRepeat[0] = 255 path).
    // Canboat's `printFields` always enters the repeat block with
    // `repetition = 1` once it reaches the set's start field, even
    // when the count field decoded to zero (see analyzer.c:1284 vs
    // 1300). The decremented `variableFields` counter only triggers
    // *additional* loops, so count=0 yields exactly one iteration as
    // long as payload remains. Match that here: floor count at 1
    // when it's set, fall back to "iterate until payload runs out"
    // when the count field is NotAvailable or otherwise unresolved.
    let raw_count: Option<u32> = count_field_order.and_then(|cf| {
        out.iter()
            .find(|d| (d.order() as u32) == cf)
            .and_then(|d| match &d.value {
                FieldValue::Integer(n) if *n >= 0 => Some(*n as u32),
                FieldValue::Number(n) if *n >= 0.0 => Some(*n as u32),
                _ => None,
            })
    });
    // count=0 still produces exactly one (truncated) iteration — see
    // the long comment above the call site. Track this case so the text
    // formatter only labels the *first* field with " 1".
    let count_was_zero = raw_count == Some(0);
    let max_iters: u32 = match raw_count {
        Some(n) => n.max(1),
        None => u32::MAX,
    };

    let set = &info.fields[start_idx..start_idx + set_size];
    if set.is_empty() {
        return base_cursor;
    }
    let payload_bits = (data.len() as u32).saturating_mul(8);

    // Use the iteration's first field's BitOffset if available — that
    // anchors iteration 0 to the canboat.json layout. After that we
    // run on a per-iteration sub-cursor that advances by each field's
    // actual `bits_consumed` (handles variable-length VARIABLE fields).
    let mut iter_cursor = set[0].bit_offset.unwrap_or(base_cursor);

    for iter in 0..max_iters {
        if iter_cursor >= payload_bits {
            break;
        }
        let mut sub_cursor = iter_cursor;
        let mut produced_any = false;
        let mut first_field_of_iter = true;
        for sf in set {
            if sf.condition.is_some() {
                continue;
            }
            // Iteration 0 honors the explicit BitOffset from JSON;
            // later iterations follow the running sub_cursor since
            // bit_offsets aren't repeated per iteration in JSON.
            let off = if iter == 0 {
                sf.bit_offset.unwrap_or(sub_cursor)
            } else {
                sub_cursor
            };
            if let Some((mut d, bits)) = decode_one_field_at(sf, info, data, db, off, ctx) {
                // Canboat's `repetition` counter is reset to 0 at the
                // top of each loop iteration when `*variableFields == 0`
                // — so a forced count=0 iteration only labels its first
                // field. Drop `repeat_index` (but keep `repeat_set` for
                // JSON list-grouping) on later fields to match.
                d.repeat_set = set_number;
                d.repeat_index = if count_was_zero && !first_field_of_iter {
                    None
                } else {
                    Some(iter)
                };
                first_field_of_iter = false;
                sub_cursor = off + bits;
                // The record is still real when a value is omitted for
                // being explicitly empty, so this iteration counts as
                // productive and the walk continues to the next record.
                if !core::mem::take(&mut ctx.skip_field) {
                    out.push(d);
                }
                produced_any = true;
            } else if let Some(bl) = sf.bit_length {
                // Field couldn't decode but has a known size; advance
                // past it so subsequent fields in this iteration still
                // line up.
                sub_cursor = off + bl;
            }
        }
        // If this iteration produced nothing decodable, stop — we ran
        // off the end of the payload.
        if !produced_any {
            break;
        }
        iter_cursor = sub_cursor;
    }
    iter_cursor
}

/// Decode one field. Returns the `DecodedField` and the number of
/// payload bits it actually consumed (which differs from `bit_length`
/// for variable-length types like STRING_LAU and VARIABLE).
fn decode_one_field_at(
    f: &'static FieldInfo,
    info: &'static PgnInfo,
    data: &[u8],
    db: &PgnDatabase,
    bit_offset: u32,
    ctx: &mut DecodeContext,
) -> Option<(DecodedField, u32)> {
    let signed = f.signed.unwrap_or(false);
    let offset_k = f.offset.unwrap_or(0);

    // STRING_LAU figures out its own length from the data byte.
    if matches!(f.field_type, Some(FieldType::StringLau)) {
        let (value, bits_consumed) = decode_string_lau(data, bit_offset);
        return Some((
            DecodedField {
                info: f,
                value,
                bit_offset: Some(bit_offset),
                bit_length: Some(bits_consumed),
                repeat_index: None,
                repeat_set: 0,
                overrides: None,
            },
            bits_consumed,
        ));
    }

    // VARIABLE: dynamic field type — look up the target field's
    // metadata via (ctx.target_pgn, ctx.current_param_idx), then
    // recursively decode with that metadata at the current cursor.
    if matches!(f.field_type, Some(FieldType::Variable)) {
        return decode_variable(f, data, db, bit_offset, ctx);
    }

    // DYNAMIC_FIELD_VALUE has `BitLengthVariable: true` and no
    // explicit `BitLength` in canboat.json — its width comes from
    // the matching DYNAMIC_FIELD_LENGTH (or the resolved KEY's table
    // entry). Handle it before the `bit_length?` check below would
    // bail out.
    if matches!(f.field_type, Some(FieldType::DynamicFieldValue)) {
        // Capture unit / resolution / precision from the resolved key
        // entry *before* `decode_dynamic_field_value` drains the
        // context slots. The override is stashed alongside the
        // DecodedField so `f.unit()` etc. resolve through it before
        // falling back to the FieldInfo.
        let overrides = ctx.dynamic_field_type.as_ref().map(|v| {
            Box::new(FieldOverrides {
                unit: v.unit,
                resolution: v.resolution,
                precision: v.precision,
            })
        });
        let (val, consumed_bits) = decode_dynamic_field_value(data, bit_offset, db, ctx);
        return Some((
            DecodedField {
                info: f,
                value: val,
                bit_offset: Some(bit_offset),
                bit_length: Some(consumed_bits),
                repeat_index: None,
                repeat_set: 0,
                overrides,
            },
            consumed_bits,
        ));
    }

    // STRING_LZ / BINARY with `BitLengthVariable: true` and no
    // explicit `BitLength` — read to the payload end (canboat's
    // equivalent: `if (*bits == 0)` in `fieldPrintBinary` /
    // `printString`). Falls through to the normal decode path with a
    // zero bit_length so the decoders know to consume what's left.
    if matches!(
        f.field_type,
        Some(FieldType::StringLz) | Some(FieldType::Binary)
    ) && f.bit_length.is_none()
    {
        let payload_bits = (data.len() as u32).saturating_mul(8);
        let avail = payload_bits.saturating_sub(bit_offset);
        let avail = avail - (avail & 7); // round down to whole bytes
        let value = match f.field_type {
            Some(FieldType::StringLz) => decode_string_lz(data, bit_offset, avail),
            _ => decode_binary(data, bit_offset, avail),
        };
        return Some((
            DecodedField {
                info: f,
                value,
                bit_offset: Some(bit_offset),
                bit_length: Some(avail),
                repeat_index: None,
                repeat_set: 0,
                overrides: None,
            },
            avail,
        ));
    }

    let bit_length = f.bit_length?;

    let value = match f.field_type {
        Some(FieldType::Number) | Some(FieldType::Decimal) => {
            decode_number(f, data, bit_offset, bit_length, signed, offset_k)
        }
        Some(FieldType::Float) => decode_float(data, bit_offset, bit_length),
        Some(FieldType::Lookup) => decode_lookup(f, data, bit_offset, bit_length, db),
        Some(FieldType::IndirectLookup) => {
            decode_indirect_lookup(f, info, data, bit_offset, bit_length, db)
        }
        Some(FieldType::BitLookup) => decode_bitlookup(f, data, bit_offset, bit_length, db),
        Some(FieldType::Reserved) => {
            decode_reserved(data, bit_offset, bit_length, signed, offset_k, true)
        }
        Some(FieldType::Spare) => {
            decode_reserved(data, bit_offset, bit_length, signed, offset_k, false)
        }
        Some(FieldType::Binary) => decode_binary(data, bit_offset, bit_length),
        Some(FieldType::Mmsi) => decode_mmsi(f, data, bit_offset, bit_length),
        Some(FieldType::Pgn) => decode_pgn_field(data, bit_offset, bit_length, db),
        Some(FieldType::Date) => decode_date(f, data, bit_offset, bit_length),
        Some(FieldType::Time) | Some(FieldType::Duration) => {
            decode_time(f, data, bit_offset, bit_length, signed)
        }
        Some(FieldType::StringFix) => decode_string_fix(data, bit_offset, bit_length),
        Some(FieldType::StringLz) => decode_string_lz(data, bit_offset, bit_length),
        Some(FieldType::StringLau) => unreachable!("STRING_LAU handled above"),
        Some(FieldType::Variable) => unreachable!("VARIABLE handled above"),
        Some(FieldType::IsoName) => decode_iso_name(data, bit_offset, bit_length, db),
        Some(FieldType::DynamicFieldKey) => {
            decode_dynamic_field_key(f, data, db, bit_offset, bit_length, ctx)
        }
        Some(FieldType::DynamicFieldLength) => {
            decode_dynamic_field_length(f, data, bit_offset, bit_length, ctx)
        }
        Some(FieldType::DynamicFieldValue) => unreachable!("DYNAMIC_FIELD_VALUE handled above"),
        Some(FieldType::FieldIndex) => decode_number(f, data, bit_offset, bit_length, signed, 0),
        None => FieldValue::Unsupported {
            field_type: "<no field type>",
        },
    };

    // Update the running context based on what we just decoded so the
    // next field can interpret VARIABLE / FIELD_INDEX correctly.
    match &value {
        FieldValue::Pgn { value: p, .. } => ctx.target_pgn = Some(*p),
        FieldValue::Integer(n) if matches!(f.field_type, Some(FieldType::FieldIndex)) => {
            ctx.current_param_idx = Some(*n as u32);
        }
        _ => {}
    }
    // canboat C also picks up dynamic field lengths by FIELD NAME
    // (`analyzer/analyzer.c::fillGlobalsBasedOnFieldName`): any field
    // literally named `Length` updates `g_length`, which the next
    // DYNAMIC_FIELD_VALUE then consumes. Several PGNs use a regular
    // NUMBER field for this rather than the explicit
    // `DYNAMIC_FIELD_LENGTH` type (e.g. Simnet Parameter Set 130846).
    if f.is_dynamic_length_marker {
        let raw_len = match &value {
            FieldValue::Integer(n) if *n >= 0 => Some(*n),
            FieldValue::Number(n) if *n >= 0.0 => Some(*n as i64),
            _ => None,
        };
        if let Some(n) = raw_len {
            ctx.dynamic_length_bytes = Some(n - i64::from(f.dynamic_field_length_overhead));
        }
    }

    Some((
        DecodedField {
            info: f,
            value,
            bit_offset: Some(bit_offset),
            bit_length: Some(bit_length),
            repeat_index: None,
            repeat_set: 0,
            overrides: None,
        },
        bit_length,
    ))
}

/// Resolve a VARIABLE field by looking up the target field's metadata
/// in the database, then decoding with that metadata at the current
/// cursor.
///
/// Used by PGN 126208 group functions where a `Parameter` /
/// `FIELD_INDEX` field picks one of the target PGN's fields, and the
/// next VARIABLE field carries that field's value in its native shape.
/// True for the four proprietary PGN ranges (`IS_PGN_PROPRIETARY`,
/// common/common.h).
fn is_pgn_proprietary(n: u32) -> bool {
    (0xEF00..=0xEFFF).contains(&n)
        || (0xFF00..=0xFFFF).contains(&n)
        || (0x1_EF00..=0x1_EFFF).contains(&n)
        || (0x1_FF00..=0x1_FFFF).contains(&n)
}

/// Pick the definition of `pgn_id` that a PGN 126208 parameter list
/// describes. Port of `getMatchingPgnByParameters` (analyzer/pgn.c).
///
/// `params` starts at the parameter-count byte: `[count, index, value…,
/// index, value…]`, where each `index` is a 1-based field number of the
/// target PGN and the value's width comes from that field. Walking the
/// list with a candidate's own field sizes is what makes it a test: a
/// definition whose `Match` fields all agree with the values present is
/// the answer, and a definition that disagrees (or that has no such
/// field at all) steps out.
///
/// Returns `None` when nothing matches — for a proprietary PGN with
/// only Manufacturer and Industry given, that is the normal outcome.
fn match_pgn_by_parameters(
    db: &PgnDatabase,
    pgn_id: u32,
    params: &[u8],
) -> Option<&'static PgnInfo> {
    let first = db.first_pgn(pgn_id)?;

    // A proprietary PGN can only be narrowed by Manufacturer (field 1)
    // and Industry Code (field 3); without both there is nothing to go
    // on and the C bails out rather than guessing.
    if is_pgn_proprietary(pgn_id)
        && (params.len() < 6 || params[0] < 2 || params[1] != 0x01 || params[4] != 3)
    {
        return None;
    }

    // `hasMatchFields` is set per *definition* (fieldtype.c), and the C
    // tests it on the first one only -- so a PGN whose leading
    // definition needs no matching is answered by that definition
    // outright. This is how 126720 resolves: its catch-all is listed
    // first and carries no `Match`, so a Raymarine request lands there
    // without ever considering the 52 manufacturer variants behind it.
    if !first.fields.iter().any(|fi| fi.match_value.is_some()) {
        return Some(first);
    }

    let nparams = *params.first()? as usize;
    'variant: for cand in db.pgn_variants(pgn_id) {
        let mut d = 1usize;
        // Exactly `nparams` pairs — walking to the end of the data
        // instead would read trailing bytes as a bogus extra parameter.
        for _ in 0..nparams {
            if d >= params.len() {
                break; // truncated list: what was there matched
            }
            let index = params[d] as usize;
            d += 1;
            if index == 0 || index > cand.fields.len() {
                continue 'variant;
            }
            let field = &cand.fields[index - 1];
            let bits = field.bit_length.unwrap_or(0) as usize;
            if let Some(want) = field.match_value {
                let Some(ex) = extract_bits(params, d * 8, bits, field.signed.unwrap_or(false), 0)
                else {
                    continue 'variant;
                };
                if ex.value != want {
                    continue 'variant;
                }
            }
            d += bits.div_ceil(8);
        }
        return Some(cand);
    }
    None
}

fn decode_variable(
    f: &'static FieldInfo,
    data: &[u8],
    db: &PgnDatabase,
    bit_offset: u32,
    ctx: &mut DecodeContext,
) -> Option<(DecodedField, u32)> {
    let target_pgn = ctx.target_pgn?;
    let target_idx = ctx.current_param_idx?;
    // Which definition of the target PGN does this parameter address?
    //
    // canboat answers this in `fieldPrintVariable` (analyzer.c): first try to
    // resolve a *variant* from the parameter list itself -- a request for
    // 130833 that sets Manufacturer=Furuno picks
    // furunoShipParametersAndAntennaPosition, and its parameter 4 is a real
    // typed field. Only when that fails (126720 has 53 variants and a
    // Manufacturer/Industry pair narrows nothing) borrow the *catch-all*
    // definition, whose leading fields every variant shares.
    //
    // It has to be the catch-all and not merely the first definition: the
    // first definition of 126720 is somebody's specific variant whose
    // Manufacturer Code is a `Match` field, so borrowing it would make every
    // request from another manufacturer decode against the wrong layout.
    //
    // Resolved once per record and cached, as the C caches `g_refPgn`.
    let (target_info, used_catch_all) = match ctx.resolved_target {
        Some(r) => r,
        None => {
            // The C hands `getMatchingPgnByParameters` the message from two
            // bytes back: [parameter count][field index][value...]. At the
            // first VARIABLE field the cursor sits on the value, so -1 is this
            // parameter's index byte and -2 the count.
            let params = (bit_offset as usize / 8)
                .checked_sub(2)
                .and_then(|s| data.get(s..))
                .unwrap_or(&[]);
            let r = match match_pgn_by_parameters(db, target_pgn, params) {
                Some(p) => (p, false),
                None => (
                    db.fallback_pgn(target_pgn)
                        .or_else(|| db.first_pgn(target_pgn))?,
                    true,
                ),
            };
            ctx.resolved_target = Some(r);
            r
        }
    };
    let target_field = target_info
        .fields
        .iter()
        .find(|tf| (tf.order as u32) == target_idx)?;
    // The catch-all's trailing `Data` field is variable-length with nothing to
    // size it, so the value is the rest of the message -- for a request or
    // command that reads as "the field starts with these bytes". Only when we
    // fell back: a *resolved* PGN's variable-length fields (126998's
    // Configuration Information strings) decode properly through the normal
    // path.
    if used_catch_all
        && (target_field.bit_length.unwrap_or(0) == 0
            || target_field.bit_length_variable == Some(true))
    {
        let start = bit_offset as usize;
        let total = data.len() * 8;
        if start >= total {
            return None;
        }
        let bits = (total - start) as u32;
        let value = decode_binary(data, bit_offset, bits);
        return Some((
            DecodedField {
                info: f,
                value,
                bit_offset: Some(bit_offset),
                bit_length: Some(bits),
                repeat_index: None,
                repeat_set: 0,
                overrides: None,
            },
            bits,
        ));
    }
    // Recurse with the target field's metadata at the current cursor.
    // The outer field's name (e.g. "Value", "Selection Value") wraps
    // the decoded result so the JSON keeps the right field label.
    let mut sub_ctx = DecodeContext::default();
    let (sub, bits) = decode_one_field_at(
        target_field,
        target_info,
        data,
        db,
        bit_offset,
        &mut sub_ctx,
    )?;
    // canboat rounds the VARIABLE field's consumption UP to whole
    // bytes — `*bits = (*bits + 7) & ~0x07` in `fieldPrintVariable`.
    // Without this, Set2 in PGN 126208 Read Fields lands 5 bits early
    // and reads the next Parameter's bits from the wrong nibble.
    let bits_byte_aligned = bits.div_ceil(8) * 8;
    // VARIABLE override: take the target field's unit/resolution/precision
    // verbatim. The outer field's id/name (e.g. "Value", "Selection Value")
    // is still what JSON should render, so we keep `info: f`.
    let overrides = Some(Box::new(FieldOverrides {
        unit: target_field.unit,
        resolution: target_field.resolution,
        precision: target_field.precision,
    }));
    Some((
        DecodedField {
            info: f,
            // For VARIABLE fields, the `-debug` formatter wants the
            // diagnostic to reflect the sub-field's actual width when
            // it has one (so a 3-bit LOOKUP target shows
            // `bits = "010"`). When the sub-field is itself variable-
            // length (e.g. STRING_LAU), fall back to the consumed
            // byte-aligned size so we still emit the bytes annotation.
            value: sub.value,
            bit_offset: Some(bit_offset),
            bit_length: target_field.bit_length.or(Some(bits_byte_aligned)),
            repeat_index: None,
            repeat_set: 0,
            overrides,
        },
        bits_byte_aligned,
    ))
}

fn decode_number(
    f: &FieldInfo,
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    signed: bool,
    _offset_k: i64,
) -> FieldValue {
    // canboat.json's `Offset` is in DISPLAY units (e.g. PEUKERT_EXPONENT
    // Offset=1 means "add 1.0 to the displayed exponent"), NOT a raw
    // J1939 Excess-K shift on the integer extraction. canboat C also
    // forces unsigned extraction when `Offset != 0` regardless of the
    // field's nominal Signed flag — see extractNumber's
    // `if (hasSign && field->offset)` path. Replicate both here.
    let display_offset = f.offset.map(|o| o as f64).unwrap_or(0.0);
    let has_display_offset = f.offset.unwrap_or(0) != 0;
    let effective_signed = if has_display_offset { false } else { signed };
    let Some(ex) = extract_bits(
        data,
        bit_offset as usize,
        bit_length as usize,
        effective_signed,
        0,
    ) else {
        return FieldValue::NotAvailable;
    };
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    let resolution = f.resolution.unwrap_or(1.0);
    let unit = f.unit;
    if resolution == 1.0
        && unit.is_none()
        && f.physical_quantity.is_none()
        && f.unit_offset == 0.0
        && display_offset == 0.0
    {
        FieldValue::Integer(ex.value)
    } else {
        FieldValue::Number(ex.value as f64 * resolution + display_offset + f.unit_offset)
    }
}

fn decode_float(data: &[u8], bit_offset: u32, bit_length: u32) -> FieldValue {
    if bit_length != 32 {
        return FieldValue::Unsupported {
            field_type: "FLOAT (non-32-bit)",
        };
    }
    let Some(ex) = extract_bits(data, bit_offset as usize, 32, false, 0) else {
        return FieldValue::NotAvailable;
    };
    let bits = ex.value as u32;
    FieldValue::Float(f32::from_bits(bits) as f64)
}

fn decode_lookup(
    f: &FieldInfo,
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    db: &PgnDatabase,
) -> FieldValue {
    let Some(ex) = extract_bits(data, bit_offset as usize, bit_length as usize, false, 0) else {
        return FieldValue::NotAvailable;
    };
    // Name wins: when the enumeration explicitly maps the raw value,
    // emit the name even if the same raw value would have been a
    // sentinel hint on the field. This mirrors the canboat 7.1.0
    // schema-2.5.0 contract — "an enumeration name always wins, so a
    // sentinel position the lookup names is not emitted" (PR #704).
    let raw = ex.raw;
    let name = f
        .lookup_enumeration
        .and_then(|n| db.lookup(n))
        .and_then(|t| t.get(raw))
        .map(|v| v.name);
    if name.is_some() {
        return FieldValue::Lookup { value: raw, name };
    }
    // No name resolved — fall through to the per-field sentinel hints
    // (schema 2.5.0 carries them on ~35 % of LOOKUP fields). For the
    // remaining "all values valid" tables (MMSI-style identifier
    // lookups) the hints are all None and we emit the raw value.
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    // Still nothing, and the table does not name this value. canboat does not
    // fall back to printing the bare number here: fieldPrintLookup treats an
    // unnamed value in the top reserved band as unavailable, using a bound it
    // computes from the field width rather than any schema hint --
    //     *bits > 1 && value >= maxValue - (*bits > 2 ? 2 : 1)
    // so a 2-bit lookup reserves its top two values and anything wider its top
    // three. Without this, PGN 127501's `Indicator11` reported {"value":2}
    // where canboat omits the field entirely.
    if bit_length > 1 {
        let band = if bit_length > 2 { 2 } else { 1 };
        if ex.value >= ex.max - band {
            return FieldValue::NotAvailable;
        }
    }
    FieldValue::Lookup {
        value: raw,
        name: None,
    }
}

/// INDIRECT_LOOKUP: resolve `(value1, value2)` where `value1` is
/// pulled from another field within the same PGN, identified by
/// `LookupIndirectEnumerationFieldOrder`.
fn decode_indirect_lookup(
    f: &FieldInfo,
    info: &PgnInfo,
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    db: &PgnDatabase,
) -> FieldValue {
    let Some(ex) = extract_bits(data, bit_offset as usize, bit_length as usize, false, 0) else {
        return FieldValue::NotAvailable;
    };
    let raw = ex.raw;
    let name = (|| -> Option<&'static str> {
        let table_name = f.lookup_indirect_enumeration?;
        let val1_order = f.lookup_indirect_enumeration_field_order?;
        let val1_field = info.fields.iter().find(|x| x.order == val1_order)?;
        let val1_off = val1_field.bit_offset?;
        let val1_len = val1_field.bit_length?;
        let val1 = extract_bits(data, val1_off as usize, val1_len as usize, false, 0)?;
        db.indirect_lookup(table_name, val1.raw, raw)
    })();
    if name.is_some() {
        return FieldValue::Lookup { value: raw, name };
    }
    // Name didn't resolve — try the per-field sentinel hints. Schema
    // 2.5.0 declares them on both INDIRECT_LOOKUP fields in the
    // database (PGN 60928 Device Function / Class).
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    FieldValue::Lookup {
        value: raw,
        name: None,
    }
}

fn decode_bitlookup(
    f: &FieldInfo,
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    db: &PgnDatabase,
) -> FieldValue {
    let Some(ex) = extract_bits(data, bit_offset as usize, bit_length as usize, false, 0) else {
        return FieldValue::NotAvailable;
    };
    let raw = ex.value as u64;
    // Keep the BitField even when no bits are set — formatters
    // handle the empty case themselves: JSON drops it, text emits
    // "None" (matches canboat).
    //
    // Walk the *bit positions*, not the enumeration's entries: canboat
    // loops `bit < *bits` and emits every set bit, naming it if the
    // table has an entry and leaving the name null if not
    // (`fieldPrintBitLookup`, print.c). Iterating the table instead
    // made a set bit that nothing names invisible — PGN 65305's `Mode`
    // sets bit 1, which SIMNET_AP_STATUS does not name, and it went
    // missing from every autopilot status record.
    let mut bits = Vec::new();
    let table = f.lookup_bit_enumeration.and_then(|n| db.bit_lookup(n));
    for bit in 0..bit_length.min(64) {
        if raw & (1u64 << bit) != 0 {
            bits.push((
                1u64 << bit,
                table.and_then(|t| t.get(bit as u8)).map(|v| v.name),
            ));
        }
    }
    FieldValue::BitField { value: raw, bits }
}

fn decode_reserved(
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    signed: bool,
    offset_k: i64,
    is_reserved: bool,
) -> FieldValue {
    let Some(ex) = extract_bits(
        data,
        bit_offset as usize,
        bit_length as usize,
        signed,
        offset_k,
    ) else {
        return FieldValue::NotAvailable;
    };
    let raw = ex.value as u64;
    // canboat skips Reserved fields whose value is all-ones (the
    // default "this is unused" state). Surface that here too — but
    // formatters do the actual omission so the `-debug` byte/bit
    // diagnostic survives in callers that care.
    if is_reserved && ex.value == ex.max {
        return FieldValue::Reserved {
            value: raw,
            bytes: Vec::new(),
            bit_length,
        };
    }
    // A RESERVED or SPARE field that does need printing is handed
    // straight to `fieldPrintBinary` by the C (`fieldPrintReserved` /
    // `fieldPrintSpare`, print.c), so every bit stays where it sat:
    // the first byte is masked to the field's width and shifted back
    // into position, and later bytes come whole off the message.
    //
    // Packing the extracted value LSB-first instead moved the bits down
    // to bit 0 — PGN 130824's 2-bit Reserved at bit 11 printed `02`
    // where canboat prints `10`, and the same shift showed up on every
    // proprietary PGN's Reserved field.
    let bytes = match decode_binary(data, bit_offset, bit_length) {
        FieldValue::Binary(b) => b,
        _ => Vec::new(),
    };
    if is_reserved {
        FieldValue::Reserved {
            value: raw,
            bytes,
            bit_length,
        }
    } else {
        FieldValue::Spare {
            value: raw,
            bytes,
            bit_length,
        }
    }
}

/// ISO_NAME: 64-bit packed identifier. The same 8 bytes also form a
/// valid PGN 60928 (ISO Address Claim) payload, so the structured form
/// is built by re-running the decoder against PGN 60928's field list
/// on those 8 bytes — matching `fieldPrintName` in `analyzer/print.c`.
fn decode_iso_name(data: &[u8], bit_offset: u32, bit_length: u32, db: &PgnDatabase) -> FieldValue {
    if bit_length != 64 || (bit_offset & 7) != 0 {
        return FieldValue::Unsupported {
            field_type: "ISO_NAME (unaligned or non-64-bit)",
        };
    }
    let byte_off = (bit_offset / 8) as usize;
    if byte_off + 8 > data.len() {
        return FieldValue::NotAvailable;
    }
    let mut value: u64 = 0;
    for i in 0..8 {
        value |= (data[byte_off + i] as u64) << (i * 8);
    }
    // An ISO NAME reserves its top *two* values rather than going
    // through the usual per-field sentinel machinery, which is
    // suppressed at 64 bits anyway (QUIRKS Q18): `fieldPrintName`
    // (print.c) hands anything above `maxValue - 2` to `printEmpty`.
    // Expanding one of those into subfields would invent a device —
    // an all-ones NAME reads as Device Instance 31, "Arbitrary address
    // capable: Yes" and so on, none of which is on the wire.
    match value {
        u64::MAX => return FieldValue::NotAvailable,
        v if v == u64::MAX - 1 => return FieldValue::OutOfRange { value },
        _ => {}
    }
    let sub = &data[byte_off..byte_off + 8];
    let subfields = match db.first_pgn(60928) {
        Some(pgn) => decode_fields(pgn, sub, db).map(|x| x.0).unwrap_or_default(),
        None => Vec::new(),
    };
    FieldValue::IsoName { value, subfields }
}

fn decode_binary(data: &[u8], bit_offset: u32, bit_length: u32) -> FieldValue {
    let bit_offset = bit_offset as usize;
    let bit_length = bit_length as usize;
    // Whole-byte aligned: take a clean slice. Fallback PGNs like the
    // proprietary catch-alls declare BINARY widths well past the
    // actual payload (e.g. 1768 bits for `Data` in PGN 126720) —
    // canboat just renders whatever bytes are present, so clamp to
    // the available range rather than emitting NotAvailable.
    if bit_offset & 7 == 0 && bit_length & 7 == 0 {
        let start = bit_offset / 8;
        if start >= data.len() {
            return FieldValue::NotAvailable;
        }
        let declared_end = start + bit_length / 8;
        let end = declared_end.min(data.len());
        return FieldValue::Binary(data[start..end].to_vec());
    }
    // Not byte-aligned. This is NOT a clean bit-extraction, and must not
    // become one: `fieldPrintBinary` masks the *first* byte to the field's
    // width but then shifts it back to where it sat in the byte, and takes
    // every later byte whole. So PGN 130829's two 4-bit nibbles render as
    // "0F" (low) and "F0" (high) — the high nibble keeps its position rather
    // than being normalised down. Packing LSB-first instead reported both as
    // "0F", losing which nibble the bits came from.
    let start_byte = bit_offset / 8;
    let start_bit = bit_offset % 8;
    // canboat clamps the width to what the payload actually holds.
    let avail = data.len().saturating_mul(8).saturating_sub(bit_offset);
    let bit_length = bit_length.min(avail);
    if bit_length == 0 {
        return FieldValue::NotAvailable;
    }
    let nbytes = bit_length.div_ceil(8);
    let mut out = Vec::with_capacity(nbytes);
    let mut remaining = bit_length as isize;
    for i in 0..nbytes {
        let Some(&raw) = data.get(start_byte + i) else {
            break;
        };
        let mut byte = raw;
        if i == 0 && start_bit != 0 {
            byte >>= start_bit;
            if (remaining as usize) + start_bit < 8 {
                byte &= (1u8 << remaining) - 1;
            }
            byte <<= start_bit;
            remaining -= (8 - start_bit) as isize;
        } else {
            if remaining < 8 {
                byte &= (1u8 << remaining) - 1;
            }
            remaining -= 8;
        }
        out.push(byte);
    }
    FieldValue::Binary(out)
}

fn decode_mmsi(f: &FieldInfo, data: &[u8], bit_offset: u32, bit_length: u32) -> FieldValue {
    if bit_length != 32 {
        return FieldValue::Unsupported {
            field_type: "MMSI (non-32-bit)",
        };
    }
    let Some(ex) = extract_bits(data, bit_offset as usize, 32, false, 0) else {
        return FieldValue::NotAvailable;
    };
    // The MMSI FieldType declares `Sentinels: None`, so the schema
    // carries no per-field hints and `sentinel_field_value` finds
    // nothing. That is not the whole rule: an MMSI field's RangeMax is
    // 999999999, well inside 32 bits, and the C reserves its top three
    // values on that basis alone (`fieldPrintMMSI` goes through
    // `extractNumberNotEmpty`). PGN 130842's `Mothership User ID` reads
    // 0xFFFFFFFF whenever the Class B unit has no mothership, and
    // canboat omits it rather than printing 4294967295.
    if let Some(sent) = range_max_sentinel(f, ex, 32) {
        return sent;
    }
    FieldValue::Mmsi(ex.value as u32)
}

fn decode_pgn_field(data: &[u8], bit_offset: u32, bit_length: u32, db: &PgnDatabase) -> FieldValue {
    if bit_length != 24 {
        return FieldValue::Unsupported {
            field_type: "PGN (non-24-bit)",
        };
    }
    let Some(ex) = extract_bits(data, bit_offset as usize, 24, false, 0) else {
        return FieldValue::NotAvailable;
    };
    let raw = ex.value as u32;
    // PDU1 (PF < 240): low byte of the PGN is the destination address,
    // so it's masked out. PDU2: low byte is the PS and part of the PGN.
    let pf = (raw >> 8) & 0xff;
    let pgn = if pf < 240 {
        raw & 0x00ff_ff00
    } else {
        raw & 0x00ff_ffff
    };
    // canboat only emits a name for the PGN field when there's
    // unambiguously one variant *without* `Match` constraints — that
    // is, the PGN number alone is enough to identify the message. PGN
    // 130820 (42 manufacturer variants, all match-gated) and PGN 65410
    // (one Airmar-only variant gated on Manufacturer Code) both fall
    // back to "value only" because we can't tell which variant applies
    // without actually decoding payload data.
    let variants: Vec<&'static PgnInfo> = db.pgn_variants(pgn).collect();
    let description =
        if variants.len() == 1 && !variants[0].fields.iter().any(|f| f.match_value.is_some()) {
            Some(variants[0].description)
        } else {
            None
        };
    FieldValue::Pgn {
        value: pgn,
        description,
    }
}

fn decode_date(f: &FieldInfo, data: &[u8], bit_offset: u32, bit_length: u32) -> FieldValue {
    if bit_length != 16 {
        return FieldValue::Unsupported {
            field_type: "DATE (non-16-bit)",
        };
    }
    let Some(ex) = extract_bits(data, bit_offset as usize, 16, false, 0) else {
        return FieldValue::NotAvailable;
    };
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    FieldValue::Date(ex.value as u16)
}

fn decode_time(
    f: &FieldInfo,
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    signed: bool,
) -> FieldValue {
    let Some(ex) = extract_bits(
        data,
        bit_offset as usize,
        bit_length as usize,
        signed,
        f.offset.unwrap_or(0),
    ) else {
        return FieldValue::NotAvailable;
    };
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    let resolution = f.resolution.unwrap_or(1.0);
    FieldValue::Time {
        raw: ex.value,
        seconds: ex.value as f64 * resolution,
    }
}

fn decode_string_fix(data: &[u8], bit_offset: u32, bit_length: u32) -> FieldValue {
    let bo = bit_offset as usize;
    let bl = bit_length as usize;
    if bo & 7 != 0 || bl & 7 != 0 {
        return FieldValue::Unsupported {
            field_type: "STRING_FIX (unaligned)",
        };
    }
    let start = bo / 8;
    if start >= data.len() {
        return FieldValue::NotAvailable;
    }
    // Cap to what is actually in the message rather than dropping the
    // field — `len = CB_MIN(len, dataLen)` in `fieldPrintStringFix`
    // (print.c). Navico's 130821 ASCII Data declares a 230-byte
    // Message and sends however much text it has, so the declared
    // width is a maximum, not a promise.
    let end = (start + bl / 8).min(data.len());
    // Canboat's `printString` first finds an embedded NUL and shortens
    // the string at that point (matches the C-string convention some
    // devices use to terminate inside a fixed-width buffer — e.g. a
    // Mastervolt Product Information field that has two records
    // separated by NULs), then trims the trailing padding run. Apply both passes here so STRING_FIX renders the leading
    // C-string portion only.
    let raw = &data[start..end];
    let mut len = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    while len > 0 && is_string_padding(raw[len - 1]) {
        len -= 1;
    }
    if len == 0 {
        return FieldValue::NotAvailable;
    }
    let s = String::from_utf8_lossy(&raw[..len]).into_owned();
    FieldValue::String(s)
}

/// STRING_LAU — length + encoding + payload, where the first byte is the
/// total size of the field (`len` including the header), the second byte
/// is an encoding control (`0` = UTF-16LE, `1` = ASCII / UTF-8), and the
/// remaining `len - 2` bytes are the payload.
///
/// Returns the decoded value plus the number of bits this field
/// consumed — variable, so callers must use this to advance the cursor.
fn decode_string_lau(data: &[u8], bit_offset: u32) -> (FieldValue, u32) {
    let bo = bit_offset as usize;
    if bo & 7 != 0 {
        return (
            FieldValue::Unsupported {
                field_type: "STRING_LAU (unaligned)",
            },
            8,
        );
    }
    let start = bo / 8;
    if start + 2 > data.len() {
        return (FieldValue::NotAvailable, 16);
    }
    let total_len = data[start] as usize;
    let encoding = data[start + 1];
    if total_len < 2 {
        return (FieldValue::NotAvailable, (total_len.max(2) * 8) as u32);
    }
    let body_len = total_len - 2;
    let body_end = (start + 2 + body_len).min(data.len());
    let body = &data[start + 2..body_end];

    let bits_consumed = (total_len * 8) as u32;
    let s = match encoding {
        0 => {
            // UTF-16LE: pairs of bytes are LE u16 code units.
            let mut code_units = Vec::with_capacity(body.len() / 2);
            let mut i = 0;
            while i + 1 < body.len() {
                code_units.push(u16::from_le_bytes([body[i], body[i + 1]]));
                i += 2;
            }
            String::from_utf16_lossy(&code_units)
        }
        _ => {
            // 1 = ASCII / UTF-8 (canboat doesn't differentiate). An
            // encoding byte of 0xff marks an *unset* field: the length
            // byte is present but the encoding and content are 0xff
            // filler (seen on the H5000 pilot in PGN 126998). Trim the
            // trailing 0xff / NUL / '@' / space run off the raw bytes
            // *before* decoding, so an all-filler body collapses to an
            // empty (omitted) field instead of a run of U+FFFD
            // replacement chars — canboat's printString trims the same
            // bytes, and its C analyzer would otherwise abort the whole
            // PGN on "Unhandled string type 255".
            let end = body
                .iter()
                .rposition(|&b| !is_string_padding(b))
                .map_or(0, |i| i + 1);
            String::from_utf8_lossy(&body[..end]).into_owned()
        }
    };
    // Canboat's `printString` trims trailing 0xff / NUL / '@' / spaces
    // *after* the UTF-16→UTF-8 conversion. If nothing's left, the field
    // is rendered as Unknown via `printEmpty`. Match that — and don't
    // trim raw UTF-16 bytes (the trailing NUL of a single ASCII glyph
    // would lose the char).
    let trimmed = trim_string_padding(&s);
    if trimmed.is_empty() {
        return (FieldValue::NotAvailable, bits_consumed);
    }
    (FieldValue::String(trimmed.to_string()), bits_consumed)
}

/// Strip trailing canboat-padding bytes (`\0`, `'@'`, space, `\xff`)
/// from a string. Matches the trailing-byte loop in `printString` in
/// analyzer/print.c.
/// The trailing bytes canboat's `printString` strips off a decoded
/// string: `0xff` (the NMEA 2000 filler), any `isspace()` character,
/// NUL, and `'@'` (badly converted AIS data).
///
/// `isspace()` is the whole C set, not just a space — Navico's 130847
/// ASCII Identifier ends its text with a newline, and trimming only
/// `' '` left it on the value.
fn is_string_padding(b: u8) -> bool {
    matches!(b, 0xff | 0x00 | b'@') || b.is_ascii_whitespace() || b == 0x0b
}

fn trim_string_padding(s: &str) -> &str {
    s.trim_end_matches(|c: char| match c {
        // 0xff survives as U+00FF when the bytes came through a
        // latin-1-ish path rather than as a UTF-8 replacement char.
        '\u{ff}' => true,
        c if c.is_ascii() => is_string_padding(c as u8),
        _ => false,
    })
}

fn decode_string_lz(data: &[u8], bit_offset: u32, bit_length: u32) -> FieldValue {
    let bo = bit_offset as usize;
    let bl = bit_length as usize;
    if bo & 7 != 0 {
        return FieldValue::Unsupported {
            field_type: "STRING_LZ (unaligned)",
        };
    }
    let start = bo / 8;
    if start >= data.len() {
        return FieldValue::NotAvailable;
    }
    // STRING_LZ format (canboat `fieldPrintStringLZ`): first byte is
    // the byte length of the content, then `len` bytes of payload,
    // then a NUL terminator. `bl` here is either the field's
    // declared bit width or 0 (BitLengthVariable → read to payload
    // end). Cap the embedded length to whatever's actually present.
    let region_end = if bl == 0 {
        data.len()
    } else {
        (start + bl / 8).min(data.len())
    };
    let len_byte = data[start] as usize;
    let content_start = start + 1;
    let max_avail = region_end.saturating_sub(content_start);
    let content_end = content_start + len_byte.min(max_avail);
    let raw = String::from_utf8_lossy(&data[content_start..content_end]);
    let trimmed = trim_string_padding(&raw);
    if trimmed.is_empty() {
        FieldValue::NotAvailable
    } else {
        FieldValue::String(trimmed.to_string())
    }
}

/// DYNAMIC_FIELD_KEY: decode the integer raw value and resolve it
/// through the field's `LookupFieldTypeEnumeration`. The resolved
/// entry is parked on the [`DecodeContext`] so the matching
/// DYNAMIC_FIELD_VALUE can read its type / bits / resolution / unit.
/// The output value mirrors a normal Lookup (number + resolved name).
fn decode_dynamic_field_key(
    f: &FieldInfo,
    data: &[u8],
    db: &PgnDatabase,
    bit_offset: u32,
    bit_length: u32,
    ctx: &mut DecodeContext,
) -> FieldValue {
    let Some(ex) = extract_bits(data, bit_offset as usize, bit_length as usize, false, 0) else {
        return FieldValue::NotAvailable;
    };
    let raw = ex.raw;
    let entry = f
        .lookup_field_type_enumeration
        .and_then(|n| db.field_type_lookup(n, raw));
    let name = entry.map(|e| e.name);
    ctx.dynamic_field_type = entry.copied();
    if name.is_some() {
        return FieldValue::Lookup { value: raw, name };
    }
    // Schema 2.5.0 declares all three hints on every DYNAMIC_FIELD_KEY
    // field in the database — fall through to the field-aware check.
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    FieldValue::Lookup {
        value: raw,
        name: None,
    }
}

/// DYNAMIC_FIELD_LENGTH: decode as a plain integer count of bytes
/// that the next DYNAMIC_FIELD_VALUE consumes.
fn decode_dynamic_field_length(
    f: &FieldInfo,
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    ctx: &mut DecodeContext,
) -> FieldValue {
    let Some(ex) = extract_bits(data, bit_offset as usize, bit_length as usize, false, 0) else {
        return FieldValue::NotAvailable;
    };
    // A sentinel-length value can't be used to drive the next
    // DYNAMIC_FIELD_VALUE decode, so don't update the context.
    if let Some(sent) = sentinel_field_value(f, ex) {
        return sent;
    }
    // The reported length may cover a per-record header sitting between
    // this field and the value — Navico's 130822/130823 dumps count the
    // class byte and the 16-bit data-type id — so take that off to get
    // the value's own width (`g_length = value - overhead`,
    // analyzer.c::fillGlobalsBasedOnFieldName).
    let len = ex.raw as i64 - i64::from(f.dynamic_field_length_overhead);
    ctx.dynamic_length_bytes = Some(len);
    FieldValue::Integer(ex.value)
}

/// DYNAMIC_FIELD_VALUE: read the resolved field-type metadata from
/// the context and decode the value using it. Falls back to a BINARY
/// dump (mirroring `fieldPrintKeyValue`'s `g_ftf == NULL` branch)
/// when the key wasn't resolved. Returns the value plus how many bits
/// it consumed so the iteration walker can advance correctly. Always
/// clears the dynamic-* slots in the context.
///
/// **No top-of-range sentinel is stripped here.** The value is decoded
/// against a pseudo-field synthesised from a fieldtype lookup entry,
/// and in the C those never get a `reservedCount`: that is resolved in
/// a pass over `pgnList`'s fields (fieldtype.c), which a lookup entry
/// is not a member of. It therefore stays 0, `extractNumberNotEmpty`'s
/// threshold becomes `maxValue`, and nothing is ever above it — so an
/// all-ones dynamic value prints as a number. B&G's `Trip 2 Time`
/// reads 0xFFFFFFFF on an unstarted trip and canboat renders it
/// `1193:02:47.295`, where stripping it as "not available" would drop
/// the field.
///
/// ISO_NAME is the exception, and not via this mechanism:
/// `fieldPrintName` tests `value > maxValue - 2` itself, so it applies
/// on this path as much as on any other.
fn decode_dynamic_field_value(
    data: &[u8],
    bit_offset: u32,
    db: &PgnDatabase,
    ctx: &mut DecodeContext,
) -> (FieldValue, u32) {
    let entry = ctx.dynamic_field_type.take();
    let explicit_len = ctx.dynamic_length_bytes.take();
    let length_bits: Option<i64> = explicit_len
        .map(|n| n * 8)
        .or_else(|| entry.as_ref().and_then(|e| e.bit_length()).map(i64::from));
    // An explicit length of zero means the value is present but empty
    // — PGN 130823 directory entries that declare only a data type.
    // canboat omits the field rather than printing `""`
    // (`g_lengthValid && *bits == 0` → `g_skip`, print.c). Note this
    // is checked before the resolved-key branch below, so a known
    // field type with a zero length is dropped too.
    if explicit_len == Some(0) {
        ctx.skip_field = true;
        return (FieldValue::NotAvailable, 0);
    }
    let bits_signed = length_bits.unwrap_or(0);
    let remaining = (data.len() as u32 * 8).saturating_sub(bit_offset);
    if bits_signed < 0 {
        // A record declaring a length below its own header width. The
        // C subtracts the overhead into a signed `g_length` and then
        // casts to `size_t` for the bounds test below, so the negative
        // width wraps to something enormous, sails past that test and
        // reaches `fieldPrintBinary`, which clamps it to what is left
        // of the packet. Accidental, but the resulting reading — dump
        // the rest of the frame — is the useful one, so match it
        // deliberately rather than by arithmetic.
        return (decode_binary(data, bit_offset, remaining), remaining);
    }
    // The declared value runs past the end of the packet: the frame
    // ended mid-record. That is how a repeating key/value group
    // normally terminates — the device packs as many records as fit
    // and the last one is cut off — so canboat drops the partial value
    // and stops there rather than failing the whole PGN (print.c).
    if (data.len() as i64) < (bit_offset as i64 + bits_signed) >> 3 {
        ctx.skip_field = true;
        return (FieldValue::NotAvailable, remaining);
    }
    let bits = bits_signed as u32;
    let Some(entry) = entry else {
        // The key resolved to no field type, so there is no width to
        // decode against. When there was no explicit length either, the
        // value is simply the rest of the frame, and canboat dumps it
        // as raw bytes — `*bits = dataLen * 8 - startBit` before
        // `fieldPrintBinary` (fieldPrintKeyValue, print.c).
        //
        // PGN 130845 is the case that needs it: its `Value` has no
        // preceding DYNAMIC_FIELD_LENGTH at all, so an unrecognised
        // SIMNET_KEY_VALUE key leaves nothing to size the field with,
        // and taking the declared zero would print `""` over bytes that
        // are plainly on the bus.
        let bits = if bits == 0 && explicit_len.is_none() {
            remaining
        } else {
            bits
        };
        return (decode_binary(data, bit_offset, bits), bits);
    };
    let signed = entry.signed || matches!(entry.field_type, Some(ft) if ft.starts_with("FIX"));
    let val = match entry.field_type {
        Some(ft) if ft.starts_with("NUMBER") || ft.starts_with("FIX") || ft.starts_with("UFIX") => {
            decode_dynamic_number(data, bit_offset, bits, signed, &entry)
        }
        Some("LOOKUP") => match entry.lookup_enumeration {
            Some(name) => {
                let Some(ex) = extract_bits(data, bit_offset as usize, bits as usize, false, 0)
                else {
                    return (FieldValue::NotAvailable, bits);
                };
                let raw = ex.value as u64;
                let nm = db.lookup(name).and_then(|t| t.get(raw)).map(|v| v.name);
                FieldValue::Lookup {
                    value: raw,
                    name: nm,
                }
            }
            None => decode_binary(data, bit_offset, bits),
        },
        Some("DURATION") | Some("TIME") => {
            // canboat C's lookup-generated-data.h is what really decides DURATION
            // signedness: most entries are `DURATION_UFIX*` (unsigned,
            // e.g. Layline Time, Sailing Time to Waypoint, Sailing
            // ETA, Trip Time), with `DURATION_FIX*` (signed) reserved
            // for fields that genuinely carry negative values like
            // Race Timer / Timezone offset. The canboat.json
            // simplifies all of these to `"FieldType":"DURATION"`,
            // so we default to UNSIGNED and only flip to signed when
            // the lookup carries an explicit `Signed:true` or the
            // name is in the small set known to be signed in
            // canboat's pgn.h.
            let known_signed_duration = matches!(entry.name, "Race Timer" | "Timezone offset");
            let want_signed = signed || known_signed_duration;
            let Some(ex_unsigned) =
                extract_bits(data, bit_offset as usize, bits as usize, false, 0)
            else {
                return (FieldValue::NotAvailable, bits);
            };
            let ex = if want_signed {
                match extract_bits(data, bit_offset as usize, bits as usize, true, 0) {
                    Some(e) => e,
                    None => return (FieldValue::NotAvailable, bits),
                }
            } else {
                ex_unsigned
            };
            let res = entry.resolution.unwrap_or(1.0);
            FieldValue::Time {
                raw: ex.value,
                seconds: ex.value as f64 * res,
            }
        }
        Some("DATE") => {
            let Some(ex) = extract_bits(data, bit_offset as usize, bits as usize, false, 0) else {
                return (FieldValue::NotAvailable, bits);
            };
            FieldValue::Date(ex.value as u16)
        }
        // The 130822 source-setting entries (Depth/Speed Source N,
        // Port/Stbd Boat Speed Source, …) carry a device's 8-byte
        // NMEA 2000 NAME; canboat C's fieldPrintKeyValue routes this
        // through the same printer as PGN 60928.
        Some("ISO_NAME") => decode_iso_name(data, bit_offset, bits, db),
        _ => decode_binary(data, bit_offset, bits),
    };
    (val, bits)
}

fn decode_dynamic_number(
    data: &[u8],
    bit_offset: u32,
    bit_length: u32,
    signed: bool,
    entry: &crate::types::LookupFieldTypeValue,
) -> FieldValue {
    let Some(ex) = extract_bits(data, bit_offset as usize, bit_length as usize, signed, 0) else {
        return FieldValue::NotAvailable;
    };
    let raw = ex.value as f64;
    let res = entry.resolution.unwrap_or(1.0);
    if res == 1.0 && entry.unit.is_none() {
        FieldValue::Integer(ex.value)
    } else {
        FieldValue::Number(raw * res)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PgnDatabase;

    fn db() -> &'static PgnDatabase {
        PgnDatabase::embedded(crate::Units::Metric)
    }

    #[test]
    fn string_lau_ff_encoding_is_unset_field() {
        // An unset STRING_LAU: length byte present, but the encoding byte
        // and all content are 0xff filler (H5000 pilot, PGN 126998).
        // canboat's C analyzer used to abort the whole PGN with
        // "Unhandled string type 255"; here it must decode as an empty
        // (NotAvailable) field, not a run of U+FFFD replacement chars.
        let data = [0x05u8, 0xff, 0xff, 0xff, 0xff]; // len=5, enc=0xff, body=3×0xff
        let (value, bits) = decode_string_lau(&data, 0);
        assert!(matches!(value, FieldValue::NotAvailable), "{value:?}");
        assert_eq!(bits, 40);
    }

    #[test]
    fn string_lau_ascii_still_decodes() {
        // 1 = ASCII, "Hi" with a trailing 0xff pad byte → "Hi".
        let data = [0x05u8, 0x01, b'H', b'i', 0xff];
        let (value, bits) = decode_string_lau(&data, 0);
        assert!(
            matches!(value, FieldValue::String(ref s) if s == "Hi"),
            "{value:?}"
        );
        assert_eq!(bits, 40);
    }

    #[test]
    fn decodes_iso_address_claim() {
        // From canboat/analyzer/tests/pgn-test.in:
        //   2022-09-10T12:10:16.614Z,6,60928,5,255,8,fb,9b,70,22,00,9b,50,c0
        // Expected (per pgn-test.out): Unique Number=1088507,
        // Manufacturer Code=Navico (275), Device Function=Rudder (155).
        let data = smallvec::smallvec![0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0];
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 60928,
            src: 5,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        assert_eq!(dec.id, "isoAddressClaim");
        assert_eq!(dec.fields.len(), 10);

        // Field 1: Unique Number (21-bit unsigned) = 1088507.
        match dec.fields[0].value {
            FieldValue::Integer(v) => assert_eq!(v, 1_088_507),
            ref other => panic!("expected integer, got {other:?}"),
        }
        // Field 2: Manufacturer Code (11-bit LOOKUP) = 275 / Navico.
        match &dec.fields[1].value {
            FieldValue::Lookup { value, name } => {
                assert_eq!(*value, 275);
                assert_eq!(name.as_deref(), Some("Navico"));
            }
            other => panic!("expected lookup, got {other:?}"),
        }
        // Field 5: Device Function (8-bit LOOKUP→INDIRECT) = 155 / Rudder.
        match &dec.fields[4].value {
            FieldValue::Lookup { value, .. } => assert_eq!(*value, 155),
            other => panic!("expected lookup, got {other:?}"),
        }

        // iso_name() re-packs the fields to the exact wire NAME, i.e.
        // the little-endian payload — the key the per-NAME 0183 filter
        // uses. Reproducing it from fields (not bytes) is what lets the
        // JSON-in daemon derive the same NAME.
        let wire_name = u64::from_le_bytes([0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0]);
        assert_eq!(dec.iso_name(), Some(wire_name));
    }

    #[test]
    fn iso_name_is_none_for_non_claim_pgn() {
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 127251,
            src: 5,
            dst: 255,
            data: smallvec::smallvec![0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        };
        let dec = db().decode(&frame).expect("decode");
        assert_eq!(dec.iso_name(), None);
    }

    #[test]
    fn unknown_pgn_below_any_fallback_returns_error() {
        // PGN 1 is below the lowest Fallback (59392) so no catchall
        // applies — decode must report UnknownPgn.
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 1,
            src: 1,
            dst: 255,
            data: smallvec::smallvec![0u8; 8],
        };
        assert!(matches!(
            db().decode(&frame),
            Err(DecodeError::UnknownPgn { pgn: 1 })
        ));
    }

    #[test]
    fn string_fix_is_capped_to_the_message_not_dropped() {
        // Navico's 130821 ASCII Data declares a 230-byte `Message` and
        // sends however much text it has, so the declared width is a
        // maximum. canboat caps to what arrived
        // (`len = CB_MIN(len, dataLen)`, fieldPrintStringFix); dropping
        // the field instead loses the whole payload.
        let mut data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![0x13, 0x99, 0x00];
        data.extend_from_slice(b"HELLO");
        let frame = RawFrame {
            timestamp: None,
            prio: 7,
            pgn: 130821,
            src: 19,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        let msg = dec
            .fields
            .iter()
            .find(|f| f.info.name == "Message")
            .expect("a Message field");
        match &msg.value {
            FieldValue::String(s) => assert_eq!(s, "HELLO"),
            other => panic!("expected the truncated text, got {other:?}"),
        }
    }

    #[test]
    fn string_padding_covers_all_of_isspace() {
        // canboat's `printString` rtrims with `isspace()`, not just a
        // space. Navico's 130847 ASCII Identifier ends its text with a
        // newline, which was being kept.
        assert_eq!(trim_string_padding("107473373\n"), "107473373");
        assert_eq!(trim_string_padding("abc\r\n"), "abc");
        assert_eq!(trim_string_padding("abc\t \u{0}@\u{ff}"), "abc");
        // Interior whitespace is untouched.
        assert_eq!(trim_string_padding("a b"), "a b");
    }

    #[test]
    fn bitlookup_keeps_set_bits_the_enumeration_does_not_name() {
        // PGN 65305 Simnet: Pilot Mode, from
        // samples/ac42-commissioning.raw. Its `Mode` bitmap reads 0x0a
        // — bits 1 and 3. SIMNET_AP_STATUS names bit 3 ("Standby") but
        // has no entry for bit 1, and canboat still emits it as
        // `{"value":2,"name":null}`.
        //
        // Walking the enumeration's entries instead of the bit
        // positions makes an unnamed set bit invisible, which lost a
        // real bit from every autopilot status record on the bus.
        let data: smallvec::SmallVec<[u8; 8]> =
            smallvec::smallvec![0x41, 0x9f, 0x00, 0x0a, 0x0a, 0x00, 0x80, 0x00];
        let frame = RawFrame {
            timestamp: None,
            prio: 7,
            pgn: 65305,
            src: 13,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        let mode = dec
            .fields
            .iter()
            .find(|f| f.info.name == "Mode")
            .expect("a Mode field");
        match &mode.value {
            FieldValue::BitField { bits, .. } => {
                assert_eq!(bits.as_slice(), &[(2, None), (8, Some("Standby"))]);
            }
            other => panic!("expected a BitField, got {other:?}"),
        }

        // …and it reaches JSON as a null-named entry, not as a gap.
        let mut json = String::new();
        crate::output::write_json(
            &mut json,
            &dec,
            &crate::output::JsonOptions {
                name_value: true,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(
            json.contains(r#""Mode":[{"value":2,"name":null},{"value":8,"name":"Standby"}]"#),
            "got: {json}"
        );
    }

    #[test]
    fn reserved_bytes_keep_their_bit_position() {
        // PGN 130824's field 2 is a 2-bit Reserved at bit 11, i.e. bits
        // 3..4 of byte 1. canboat prints it through `fieldPrintBinary`,
        // which masks the first byte and shifts it back where it was,
        // so a value of 0b10 renders `10` — not `02`.
        let data: smallvec::SmallVec<[u8; 8]> =
            smallvec::smallvec![0x7d, 0x91, 0x64, 0x20, 0x2d, 0x00];
        let frame = RawFrame {
            timestamp: None,
            prio: 3,
            pgn: 130824,
            src: 26,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        let r = dec
            .fields
            .iter()
            .find(|f| f.info.name == "Reserved")
            .expect("a Reserved field");
        match &r.value {
            FieldValue::Reserved { bytes, .. } => assert_eq!(bytes.as_slice(), &[0x10]),
            other => panic!("expected Reserved bytes, got {other:?}"),
        }
    }

    #[test]
    fn all_ones_mmsi_is_suppressed() {
        // A real PGN 130842 Part B frame
        // (samples/merrimac-actisense-serial-2011.raw) whose
        // `Mothership User ID` is 0xFFFFFFFF — this Class B unit has no
        // mothership.
        //
        // The MMSI FieldType declares `Sentinels: None`, so no
        // per-field hint catches it. The field's RangeMax is 999999999
        // though, and the C reserves the top three values on that basis
        // alone, so canboat omits it rather than printing 4294967295.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![
            0x41, 0x9f, 0x81, 0xff, 0x18, 0x88, 0x87, 0x94, 0x0e, 0x24, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x50, 0x49, 0x36, 0x34, 0x34, 0x36, 0x00, 0x6e, 0x00, 0x28, 0x00,
            0x1e, 0x00, 0x3c, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc0,
        ];
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 130842,
            src: 8,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");

        let mothership = dec
            .fields
            .iter()
            .find(|f| f.info.name == "Mothership User ID")
            .expect("a Mothership User ID field");
        assert!(
            mothership.value.is_sentinel(),
            "an all-ones MMSI must read as a sentinel, got {:?}",
            mothership.value
        );

        // The ordinary MMSI in the same frame still decodes.
        let user_id = dec
            .fields
            .iter()
            .find(|f| f.info.name == "User ID")
            .expect("a User ID field");
        assert!(matches!(user_id.value, FieldValue::Mmsi(244_615_048)));
    }

    #[test]
    fn truncated_trailing_reserved_is_omitted() {
        // A 26-byte AIS Class B position report (PGN 129039 declares
        // 27). Its trailing 15-bit `Reserved` starts at bit 201, so
        // only 7 bits survive — and those read all-ones, which canboat
        // skips. The all-ones test has to be made against the width
        // actually read: 0x7F is not 0x7FFF.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![
            0x12, 0x10, 0xad, 0x8c, 0x0e, 0xb2, 0x33, 0x2c, 0x03, 0xe6, 0x96, 0x6d, 0x1f, 0x3e,
            0xb8, 0x65, 0x71, 0x00, 0xc0, 0x00, 0xc1, 0xff, 0x7f, 0x00, 0xe2, 0xff,
        ];
        let frame = RawFrame {
            timestamp: None,
            prio: 4,
            pgn: 129039,
            src: 8,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        let mut json = String::new();
        crate::output::write_json(&mut json, &dec, &crate::output::JsonOptions::default()).unwrap();
        assert!(!json.contains("Reserved"), "got: {json}");
    }

    #[test]
    fn unresolved_key_takes_the_rest_of_the_frame_as_its_value() {
        // PGN 130845 Simnet: Key Value, from
        // samples/ac42-commissioning.raw. Key 17671 is not in
        // SIMNET_KEY_VALUE, so no field type resolves — and this PGN
        // has no DYNAMIC_FIELD_LENGTH either, so nothing declares a
        // width. canboat falls back to the rest of the frame and prints
        // the bytes; the alternative is `""` over data that is plainly
        // there.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![
            0x41, 0x9f, 0xff, 0xff, 0x01, 0xff, 0x07, 0x45, 0x00, 0x01, 0x2d, 0x7d, 0x10, 0x14,
        ];
        let frame = RawFrame {
            timestamp: None,
            prio: 2,
            pgn: 130845,
            src: 13,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        let value = dec
            .fields
            .iter()
            .find(|f| f.info.name == "Value")
            .expect("a Value field");
        match &value.value {
            FieldValue::Binary(b) => assert_eq!(b.as_slice(), &[0x2d, 0x7d, 0x10, 0x14]),
            other => panic!("expected the trailing bytes, got {other:?}"),
        }
    }

    #[test]
    fn all_ones_dynamic_value_is_a_value_not_a_sentinel() {
        // A B&G key-value frame (samples/pgn130824.raw line 2) whose
        // `Trip Time` and `Trip 2 Time` both read 0xFFFFFFFF, the trips
        // never having been started.
        //
        // canboat prints those as `1193:02:47.295`. Its dynamic values
        // are decoded against a pseudo-field built from a fieldtype
        // lookup entry, which never gets a `reservedCount` — that pass
        // walks `pgnList`'s fields — so no top-of-range value is ever
        // stripped on this path. Treating all-ones as "not available"
        // here silently drops the field instead.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![
            0x7d, 0x99, 0x64, 0x20, 0x2d, 0x00, 0x0d, 0x21, 0x43, 0x00, 0x0e, 0x41, 0xc0, 0xa1,
            0xb2, 0x1f, 0x0f, 0x41, 0xce, 0x50, 0x3c, 0x03, 0x69, 0x20, 0x43, 0x7d, 0xd3, 0x20,
            0x2a, 0x74, 0x81, 0x40, 0x70, 0xd3, 0x02, 0x00, 0x9a, 0x20, 0x35, 0xf0, 0x82, 0x20,
            0x00, 0x00, 0x0a, 0x21, 0x98, 0x02, 0x0c, 0x21, 0xb8, 0x02, 0x75, 0x40, 0x20, 0x6c,
            0xfb, 0xff, 0x09, 0x41, 0xff, 0xff, 0xff, 0xff, 0x0b, 0x41, 0xff, 0xff, 0xff, 0xff,
            0xd0, 0x40, 0xe0, 0xaa, 0x8c, 0x0e, 0x7f, 0x20, 0x00, 0x00, 0x9d, 0x20, 0x54, 0xad,
        ];
        let frame = RawFrame {
            timestamp: None,
            prio: 3,
            pgn: 130824,
            src: 26,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");

        // Each `Key` of 265 / 267 must be followed by a Value carrying
        // the raw 0xFFFFFFFF, not by nothing at all.
        let mut expecting_value_for: Option<u64> = None;
        let mut seen = 0;
        for f in &dec.fields {
            match (f.info.name, &f.value) {
                ("Key", FieldValue::Lookup { value, .. }) => {
                    assert!(
                        expecting_value_for.is_none(),
                        "key {} got no Value at all",
                        expecting_value_for.unwrap()
                    );
                    if matches!(value, 265 | 267) {
                        expecting_value_for = Some(*value);
                    }
                }
                ("Value", v) => {
                    if let Some(key) = expecting_value_for.take() {
                        match v {
                            FieldValue::Time { raw, .. } => {
                                assert_eq!(*raw, 0xFFFF_FFFF, "key {key}");
                                seen += 1;
                            }
                            other => panic!("key {key}: expected a Time, got {other:?}"),
                        }
                    }
                }
                _ => {}
            }
        }
        assert!(expecting_value_for.is_none(), "trailing key had no Value");
        assert_eq!(seen, 2, "expected both trip-time records");
    }

    #[test]
    fn dynamic_length_overhead_sizes_130823_records() {
        // A reassembled PGN 130823 directory report (the first record
        // of samples/navico-130823.raw). Each entry is
        // [Length][Type][Data Type:16][Value…] and `Length` counts the
        // three header bytes as well, so the value is `Length - 3`.
        // Without the overhead the walk reads three bytes too many per
        // record and every entry after the first is garbage.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![
            0x13, 0x99, 0xff, 0x00, 0x05, 0x01, 0x00, 0x0a, 0x00, 0x55, 0x02, 0xbc, 0x2f, 0x00,
            0x96, 0x50, 0xc0, 0xff, 0x04, 0x02, 0x26, 0x00, 0x00, 0x04, 0x02, 0x27, 0x00, 0x00,
            0x04, 0x02, 0x7d, 0x00, 0x00, 0x04, 0x02, 0xa5, 0x00, 0x00, 0x04, 0x02, 0x99, 0x01,
            0x00, 0x04, 0x02, 0xf9, 0x01, 0x00,
        ];
        let frame = RawFrame {
            timestamp: None,
            prio: 3,
            pgn: 130823,
            src: 19,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");

        // Walk the repeating set and check every value against the
        // length that introduced it.
        let mut pending: Option<i64> = None;
        let mut records = 0;
        for f in &dec.fields {
            match f.info.name {
                "Length" => {
                    if let FieldValue::Integer(n) = f.value {
                        // A record that got its Value dropped for being
                        // empty must have declared exactly the header.
                        if let Some(prev) = pending.take() {
                            assert_eq!(prev, 3, "record with no Value declared Length {prev}");
                        }
                        pending = Some(n);
                        records += 1;
                    }
                }
                "Value" => {
                    let len = pending.take().expect("Value without a preceding Length");
                    let bytes = match &f.value {
                        FieldValue::Binary(b) => b.len() as i64,
                        other => panic!("expected binary Value, got {other:?}"),
                    };
                    assert_eq!(bytes, len - 3, "Length {len} should give {} bytes", len - 3);
                }
                _ => {}
            }
        }
        if let Some(last) = pending {
            assert_eq!(
                last, 3,
                "trailing record with no Value declared Length {last}"
            );
        }
        assert!(records >= 6, "expected several records, got {records}");
    }

    #[test]
    fn parameters_resolve_a_proprietary_variant() {
        // A 126208 request against PGN 130833 naming Manufacturer =
        // Furuno (1855) and Industry = Marine (4) picks Furuno's
        // definition out of the three, so parameter 4 decodes against a
        // real typed field instead of the catch-all's raw `Data`. This
        // is the half a "several variants → use the catch-all" shortcut
        // gets wrong.
        //   [count][1][mfg lo,hi][3][industry][4]
        let params = [4u8, 0x01, 0x3f, 0x07, 0x03, 0x04, 0x04];
        let picked = match_pgn_by_parameters(db(), 130833, &params).expect("a variant");
        assert_eq!(picked.id, "furunoShipParametersAndAntennaPosition");
    }

    #[test]
    fn manufacturer_alone_lands_on_the_126720_catch_all() {
        // From samples/raymarine-ev1.raw: Manufacturer = Raymarine
        // (1851), Industry = Marine, then parameter 4's value. 126720's
        // catch-all is listed first and carries no `Match`, so it
        // answers immediately — none of the 52 manufacturer variants is
        // ever reachable this way, which is the point: their layouts
        // would be a guess.
        let params = [4u8, 0x01, 0x3b, 0x07, 0x03, 0x04, 0x04, 0x5c, 0x05, 0x0f];
        let picked = match_pgn_by_parameters(db(), 126720, &params).expect("the catch-all");
        assert!(
            picked.fallback.unwrap_or(false),
            "expected the Fallback definition, got id={}",
            picked.id
        );
    }

    #[test]
    fn proprietary_target_without_manufacturer_does_not_resolve() {
        // Without a Manufacturer/Industry pair in the first two
        // parameters there is nothing to narrow a proprietary PGN with,
        // so the caller has to fall back rather than pick a layout.
        let params = [1u8, 0x04, 0x00];
        assert!(match_pgn_by_parameters(db(), 126720, &params).is_none());
    }

    #[test]
    fn match_variant_beats_fallback_listed_first() {
        // PGN 126208 has the FALLBACK variant first in JSON, then 7
        // specific variants distinguished by Function Code (field 1,
        // 8 bits at offset 0). Function Code = 3 must select
        // nmeaReadFieldsGroupFunction, NOT the leading fallback.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![
            3, // Function Code = 3 → Read Fields
            0, 0, 0, 0, 0, 0, 0,
        ];
        let frame = RawFrame {
            timestamp: None,
            prio: 3,
            pgn: 126208,
            src: 0,
            dst: 0,
            data,
        };
        let picked = db().pick_variant(&frame).expect("variant");
        assert_eq!(picked.id, "nmeaReadFieldsGroupFunction");
    }

    #[test]
    fn fallback_when_no_specific_variant_matches() {
        // PGN 126208 with Function Code = 99 (none of the 0..6
        // specific variants match) → return the listed fallback.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![99, 0, 0, 0, 0, 0, 0, 0];
        let frame = RawFrame {
            timestamp: None,
            prio: 3,
            pgn: 126208,
            src: 0,
            dst: 0,
            data,
        };
        let picked = db().pick_variant(&frame).expect("variant");
        assert!(
            picked.fallback.unwrap_or(false),
            "expected Fallback variant, got id={}",
            picked.id
        );
    }

    #[test]
    fn pgn_59392_picks_iso_acknowledgement_over_range_catchall() {
        // Both PGN 59392 variants in canboat.json are no-Match,
        // 8-byte single-frame:
        //   * `0xE800-0xEE00: Standardized single-frame addressed`
        //     — Fallback: true, listed first
        //   * `ISO Acknowledgement` — Fallback: null, listed second
        //
        // The dispatch must pick `isoAcknowledgement` here — the
        // catch-all is only a tie-breaker when no specific variant
        // exists. Real-world bug: NAC3 autopilot ACK records used to
        // surface as the range catch-all.
        //
        // Payload bytes are a real PGN 59392 NAK observed on the bus:
        // control=1 (NAK), reserved, then PGN 126998 (0x01F016) LE.
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 59392,
            src: 17,
            dst: 200,
            data: smallvec::smallvec![0x01, 0xff, 0xff, 0xff, 0xff, 0x16, 0xf0, 0x01],
        };
        let picked = db().pick_variant(&frame).expect("variant");
        assert_eq!(picked.id, "isoAcknowledgement", "got id={}", picked.id);
        assert_eq!(picked.fallback, None);
    }

    #[test]
    fn cross_pgn_catchall_for_unknown_proprietary() {
        // PGN 65500 is not defined in canboat.json. The latest
        // Fallback:true PGN with pgn <= 65500 covers it — that's
        // 65280 / "0xff000xffffManufacturerProprietarySingleFrameNonAddressed".
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 65500,
            src: 0,
            dst: 255,
            data: smallvec::smallvec![0u8; 8],
        };
        let picked = db().pick_variant(&frame).expect("catchall");
        assert!(picked.fallback.unwrap_or(false));
        assert_eq!(picked.pgn, 65280);
    }

    #[test]
    fn not_available_sentinel_on_all_ones() {
        // PGN 127245 Rudder: first field is Instance (8 bits). 0xFF is
        // the unavailable sentinel.
        let data: smallvec::SmallVec<[u8; 8]> = smallvec::smallvec![0xff; 8];
        let frame = RawFrame {
            timestamp: None,
            prio: 2,
            pgn: 127245,
            src: 0,
            dst: 255,
            data,
        };
        let dec = db().decode(&frame).expect("decode");
        let instance = &dec.fields[0];
        assert!(matches!(instance.value, FieldValue::NotAvailable));
    }
}
