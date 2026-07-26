// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Canboat PGN database — const-friendly type definitions.
//!
//! This crate carries no data of its own. The full database is emitted
//! by `canboat-core`'s build script from `crates/canboat-core/data/canboat.json` (plus the
//! synthetic-PGNs blob) into `&'static` tables that use these types.
//!
//! Everything here is `Copy` or a slice of `Copy`/struct types, so the
//! tables compose into a single `static PgnDatabase`. No runtime parse,
//! no allocator, no `Arc`.

#![no_std]

/// How a PGN is transmitted on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketType {
    Single,
    Fast,
    Iso,
    Mixed,
}

/// All field types found in `canboat.json`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldType {
    Number,
    Decimal,
    Float,
    Binary,
    Lookup,
    BitLookup,
    IndirectLookup,
    Date,
    Time,
    Duration,
    StringFix,
    StringLz,
    StringLau,
    Variable,
    Mmsi,
    Pgn,
    IsoName,
    Reserved,
    Spare,
    DynamicFieldKey,
    DynamicFieldLength,
    DynamicFieldValue,
    FieldIndex,
}

/// A single field within a PGN definition.
#[derive(Debug, Clone, Copy)]
pub struct FieldInfo {
    pub order: u8,
    pub id: &'static str,
    pub name: &'static str,
    pub description: Option<&'static str>,
    pub bit_length: Option<u32>,
    pub bit_length_field: Option<&'static str>,
    pub bit_length_variable: Option<bool>,
    pub bit_offset: Option<u32>,
    pub bit_start: Option<u32>,
    pub resolution: Option<f64>,
    pub signed: Option<bool>,
    pub offset: Option<i64>,
    pub range_min: Option<f64>,
    pub range_max: Option<f64>,
    /// Schema 2.4.0 per-field "data not available" sentinel.
    pub unknown_value: Option<u64>,
    pub out_of_range_value: Option<u64>,
    pub reserved_value: Option<u64>,
    pub unit: Option<&'static str>,
    pub physical_quantity: Option<&'static str>,
    pub field_type: Option<FieldType>,
    pub lookup_enumeration: Option<&'static str>,
    pub lookup_bit_enumeration: Option<&'static str>,
    pub lookup_indirect_enumeration: Option<&'static str>,
    pub lookup_indirect_enumeration_field_order: Option<u8>,
    pub lookup_field_type_enumeration: Option<&'static str>,
    /// PGN-variant disambiguation: variants of the same PGN number are
    /// distinguished by required raw field values.
    pub match_value: Option<i64>,
    pub part_of_primary_key: Option<bool>,
    pub condition: Option<&'static str>,
    /// Additive offset applied at decode time AFTER multiplying by
    /// `resolution`. Computed at codegen by the non-SI unit fix-up
    /// (e.g. K → °C subtracts 273.15). `0.0` when no offset applies.
    pub unit_offset: f64,
    /// Explicit decimal precision override. `0` means
    /// "compute from resolution" (canboat default).
    pub precision: u8,
    /// `true` iff [`Self::name`] is exactly `"Length"`. Precomputed so
    /// the decoder's dynamic-length dispatch doesn't `memcmp` every
    /// field.
    pub is_dynamic_length_marker: bool,
}

/// A PGN definition.
#[derive(Debug, Clone, Copy)]
pub struct PgnInfo {
    pub pgn: u32,
    pub id: &'static str,
    pub description: &'static str,
    pub explanation: Option<&'static str>,
    pub url: Option<&'static str>,
    pub packet_type: PacketType,
    pub complete: Option<bool>,
    pub fallback: Option<bool>,
    pub missing: &'static [&'static str],
    pub priority: Option<u8>,
    pub transmission_interval: Option<u32>,
    pub transmission_irregular: Option<bool>,
    pub field_count: u32,
    pub length: Option<u32>,
    pub min_length: Option<u32>,
    pub fields: &'static [FieldInfo],
    pub repeating_field_set1_size: Option<u32>,
    pub repeating_field_set1_start_field: Option<u32>,
    pub repeating_field_set1_count_field: Option<u32>,
    pub repeating_field_set2_size: Option<u32>,
    pub repeating_field_set2_start_field: Option<u32>,
    pub repeating_field_set2_count_field: Option<u32>,
    /// `true` when the canboat C source pinned the PGN's
    /// `Id` explicitly (i.e. it doesn't match the natural camelCase
    /// of [`Self::description`]). canboat 7.1.0's JSON formatter
    /// wraps such PGNs in `{"<id>":{...}}` so downstream consumers
    /// can rely on a stable id even after a Description rename. Only
    /// PGN 130846 carries this today; the contract is documented in
    /// canboat's `analyzer/pgn.c::camelCase()`.
    pub id_is_pinned: bool,
}

/// A compile-time reference to one field within a specific PGN
/// definition: the field descriptor plus the PGN it lives in.
///
/// `canboat-core`'s build script generates one `static` of this type per
/// (PGN, field) under `canboat_core::field::<pgn_id>::<FIELD_ID>`, so
/// call sites use a compile-checked symbol
/// (`field::wind_data::WIND_ANGLE`) instead of a `("windData",
/// "windAngle")` string pair — a rename in `canboat.json` then breaks the
/// build instead of silently returning `None`. Because it carries its
/// PGN, one value is enough to both resolve an `O(1)` handle
/// (`PgnDatabase::handle`) and read the field's `id`/`name` for JSON key
/// selection. It points at the SI schema arrays; `id`/`name`/`order` are
/// unit-invariant, so the same `FieldRef` is valid against either
/// `Units`.
#[derive(Debug, Clone, Copy)]
pub struct FieldRef {
    pub pgn: &'static PgnInfo,
    pub field: &'static FieldInfo,
}

/// One (value, name) entry inside a LOOKUP enumeration.
#[derive(Debug, Clone, Copy)]
pub struct LookupValue {
    pub value: u64,
    pub name: &'static str,
    pub id: Option<&'static str>,
}

/// A named LOOKUP enumeration (plain value → name).
///
/// Values keep their canboat.json declaration order so any iterator
/// reading from `values` is order-stable. The companion `by_value`
/// slice is sorted by raw value for O(log n) lookup via
/// [`Self::get`].
pub struct LookupTable {
    pub name: &'static str,
    pub max_value: Option<u64>,
    pub values: &'static [LookupValue],
    /// `(value, index_into_values)`, sorted by `value`.
    pub by_value: &'static [(u64, u32)],
}

impl LookupTable {
    /// O(log n) lookup of a single enum value.
    pub fn get(&self, value: u64) -> Option<&'static LookupValue> {
        match self.by_value.binary_search_by_key(&value, |&(v, _)| v) {
            Ok(i) => Some(&self.values[self.by_value[i].1 as usize]),
            Err(_) => None,
        }
    }
}

/// One (bit, name) entry inside a BITLOOKUP enumeration.
#[derive(Debug, Clone, Copy)]
pub struct BitLookupValue {
    pub bit: u8,
    pub name: &'static str,
    pub id: Option<&'static str>,
}

/// A BITLOOKUP enumeration.
pub struct BitLookupTable {
    pub name: &'static str,
    pub max_value: Option<u64>,
    pub values: &'static [BitLookupValue],
    pub by_bit: &'static [(u8, u32)],
}

impl BitLookupTable {
    pub fn get(&self, bit: u8) -> Option<&'static BitLookupValue> {
        match self.by_bit.binary_search_by_key(&bit, |&(b, _)| b) {
            Ok(i) => Some(&self.values[self.by_bit[i].1 as usize]),
            Err(_) => None,
        }
    }
}

/// One (Value1, Value2) → Name mapping inside an INDIRECT_LOOKUP table.
#[derive(Debug, Clone, Copy)]
pub struct IndirectLookupValue {
    pub value1: u64,
    pub value2: u64,
    pub name: &'static str,
    pub id: Option<&'static str>,
}

/// A two-key lookup (INDIRECT_LOOKUP).
pub struct IndirectLookupTable {
    pub name: &'static str,
    pub max_value: Option<u64>,
    pub values: &'static [IndirectLookupValue],
    /// `((value1, value2), index_into_values)`, sorted lexicographically.
    pub by_pair: &'static [((u64, u64), u32)],
}

impl IndirectLookupTable {
    pub fn get(&self, value1: u64, value2: u64) -> Option<&'static IndirectLookupValue> {
        match self
            .by_pair
            .binary_search_by_key(&(value1, value2), |&(k, _)| k)
        {
            Ok(i) => Some(&self.values[self.by_pair[i].1 as usize]),
            Err(_) => None,
        }
    }
}

/// One key→field-type entry inside a [`LookupFieldTypeTable`].
#[derive(Debug, Clone, Copy)]
pub struct LookupFieldTypeValue {
    pub value: u64,
    pub name: &'static str,
    pub field_type: Option<&'static str>,
    /// Field width in bits.
    pub bits: Option<u32>,
    pub resolution: Option<f64>,
    pub unit: Option<&'static str>,
    pub lookup_enumeration: Option<&'static str>,
    pub lookup_bit_enumeration: Option<&'static str>,
    /// Codegen-applied: signed iff unit is rad/rad-per-s/deg/deg-per-s
    /// (canboat's underlying ANGLE_FIX type).
    pub signed: bool,
    /// Codegen-applied display precision (0 = derive from resolution).
    pub precision: u8,
}

impl LookupFieldTypeValue {
    pub fn bit_length(&self) -> Option<u32> {
        self.bits
    }
}

/// A LookupFieldTypeEnumeration — value → field type + metadata. Used
/// by DYNAMIC_FIELD_KEY / DYNAMIC_FIELD_VALUE pairs to resolve a
/// heterogeneous value field at decode time.
pub struct LookupFieldTypeTable {
    pub name: &'static str,
    pub max_value: Option<u64>,
    pub values: &'static [LookupFieldTypeValue],
    pub by_value: &'static [(u64, u32)],
}

impl LookupFieldTypeTable {
    pub fn get(&self, value: u64) -> Option<&'static LookupFieldTypeValue> {
        match self.by_value.binary_search_by_key(&value, |&(v, _)| v) {
            Ok(i) => Some(&self.values[self.by_value[i].1 as usize]),
            Err(_) => None,
        }
    }
}
