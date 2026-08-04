// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Encode a PGN from field values into a wire [`RawFrame`] — the
//! inverse of [`crate::decode`].
//!
//! The entry point is [`PgnDatabase::encode`] /
//! [`PgnDatabase::encode_by_pgn`] (by id / by number), or
//! [`PgnDatabase::encode_for`] with a generated `pgn::…` constant, which
//! hand back a [`PgnBuilder`] for a specific schema PGN variant. Set
//! field values by a compile-checked `field::…` constant
//! ([`push`](PgnBuilder::push) — the O(1) default) or by schema name
//! ([`push_by_name`](PgnBuilder::push_by_name)), mirroring the decode
//! side's [`DecodedPgn::field`] / [`DecodedPgn::field_by_name`];
//! then [`build`](PgnBuilder::build) packs them at their schema bit
//! offsets — LSB-first, exactly the layout [`crate::bits::extract_bits`]
//! reads back — and returns a [`RawFrame`]. Because the result is a
//! `RawFrame`, any [`crate::format`] writer (PLAIN, YDWG02, Actisense,
//! `.ebl`) turns it into wire text/bytes.
//!
//! Fields the caller doesn't set are filled automatically: a field with
//! a `match_value` (the raw value that selects this PGN variant) gets
//! that value, so proprietary/variant PGNs come out valid without the
//! caller restating their manufacturer/industry/selector fields; a
//! `Spare` field gets zero; everything else gets its "not available"
//! sentinel (the schema `unknown_value`, else all-ones).
//!
//! Scope: every self-contained field type encodes — the numeric,
//! enum/lookup, PGN/MMSI, `FLOAT`, `ISO_NAME`, `BINARY` and all four
//! string families (`STRING_FIX`/`LZ`/`LAU`). The key/value family
//! ([`DynamicFieldKey`](FieldType::DynamicFieldKey) /
//! [`DynamicFieldValue`](FieldType::DynamicFieldValue), e.g. PGN 130845
//! `simnetKeyValue`) encodes when set explicitly: the KEY is a
//! fixed-width scalar (set it as an `Int`), and the VALUE takes its bytes
//! verbatim (set it as `Bytes` — the caller owns matching the width the
//! resolved key expects). Left unset, the VALUE and the PGN 126208
//! group-function types (`VARIABLE`, `DYNAMIC_FIELD_LENGTH`) can't be
//! defaulted in isolation and return [`EncodeError::NotFixedLength`].

use std::error::Error;
use std::fmt;

use canboat_schema::{FieldInfo, FieldRef, FieldType, PgnInfo};

use crate::db::PgnDatabase;
use crate::frame::RawFrame;

/// A value to place into a PGN field when encoding. Owned and
/// caller-constructible (unlike the decode [`crate::FieldValue`], which
/// borrows `&'static` schema references).
#[derive(Debug, Clone, PartialEq)]
pub enum EncodeValue {
    /// A scaled physical value in the field's unit (e.g. `1.23` rad/s).
    /// Inverse of the decoder's `raw * resolution + offset + unit_offset`.
    Number(f64),
    /// A raw, unscaled integer written to the field bits verbatim
    /// (instances, counts, enum values, a PGN number, …). Signed values
    /// are two's-complemented into the field width.
    Int(i64),
    /// A `LOOKUP` / `BITLOOKUP` selected by its label (e.g. `"Apparent"`).
    Lookup(String),
    /// Text for a `STRING_FIX` / `STRING_LZ` / `STRING_LAU` field.
    Text(String),
    /// Raw bytes for a `BINARY` field.
    Bytes(Vec<u8>),
    /// A PGN number for a `PGN`-type field (packed as its bit-width,
    /// LSB-first → little-endian bytes).
    Pgn(u32),
    /// Leave the field at its "not available" sentinel.
    NotAvailable,
}

impl From<f64> for EncodeValue {
    fn from(v: f64) -> Self {
        EncodeValue::Number(v)
    }
}
impl From<i64> for EncodeValue {
    fn from(v: i64) -> Self {
        EncodeValue::Int(v)
    }
}
impl From<i32> for EncodeValue {
    fn from(v: i32) -> Self {
        EncodeValue::Int(v as i64)
    }
}
impl From<u32> for EncodeValue {
    fn from(v: u32) -> Self {
        EncodeValue::Int(v as i64)
    }
}
impl From<&str> for EncodeValue {
    fn from(v: &str) -> Self {
        EncodeValue::Text(v.to_string())
    }
}
impl From<String> for EncodeValue {
    fn from(v: String) -> Self {
        EncodeValue::Text(v)
    }
}
impl From<u64> for EncodeValue {
    /// A raw 64-bit pattern (e.g. an ISO NAME). Reinterpreted as `Int`;
    /// the field width masks it back, so the full `u64` range survives.
    fn from(v: u64) -> Self {
        EncodeValue::Int(v as i64)
    }
}
impl From<Vec<u8>> for EncodeValue {
    fn from(v: Vec<u8>) -> Self {
        EncodeValue::Bytes(v)
    }
}
impl From<&[u8]> for EncodeValue {
    fn from(v: &[u8]) -> Self {
        EncodeValue::Bytes(v.to_vec())
    }
}

/// Why a message could not be encoded.
#[derive(Debug, Clone, PartialEq)]
pub enum EncodeError {
    /// No PGN with this schema id (e.g. `"isoRequest"`).
    NoSuchPgnId(String),
    /// No PGN with this number in the schema.
    NoSuchPgn(u32),
    /// This PGN number has several schema variants; use
    /// [`PgnDatabase::encode`] with the specific id.
    AmbiguousPgn { pgn: u32, variants: usize },
    /// No field with this name/id in the PGN.
    NoSuchField { pgn_id: &'static str, field: String },
    /// A `LOOKUP` field was given a label that isn't in its table.
    UnknownLookupLabel { field: &'static str, label: String },
    /// A value doesn't fit the field's bit width / range.
    ValueOutOfRange { field: &'static str, value: f64 },
    /// A group-function *dynamic* field (`VARIABLE` /
    /// `DYNAMIC_FIELD_KEY`/`LENGTH`/`VALUE`) was left unset — its width is
    /// determined by sibling fields, so it can't be defaulted in isolation.
    NotFixedLength(&'static str),
    /// The field type can't accept the [`EncodeValue`] variant given.
    TypeMismatch {
        field: &'static str,
        expected: &'static str,
    },
    /// The packed byte length didn't match the PGN's declared length —
    /// an internal consistency failure (repeating/variable PGN, or a
    /// schema/encoder mismatch).
    LengthMismatch { pgn: u32, expected: u32, got: usize },
    /// A group-function `VARIABLE` value could not be resolved against
    /// the referenced PGN's field (missing/unknown target PGN or
    /// FIELD_INDEX).
    VariableUnresolved {
        field: &'static str,
        reason: &'static str,
    },
    /// [`PgnBuilder::add_set_instance`] / [`PgnBuilder::push_in_set`]
    /// was called for a repeating set this PGN does not declare.
    NoRepeatingSet { pgn_id: &'static str, set: usize },
    /// [`PgnBuilder::push_in_set`] addressed an instance index that was
    /// never created with [`PgnBuilder::add_set_instance`].
    NoSuchInstance {
        pgn_id: &'static str,
        set: usize,
        instance: usize,
    },
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            EncodeError::NoSuchPgnId(id) => write!(f, "no PGN with id '{id}'"),
            EncodeError::NoSuchPgn(p) => write!(f, "no PGN {p} in the schema"),
            EncodeError::AmbiguousPgn { pgn, variants } => {
                write!(f, "PGN {pgn} has {variants} variants; select one by id")
            }
            EncodeError::NoSuchField { pgn_id, field } => {
                write!(f, "PGN '{pgn_id}' has no field '{field}'")
            }
            EncodeError::UnknownLookupLabel { field, label } => {
                write!(f, "field '{field}': '{label}' is not a valid value")
            }
            EncodeError::ValueOutOfRange { field, value } => {
                write!(f, "field '{field}': value {value} out of range")
            }
            EncodeError::NotFixedLength(field) => {
                write!(
                    f,
                    "field '{field}': dynamic group-function field must be set explicitly"
                )
            }
            EncodeError::VariableUnresolved { field, reason } => {
                write!(f, "field '{field}': {reason}")
            }
            EncodeError::NoRepeatingSet { pgn_id, set } => {
                write!(f, "PGN '{pgn_id}' has no repeating field set {set}")
            }
            EncodeError::NoSuchInstance {
                pgn_id,
                set,
                instance,
            } => {
                write!(
                    f,
                    "PGN '{pgn_id}' set {set}: instance {instance} was never added"
                )
            }
            EncodeError::TypeMismatch { field, expected } => {
                write!(f, "field '{field}': expected {expected}")
            }
            EncodeError::LengthMismatch { pgn, expected, got } => {
                write!(f, "PGN {pgn}: packed {got} bytes, schema says {expected}")
            }
        }
    }
}

impl Error for EncodeError {}

/// Fluent builder for one PGN message. Created via
/// [`PgnDatabase::encode`] / [`PgnDatabase::encode_by_pgn`].
pub struct PgnBuilder {
    db: &'static PgnDatabase,
    pgn: &'static PgnInfo,
    prio: u8,
    src: u8,
    dst: u8,
    timestamp: Option<String>,
    /// Staged values per field, indexed by `order - 1`. `None` → fill
    /// with the field default at build time. For fields inside a
    /// repeating set these slots are ignored; instances live in `sets`.
    staged: Vec<Option<Staged>>,
    /// Staged repeating-set instances: `sets[0]` for set 1, `sets[1]`
    /// for set 2. Each instance stages the set's fields by position
    /// within the set (`order - start_field`). Zero instances → the
    /// payload simply ends before the set, mirroring decode.
    sets: [Vec<Vec<Option<Staged>>>; 2],
}

/// A staged field value: either a scalar bit pattern (numbers, lookups,
/// PGN references) or already-packed bytes (a fixed-length string).
#[derive(Clone)]
enum Staged {
    /// Raw bits, LSB-first, of the field's declared width.
    Scalar(u64),
    /// Byte-aligned payload of exactly `bit_length / 8` bytes.
    Bytes(Vec<u8>),
    /// A group-function `VARIABLE` value, staged as-given: its width,
    /// scaling and type come from the *referenced* PGN's field, which
    /// is only known at build time (the record's PGN field plus the
    /// instance's FIELD_INDEX select it). Resolved by
    /// [`PgnBuilder::build`] exactly as the decoder resolves it.
    Deferred(EncodeValue),
}

impl PgnBuilder {
    /// Construct for a resolved PGN variant. Crate-internal; callers go
    /// through [`PgnDatabase::encode`].
    pub(crate) fn for_pgn(db: &'static PgnDatabase, pgn: &'static PgnInfo) -> Self {
        Self {
            db,
            pgn,
            prio: pgn.priority.unwrap_or(6),
            src: 0,
            dst: 255,
            timestamp: None,
            staged: vec![None; pgn.fields.len()],
            sets: [Vec::new(), Vec::new()],
        }
    }

    /// Override the CAN priority (default: the schema priority, else 6).
    pub fn priority(mut self, prio: u8) -> Self {
        self.prio = prio;
        self
    }
    /// Set the source address (default 0).
    pub fn source(mut self, src: u8) -> Self {
        self.src = src;
        self
    }
    /// Set the destination address (default 255 = broadcast).
    pub fn destination(mut self, dst: u8) -> Self {
        self.dst = dst;
        self
    }
    /// Stamp a timestamp onto the produced frame (default: none — the
    /// caller/writer decides).
    pub fn timestamp(mut self, ts: impl Into<String>) -> Self {
        self.timestamp = Some(ts.into());
        self
    }

    /// Set a field by its generated [`FieldRef`] constant
    /// (`canboat_core::field::wind_data::WIND_ANGLE`) — the compile-checked,
    /// `O(1)` path, and the recommended default. The exact mirror of the
    /// decode side's [`DecodedPgn::field`](crate::DecodedPgn::field): index
    /// by the field's schema `order` (one array load), with a debug-only
    /// assert that the constant belongs to this builder's PGN — a `&str`
    /// compare, free in release builds, no scan and no hashing.
    ///
    /// The value scales in the builder's unit system: the constant points at
    /// the SI schema, but a field's `order` is identical across unit schemas,
    /// so the builder's own [`FieldInfo`] (in its units) does the staging.
    pub fn push(
        &mut self,
        field: FieldRef,
        value: impl Into<EncodeValue>,
    ) -> Result<&mut Self, EncodeError> {
        debug_assert_eq!(
            field.pgn.id, self.pgn.id,
            "FieldRef/PGN mismatch: a `{}` field ref was used on a `{}` builder",
            field.pgn.id, self.pgn.id,
        );
        let idx = (field.field.order as usize)
            .checked_sub(1)
            .filter(|&i| i < self.pgn.fields.len())
            .ok_or(EncodeError::NoSuchField {
                pgn_id: self.pgn.id,
                field: field.field.id.to_string(),
            })?;
        let staged = self.stage_value(&self.pgn.fields[idx], value.into())?;
        self.staged[idx] = Some(staged);
        Ok(self)
    }

    /// Set a field by schema **name** (`"Wind Speed"`) — an `O(n)` scan, for
    /// callers without a [`FieldRef`] constant (runtime-chosen or
    /// config-driven fields). The mirror of
    /// [`DecodedPgn::field_by_name`](crate::DecodedPgn::field_by_name).
    /// Prefer [`push`](Self::push) with a generated constant wherever the
    /// field is known at compile time: it is O(1) and a typo fails the build
    /// instead of returning [`EncodeError::NoSuchField`] at runtime.
    pub fn push_by_name(
        &mut self,
        field: &str,
        value: impl Into<EncodeValue>,
    ) -> Result<&mut Self, EncodeError> {
        let idx = self.find_by_name(field)?;
        let staged = self.stage_value(&self.pgn.fields[idx], value.into())?;
        self.staged[idx] = Some(staged);
        Ok(self)
    }

    /// Set a field from a textual value, coercing per the field's type
    /// (numbers, `0x`-hex, lookup labels). For CLI `FIELD=VALUE` args, where
    /// `FIELD` may be the schema name (`"Wind Speed"`) or id (`"windSpeed"`).
    pub fn push_arg(&mut self, field: &str, value: &str) -> Result<&mut Self, EncodeError> {
        let idx = self.find_by_name_or_id(field)?;
        let f = &self.pgn.fields[idx];
        let ev = coerce_arg(f, value)?;
        let staged = self.stage_value(f, ev)?;
        self.staged[idx] = Some(staged);
        Ok(self)
    }

    /// Resolve repeating set `set` (1 or 2) to `(start index, size,
    /// count-field index)`, all zero-based, or `None` if this PGN does
    /// not declare that set.
    fn set_layout(&self, set: usize) -> Option<(usize, usize, Option<usize>)> {
        let (start, size, count) = match set {
            1 => (
                self.pgn.repeating_field_set1_start_field,
                self.pgn.repeating_field_set1_size,
                self.pgn.repeating_field_set1_count_field,
            ),
            2 => (
                self.pgn.repeating_field_set2_start_field,
                self.pgn.repeating_field_set2_size,
                self.pgn.repeating_field_set2_count_field,
            ),
            _ => (None, None, None),
        };
        Some((
            start? as usize - 1,
            size? as usize,
            count.map(|c| c as usize - 1),
        ))
    }

    /// Append a new instance of repeating set `set` (1 or 2) and return
    /// its index for [`push_in_set`](Self::push_in_set). Fields left
    /// unset in an instance are filled with their "not available"
    /// defaults at build time, exactly like top-level fields. The set's
    /// count field (when the schema declares one) is set automatically
    /// from the number of instances unless staged explicitly.
    pub fn add_set_instance(&mut self, set: usize) -> Result<usize, EncodeError> {
        let (_, size, _) = self.set_layout(set).ok_or(EncodeError::NoRepeatingSet {
            pgn_id: self.pgn.id,
            set,
        })?;
        let instances = &mut self.sets[set - 1];
        instances.push(vec![None; size]);
        Ok(instances.len() - 1)
    }

    /// Set a field of repeating-set instance `instance` by schema name
    /// (`"Parameter"`) or id (`"parameter"`). The instance must have
    /// been created with [`add_set_instance`](Self::add_set_instance).
    pub fn push_in_set(
        &mut self,
        set: usize,
        instance: usize,
        field: &str,
        value: impl Into<EncodeValue>,
    ) -> Result<&mut Self, EncodeError> {
        let (start, size, _) = self.set_layout(set).ok_or(EncodeError::NoRepeatingSet {
            pgn_id: self.pgn.id,
            set,
        })?;
        let rel = self.pgn.fields[start..start + size]
            .iter()
            .position(|f| f.name == field || f.id == field)
            .ok_or_else(|| EncodeError::NoSuchField {
                pgn_id: self.pgn.id,
                field: field.to_string(),
            })?;
        let f = &self.pgn.fields[start + rel];
        // A VARIABLE value can't be staged yet — its metadata lives in
        // the referenced PGN's field, selected by the record's PGN
        // field and this instance's FIELD_INDEX. Defer to build().
        let staged = if f.field_type == Some(FieldType::Variable) {
            Staged::Deferred(value.into())
        } else {
            self.stage_value(f, value.into())?
        };
        let inst = self.sets[set - 1]
            .get_mut(instance)
            .ok_or(EncodeError::NoSuchInstance {
                pgn_id: self.pgn.id,
                set,
                instance,
            })?;
        inst[rel] = Some(staged);
        Ok(self)
    }

    /// Build the wire frame: pack every field (staged value or default)
    /// LSB-first at its schema offset, verify the length, and return the
    /// [`RawFrame`]. Repeating sets are emitted at their schema position,
    /// one group per staged instance (zero instances → the payload ends
    /// there, mirroring decode); a declared count field left unset is
    /// filled with the instance count.
    pub fn build(&self) -> Result<RawFrame, EncodeError> {
        let mut buf: Vec<u8> = Vec::new();
        let mut next_bit = 0usize;
        let set1 = self.set_layout(1);
        let set2 = self.set_layout(2);
        let mut idx = 0usize;
        while idx < self.pgn.fields.len() {
            // A repeating set starts at this field? Emit its staged
            // instances and skip past the group's schema fields.
            let mut in_set = false;
            for (layout, instances) in [(&set1, &self.sets[0]), (&set2, &self.sets[1])] {
                if let Some((start, size, _)) = *layout
                    && idx == start
                {
                    for inst in instances {
                        for (j, f) in self.pgn.fields[start..start + size].iter().enumerate() {
                            match &inst[j] {
                                Some(Staged::Deferred(v)) => self.emit_variable(
                                    &mut buf,
                                    &mut next_bit,
                                    f,
                                    inst,
                                    start,
                                    size,
                                    v,
                                )?,
                                other => emit_field(&mut buf, &mut next_bit, f, other.as_ref())?,
                            }
                        }
                    }
                    idx += size;
                    in_set = true;
                    break;
                }
            }
            if in_set {
                continue;
            }
            let f = &self.pgn.fields[idx];
            // A declared count field left unset takes the number of
            // staged instances for its set (0 when none were added — a
            // zero-iteration message, not "unavailable").
            let auto_count = [(&set1, &self.sets[0]), (&set2, &self.sets[1])]
                .into_iter()
                .find_map(|(layout, instances)| match *layout {
                    Some((_, _, Some(count_idx)))
                        if count_idx == idx && self.staged[idx].is_none() =>
                    {
                        Some(Staged::Scalar(instances.len() as u64))
                    }
                    _ => None,
                });
            emit_field(
                &mut buf,
                &mut next_bit,
                f,
                auto_count.as_ref().or(self.staged[idx].as_ref()),
            )?;
            idx += 1;
        }
        // Flush a trailing partial byte (LSB-first packing already grew
        // the buffer as it went, so this is only defensive).
        if !next_bit.is_multiple_of(8) {
            // pad the final byte with 1s (canboat reserved/pad convention)
            let pad = 8 - (next_bit % 8);
            write_bits(&mut buf, &mut next_bit, pad, u64::MAX);
        }
        if let Some(expected) = self.pgn.length
            && buf.len() != expected as usize
        {
            return Err(EncodeError::LengthMismatch {
                pgn: self.pgn.pgn,
                expected,
                got: buf.len(),
            });
        }
        Ok(RawFrame {
            timestamp: self.timestamp.clone(),
            prio: self.prio,
            pgn: self.pgn.pgn,
            src: self.src,
            dst: self.dst,
            data: buf.into_iter().collect(),
        })
    }

    /// The resolved PGN this builder targets (for CLI help / listing).
    pub fn pgn_info(&self) -> &'static PgnInfo {
        self.pgn
    }

    /// Field index by schema name only (for [`push_by_name`](Self::push_by_name)).
    fn find_by_name(&self, name: &str) -> Result<usize, EncodeError> {
        self.pgn
            .fields
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| EncodeError::NoSuchField {
                pgn_id: self.pgn.id,
                field: name.to_string(),
            })
    }

    /// Field index by schema name or id (for CLI [`push_arg`](Self::push_arg)).
    fn find_by_name_or_id(&self, key: &str) -> Result<usize, EncodeError> {
        self.pgn
            .fields
            .iter()
            .position(|f| f.name == key || f.id == key)
            .ok_or_else(|| EncodeError::NoSuchField {
                pgn_id: self.pgn.id,
                field: key.to_string(),
            })
    }

    /// Emit a deferred group-function `VARIABLE` value: resolve the
    /// referenced PGN (the record's staged PGN field) and the target
    /// field (this instance's staged FIELD_INDEX), stage the value with
    /// the target field's metadata — width, resolution, lookup table —
    /// and pack it rounded up to whole bytes, mirroring the decoder's
    /// `fieldPrintVariable` byte alignment.
    #[allow(clippy::too_many_arguments)]
    fn emit_variable(
        &self,
        buf: &mut Vec<u8>,
        next_bit: &mut usize,
        f: &FieldInfo,
        inst: &[Option<Staged>],
        start: usize,
        size: usize,
        v: &EncodeValue,
    ) -> Result<(), EncodeError> {
        let unresolved = |reason| EncodeError::VariableUnresolved {
            field: f.name,
            reason,
        };
        let target_pgn = self
            .pgn
            .fields
            .iter()
            .position(|tf| tf.field_type == Some(FieldType::Pgn))
            .and_then(|i| match self.staged[i] {
                Some(Staged::Scalar(p)) => Some(p as u32),
                _ => None,
            })
            .ok_or(unresolved("no staged PGN field names the referenced PGN"))?;
        // Prefer the specific definition; the range catch-all is a last
        // resort, mirroring decode (which only borrows it when variant
        // matching fails).
        let target_info = self
            .db
            .first_pgn(target_pgn)
            .or_else(|| self.db.fallback_pgn(target_pgn))
            .ok_or(unresolved("referenced PGN is not in the schema"))?;
        let param_idx = self.pgn.fields[start..start + size]
            .iter()
            .enumerate()
            .find(|(_, tf)| tf.field_type == Some(FieldType::FieldIndex))
            .and_then(|(j, _)| match inst[j] {
                Some(Staged::Scalar(n)) => Some(n as u32),
                _ => None,
            })
            .ok_or(unresolved("no staged FIELD_INDEX selects the target field"))?;
        let target_field = target_info
            .fields
            .iter()
            .find(|tf| (tf.order as u32) == param_idx)
            .ok_or(unresolved(
                "FIELD_INDEX is beyond the referenced PGN's fields",
            ))?;
        // A label for a lookup-typed target arrives as Text (the caller
        // cannot know the target's type up front); retype it.
        let value = match v {
            EncodeValue::Text(s) if target_field.lookup_enumeration.is_some() => {
                EncodeValue::Lookup(s.clone())
            }
            other => other.clone(),
        };
        match self.stage_value(target_field, value)? {
            Staged::Scalar(raw) => {
                let bl = target_field
                    .bit_length
                    .ok_or(EncodeError::NotFixedLength(target_field.name))?
                    as usize;
                write_bits(buf, next_bit, bl, raw);
                // The decoder consumes VARIABLE values rounded up to
                // whole bytes; pad with 1s like every other fill.
                if !next_bit.is_multiple_of(8) {
                    let pad = 8 - (*next_bit % 8);
                    write_bits(buf, next_bit, pad, u64::MAX);
                }
            }
            Staged::Bytes(bytes) => {
                for b in bytes {
                    write_bits(buf, next_bit, 8, u64::from(b));
                }
            }
            Staged::Deferred(_) => unreachable!("stage_value never defers"),
        }
        Ok(())
    }

    /// Stage a value for field `f`: strings and binary become byte
    /// payloads (self-describing or fixed-width), everything else a
    /// scalar bit pattern.
    fn stage_value(&self, f: &FieldInfo, v: EncodeValue) -> Result<Staged, EncodeError> {
        match f.field_type {
            Some(FieldType::StringFix) => stage_string_fix(f, v),
            Some(FieldType::StringLz) => stage_string_lz(f, v),
            Some(FieldType::StringLau) => stage_string_lau(f, v),
            Some(FieldType::Binary) => stage_binary(f, v),
            // DYNAMIC_FIELD_VALUE (the value slot of a key/value PGN like
            // 130845) has no schema `BitLength` — its width is set by the
            // sibling KEY at decode time, so it can't be defaulted, but the
            // caller *can* supply the already-encoded value bytes. Written
            // verbatim (variable length); the caller owns matching the
            // width the resolved key expects. The sibling KEY field is a
            // fixed-width scalar, so it takes the normal `Int` path below.
            Some(FieldType::DynamicFieldValue) => stage_dynamic_value(f, v),
            _ => Ok(Staged::Scalar(self.value_to_raw(f, v)?)),
        }
    }

    /// Convert an [`EncodeValue`] to the raw bit pattern for `f`,
    /// masked/two's-complemented into the field width.
    fn value_to_raw(&self, f: &FieldInfo, v: EncodeValue) -> Result<u64, EncodeError> {
        let bl = f.bit_length.ok_or(EncodeError::NotFixedLength(f.name))?;
        let raw_i64: i64 = match (f.field_type, v) {
            // Explicit "leave unset".
            (_, EncodeValue::NotAvailable) => return Ok(default_raw(f)),

            // Lookup by label → its raw value; or a raw integer.
            (
                Some(FieldType::Lookup) | Some(FieldType::IndirectLookup),
                EncodeValue::Lookup(label),
            ) => self.resolve_lookup(f, &label)? as i64,
            (Some(FieldType::Lookup) | Some(FieldType::IndirectLookup), EncodeValue::Int(n)) => n,

            // BitLookup: a raw bitmask integer (label-per-bit is phase 2).
            (Some(FieldType::BitLookup), EncodeValue::Int(n)) => n,

            // PGN-type: the requested PGN number.
            (Some(FieldType::Pgn), EncodeValue::Pgn(p)) => p as i64,
            (Some(FieldType::Pgn), EncodeValue::Int(n)) => n,

            // FLOAT: IEEE-754 32-bit — the raw bit pattern, not a scaled
            // integer. (The only non-integer scalar wire form.)
            (Some(FieldType::Float), EncodeValue::Number(x)) => {
                return Ok(u64::from((x as f32).to_bits()));
            }
            (Some(FieldType::Float), EncodeValue::Int(n)) => {
                return Ok(u64::from((n as f32).to_bits()));
            }

            // Text / binary reach `value_to_raw` only on a scalar field
            // (e.g. a string value handed to a NUMBER) — a type error.
            (_, EncodeValue::Text(_)) | (_, EncodeValue::Bytes(_)) => {
                return Err(EncodeError::TypeMismatch {
                    field: f.name,
                    expected: "a numeric value",
                });
            }

            // A PGN number on any field is just a raw integer.
            (_, EncodeValue::Pgn(p)) => p as i64,

            // Numeric families: a raw integer is written verbatim…
            (_, EncodeValue::Int(n)) => n,
            // …a scaled Number is inverted through resolution/offset.
            (_, EncodeValue::Number(x)) => scaled_to_raw(f, x)?,

            // A bare Lookup label on a non-lookup field.
            (_, EncodeValue::Lookup(_)) => {
                return Err(EncodeError::TypeMismatch {
                    field: f.name,
                    expected: "a numeric value",
                });
            }
        };
        Ok(mask_to_width(raw_i64, bl))
    }

    fn resolve_lookup(&self, f: &FieldInfo, label: &str) -> Result<u64, EncodeError> {
        let table_name = f.lookup_enumeration.ok_or(EncodeError::TypeMismatch {
            field: f.name,
            expected: "a value for a lookup field",
        })?;
        if let Some(table) = self.db.lookup(table_name) {
            for lv in table.values {
                if lv.name == label || lv.id == Some(label) {
                    return Ok(lv.value);
                }
            }
        }
        Err(EncodeError::UnknownLookupLabel {
            field: f.name,
            label: label.to_string(),
        })
    }
}

/// Invert the decoder's `raw * resolution + offset + unit_offset`.
/// Mirrors the extraction-signedness quirk: a non-zero schema `offset`
/// forces the field unsigned, so the raw is a plain magnitude.
fn scaled_to_raw(f: &FieldInfo, scaled: f64) -> Result<i64, EncodeError> {
    let resolution = f.resolution.unwrap_or(1.0);
    let display_offset = f.offset.map(|o| o as f64).unwrap_or(0.0);
    let raw = (scaled - f.unit_offset - display_offset) / resolution;
    let rounded = raw.round();
    if !rounded.is_finite() {
        return Err(EncodeError::ValueOutOfRange {
            field: f.name,
            value: scaled,
        });
    }
    Ok(rounded as i64)
}

/// The default raw bit pattern for a field the caller left unset.
/// Stage a `STRING_FIX` field: the ASCII bytes right-padded with `0x00`
/// to the field's fixed byte width. Zero padding is what real devices
/// (e.g. the Furuno SCX-20) emit and round-trips through the decoder,
/// which trims trailing NUL / `0xff` / `@` / space.
fn stage_string_fix(f: &FieldInfo, v: EncodeValue) -> Result<Staged, EncodeError> {
    let text = match v {
        EncodeValue::Text(s) => s,
        EncodeValue::NotAvailable => String::new(),
        _ => {
            return Err(EncodeError::TypeMismatch {
                field: f.name,
                expected: "text",
            });
        }
    };
    let byte_len = f.bit_length.ok_or(EncodeError::NotFixedLength(f.name))? as usize / 8;
    let bytes = text.as_bytes();
    if bytes.len() > byte_len {
        return Err(EncodeError::ValueOutOfRange {
            field: f.name,
            value: bytes.len() as f64,
        });
    }
    // Pad with 0xff, the convention real devices and canboatjs use
    // (decoders trim 0xff, 0x00 and '@' runs alike, but matching the
    // dominant on-wire bytes keeps re-encodes bit-identical).
    let mut buf = vec![0xffu8; byte_len];
    buf[..bytes.len()].copy_from_slice(bytes);
    Ok(Staged::Bytes(buf))
}

/// The text of a string [`EncodeValue`], or an error for the wrong variant.
fn text_of(f: &FieldInfo, v: EncodeValue) -> Result<String, EncodeError> {
    match v {
        EncodeValue::Text(s) => Ok(s),
        EncodeValue::NotAvailable => Ok(String::new()),
        _ => Err(EncodeError::TypeMismatch {
            field: f.name,
            expected: "text",
        }),
    }
}

/// Stage a `STRING_LZ` field: a `[length][content]` run (canboat's
/// `fieldPrintStringLZ` — one length byte then that many content bytes).
/// A fixed-width field is `0x00`-padded to its declared byte width.
fn stage_string_lz(f: &FieldInfo, v: EncodeValue) -> Result<Staged, EncodeError> {
    let text = text_of(f, v)?;
    let content = text.as_bytes();
    if content.len() > u8::MAX as usize {
        return Err(EncodeError::ValueOutOfRange {
            field: f.name,
            value: content.len() as f64,
        });
    }
    let mut buf = Vec::with_capacity(content.len() + 1);
    buf.push(content.len() as u8);
    buf.extend_from_slice(content);
    if let Some(bl) = f.bit_length {
        let want = bl as usize / 8;
        if buf.len() > want {
            return Err(EncodeError::ValueOutOfRange {
                field: f.name,
                value: content.len() as f64,
            });
        }
        buf.resize(want, 0);
    }
    Ok(Staged::Bytes(buf))
}

/// Stage a `STRING_LAU` field: `[total_len][encoding=1][content]`, where
/// `total_len` counts the two header bytes (canboat's `fieldPrintStringLAU`).
/// Encoding `1` is ASCII/UTF-8. Always variable-length (no padding).
fn stage_string_lau(f: &FieldInfo, v: EncodeValue) -> Result<Staged, EncodeError> {
    let text = text_of(f, v)?;
    let content = text.as_bytes();
    if content.len() + 2 > u8::MAX as usize {
        return Err(EncodeError::ValueOutOfRange {
            field: f.name,
            value: content.len() as f64,
        });
    }
    let mut buf = Vec::with_capacity(content.len() + 2);
    buf.push((content.len() + 2) as u8);
    buf.push(1); // 1 = ASCII / UTF-8
    buf.extend_from_slice(content);
    Ok(Staged::Bytes(buf))
}

/// Stage a `BINARY` field: raw bytes, `0x00`-padded to the declared byte
/// width when the field is fixed-length.
fn stage_binary(f: &FieldInfo, v: EncodeValue) -> Result<Staged, EncodeError> {
    let mut bytes = match v {
        EncodeValue::Bytes(b) => b,
        EncodeValue::NotAvailable => Vec::new(),
        _ => {
            return Err(EncodeError::TypeMismatch {
                field: f.name,
                expected: "bytes",
            });
        }
    };
    if let Some(bl) = f.bit_length {
        // Not every BINARY field is byte-aligned (the 19-bit AIS
        // Communication State, for one). A sub-64-bit unaligned width
        // stages as a masked scalar so the packer writes exactly `bl`
        // bits; rendered hex always shows whole bytes, so one extra
        // padding byte is tolerated on the way in.
        if bl % 8 != 0 && bl <= 64 {
            let want = bl.div_ceil(8) as usize;
            if bytes.len() > want {
                return Err(EncodeError::ValueOutOfRange {
                    field: f.name,
                    value: bytes.len() as f64,
                });
            }
            let mut raw: u64 = 0;
            for (i, b) in bytes.iter().enumerate() {
                raw |= u64::from(*b) << (8 * i);
            }
            raw &= u64::MAX >> (64 - bl);
            return Ok(Staged::Scalar(raw));
        }
        let want = bl as usize / 8;
        if bytes.len() > want {
            return Err(EncodeError::ValueOutOfRange {
                field: f.name,
                value: bytes.len() as f64,
            });
        }
        bytes.resize(want, 0);
    }
    Ok(Staged::Bytes(bytes))
}

/// Stage a `DYNAMIC_FIELD_VALUE` field: the caller-supplied value bytes,
/// written verbatim. Unlike `BINARY` this field carries no declared
/// `BitLength` (its width comes from the resolved sibling KEY), so there
/// is nothing to pad to — the bytes are the value. Only `Bytes` (or an
/// explicit `NotAvailable` → empty) is meaningful here.
fn stage_dynamic_value(f: &FieldInfo, v: EncodeValue) -> Result<Staged, EncodeError> {
    match v {
        EncodeValue::Bytes(b) => Ok(Staged::Bytes(b)),
        EncodeValue::NotAvailable => Ok(Staged::Bytes(Vec::new())),
        _ => Err(EncodeError::TypeMismatch {
            field: f.name,
            expected: "value bytes for a dynamic (key/value) field",
        }),
    }
}

fn default_raw(f: &FieldInfo) -> u64 {
    let bl = f.bit_length.unwrap_or(0);
    // A variant-selector field: emit the value that identifies this PGN.
    if let Some(mv) = f.match_value {
        return mask_to_width(mv, bl);
    }
    match f.field_type {
        Some(FieldType::Spare) => 0,
        _ => f.unknown_value.unwrap_or_else(|| all_ones(bl)),
    }
}

/// Write the "not available" default for a field left unset by the caller.
/// Emit one field: a staged value verbatim, or the field's "not
/// available" default when unset. Shared by the top-level field walk
/// and repeating-set instance emission in [`PgnBuilder::build`].
fn emit_field(
    buf: &mut Vec<u8>,
    next_bit: &mut usize,
    f: &FieldInfo,
    staged: Option<&Staged>,
) -> Result<(), EncodeError> {
    match staged {
        // A string / binary: its packed bytes are written verbatim
        // (byte-aligned — every string/binary field is).
        Some(Staged::Bytes(bytes)) => {
            for &b in bytes {
                write_bits(buf, next_bit, 8, u64::from(b));
            }
            Ok(())
        }
        // A fixed-width scalar (number, lookup, PGN, float, …).
        Some(Staged::Scalar(v)) => {
            let bl = f.bit_length.ok_or(EncodeError::NotFixedLength(f.name))? as usize;
            write_bits(buf, next_bit, bl, *v);
            Ok(())
        }
        // Deferred values are resolved by the instance walk in build();
        // one reaching here means a VARIABLE field outside a repeating
        // set, which the schema does not produce.
        Some(Staged::Deferred(_)) => Err(EncodeError::NotFixedLength(f.name)),
        None => write_unset(buf, next_bit, f),
    }
}

fn write_unset(buf: &mut Vec<u8>, next_bit: &mut usize, f: &FieldInfo) -> Result<(), EncodeError> {
    match f.field_type {
        // Fixed string / binary: all-1s (the decoder trims the 0xff run to
        // an empty string; binary fields are whole bytes).
        Some(FieldType::StringFix) | Some(FieldType::Binary) if f.bit_length.is_some() => {
            for _ in 0..f.bit_length.unwrap() as usize / 8 {
                write_bits(buf, next_bit, 8, 0xff);
            }
        }
        // Variable-length binary: nothing to emit.
        Some(FieldType::Binary) => {}
        // Self-describing strings: the minimal empty form.
        Some(FieldType::StringLz) => write_bits(buf, next_bit, 8, 0), // length 0
        Some(FieldType::StringLau) => {
            write_bits(buf, next_bit, 8, 2); // total_len = 2 (header only)
            write_bits(buf, next_bit, 8, 1); // encoding = ASCII/UTF-8
        }
        // The group-function dynamic types take their width/shape from
        // sibling fields at decode time, so they can't be defaulted in
        // isolation — set them explicitly (or use a purpose-built 126208
        // frame).
        Some(FieldType::Variable)
        | Some(FieldType::DynamicFieldKey)
        | Some(FieldType::DynamicFieldLength)
        | Some(FieldType::DynamicFieldValue) => {
            return Err(EncodeError::NotFixedLength(f.name));
        }
        // Every fixed-width scalar: its default at the field's bit width.
        _ => {
            let bl = f.bit_length.ok_or(EncodeError::NotFixedLength(f.name))? as usize;
            write_bits(buf, next_bit, bl, default_raw(f));
        }
    }
    Ok(())
}

/// Two's-complement / mask `value` into `bits` low bits.
fn mask_to_width(value: i64, bits: u32) -> u64 {
    (value as u64) & all_ones(bits)
}

fn all_ones(bits: u32) -> u64 {
    if bits >= 64 {
        u64::MAX
    } else {
        (1u64 << bits) - 1
    }
}

/// Append `bits` low bits of `value`, LSB-first, growing `buf` as needed
/// — the exact layout [`crate::bits::extract_bits`] reads back.
fn write_bits(buf: &mut Vec<u8>, next_bit: &mut usize, bits: usize, mut value: u64) {
    for _ in 0..bits {
        let idx = *next_bit >> 3;
        let bitpos = *next_bit & 7;
        if bitpos == 0 {
            buf.push(0);
        }
        if value & 1 != 0 {
            buf[idx] |= 1 << bitpos;
        }
        value >>= 1;
        *next_bit += 1;
    }
}

/// Coerce a `FIELD=VALUE` string argument per the field's type.
fn coerce_arg(f: &FieldInfo, s: &str) -> Result<EncodeValue, EncodeError> {
    let s = s.trim();
    if s.eq_ignore_ascii_case("n/a") || s.is_empty() {
        return Ok(EncodeValue::NotAvailable);
    }
    // 0x-hex → raw integer.
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X"))
        && let Ok(n) = i64::from_str_radix(hex, 16)
    {
        return Ok(EncodeValue::Int(n));
    }
    match f.field_type {
        Some(FieldType::Lookup) | Some(FieldType::IndirectLookup) => {
            // Numeric → raw value; otherwise treat as a label.
            match s.parse::<i64>() {
                Ok(n) => Ok(EncodeValue::Int(n)),
                Err(_) => Ok(EncodeValue::Lookup(s.to_string())),
            }
        }
        Some(FieldType::Pgn) => {
            s.parse::<u32>()
                .map(EncodeValue::Pgn)
                .map_err(|_| EncodeError::ValueOutOfRange {
                    field: f.name,
                    value: f64::NAN,
                })
        }
        Some(FieldType::StringFix)
        | Some(FieldType::StringLz)
        | Some(FieldType::StringLau)
        | Some(FieldType::Variable) => Ok(EncodeValue::Text(s.to_string())),
        _ => {
            // Numeric field: an integer literal is a raw value, a decimal
            // is a scaled physical value.
            if let Ok(n) = s.parse::<i64>() {
                Ok(EncodeValue::Int(n))
            } else if let Ok(x) = s.parse::<f64>() {
                Ok(EncodeValue::Number(x))
            } else {
                Err(EncodeError::ValueOutOfRange {
                    field: f.name,
                    value: f64::NAN,
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn db() -> &'static PgnDatabase {
        PgnDatabase::embedded(crate::Units::Metric)
    }

    #[test]
    fn iso_request_matches_c_format_message() {
        // The canboat `format-message` example: request Product Info
        // (126996 = 0x1F014) → PGN 59904, data 14 f0 01.
        let frame = db()
            .encode_for(crate::pgn::ISO_REQUEST)
            .priority(6)
            .destination(255)
            .push(crate::field::iso_request::PGN, 126996u32)
            .unwrap()
            .build()
            .unwrap();
        assert_eq!(frame.pgn, 59904);
        assert_eq!(frame.prio, 6);
        assert_eq!(frame.dst, 255);
        assert_eq!(frame.data.as_slice(), &[0x14, 0xf0, 0x01]);
    }

    #[test]
    fn ambiguous_pgn_number_is_rejected() {
        // 59904 is unique; 126208 (group function) has several variants.
        assert!(db().encode_by_pgn(59904).is_ok());
        assert!(matches!(
            db().encode_by_pgn(126208),
            Err(EncodeError::AmbiguousPgn { pgn: 126208, .. })
        ));
    }

    #[test]
    fn unset_fields_default_and_length_checks() {
        // Encoding with no fields set still produces a schema-length
        // frame (defaults fill in), and match_value selector fields are
        // auto-populated so the variant stays valid.
        let frame = db().encode_for(crate::pgn::ISO_REQUEST).build();
        // isoRequest has a single PGN field with no match_value; unset →
        // not-available (all ones), 3 bytes.
        let f = frame.unwrap();
        assert_eq!(f.data.len(), 3);
        assert_eq!(f.data.as_slice(), &[0xff, 0xff, 0xff]);
    }

    #[test]
    fn wind_data_round_trips_through_decode() {
        // The real invariant: decode(encode(x)) == x. Encode a scaled
        // number, an integer, and a lookup-by-label, then decode the
        // frame back and compare physical values (unit-agnostic — the
        // compiled schema may store angles in degrees, etc.).
        use crate::field::wind_data as wd;
        let db = db();
        let frame = db
            .encode_for(crate::pgn::WIND_DATA)
            .push(wd::SID, 7)
            .unwrap()
            .push(wd::WIND_SPEED, 5.23)
            .unwrap()
            .push(wd::WIND_ANGLE, 1.5)
            .unwrap()
            // The lookup *label* stays a string — value-name constants are
            // out of scope for now.
            .push(wd::REFERENCE, EncodeValue::Lookup("Apparent".into()))
            .unwrap()
            .build()
            .unwrap();
        assert_eq!(frame.pgn, 130306);
        assert_eq!(frame.data.len(), 8);

        let decoded = db.decode(&frame).unwrap();
        assert_eq!(decoded.id, "windData");

        let num = |name: &str| match &decoded.field_by_name(name).unwrap().value {
            crate::FieldValue::Number(x) => *x,
            crate::FieldValue::Integer(n) => *n as f64,
            other => panic!("field {name}: expected a number, got {other:?}"),
        };
        // Within a resolution step (rounding on the way in).
        assert!((num("Wind Speed") - 5.23).abs() < 0.01);
        assert!((num("Wind Angle") - 1.5).abs() < 0.01);
        assert_eq!(num("SID"), 7.0);

        match &decoded.field_by_name("Reference").unwrap().value {
            crate::FieldValue::Lookup { value, name } => {
                assert_eq!(*value, 2);
                assert_eq!(*name, Some("Apparent"));
            }
            other => panic!("Reference: expected lookup, got {other:?}"),
        }
    }

    #[test]
    fn repeating_set_round_trips_through_decode() {
        // 129540 GNSS Sats in View: fixed prefix (sid, mode, reserved,
        // count) then N × {prn, elevation, azimuth, snr, residuals,
        // status}. Stage two instances, leave the count field unset (it
        // must auto-fill with 2), decode the frame back and compare the
        // per-iteration values.
        let db = db();
        let mut b = db.encode("gnssSatsInView").unwrap();
        b.push_by_name("SID", 9).unwrap();
        for (prn, snr) in [(7i64, 30.0f64), (11, 42.5)] {
            let i = b.add_set_instance(1).unwrap();
            b.push_in_set(1, i, "PRN", prn).unwrap();
            b.push_in_set(1, i, "SNR", snr).unwrap();
        }
        let frame = b.build().unwrap();
        assert_eq!(frame.pgn, 129540);

        let decoded = db.decode(&frame).unwrap();
        assert!(decoded.has_repeating_set[0]);
        let count = match &decoded.field_by_name("Sats in View").unwrap().value {
            crate::FieldValue::Integer(n) => *n,
            crate::FieldValue::Number(x) => *x as i64,
            other => panic!("Sats in View: expected a number, got {other:?}"),
        };
        assert_eq!(count, 2, "count field must auto-fill from instances");

        let per_iter = |id: &str| -> Vec<f64> {
            decoded
                .fields
                .iter()
                .filter(|f| f.info.id == id && f.repeat_set == 1)
                .map(|f| match &f.value {
                    crate::FieldValue::Integer(n) => *n as f64,
                    crate::FieldValue::Number(x) => *x,
                    other => panic!("{id}: expected a number, got {other:?}"),
                })
                .collect()
        };
        assert_eq!(per_iter("prn"), vec![7.0, 11.0]);
        let snrs = per_iter("snr");
        assert_eq!(snrs.len(), 2);
        assert!((snrs[0] - 30.0).abs() < 0.01);
        assert!((snrs[1] - 42.5).abs() < 0.01);
    }

    #[test]
    fn repeating_set_errors() {
        let db = db();
        // windData has no repeating sets.
        assert!(matches!(
            db.encode("windData").unwrap().add_set_instance(1),
            Err(EncodeError::NoRepeatingSet { set: 1, .. })
        ));
        // An instance index that was never added is rejected.
        assert!(matches!(
            db.encode("gnssSatsInView")
                .unwrap()
                .push_in_set(1, 0, "PRN", 1),
            Err(EncodeError::NoSuchInstance { instance: 0, .. })
        ));
    }

    #[test]
    fn group_function_variable_values_encode_via_the_referenced_pgn() {
        // A 126208 Command Group Function for PGN 127250 (Vessel
        // Heading): parameter 2 is the Heading field, whose width
        // (16 bits) and resolution (1e-4 rad) live in 127250, not in
        // 126208. The deferred VARIABLE path must resolve them.
        let db = db();
        let mut b = db.encode("nmeaCommandGroupFunction").unwrap();
        b.push_by_name("PGN", EncodeValue::Pgn(127250)).unwrap();
        let heading_order = db
            .first_pgn(127250)
            .unwrap()
            .fields
            .iter()
            .find(|f| f.id == "heading")
            .unwrap()
            .order as i64;
        let i = b.add_set_instance(1).unwrap();
        b.push_in_set(1, i, "Parameter", heading_order).unwrap();
        b.push_in_set(1, i, "Value", 1.5f64).unwrap();
        let frame = b.build().map_err(|e| format!("{e}")).unwrap();
        assert_eq!(frame.pgn, 126208);

        let decoded = db.decode(&frame).unwrap();
        let val = decoded
            .fields
            .iter()
            .find(|f| f.repeat_set == 1 && f.info.field_type == Some(FieldType::Variable))
            .expect("decoded VARIABLE value");
        match &val.value {
            crate::FieldValue::Number(x) => assert!((x - 1.5).abs() < 0.01, "got {x}"),
            crate::FieldValue::Integer(n) => assert!((*n as f64 - 1.5).abs() < 1.0, "got {n}"),
            other => panic!("value: {other:?}"),
        }
    }

    #[test]
    fn variable_without_target_is_a_clear_error() {
        let db = db();
        let mut b = db.encode("nmeaCommandGroupFunction").unwrap();
        let i = b.add_set_instance(1).unwrap();
        b.push_in_set(1, i, "Value", 1i64).unwrap();
        assert!(matches!(
            b.build(),
            Err(EncodeError::VariableUnresolved { .. })
        ));
    }

    #[test]
    fn unknown_lookup_label_is_rejected() {
        assert!(matches!(
            db().encode("windData")
                .unwrap()
                .push_by_name("Reference", EncodeValue::Lookup("Nonsense".into())),
            Err(EncodeError::UnknownLookupLabel { .. })
        ));
    }

    #[test]
    fn no_such_field_and_pgn() {
        assert!(matches!(
            db().encode("isoRequest").unwrap().push_by_name("Nope", 1),
            Err(EncodeError::NoSuchField { .. })
        ));
        assert!(matches!(
            db().encode("notAPgnId"),
            Err(EncodeError::NoSuchPgnId(_))
        ));
    }

    #[test]
    fn string_fix_round_trips() {
        use crate::field::product_information::MODEL_ID;
        let frame = db()
            .encode("productInformation")
            .unwrap()
            .push(MODEL_ID, "SCX-20")
            .unwrap()
            .build()
            .unwrap();
        let d = db().decode(&frame).unwrap();
        assert_eq!(
            d.field(MODEL_ID).and_then(|f| f.value.as_str()),
            Some("SCX-20")
        );
    }

    #[test]
    fn string_lau_round_trips() {
        use crate::field::configuration_information::INSTALLATION_DESCRIPTION1 as DESC1;
        let frame = db()
            .encode("configurationInformation")
            .unwrap()
            .push(DESC1, "Helm Station")
            .unwrap()
            .build()
            .unwrap();
        let d = db().decode(&frame).unwrap();
        assert_eq!(
            d.field(DESC1).and_then(|f| f.value.as_str()),
            Some("Helm Station")
        );
    }

    #[test]
    fn string_lz_round_trips() {
        use crate::field::navico_ascii_identifier::IDENTIFIER;
        let frame = db()
            .encode("navicoAsciiIdentifier")
            .unwrap()
            .push(IDENTIFIER, "BOW")
            .unwrap()
            .build()
            .unwrap();
        let d = db().decode(&frame).unwrap();
        assert_eq!(
            d.field(IDENTIFIER).and_then(|f| f.value.as_str()),
            Some("BOW")
        );
    }

    #[test]
    fn binary_round_trips() {
        // A fixed-width (7-byte) BINARY field: a full-width payload round-
        // trips exactly (a shorter one would be 0x00-padded to the width).
        use crate::field::iso_transport_protocol_data_transfer::DATA;
        let payload = vec![0xde_u8, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03];
        let frame = db()
            .encode("isoTransportProtocolDataTransfer")
            .unwrap()
            .push(DATA, payload.clone())
            .unwrap()
            .build()
            .unwrap();
        let d = db().decode(&frame).unwrap();
        match d.field(DATA).map(|f| &f.value) {
            Some(crate::FieldValue::Binary(v)) => assert_eq!(v, &payload),
            other => panic!("expected binary, got {other:?}"),
        }
    }

    #[test]
    fn float_round_trips() {
        // FLOAT is written as raw IEEE-754 bits (no resolution scaling),
        // so it survives a round-trip within f32 precision.
        use crate::field::garmin_autopilot_heading_to_steer::HEADING_TO_STEER as HTS;
        let frame = db()
            .encode("garminAutopilotHeadingToSteer")
            .unwrap()
            .push(HTS, 1.5)
            .unwrap()
            .build()
            .unwrap();
        let d = db().decode(&frame).unwrap();
        let got = d.field(HTS).and_then(|f| f.value.as_f64()).unwrap();
        assert!((got - 1.5).abs() < 1e-6, "got {got}");
    }

    #[test]
    fn iso_name_round_trips() {
        // A NAME with the top (arbitrary-address-capable) bit set exercises
        // the full u64 range that an i64 alone couldn't express.
        use crate::field::simnet_data_source_selection::SOURCE;
        let name: u64 = 0xC078_8C00_E7E0_4364;
        let frame = db()
            .encode("simnetDataSourceSelection")
            .unwrap()
            .push(SOURCE, name)
            .unwrap()
            .build()
            .unwrap();
        let d = db().decode(&frame).unwrap();
        match d.field(SOURCE).map(|f| &f.value) {
            Some(crate::FieldValue::IsoName { value, .. }) => assert_eq!(*value, name),
            other => panic!("expected iso name, got {other:?}"),
        }
    }

    #[test]
    fn simnet_key_value_encodes_key_and_value() {
        // PGN 130845 simnetKeyValue is a key/value PGN: the KEY is a
        // fixed 24-bit DYNAMIC_FIELD_KEY (set as a raw Int) and the VALUE
        // is a DYNAMIC_FIELD_VALUE whose bytes are written verbatim. This
        // is the frame merrimac sends to set Simrad display night mode
        // (key 9983, value 0x04).
        let frame = db()
            .encode("simnetKeyValue")
            .unwrap()
            .destination(255)
            .push_by_name("Address", EncodeValue::Int(255))
            .unwrap()
            .push_by_name("Instance", EncodeValue::Int(255))
            .unwrap()
            .push_by_name("Network Group", EncodeValue::Int(1))
            .unwrap()
            .push_by_name("Source", EncodeValue::Int(255))
            .unwrap()
            .push_by_name("Key", EncodeValue::Int(9983))
            .unwrap()
            .push_by_name("Operation", EncodeValue::Int(1))
            .unwrap()
            .push_by_name("Value", EncodeValue::Bytes(vec![0x04]))
            .unwrap()
            .build()
            .unwrap();
        assert_eq!(frame.pgn, 130845);
        // 6 header bytes (11+2+3+8+8+8+8 = 48 bits) + 3 key + 1 op + 1 value.
        assert_eq!(frame.data.len(), 11);
        // Manufacturer Code (match 1857) + Industry Code (match 4) auto-fill.
        // Key 9983 = 0x26FF, little-endian 24-bit at bytes 6..9.
        assert_eq!(&frame.data[6..9], &[0xff, 0x26, 0x00]);
        assert_eq!(frame.data[9], 0x01); // Operation = Set
        assert_eq!(frame.data[10], 0x04); // Value byte, verbatim
    }

    #[test]
    fn dynamic_field_value_rejects_non_bytes() {
        // The value slot has no declared width; a scalar can't be sized.
        assert!(matches!(
            db().encode("simnetKeyValue")
                .unwrap()
                .push_by_name("Value", EncodeValue::Int(4)),
            Err(EncodeError::TypeMismatch { .. })
        ));
    }

    #[test]
    fn dynamic_group_function_field_is_a_clear_error() {
        // The 126208 group-function dynamic types can't be defaulted in
        // isolation; an unset one inside a staged parameter pair is a
        // descriptive error, not wrong bytes.
        let mut b = db().encode("nmeaRequestGroupFunction").unwrap();
        b.add_set_instance(1).unwrap();
        let err = b.build();
        assert!(
            matches!(err, Err(EncodeError::NotFixedLength(_))),
            "got {err:?}"
        );
    }

    #[test]
    fn zero_instance_repeating_pgn_builds_with_zero_count() {
        // A bare group-function request is a legitimate zero-parameter
        // message: the repeating set is absent from the payload and the
        // count field auto-fills with 0 (a real count, not
        // "unavailable"). The repeating fields' dynamic widths never
        // come into play.
        let frame = db()
            .encode("nmeaRequestGroupFunction")
            .unwrap()
            .build()
            .unwrap();
        assert_eq!(frame.pgn, 126208);
        let decoded = db().decode(&frame).unwrap();
        match &decoded
            .field_by_name("Number of Parameters")
            .expect("count field decoded")
            .value
        {
            crate::FieldValue::Integer(n) => assert_eq!(*n, 0),
            crate::FieldValue::Number(x) => assert_eq!(*x, 0.0),
            other => panic!("count: expected a number, got {other:?}"),
        }
    }
}
