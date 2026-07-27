// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Build script: read this crate's `data/canboat.json` (+ `data/synthetic-pgns.json`)
//! and emit a Rust source file with the full schema as `&'static`
//! tables. The output is `include!`d from `src/schema_data.rs` and
//! published as a `static PgnDatabase` (see `src/db.rs`).
//!
//! Replaces what used to be a runtime `serde_json::from_reader` over a
//! 2.3 MB JSON document plus a HashMap build pass. After this, decoder
//! callers pay zero startup cost for the schema.

use serde::Deserialize;
use std::collections::BTreeMap;
use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::PathBuf;

// ---------------------------------------------------------------------
// Build-script deserialization types.
//
// These mirror canboat.json's PascalCase shape. They are PRIVATE to the
// build script and not exposed anywhere else — runtime types live in
// `canboat-schema`. The mapping from these owned types to the const
// form happens in the emit_* functions below.
// ---------------------------------------------------------------------

fn de_loose_string<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<String>, D::Error> {
    use serde_json::Value;
    let v = Option::<Value>::deserialize(d)?;
    Ok(match v {
        None | Some(Value::Null) => None,
        Some(Value::String(s)) => Some(s),
        Some(Value::Number(n)) => Some(n.to_string()),
        Some(Value::Bool(b)) => Some(b.to_string()),
        Some(other) => Some(other.to_string()),
    })
}

#[derive(Deserialize)]
#[serde(rename_all = "PascalCase")]
struct CanboatJson {
    schema_version: String,
    version: String,
    copyright: String,
    #[serde(rename = "PGNs")]
    pgns: Vec<RawPgn>,
    #[serde(default)]
    lookup_enumerations: Vec<RawLookup>,
    #[serde(default)]
    lookup_bit_enumerations: Vec<RawBitLookup>,
    #[serde(default)]
    lookup_indirect_enumerations: Vec<RawIndirectLookup>,
    #[serde(default)]
    lookup_field_type_enumerations: Vec<RawFieldTypeLookup>,
}

#[derive(Deserialize)]
struct SyntheticJson {
    #[serde(rename = "PGNs")]
    pgns: Vec<RawPgn>,
}

#[derive(Deserialize, Clone)]
#[serde(rename_all = "PascalCase")]
struct RawPgn {
    #[serde(rename = "PGN")]
    pgn: u32,
    id: String,
    description: String,
    explanation: Option<String>,
    #[serde(rename = "URL")]
    url: Option<String>,
    #[serde(rename = "Type")]
    packet_type: String,
    complete: Option<bool>,
    fallback: Option<bool>,
    #[serde(default)]
    missing: Option<Vec<String>>,
    priority: Option<u8>,
    transmission_interval: Option<u32>,
    transmission_irregular: Option<bool>,
    field_count: u32,
    length: Option<u32>,
    min_length: Option<u32>,
    fields: Vec<RawField>,
    repeating_field_set1_size: Option<u32>,
    repeating_field_set1_start_field: Option<u32>,
    repeating_field_set1_count_field: Option<u32>,
    repeating_field_set2_size: Option<u32>,
    repeating_field_set2_start_field: Option<u32>,
    repeating_field_set2_count_field: Option<u32>,
}

#[derive(Deserialize, Clone)]
#[serde(rename_all = "PascalCase")]
struct RawField {
    order: u8,
    id: String,
    name: String,
    #[serde(default, deserialize_with = "de_loose_string")]
    description: Option<String>,
    bit_length: Option<u32>,
    #[serde(default, deserialize_with = "de_loose_string")]
    bit_length_field: Option<String>,
    bit_length_variable: Option<bool>,
    bit_offset: Option<u32>,
    bit_start: Option<u32>,
    resolution: Option<f64>,
    signed: Option<bool>,
    offset: Option<i64>,
    range_min: Option<f64>,
    range_max: Option<f64>,
    #[serde(rename = "UnknownValue")]
    unknown_value: Option<u64>,
    #[serde(rename = "OutOfRangeValue")]
    out_of_range_value: Option<u64>,
    #[serde(rename = "ReservedValue")]
    reserved_value: Option<u64>,
    unit: Option<String>,
    physical_quantity: Option<String>,
    field_type: Option<String>,
    lookup_enumeration: Option<String>,
    lookup_bit_enumeration: Option<String>,
    lookup_indirect_enumeration: Option<String>,
    lookup_indirect_enumeration_field_order: Option<u8>,
    lookup_field_type_enumeration: Option<String>,
    #[serde(rename = "Match")]
    match_value: Option<i64>,
    part_of_primary_key: Option<bool>,
    condition: Option<String>,
}

#[derive(Deserialize)]
struct RawLookup {
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "MaxValue")]
    max_value: Option<u64>,
    #[serde(rename = "EnumValues")]
    values: Vec<RawLookupValue>,
}
#[derive(Deserialize)]
struct RawLookupValue {
    #[serde(rename = "Value")]
    value: u64,
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "Id")]
    id: Option<String>,
}

#[derive(Deserialize)]
struct RawBitLookup {
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "MaxValue")]
    max_value: Option<u64>,
    #[serde(rename = "EnumBitValues")]
    values: Vec<RawBitLookupValue>,
}
#[derive(Deserialize)]
struct RawBitLookupValue {
    #[serde(rename = "Bit")]
    bit: u8,
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "Id")]
    id: Option<String>,
}

#[derive(Deserialize)]
struct RawIndirectLookup {
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "MaxValue")]
    max_value: Option<u64>,
    #[serde(rename = "EnumValues")]
    values: Vec<RawIndirectLookupValue>,
}
#[derive(Deserialize)]
struct RawIndirectLookupValue {
    #[serde(rename = "Value1")]
    value1: u64,
    #[serde(rename = "Value2")]
    value2: u64,
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "Id")]
    id: Option<String>,
}

#[derive(Deserialize)]
struct RawFieldTypeLookup {
    #[serde(rename = "Name")]
    name: String,
    #[serde(rename = "MaxValue")]
    max_value: Option<u64>,
    #[serde(rename = "EnumFieldTypeValues")]
    values: Vec<RawFieldTypeValue>,
}
#[derive(Deserialize, Clone)]
struct RawFieldTypeValue {
    #[serde(rename = "value")]
    value: u64,
    #[serde(rename = "name")]
    name: String,
    #[serde(rename = "FieldType")]
    field_type: Option<String>,
    #[serde(rename = "Bits", default, deserialize_with = "de_loose_string")]
    bits: Option<String>,
    #[serde(rename = "Resolution")]
    resolution: Option<f64>,
    #[serde(rename = "Unit")]
    unit: Option<String>,
    #[serde(rename = "LookupEnumeration")]
    lookup_enumeration: Option<String>,
    #[serde(rename = "LookupBitEnumeration")]
    lookup_bit_enumeration: Option<String>,
}

// ---------------------------------------------------------------------
// Non-SI unit fix-up — mirrors `db.rs::apply_non_si_unit_fixup`.
//
// Applied at codegen so the runtime tables already carry display units
// (`deg`, `°C`, `bar`, `Ah`) instead of canboat.json's SI base units
// (`rad`, `K`, `Pa`, `C`).
// ---------------------------------------------------------------------

const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;

/// Which unit system a generated schema presents. `Si` keeps
/// canboat.json's base units (`rad`, `K`, `Pa`, `C`); `Metric` applies
/// canboat's practical `fixupUnit` (`deg`, `°C`, `bar`, `Ah`).
#[derive(PartialEq, Eq, Clone, Copy)]
enum Units {
    Si,
    Metric,
}

/// `(absolute_idx_into_PGNS, parsed PGN record, parsed fields)` — the
/// unit the Phase 3 dispatcher emitter walks.
type VariantEntry<'a> = (usize, &'a RawPgn, &'a Vec<RawField>);

/// The units-dependent presentation of a field: everything `fixupUnit`
/// touches, plus the two codegen-derived display fields. Computed
/// per-`Units` at emit time so the same [`RawField`] can back both the
/// SI and Metric schemas.
struct FieldView {
    resolution: Option<f64>,
    unit: Option<String>,
    range_min: Option<f64>,
    range_max: Option<f64>,
    unit_offset: f64,
    precision: u8,
    is_dynamic_length_marker: bool,
}

/// True when `f`'s unit is one canboat's `fixupUnit` rewrites, i.e. the
/// SI and Metric views of the field differ. A PGN with no such field
/// can share one `FieldInfo` slice between both schemas.
fn field_converts(f: &RawField) -> bool {
    match f.unit.as_deref() {
        Some("rad") | Some("rad/s") | Some("Pa") | Some("C") => true,
        Some("K") => !f.signed.unwrap_or(false),
        _ => false,
    }
}

fn field_view(f: &RawField, units: Units) -> FieldView {
    let mut v = FieldView {
        resolution: f.resolution,
        unit: f.unit.clone(),
        range_min: f.range_min,
        range_max: f.range_max,
        unit_offset: 0.0,
        precision: 0,
        is_dynamic_length_marker: f.name == "Length",
    };

    // canboat hard-codes 7-decimal display precision for lat/lon
    // (units-independent).
    if matches!(
        f.physical_quantity.as_deref(),
        Some("GEOGRAPHICAL_LATITUDE") | Some("GEOGRAPHICAL_LONGITUDE")
    ) {
        v.precision = 7;
    }

    if units == Units::Si {
        return v; // strict SI base units — no fixupUnit
    }

    match f.unit.as_deref() {
        Some("rad") => {
            v.resolution = f.resolution.map(|r| r * RAD_TO_DEG);
            v.range_min = f.range_min.map(|x| x * RAD_TO_DEG);
            v.range_max = f.range_max.map(|x| (x * RAD_TO_DEG).max(360.0));
            v.unit = Some("deg".to_string());
            v.precision = 1;
        }
        Some("rad/s") => {
            v.resolution = f.resolution.map(|r| r * RAD_TO_DEG);
            v.range_min = f.range_min.map(|x| x * RAD_TO_DEG);
            v.range_max = f.range_max.map(|x| x * RAD_TO_DEG);
            v.unit = Some("deg/s".to_string());
        }
        Some("K") if !f.signed.unwrap_or(false) => {
            v.unit_offset = -273.15;
            v.range_min = f.range_min.map(|x| x - 273.15);
            // Match canboat's faithfully-quirky 275.15 subtraction.
            v.range_max = f.range_max.map(|x| x - 275.15);
            v.unit = Some("C".to_string());
        }
        Some("Pa") => {
            v.resolution = f.resolution.map(|r| r / 100_000.0);
            v.range_min = f.range_min.map(|x| x / 100_000.0);
            v.range_max = f.range_max.map(|x| x / 100_000.0);
            v.unit = Some("bar".to_string());
            v.precision = 3;
        }
        Some("C") => {
            v.resolution = f.resolution.map(|r| r / 3600.0);
            v.range_min = f.range_min.map(|x| x / 3600.0);
            v.range_max = f.range_max.map(|x| x / 3600.0);
            v.unit = Some("Ah".to_string());
        }
        _ => {}
    }
    v
}

#[derive(Default, Clone, Copy)]
struct ComputedFt {
    signed: bool,
    precision: u8,
}

fn compute_ft(v: &mut RawFieldTypeValue) -> ComputedFt {
    let mut c = ComputedFt::default();
    let Some(unit) = v.unit.clone() else {
        return c;
    };
    if matches!(unit.as_str(), "rad" | "rad/s" | "deg" | "deg/s") {
        c.signed = true;
    }
    match unit.as_str() {
        "rad" => {
            if let Some(r) = v.resolution.as_mut() {
                *r *= RAD_TO_DEG;
            }
            v.unit = Some("deg".to_string());
            c.precision = 1;
        }
        "rad/s" => {
            if let Some(r) = v.resolution.as_mut() {
                *r *= RAD_TO_DEG;
            }
            v.unit = Some("deg/s".to_string());
        }
        "Pa" => {
            if let Some(r) = v.resolution.as_mut() {
                *r /= 100_000.0;
            }
            v.unit = Some("bar".to_string());
            c.precision = 3;
        }
        "C" => {
            if let Some(r) = v.resolution.as_mut() {
                *r /= 3600.0;
            }
            v.unit = Some("Ah".to_string());
        }
        _ => {}
    }
    c
}

// ---------------------------------------------------------------------
// Emit helpers.
// ---------------------------------------------------------------------

/// Quote any `&str` as a valid Rust string literal. Uses `{:?}` which
/// produces a debug-formatted string that round-trips (escapes quotes,
/// backslashes, non-printable chars).
fn quote(s: &str) -> String {
    format!("{s:?}")
}

/// Mirror canboat C's `camelize()` in `analyzer/pgn.c`. Strips every
/// non-alphanumeric character, lowercases the first surviving char,
/// and uppercases the first char following a separator.
///
/// Used at build time to decide whether a PGN's `Id` was explicitly
/// pinned: when `camelize(description) != id`, the canboat C source
/// set `.camelDescription` to override the naturally-derived id (the
/// only case in v7.1.0 is PGN 130846 — `Id="simnetParameterSet"`
/// preserving the v7.0.0 contract after the Description rename), and
/// the JSON formatter wraps that PGN's record in `{"<id>":{…}}` to
/// signal the stable id to downstream consumers.
fn camelize_lower(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut last_is_alpha = true; // mirrors `!upperCamelCase`
    for c in s.chars() {
        if c.is_ascii_alphanumeric() {
            if last_is_alpha {
                out.extend(c.to_lowercase());
            } else {
                out.extend(c.to_uppercase());
                last_is_alpha = true;
            }
        } else {
            last_is_alpha = false;
        }
    }
    out
}

fn opt_str(s: &Option<String>) -> String {
    match s {
        Some(v) => format!("Some({})", quote(v)),
        None => "None".to_string(),
    }
}

fn opt_int<T: std::fmt::Display>(v: &Option<T>) -> String {
    match v {
        Some(x) => format!("Some({x})"),
        None => "None".to_string(),
    }
}

fn opt_bool(v: &Option<bool>) -> String {
    match v {
        Some(x) => format!("Some({x})"),
        None => "None".to_string(),
    }
}

/// Float emitter that round-trips bit-exactly. Falls back to a debug
/// repr (`{:?}`) which Rust accepts as a literal for finite values; for
/// the (unlikely) non-finite case in the schema we'd want to know loudly.
fn float(v: f64) -> String {
    if !v.is_finite() {
        panic!("non-finite value in canboat schema: {v}");
    }
    // `{:?}` always emits with a decimal point (or e-notation) so the
    // result is a valid `f64` literal in Rust source.
    format!("{v:?}")
}

fn opt_float(v: &Option<f64>) -> String {
    match v {
        Some(x) => format!("Some({})", float(*x)),
        None => "None".to_string(),
    }
}

fn packet_type(t: &str) -> &'static str {
    match t {
        "Single" => "PacketType::Single",
        "Fast" => "PacketType::Fast",
        "ISO" => "PacketType::Iso",
        "Mixed" => "PacketType::Mixed",
        other => panic!("unknown PacketType in canboat.json: {other:?}"),
    }
}

fn field_type(t: &str) -> &'static str {
    match t {
        "NUMBER" => "FieldType::Number",
        "DECIMAL" => "FieldType::Decimal",
        "FLOAT" => "FieldType::Float",
        "BINARY" => "FieldType::Binary",
        "LOOKUP" => "FieldType::Lookup",
        "BITLOOKUP" => "FieldType::BitLookup",
        "INDIRECT_LOOKUP" => "FieldType::IndirectLookup",
        "DATE" => "FieldType::Date",
        "TIME" => "FieldType::Time",
        "DURATION" => "FieldType::Duration",
        "STRING_FIX" => "FieldType::StringFix",
        "STRING_LZ" => "FieldType::StringLz",
        "STRING_LAU" => "FieldType::StringLau",
        "VARIABLE" => "FieldType::Variable",
        "MMSI" => "FieldType::Mmsi",
        "PGN" => "FieldType::Pgn",
        "ISO_NAME" => "FieldType::IsoName",
        "RESERVED" => "FieldType::Reserved",
        "SPARE" => "FieldType::Spare",
        "DYNAMIC_FIELD_KEY" => "FieldType::DynamicFieldKey",
        "DYNAMIC_FIELD_LENGTH" => "FieldType::DynamicFieldLength",
        "DYNAMIC_FIELD_VALUE" => "FieldType::DynamicFieldValue",
        "FIELD_INDEX" => "FieldType::FieldIndex",
        other => panic!("unknown FieldType in canboat.json: {other:?}"),
    }
}

fn opt_field_type(t: &Option<String>) -> String {
    match t {
        Some(s) => format!("Some({})", field_type(s)),
        None => "None".to_string(),
    }
}

fn missing_arr(m: &Option<Vec<String>>) -> String {
    let mut out = String::from("&[");
    if let Some(v) = m {
        for s in v {
            out.push_str(&quote(s));
            out.push(',');
        }
    }
    out.push(']');
    out
}

fn emit_field(out: &mut String, f: &RawField, v: &FieldView) {
    write!(
        out,
        "FieldInfo{{order:{order},id:{id},name:{name},description:{description},\
         bit_length:{bit_length},bit_length_field:{blf},bit_length_variable:{blv},\
         bit_offset:{bit_offset},bit_start:{bit_start},resolution:{resolution},\
         signed:{signed},offset:{offset},range_min:{range_min},range_max:{range_max},\
         unknown_value:{uv},out_of_range_value:{orv},reserved_value:{rv},\
         unit:{unit},physical_quantity:{pq},field_type:{ft},\
         lookup_enumeration:{le},lookup_bit_enumeration:{lbe},lookup_indirect_enumeration:{lie},\
         lookup_indirect_enumeration_field_order:{liefo},lookup_field_type_enumeration:{lfte},\
         match_value:{mv},part_of_primary_key:{popk},condition:{cond},\
         unit_offset:{unit_offset},precision:{precision},is_dynamic_length_marker:{dlm}}},",
        order = f.order,
        id = quote(&f.id),
        name = quote(&f.name),
        description = opt_str(&f.description),
        bit_length = opt_int(&f.bit_length),
        blf = opt_str(&f.bit_length_field),
        blv = opt_bool(&f.bit_length_variable),
        bit_offset = opt_int(&f.bit_offset),
        bit_start = opt_int(&f.bit_start),
        // Units-dependent presentation comes from the FieldView.
        resolution = opt_float(&v.resolution),
        signed = opt_bool(&f.signed),
        offset = opt_int(&f.offset),
        range_min = opt_float(&v.range_min),
        range_max = opt_float(&v.range_max),
        uv = opt_int(&f.unknown_value),
        orv = opt_int(&f.out_of_range_value),
        rv = opt_int(&f.reserved_value),
        unit = opt_str(&v.unit),
        pq = opt_str(&f.physical_quantity),
        ft = opt_field_type(&f.field_type),
        le = opt_str(&f.lookup_enumeration),
        lbe = opt_str(&f.lookup_bit_enumeration),
        lie = opt_str(&f.lookup_indirect_enumeration),
        liefo = opt_int(&f.lookup_indirect_enumeration_field_order),
        lfte = opt_str(&f.lookup_field_type_enumeration),
        mv = opt_int(&f.match_value),
        popk = opt_bool(&f.part_of_primary_key),
        cond = opt_str(&f.condition),
        unit_offset = float(v.unit_offset),
        precision = v.precision,
        dlm = v.is_dynamic_length_marker,
    )
    .unwrap();
}

/// Emit `static {ident}: &[FieldInfo] = &[ … ];` — one PGN's fields in
/// the given unit system. Shared between both schemas when the PGN has
/// no convertible field.
fn emit_field_array(out: &mut String, ident: &str, fields: &[RawField], units: Units) {
    write!(out, "static {ident}:&[FieldInfo]=&[").unwrap();
    for f in fields {
        emit_field(out, f, &field_view(f, units));
    }
    writeln!(out, "];").unwrap();
}

/// Emit one `PgnInfo` literal that references its fields by the named
/// static `fields_ident` (so SI and Metric can share the same slice).
fn emit_pgn(out: &mut String, p: &RawPgn, fields_ident: &str, from_synthetic: bool) {
    // Synthetic PGNs (post-canboat.json list) never wrap — they're
    // not in canboat C's PGN list, so the camelDescription contract
    // doesn't apply. Otherwise: pinned iff the natural camelize of
    // the description doesn't yield the declared Id.
    let id_is_pinned = !from_synthetic && camelize_lower(&p.description) != p.id;
    writeln!(
        out,
        "PgnInfo{{pgn:{pgn},id:{id},description:{description},explanation:{explanation},\
         url:{url},packet_type:{packet_type},complete:{complete},fallback:{fallback},\
         missing:{missing},priority:{priority},transmission_interval:{ti},\
         transmission_irregular:{tirr},field_count:{fc},length:{length},min_length:{ml},\
         fields:{fields_ident},\
         repeating_field_set1_size:{r1s},repeating_field_set1_start_field:{r1sf},\
         repeating_field_set1_count_field:{r1cf},repeating_field_set2_size:{r2s},\
         repeating_field_set2_start_field:{r2sf},repeating_field_set2_count_field:{r2cf},\
         id_is_pinned:{pinned}}},",
        pgn = p.pgn,
        id = quote(&p.id),
        description = quote(&p.description),
        explanation = opt_str(&p.explanation),
        url = opt_str(&p.url),
        packet_type = packet_type(&p.packet_type),
        complete = opt_bool(&p.complete),
        fallback = opt_bool(&p.fallback),
        missing = missing_arr(&p.missing),
        priority = opt_int(&p.priority),
        ti = opt_int(&p.transmission_interval),
        tirr = opt_bool(&p.transmission_irregular),
        fc = p.field_count,
        length = opt_int(&p.length),
        ml = opt_int(&p.min_length),
        r1s = opt_int(&p.repeating_field_set1_size),
        r1sf = opt_int(&p.repeating_field_set1_start_field),
        r1cf = opt_int(&p.repeating_field_set1_count_field),
        r2s = opt_int(&p.repeating_field_set2_size),
        r2sf = opt_int(&p.repeating_field_set2_start_field),
        r2cf = opt_int(&p.repeating_field_set2_count_field),
        pinned = id_is_pinned,
    )
    .unwrap();
}

/// camelCase canboat id → `SCREAMING_SNAKE_CASE` (a static/const name).
/// Breaks only at a lower/digit → upper boundary, so `cogSogRapidUpdate`
/// → `COG_SOG_RAPID_UPDATE` and `windAngle` → `WIND_ANGLE`. Any non
/// `[A-Za-z0-9]` byte becomes `_`, and a leading digit is `_`-prefixed so
/// the result is always a valid identifier.
fn screaming(id: &str) -> String {
    let mut out = String::with_capacity(id.len() + 4);
    let mut prev_lower_or_digit = false;
    for c in id.chars() {
        if c.is_ascii_uppercase() && prev_lower_or_digit {
            out.push('_');
        }
        out.push(if c.is_ascii_alphanumeric() {
            c.to_ascii_uppercase()
        } else {
            '_'
        });
        prev_lower_or_digit = c.is_ascii_lowercase() || c.is_ascii_digit();
    }
    if out.starts_with(|c: char| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    out
}

/// camelCase canboat id → `snake_case` module name. Keyword collisions
/// (none today, but pgn ids come from an external file) get a trailing
/// `_` so the module name always parses.
fn snake(id: &str) -> String {
    let s = screaming(id).to_ascii_lowercase();
    const KEYWORDS: &[&str] = &[
        "as", "break", "const", "continue", "crate", "dyn", "else", "enum", "extern", "false",
        "fn", "for", "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub",
        "ref", "return", "self", "static", "struct", "super", "trait", "true", "type", "union",
        "unsafe", "use", "where", "while", "async", "await", "gen",
    ];
    if KEYWORDS.contains(&s.as_str()) {
        format!("{s}_")
    } else {
        s
    }
}

/// Emit the id-keyed constant references:
///
/// * `pub mod pgn` — one `pub static <ID>: &PgnInfo` per PGN definition.
/// * `pub mod field` — a `pub mod <pgn_id>` per PGN, each with a
///   `pub static <FIELD_ID>: FieldRef` per field.
///
/// Both index the SI arrays emitted above (`PGNS_SI`, `F{i}`), which is
/// legal because a `static`'s initializer may reference another `static`
/// by address; id/name/order are unit-invariant so these double for
/// Metric. Duplicate ids (repeated `reserved`/`spare` fields, and the
/// rare shared PGN id) keep the first occurrence, mirroring
/// [`PgnDatabase::pgn_by_id`].
fn emit_id_constants(out: &mut String, pgns: &[(RawPgn, Vec<RawField>)]) {
    use std::collections::HashSet;

    writeln!(
        out,
        "/// Compile-time `&'static PgnInfo` per PGN definition, keyed by canboat id.\n\
         ///\n\
         /// `pgn::WIND_DATA` is the SI-schema line for `windData`; the id,\n\
         /// description, and field metadata are unit-invariant so it also\n\
         /// describes the Metric schema.\n\
         pub mod pgn {{\n\
         use super::PGNS_SI;\n\
         use canboat_schema::PgnInfo;"
    )
    .unwrap();
    let mut used: HashSet<&str> = HashSet::new();
    for (i, (p, _)) in pgns.iter().enumerate() {
        let name = screaming(&p.id);
        if name.is_empty() || !used.insert(p.id.as_str()) {
            continue;
        }
        writeln!(out, "pub static {name}: &PgnInfo = &PGNS_SI[{i}];").unwrap();
    }
    writeln!(out, "}}").unwrap();

    writeln!(
        out,
        "/// Compile-time [`FieldRef`] per (PGN, field), grouped by PGN id:\n\
         /// `field::wind_data::WIND_ANGLE`.\n\
         pub mod field {{"
    )
    .unwrap();
    let mut used_mods: HashSet<String> = HashSet::new();
    for (i, (p, fields)) in pgns.iter().enumerate() {
        let module = snake(&p.id);
        if module.is_empty() || !used_mods.insert(module.clone()) {
            continue;
        }
        // Collect first so an empty (or fully-deduped) PGN emits no module
        // and therefore no unused `use`.
        let mut consts: Vec<(String, usize)> = Vec::with_capacity(fields.len());
        let mut used_fields: HashSet<&str> = HashSet::new();
        for (j, f) in fields.iter().enumerate() {
            let name = screaming(&f.id);
            if name.is_empty() || !used_fields.insert(f.id.as_str()) {
                continue;
            }
            consts.push((name, j));
        }
        if consts.is_empty() {
            continue;
        }
        writeln!(
            out,
            "pub mod {module} {{\nuse super::super::{{PGNS_SI, F{i}}};\nuse canboat_schema::FieldRef;"
        )
        .unwrap();
        for (name, j) in consts {
            writeln!(
                out,
                "pub static {name}: FieldRef = FieldRef {{ pgn: &PGNS_SI[{i}], field: &F{i}[{j}] }};"
            )
            .unwrap();
        }
        writeln!(out, "}}").unwrap();
    }
    writeln!(out, "}}").unwrap();
}

fn emit_lookup_value(out: &mut String, v: &RawLookupValue) {
    write!(
        out,
        "LookupValue{{value:{},name:{},id:{}}},",
        v.value,
        quote(&v.name),
        opt_str(&v.id),
    )
    .unwrap();
}

fn emit_lookup(out: &mut String, t: &RawLookup) {
    write!(
        out,
        "LookupTable{{name:{},max_value:{},values:&[",
        quote(&t.name),
        opt_int(&t.max_value),
    )
    .unwrap();
    for v in &t.values {
        emit_lookup_value(out, v);
    }
    write!(out, "],by_value:&[").unwrap();
    // Build sorted (value, index) pairs.
    let mut pairs: Vec<(u64, u32)> = t
        .values
        .iter()
        .enumerate()
        .map(|(i, v)| (v.value, i as u32))
        .collect();
    pairs.sort_by_key(|&(v, _)| v);
    for (v, i) in pairs {
        write!(out, "({v},{i}),").unwrap();
    }
    writeln!(out, "]}},").unwrap();
}

fn emit_bit_lookup_value(out: &mut String, v: &RawBitLookupValue) {
    write!(
        out,
        "BitLookupValue{{bit:{},name:{},id:{}}},",
        v.bit,
        quote(&v.name),
        opt_str(&v.id),
    )
    .unwrap();
}

fn emit_bit_lookup(out: &mut String, t: &RawBitLookup) {
    write!(
        out,
        "BitLookupTable{{name:{},max_value:{},values:&[",
        quote(&t.name),
        opt_int(&t.max_value),
    )
    .unwrap();
    for v in &t.values {
        emit_bit_lookup_value(out, v);
    }
    write!(out, "],by_bit:&[").unwrap();
    let mut pairs: Vec<(u8, u32)> = t
        .values
        .iter()
        .enumerate()
        .map(|(i, v)| (v.bit, i as u32))
        .collect();
    pairs.sort_by_key(|&(b, _)| b);
    for (b, i) in pairs {
        write!(out, "({b},{i}),").unwrap();
    }
    writeln!(out, "]}},").unwrap();
}

fn emit_indirect_lookup_value(out: &mut String, v: &RawIndirectLookupValue) {
    write!(
        out,
        "IndirectLookupValue{{value1:{},value2:{},name:{},id:{}}},",
        v.value1,
        v.value2,
        quote(&v.name),
        opt_str(&v.id),
    )
    .unwrap();
}

fn emit_indirect_lookup(out: &mut String, t: &RawIndirectLookup) {
    write!(
        out,
        "IndirectLookupTable{{name:{},max_value:{},values:&[",
        quote(&t.name),
        opt_int(&t.max_value),
    )
    .unwrap();
    for v in &t.values {
        emit_indirect_lookup_value(out, v);
    }
    write!(out, "],by_pair:&[").unwrap();
    let mut pairs: Vec<((u64, u64), u32)> = t
        .values
        .iter()
        .enumerate()
        .map(|(i, v)| ((v.value1, v.value2), i as u32))
        .collect();
    pairs.sort_by_key(|&(k, _)| k);
    for ((a, b), i) in pairs {
        write!(out, "(({a},{b}),{i}),").unwrap();
    }
    writeln!(out, "]}},").unwrap();
}

fn emit_ft_value(out: &mut String, v: &RawFieldTypeValue, c: ComputedFt) {
    write!(
        out,
        "LookupFieldTypeValue{{value:{value},name:{name},field_type:{field_type},\
         bits:{bits},resolution:{resolution},unit:{unit},\
         lookup_enumeration:{le},lookup_bit_enumeration:{lbe},signed:{signed},precision:{precision}}},",
        value = v.value,
        name = quote(&v.name),
        field_type = opt_str(&v.field_type),
        bits = opt_int(&v.bits.as_ref().and_then(|s| s.parse::<u32>().ok())),
        resolution = opt_float(&v.resolution),
        unit = opt_str(&v.unit),
        le = opt_str(&v.lookup_enumeration),
        lbe = opt_str(&v.lookup_bit_enumeration),
        signed = c.signed,
        precision = c.precision,
    )
    .unwrap();
}

fn emit_ft_lookup(out: &mut String, t: &RawFieldTypeLookup, computed: &[ComputedFt]) {
    write!(
        out,
        "LookupFieldTypeTable{{name:{},max_value:{},values:&[",
        quote(&t.name),
        opt_int(&t.max_value),
    )
    .unwrap();
    for (v, c) in t.values.iter().zip(computed.iter()) {
        emit_ft_value(out, v, *c);
    }
    write!(out, "],by_value:&[").unwrap();
    let mut pairs: Vec<(u64, u32)> = t
        .values
        .iter()
        .enumerate()
        .map(|(i, v)| (v.value, i as u32))
        .collect();
    pairs.sort_by_key(|&(v, _)| v);
    for (v, i) in pairs {
        write!(out, "({v},{i}),").unwrap();
    }
    writeln!(out, "]}},").unwrap();
}

// ---------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    // The schema is read straight from the repository's own generated
    // docs/canboat.json — the single source of truth, produced by keel from
    // database/. This crate used to vendor a copy under data/ with a
    // CANBOAT_REF pin and a weekly sync job; co-locating the two codebases
    // made all of that redundant, and a vendored copy can only ever be stale
    // (MERGE-CANBOAT-RS.md §3 and goal #2).
    //
    // The path reaches outside the package, so `cargo package` / a crates.io
    // publish of this crate would not carry the schema. Nothing is published
    // today; solve that when a publish is actually wanted, most likely by
    // giving keel an emit_rust.rs and committing the generated tables so no
    // build script is needed at all (§5).
    let repo_root = manifest
        .parent()
        .and_then(|p| p.parent())
        .expect("crate is at <repo>/crates/canboat-core");
    let canboat_path = repo_root.join("docs").join("canboat.json");
    // The CANBOAT_BEM pseudo-PGNs (0x40000+) that analyzer/pgn.h defines
    // directly never appear in canboat.json, so this mirror stays for now.
    // keel already owns them in the YAML; folding them in is §7.
    let synthetic_path = manifest.join("data").join("synthetic-pgns.json");

    println!("cargo:rerun-if-changed={}", canboat_path.display());
    println!("cargo:rerun-if-changed={}", synthetic_path.display());

    let canboat_bytes =
        fs::read(&canboat_path).unwrap_or_else(|e| panic!("read {}: {e}", canboat_path.display()));
    let canboat: CanboatJson = serde_json::from_slice(&canboat_bytes)
        .unwrap_or_else(|e| panic!("parse {}: {e}", canboat_path.display()));

    let synthetic_bytes = fs::read(&synthetic_path)
        .unwrap_or_else(|e| panic!("read {}: {e}", synthetic_path.display()));
    let synthetic: SyntheticJson = serde_json::from_slice(&synthetic_bytes)
        .unwrap_or_else(|e| panic!("parse {}: {e}", synthetic_path.display()));

    // Concatenate: main PGNs + synthetic PGNs. Synthetic come last so
    // the in-load-order `fallback_pgn` walk still matches today's
    // behaviour (the synthetic range is at 0x40000+, well above any
    // proprietary catch-all).
    let mut pgns: Vec<RawPgn> = canboat.pgns;
    let canboat_pgn_count = pgns.len();
    pgns.extend(synthetic.pgns);

    // Apply non-SI fix-up to every field, capturing the derived
    // unit_offset / precision / is_dynamic_length_marker.
    // Fields are kept in canboat.json (SI) form; the SI/Metric
    // presentation is computed per-`Units` at emit time by `field_view`.
    let pgn_with_fields: Vec<(RawPgn, Vec<RawField>)> = pgns
        .into_iter()
        .map(|mut p| {
            let fields = std::mem::take(&mut p.fields);
            (p, fields)
        })
        .collect();

    // Field-type lookups also need fix-up.
    let mut ft_tables = canboat.lookup_field_type_enumerations;
    let ft_computed: Vec<Vec<ComputedFt>> = ft_tables
        .iter_mut()
        .map(|t| t.values.iter_mut().map(compute_ft).collect())
        .collect();

    // Sort lookups alphabetically by name for binary-search lookup at
    // runtime.
    let mut lookups = canboat.lookup_enumerations;
    lookups.sort_by(|a, b| a.name.cmp(&b.name));
    let mut bit_lookups = canboat.lookup_bit_enumerations;
    bit_lookups.sort_by(|a, b| a.name.cmp(&b.name));
    let mut indirect_lookups = canboat.lookup_indirect_enumerations;
    indirect_lookups.sort_by(|a, b| a.name.cmp(&b.name));

    // Sort ft_tables (and parallel computed array) by name.
    let mut ft_indexed: Vec<(RawFieldTypeLookup, Vec<ComputedFt>)> =
        ft_tables.into_iter().zip(ft_computed).collect();
    ft_indexed.sort_by(|a, b| a.0.name.cmp(&b.0.name));

    // pgn number -> list of indices (declaration order preserved).
    let mut pgn_index: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (i, (p, _)) in pgn_with_fields.iter().enumerate() {
        pgn_index.entry(p.pgn).or_default().push(i);
    }

    let mut out = String::with_capacity(8 << 20); // 8 MiB initial.
    writeln!(
        out,
        "// AUTO-GENERATED by canboat-core/build.rs — DO NOT EDIT."
    )
    .unwrap();
    writeln!(
        out,
        "// Source: data/canboat.json + data/synthetic-pgns.json"
    )
    .unwrap();
    writeln!(
        out,
        "use canboat_schema::{{BitLookupTable, BitLookupValue, FieldInfo, FieldType, \
         IndirectLookupTable, IndirectLookupValue, LookupFieldTypeTable, LookupFieldTypeValue, \
         LookupTable, LookupValue, PacketType, PgnInfo}};"
    )
    .unwrap();

    writeln!(
        out,
        "pub const SCHEMA_VERSION: &str = {};",
        quote(&canboat.schema_version)
    )
    .unwrap();
    writeln!(
        out,
        "pub const VERSION: &str = {};",
        quote(&canboat.version)
    )
    .unwrap();

    // Content hash over the raw schema source bytes (canboat.json then
    // synthetic-pgns.json). This is the schema identity two processes
    // would exchange to prove they were built from byte-identical schema
    // data before trusting each other's field indices. Any edit to either
    // file — versioned or not —
    // changes this, so a mismatch fails the handshake loudly instead of
    // silently mis-decoding fields. FNV-1a/64: dependency-free, stable,
    // and drift-detection is all it needs to be (not security).
    let schema_hash: u64 = {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for &b in canboat_bytes.iter().chain(synthetic_bytes.iter()) {
            h ^= b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    };
    writeln!(out, "pub const SCHEMA_HASH: u64 = {};", schema_hash).unwrap();

    // The Copyright field is a multi-line banner (version, (C) line,
    // Apache license boilerplate). The (C) line is the one the C
    // tools print in their usage/verbose output — extract just that.
    let copyright_line = canboat
        .copyright
        .lines()
        .map(str::trim)
        .find(|l| l.starts_with("(C)"))
        .expect("no \"(C) ...\" line in canboat.json Copyright field");
    writeln!(
        out,
        "pub const COPYRIGHT_ID: &str = {};",
        quote(copyright_line)
    )
    .unwrap();

    // Field arrays, one (or two) per PGN. `F{i}` is the SI/base slice;
    // a `F{i}M` Metric slice is emitted only when the PGN actually
    // contains a convertible field — otherwise both schemas share `F{i}`.
    // Strings inside the FieldInfo literals dedupe across both via the
    // linker's `.rodata` string merging, so only the ~74 differing PGNs
    // cost extra struct bytes.
    for (i, (_p, fields)) in pgn_with_fields.iter().enumerate() {
        emit_field_array(&mut out, &format!("F{i}"), fields, Units::Si);
        if fields.iter().any(field_converts) {
            emit_field_array(&mut out, &format!("F{i}M"), fields, Units::Metric);
        }
    }

    // PGNS_SI / PGNS_METRIC. Entries past `canboat_pgn_count` come from
    // synthetic-pgns.json — canboat C doesn't know about those, so their
    // Ids shouldn't trigger the camelDescription wrapping even when they
    // happen to differ from camelize(description). The two arrays differ
    // only in which field slice each PgnInfo points at.
    writeln!(out, "pub static PGNS_SI: &[PgnInfo] = &[").unwrap();
    for (i, (p, _fields)) in pgn_with_fields.iter().enumerate() {
        let from_synthetic = i >= canboat_pgn_count;
        emit_pgn(&mut out, p, &format!("F{i}"), from_synthetic);
    }
    writeln!(out, "];").unwrap();

    writeln!(out, "pub static PGNS_METRIC: &[PgnInfo] = &[").unwrap();
    for (i, (p, fields)) in pgn_with_fields.iter().enumerate() {
        let from_synthetic = i >= canboat_pgn_count;
        let ident = if fields.iter().any(field_converts) {
            format!("F{i}M")
        } else {
            format!("F{i}")
        };
        emit_pgn(&mut out, p, &ident, from_synthetic);
    }
    writeln!(out, "];").unwrap();

    // Id-keyed constant references into the arrays above, so code can say
    // `pgn::WIND_DATA` / `field::wind_data::WIND_ANGLE` instead of the
    // stringly-typed `("windData","windAngle")` pair. They index the SI
    // arrays (`PGNS_SI` / `F{i}`); id/name/order are unit-invariant.
    emit_id_constants(&mut out, &pgn_with_fields);

    // PGN_INDEX — sorted by pgn number, value is &[u32] of indices.
    writeln!(out, "pub static PGN_INDEX: &[(u32, &[u32])] = &[").unwrap();
    for (pgn, idxs) in &pgn_index {
        write!(out, "({},&[", pgn).unwrap();
        for i in idxs {
            write!(out, "{},", i).unwrap();
        }
        writeln!(out, "]),").unwrap();
    }
    writeln!(out, "];").unwrap();

    // LOOKUPS — sorted by name.
    writeln!(out, "pub static LOOKUPS: &[LookupTable] = &[").unwrap();
    for t in &lookups {
        emit_lookup(&mut out, t);
    }
    writeln!(out, "];").unwrap();

    // BIT_LOOKUPS.
    writeln!(out, "pub static BIT_LOOKUPS: &[BitLookupTable] = &[").unwrap();
    for t in &bit_lookups {
        emit_bit_lookup(&mut out, t);
    }
    writeln!(out, "];").unwrap();

    // INDIRECT_LOOKUPS.
    writeln!(
        out,
        "pub static INDIRECT_LOOKUPS: &[IndirectLookupTable] = &["
    )
    .unwrap();
    for t in &indirect_lookups {
        emit_indirect_lookup(&mut out, t);
    }
    writeln!(out, "];").unwrap();

    // FIELD_TYPE_LOOKUPS.
    writeln!(
        out,
        "pub static FIELD_TYPE_LOOKUPS: &[LookupFieldTypeTable] = &["
    )
    .unwrap();
    for (t, c) in &ft_indexed {
        emit_ft_lookup(&mut out, t, c);
    }
    writeln!(out, "];").unwrap();

    // --- Phase 3 codegen: per-PGN dispatch on Match fields. ---
    //
    // For each PGN number we know about, emit either:
    //   * `pub fn dispatch_<pgn>(payload: &[u8]) -> Option<usize>`
    //     when at least one variant carries Match fields, or
    //   * nothing (the top-level `dispatch` returns the single variant
    //     directly).
    //
    // The per-PGN function extracts every unique (offset, length, signed,
    // offset_k) tuple used by any variant's Match fields ONCE at the top,
    // then checks each Match-having variant in JSON declaration order
    // against the extracted values. The no-Match variant (if any) is
    // returned as the in-PGN fallback at the bottom.
    //
    // What this saves vs. the old `pick_variant` linear scan:
    //   * PGN 130820's 50 variants share the same two leading match
    //     fields (Manufacturer Code, Industry Code). Old code called
    //     extract_bits up to 100 times per frame; new code calls it
    //     exactly 2 times and then runs a tight chain of integer
    //     comparisons the compiler can lower to a jump table.
    let mut multi: Vec<(u32, Vec<VariantEntry<'_>>)> = Vec::new();
    for (pgn_num, idxs) in &pgn_index {
        let entries: Vec<_> = idxs
            .iter()
            .map(|&i| {
                let (p, fs) = &pgn_with_fields[i];
                (i, p, fs)
            })
            .collect();
        multi.push((*pgn_num, entries));
    }

    for (pgn_num, variants) in &multi {
        if !needs_dispatch_fn(variants) {
            continue;
        }
        emit_per_pgn_dispatch(&mut out, *pgn_num, variants);
    }

    // Top-level dispatch entry point.
    writeln!(
        out,
        "/// Pick the right PgnInfo variant for `pgn` given the frame payload."
    )
    .unwrap();
    writeln!(
        out,
        "pub fn dispatch(pgn: u32, payload: &[u8]) -> Option<usize> {{"
    )
    .unwrap();
    writeln!(out, "    match pgn {{").unwrap();
    for (pgn_num, variants) in &multi {
        if needs_dispatch_fn(variants) {
            writeln!(out, "        {pgn_num} => dispatch_{pgn_num}(payload),").unwrap();
        } else {
            // Single variant, no Match fields — direct return.
            let idx = variants[0].0;
            writeln!(out, "        {pgn_num} => Some({idx}),").unwrap();
        }
    }
    writeln!(out, "        _ => None,").unwrap();
    writeln!(out, "    }}").unwrap();
    writeln!(out, "}}").unwrap();

    // Sparse cross-PGN fallback table: `(pgn_number, idx_into_PGNS)`
    // for every Fallback:true entry, in declaration (== ascending PGN)
    // order. `find_catchall` is O(log n) via binary search instead of
    // the previous 600-entry linear scan.
    let fallbacks: Vec<(u32, usize)> = pgn_with_fields
        .iter()
        .enumerate()
        .filter_map(|(i, (p, _))| {
            if p.fallback == Some(true) {
                Some((p.pgn, i))
            } else {
                None
            }
        })
        .collect();
    writeln!(out, "pub static FALLBACKS: &[(u32, u32)] = &[").unwrap();
    for (pgn, idx) in &fallbacks {
        writeln!(out, "    ({pgn}, {idx}),").unwrap();
    }
    writeln!(out, "];").unwrap();
    writeln!(
        out,
        "/// Largest Fallback:true PgnInfo whose pgn number is <= `pgn`.\n\
         /// Mirrors canboat's `searchForUnknownPgn`."
    )
    .unwrap();
    writeln!(out, "pub fn find_catchall(pgn: u32) -> Option<usize> {{").unwrap();
    writeln!(
        out,
        "    let i = FALLBACKS.partition_point(|&(p, _)| p <= pgn);"
    )
    .unwrap();
    writeln!(
        out,
        "    if i == 0 {{ None }} else {{ Some(FALLBACKS[i - 1].1 as usize) }}"
    )
    .unwrap();
    writeln!(out, "}}").unwrap();

    let out_dir = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR"));
    let out_path = out_dir.join("schema_generated.rs");
    fs::write(&out_path, &out).unwrap_or_else(|e| panic!("write {}: {e}", out_path.display()));
}

/// `true` iff this PGN number needs a generated dispatch function —
/// it has either (a) more than one variant or (b) a single variant
/// with Match fields that gate decoding.
fn needs_dispatch_fn(variants: &[VariantEntry<'_>]) -> bool {
    if variants.len() > 1 {
        return true;
    }
    variants[0].2.iter().any(|f| f.match_value.is_some())
}

/// Emit `fn dispatch_<pgn>(payload: &[u8]) -> Option<usize>`.
fn emit_per_pgn_dispatch(out: &mut String, pgn_num: u32, variants: &[VariantEntry<'_>]) {
    use std::collections::BTreeSet;

    // Collect every distinct (offset, length, signed, offset_k) used by
    // a Match field anywhere in this PGN. We extract each once at the
    // top of the function and bind it to a stable local name so the
    // subsequent per-variant arms turn into integer comparisons.
    let mut ranges: BTreeSet<(u32, u32, bool, i64)> = BTreeSet::new();
    for (_, _, fields) in variants {
        for f in *fields {
            if f.match_value.is_none() {
                continue;
            }
            let (Some(off), Some(len)) = (f.bit_offset, f.bit_length) else {
                continue;
            };
            ranges.insert((off, len, f.signed.unwrap_or(false), f.offset.unwrap_or(0)));
        }
    }

    let var_name = |off: u32, len: u32, signed: bool, off_k: i64| -> String {
        let s = if signed { "s" } else { "u" };
        let k = if off_k < 0 {
            format!("n{}", -off_k)
        } else {
            format!("{off_k}")
        };
        format!("m_{off}_{len}_{s}_{k}")
    };

    writeln!(
        out,
        "fn dispatch_{pgn_num}(payload: &[u8]) -> Option<usize> {{"
    )
    .unwrap();
    // `let _ = payload;` keeps the parameter named (clearer at call sites)
    // even on the rare PGN whose variants carry no usable Match fields,
    // where no extract_bits call would otherwise reference it.
    writeln!(out, "    let _ = payload;").unwrap();
    for (off, len, signed, off_k) in &ranges {
        let name = var_name(*off, *len, *signed, *off_k);
        writeln!(
            out,
            "    let {name} = crate::bits::extract_bits(payload, {off}, {len}, {signed}, {off_k}).map(|e| e.value);"
        )
        .unwrap();
    }

    // First pass: Match-having variants in JSON declaration order
    // (emitted as `if … { return … }` arms below).
    //
    // Second pass: pick the in-PGN no-Match "fallback" variant that
    // runs when nothing matched. A `Fallback: true` variant is a
    // *catch-all* for the whole PGN-range bracket — it should only
    // win when no other no-Match variant for this PGN exists.
    // Otherwise the first non-Fallback no-Match variant in JSON
    // order wins.
    //
    // Concrete case: PGN 59392 has the range catch-all
    // (`Fallback: true`) listed first and `isoAcknowledgement`
    // (no Match, no Fallback) second. The previous logic preferred
    // `Fallback: true` and decoded every ISO Ack as
    // "0xE800-0xEE00: Standardized single-frame addressed" — a
    // NAC3 autopilot's PGN 59392 reply surfaces correctly as ISO
    // Acknowledgement only with this preference flipped.
    let mut specific_no_match: Option<usize> = None;
    let mut catchall_no_match: Option<usize> = None;
    for (idx, p, fields) in variants {
        let match_fields: Vec<&RawField> =
            fields.iter().filter(|f| f.match_value.is_some()).collect();
        if match_fields.is_empty() {
            if p.fallback == Some(true) {
                if catchall_no_match.is_none() {
                    catchall_no_match = Some(*idx);
                }
            } else if specific_no_match.is_none() {
                specific_no_match = Some(*idx);
            }
            continue;
        }
        write!(out, "    if ").unwrap();
        let mut first = true;
        let mut emitted_anything = false;
        for f in &match_fields {
            let (Some(off), Some(len)) = (f.bit_offset, f.bit_length) else {
                if !first {
                    write!(out, " && ").unwrap();
                }
                write!(out, "false").unwrap();
                first = false;
                emitted_anything = true;
                continue;
            };
            if !first {
                write!(out, " && ").unwrap();
            }
            let name = var_name(off, len, f.signed.unwrap_or(false), f.offset.unwrap_or(0));
            write!(out, "{name} == Some({})", f.match_value.unwrap()).unwrap();
            first = false;
            emitted_anything = true;
        }
        if !emitted_anything {
            // Pathological: variant claimed Match but had no usable fields. Skip.
            writeln!(out, "false {{}}").unwrap();
            continue;
        }
        writeln!(out, " {{ return Some({idx}); }}").unwrap();
    }
    match specific_no_match.or(catchall_no_match) {
        Some(idx) => writeln!(out, "    Some({idx})").unwrap(),
        None => writeln!(out, "    None").unwrap(),
    }
    writeln!(out, "}}").unwrap();
}
