// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Emit the Rust schema tables that `canboat-core` compiles in.
//!
//! This was `crates/canboat-core/build.rs` until the crates became
//! publishable: a build script cannot reach `database/`, which lives above
//! the package root, so the tables are generated here and committed instead.
//! The code is otherwise a straight move — the derivation it performs (the
//! non-SI unit fix-up, sentinel resolution, precision) decides how every
//! field decodes, so it was not rewritten in passing.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------
// Build-script deserialization types.
//
// These mirror canboat.json's PascalCase shape. They are PRIVATE to the
// build script and not exposed anywhere else — runtime types live in
// `canboat-schema`. The mapping from these owned types to the const
// form happens in the emit_* functions below.
struct CanboatJson {
    schema_version: String,
    version: String,
    copyright: String,
    pgns: Vec<RawPgn>,
    lookup_enumerations: Vec<RawLookup>,
    lookup_bit_enumerations: Vec<RawBitLookup>,
    lookup_indirect_enumerations: Vec<RawIndirectLookup>,
    lookup_field_type_enumerations: Vec<RawFieldTypeLookup>,
}

#[derive(Clone)]
struct RawPgn {
    pgn: u32,
    id: String,
    description: String,
    explanation: Option<String>,
    url: Option<String>,
    packet_type: String,
    complete: Option<bool>,
    fallback: Option<bool>,
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

#[derive(Clone)]
struct RawField {
    order: u8,
    id: String,
    name: String,
    description: Option<String>,
    bit_length: Option<u32>,
    bit_length_field: Option<String>,
    bit_length_variable: Option<bool>,
    bit_offset: Option<u32>,
    bit_start: Option<u32>,
    resolution: Option<f64>,
    signed: Option<bool>,
    offset: Option<i64>,
    range_min: Option<f64>,
    range_max: Option<f64>,
    unknown_value: Option<u64>,
    out_of_range_value: Option<u64>,
    reserved_value: Option<u64>,
    unit: Option<String>,
    physical_quantity: Option<String>,
    field_type: Option<String>,
    lookup_enumeration: Option<String>,
    lookup_bit_enumeration: Option<String>,
    lookup_indirect_enumeration: Option<String>,
    lookup_indirect_enumeration_field_order: Option<u8>,
    lookup_field_type_enumeration: Option<String>,
    match_value: Option<i64>,
    part_of_primary_key: Option<bool>,
    condition: Option<String>,
    /// Bytes of per-record header that sit between a
    /// DYNAMIC_FIELD_LENGTH field and the value it sizes, and that the
    /// reported length counts. Subtracted to get the value's own width.
    dynamic_field_length_overhead: u32,
}

struct RawLookup {
    name: String,
    max_value: Option<u64>,
    values: Vec<RawLookupValue>,
}
struct RawLookupValue {
    value: u64,
    name: String,
    id: Option<String>,
}

struct RawBitLookup {
    name: String,
    max_value: Option<u64>,
    values: Vec<RawBitLookupValue>,
}
struct RawBitLookupValue {
    bit: u8,
    name: String,
    id: Option<String>,
}

struct RawIndirectLookup {
    name: String,
    max_value: Option<u64>,
    values: Vec<RawIndirectLookupValue>,
}
struct RawIndirectLookupValue {
    value1: u64,
    value2: u64,
    name: String,
    id: Option<String>,
}

struct RawFieldTypeLookup {
    name: String,
    max_value: Option<u64>,
    values: Vec<RawFieldTypeValue>,
}
#[derive(Clone)]
struct RawFieldTypeValue {
    value: u64,
    name: String,
    field_type: Option<String>,
    bits: Option<String>,
    resolution: Option<f64>,
    unit: Option<String>,
    lookup_enumeration: Option<String>,
    lookup_bit_enumeration: Option<String>,
    /// Signedness of the fieldtype this entry resolves to — e.g.
    /// `CURRENT_FIX32_MA` for VICTRON_VREG's "DC Current". Carried
    /// explicitly because `field_type` above is the *root* name
    /// (`NUMBER`), which says nothing about the sign.
    signed: bool,
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
    let mut c = ComputedFt {
        // The fieldtype's own signedness is the answer. The unit test
        // below only ever *adds* to it, for angle types that canboat
        // spells FIX under the hood.
        signed: v.signed,
        ..ComputedFt::default()
    };
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
         unit_offset:{unit_offset},precision:{precision},is_dynamic_length_marker:{dlm},\
         dynamic_field_length_overhead:{dflo}}},",
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
        dflo = f.dynamic_field_length_overhead,
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
fn emit_pgn(out: &mut String, p: &RawPgn, fields_ident: &str, is_bem: bool) {
    // Synthetic PGNs (post-canboat.json list) never wrap — they're
    // not in canboat C's PGN list, so the camelDescription contract
    // doesn't apply. Otherwise: pinned iff the natural camelize of
    // the description doesn't yield the declared Id.
    let id_is_pinned = !is_bem && camelize_lower(&p.description) != p.id;
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
// keel -> Raw*: build the schema straight from database/*.yaml
// ---------------------------------------------------------------------
//
// keel already stores, or derives, everything the emitter below needs. The
// only two things it computes at *emission* time rather than storing are the
// running bit offset and the sentinel triple, both reproduced here exactly as
// emit_xml does (which is what canboat.json carried).

fn ft_of<'a>(
    db: &'a crate::model::Database,
    f: &crate::model::Field,
) -> &'a crate::model::FieldType {
    &db.fieldtypes[f.ft]
}

/// Sentinels, as emit_xml::field() emits them: only for a TopOfRange root with
/// a real range, under 64 bits, and never on a match field.
fn sentinels(
    db: &crate::model::Database,
    f: &crate::model::Field,
) -> (Option<u64>, Option<u64>, Option<u64>) {
    let ft = ft_of(db, f);
    if f.reserved_count == 0
        || f.res_range_min.is_nan()
        || f.match_.is_some()
        || ft.root_sentinels != "TopOfRange"
    {
        return (None, None, None);
    }
    // NB: emit_xml additionally suppresses these for 64-bit fields, and the
    // XML/JSON therefore carry no UnknownValue for e.g. 129029's latitude
    // (QUIRKS Q18). That is right for the *document* but wrong for a runtime
    // schema: the decoder has nothing left to compare against, so an
    // unavailable 64-bit lat/lon decoded as the number 922.3372037 instead of
    // being reported unavailable. The C never had the problem because it
    // recomputes the bound from the bit width at decode time. So: no width
    // guard here.
    let highbit = if ft.has_sign == Some(true) && f.res_offset == 0 {
        f.res_bits - 1
    } else {
        f.res_bits
    };
    // highbit == 64 for an unsigned 64-bit field, where 1u64 << 64 is UB.
    let raw = if highbit >= 64 {
        u64::MAX
    } else {
        (1u64 << highbit) - 1
    };
    (
        Some(raw),
        (f.reserved_count >= 2).then(|| raw - 1),
        (f.reserved_count >= 3).then(|| raw - 2),
    )
}

fn raw_field(
    db: &crate::model::Database,
    f: &crate::model::Field,
    bit_offset: u32,
    show_offset: bool,
) -> RawField {
    let ft = ft_of(db, f);
    let (unknown, oor, reserved) = sentinels(db, f);
    let lookup_ref = f.lookup_ref();
    let kind = |k: &str| lookup_ref.and_then(|(t, n)| (t == k).then(|| n.to_string()));
    RawField {
        order: f.order as u8,
        id: f.id.clone(),
        name: f.name.clone(),
        description: f.description.clone(),
        bit_length: (f.res_bits != 0).then_some(f.res_bits),
        bit_length_field: f.bit_length_field_order.map(|o| o.to_string()),
        bit_length_variable: (f.res_bits == 0).then_some(true),
        bit_offset: show_offset.then_some(bit_offset),
        bit_start: show_offset.then_some(bit_offset % 8),
        // canboat.json carries Resolution even when it is 1 (the XSLT
        // defaults it); only a genuinely unset resolution is absent.
        resolution: (f.res_resolution != 0.0).then_some(f.res_resolution),
        signed: ft.has_sign,
        // Scaled, as the XML/JSON carry it: the model keeps the offset in raw
        // units (PEUKERT_EXPONENT is 500) but the published Offset is in the
        // field's own units (500 * 0.002 = 1), and the decoder adds it after
        // scaling. Passing the raw value made Peukert read 500.002.
        offset: Some((f.res_offset as f64 * f.res_resolution) as i64).filter(|o| *o != 0),
        // Ranges follow emit_xml exactly, including its two special cases:
        // a non-match lookup field with a NaN range still reports 0 ..
        // 2^bits-1 (QUIRKS Q16), and an unsigned 64-bit resolution-1 field
        // reports the integer u64::MAX rather than a rounded float (Q7).
        range_min: if !f.res_range_min.is_nan() {
            Some(f.res_range_min)
        } else if lookup_ref.is_some() && f.match_.is_none() {
            Some(0.0)
        } else {
            None
        },
        range_max: if !f.res_range_max.is_nan() {
            if f.res_resolution == 1.0
                && f.res_bits == 64
                && ft.has_sign == Some(false)
                && f.res_offset == 0
            {
                Some(u64::MAX as f64)
            } else {
                Some(f.res_range_max)
            }
        } else if lookup_ref.is_some() && f.match_.is_none() {
            Some(((1u128 << f.res_bits) - 1) as f64)
        } else {
            None
        },
        unknown_value: unknown,
        out_of_range_value: oor,
        reserved_value: reserved,
        unit: f.res_unit.clone(),
        physical_quantity: ft.physical.clone(),
        field_type: Some(ft.root_name.clone()),
        lookup_enumeration: kind("pair"),
        lookup_bit_enumeration: kind("bit"),
        lookup_indirect_enumeration: kind("triplet"),
        lookup_indirect_enumeration_field_order: f.lookup_indirect_order.map(|o| o as u8),
        lookup_field_type_enumeration: kind("fieldtype"),
        match_value: db.resolve_match(f).map(|v| v as i64),
        part_of_primary_key: f.primary_key.then_some(true),
        condition: f.proprietary.then(|| "PGNIsProprietary".to_string()),
        dynamic_field_length_overhead: f.dynamic_field_length_overhead,
    }
}

fn from_keel(db: &crate::model::Database) -> CanboatJson {
    let mut pgns = Vec::new();
    for p in &db.pgns {
        // emit_xml's running bit offset: it stops being meaningful once a
        // variable-length field has been seen.
        let mut bit_offset = 0u32;
        let mut show = true;
        let mut fields = Vec::with_capacity(p.fields.len());
        for f in &p.fields {
            fields.push(raw_field(db, f, bit_offset, show));
            bit_offset += f.res_bits;
            // Same kill switch as emit_xml: once a field's position stops
            // being statically knowable, no later field reports an offset.
            if db.fieldtypes[f.ft].variable_size || f.proprietary || f.res_bits == 0 {
                show = false;
            }
        }
        let rep = |r: &Option<crate::model::Repeating>| {
            r.as_ref().map(|r| (r.count, r.start, r.count_field))
        };
        let (r1, r2) = (rep(&p.repeating1), rep(&p.repeating2));
        pgns.push(RawPgn {
            pgn: p.pgn,
            id: p.id.clone(),
            description: p.description.clone(),
            explanation: p.explanation.clone(),
            url: p.url.clone(),
            packet_type: p.type_.clone(),
            complete: Some(p.is_complete()),
            fallback: p.fallback.then_some(true),
            missing: {
                let m: Vec<String> = p
                    .missing_effective()
                    .iter()
                    .map(|s| s.to_string())
                    .collect();
                (!m.is_empty()).then_some(m)
            },
            priority: (p.priority != 0).then_some(p.priority as u8),
            transmission_interval: match p.interval {
                crate::model::Interval::Ms(ms) => Some(ms),
                _ => None,
            },
            transmission_irregular: matches!(p.interval, crate::model::Interval::Irregular)
                .then_some(true),
            field_count: p.field_count,
            // A variable-length PGN (one with a repeating set) publishes
            // MinLength and no Length -- its real length depends on the
            // repeat count. Emitting Length for those made the runtime treat
            // 43 as fixed and reject a legitimate 47-byte 129029.
            length: (!p.is_variable).then_some(p.length),
            min_length: p.is_variable.then_some(p.length),
            fields,
            repeating_field_set1_size: r1.map(|r| r.0),
            repeating_field_set1_start_field: r1.map(|r| r.1),
            repeating_field_set1_count_field: r1.and_then(|r| r.2),
            repeating_field_set2_size: r2.map(|r| r.0),
            repeating_field_set2_start_field: r2.map(|r| r.1),
            repeating_field_set2_count_field: r2.and_then(|r| r.2),
        });
    }

    let of_kind = |k: &str| -> Vec<&crate::model::Lookup> { db.ordered_lookups(k) };
    CanboatJson {
        schema_version: db.schema_version.clone(),
        version: db.version.clone(),
        copyright: crate::emit_xml::copyright(&db.version),
        pgns,
        lookup_enumerations: of_kind("pair")
            .iter()
            .map(|l| RawLookup {
                name: l.name.clone(),
                max_value: Some((1u64 << l.bits) - 1),
                values: l
                    .pairs
                    .iter()
                    .map(|(v, n)| RawLookupValue {
                        value: *v,
                        name: n.clone(),
                        id: None,
                    })
                    .collect(),
            })
            .collect(),
        lookup_bit_enumerations: of_kind("bit")
            .iter()
            .map(|l| RawBitLookup {
                name: l.name.clone(),
                max_value: Some(l.bits as u64 - 1),
                values: l
                    .pairs
                    .iter()
                    .map(|(b, n)| RawBitLookupValue {
                        bit: *b as u8,
                        name: n.clone(),
                        id: None,
                    })
                    .collect(),
            })
            .collect(),
        lookup_indirect_enumerations: of_kind("triplet")
            .iter()
            .map(|l| RawIndirectLookup {
                name: l.name.clone(),
                max_value: Some((1u64 << l.bits) - 1),
                values: l
                    .triplets
                    .iter()
                    .map(|(a, b, n)| RawIndirectLookupValue {
                        value1: *a,
                        value2: *b,
                        name: n.clone(),
                        id: None,
                    })
                    .collect(),
            })
            .collect(),
        lookup_field_type_enumerations: of_kind("fieldtype")
            .iter()
            .map(|l| RawFieldTypeLookup {
                name: l.name.clone(),
                max_value: Some((1u64 << l.bits) - 1),
                values: l
                    .fieldtypes
                    .iter()
                    .map(|e| {
                        let ftv = db.fieldtype(&e.fieldtype).ok().map(|i| &db.fieldtypes[i]);
                        RawFieldTypeValue {
                            value: e.value,
                            name: e.name.clone(),
                            field_type: ftv.map(|f| f.root_name.clone()),
                            bits: e
                                .bits
                                .or_else(|| ftv.map(|f| f.size))
                                .filter(|b| *b != 0)
                                .map(|b| b.to_string()),
                            resolution: ftv.map(|f| f.resolution).filter(|r| *r != 0.0),
                            unit: ftv.and_then(|f| f.unit.clone()),
                            signed: ftv.and_then(|f| f.has_sign).unwrap_or(false),
                            lookup_enumeration: e
                                .lookup
                                .clone()
                                .filter(|_| e.lookup_kind.as_deref() != Some("bit")),
                            lookup_bit_enumeration: e
                                .lookup
                                .clone()
                                .filter(|_| e.lookup_kind.as_deref() == Some("bit")),
                        }
                    })
                    .collect(),
            })
            .collect(),
    }
}

// ---------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------

/// Render the whole `schema_generated.rs` body.
///
/// `root` is the repository root: the SCHEMA_HASH below is computed by
/// walking the authored `database/*.yaml` on disk, so the emitter needs to
/// know where they are.
pub fn emit_schema(db: &crate::model::Database, root: &Path) -> String {
    let canboat = from_keel(db);

    // The CANBOAT_BEM pseudo-PGNs (0x40000+) are ordinary members of the
    // database now, and keel hands them over sorted by PGN — so they still
    // land last, which is what the in-load-order `fallback_pgn` walk wants
    // (the BEM range is well above any proprietary catch-all).
    let pgns: Vec<RawPgn> = canboat.pgns;
    let canboat_pgn_count = pgns.iter().filter(|p| p.pgn < 0x40000).count();

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
        "\
// ==========================================================================\n\
//\n\
//   GENERATED FILE - DO NOT EDIT.\n\
//\n\
//   Written by canboat-core/build.rs from the YAML database in database/,\n\
//   via keel. It is regenerated on every build, so an edit here is lost the\n\
//   moment you run cargo.\n\
//\n\
//   To change a PGN, a lookup or a field type, edit the YAML:\n\
//\n\
//       database/pgns/<pgn>-<id>.yaml     one file per PGN variant\n\
//       database/lookups/<NAME>.yaml      one file per enumeration\n\
//       database/fieldtypes.yaml          the field-type hierarchy\n\
//\n\
//   `cargo build` picks the change up on its own; `make generated` also\n\
//   refreshes the C tables and canboat.xml.\n\
//\n\
// =========================================================================="
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

    // Content hash over the authored schema source (database/**.yaml).
    // This is the schema identity two processes
    // would exchange to prove they were built from byte-identical schema
    // data before trusting each other's field indices. Any edit to the
    // database — versioned or not — changes this, so a mismatch fails the handshake loudly instead of
    // silently mis-decoding fields. FNV-1a/64: dependency-free, stable,
    // and drift-detection is all it needs to be (not security).
    let schema_hash: u64 = {
        // Hash every authored database file, path included, in a stable
        // (sorted) order. Path as well as content, so a rename alone still
        // moves the hash.
        let mut files: Vec<PathBuf> = Vec::new();
        let mut stack = vec![root.join("database")];
        while let Some(dir) = stack.pop() {
            let Ok(rd) = fs::read_dir(&dir) else { continue };
            for e in rd.flatten() {
                let p = e.path();
                if p.is_dir() {
                    stack.push(p);
                } else if p.extension().is_some_and(|x| x == "yaml") {
                    files.push(p);
                }
            }
        }
        files.sort();
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        let feed = |bytes: &[u8], h: &mut u64| {
            for &b in bytes {
                *h ^= b as u64;
                *h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        };
        for f in &files {
            feed(
                f.strip_prefix(root)
                    .unwrap_or(f)
                    .to_string_lossy()
                    .as_bytes(),
                &mut h,
            );
            feed(&fs::read(f).unwrap_or_default(), &mut h);
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
    // the CANBOAT_BEM range — the C analyzer has these too, but their
    // Ids shouldn't trigger the camelDescription wrapping even when they
    // happen to differ from camelize(description). The two arrays differ
    // only in which field slice each PgnInfo points at.
    writeln!(out, "pub static PGNS_SI: &[PgnInfo] = &[").unwrap();
    for (i, (p, _fields)) in pgn_with_fields.iter().enumerate() {
        let is_bem = i >= canboat_pgn_count;
        emit_pgn(&mut out, p, &format!("F{i}"), is_bem);
    }
    writeln!(out, "];").unwrap();

    writeln!(out, "pub static PGNS_METRIC: &[PgnInfo] = &[").unwrap();
    for (i, (p, fields)) in pgn_with_fields.iter().enumerate() {
        let is_bem = i >= canboat_pgn_count;
        let ident = if fields.iter().any(field_converts) {
            format!("F{i}M")
        } else {
            format!("F{i}")
        };
        emit_pgn(&mut out, p, &ident, is_bem);
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

    out
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

// ---------------------------------------------------------------------
// canboat-io: the fast-packet lookup table
// ---------------------------------------------------------------------

const MIXED_START: u32 = 0x1F000;
const MIXED_END: u32 = 0x20000; // exclusive; table covers 0x1F000..0x1FFFF
const PROPRIETARY_START: u32 = 0x1FF00;

/// Render `fastpacket_generated.rs` for `canboat-io`.
///
/// PGNs 0x1F000..0x1FFFF are the one range where single-frame and
/// fast-packet messages are interleaved, so a receiver cannot tell them
/// apart from the CAN id alone and needs this table.
///
/// Was `canboat-io/build.rs`, which derived the same table from
/// `docs/canboat.json`. Reading the model directly removes the last
/// out-of-package file read and drops the `serde_json` build dependency.
pub fn emit_fastpacket(db: &crate::model::Database) -> String {
    let size = (MIXED_END - MIXED_START) as usize;
    // Default: the manufacturer-proprietary tail is fast-packet, the
    // standardized part defaults to single frame; explicit PGN
    // definitions override below.
    let mut fast = vec![false; size];
    for (i, slot) in fast.iter_mut().enumerate() {
        *slot = MIXED_START + i as u32 >= PROPRIETARY_START;
    }

    for p in &db.pgns {
        if !(MIXED_START..MIXED_END).contains(&p.pgn) {
            continue;
        }
        if p.fallback {
            continue; // range fallbacks, not a real PGN
        }
        fast[(p.pgn - MIXED_START) as usize] = p.type_ == "Fast";
    }

    let mut out = String::new();
    out.push_str(&format!(
        "pub const FASTPACKET_MIXED_START: u32 = 0x{MIXED_START:05X};\n"
    ));
    out.push_str(&format!(
        "pub const FASTPACKET_MIXED_END: u32 = 0x{MIXED_END:05X};\n"
    ));
    out.push_str(&format!(
        "/// 1 = fast-packet, 0 = single-frame, for PGNs 0x{:05X}..0x{:05X}.\n",
        MIXED_START,
        MIXED_END - 1
    ));
    out.push_str(&format!(
        "pub static FASTPACKET_MIXED: [bool; 0x{size:X}] = ["
    ));
    for (i, v) in fast.iter().enumerate() {
        if i % 32 == 0 {
            out.push_str("\n    ");
        }
        out.push_str(if *v { "true," } else { "false," });
    }
    out.push_str("\n];\n");
    out
}
