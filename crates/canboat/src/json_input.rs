// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Analyzer JSON records → wire frames: the driver behind `canboat
//! convert --from json`, the gateway bridges' stdin TX, and the wasm
//! bindings.
//!
//! The input is line-delimited analyzer JSON, one record per line, in
//! either shape the analyzer emits: bare `-json` (unwrapped, fields
//! keyed by human name) or `-camel` (wrapped `{"<pgnId>":{…}}`, fields
//! keyed by camelCase id). The camel wrapper id names the exact PGN
//! variant and is the recommended form — bare records resolve by PGN
//! number alone and are rejected when the number is ambiguous (e.g. the
//! 126208 group functions).
//!
//! The `-nv` flavour (`{"value":N,"name":"…"}` per lookup/measured
//! field) is the lossless interchange form: the raw `value` is written
//! back verbatim. Plain scalars are accepted for numeric and lookup
//! fields, and strings for string / lookup-label / hex-binary fields;
//! date/time/duration display strings are NOT parsed back — feed `-nv`
//! JSON for those.
//!
//! Repeating sets arrive exactly as the analyzer emits them: `"list"`
//! (set 1) and `"list2"` (set 2) arrays inside `fields`, one object per
//! iteration.
//!
//! Bare physical values are read against the unit system the stream's
//! `{"version":…,"units":…}` banner declares — degrees in a `"std"`
//! stream, radians in an `"si"` one. Without a banner the caller's
//! assumption stands (`--units`, SI by default). `-nv` raw values are
//! unit-agnostic and unaffected.
//!
//! This lives in the `canboat` crate behind the `json-input` feature
//! rather than in the engine because it is the one place a real
//! JSON parser is warranted — nested repeating lists and `-nv` objects
//! are beyond `analyzer_json`'s deliberate substring-scan minimalism —
//! and `serde_json` has no business in the core's dependency closure.
//! The `cli` feature folds it in, so the binary always has it.

use std::io::BufRead;

use crate::engine::source::FrameSource;
use crate::engine::types::{FieldInfo, FieldType};
use crate::engine::{BusProtocol, EncodeValue, PgnBuilder, PgnDatabase, RawFrame};
use anyhow::{Context, Result, anyhow, bail};
use serde_json::{Map, Value};

/// Top-level keys of a bare (non-camel) analyzer record. A single-key
/// object whose key is none of these is a `-camel` envelope and the key
/// is the PGN variant id.
const BARE_TOP_LEVEL: [&str; 7] = [
    "timestamp",
    "prio",
    "src",
    "dst",
    "pgn",
    "description",
    "fields",
];

/// A [`FrameSource`] that reads analyzer-JSON lines from `src` and
/// yields the re-encoded frames. Lines that fail to encode are skipped
/// with a warning (mirroring the analyzer's tolerance of undecodable
/// input).
///
/// The stream's `{"version":…,"units":…}` banner is not merely skipped:
/// its `units` decides how bare physical values in the following
/// records are read back. A `43.0` in a `"units":"std"` stream is
/// degrees; the same number in an SI stream is radians, and encoding it
/// against the wrong schema puts the wrong bits on the wire. The
/// constructor's `db` is only the assumption for a bannerless stream —
/// the banner always wins.
///
/// Its `protocol` is checked rather than adopted: the same PGN number
/// means different things on NMEA 2000 and J1939, so a J1939 stream
/// read against the NMEA 2000 table (or the other way round) would
/// encode garbage. A mismatch is an error naming the `--protocol` to
/// use. A banner without `protocol` (canboat C, older Rust) is NMEA
/// 2000.
pub struct JsonFrameReader<R> {
    src: R,
    db: &'static PgnDatabase,
    buf: String,
    line_no: usize,
}

impl<R: BufRead> JsonFrameReader<R> {
    /// Read analyzer-JSON lines from `src`, encoding against `db` —
    /// which supplies the unit system to assume until (and unless) the
    /// stream declares its own in a banner.
    pub fn new(src: R, db: &'static PgnDatabase) -> Self {
        Self {
            src,
            db,
            buf: String::with_capacity(512),
            line_no: 0,
        }
    }
}

/// What an analyzer banner declares.
#[derive(Debug, PartialEq, Eq)]
struct Banner {
    /// `"si"` (strict SI) or `"std"` (canboat's practical Metric);
    /// `None` when the banner doesn't say.
    units: Option<crate::engine::Units>,
    /// The table the records were decoded against: `nmea2000` when the
    /// banner predates the key.
    protocol: std::result::Result<BusProtocol, String>,
}

/// Parse `line` as an analyzer banner; `None` when it isn't one.
fn banner(line: &str) -> Option<Banner> {
    if !line.starts_with("{\"version\"") {
        return None;
    }
    let root: Value = serde_json::from_str(line).ok()?;
    let root = root.as_object()?;
    let units = match root.get("units").and_then(Value::as_str) {
        Some("si") => Some(crate::engine::Units::Si),
        Some("std") => Some(crate::engine::Units::Metric),
        _ => None,
    };
    let protocol = match root.get("protocol").and_then(Value::as_str) {
        None => Ok(BusProtocol::Nmea2000),
        Some(p) => p.parse::<BusProtocol>().map_err(|_| p.to_string()),
    };
    Some(Banner { units, protocol })
}

impl<R: BufRead> FrameSource for JsonFrameReader<R> {
    fn read_frame(&mut self) -> std::io::Result<Option<RawFrame>> {
        loop {
            self.buf.clear();
            if self.src.read_line(&mut self.buf)? == 0 {
                return Ok(None);
            }
            self.line_no += 1;
            let line = self.buf.trim();
            if line.is_empty() {
                continue;
            }
            // Adopt the producer's declared unit system for the rest of
            // the stream. The banner itself is not a record, so either
            // way this line yields no frame.
            if let Some(banner) = banner(line) {
                let ours = self.db.protocol();
                match banner.protocol {
                    Ok(theirs) if theirs == ours => {}
                    Ok(theirs) => {
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::InvalidData,
                            format!(
                                "line {}: the input was decoded as {theirs}, but is read as \
                                 {ours}; pass --protocol {theirs}",
                                self.line_no
                            ),
                        ));
                    }
                    Err(theirs) => {
                        return Err(std::io::Error::new(
                            std::io::ErrorKind::InvalidData,
                            format!(
                                "line {}: the input was decoded as protocol '{theirs}', \
                                 which this version cannot read",
                                self.line_no
                            ),
                        ));
                    }
                }
                if let Some(units) = banner.units
                    && units != self.db.units()
                {
                    log::info!(
                        "input declares {} units; reading values against that schema",
                        if units == crate::engine::Units::Si {
                            "SI (rad/K/Pa)"
                        } else {
                            "Metric (deg/°C/bar)"
                        }
                    );
                    self.db = ours.database(units);
                }
                continue;
            }
            match frame_from_json(self.db, line) {
                Ok(Some(frame)) => return Ok(Some(frame)),
                Ok(None) => continue,
                Err(e) => {
                    log::warn!("line {}: {e:#}", self.line_no);
                    continue;
                }
            }
        }
    }
}

/// Encode one analyzer-JSON line. `Ok(None)` for records that are not
/// messages (the `{"version":…}` startup header).
pub fn frame_from_json(db: &'static PgnDatabase, line: &str) -> Result<Option<RawFrame>> {
    let root: Value = serde_json::from_str(line).context("parsing JSON")?;
    let root = root
        .as_object()
        .ok_or_else(|| anyhow!("not a JSON object"))?;

    // `-camel` wraps the record under the PGN variant id.
    let (variant_id, record) = match root.iter().next() {
        Some((key, Value::Object(inner)))
            if root.len() == 1 && !BARE_TOP_LEVEL.contains(&key.as_str()) =>
        {
            (Some(key.as_str()), inner)
        }
        _ => (None, root),
    };

    let Some(pgn) = record.get("pgn").and_then(Value::as_u64) else {
        if record.contains_key("version") {
            return Ok(None); // analyzer startup header
        }
        bail!("record has no 'pgn'");
    };
    // Synthetic canboat-internal PGNs (gateway status et al.) never
    // exist on the wire; skip them silently on the way back.
    if pgn >= 0x40000 {
        return Ok(None);
    }

    let mut builder = match variant_id {
        Some(id) => db
            .encode(id)
            .or_else(|_| db.encode_by_pgn(pgn as u32))
            .map_err(|e| anyhow!("{e}"))?,
        None => {
            // A bare record may still name its variant: `description`
            // carries either the human description ("System Time") or a
            // camel id — enough to disambiguate multi-variant PGNs
            // (canboatjs-style objects always include it).
            let by_desc = record
                .get("description")
                .and_then(Value::as_str)
                .and_then(|d| {
                    db.pgn_variants(pgn as u32)
                        .find(|p| p.description == d || p.id == d)
                });
            let by_match = || variant_by_match_fields(db, pgn as u32, record.get("fields"));
            match by_desc.or_else(by_match) {
                Some(info) => db.encode_for(info),
                None => db.encode_by_pgn(pgn as u32).map_err(|e| {
                    anyhow!("{e}; use the -camel wrapped form to select the exact variant")
                })?,
            }
        }
    };

    if let Some(p) = record.get("prio").and_then(Value::as_u64) {
        builder = builder.priority(p as u8);
    }
    if let Some(s) = record.get("src").and_then(Value::as_u64) {
        builder = builder.source(s as u8);
    }
    if let Some(d) = record.get("dst").and_then(Value::as_u64) {
        builder = builder.destination(d as u8);
    }
    if let Some(ts) = record.get("timestamp").and_then(Value::as_str) {
        builder = builder.timestamp(ts);
    }

    if let Some(Value::Object(fields)) = record.get("fields") {
        stage_fields(db, &mut builder, fields)?;
    }

    builder.build().map(Some).map_err(|e| anyhow!("{e}"))
}

/// Stage every member of a `fields` object onto the builder —
/// top-level fields directly, `"list"` / `"list2"` arrays as repeating
/// set 1 / 2 instances.
fn stage_fields(
    db: &'static PgnDatabase,
    builder: &mut PgnBuilder,
    fields: &Map<String, Value>,
) -> Result<()> {
    for (key, value) in fields {
        let set = match key.as_str() {
            "list" => Some(1),
            "list2" => Some(2),
            _ => None,
        };
        if let Some(set) = set {
            let Value::Array(instances) = value else {
                bail!("'{key}' is not an array");
            };
            for inst in instances {
                let Value::Object(inst_fields) = inst else {
                    bail!("'{key}' entry is not an object");
                };
                let idx = builder.add_set_instance(set).map_err(|e| anyhow!("{e}"))?;
                for (fkey, fval) in inst_fields {
                    let Some(info) = find_field(builder, fkey) else {
                        log::warn!("{key}[{idx}]: unknown field '{fkey}' ignored");
                        continue;
                    };
                    if let Some(ev) = to_encode_value(db, info, fval)
                        .with_context(|| format!("{key}[{idx}].{fkey}"))?
                    {
                        builder
                            .push_in_set(set, idx, info.name, ev)
                            .map_err(|e| anyhow!("{key}[{idx}].{fkey}: {e}"))?;
                    }
                }
            }
            continue;
        }
        let Some(info) = find_field(builder, key) else {
            log::warn!("unknown field '{key}' ignored");
            continue;
        };
        if let Some(ev) = to_encode_value(db, info, value).with_context(|| key.clone())? {
            // `push_by_name` matches schema names; `info` was resolved
            // from either the name or the camel id, so push by its name.
            builder
                .push_by_name(info.name, ev)
                .map_err(|e| anyhow!("{key}: {e}"))?;
        }
    }
    // A record carrying neither the count field nor its list encodes
    // a count of zero — the builder's auto-fill. canboat#812 settled
    // this: zero over the unavailable sentinel (canboatjs writes
    // unavailable; that difference is canboatjs-side now).
    Ok(())
}

fn find_field(builder: &PgnBuilder, key: &str) -> Option<&'static FieldInfo> {
    builder
        .pgn_info()
        .fields
        .iter()
        .find(|f| f.id == key || f.name == key)
}

const LOOKUP_TYPES: [FieldType; 4] = [
    FieldType::Lookup,
    FieldType::IndirectLookup,
    FieldType::BitLookup,
    FieldType::DynamicFieldKey,
];

fn is_lookup(f: &FieldInfo) -> bool {
    f.field_type.is_some_and(|t| LOOKUP_TYPES.contains(&t))
}

/// Translate one JSON field value into an [`EncodeValue`], or `None` to
/// leave the field at its default. The `-nv` object form carries the
/// raw wire value and is written back verbatim — the lossless path.
fn to_encode_value(
    db: &'static PgnDatabase,
    f: &'static FieldInfo,
    v: &Value,
) -> Result<Option<EncodeValue>> {
    Ok(match v {
        Value::Null => None,
        // `-nv` form: {"value":N,"name":"…"} — take the raw value.
        // Some field types render their value as a string even in `-nv`
        // (MMSIs, hex binaries); those route through the same per-type
        // string handling as bare string values.
        // A TIME / DURATION's `name` is its clock form, which reads the
        // same whatever `value` held: seconds now, the wire count in
        // captures from before canboat gave seconds.
        Value::Object(o)
            if matches!(
                f.field_type,
                Some(FieldType::Time) | Some(FieldType::Duration)
            ) && let Some(seconds) = o
                .get("name")
                .and_then(Value::as_str)
                .and_then(crate::engine::output::parse_time) =>
        {
            Some(EncodeValue::Number(seconds))
        }
        Value::Object(o) => match o.get("value") {
            Some(Value::Number(n)) => Some(raw_number(f, n)?),
            Some(Value::String(s)) => Some(string_value(f, s)?),
            _ => None,
        },
        Value::Number(n) => {
            if is_lookup(f) {
                // A bare number on a lookup field is the raw enum value.
                Some(EncodeValue::Int(n.as_i64().ok_or_else(|| {
                    anyhow!("lookup value {n} is not an integer")
                })?))
            } else if matches!(f.field_type, Some(FieldType::Binary)) {
                // canboatjs renders short BINARY fields as numbers (the
                // raw bits); rebuild the little-endian bytes.
                let raw = n
                    .as_u64()
                    .ok_or_else(|| anyhow!("binary value {n} is not an integer"))?;
                let width = f.bit_length.map(|bl| bl.div_ceil(8) as usize).unwrap_or(8);
                Some(EncodeValue::Bytes(
                    raw.to_le_bytes()[..width.min(8)].to_vec(),
                ))
            } else {
                // A physical value in the field's display unit.
                Some(EncodeValue::Number(
                    n.as_f64().ok_or_else(|| anyhow!("bad number {n}"))?,
                ))
            }
        }
        Value::String(s) => Some(string_value(f, s)?),
        // A bit lookup: the set bits, as `-nv` objects (mask values) or
        // bit names. OR them into one raw value.
        Value::Array(entries) => {
            let mut raw: u64 = 0;
            for e in entries {
                match e {
                    Value::Object(o) => {
                        let mask = o
                            .get("value")
                            .and_then(Value::as_u64)
                            .ok_or_else(|| anyhow!("bit entry without numeric 'value'"))?;
                        raw |= mask;
                    }
                    Value::String(name) => {
                        raw |= 1u64 << bit_by_name(db, f, name)?;
                    }
                    Value::Number(n) => {
                        raw |= n.as_u64().ok_or_else(|| anyhow!("bad bit mask {n}"))?;
                    }
                    other => bail!("unsupported bit entry {other}"),
                }
            }
            Some(EncodeValue::Int(raw as i64))
        }
        Value::Bool(_) => bail!("boolean values are not part of analyzer JSON"),
    })
}

/// The `-nv` raw value: written back verbatim as field bits. Fractional
/// raws do not exist on the wire, so a float here means the input was
/// not `-nv` after all — treat it as a physical value. That includes a
/// FLOAT under `-debug`, which wraps every field and gives a FLOAT's
/// value in the schema's unit (156.27 deg for 2.7274 rad in Metric), so
/// the encoder must take its resolution off, not write it as bits —
/// even when that value happens to be whole (180 deg). `-nv` writes a
/// FLOAT bare, so an object-form FLOAT is never a raw value.
fn raw_number(f: &FieldInfo, n: &serde_json::Number) -> Result<EncodeValue> {
    // A FLOAT, TIME or DURATION is never the wire integer, so even a
    // whole number is scaled: a FLOAT is in the schema's unit, a TIME or
    // DURATION is seconds.
    let physical = matches!(
        f.field_type,
        Some(FieldType::Float) | Some(FieldType::Time) | Some(FieldType::Duration)
    );
    if let Some(i) = n.as_i64().filter(|_| !physical) {
        Ok(EncodeValue::Int(i))
    } else {
        Ok(EncodeValue::Number(n.as_f64().ok_or_else(|| {
            anyhow!("field '{}': bad number {n}", f.name)
        })?))
    }
}

fn string_value(f: &'static FieldInfo, s: &str) -> Result<EncodeValue> {
    match f.field_type {
        Some(FieldType::StringFix) | Some(FieldType::StringLz) | Some(FieldType::StringLau) => {
            Ok(EncodeValue::Text(s.to_string()))
        }
        Some(FieldType::Binary) | Some(FieldType::DynamicFieldValue) => {
            Ok(EncodeValue::Bytes(parse_hex(s)?))
        }
        // A group-function VARIABLE value: its real type lives in the
        // referenced PGN's field and is resolved at build time — pass
        // the text through, the builder retypes labels for lookups.
        Some(FieldType::Variable) => Ok(EncodeValue::Text(s.to_string())),
        // The display renderings parse back: "2017.04.15" → days since
        // epoch, "14:57:57(.1234)" → seconds. Both are the physical
        // value in the field's unit (d / s), so Number staging scales
        // them through the schema resolution.
        Some(FieldType::Date) => parse_date_days(s)
            .map(EncodeValue::Number)
            .ok_or_else(|| anyhow!("field '{}': bad date '{s}'", f.name)),
        Some(FieldType::Time) | Some(FieldType::Duration) => crate::engine::output::parse_time(s)
            .map(EncodeValue::Number)
            .ok_or_else(|| anyhow!("field '{}': bad time '{s}'", f.name)),
        // A digit string on a lookup is the raw value beyond the
        // enumeration (canboatjs renders unknown entries that way);
        // anything else is a label.
        _ if is_lookup(f) => Ok(match s.parse::<i64>() {
            Ok(raw) => EncodeValue::Int(raw),
            Err(_) => EncodeValue::Lookup(s.to_string()),
        }),
        // Digit strings on numeric fields (e.g. an MMSI rendered as a
        // string) parse back.
        _ => s.parse::<f64>().map(EncodeValue::Number).map_err(|_| {
            anyhow!(
                "field '{}': cannot re-encode display string '{s}'; use -nv JSON input",
                f.name
            )
        }),
    }
}

/// `"YYYY.MM.DD"` (the analyzer/canboatjs date rendering) → days since
/// the epoch, the DATE field's physical value.
fn parse_date_days(s: &str) -> Option<f64> {
    let mut it = s.split('.');
    let (y, m, d) = (
        it.next()?.parse::<i32>().ok()?,
        it.next()?.parse::<u32>().ok()?,
        it.next()?.parse::<u32>().ok()?,
    );
    if it.next().is_some() {
        return None;
    }
    let date = chrono::NaiveDate::from_ymd_opt(y, m, d)?;
    let epoch = chrono::NaiveDate::from_ymd_opt(1970, 1, 1)?;
    Some((date - epoch).num_days() as f64)
}

/// Select a PGN variant from a bare record's field values, the way
/// canboatjs's encoder does: a variant qualifies when every one of its
/// match-valued fields that the record provides agrees with the
/// declared match value; among qualifiers the one agreeing on the most
/// provided fields wins (a strict winner — a tie stays ambiguous).
/// This is what resolves e.g. a 126208 with `"Function Code":"Command"`
/// to the command variant without a camel wrapper or description.
fn variant_by_match_fields(
    db: &'static PgnDatabase,
    pgn: u32,
    fields: Option<&Value>,
) -> Option<&'static crate::engine::types::PgnInfo> {
    let fields = fields?.as_object()?;
    let mut best: Option<(usize, &'static crate::engine::types::PgnInfo)> = None;
    let mut tied = false;
    'variants: for info in db.pgn_variants(pgn) {
        let mut agreed = 0usize;
        for f in info.fields {
            let Some(mv) = f.match_value else { continue };
            let Some(v) = fields.get(f.id).or_else(|| fields.get(f.name)) else {
                continue;
            };
            match match_field_agrees(db, f, v, mv) {
                Some(true) => agreed += 1,
                Some(false) => continue 'variants,
                None => continue,
            }
        }
        if agreed == 0 {
            continue;
        }
        match &best {
            Some((n, _)) if *n == agreed => tied = true,
            Some((n, _)) if *n > agreed => {}
            _ => {
                best = Some((agreed, info));
                tied = false;
            }
        }
    }
    if tied { None } else { best.map(|(_, i)| i) }
}

/// Does a provided JSON value agree with a field's declared match
/// value? `None` when the value cannot be interpreted for comparison.
fn match_field_agrees(
    db: &'static PgnDatabase,
    f: &'static FieldInfo,
    v: &Value,
    mv: i64,
) -> Option<bool> {
    match v {
        Value::Number(n) => Some(n.as_i64()? == mv),
        Value::String(s) => {
            let table = f.lookup_enumeration.and_then(|t| db.lookup(t))?;
            let val = table
                .values
                .iter()
                .find(|lv| lv.name == *s || lv.id == Some(s.as_str()))?
                .value;
            Some(val as i64 == mv)
        }
        Value::Object(o) => o.get("value").and_then(Value::as_i64).map(|x| x == mv),
        _ => None,
    }
}

/// Resolve a bit name (`"Low Oil Pressure"`) to its bit position via
/// the field's BITLOOKUP table.
fn bit_by_name(db: &'static PgnDatabase, f: &'static FieldInfo, name: &str) -> Result<u8> {
    let table = f
        .lookup_bit_enumeration
        .and_then(|t| db.bit_lookup(t))
        .ok_or_else(|| anyhow!("field '{}' has no bit lookup table", f.name))?;
    table
        .values
        .iter()
        .find(|bv| bv.name == name || bv.id == Some(name))
        .map(|bv| bv.bit)
        .ok_or_else(|| anyhow!("field '{}': unknown bit '{name}'", f.name))
}

/// Parse analyzer hex-byte strings: `"d1 08 00"`, `"D108"`, with or
/// without spaces.
fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let compact: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !compact.len().is_multiple_of(2) || compact.is_empty() {
        bail!("bad hex string '{s}'");
    }
    (0..compact.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&compact[i..i + 2], 16).map_err(|e| anyhow!("{e}")))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::Units;

    fn db() -> &'static PgnDatabase {
        PgnDatabase::embedded(Units::Metric)
    }

    #[test]
    fn si_energy_and_charge_encode_from_joules_and_coulombs() {
        // The database keeps kWh and Ah; the SI schema reads J and C
        // back, so both unit systems put the same bits on the wire.
        let si = PgnDatabase::embedded(Units::Si);
        for (si_line, metric_line) in [
            (
                r#"{"pgn":65005,"fields":{"totalEnergyExport":14806800000,"totalEnergyImport":1080000000}}"#,
                r#"{"pgn":65005,"fields":{"totalEnergyExport":4113,"totalEnergyImport":300}}"#,
            ),
            (
                r#"{"pgn":127506,"fields":{"instance":1,"remainingCapacity":360000}}"#,
                r#"{"pgn":127506,"fields":{"instance":1,"remainingCapacity":100}}"#,
            ),
        ] {
            let from_si = frame_from_json(si, si_line).unwrap().unwrap();
            let from_metric = frame_from_json(db(), metric_line).unwrap().unwrap();
            assert_eq!(from_si.data, from_metric.data, "{si_line}");
        }
        let f = frame_from_json(
            si,
            r#"{"pgn":65005,"fields":{"totalEnergyExport":14806800000}}"#,
        )
        .unwrap()
        .unwrap();
        assert_eq!(f.data[..4], [0x11, 0x10, 0x00, 0x00]);
    }

    #[test]
    fn si_percentages_encode_from_ratios() {
        // The database keeps %; the SI schema reads a ratio, signed ones
        // included, so both unit systems put the same bits on the wire.
        let si = PgnDatabase::embedded(Units::Si);
        for (si_line, metric_line) in [
            (
                r#"{"pgn":127505,"fields":{"instance":0,"type":"Fuel","level":0.97536}}"#,
                r#"{"pgn":127505,"fields":{"instance":0,"type":"Fuel","level":97.536}}"#,
            ),
            (
                r#"{"pgn":130576,"fields":{"portTrimTab":-0.5,"starboardTrimTab":0.25}}"#,
                r#"{"pgn":130576,"fields":{"portTrimTab":-50,"starboardTrimTab":25}}"#,
            ),
        ] {
            let from_si = frame_from_json(si, si_line).unwrap().unwrap();
            let from_metric = frame_from_json(db(), metric_line).unwrap().unwrap();
            assert_eq!(from_si.data, from_metric.data, "{si_line}");
        }
        let f = frame_from_json(
            si,
            r#"{"pgn":130576,"fields":{"portTrimTab":-0.5,"starboardTrimTab":0.25}}"#,
        )
        .unwrap()
        .unwrap();
        assert_eq!(f.data[..2], [0xce, 0x19]);
    }

    #[test]
    fn time_encodes_from_seconds_or_any_clock_form() {
        // 126992 System Time 09:10:20.2240 = 33020.2240 s = raw 330202240
        // (0.0001 s). Seconds, bare or -nv, are what canboat writes now;
        // the clock string, a number of seconds as text, and an older
        // -nv object whose value was the raw count (its name still the
        // clock) are all read too.
        let want = [0x80u8, 0x7c, 0xae, 0x13];
        for time in [
            "33020.224",
            r#"{"value":33020.2240,"name":"09:10:20.2240"}"#,
            r#""09:10:20.2240""#,
            r#""33020.224""#,
            r#"{"value":330202240,"name":"09:10:20.2240"}"#,
        ] {
            let line = format!(
                r#"{{"pgn":126992,"fields":{{"source":"GPS","date":"2024.07.30","time":{time}}}}}"#
            );
            let frame = frame_from_json(db(), &line).unwrap().unwrap();
            assert_eq!(frame.data[frame.data.len() - 4..], want, "{time}");
        }
    }

    #[test]
    fn negative_duration_clock_encodes_negative() {
        // "-00:05:00.000" is minus five minutes, not minus zero hours
        // plus five minutes.
        assert_eq!(
            crate::engine::output::parse_time("-00:05:00.000"),
            Some(-300.0)
        );
        assert_eq!(crate::engine::output::parse_time("-300"), Some(-300.0));
        assert_eq!(crate::engine::output::parse_time("01:10:10"), Some(4210.0));
        assert_eq!(crate::engine::output::parse_time("later"), None);
        assert_eq!(crate::engine::output::parse_time("--00:05:00"), None);
        assert_eq!(crate::engine::output::parse_time("00:-05:00"), None);
        assert_eq!(crate::engine::output::parse_time("inf:00"), None);
    }

    #[test]
    fn si_volumes_rotations_and_angles_encode_from_si_values() {
        // The database keeps rpm, L, L/h and degree offsets; the SI schema
        // reads Hz, m3, m3/s and rad, so both unit systems put the same
        // bits on the wire.
        let si = PgnDatabase::embedded(Units::Si);
        for (si_line, metric_line) in [
            (
                r#"{"pgn":127488,"fields":{"instance":0,"speed":30.0}}"#,
                r#"{"pgn":127488,"fields":{"instance":0,"speed":1800.0}}"#,
            ),
            (
                r#"{"pgn":127497,"fields":{"instance":0,"tripFuelUsed":0.045,"fuelRateAverage":0.0000029166666666666666}}"#,
                r#"{"pgn":127497,"fields":{"instance":0,"tripFuelUsed":45.0,"fuelRateAverage":10.5}}"#,
            ),
            (
                r#"{"pgn":130818,"fields":{"manufacturerCode":"Furuno","industryCode":"Marine Industry","headingOffset":0.21816615649929116,"pitchOffset":-0.04363323129985824}}"#,
                r#"{"pgn":130818,"fields":{"manufacturerCode":"Furuno","industryCode":"Marine Industry","headingOffset":12.5,"pitchOffset":-2.5}}"#,
            ),
        ] {
            let from_si = frame_from_json(si, si_line).unwrap().unwrap();
            let from_metric = frame_from_json(db(), metric_line).unwrap().unwrap();
            assert_eq!(from_si.data, from_metric.data, "{si_line}");
        }
    }

    #[test]
    fn debug_float_encodes_from_its_presented_value() {
        // -debug wraps a FLOAT as {"value":…} in the schema's unit: deg in
        // Metric, rad in SI. Both put Heading to Steer's own bits back.
        let si = PgnDatabase::embedded(Units::Si);
        for (db, value) in [(db(), "156.26844443585534"), (si, "2.7273988723754883")] {
            let line = format!(
                r#"{{"pgn":126720,"fields":{{"manufacturerCode":"Garmin","industryCode":"Marine Industry","subProtocolId":"Autopilot transport","field":11,"headingToSteer":{{"value":{value},"bytes":"B4 8D 2E 40"}}}}}}"#
            );
            let frame = frame_from_json(db, &line).unwrap().unwrap();
            assert_eq!(
                frame.data[frame.data.len() - 4..],
                [0xb4, 0x8d, 0x2e, 0x40],
                "{value}"
            );
        }
    }

    #[test]
    fn debug_float_with_a_whole_value_is_still_scaled() {
        // 180 deg in Metric is π rad on the wire, not 180.0f32.
        let line = r#"{"pgn":126720,"fields":{"manufacturerCode":"Garmin","industryCode":"Marine Industry","subProtocolId":"Autopilot transport","field":11,"headingToSteer":{"value":180}}}"#;
        let frame = frame_from_json(db(), line).unwrap().unwrap();
        let wire = f32::from_le_bytes(frame.data[frame.data.len() - 4..].try_into().unwrap());
        assert!((wire - std::f32::consts::PI).abs() < 1e-5, "{wire}");
    }

    #[test]
    fn camel_envelope_encodes_byte_exact() {
        // Integer-only PGN → wire-exact round trip. The envelope id
        // selects the variant; -nv objects carry raw values verbatim.
        let line = r#"{"isoAddressClaim":{"prio":6,"src":3,"dst":255,"pgn":60928,
            "fields":{"uniqueNumber":1631699,"manufacturerCode":{"value":1580},
            "deviceInstanceLower":0,"deviceInstanceUpper":0,
            "deviceFunction":{"value":130,"name":"PC Gateway"},
            "deviceClass":{"value":25,"name":"Internetwork device"},
            "systemInstance":0,"industryGroup":{"value":4,"name":"Marine Industry"},
            "arbitraryAddressCapable":{"value":1,"name":"Yes"}}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(frame.pgn, 60928);
        assert_eq!(frame.src, 3);
        assert_eq!(
            frame.data.as_slice(),
            &[0xd3, 0xe5, 0x98, 0xc5, 0x00, 0x82, 0x32, 0xc0]
        );
    }

    #[test]
    fn repeating_list_encodes_instances() {
        let line = r#"{"gnssSatsInView":{"pgn":129540,"src":3,"dst":255,"prio":6,
            "fields":{"sid":9,"list":[
              {"prn":7,"snr":30.0},
              {"prn":11,"snr":42.5}]}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        let decoded = db().decode(&frame).unwrap();
        assert!(decoded.has_repeating_set[0]);
        let prns: Vec<i64> = decoded
            .fields
            .iter()
            .filter(|f| f.info.id == "prn" && f.repeat_set == 1)
            .map(|f| match &f.value {
                crate::engine::FieldValue::Integer(n) => *n,
                crate::engine::FieldValue::Number(x) => *x as i64,
                other => panic!("prn: {other:?}"),
            })
            .collect();
        assert_eq!(prns, vec![7, 11]);
    }

    #[test]
    fn startup_header_and_synthetic_pgns_are_skipped() {
        assert!(
            frame_from_json(db(), r#"{"version":"8.0.0","units":"si"}"#)
                .unwrap()
                .is_none()
        );
        assert!(
            frame_from_json(db(), r#"{"canboatStartup":{"pgn":262656,"fields":{}}}"#)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn sub_byte_binary_hex_round_trips() {
        // AIS Communication State is a 19-bit BINARY rendered as three
        // hex bytes; it must stage as masked bits, not whole bytes.
        let line = r#"{"positionReportClassB":{"pgn":129039,"src":43,"dst":255,"prio":2,
            "fields":{"userId":{"value":"338654321"},"communicationState":"2A 4C 01"}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        let decoded = db().decode(&frame).unwrap();
        match &decoded.field_by_name("Communication State").unwrap().value {
            crate::engine::FieldValue::Binary(b) => {
                assert_eq!(&b[..], &[0x2a, 0x4c, 0x01]);
            }
            other => panic!("communicationState: {other:?}"),
        }
    }

    #[test]
    fn bare_record_selects_variant_by_match_fields() {
        // The exact shape signalk-server's device manager emits for an
        // NMEA instance change: bare (no wrapper, no description), the
        // variant named only by the Function Code match field. Must
        // encode byte-identical to canboatjs.
        let line = r#"{"pgn":126208,"prio":3,"dst":42,"fields":{
            "Function Code":"Command","PGN":60928,"priority":8,
            "numberOfParameters":2,
            "list":[{"parameter":3,"value":5},{"parameter":4,"value":0}]}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(frame.pgn, 126208);
        assert_eq!(frame.dst, 42);
        assert_eq!(
            frame.data.as_slice(),
            &[0x01, 0x00, 0xee, 0x00, 0xf8, 0x02, 0x03, 0x05, 0x04, 0x00]
        );
    }

    #[test]
    fn ambiguous_bare_pgn_is_a_clear_error() {
        let err = frame_from_json(db(), r#"{"pgn":126208,"fields":{}}"#).unwrap_err();
        assert!(err.to_string().contains("-camel"), "got: {err:#}");
    }

    #[test]
    fn banner_reads_the_producers_declaration() {
        let si = r#"{"version":"8.0.0","commit":"abc","units":"si","showLookupValues":true}"#;
        let std = r#"{"version":"7.1.0","units":"std","showLookupValues":true}"#;
        let j1939 =
            r#"{"version":"8.4.0","units":"si","protocol":"j1939","showLookupValues":true}"#;
        assert_eq!(banner(si).unwrap().units, Some(Units::Si));
        assert_eq!(banner(std).unwrap().units, Some(Units::Metric));
        // A banner from before the key, canboat C's included, is NMEA 2000.
        assert_eq!(banner(si).unwrap().protocol, Ok(BusProtocol::Nmea2000));
        assert_eq!(banner(j1939).unwrap().protocol, Ok(BusProtocol::J1939));
        assert_eq!(
            banner(r#"{"version":"8.4.0","protocol":"quick"}"#)
                .unwrap()
                .protocol,
            Ok(BusProtocol::Quick)
        );
        assert_eq!(
            banner(r#"{"version":"9.0.0","protocol":"canopen"}"#)
                .unwrap()
                .protocol,
            Err("canopen".to_string())
        );
        // Not a banner, or a banner that doesn't say — the caller's
        // assumption stands rather than being silently overridden.
        assert_eq!(banner(r#"{"version":"7.1.0"}"#).unwrap().units, None);
        assert_eq!(banner(r#"{"pgn":127250,"fields":{}}"#), None);
    }

    /// A banner's units are adopted, but in the reader's own protocol: a
    /// J1939 reader stays J1939 when the banner switches it to Metric.
    #[test]
    fn banner_units_keep_the_protocol() {
        let input = "{\"version\":\"8.4.0\",\"units\":\"std\",\"protocol\":\"j1939\",\"showLookupValues\":true}\n";
        let mut reader =
            JsonFrameReader::new(input.as_bytes(), BusProtocol::J1939.database(Units::Si));
        assert!(reader.read_frame().unwrap().is_none());
        assert!(reader.db.is_j1939());
        assert_eq!(reader.db.units(), Units::Metric);
    }

    /// A J1939 stream read against the NMEA 2000 table is refused,
    /// naming the option that fixes it.
    #[test]
    fn a_protocol_mismatch_is_an_error() {
        let input = "{\"version\":\"8.4.0\",\"units\":\"si\",\"protocol\":\"j1939\",\"showLookupValues\":true}\n";
        let mut reader = JsonFrameReader::new(input.as_bytes(), db());
        let err = reader.read_frame().unwrap_err();
        assert!(err.to_string().contains("--protocol j1939"), "{err}");
    }

    /// The whole point of tracking the banner: the same decimal means
    /// different bits depending on the stream's units.
    #[test]
    fn banner_units_decide_how_bare_values_encode() {
        let deg = frame_from_json(
            PgnDatabase::embedded(Units::Metric),
            r#"{"pgn":127250,"src":27,"description":"Vessel Heading","fields":{"heading":210.9}}"#,
        )
        .unwrap()
        .unwrap();
        let rad = frame_from_json(
            PgnDatabase::embedded(Units::Si),
            r#"{"pgn":127250,"src":27,"description":"Vessel Heading","fields":{"heading":210.9}}"#,
        )
        .unwrap()
        .unwrap();
        assert_ne!(
            deg.data.as_slice(),
            rad.data.as_slice(),
            "210.9 deg and 210.9 rad must not encode alike"
        );
        // 210.9 deg = 3.68094 rad, i.e. 36809 in 0.0001 rad units.
        assert_eq!(&deg.data[1..3], &[0xc9, 0x8f]);
    }
}
