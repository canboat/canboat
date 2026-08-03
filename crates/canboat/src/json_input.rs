// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat convert --from json` — re-encode analyzer JSON into wire
//! frames.
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
//! This lives in the CLI crate (not `canboat-core`) because it is the
//! one place a real JSON parser is warranted: nested repeating lists
//! and `-nv` objects are beyond `analyzer_json`'s deliberate
//! substring-scan minimalism.

use std::io::BufRead;

use anyhow::{Context, Result, anyhow, bail};
use canboat_core::source::FrameSource;
use canboat_core::types::{FieldInfo, FieldType};
use canboat_core::{EncodeValue, PgnBuilder, PgnDatabase, RawFrame};
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
/// input) — the startup `{"version":…}` header is skipped silently.
pub struct JsonFrameReader<R> {
    src: R,
    db: &'static PgnDatabase,
    buf: String,
    line_no: usize,
}

impl<R: BufRead> JsonFrameReader<R> {
    pub fn new(src: R, db: &'static PgnDatabase) -> Self {
        Self {
            src,
            db,
            buf: String::with_capacity(512),
            line_no: 0,
        }
    }
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
        None => db
            .encode_by_pgn(pgn as u32)
            .map_err(|e| anyhow!("{e}; use the -camel wrapped form to select the exact variant"))?,
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
/// not `-nv` after all — treat it as a physical value.
fn raw_number(f: &FieldInfo, n: &serde_json::Number) -> Result<EncodeValue> {
    if let Some(i) = n.as_i64() {
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
        _ if is_lookup(f) => Ok(EncodeValue::Lookup(s.to_string())),
        // Digit strings on numeric fields (e.g. an MMSI rendered as a
        // string) parse back; display-formatted date/time/duration
        // strings do not — that data needs the `-nv` form.
        _ => s.parse::<f64>().map(EncodeValue::Number).map_err(|_| {
            anyhow!(
                "field '{}': cannot re-encode display string '{s}'; use -nv JSON input",
                f.name
            )
        }),
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
    use canboat_core::Units;

    fn db() -> &'static PgnDatabase {
        PgnDatabase::embedded(Units::Metric)
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
                canboat_core::FieldValue::Integer(n) => *n,
                canboat_core::FieldValue::Number(x) => *x as i64,
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
            canboat_core::FieldValue::Binary(b) => {
                assert_eq!(&b[..], &[0x2a, 0x4c, 0x01]);
            }
            other => panic!("communicationState: {other:?}"),
        }
    }

    #[test]
    fn ambiguous_bare_pgn_is_a_clear_error() {
        let err = frame_from_json(db(), r#"{"pgn":126208,"fields":{}}"#).unwrap_err();
        assert!(err.to_string().contains("-camel"), "got: {err:#}");
    }
}
