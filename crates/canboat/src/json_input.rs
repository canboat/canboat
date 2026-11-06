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
use crate::engine::types::{FieldInfo, FieldRef, FieldType};
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
            match decoded_frame_from_json(self.db, line) {
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
    frame_from_json_as(db, line, false)
}

/// [`frame_from_json`] for a record canboat decoded, as `convert --from
/// json` reads it: a key/value value the decoder left out is written as it
/// was left out ([`PgnBuilder::read_as_decoded`]). Not for a command meant for
/// the bus, where a value forgotten should be an error.
pub(crate) fn decoded_frame_from_json(
    db: &'static PgnDatabase,
    line: &str,
) -> Result<Option<RawFrame>> {
    frame_from_json_as(db, line, true)
}

fn frame_from_json_as(
    db: &'static PgnDatabase,
    line: &str,
    decoded: bool,
) -> Result<Option<RawFrame>> {
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
            // That includes the range's catch-all, which the decoder
            // falls back to when no definition of the number matches.
            let by_desc = record
                .get("description")
                .and_then(Value::as_str)
                .and_then(|d| {
                    db.pgn_variants(pgn as u32)
                        .chain(db.fallback_pgn(pgn as u32))
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

    if decoded {
        builder = builder.read_as_decoded();
    }
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

    // A catch-all covers a range of PGNs: the record's own is the one
    // on the wire, not the first of the range.
    let catch_all = builder.pgn_info().fallback == Some(true);
    let mut frame = builder.build().map_err(|e| anyhow!("{e}"))?;
    if catch_all
        && db
            .fallback_pgn(pgn as u32)
            .is_some_and(|c| c.pgn == frame.pgn)
    {
        frame.pgn = pgn as u32;
    }
    Ok(Some(frame))
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
                            .push_in_set(set, idx, info.id, ev)
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
            // Push the field `info` itself: several fields can share a
            // name (a PGN's Reserved fields), and pushing by name would
            // set the first of them.
            let field = FieldRef {
                pgn: builder.pgn_info(),
                field: info,
            };
            builder.push(field, ev).map_err(|e| anyhow!("{key}: {e}"))?;
        }
    }
    // A record carrying neither the count field nor its list encodes
    // a count of zero — the builder's auto-fill. canboat#812 settled
    // this: zero over the unavailable sentinel (canboatjs writes
    // unavailable; that difference is canboatjs-side now). Read as
    // decoded, a count left out was not a number, so it stays
    // unavailable.
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
            // -debug writes a bit lookup as {"value":[…],"bytes":…}; the
            // list reads as it does bare, `[]` as no bits set.
            Some(list @ Value::Array(_)) => to_encode_value(db, f, list)?,
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
            } else if let Some(i) = n.as_i64() {
                // A whole number, kept exact: a 64-bit one is more than an
                // f64 holds.
                Some(EncodeValue::Int(i))
            } else if let Some(u) = n.as_u64() {
                Some(EncodeValue::Raw(u))
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
    // DURATION is seconds. Nor is a group-function VARIABLE: its value is
    // the target field's, written like that field's own.
    let physical = matches!(
        f.field_type,
        Some(FieldType::Float)
            | Some(FieldType::Time)
            | Some(FieldType::Duration)
            | Some(FieldType::Variable)
    );
    // The wire bits: a signed field's negative raw two's-complements back
    // into its width, and a 64-bit one (an ISO NAME) can exceed i64.
    let raw = n.as_u64().or_else(|| n.as_i64().map(|i| i as u64));
    if let Some(raw) = raw.filter(|_| !physical) {
        Ok(EncodeValue::Raw(raw))
    } else if let Some(i) = n.as_i64() {
        Ok(EncodeValue::Int(i)) // the same value as a Number, kept exact
    } else {
        Ok(EncodeValue::Number(n.as_f64().ok_or_else(|| {
            anyhow!("field '{}': bad number {n}", f.name)
        })?))
    }
}

/// Turn a JSON string value for field `f` into the value the encoder
/// stages: text, hex bytes, a date or time, a lookup label or raw code, or
/// a number (64-bit digit strings exactly). A key/value value of digits
/// stays text, for the encoder to type by its key. Errors when the string
/// can't be read back for that field.
fn string_value(f: &'static FieldInfo, s: &str) -> Result<EncodeValue> {
    match f.field_type {
        Some(FieldType::StringFix) | Some(FieldType::StringLz) | Some(FieldType::StringLau) => {
            Ok(EncodeValue::Text(s.to_string()))
        }
        Some(FieldType::Binary) => Ok(EncodeValue::Bytes(parse_hex(s)?)),
        // A key/value value is its bytes as hex, or a lookup name, time or
        // a NAME's digits ("Off", "10:00:00", "13869005477589862183") that
        // the record's key gives meaning to. Digits alone ("10") can be
        // either a number or hex bytes, so the encoder decides by the key.
        Some(FieldType::DynamicFieldValue)
            if !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()) =>
        {
            Ok(EncodeValue::Text(s.to_string()))
        }
        Some(FieldType::DynamicFieldValue) => Ok(parse_hex(s)
            .map(EncodeValue::Bytes)
            .unwrap_or_else(|_| EncodeValue::Text(s.to_string()))),
        // A Reserved or Spare field printed as hex (one that isn't all
        // ones, or for spare all zeros): the encoder writes the bytes back
        // where the decoder found them.
        Some(FieldType::Reserved) | Some(FieldType::Spare) => Ok(EncodeValue::Bytes(parse_hex(s)?)),
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
        // Digit strings on numeric fields (an MMSI, or a NAME or other
        // 64-bit number, which canboat writes as its digits, canboat#1157)
        // parse back exactly.
        _ if s.parse::<i64>().is_ok() => Ok(EncodeValue::Int(s.parse().unwrap_or_default())),
        _ if s.parse::<u64>().is_ok() => Ok(EncodeValue::Raw(s.parse().unwrap_or_default())),
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
    if !compact.len().is_multiple_of(2)
        || compact.is_empty()
        || !compact.chars().all(|c| c.is_ascii_hexdigit())
    {
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
        assert_eq!(crate::engine::output::parse_time("1e308:00"), None);
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

    /// Key/value records whose values are typed by their key decode and
    /// encode back to the same bytes, from the -nv JSON and from the
    /// plain one: B&G's 130824 (numbers, a negative angle, a time and a
    /// value left out as unknown), Simnet's 130845 (a time, in 60 s steps)
    /// and a Navico 130822 Object Dump whose last record the frame cuts off
    /// (canboat#1114).
    /// A NAME and a 64-bit number are written as their digits, as most
    /// JSON readers read a number as a double (canboat#1157), and read back
    /// exactly from that and from a bare number.
    #[test]
    fn a_64_bit_value_is_written_as_its_digits_and_reads_back_exactly() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        // 130840 Navico Data Source with its source's NAME, and Maretron's
        // 130833 with a memory capacity of 0xF000000000000001, above
        // i64::MAX.
        let frames: [(u32, Vec<u8>, [&str; 2]); 2] = [
            (
                130840,
                vec![
                    0x41, 0x9f, 0xff, 0x00, 0x01, 0x34, 0xff, 0x73, 0x64, 0x43, 0xe0, 0xe7, 0x00,
                    0x8c, 0x78, 0xc0, 0xff,
                ],
                [r#""source":"13"#, r#""source":{"value":"13"#],
            ),
            (
                130833,
                vec![
                    0x89, 0x98, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 0xf0, 0xe8, 0x03, 0, 0, 0, 0, 0, 0,
                ],
                [r#""memoryCapacity":"17293822569102704641""#; 2],
            ),
        ];
        for (pgn, data, forms) in frames {
            let frame = RawFrame::new(None, 3, pgn, 13, 255, data.clone());
            let decoded = db.decode(&frame).unwrap();
            for name_value in [true, false] {
                let opts = crate::engine::output::JsonOptions {
                    name_value,
                    ..Default::default()
                };
                let mut line = String::new();
                crate::engine::output::write_json(&mut line, &decoded, &opts).unwrap();
                let form = forms[usize::from(name_value)];
                assert!(line.contains(form), "{pgn} nv={name_value}: {line}");
                let back = decoded_frame_from_json(db, &line).unwrap().unwrap();
                assert_eq!(back.data.to_vec(), data, "{pgn} nv={name_value}: {line}");
                // Also as a bare number, as canboat wrote it before.
                let bare = line.replace(r#""17293822569102704641""#, "17293822569102704641");
                let back = decoded_frame_from_json(db, &bare).unwrap().unwrap();
                assert_eq!(back.data.to_vec(), data, "{pgn} bare: {bare}");
            }
        }
    }

    /// A key/value value given as digits is a number when the record's key
    /// makes it one, however short ("10" as a NAME), scaled by the key as a
    /// bare number is, and hex bytes otherwise.
    #[test]
    fn key_value_digits_are_typed_by_the_key() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        // A Navico 130822 Configuration Set whose value is a NAME of 10, and
        // one of 2^64 - 3, the largest a NAME holds short of its two
        // sentinels.
        for name in [
            [0x0a, 0, 0, 0, 0, 0, 0, 0],
            [0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
        ] {
            let mut data = vec![
                0x13, 0x99, 0xff, 0x03, 0x00, 0x0a, 0x01, 0x00, 0x15, 0x00, 0x00, 0x00, 0x84, 0xca,
                0x08, 0x00,
            ];
            data.extend(name);
            let frame = RawFrame::new(None, 3, 130822, 49, 255, data.clone());
            let decoded = db.decode(&frame).unwrap();
            for name_value in [true, false] {
                let opts = crate::engine::output::JsonOptions {
                    name_value,
                    ..Default::default()
                };
                let mut line = String::new();
                crate::engine::output::write_json(&mut line, &decoded, &opts).unwrap();
                let back = decoded_frame_from_json(db, &line).unwrap().unwrap();
                assert_eq!(back.data.to_vec(), data, "nv={name_value}: {line}");
            }
        }

        // B&G 130824 Course (key 105, 0.0001 rad): "5" is 5 rad, as 5 is.
        let record = |value: &str| {
            format!(
                r#"{{"prio":3,"src":16,"dst":255,"pgn":130824,"fields":{{"manufacturerCode":381,"industryCode":4,"list":[{{"key":105,"length":2,"value":{value}}}]}}}}"#
            )
        };
        let bare = frame_from_json(db, &record("5")).unwrap().unwrap();
        let digits = frame_from_json(db, &record(r#""5""#)).unwrap().unwrap();
        assert_eq!(digits.data, bare.data);

        // Digits too big for an unscaled key are refused, as the number is:
        // B&G key 0 (Altitude) is a signed 16-bit number, whose top three
        // values, 32765 to 32767, are its sentinels.
        let altitude = |value: &str| {
            format!(
                r#"{{"prio":3,"src":16,"dst":255,"pgn":130824,"fields":{{"manufacturerCode":381,"industryCode":4,"list":[{{"key":0,"length":2,"value":{value}}}]}}}}"#
            )
        };
        for sentinel in ["32765", "32767", "32768"] {
            assert!(
                frame_from_json(db, &altitude(sentinel)).is_err(),
                "{sentinel}"
            );
            let digits = format!(r#""{sentinel}""#);
            assert!(frame_from_json(db, &altitude(&digits)).is_err(), "{digits}");
        }
        assert_eq!(
            frame_from_json(db, &altitude(r#""32764""#))
                .unwrap()
                .unwrap()
                .data,
            frame_from_json(db, &altitude("32764"))
                .unwrap()
                .unwrap()
                .data
        );
    }

    #[test]
    fn key_value_records_encode_back() {
        let frames: [(u32, &str); 4] = [
            (
                130824,
                "7d,99,2b,41,26,24,ad,0c,2c,41,3f,75,b8,63,37,21,d1,35,38,21,bb,03,7e,20,9b,01,7c,20,f8,03,32,21,e4,a0,33,21,29,cd,1d,21,87,03",
            ),
            (
                130824,
                "7d,99,64,20,57,01,0d,21,48,01,0e,41,40,4e,57,12,0f,41,82,96,82,59,69,20,ef,d7,d3,20,94,c8,81,40,20,1c,00,00,9a,20,e5,9f,82,20,00,00,0a,21,80,01,0c,21,4f,02,35,20,7c,d9,7d,20,8a,01,53,20,b7,7a,75,40,e6,b2,fc,ff,09,41,46,34,03,00,0b,41,ff,ff,ff,ff,d0,40,70,89,a0,0d,1d,21,db,02,7f,20,20,01,9d,20,92,4b",
            ),
            // Two bytes past the 16-bit value are left out by the decoder.
            (130845, "41,9f,ff,ff,02,ff,29,00,00,01,58,02"),
            (
                130822,
                "13,99,ff,06,00,05,03,e8,52,0f,01,00,37,e1,04,02,01,00,00,04,02,21,00,00,04,02",
            ),
        ];
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        for (pgn, hex) in frames {
            let data: Vec<u8> = hex
                .split(',')
                .map(|b| u8::from_str_radix(b, 16).unwrap())
                .collect();
            let frame = RawFrame::new(None, 3, pgn, 30, 255, data.clone());
            let decoded = db.decode(&frame).unwrap();
            for name_value in [true, false] {
                let opts = crate::engine::output::JsonOptions {
                    name_value,
                    ..Default::default()
                };
                let mut line = String::new();
                crate::engine::output::write_json(&mut line, &decoded, &opts).unwrap();
                let back = decoded_frame_from_json(db, &line)
                    .unwrap_or_else(|e| panic!("{pgn} nv={name_value}: {e}\n{line}"))
                    .unwrap();
                assert_eq!(back.data.to_vec(), data, "{pgn} nv={name_value}: {line}");
            }
        }
    }

    /// A value the decoder left out is written as it was left out only
    /// when reading what canboat decoded (`convert --from json`); for a
    /// command meant for the bus (`interface`) it is an error, so a value
    /// forgotten is not sent as "unknown".
    /// A key/value value written as its lookup name, as plain JSON has
    /// it, encodes by the record's key; one given as hex is its bytes.
    #[test]
    fn a_key_value_value_by_its_name_encodes_by_its_key() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let record = |value: &str| {
            format!(
                r#"{{"prio":3,"src":30,"dst":255,"pgn":130845,"fields":{{"manufacturerCode":1857,"industryCode":4,"address":255,"networkGroup":2,"key":53760,"operation":1,"value":{value}}}}}"#
            )
        };
        let by_name = frame_from_json(db, &record(r#""On""#)).unwrap().unwrap();
        let by_hex = frame_from_json(db, &record(r#""01""#)).unwrap().unwrap();
        assert_eq!(by_name.data, by_hex.data);
        assert_eq!(by_name.data[by_name.data.len() - 1], 0x01);
    }

    /// A key given by its name encodes as its code; a name the lookup
    /// does not have is refused.
    #[test]
    fn a_key_by_its_name_encodes_as_its_code() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let line = r#"{"prio":3,"src":16,"dst":255,"pgn":130824,"fields":{"manufacturerCode":381,"industryCode":4,"list":[{"key":"Outside Temperature (F channel)","length":2,"value":288.15}]}}"#;
        let by_name = frame_from_json(db, line).unwrap().unwrap();
        let by_code = line.replace(r#""Outside Temperature (F channel)""#, "28");
        let by_code = frame_from_json(db, &by_code).unwrap().unwrap();
        assert_eq!(by_name.data, by_code.data);
        let unknown = line.replace("Outside Temperature (F channel)", "No Such Key");
        assert!(frame_from_json(db, &unknown).is_err());
    }

    /// A BITLOOKUP given as an empty list, as the decoder writes one with no
    /// bits set, encodes as 0; one left out is unknown, all ones
    /// (canboat#1147).
    #[test]
    fn an_empty_bitlookup_list_encodes_as_no_bits_set() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let line = r#"{"prio":2,"src":4,"dst":255,"pgn":127489,"fields":{"instance":0,"discreteStatus1":[],"discreteStatus2":[]}}"#;
        let frame = frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(frame.data[20..24], [0x00; 4]);
        let line = line.replace(r#","discreteStatus1":[],"discreteStatus2":[]"#, "");
        let frame = frame_from_json(db, &line).unwrap().unwrap();
        assert_eq!(frame.data[20..24], [0xff; 4]);
    }

    /// Under -debug a bit lookup is {"value":[…],"bytes":…}: the list
    /// encodes as it does bare, empty or not (canboat#1147).
    #[test]
    fn a_debug_bitlookup_list_encodes_its_bits() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let data: Vec<u8> = "0b,00,00,ff,ff,b9,79,46,0a,ff,ff,c8,d3,ea,00,ff,ff,ff,ff,ff,\
             05,00,00,00,ff,ff"
            .split(',')
            .map(|h| u8::from_str_radix(h.trim(), 16).unwrap())
            .collect();
        let frame = crate::engine::RawFrame::new(None, 2, 127489, 4, 255, data.clone());
        let decoded = db.decode(&frame).unwrap();
        for name_value in [false, true] {
            let opts = crate::engine::output::JsonOptions {
                debug: true,
                name_value,
                ..Default::default()
            };
            let mut line = String::new();
            crate::engine::output::write_json(&mut line, &decoded, &opts).unwrap();
            assert!(line.contains(r#""discreteStatus2":{"value":[]"#), "{line}");
            let back = frame_from_json(db, &line).unwrap().unwrap();
            assert_eq!(back.data[..], data[..], "{line}");
        }
    }

    /// A BINARY field that starts mid-byte is its value, from bit 0, both
    /// ways: 130829's Helm "0A" goes back to the high nibble of byte 3
    /// (canboat#1143).
    #[test]
    fn a_mid_byte_binary_field_encodes_as_its_value() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let line = r#"{"prio":7,"src":2,"dst":255,"pgn":130829,"fields":{"manufacturerCode":144,"industryCode":4,"fieldA":"05","subHelm":"03","helm":"0A","capabilities":"01"}}"#;
        let frame = frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(frame.data[..], [0x90, 0x98, 0x5f, 0xa3, 0x01]);
    }

    /// A record whose length leaves no value bytes, its value left out,
    /// writes nothing for it rather than failing.
    #[test]
    fn a_zero_length_value_left_out_writes_nothing() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let line = r#"{"prio":3,"src":16,"dst":255,"pgn":130824,"fields":{"manufacturerCode":381,"industryCode":4,"list":[{"key":11,"length":0},{"key":11,"length":2,"value":0}]}}"#;
        let frame = decoded_frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(
            frame.data[..],
            [0x7d, 0x99, 0x0b, 0x00, 0x0b, 0x20, 0x00, 0x00]
        );
    }

    #[test]
    fn a_value_left_out_is_an_error_unless_read_as_decoded() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        // B&G 130824, Trip 2 Time (key 267) without its value.
        let line = r#"{"prio":3,"src":38,"dst":255,"pgn":130824,"fields":{"manufacturerCode":381,"industryCode":4,"list":[{"key":105,"length":2,"value":5.2575},{"key":267,"length":4}]}}"#;
        assert!(frame_from_json(db, line).is_err());
        let frame = decoded_frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(frame.data[frame.data.len() - 4..], [0xff; 4]);
        // A signed value's unknown is its largest positive value: Simnet
        // 130845's Timezone offset (key 41) left out decodes left out
        // again, not as -1 minute.
        let line = r#"{"prio":3,"src":30,"dst":255,"pgn":130845,"fields":{"manufacturerCode":1857,"industryCode":4,"address":255,"networkGroup":2,"key":41,"operation":1}}"#;
        let frame = decoded_frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(frame.data[frame.data.len() - 2..], [0xff, 0x7f]);
    }

    /// A count the decoder left out, as 129029's Reference Stations when
    /// it is 0xff, is written back as 0xff when reading what canboat
    /// decoded; a record written by hand still gets the number of its
    /// instances (canboat#812).
    #[test]
    fn a_count_left_out_is_unavailable_when_read_as_decoded() {
        let db = PgnDatabase::embedded(crate::engine::Units::Si);
        let line = r#"{"prio":3,"src":21,"dst":255,"pgn":129029,"fields":{"sid":196,"gnssType":0,"method":0,"integrity":0,"numberOfSvs":9}}"#;
        let decoded = decoded_frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(decoded.data.len(), 43);
        assert_eq!(decoded.data[42], 0xff);
        let by_hand = frame_from_json(db, line).unwrap().unwrap();
        assert_eq!(by_hand.data[42], 0x00);
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
    fn reserved_and_spare_hex_round_trips() {
        // The analyzer prints a Reserved field that isn't all ones as hex
        // (canboat#1114). It must encode back to the same bits, and each
        // of a PGN's several Reserved fields must land in its own place.
        let line = r#"{"heave":{"prio":3,"src":19,"dst":255,"pgn":127252,
            "fields":{"sid":0,"heave":-0.04,"reserved6":"7F FD"}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(
            frame.data.as_slice(),
            &[0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xfd]
        );

        // 127258: a 4-bit Reserved in the high nibble of byte 1 is printed
        // as its value ("05"), and `reserved6` is the second Reserved field.
        let line = r#"{"magneticVariation":{"prio":7,"src":33,"dst":255,"pgn":127258,
            "fields":{"sid":199,"source":{"value":0},"reserved":"05","reserved6":"CF 2A"}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(frame.data[..2], [0xc7, 0x50]);
        assert_eq!(frame.data[6..], [0xcf, 0x2a]);
    }

    #[test]
    fn a_conditional_reserved_is_written_where_it_was_read() {
        // 126208 Read Fields for a proprietary PGN: the Reserved after the
        // manufacturer code starts at bit 3 of its byte, which only the
        // encoder knows. Printed as its value, "02" (canboat#1143).
        let line = r#"{"nmeaReadFieldsGroupFunction":{"prio":3,"src":1,"dst":2,"pgn":126208,
            "fields":{"functionCode":{"value":3},"pgn":{"value":130816},
            "manufacturerCode":{"value":295},"reserved":"02","industryCode":{"value":4},
            "uniqueId":1,"numberOfSelectionPairs":0,"numberOfParameters":0}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(frame.data[4..6], [0x27, 0x91]);
    }

    #[test]
    fn an_unaligned_reserved_crossing_a_byte_round_trips() {
        // 129814's reserved8 sits at bits 51-58: the analyzer prints its
        // value as one byte ("FE"), and all eight bits go back across the
        // byte boundary. The 1-bit spare at bit 50 is "01".
        let line = r#"{"pgn":129814,"prio":6,"src":1,"dst":255,"fields":{
            "sequenceId":24,"messageId":1,"repeatIndicator":0,
            "destinationIndicator":1,"binaryDataFlag":1,"spare":"01",
            "reserved8":"FE","aisTransceiverInformation":0}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(frame.data[6..8], [0xf7, 0x07]);
    }

    #[test]
    fn a_reserved_wider_than_64_bits_round_trips() {
        // 130816's 144-bit Reserved (bytes 5-22) with one byte cleared: the
        // analyzer prints it, and every byte goes back where it was.
        let mut reserved = ["FF"; 18];
        reserved[5] = "00";
        let line = format!(
            r#"{{"bepMarineCzoneZcfBusDistribution":{{"pgn":130816,"prio":7,"src":1,"dst":255,
            "fields":{{"chunkIndex":1,"flag":0,"reserved6":"{}","data":"01 02 03"}}}}}}"#,
            reserved.join(" ")
        )
        .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        let mut expected = [0xff; 18];
        expected[5] = 0x00;
        assert_eq!(frame.data[5..23], expected);
        let decoded = db().decode(&frame).unwrap();
        let field = decoded
            .fields
            .iter()
            .find(|f| f.info.id == "reserved6")
            .unwrap();
        match &field.value {
            crate::engine::FieldValue::Reserved { bytes, .. } => {
                assert_eq!(bytes.as_slice(), &frame.data[5..23])
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn reserved_hex_longer_than_the_field_is_an_error() {
        // 127258's 4-bit Reserved shares a byte with Source: one byte.
        let line = r#"{"magneticVariation":{"prio":7,"src":33,"dst":255,"pgn":127258,
            "fields":{"reserved":"50 00"}}}"#
            .replace('\n', " ");
        assert!(frame_from_json(db(), &line).is_err());
    }

    #[test]
    fn non_ascii_hex_is_an_error() {
        let line = r#"{"magneticVariation":{"prio":7,"src":33,"dst":255,"pgn":127258,
            "fields":{"reserved":"€a"}}}"#
            .replace('\n', " ");
        assert!(frame_from_json(db(), &line).is_err());
    }

    #[test]
    fn an_unset_reserved_wider_than_64_bits_is_all_ones() {
        // 130816 BEP CZone ZCF Bus Distribution: a 144-bit Reserved at bit 40.
        let line = r#"{"bepMarineCzoneZcfBusDistribution":{"prio":7,"src":1,"dst":255,
            "pgn":130816,"fields":{}}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert!(
            frame.data[5..23].iter().all(|&b| b == 0xff),
            "{:02x?}",
            frame.data
        );
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
    fn request_names_a_126720_variant_behind_the_catch_all() {
        // An Airmar DST console reading the sensor's Simulate Mode:
        // a Request for 126720 naming Manufacturer = Airmar, Industry =
        // Marine and Proprietary ID 35. 126720's catch-all is listed
        // first; like the C analyzer, the parameters are matched against
        // the manufacturer variants behind it, so parameter 4 takes
        // airmarSimulateMode's 8-bit Proprietary ID instead of the
        // catch-all's raw Data. Byte-identical to canboatjs.
        let line = r#"{"pgn":126208,"prio":3,"dst":35,"fields":{
            "functionCode":"Request","pgn":126720,"numberOfParameters":3,
            "list":[{"parameter":1,"value":135},{"parameter":3,"value":4},
                    {"parameter":4,"value":35}]}}"#
            .replace('\n', " ");
        let frame = frame_from_json(db(), &line).unwrap().unwrap();
        assert_eq!(
            frame.data.as_slice(),
            &[
                0x00, 0x00, 0xef, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x01, 0x87, 0x00,
                0x03, 0x04, 0x04, 0x23
            ]
        );

        // The decoder resolves the same variant from the wire bytes.
        let decoded = db().decode(&frame).unwrap();
        let id = decoded
            .fields
            .iter()
            .rfind(|f| f.repeat_set == 1)
            .expect("parameter 4's value");
        assert!(
            matches!(
                id.value,
                crate::engine::FieldValue::Lookup { value: 35, .. }
            ),
            "got {:?}",
            id.value
        );
    }

    #[test]
    fn variable_time_target_encodes_from_seconds_or_clock() {
        // A 126208 Command setting 126992 System Time's Time (field 5)
        // through a VARIABLE: seconds, whole or not, bare or -nv, and the
        // clock string all reach the same wire value (0.0001 s units).
        let encode = |value: &str| {
            let line = format!(
                r#"{{"pgn":126208,"prio":3,"dst":42,"fields":{{"Function Code":"Command","PGN":126992,"priority":8,"numberOfParameters":1,"list":[{{"parameter":5,"value":{value}}}]}}}}"#
            );
            frame_from_json(db(), &line).unwrap().unwrap().data.to_vec()
        };
        let want = encode("33020.5");
        for value in [
            r#""09:10:20.5""#,
            r#"{"value":33020.5,"name":"09:10:20.5000"}"#,
        ] {
            assert_eq!(encode(value), want, "{value}");
        }
        // A whole number of seconds is seconds, not the wire count.
        assert_eq!(encode("600"), encode("600.0"));
        assert_eq!(
            encode(r#"{"value":600,"name":"00:10:00"}"#),
            encode("600.0")
        );
    }

    /// A record the decoder read with a range's catch-all, as a Fusion
    /// 130820 message of no known kind, encodes back under its own PGN,
    /// bare (by its description) or wrapped (by the catch-all's id).
    #[test]
    fn a_catch_all_record_encodes_back_under_its_own_pgn() {
        for line in [
            r#"{"pgn":130820,"prio":7,"src":51,"dst":255,"description":"0x1FF00-0x1FFFF: Manufacturer Specific fast-packet non-addressed","fields":{"manufacturerCode":419,"industryCode":4,"data":"70 80 00"}}"#,
            r#"{"0x1ff000x1ffffManufacturerSpecificFastPacketNonAddressed":{"pgn":130820,"prio":7,"src":51,"dst":255,"fields":{"manufacturerCode":419,"industryCode":4,"data":"70 80 00"}}}"#,
        ] {
            let frame = frame_from_json(db(), line).unwrap().unwrap();
            assert_eq!(frame.pgn, 130820, "{line}");
            assert_eq!(frame.data[..], [0xa3, 0x99, 0x70, 0x80, 0x00], "{line}");
        }
        let line = r#"{"pgn":65450,"prio":7,"src":51,"dst":255,"description":"0xFF00-0xFFFF: Manufacturer Proprietary single-frame non-addressed","fields":{"manufacturerCode":419,"industryCode":4,"data":"01 02 03 04 05 06"}}"#;
        let frame = frame_from_json(db(), line).unwrap().unwrap();
        assert_eq!(frame.pgn, 65450);
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
        let heading = |units, value: f64| {
            frame_from_json(
                PgnDatabase::embedded(units),
                &format!(
                    r#"{{"pgn":127250,"src":27,"description":"Vessel Heading","fields":{{"heading":{value}}}}}"#
                ),
            )
        };
        let deg = heading(Units::Metric, 3.5).unwrap().unwrap();
        let rad = heading(Units::Si, 3.5).unwrap().unwrap();
        assert_ne!(
            deg.data.as_slice(),
            rad.data.as_slice(),
            "3.5 deg and 3.5 rad must not encode alike"
        );
        // 3.5 deg = 0.0610865 rad, i.e. 611 in 0.0001 rad units.
        assert_eq!(&deg.data[1..3], &[0x63, 0x02]);
        // 210.9 deg is a heading; 210.9 rad is outside the field's range
        // and is refused rather than wrapped (#970).
        assert_eq!(
            &heading(Units::Metric, 210.9).unwrap().unwrap().data[1..3],
            &[0xc9, 0x8f]
        );
        assert!(heading(Units::Si, 210.9).is_err());
    }
}
