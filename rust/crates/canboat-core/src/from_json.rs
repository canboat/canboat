// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Rebuild a [`DecodedPgn`] from one analyzer name-value JSON line.
//!
//! This is the near-inverse of [`crate::output::write_json`]: given a
//! line of canboat analyzer JSON (the `-nv` / NameValue variant),
//! reconstruct the [`DecodedPgn`] a live decode would have produced. It
//! is the input adapter that lets a JSON stream feed the same
//! `DecodedPgn`-based pipeline a live device does — the `.j2k` front
//! door to the n2kd / server converters.
//!
//! Both key styles are accepted. Bare `-json` keys fields by their human
//! `name` (`"Wind Angle"`) and carries the record unwrapped. `-camel`
//! wraps each record in `{"<pgnId>":{…}}` and keys fields by their
//! camelCase `id` (`"windAngle"`); the wrapper id also names the exact
//! PGN variant, which we use to pick it precisely (a PGN number can have
//! several). The style is auto-detected per line, so a single reader
//! handles either analyzer flavour (and `-si` is orthogonal — the caller
//! passes the SI or Metric schema as `db`).
//!
//! It reconstructs only what those downstream consumers read: top-level
//! (non-repeating) fields, in the [`FieldValue`] variants the converters
//! match on (`Number`/`Integer`/`Float`/`Lookup`/`Date`/`Time`/`String`/
//! `Mmsi`/`Pgn`). Repeating sets, `data` bytes, and lossless round-trip
//! of every field type are out of scope — this feeds conversion and
//! serving, not wire re-encoding.

use crate::analyzer_json as json;
use crate::db::PgnDatabase;
use crate::decode::{DecodedField, DecodedPgn, FieldValue, build_index_by_order};
use crate::types::{FieldInfo, FieldType};

/// Reconstruct a [`DecodedPgn`] from one analyzer `-nv` JSON line, or
/// `None` if the line has no `pgn`, the PGN is unknown, or it isn't the
/// object we expect.
pub fn json_to_decoded(line: &str, db: &PgnDatabase) -> Option<DecodedPgn> {
    // `-camel` output wraps the record in `{"<pgnId>":{…}}` and keys
    // fields by their camelCase `id`; bare `-json` is unwrapped and
    // keys by human `name`. Detect once so the rest of the reconstruction
    // uses the matching key and, for camel, the exact variant.
    let camel_id = json::camel_wrapper_id(line);
    let use_camel = camel_id.is_some();
    let pgn = json::int(line, "pgn")? as u32;
    let info = match camel_id {
        // The wrapper id names one exact variant of `pgn`; fall back to
        // the PGN number if this schema doesn't know that id.
        Some(id) => db.pgn_by_id(id).or_else(|| db.first_pgn(pgn))?,
        None => db.first_pgn(pgn)?,
    };

    let src = json::int(line, "src").unwrap_or(0) as u8;
    let dst = json::int(line, "dst").unwrap_or(255) as u8;
    let prio = json::int(line, "prio").unwrap_or(0) as u8;
    let timestamp = json::value(line, "timestamp").map(str::to_owned);

    let mut fields = Vec::with_capacity(info.fields.len());
    for fi in info.fields {
        if let Some(value) = field_value_from_json(line, fi, db, use_camel) {
            fields.push(DecodedField {
                info: fi,
                value,
                bit_offset: None,
                bit_length: None,
                repeat_index: None,
                repeat_set: 0,
                overrides: None,
            });
        }
    }
    let index_by_order = build_index_by_order(&fields);

    Some(DecodedPgn {
        timestamp,
        prio,
        pgn,
        src,
        dst,
        description: info.description,
        id: info.id,
        id_is_pinned: info.id_is_pinned,
        data: Vec::new(),
        fields,
        has_repeating_set: [false, false],
        index_by_order,
    })
}

/// Build the [`FieldValue`] for one field from its JSON representation.
/// The key is the field's human `name` for bare `-json`, or its
/// camelCase `id` for `-camel` (`use_camel`). Returns `None` when the
/// field is absent from the line or its type isn't one the converters
/// consume.
fn field_value_from_json(
    line: &str,
    fi: &FieldInfo,
    db: &PgnDatabase,
    use_camel: bool,
) -> Option<FieldValue> {
    use FieldType::*;
    let name = if use_camel { fi.id } else { fi.name };
    match fi.field_type {
        // Lookups (and lookup-shaped fields): `-nv` carries the integer
        // in a nested `"value"`, which `lookup_int` extracts. The
        // resolved name is `&'static` in a live decode; we don't have
        // one here, and no `-nv` consumer needs it (they read the int).
        Some(Lookup | BitLookup | IndirectLookup | DynamicFieldKey | FieldIndex) => {
            let value = match json::lookup_int(line, name) {
                Some(v) => v as u64,
                // Fallback for non-`-nv` JSON (older / hand-written
                // captures): resolve the bare name-string against the
                // field's lookup enumeration. Production `.j2k` is
                // always `-nv`, so this is the slow path.
                None => resolve_lookup_name(db, fi, json::value(line, name)?)?,
            };
            Some(FieldValue::Lookup { value, name: None })
        }
        // `-nv`: {"value":<days>,"name":"YYYY.MM.DD"} — take the raw days.
        Some(Date) => Some(FieldValue::Date(nested_or_bare_int(line, name)? as u16)),
        // `-nv`: {"value":N,"name":"HH:MM:SS.SSSS"}. N is `seconds` for
        // resolution >= 1 s, the raw scaled integer for sub-second — see
        // output::json. Recover `seconds` (what the converters read)
        // accordingly; keep the wire integer in `raw`.
        Some(Time | Duration) => {
            let v = nested_or_bare_int(line, name)?;
            let res = fi.resolution.unwrap_or(1.0);
            let seconds = if res >= 1.0 { v as f64 } else { v as f64 * res };
            Some(FieldValue::Time { raw: v, seconds })
        }
        Some(Mmsi) => Some(FieldValue::Mmsi(mmsi_from_json(line, name)?)),
        Some(Pgn) => Some(FieldValue::Pgn {
            value: nested_or_bare_int(line, name)? as u32,
            description: None,
        }),
        Some(StringFix | StringLz | StringLau | Variable) => {
            Some(FieldValue::String(json::value(line, name)?.to_owned()))
        }
        Some(Float) => Some(FieldValue::Float(json::number(line, name)?)),
        // Plain numerics: an unscaled integer (resolution 1, no unit) is
        // an `Integer` — matching the decoder — so `as_i64` stays exact;
        // everything else is a scaled `Number`.
        Some(Number | Decimal) | None => {
            if is_integer_field(fi) {
                Some(FieldValue::Integer(json::int(line, name)?))
            } else {
                Some(FieldValue::Number(json::number(line, name)?))
            }
        }
        // Binary / IsoName / Reserved / Spare / dynamic-length markers:
        // no converter reads them; skip.
        _ => None,
    }
}

/// Reverse-resolve a lookup `name`-string to its enum integer via the
/// field's lookup enumeration. Only reached for non-`-nv` input.
fn resolve_lookup_name(db: &PgnDatabase, fi: &FieldInfo, name: &str) -> Option<u64> {
    let table = fi
        .lookup_enumeration
        .or(fi.lookup_bit_enumeration)
        .or(fi.lookup_indirect_enumeration)?;
    db.lookup(table)?
        .values
        .iter()
        .find(|v| v.name == name)
        .map(|v| v.value)
}

/// `true` for fields the decoder keeps as a bare integer: resolution of
/// exactly 1 (or unset) and no unit.
fn is_integer_field(fi: &FieldInfo) -> bool {
    fi.resolution.is_none_or(|r| r == 1.0) && fi.unit.is_none()
}

/// Read an integer that may be either bare (`"F":5`) or wrapped by `-nv`
/// (`"F":{"value":5,...}`).
fn nested_or_bare_int(line: &str, field: &str) -> Option<i64> {
    json::lookup_int(line, field).or_else(|| json::int(line, field))
}

/// MMSI is emitted as a 9-digit quoted string (`"F":"123456789"`) or,
/// for primary-key fields under `-nv`, as `{"value":123456789,...}`.
fn mmsi_from_json(line: &str, field: &str) -> Option<u32> {
    if let Some(v) = json::lookup_int(line, field) {
        return u32::try_from(v).ok();
    }
    json::value(line, field)?.trim().parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Units;

    fn field_f64(d: &DecodedPgn, id: &str) -> Option<f64> {
        d.fields
            .iter()
            .find(|f| f.info.id == id)
            .and_then(|f| f.value.as_f64())
    }

    #[test]
    fn camel_and_bare_decode_the_same_record() {
        let db = crate::PgnDatabase::embedded(Units::Metric);
        // PGN 130306 Wind Data, bare `-json -nv` vs `-camel`: the camel
        // line wraps the record under the pgn id and keys fields by their
        // camelCase id. Both must reconstruct the same DecodedPgn.
        let bare = r#"{"prio":2,"src":7,"dst":255,"pgn":130306,"description":"Wind Data","fields":{"Wind Speed":5.0,"Wind Angle":1.2,"Reference":{"value":2,"name":"Apparent"}}}"#;
        let camel = r#"{"windData":{"prio":2,"src":7,"dst":255,"pgn":130306,"description":"windData","fields":{"windSpeed":5.0,"windAngle":1.2,"reference":{"value":2,"name":"Apparent"}}}}"#;

        let b = json_to_decoded(bare, db).expect("bare decodes");
        let c = json_to_decoded(camel, db).expect("camel decodes");

        assert_eq!(b.pgn, 130306);
        assert_eq!(c.pgn, 130306);
        assert_eq!(c.id, "windData");
        assert_eq!(c.src, 7);
        assert_eq!(field_f64(&b, "windSpeed"), Some(5.0));
        assert_eq!(field_f64(&c, "windSpeed"), Some(5.0));
        assert_eq!(field_f64(&b, "windAngle"), Some(1.2));
        assert_eq!(field_f64(&c, "windAngle"), Some(1.2));
        // The lookup keyed by camel id resolves to the same integer.
        assert_eq!(field_f64(&b, "reference"), field_f64(&c, "reference"));
    }

    #[test]
    fn camel_wrapper_detection() {
        // Bare lines open with a record key; camel lines open with the
        // pgn id.
        assert_eq!(
            json::camel_wrapper_id(r#"{"prio":2,"src":7,"pgn":130306,"fields":{}}"#),
            None
        );
        assert_eq!(
            json::camel_wrapper_id(r#"{"timestamp":"x","pgn":130306}"#),
            None
        );
        assert_eq!(
            json::camel_wrapper_id(r#"{"windData":{"pgn":130306}}"#),
            Some("windData")
        );
    }

    #[test]
    fn unknown_camel_id_falls_back_to_pgn_number() {
        // A wrapper id this schema doesn't know still decodes via the
        // PGN number (single-variant fallback), so a stream stays usable.
        let db = crate::PgnDatabase::embedded(Units::Metric);
        let line = r#"{"notARealId":{"prio":2,"src":7,"pgn":130306,"description":"notARealId","fields":{"windAngle":1.2}}}"#;
        let d = json_to_decoded(line, db).expect("falls back to pgn 130306");
        assert_eq!(d.pgn, 130306);
        assert_eq!(d.id, "windData");
    }
}
