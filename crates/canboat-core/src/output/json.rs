// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! JSON format — matches the canboat C analyzer's `-json` output.
//!
//! Output shape:
//!
//! ```json
//! {"timestamp":"...","prio":N,"src":N,"dst":N,"pgn":N,
//!  "description":"...","fields":{"Name":Value,"Name":Value,...}}
//! ```
//!
//! Compact (no whitespace), keys in canboat order. Field names use the
//! human-readable `Name` from canboat.json by default. The output is
//! written without a trailing newline.
//!
//! JSON is hand-emitted rather than constructed via serde_json so the
//! output bytes match canboat exactly (number formatting, key order,
//! lack of whitespace, escape handling).

use std::fmt::{self, Write as _};

use crate::decode::{DecodedField, DecodedPgn, FieldValue};

use super::{effective_precision, write_fixed_float};

/// Display name for a field under the current camelCase mode.
/// Off → human-readable `name`; Lower → camelCase `id` (already
/// pre-camelized in canboat.json); Upper → `id` with the first
/// character capitalised.
fn field_display_name(field: &DecodedField, mode: CamelCase) -> std::borrow::Cow<'_, str> {
    use std::borrow::Cow;
    match mode {
        CamelCase::Off => Cow::Borrowed(field.name()),
        CamelCase::Lower => Cow::Borrowed(field.id()),
        CamelCase::Upper => Cow::Owned(upper_camel(field.id())),
    }
}

/// Display description for a PGN. canboat uses `id` as the camelized
/// form (e.g. `isoAddressClaim`), so the same id maps cleanly.
fn pgn_display_description(pgn: &DecodedPgn, mode: CamelCase) -> std::borrow::Cow<'_, str> {
    use std::borrow::Cow;
    match mode {
        CamelCase::Off => Cow::Borrowed(pgn.description),
        CamelCase::Lower => Cow::Borrowed(pgn.id),
        CamelCase::Upper => Cow::Owned(upper_camel(pgn.id)),
    }
}

/// Convert a lowerCamelCase string to UpperCamelCase by uppercasing
/// the first character. Other characters are left alone.
fn upper_camel(s: &str) -> String {
    let mut chars = s.chars();
    match chars.next() {
        Some(c) => c.to_uppercase().collect::<String>() + chars.as_str(),
        None => String::new(),
    }
}

/// Knobs for JSON output.
#[derive(Debug, Default, Clone)]
pub struct JsonOptions {
    /// Emit `null` for unavailable fields instead of omitting them
    /// (matches canboat's `-empty`).
    pub include_empty: bool,
    /// Emit lookup values as `{"value":N,"name":"..."}` instead of the
    /// bare name string (matches canboat's `-nv`).
    pub name_value: bool,
    /// Wrap every field in a `{"value":...,"bytes":"..."[,"bits":"..."]}`
    /// object with per-field byte/bit-level diagnostics (matches
    /// canboat's `-debug`).
    pub debug: bool,
    /// Emit field keys + PGN descriptions as camelCase or
    /// UpperCamelCase identifiers instead of canboat's human-
    /// readable form. Matches canboat C's `-camel` / `-upper-camel`
    /// flags. See [`CamelCase`].
    pub camel_case: CamelCase,
}

/// Identifier style selector for `-camel` / `-upper-camel`.
///
/// canboat C builds these by stripping spaces/punctuation from
/// `"Unique Number"` → `"uniqueNumber"` (camelCase) or
/// `"UniqueNumber"` (UpperCamelCase). The Rust port reads the
/// pre-camelized `id` field straight from canboat.json — for
/// [`CamelCase::Upper`] we just capitalise the first character.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum CamelCase {
    /// Use human-readable names (`"Unique Number"`, `"ISO Address
    /// Claim"`). Default; matches canboat C's default output.
    #[default]
    Off,
    /// lowerCamelCase identifiers (`"uniqueNumber"`).
    Lower,
    /// UpperCamelCase identifiers (`"UniqueNumber"`).
    Upper,
}

thread_local! {
    /// Per-thread scratch buffer for the inner `fields` JSON object.
    /// Re-used across calls to amortise the allocation cost — the
    /// profiler showed a fresh `String::new()` per record + a
    /// `raw_vec::finish_grow` chain dominating the JSON writer's
    /// self-time on hot workloads. `RefCell::take()` swaps an empty
    /// String in for the duration of the call so reentrancy panics
    /// rather than corrupts state.
    static FIELDS_BUF: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}

pub fn write_json<W: fmt::Write>(w: &mut W, pgn: &DecodedPgn, opts: &JsonOptions) -> fmt::Result {
    // In camelCase mode (`-camel` / `-upper-camel`) every record is
    // wrapped in `{"<camelId>":{...}}`, keyed on the PGN's camelCase
    // identifier; the trailing `}` is balanced after the body closes.
    // Plain `-json` never wraps. canboat C originally keyed this on the
    // presence of a pinned `camelDescription` (so PGN 130846 wrapped
    // even in plain `-json`); canboat#746 corrected it to key on the
    // `-camel` flag instead. See `analyzer/analyzer.c::printPgn`.
    let wrap = opts.camel_case != CamelCase::Off;
    if wrap {
        w.write_char('{')?;
        w.write_str("\"")?;
        w.write_str(pgn_display_description(pgn, opts.camel_case).as_ref())?;
        w.write_str("\":")?;
    }
    w.write_char('{')?;
    if let Some(ts) = &pgn.timestamp {
        w.write_str("\"timestamp\":")?;
        write_json_string(w, &crate::format::normalize_timestamp(ts))?;
        w.write_char(',')?;
    }
    write!(
        w,
        "\"prio\":{},\"src\":{},\"dst\":{},\"pgn\":{}",
        pgn.prio, pgn.src, pgn.dst, pgn.pgn
    )?;
    w.write_str(",\"description\":")?;
    write_json_string(w, pgn_display_description(pgn, opts.camel_case).as_ref())?;

    // Build the body of the `fields` object into a buffer so we can
    // skip the wrapping `,"fields":{ ... }` entirely when no field
    // ends up emitted (canboat's `printPgn` defers the open separator
    // until the first field — when every field is suppressed it just
    // closes the top-level object without ever opening fields).
    //
    // Re-use a thread-local buffer between calls — its capacity
    // grows once and then stays put across millions of records.
    let mut fields_buf = FIELDS_BUF.with(|c| c.take());
    fields_buf.clear();
    let result = write_json_inner(w, pgn, opts, &mut fields_buf);
    FIELDS_BUF.with(|c| *c.borrow_mut() = fields_buf);
    result?;
    if wrap {
        w.write_char('}')?;
    }
    Ok(())
}

fn write_json_inner<W: fmt::Write>(
    w: &mut W,
    pgn: &DecodedPgn,
    opts: &JsonOptions,
    fields_buf: &mut String,
) -> fmt::Result {
    let mut top_sep = "";
    // Active repeating-list state: which set (1 → "list", 2 → "list2")
    // and which iteration index we're inside.
    let mut current_set: u8 = 0;
    let mut current_iter: Option<u32> = None;

    let payload_bits = (pgn.data.len() as u32).saturating_mul(8);
    for f in &pgn.fields {
        // Fields whose first byte sits entirely past the payload are
        // dropped — canboat suppresses them regardless of `-empty` /
        // `-debug` (e.g. PGN 126998's third STRING_LAU when only two
        // descriptions fit in the wire). Same gate the text formatter
        // already applies.
        if let Some(bo) = f.bit_offset
            && bo >= payload_bits
        {
            continue;
        }
        // Under `-debug` we keep unavailable fields so the byte/bit
        // diagnostic is preserved; otherwise honour the canboat
        // suppress rule.
        if !opts.include_empty && !opts.debug && matches!(f.value, FieldValue::NotAvailable) {
            continue;
        }
        // SPARE fields: canboat C's `fieldPrintSpare` (print.c:900)
        // drops them when zero but renders the bytes as BINARY when
        // non-zero — the comment there calls a non-zero SPARE "an
        // incorrect PGN definition". The field will then be emitted
        // by the value writer's `Spare { bytes, .. }` arm.
        if let FieldValue::Spare { value, .. } = &f.value
            && *value == 0
        {
            continue;
        }
        // Reserved fields whose raw value is all-ones (the "unused"
        // default) are skipped entirely — even under -debug, matching
        // canboat. Other Reserved values flow through and emit their
        // hex string.
        if let FieldValue::Reserved {
            value, bit_length, ..
        } = &f.value
        {
            let max = if *bit_length >= 64 {
                u64::MAX
            } else {
                (1u64 << bit_length) - 1
            };
            if *value == max {
                continue;
            }
        }
        // Empty BITLOOKUPs (no bits set) are dropped in JSON output —
        // canboat doesn't emit them either. The text formatter, by
        // contrast, prints "None" for these (see write_field_value
        // in output/text.rs).
        if !opts.include_empty
            && !opts.debug
            && let FieldValue::BitField { bits, .. } = &f.value
            && bits.is_empty()
        {
            continue;
        }

        // Determine "where is this field going":
        // - non-repeating (repeat_set == 0): top-level field.
        // - repeat_set == 1: under "list".
        // - repeat_set == 2: under "list2".
        // Crossing set boundaries closes the previous list and opens the
        // next; crossing iterations within the same set inserts "},{".
        if f.repeat_set == 0 {
            if current_set != 0 {
                fields_buf.write_str("}]")?;
                current_set = 0;
                current_iter = None;
            }
            fields_buf.write_str(top_sep)?;
            write_json_string(fields_buf, field_display_name(f, opts.camel_case).as_ref())?;
            fields_buf.write_char(':')?;
            write_field_value(fields_buf, f, opts, &pgn.data)?;
            top_sep = ",";
        } else {
            let iter = f.repeat_index.unwrap_or(0);
            if current_set != f.repeat_set {
                if current_set != 0 {
                    fields_buf.write_str("}]")?;
                }
                fields_buf.write_str(top_sep)?;
                let key = if f.repeat_set == 1 {
                    "\"list\":[{"
                } else {
                    "\"list2\":[{"
                };
                fields_buf.write_str(key)?;
                current_set = f.repeat_set;
                current_iter = Some(iter);
                top_sep = ",";
                write_json_string(fields_buf, field_display_name(f, opts.camel_case).as_ref())?;
                fields_buf.write_char(':')?;
                write_field_value(fields_buf, f, opts, &pgn.data)?;
            } else if Some(iter) != current_iter {
                fields_buf.write_str("},{")?;
                current_iter = Some(iter);
                write_json_string(fields_buf, field_display_name(f, opts.camel_case).as_ref())?;
                fields_buf.write_char(':')?;
                write_field_value(fields_buf, f, opts, &pgn.data)?;
            } else {
                fields_buf.write_char(',')?;
                write_json_string(fields_buf, field_display_name(f, opts.camel_case).as_ref())?;
                fields_buf.write_char(':')?;
                write_field_value(fields_buf, f, opts, &pgn.data)?;
            }
        }
    }
    if current_set != 0 {
        fields_buf.write_str("}]")?;
    }
    // Empty-list placeholder: canboat C unconditionally emits
    // `"list":[{` the moment it sees the repeating set's start field,
    // regardless of how many iterations the count produces. When the
    // count is 0 the result is `"list":[{}]`. Mirror that here so
    // `Reference Stations: 0` style records line up byte-for-byte.
    // "Did we actually emit any list fields?" — match the same
    // filtering the JSON walker applies (NotAvailable / Spare are
    // dropped in default mode). pgn.fields.iter().any(...) by itself
    // would over-count, because the count=0 + truncated case still
    // pushes NotAvailable fields onto the decoded set but the JSON
    // emits none of them.
    let renderable = |f: &DecodedField| {
        if let FieldValue::Spare { value, .. } = &f.value
            && *value == 0
        {
            return false;
        }
        if matches!(f.value, FieldValue::NotAvailable) && !opts.include_empty && !opts.debug {
            return false;
        }
        true
    };
    let saw_set1 = pgn
        .fields
        .iter()
        .any(|f| f.repeat_set == 1 && renderable(f));
    let saw_set2 = pgn
        .fields
        .iter()
        .any(|f| f.repeat_set == 2 && renderable(f));
    if pgn.has_repeating_set[0] && !saw_set1 {
        if !fields_buf.is_empty() && current_set == 0 {
            fields_buf.push_str(top_sep);
        }
        fields_buf.push_str("\"list\":[{}]");
    }
    if pgn.has_repeating_set[1] && !saw_set2 {
        if !fields_buf.is_empty() {
            fields_buf.push(',');
        }
        fields_buf.push_str("\"list2\":[{}]");
    }
    if !fields_buf.is_empty() {
        w.write_str(",\"fields\":{")?;
        w.write_str(fields_buf)?;
        w.write_char('}')?;
    }
    w.write_char('}')?; // close top
    Ok(())
}

/// Emit a `bytes` / `bits` suffix for `-debug` output. Caller must
/// have already opened the wrapping object and written the value (and
/// any `name`); this appends the diagnostic keys and nothing else.
///
/// canboat's `bytes` is the field's *raw value bits placed in their
/// byte slot* — not the underlying payload bytes. So a 3-bit field at
/// bit offset 13 with value `4` shows up as `"80"` (4 shifted left by
/// 5 to land at bits 5–7 of one byte), not `"9F"` (the underlying
/// payload byte). For whole-byte-aligned fields the two are the same.
fn write_debug_suffix<W: fmt::Write>(w: &mut W, f: &DecodedField, payload: &[u8]) -> fmt::Result {
    let (Some(bo), Some(bl)) = (f.bit_offset, f.bit_length) else {
        return Ok(());
    };
    if bl == 0 {
        return Ok(());
    }
    // Whole-byte-aligned fields (STRING_LAU, BINARY, fixed strings)
    // emit their underlying payload bytes directly — matching the
    // text formatter's same-named branch. extract_bits below tops out
    // at 64 bits, so without this fall-through wide string/binary
    // fields would render no `bytes` annotation at all.
    if bo % 8 == 0 && bl % 8 == 0 {
        let start = (bo / 8) as usize;
        let end = ((bo + bl) / 8) as usize;
        let end_clamped = end.min(payload.len());
        if start < end_clamped {
            w.write_str(",\"bytes\":\"")?;
            for (i, b) in payload[start..end_clamped].iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            w.write_char('"')?;
        }
        return Ok(());
    }
    use crate::bits::extract_bits;
    let signed = matches!(
        f.value,
        FieldValue::Number(_) if f.unit().is_some()
    );
    let Some(ex) = extract_bits(payload, bo as usize, bl as usize, signed, 0) else {
        return Ok(());
    };
    let raw_unsigned = ex.value as u64;
    let shift = bo % 8;
    let byte_span = ((shift + bl).div_ceil(8)) as usize;
    let shifted: u128 = (raw_unsigned as u128) << shift;
    w.write_str(",\"bytes\":\"")?;
    for i in 0..byte_span {
        if i > 0 {
            w.write_char(' ')?;
        }
        let byte = ((shifted >> (i * 8)) & 0xff) as u8;
        write!(w, "{:02X}", byte)?;
    }
    w.write_char('"')?;
    // `bits`: only emitted when the field width isn't a whole number
    // of bytes — diagnostic for sub-byte fields.
    if bl % 8 != 0 {
        w.write_str(",\"bits\":\"")?;
        // Reproduce canboat's `showBytesOrBits` bit-emission verbatim:
        //   for i in bits-1 .. 0:
        //       byte = (value >> (i / 8)) & 0xff   // shift by BYTE INDEX, not *8
        //       emit (byte >> (i % 8)) & 1
        // The single-bit shifts per byte-index produce a peculiar
        // ordering that doesn't quite represent the value MSB-first,
        // but matches canboat's golden output exactly.
        for i in (0..bl).rev() {
            let byte = (raw_unsigned >> (i / 8)) & 0xff;
            let bit = (byte >> (i % 8)) & 1;
            w.write_char(if bit == 1 { '1' } else { '0' })?;
        }
        w.write_char('"')?;
    }
    Ok(())
}

/// Under `-debug`, every field is wrapped in
/// `{"value":<bare>,"name":"..."?,"bytes":"...","bits":"..."?,"key":true?}`.
///
/// This is canboat's `-debug` JSON form: even Number / String / Float
/// fields that are normally emitted bare get wrapped so the
/// per-field byte/bit annotation is attached.
fn write_field_value_debug<W: fmt::Write>(
    w: &mut W,
    f: &DecodedField,
    opts: &JsonOptions,
    payload: &[u8],
) -> fmt::Result {
    w.write_char('{')?;
    w.write_str("\"value\":")?;
    // The "bare value" emission for the inner `value` key.
    match &f.value {
        FieldValue::Number(v) => {
            let p = effective_precision(f.precision(), f.resolution());
            let min_w = if p == 7 && f.unit() == Some("deg") {
                10
            } else {
                0
            };
            write_fixed_float(w, *v, p, min_w)?;
        }
        FieldValue::Integer(v) => write!(w, "{}", v)?,
        FieldValue::Float(v) => write!(w, "{}", v)?,
        FieldValue::Binary(bytes) => {
            w.write_char('"')?;
            for (i, b) in bytes.iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            w.write_char('"')?;
        }
        FieldValue::Lookup { value, name } => {
            // Without -nv, -debug puts the lookup *string* directly as
            // `value` (matches canboat's `fieldPrintLookup` JSON path
            // when `showJsonValue` is false). With -nv, the full
            // {value, name} object form is kept, and the `name` key is
            // present even when the lookup didn't resolve (so
            // out-of-range values still emit `"name":null`).
            if opts.name_value {
                write!(w, "{},\"name\":", value)?;
                match name {
                    Some(n) => write_json_string(w, n)?,
                    None => w.write_str("null")?,
                }
            } else {
                match name {
                    Some(n) => write_json_string(w, n)?,
                    None => write!(w, "{}", value)?,
                }
            }
        }
        FieldValue::BitField { value, bits } => {
            if bits.is_empty() {
                // Empty BITLOOKUP → null in JSON. Matches canboat's
                // `printEmpty` path for `value == 0` in
                // `fieldPrintBitLookup` (print.c:940-945).
                let _ = value;
                w.write_str("null")?;
            } else if opts.name_value {
                // -nv: array of `{value, name}` objects.
                w.write_char('[')?;
                for (i, (bv, n)) in bits.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    write!(w, "{{\"value\":{},\"name\":", bv)?;
                    write_json_string(w, n)?;
                    w.write_char('}')?;
                }
                w.write_char(']')?;
            } else {
                // Plain JSON: array of bare strings.
                w.write_char('[')?;
                for (i, (_, n)) in bits.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    write_json_string(w, n)?;
                }
                w.write_char(']')?;
            }
        }
        FieldValue::String(s) => write_field_json_string(w, s)?,
        FieldValue::Date(d) => {
            let mut buf = String::with_capacity(10);
            super::format_date(*d, &mut buf)?;
            // -nv keeps the numeric raw + textual name; without -nv the
            // textual name is emitted directly as `value`.
            if opts.name_value {
                write!(w, "{}", d)?;
                w.write_str(",\"name\":")?;
                write_json_string(w, &buf)?;
            } else {
                write_json_string(w, &buf)?;
            }
        }
        FieldValue::Time { raw, seconds } => {
            let p = effective_precision(f.precision(), f.resolution());
            let mut buf = String::with_capacity(12);
            super::format_time(*seconds, p, false, &mut buf)?;
            if opts.name_value {
                // Same scaling rule as the non-debug `Time` arm
                // (canboat C's `fieldPrintTime`): emit `seconds` for
                // resolution >= 1, raw for sub-second resolution.
                let print_value: i64 = if f.resolution().is_some_and(|r| r >= 1.0) {
                    *seconds as i64
                } else {
                    *raw
                };
                write!(w, "{}", print_value)?;
                w.write_str(",\"name\":")?;
                write_json_string(w, &buf)?;
            } else {
                write_json_string(w, &buf)?;
            }
        }
        FieldValue::Mmsi(v) => write!(w, "\"{:09}\"", v)?,
        FieldValue::Pgn { value, description } => {
            // -debug emits the numeric PGN as `value`. The `name` key
            // is only present with -nv — canboat omits it entirely in
            // non-nv even when the PGN is known (and emits `"name":null`
            // for unknown PGNs *only* with -nv).
            write!(w, "{}", value)?;
            if opts.name_value {
                w.write_str(",\"name\":")?;
                match description {
                    Some(desc) => write_json_string(w, desc)?,
                    None => w.write_str("null")?,
                }
            }
        }
        FieldValue::IsoName { value, subfields } => {
            write!(w, "{}", value)?;
            if opts.name_value {
                // -nv + -debug expands the embedded PGN 60928 field
                // set like the non-debug path, but each subfield
                // carries bytes/bits relative to the 8-byte NAME
                // payload; the outer wrapper still gets the whole
                // NAME's `bytes` suffix after the match.
                let name_payload: &[u8] = match f.bit_offset {
                    Some(bo) if bo % 8 == 0 && (bo / 8) as usize + 8 <= payload.len() => {
                        let s = (bo / 8) as usize;
                        &payload[s..s + 8]
                    }
                    _ => &[],
                };
                w.write_str(",\"name\":{")?;
                let mut sep = "";
                for sf in subfields {
                    // Same filtering as the non-debug IsoName arm.
                    if !opts.include_empty && matches!(sf.value, FieldValue::NotAvailable) {
                        continue;
                    }
                    if matches!(sf.value, FieldValue::Spare { .. }) {
                        continue;
                    }
                    w.write_str(sep)?;
                    write_json_string(w, field_display_name(sf, opts.camel_case).as_ref())?;
                    w.write_char(':')?;
                    write_field_value_debug(w, sf, opts, name_payload)?;
                    sep = ",";
                }
                w.write_char('}')?;
            }
        }
        FieldValue::Reserved { bytes, .. } => {
            // Same shape as canboat's Binary: uppercase hex, space-
            // separated.
            w.write_char('"')?;
            for (i, b) in bytes.iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            w.write_char('"')?;
        }
        FieldValue::Spare { .. } | FieldValue::NotAvailable => {
            w.write_str("null")?;
        }
        FieldValue::OutOfRange { .. } => {
            // Schema-2.4.0 `OutOfRangeValue` sentinel — render as the
            // canboat C label. Matches `analyzer/print.c` after PR #672.
            // TODO: thread the raw u64 through `-debug` byte/bit suffix.
            write_json_string(w, "Out Of Range")?;
        }
        FieldValue::ReservedValue { .. } => {
            // Schema-2.4.0 `ReservedValue` sentinel. See above.
            write_json_string(w, "Reserved")?;
        }
        FieldValue::Unsupported { field_type } => {
            let mut buf = String::with_capacity(field_type.len() + 16);
            buf.push_str("<unsupported:");
            buf.push_str(field_type);
            buf.push('>');
            write_json_string(w, &buf)?;
        }
    }
    write_debug_suffix(w, f, payload)?;
    if opts.name_value && f.part_of_primary_key() {
        w.write_str(",\"key\":true")?;
    }
    w.write_char('}')
}

fn write_field_value<W: fmt::Write>(
    w: &mut W,
    f: &DecodedField,
    opts: &JsonOptions,
    payload: &[u8],
) -> fmt::Result {
    if opts.debug {
        return write_field_value_debug(w, f, opts, payload);
    }
    match &f.value {
        FieldValue::Number(v) => {
            let p = effective_precision(f.precision(), f.resolution());
            // canboat's fieldPrintLatLon uses `%10.7f` — width 10
            // + precision 7 — which left-pads short longitudes
            // (`5.1815566` → ` 5.1815566`). We detect that field type
            // by the load-time precision=7 + unit=deg signal set in
            // db.rs.
            let min_w = if p == 7 && f.unit() == Some("deg") {
                10
            } else {
                0
            };
            write_fixed_float(w, *v, p, min_w)
        }
        FieldValue::Integer(v) => {
            // Under -nv, primary-key fields wear an annotation matching
            // canboat's JSON: {"value":N,"key":true}.
            if opts.name_value && f.part_of_primary_key() {
                write!(w, "{{\"value\":{},\"key\":true}}", v)
            } else {
                write!(w, "{}", v)
            }
        }
        FieldValue::Float(v) => {
            // canboat uses %g — Rust's `{}` is acceptably close.
            write!(w, "{}", v)
        }
        FieldValue::Binary(bytes) => {
            // canboat emits binary as uppercase hex with space-separated
            // bytes (matches fieldPrintBinary's `%s%2.02X` w/ " " sep).
            w.write_char('"')?;
            for (i, b) in bytes.iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            w.write_char('"')
        }
        FieldValue::Lookup { value, name } => {
            if opts.name_value {
                w.write_char('{')?;
                write!(w, "\"value\":{}", value)?;
                match (name, opts.include_empty) {
                    // Resolved → always emit the name.
                    (Some(n), _) => {
                        w.write_str(",\"name\":")?;
                        write_json_string(w, n)?;
                    }
                    // Unresolved + -empty: emit null. Matches canboat's
                    // print.c:725-728 path.
                    (None, true) => w.write_str(",\"name\":null")?,
                    // Unresolved + default: omit "name" entirely.
                    // Matches print.c when showJsonEmpty is false.
                    (None, false) => {}
                }
                // Same primary-key annotation rule as Integer.
                if f.part_of_primary_key() {
                    w.write_str(",\"key\":true")?;
                }
                w.write_char('}')
            } else {
                match name {
                    Some(n) => write_json_string(w, n),
                    None => write!(w, "{}", value),
                }
            }
        }
        FieldValue::BitField { bits, value } => {
            if bits.is_empty() {
                // Empty BITLOOKUP → null (canboat's `printEmpty`).
                let _ = value;
                w.write_str("null")
            } else if opts.name_value {
                // -nv: [{"value":bit_value,"name":"..."},...]
                w.write_char('[')?;
                for (i, (bv, n)) in bits.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    write!(w, "{{\"value\":{},\"name\":", bv)?;
                    write_json_string(w, n)?;
                    w.write_char('}')?;
                }
                w.write_char(']')
            } else {
                // Plain JSON: bare-string array.
                w.write_char('[')?;
                for (i, (_, n)) in bits.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    write_json_string(w, n)?;
                }
                w.write_char(']')
            }
        }
        FieldValue::String(s) => write_field_json_string(w, s),
        FieldValue::Date(d) => {
            let mut buf = String::with_capacity(10);
            super::format_date(*d, &mut buf)?;
            if opts.name_value {
                // canboat -nv: {"value":<days>,"name":"YYYY.MM.DD"}
                w.write_str("{\"value\":")?;
                write!(w, "{}", d)?;
                w.write_str(",\"name\":")?;
                write_json_string(w, &buf)?;
                w.write_char('}')
            } else {
                write_json_string(w, &buf)
            }
        }
        FieldValue::Time { raw, seconds } => {
            let p = effective_precision(f.precision(), f.resolution());
            let mut buf = String::with_capacity(12);
            super::format_time(*seconds, p, false, &mut buf)?;
            if opts.name_value {
                // canboat -nv: `{"value":N,"name":"HH:MM:SS.SSSS"}`.
                // `fieldPrintTime` in canboat C scales `value` by
                // `resolution` when resolution >= 1 (so `value` ends
                // up in seconds), and leaves it raw when resolution
                // < 1 (so a 0.0001-s System Time emits the raw
                // 10000-units-per-second integer rather than a
                // fractional second). Mirror that.
                let print_value: i64 = if f.resolution().is_some_and(|r| r >= 1.0) {
                    *seconds as i64
                } else {
                    *raw
                };
                w.write_str("{\"value\":")?;
                write!(w, "{}", print_value)?;
                w.write_str(",\"name\":")?;
                write_json_string(w, &buf)?;
                w.write_char('}')
            } else {
                write_json_string(w, &buf)
            }
        }
        FieldValue::Mmsi(v) => {
            // canboat emits MMSI as a 9-digit zero-padded string. Under
            // -nv, primary-key MMSI fields wear the same {"value":...,
            // "key":true} annotation as primary-key integers.
            if opts.name_value && f.part_of_primary_key() {
                write!(w, "{{\"value\":\"{:09}\",\"key\":true}}", v)
            } else {
                write!(w, "\"{:09}\"", v)
            }
        }
        FieldValue::Pgn { value, description } => {
            if opts.name_value {
                w.write_char('{')?;
                write!(w, "\"value\":{}", value)?;
                if let Some(desc) = description {
                    w.write_str(",\"name\":")?;
                    write_json_string(w, desc)?;
                }
                w.write_char('}')
            } else {
                write!(w, "{}", value)
            }
        }
        FieldValue::Reserved { bytes, .. } => {
            // canboat emits Reserved as the field's bytes hex-
            // stringified, uppercase, space-separated (same shape as
            // Binary).
            w.write_char('"')?;
            for (i, b) in bytes.iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            w.write_char('"')
        }
        FieldValue::Spare { bytes, .. } => {
            // canboat C falls into `fieldPrintBinary` for non-zero
            // SPARE fields (print.c:920), emitting them as the same
            // space-separated uppercase hex string used for Reserved.
            w.write_char('"')?;
            for (i, b) in bytes.iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            w.write_char('"')
        }
        FieldValue::IsoName { value, subfields } => {
            // -nv: {"value":N,"name":{<recursive>}}
            // default: bare N
            if opts.name_value {
                w.write_char('{')?;
                write!(w, "\"value\":{}", value)?;
                w.write_str(",\"name\":{")?;
                let mut sep = "";
                for sf in subfields {
                    // The recursive sub-decode runs the full field set;
                    // drop unavailable subfields (unless -empty) and
                    // collapse Reserved per the parent rules.
                    if !opts.include_empty && matches!(sf.value, FieldValue::NotAvailable) {
                        continue;
                    }
                    if matches!(sf.value, FieldValue::Spare { .. }) {
                        continue;
                    }
                    w.write_str(sep)?;
                    write_json_string(w, field_display_name(sf, opts.camel_case).as_ref())?;
                    w.write_char(':')?;
                    write_field_value(w, sf, opts, payload)?;
                    sep = ",";
                }
                w.write_char('}')?;
                w.write_char('}')
            } else {
                write!(w, "{}", value)
            }
        }
        FieldValue::NotAvailable => w.write_str("null"),
        FieldValue::OutOfRange { .. } => write_json_string(w, "Out Of Range"),
        FieldValue::ReservedValue { .. } => write_json_string(w, "Reserved"),
        FieldValue::Unsupported { field_type } => {
            // Encode as a string so the JSON stays valid; consumers can
            // detect by leading "<".
            let mut buf = String::with_capacity(field_type.len() + 16);
            buf.push_str("<unsupported:");
            buf.push_str(field_type);
            buf.push('>');
            write_json_string(w, &buf)
        }
    }
}

/// Per-byte escape table. Each entry is `Some(replacement)` for
/// bytes that need quoting in a JSON string; `None` for bytes that
/// pass through unchanged. UTF-8 continuation bytes (>= 0x80) all
/// pass through since the escapes are ASCII-only.
const ESCAPE_TABLE: [Option<&str>; 256] = build_escape_table(false);
const ESCAPE_TABLE_WITH_SLASH: [Option<&str>; 256] = build_escape_table(true);

const fn build_escape_table(slash: bool) -> [Option<&'static str>; 256] {
    let mut t: [Option<&'static str>; 256] = [None; 256];
    t[b'"' as usize] = Some("\\\"");
    t[b'\\' as usize] = Some("\\\\");
    t[0x08] = Some("\\b");
    t[0x09] = Some("\\t");
    t[0x0a] = Some("\\n");
    t[0x0c] = Some("\\f");
    t[0x0d] = Some("\\r");
    // Other control characters (< 0x20) use \u00XX form — handled by
    // the slow path. Mark them with a single-char sentinel "*" that
    // the caller treats as "use \uXXXX".
    let mut i = 0u8;
    while i < 0x20 {
        if t[i as usize].is_none() {
            t[i as usize] = Some("*"); // sentinel for "\u00XX"
        }
        i += 1;
    }
    if slash {
        t[b'/' as usize] = Some("\\/");
    }
    t
}

/// Write `s` as a JSON-quoted string, escaping per RFC 8259.
fn write_json_string<W: fmt::Write>(w: &mut W, s: &str) -> fmt::Result {
    write_json_string_with_table(w, s, &ESCAPE_TABLE)
}

/// canboat C's `print_ascii_json_escaped` (analyzer/print.c:1255)
/// is only used for decoded STRING field values, and it escapes `/`
/// as `\/` alongside the usual JSON escapes. Descriptions / field
/// names come from `pgn.h` C-string literals and skip that function,
/// so their `/` characters stay bare. We keep the same split: the
/// standard [`write_json_string`] is unchanged, and this variant —
/// used only by `FieldValue::String` formatters — adds the `/`
/// escape for byte-for-byte parity.
fn write_field_json_string<W: fmt::Write>(w: &mut W, s: &str) -> fmt::Result {
    write_json_string_with_table(w, s, &ESCAPE_TABLE_WITH_SLASH)
}

/// Body of both `write_json_string` flavours. Walks `s` as bytes and
/// emits the longest unescaped run via a single `write_str` call,
/// inserting escape sequences only at the boundaries — replaces a
/// per-char `match` + virtual `write_char` dispatch + UTF-8 decode
/// with a tight ASCII byte loop. Multibyte UTF-8 sequences pass
/// through because every byte of every escape-needing character is
/// in the ASCII range (< 0x80) and continuation bytes are >= 0x80,
/// so they never match an escape table entry.
#[inline]
fn write_json_string_with_table<W: fmt::Write>(
    w: &mut W,
    s: &str,
    table: &[Option<&str>; 256],
) -> fmt::Result {
    w.write_char('"')?;
    let bytes = s.as_bytes();
    let mut start = 0usize;
    for (i, &b) in bytes.iter().enumerate() {
        if let Some(esc) = table[b as usize] {
            // Flush any pending unescaped run.
            if i > start {
                // SAFETY: `start..i` is a valid char boundary because
                // we never escape continuation bytes (escapes are all
                // < 0x80, continuations are >= 0x80).
                w.write_str(unsafe { std::str::from_utf8_unchecked(&bytes[start..i]) })?;
            }
            if esc.as_bytes() == b"*" {
                // \u00XX form for non-special control codes.
                write!(w, "\\u{:04x}", b as u32)?;
            } else {
                w.write_str(esc)?;
            }
            start = i + 1;
        }
    }
    if start < bytes.len() {
        w.write_str(unsafe { std::str::from_utf8_unchecked(&bytes[start..]) })?;
    }
    w.write_char('"')?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode::{DecodedField, FieldValue};

    fn sample_pgn() -> DecodedPgn {
        // PGN 128267 (Water Depth) field 3 is `Offset` — m, res 0.001 —
        // which matches the legacy hand-rolled fixture this test used
        // before Phase 5 (when DecodedField duplicated id/name/unit
        // inline). Use a real &'static FieldInfo so the new shape
        // compiles without us standing up a parallel test schema.
        let info = crate::PgnDatabase::embedded(crate::Units::Metric)
            .first_pgn(128267)
            .expect("PGN 128267 present");
        let offset_field = &info.fields[2];
        DecodedPgn {
            timestamp: Some("2018-10-16T22:25:25.166".into()),
            prio: 3,
            pgn: 128267,
            src: 35,
            dst: 255,
            description: "Water Depth",
            id: "waterDepth",
            id_is_pinned: false,
            data: Vec::new(),
            fields: vec![DecodedField {
                info: offset_field,
                value: FieldValue::Number(0.0),
                bit_offset: None,
                bit_length: None,
                repeat_index: None,
                repeat_set: 0,
                overrides: None,
            }],
            has_repeating_set: [false, false],
            index_by_order: [i8::MIN; 32],
        }
    }

    #[test]
    fn matches_canboat_json_shape() {
        let pgn = sample_pgn();
        let mut out = String::new();
        write_json(&mut out, &pgn, &JsonOptions::default()).unwrap();
        // The sample timestamp lacks a trailing `Z`; the formatter
        // canonicalises it to ISO-8601 UTC on emit.
        assert_eq!(
            out,
            r#"{"timestamp":"2018-10-16T22:25:25.166Z","prio":3,"src":35,"dst":255,"pgn":128267,"description":"Water Depth","fields":{"Offset":0.000}}"#
        );
    }

    #[test]
    fn fields_object_omitted_when_all_unavailable() {
        // Canboat suppresses the `,"fields":{...}` wrapper entirely
        // when no field actually prints — matches `printPgn`'s lazy
        // open of the fields object. We do the same.
        let mut pgn = sample_pgn();
        pgn.fields[0].value = FieldValue::NotAvailable;
        let mut out = String::new();
        write_json(&mut out, &pgn, &JsonOptions::default()).unwrap();
        assert!(
            !out.contains("\"fields\""),
            "fields wrapper leaked through: {out}"
        );
        assert!(out.ends_with("}"), "got: {out}");
    }

    #[test]
    fn include_empty_emits_null() {
        let mut pgn = sample_pgn();
        pgn.fields[0].value = FieldValue::NotAvailable;
        let mut out = String::new();
        write_json(
            &mut out,
            &pgn,
            &JsonOptions {
                include_empty: true,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(out.contains(r#""Offset":null"#), "got: {}", out);
    }

    #[test]
    fn name_value_emits_object_for_lookup() {
        let mut pgn = sample_pgn();
        pgn.fields[0].value = FieldValue::Lookup {
            value: 275,
            name: Some("Navico"),
        };
        // Suppress the sample fixture's resolution / unit so the
        // formatter renders a plain lookup pair without numeric
        // formatting. (Old test set the fields directly; with Phase 5
        // those go through `overrides`.)
        pgn.fields[0].overrides = Some(Box::new(crate::decode::FieldOverrides {
            unit: None,
            resolution: Some(1.0),
            precision: 0,
        }));
        let mut out = String::new();
        write_json(
            &mut out,
            &pgn,
            &JsonOptions {
                name_value: true,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(
            out.contains(r#""Offset":{"value":275,"name":"Navico"}"#),
            "got: {}",
            out
        );
    }

    #[test]
    fn escapes_quotes_in_strings() {
        let mut pgn = sample_pgn();
        pgn.description = r#"He said "hi""#;
        let mut out = String::new();
        write_json(&mut out, &pgn, &JsonOptions::default()).unwrap();
        assert!(
            out.contains(r#""description":"He said \"hi\"""#),
            "got: {}",
            out
        );
    }
}
