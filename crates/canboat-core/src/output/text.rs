// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Text format — matches the canboat C analyzer's default text output.
//!
//! Layout, replicated from `analyzer/analyzer.c:1205` and `print.c`:
//!
//! ```text
//!   <ts> <prio> <src:3> <dst:3> <pgn:6> <description>: <field>; <field>; ...
//! ```
//!
//! The first field is prefixed with a single space (so `:` is followed
//! by two spaces). Subsequent fields are prefixed `; `. Each field is
//! emitted as `<Field Name> = <value>[ <unit>]`. Date/time values use
//! their canboat printable forms.

use std::fmt;

use crate::decode::{DecodedField, DecodedPgn, FieldValue};

use super::{effective_precision, format_date, format_time};

/// Knobs for text output. Reserved for `-si`, `-geo` extensions —
/// for now `show_unavailable` and `debug` only.
#[derive(Debug, Default, Clone)]
pub struct TextOptions {
    /// When true, emit fields whose value is
    /// [`FieldValue::NotAvailable`] (matches `-empty` semantics in the
    /// C analyzer). Default: omit them entirely.
    pub show_unavailable: bool,
    /// When true, append `(bytes = "...", bits = "...")` diagnostic
    /// to each field (matches canboat's `-debug`). Unavailable fields
    /// stay in the output and emit as `Unknown (bytes = "...")`.
    pub debug: bool,
    /// Lat/lon display format — matches canboat's `-geo {dd|dm|dms}`.
    pub geo: GeoFormat,
}

/// Latitude/longitude display format.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum GeoFormat {
    /// `%10.7f` decimal degrees — canboat's default.
    #[default]
    Dd,
    /// `02d 06.3f N` (degrees + decimal minutes + N/S/E/W).
    Dm,
    /// `02d 02' 06.3f"N` (degrees + minutes + decimal seconds).
    Dms,
}

/// Write one decoded PGN as a canboat text line. Does not append a
/// trailing newline — the caller decides.
pub fn write_text<W: fmt::Write>(w: &mut W, pgn: &DecodedPgn, opts: &TextOptions) -> fmt::Result {
    // Header: `<ts> <prio> <src:3> <dst:3> <pgn:6> <description>:`
    if let Some(ts) = &pgn.timestamp {
        w.write_str(&crate::format::normalize_timestamp(ts))?;
        w.write_char(' ')?;
    }
    write!(
        w,
        "{prio} {src:>3} {dst:>3} {pgn:>6} {desc}:",
        prio = pgn.prio,
        src = pgn.src,
        dst = pgn.dst,
        pgn = pgn.pgn,
        desc = pgn.description,
    )?;

    // First-field separator is " " (single space); after `:` that puts
    // a total of two spaces before the first field name. Subsequent
    // separators are "; ".
    let mut sep = " ";
    for f in &pgn.fields {
        // canboat text mode always emits unavailable fields as
        // `Unknown` (see `printEmpty` in print.c). We keep them too,
        // regardless of -debug / -empty.
        // Reserved/Spare drop unconditionally (handled below).
        // Reserved/Spare are noise in text output — drop them
        // unconditionally (canboat does too).
        if matches!(
            f.value,
            FieldValue::Reserved { .. } | FieldValue::Spare { .. }
        ) {
            continue;
        }
        // Fields whose first byte is past the payload end are dropped —
        // canboat's text mode doesn't emit "Manufacturer Information =
        // Unknown" when the payload ran out before the field could
        // start.
        if let Some(bo) = f.bit_offset {
            let payload_bits = (pgn.data.len() as u32).saturating_mul(8);
            if bo >= payload_bits {
                continue;
            }
        }
        // C format string is `"%s %s = "` (sep + space + name + space
        // + = + space). With sep=" " on the first field that yields two
        // spaces after the header's `:`. With sep=";" on subsequent
        // fields it's "; Name = ".
        w.write_str(sep)?;
        w.write_char(' ')?;
        // Repeating fields get their 1-based iteration index appended
        // to disambiguate at the top level (text mode can't nest like
        // JSON's "list":[{...}]). When `repeat_index` is None the field
        // belongs to a forced count=0 iteration past its first field —
        // canboat's `repetition` counter resets to 0 there, so the
        // suffix is suppressed for these fields only.
        if let (Some(iter), true) = (f.repeat_index, f.repeat_set != 0) {
            write!(w, "{name} {iter} = ", name = f.name(), iter = iter + 1)?;
        } else {
            write!(w, "{name} = ", name = f.name())?;
        }
        write_field_value(w, f, opts.geo, opts.debug, &pgn.data)?;
        if opts.debug {
            write_text_debug_suffix(w, f, &pgn.data)?;
        }
        sep = ";";
    }
    Ok(())
}

/// Append the canboat text-mode `(bytes = "FF FF"[, bits = "..."])`
/// diagnostic for one field. Mirrors the JSON `-debug` output —
/// `bytes` is the field's value positioned within its byte slot
/// (uppercase, space-separated), `bits` is the LSB-padded bit string
/// only emitted for fields whose width isn't a whole number of bytes.
fn write_text_debug_suffix<W: fmt::Write>(
    w: &mut W,
    f: &DecodedField,
    payload: &[u8],
) -> fmt::Result {
    let (Some(bo), Some(bl)) = (f.bit_offset, f.bit_length) else {
        return Ok(());
    };
    if bl == 0 {
        return Ok(());
    }
    // Whole-byte-aligned fields (start and length both byte-aligned)
    // emit their underlying payload bytes directly. This is the only
    // way to handle STRING_LAU and other multi-byte fields whose total
    // width exceeds 64 bits; it also matches canboat's behaviour.
    if bo % 8 == 0 && bl % 8 == 0 {
        let start = (bo / 8) as usize;
        let end = ((bo + bl) / 8) as usize;
        let end_clamped = end.min(payload.len());
        if start >= end_clamped {
            return Ok(());
        }
        w.write_str(" (bytes = \"")?;
        for (i, b) in payload[start..end_clamped].iter().enumerate() {
            if i > 0 {
                w.write_char(' ')?;
            }
            write!(w, "{:02X}", b)?;
        }
        w.write_str("\")")?;
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
    let raw = ex.value as u64;
    let shift = bo % 8;
    let byte_span = (shift + bl).div_ceil(8) as usize;
    let shifted: u128 = (raw as u128) << shift;
    w.write_str(" (bytes = \"")?;
    for i in 0..byte_span {
        if i > 0 {
            w.write_char(' ')?;
        }
        let byte = ((shifted >> (i * 8)) & 0xff) as u8;
        write!(w, "{:02X}", byte)?;
    }
    w.write_char('"')?;
    if bl % 8 != 0 {
        w.write_str(", bits = \"")?;
        // Matches canboat's `showBytesOrBits` byte-indexed bit-shift
        // — see the JSON-side comment for the gory details.
        for i in (0..bl).rev() {
            let byte = (raw >> (i / 8)) & 0xff;
            let bit = (byte >> (i % 8)) & 1;
            w.write_char(if bit == 1 { '1' } else { '0' })?;
        }
        w.write_char('"')?;
    }
    w.write_char(')')
}

/// `unit == "deg"` and the field name reads like a coordinate axis.
/// Canboat selects `fieldPrintLatLon` at parser-generation time based
/// on the field's resolution/range; we don't carry that bit, so match
/// by name + unit instead. Same `strstr(fieldName, "ongit")` shape.
fn is_latlon(f: &DecodedField) -> bool {
    if f.unit() != Some("deg") {
        return false;
    }
    let n: &str = f.name();
    n.contains("atitude") || n.contains("ongitude") || n.contains("atit") || n.contains("ongit")
}

/// `%10.7f` (decimal degrees) / `%02ud %6.3f %c` (degrees+minutes) /
/// `%02ud %02u' %06.3f"%c` (degrees+minutes+seconds). Matches canboat
/// `fieldPrintLatLon` in print.c. `is_lon` decides the N/S vs E/W
/// suffix.
fn write_latlon<W: fmt::Write>(w: &mut W, dd: f64, name: &str, geo: GeoFormat) -> fmt::Result {
    if matches!(geo, GeoFormat::Dd) {
        return write!(w, "{:>10.7}", dd);
    }
    let is_lon = name.contains("ongit");
    let dir = if is_lon {
        if dd >= 0.0 { 'E' } else { 'W' }
    } else if dd >= 0.0 {
        'N'
    } else {
        'S'
    };
    let abs = dd.abs();
    let deg = abs.floor();
    let rem = abs - deg;
    match geo {
        GeoFormat::Dm => {
            let minutes = rem * 60.0;
            write!(w, "{:02}d {:6.3} {dir}", deg as u32, minutes)
        }
        GeoFormat::Dms => {
            let mut minutes = (rem * 60.0).floor();
            let mut seconds = rem * 3600.0 - 60.0 * minutes;
            // canboat caps fractional seconds at 59.9995; round up.
            if seconds >= 59.9995 {
                minutes += 1.0;
                seconds = 0.0;
            }
            let mut deg = deg;
            if minutes >= 60.0 {
                deg += 1.0;
                minutes = 0.0;
            }
            write!(
                w,
                "{:02}d {:02}' {:06.3}\"{dir}",
                deg as u32, minutes as u32, seconds
            )
        }
        GeoFormat::Dd => unreachable!(),
    }
}

/// Append the unit for a numeric field, ` <unit>` style.
///
/// This used to synthesize a pseudo-unit `=<match>` for match fields, to stay
/// bug-compatible with canboat C: that stored a match field's expected value
/// in the Field's `unit` string (`"=15"`), and print.c appended it like any
/// real unit, so `Report Type = 15` came out as `Report Type = 15 =15`.
///
/// canboat gave match fields a real `hasMatchValue` / `matchValue` member and
/// dropped the pun (QUIRKS.md Q19), so the stray suffix is gone from the C
/// output and its fixtures. Matching that is now simply: a match value is not
/// a unit, and is not printed as one.
fn write_unit<W: fmt::Write>(w: &mut W, f: &DecodedField) -> fmt::Result {
    if let Some(unit) = &f.unit() {
        write!(w, " {}", unit)?;
    }
    Ok(())
}

fn write_field_value<W: fmt::Write>(
    w: &mut W,
    f: &DecodedField,
    geo: GeoFormat,
    debug: bool,
    payload: &[u8],
) -> fmt::Result {
    match &f.value {
        FieldValue::Number(v) => {
            // Lat/lon are width 10, precision 7, and intentionally
            // suppress the `deg` unit (canboat's fieldPrintLatLon
            // uses `%10.7f` with no unit suffix). Detection mirrors
            // canboat's `fieldPrintLatLon` selection — unit "deg"
            // with a lat/lon-shaped field name. `-geo dm` / `-geo dms`
            // switch to compass-style output.
            if is_latlon(f) {
                return write_latlon(w, *v, f.name(), geo);
            }
            let p = effective_precision(f.precision(), f.resolution());
            write!(w, "{:.*}", p, v)?;
            write_unit(w, f)?;
            Ok(())
        }
        FieldValue::Integer(v) => {
            write!(w, "{}", v)?;
            write_unit(w, f)?;
            Ok(())
        }
        FieldValue::Float(v) => {
            super::write_c_g(w, *v)?;
            if let Some(unit) = &f.unit() {
                write!(w, " {}", unit)?;
            }
            Ok(())
        }
        FieldValue::Binary(bytes) => {
            for (i, b) in bytes.iter().enumerate() {
                if i > 0 {
                    w.write_char(' ')?;
                }
                write!(w, "{:02X}", b)?;
            }
            Ok(())
        }
        FieldValue::Lookup { value, name } => {
            if let Some(n) = name {
                w.write_str(n)
            } else {
                write!(w, "{}", value)
            }
        }
        FieldValue::BitField { bits, .. } => {
            if bits.is_empty() {
                // canboat text-mode prints `None` for a zero-bitmap
                // BITLOOKUP rather than omitting the field.
                w.write_str("None")
            } else {
                for (i, (_, n)) in bits.iter().enumerate() {
                    if i > 0 {
                        w.write_char(',')?;
                    }
                    w.write_str(n)?;
                }
                Ok(())
            }
        }
        FieldValue::String(s) => w.write_str(s),
        FieldValue::Date(d) => format_date(*d, w),
        FieldValue::Time { seconds, .. } => {
            let p = effective_precision(f.precision(), f.resolution());
            format_time(*seconds, p, true, w)
        }
        FieldValue::Mmsi(v) => write!(w, "\"{:09}\"", v),
        FieldValue::Pgn { value, description } => {
            write!(w, "{}", value)?;
            if let Some(desc) = description {
                write!(w, " ({})", desc)?;
            }
            Ok(())
        }
        FieldValue::Reserved { .. } | FieldValue::Spare { .. } => Ok(()),
        FieldValue::IsoName { value, subfields } => {
            // canboat text format: 0x<hex> name = [<sub1>;<sub2>;...]
            // Under -debug each subfield carries its own
            // `(bytes = ..., bits = ...)` relative to the 8-byte NAME
            // payload; the whole-NAME bytes suffix is appended by the
            // top-level walker as usual.
            write!(w, "0x{:x}", value)?;
            if !subfields.is_empty() {
                let name_payload: &[u8] = match f.bit_offset {
                    Some(bo) if bo % 8 == 0 && (bo / 8) as usize + 8 <= payload.len() => {
                        let s = (bo / 8) as usize;
                        &payload[s..s + 8]
                    }
                    _ => &[],
                };
                w.write_str(" name = [")?;
                let mut sep = "";
                for sf in subfields {
                    if matches!(
                        sf.value,
                        FieldValue::NotAvailable
                            | FieldValue::Reserved { .. }
                            | FieldValue::Spare { .. }
                    ) {
                        continue;
                    }
                    w.write_str(sep)?;
                    write!(w, " {name} = ", name = sf.name())?;
                    write_field_value(w, sf, geo, debug, name_payload)?;
                    if debug {
                        write_text_debug_suffix(w, sf, name_payload)?;
                    }
                    sep = ";";
                }
                w.write_str("]")?;
            }
            Ok(())
        }
        FieldValue::NotAvailable => w.write_str("Unknown"),
        FieldValue::OutOfRange { .. } => w.write_str("Out Of Range"),
        FieldValue::ReservedValue { .. } => w.write_str("Reserved"),
        FieldValue::Unsupported { field_type } => write!(w, "<unsupported:{}>", field_type),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode::{DecodedField, DecodedPgn, FieldValue};

    fn sample_pgn() -> DecodedPgn {
        DecodedPgn {
            timestamp: Some("2022-09-10T12:10:16.614Z".into()),
            prio: 6,
            pgn: 60928,
            src: 5,
            dst: 255,
            description: "ISO Address Claim",
            id: "isoAddressClaim",
            id_is_pinned: false,
            data: Vec::new(),
            fields: {
                // PGN 60928 fields 1 and 2 are Unique Number + Manufacturer
                // Code — the exact two records the legacy fixture
                // hand-rolled before Phase 5.
                let info = crate::PgnDatabase::embedded(crate::Units::Metric)
                    .first_pgn(60928)
                    .expect("PGN 60928 present");
                vec![
                    DecodedField {
                        info: &info.fields[0],
                        value: FieldValue::Integer(1_088_507),
                        bit_offset: None,
                        bit_length: None,
                        repeat_index: None,
                        repeat_set: 0,
                        overrides: None,
                    },
                    DecodedField {
                        info: &info.fields[1],
                        value: FieldValue::Lookup {
                            value: 275,
                            name: Some("Navico"),
                        },
                        bit_offset: None,
                        bit_length: None,
                        repeat_index: None,
                        repeat_set: 0,
                        overrides: None,
                    },
                ]
            },
            has_repeating_set: [false, false],
            index_by_order: [i8::MIN; 32],
        }
    }

    #[test]
    fn header_format_matches_canboat() {
        let pgn = sample_pgn();
        let mut out = String::new();
        write_text(&mut out, &pgn, &TextOptions::default()).unwrap();
        // The header is `<ts> <prio> <src:3> <dst:3> <pgn:6> <desc>:`
        // The first field is prefixed with " " yielding ": " before the
        // separator's space and a further space-then-name: total 2
        // spaces between `:` and `Unique`.
        assert!(out.starts_with(
            "2022-09-10T12:10:16.614Z 6   5 255  60928 ISO Address Claim:  Unique Number = 1088507"
        ));
    }

    #[test]
    fn separates_fields_with_semicolon_space() {
        let pgn = sample_pgn();
        let mut out = String::new();
        write_text(&mut out, &pgn, &TextOptions::default()).unwrap();
        assert!(out.contains("; Manufacturer Code = Navico"));
    }

    #[test]
    fn end_to_end_iso_address_claim() {
        // Decode the exact PGN 60928 frame from canboat tests and
        // verify the text header + first three fields render in the
        // expected shape.
        use crate::{PgnDatabase, RawFrame};
        let db = PgnDatabase::embedded(crate::Units::Metric);
        let frame = RawFrame {
            timestamp: Some("2022-09-10T12:10:16.614Z".into()),
            prio: 6,
            pgn: 60928,
            src: 5,
            dst: 255,
            data: smallvec::smallvec![0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0],
        };
        let pgn = db.decode(&frame).unwrap();
        let mut out = String::new();
        write_text(&mut out, &pgn, &TextOptions::default()).unwrap();
        // Canboat reference:
        //   2022-09-10T12:10:16.614Z 6   5 255  60928 ISO Address Claim:
        //     Unique Number = 1088507; Manufacturer Code = Navico; ...
        assert!(
            out.starts_with(
                "2022-09-10T12:10:16.614Z 6   5 255  60928 ISO Address Claim:  \
                 Unique Number = 1088507; Manufacturer Code = Navico;"
            ),
            "got: {}",
            out
        );
    }

    #[test]
    fn unavailable_renders_as_unknown() {
        // canboat text mode emits unavailable fields as "Unknown"
        // (rather than dropping them, which is the JSON behaviour).
        let mut pgn = sample_pgn();
        pgn.fields[1].value = FieldValue::NotAvailable;
        let mut out = String::new();
        write_text(&mut out, &pgn, &TextOptions::default()).unwrap();
        assert!(out.contains("Manufacturer Code = Unknown"));
        assert!(out.contains("Unique Number = 1088507"));
    }
}
