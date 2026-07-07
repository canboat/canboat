// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Bit-level extraction primitives shared by the field decoder.
//!
//! Lifted from canboat-rs (crates/canboat-core/src/bits.rs, same author &
//! license); kept as a copy because canboat-rs depends on this repo's
//! canboat.json - a crate dependency would be circular.
//!
//! NMEA 2000 packs fields LSB-first within each byte, advancing byte by
//! byte from low to high. A 12-bit field at bit offset 4 of an 8-byte
//! payload therefore occupies the high 4 bits of `data[0]` and the low
//! 8 bits of `data[1]`. Signed fields are two's-complement and sign-
//! extended to `i64`. J1939 offset (Excess-K notation) is applied here
//! when the field defines an `Offset`.

/// Result of an integer field extraction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Extracted {
    /// The decoded raw integer value (sign-extended if signed,
    /// post-offset).
    pub value: i64,
    /// The maximum representable value for this field's bit width
    /// (also sign-shifted if signed). Used to detect the canboat
    /// "not-available" / "error" sentinels.
    pub max: i64,
    /// The unsigned bit pattern that was extracted — before any
    /// sign extension or offset was applied. This is what the
    /// schema-2.4.0 `UnknownValue` / `OutOfRangeValue` /
    /// `ReservedValue` hints are stated against, and it round-trips
    /// losslessly through u64 for any field width up to 64 bits
    /// (where `value: i64` would lose the sign bit and `value as
    /// f64` would lose precision for >53-bit fields).
    pub raw: u64,
}

/// Extract `bits` bits starting at the bit offset `bit_offset` from
/// `data`. Returns `None` if the field starts entirely past the end of
/// `data`; if the field starts in-bounds but runs off the end (a short
/// fast-packet payload), missing bytes are padded with `0xFF` so the
/// caller's `is_unavailable` check still fires. Matches canboat's
/// reassembly behavior since issue #623.
///
/// Mirrors `extractNumber()` in `analyzer/print.c`.
pub fn extract_bits(
    data: &[u8],
    bit_offset: usize,
    bits: usize,
    signed: bool,
    offset: i64,
) -> Option<Extracted> {
    if bits == 0 || bits > 64 {
        return None;
    }

    // Advance the data slice past whole-byte offsets.
    let byte_offset = bit_offset >> 3;
    if byte_offset >= data.len() {
        return None;
    }
    let mut data = &data[byte_offset..];
    let mut first_bit = bit_offset & 7;

    let mut value: u64 = 0;
    let mut max_v: u64 = 0;
    let mut magnitude: usize = 0;
    let mut remaining = bits;

    // canboat C's `analyzer.c:1009` truncates the field's bit length
    // to whatever fits in the remaining payload before calling
    // `extractNumber`:
    //
    //   bytes = min(bytes_needed, dataLen - startBit/8);
    //   bits = min(bytes * 8, bits);
    //
    // So a 32-bit DISTANCE_FIX32_MMM at the truncated tail of a
    // fast-packet becomes a 16-bit extract instead of a decode error.
    // Mirror that truncation here — earlier behaviour padded with
    // 0xFF (gave wrong values like -0.65536) and the previous fix
    // returned None (dropped the field, but C emits the truncated
    // value). Both diverged from canboat's actual output.
    let mut bits_extracted = 0usize;
    while remaining > 0 {
        let Some(byte) = data.first().copied() else {
            break;
        };
        let in_this_byte = (8 - first_bit).min(remaining);
        let all_ones = (1u64 << in_this_byte) - 1;
        let mask = all_ones << first_bit;
        let chunk = ((byte as u64) & mask) >> first_bit;
        value |= chunk << magnitude;
        max_v |= all_ones << magnitude;
        magnitude += in_this_byte;
        remaining -= in_this_byte;
        first_bit += in_this_byte;
        bits_extracted += in_this_byte;
        if first_bit >= 8 {
            first_bit -= 8;
            if !data.is_empty() {
                data = &data[1..];
            }
        }
    }
    // No bytes at all → caller treats as NotAvailable; same as before.
    if bits_extracted == 0 {
        return None;
    }
    // Note: if `bits_extracted < bits`, the canboat C behaviour is
    // to interpret the value over the *truncated* field width. The
    // `bits` shadow we use below for sign extension reflects the
    // actual width we managed to read.
    let bits = bits_extracted;

    let mut value_signed = value as i64;
    let mut max_signed = max_v as i64;

    if signed {
        // For signed fields, max is halved (positive range only) and
        // negative values are sign-extended to i64.
        max_signed = (max_v >> 1) as i64;
        if offset != 0 {
            // J1939 Excess-K notation — caller-supplied offset shifts
            // the representable range. Apply to both value and max.
            value_signed = value as i64 + offset;
            max_signed += offset;
        } else {
            let sign_bit = 1u64 << (bits - 1);
            if value & sign_bit != 0 {
                // Sign-extend by OR-ing in the inverse of the unsigned
                // max — exactly canboat's `*value |= ~maxv` trick.
                let extended = value as i64 | !(max_v as i64);
                value_signed = extended;
            }
        }
    } else if offset != 0 {
        value_signed = value as i64 + offset;
        max_signed += offset;
    }

    Some(Extracted {
        value: value_signed,
        max: max_signed,
        raw: value,
    })
}

/// Canboat "not-available" / "error" sentinel detector.
///
/// Mirrors `extractNumberNotEmpty()` in `analyzer/print.c`:
///   - 1-bit fields: no sentinel
///   - 2-bit (max=3): top value reserved (N/A)
///   - 3+-bit (max>=7): top two values reserved (error & unknown)
pub fn is_unavailable(extracted: Extracted) -> bool {
    let reserved: i64 = if extracted.max >= 7 {
        2
    } else if extracted.max > 1 {
        1
    } else {
        0
    };
    reserved > 0 && extracted.value > extracted.max - reserved
}

/// One of the four classifications [`classify_sentinel`] returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sentinel {
    /// Raw value is a legitimate reading.
    None,
    /// "Data not available" — `UnknownValue` hit, or (when no hints
    /// are present) the bit-width-heuristic top sentinel.
    Unknown,
    /// `OutOfRangeValue` hit. Schema 2.4.0 only.
    OutOfRange,
    /// `ReservedValue` hit — top-of-range value reserved for future
    /// use. Schema 2.4.0 only. Distinct from the `RESERVED` FieldType
    /// (which marks an explicit reserved/padding field in the schema).
    Reserved,
}

/// Classify a numeric extraction against the field's schema-2.4.0
/// sentinel hints.
///
/// All three of [`unknown`, `out_of_range`, `reserved`] are matched
/// against [`Extracted::raw`] — the pre-sign-extension / pre-offset
/// unsigned bit pattern that the schema values are stated against.
///
/// When **all three are `None`**, the field reserves no sentinels and
/// every raw value is a legitimate reading — the call returns
/// [`Sentinel::None`]. This is the canboat 2.4.0 "all values valid"
/// idiom for fields like `Device Instance`, MMSI, PGN, FIELD_INDEX,
/// and the ~5 % of NUMBER fields where the declared range spans the
/// full bit width. There is **no fallback to a bit-width heuristic**:
/// the schema is fully prescriptive, and the pre-2.4.0 heuristic
/// flagged legitimate near-max values as unavailable.
pub fn classify_sentinel(
    extracted: Extracted,
    unknown: Option<u64>,
    out_of_range: Option<u64>,
    reserved: Option<u64>,
) -> Sentinel {
    let raw = extracted.raw;
    if unknown == Some(raw) {
        Sentinel::Unknown
    } else if out_of_range == Some(raw) {
        Sentinel::OutOfRange
    } else if reserved == Some(raw) {
        Sentinel::Reserved
    } else {
        Sentinel::None
    }
}

/// Yes/no wrapper around [`classify_sentinel`] — `true` iff `raw`
/// matches any of the three hint values.
pub fn is_unavailable_with(
    extracted: Extracted,
    unknown: Option<u64>,
    out_of_range: Option<u64>,
    reserved: Option<u64>,
) -> bool {
    classify_sentinel(extracted, unknown, out_of_range, reserved) != Sentinel::None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extracts_8_bit_unsigned() {
        let data = [0xab];
        let e = extract_bits(&data, 0, 8, false, 0).unwrap();
        assert_eq!(e.value, 0xab);
        assert_eq!(e.max, 0xff);
    }

    #[test]
    fn extracts_across_byte_boundary() {
        // Bits 4..16 = high 4 of data[0] (0xa) + all 8 of data[1] (0xbc)
        // → 0xbca little-endian: high-nibble of byte 0 is the LOW bits
        //   of the 12-bit field. So value = 0xbc << 4 | 0xa = 0xbca.
        let data = [0xab, 0xbc];
        let e = extract_bits(&data, 4, 12, false, 0).unwrap();
        assert_eq!(e.value, 0xbca);
        assert_eq!(e.max, 0xfff);
    }

    #[test]
    fn extracts_signed_negative() {
        // 16-bit signed value -2 → 0xFFFE little-endian → data[0]=0xfe data[1]=0xff
        let data = [0xfe, 0xff];
        let e = extract_bits(&data, 0, 16, true, 0).unwrap();
        assert_eq!(e.value, -2);
        // max for 16-bit signed is half of 0xffff → 0x7fff
        assert_eq!(e.max, 0x7fff);
    }

    #[test]
    fn extracts_signed_positive() {
        let data = [0x05, 0x00];
        let e = extract_bits(&data, 0, 16, true, 0).unwrap();
        assert_eq!(e.value, 5);
    }

    #[test]
    fn detects_unavailable_8bit() {
        // 8-bit field, value = 0xff (max). reserved = 2 (since max=255>=7),
        // threshold = 253; 255 > 253 → unavailable.
        let e = Extracted {
            value: 255,
            max: 255,
            raw: 255,
        };
        assert!(is_unavailable(e));
        // 254 also reserved
        assert!(is_unavailable(Extracted {
            value: 254,
            max: 255,
            raw: 254,
        }));
        // 253 valid
        assert!(!is_unavailable(Extracted {
            value: 253,
            max: 255,
            raw: 253,
        }));
    }

    #[test]
    fn one_bit_never_unavailable() {
        assert!(!is_unavailable(Extracted {
            value: 1,
            max: 1,
            raw: 1
        }));
        assert!(!is_unavailable(Extracted {
            value: 0,
            max: 1,
            raw: 0
        }));
    }

    #[test]
    fn truncates_field_when_data_runs_out() {
        // Field starts in-bounds but runs off the end: canboat C
        // (`analyzer.c:1009`) shrinks the field's bit-length to fit
        // the remaining payload. We mirror that — 16 bits requested
        // at offset 0 with only 1 byte gives value=0xAB extracted
        // over 8 bits, max=0xff (not the full 16-bit max).
        let data = [0xab];
        let e = extract_bits(&data, 0, 16, false, 0).unwrap();
        assert_eq!(e.value, 0xab);
        assert_eq!(e.max, 0xff);
    }

    #[test]
    fn returns_none_when_starts_past_end() {
        // Field starts entirely past the buffer end → still None so
        // callers can drop the field rather than emit a synthetic value.
        let data = [0xff];
        assert!(extract_bits(&data, 8, 8, false, 0).is_none());
    }
}
