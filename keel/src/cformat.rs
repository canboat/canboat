//! C printf-compatible formatting.
//!
//! The canboat.xml emitter must reproduce analyzer-explain.c's printf output
//! byte-for-byte (DESIGN.md, migration step 1). Rust's format! has no %g, so
//! floats go through libc snprintf - literally the same code path the C
//! analyzer uses. See QUIRKS.md Q6.

use std::ffi::c_char;
use std::ffi::c_int;

// On MSVC, snprintf is inline-only in the UCRT headers - there is no importable
// symbol, so a raw FFI reference fails to link (LNK2019). legacy_stdio_definitions
// provides the real definition. Other targets (glibc, musl, mingw) export it.
#[cfg_attr(target_env = "msvc", link(name = "legacy_stdio_definitions"))]
unsafe extern "C" {
    fn snprintf(s: *mut c_char, n: usize, format: *const c_char, ...) -> c_int;
}

fn cfmt(format: &[u8], value: f64) -> String {
    let mut buf = [0u8; 64];
    let len = unsafe {
        snprintf(
            buf.as_mut_ptr() as *mut c_char,
            buf.len(),
            format.as_ptr() as *const c_char,
            value,
        )
    };
    assert!(len > 0 && (len as usize) < buf.len(), "snprintf failed");
    String::from_utf8_lossy(&buf[..len as usize]).into_owned()
}

/// C printf %g (6 significant digits, trailing zeros stripped).
pub fn c_g(value: f64) -> String {
    cfmt(b"%g\0", value)
}

/// C printf %.15g.
pub fn c_15g(value: f64) -> String {
    cfmt(b"%.15g\0", value)
}

/// `%g` style, but with the fewest significant digits that still round-trip
/// to the same f64 (QUIRKS.md Q6).
///
/// Used only for `Resolution`. Plain `%g` gives 6 significant digits, which is
/// lossy for the binary fractions resolutions actually are: `1/16384` printed
/// as `6.10352e-05`, `2^-38` as `3.63798e-12`. That mattered historically —
/// the bootstrap converter read these back out of the XML, so the truncated
/// value became the stored one, and an explicit `rangeMax` had to be authored
/// alongside to preserve a bound the truncated resolution no longer produced.
///
/// Deliberately keeps `%g`'s formatting (including its switch to exponent
/// notation) rather than Rust's `{}`, so the document's number style stays
/// uniform; only the digit count changes, and only where 6 digits would lose
/// information. A value that already round-trips at 6 digits is untouched.
///
/// The search starts at 6 — `%g`'s own precision — and only ever *adds*
/// digits. Searching from 1 would "shorten" values that `%g` already prints
/// exactly: `10` round-trips as `1e+01`, `60` as `6e+01`, so a
/// fewest-digits-wins rule silently rewrites clean integers into exponent
/// notation. Never printing less than `%g` keeps this strictly a
/// precision-recovery function.
pub fn c_g_roundtrip(value: f64) -> String {
    for p in 6..=17 {
        let s = cfmt(format!("%.{p}g\0").as_bytes(), value);
        if s.parse::<f64>() == Ok(value) {
            return s;
        }
    }
    cfmt(b"%.17g\0", value)
}

/// Escape text for XML element content or a double-quoted attribute value.
///
/// Every attribute keel emits is double-quoted, so `"` is the only quote that
/// needs escaping; `'` is legal as-is in both positions. (The C emitter mixed
/// single- and double-quoted attributes and escaped neither `'` nor, in the
/// FieldTypes/PhysicalQuantities sections, anything at all — QUIRKS Q3/Q4/Q5.)
pub fn xml_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

#[cfg(test)]
mod roundtrip_tests {
    use super::*;

    #[test]
    fn never_shortens_what_g_prints_exactly() {
        // %g already round-trips these; the output must be byte-identical.
        for v in [
            10.0, 60.0, 100.0, 500.0, 1000.0, 2000.0, 4096.0, 36.0, 864.0, 0.01, 0.1, 1.0,
        ] {
            assert_eq!(c_g_roundtrip(v), c_g(v), "{v} must match plain %g");
        }
    }

    #[test]
    fn lengthens_only_where_g_loses_information() {
        // Binary fractions %g truncates: recovered in full, exponent style kept.
        assert_eq!(c_g_roundtrip(1.0 / 16384.0), "6.103515625e-05");
        assert_eq!(c_g_roundtrip(2f64.powi(-38)), "3.637978807091713e-12");
        assert_eq!(c_g_roundtrip(2f64.powi(-11)), "0.00048828125");
        // and every result must parse back to the exact input
        for v in [
            1.0 / 16384.0,
            1.0 / 11.0,
            2f64.powi(-23),
            2.0 * std::f64::consts::PI / 65536.0,
        ] {
            assert_eq!(c_g_roundtrip(v).parse::<f64>().unwrap(), v);
        }
    }
}
