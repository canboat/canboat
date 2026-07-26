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

/// Candidate replacement for %g: Rust's shortest round-trip formatting.
/// Not used for the golden gate; `keel generate --float-style rust` emits
/// with these so the delta against C %g can be audited (QUIRKS.md Q6).
/// Differences: full precision instead of 6/15 significant digits, and no
/// automatic switch to exponent notation.
pub fn rust_g(value: f64) -> String {
    format!("{}", value)
}

pub fn rust_15g(value: f64) -> String {
    format!("{}", value)
}

/// Port of printXML(): escapes & < > " -- and deliberately NOT the
/// apostrophe, even though lookup attribute values are single-quoted
/// (QUIRKS.md Q3). The & replacement must run first.
pub fn xml_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}
