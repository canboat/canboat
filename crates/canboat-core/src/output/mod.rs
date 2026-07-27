// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Output formatters for `DecodedPgn`.
//!
//! Both formatters write to a `&mut dyn fmt::Write` — no I/O. The
//! caller decides whether that's stdout, a `String` buffer, or a
//! TCP socket.
//!
//! For v0, the formatters cover the canboat default outputs (text
//! and compact JSON). `-debug` byte/bit annotation, `-empty`,
//! `-nv`, and camelCase variants are option fields on the format
//! options structs that the formatters honor as they grow.

pub mod json;
pub mod text;

pub use json::{CamelCase, JsonOptions, write_json};
pub use text::{GeoFormat, TextOptions, write_text};

use crate::format::timestamp::days_to_ymd;

/// Round-up decimal precision implied by `resolution`, matching
/// canboat's algorithm in `analyzer/print.c`:
///
/// ```text
///   precision = 0
///   for r = resolution; 0 < r < 1.0; r *= 10:
///       precision++
/// ```
///
/// `resolution = 0.01` → 2 decimals; `0.0001` → 4; integer
/// resolutions → 0.
pub(crate) fn precision_for(resolution: f64) -> usize {
    if !resolution.is_finite() || resolution <= 0.0 {
        return 0;
    }
    let mut p = 0usize;
    let mut r = resolution;
    while r > 0.0 && r < 1.0 && p < 10 {
        p += 1;
        r *= 10.0;
    }
    p
}

/// Effective precision for a decoded field: honors the load-time
/// override from canboat's unit fix-up when non-zero, otherwise
/// derives it from the resolution.
pub(crate) fn effective_precision(precision: u8, resolution: Option<f64>) -> usize {
    if precision > 0 {
        precision as usize
    } else {
        precision_for(resolution.unwrap_or(1.0))
    }
}

/// Format `v` as `{:.precision$}` without going through
/// `core::fmt::float::float_to_decimal_common_exact`, which the
/// profiler had as the single biggest leaf in `write_field_value`.
///
/// Scale + round + split-int-and-frac is the standard fast path for
/// fixed-precision output (the algorithm Ryu/lexical-write-float
/// would internally degrade to once you ask for a precision instead
/// of shortest-round-trip). The integer halves go through
/// `lexical-write-integer`, which beats `core::fmt::Display` for
/// `u64` by a wide margin and lets us write straight into a
/// stack-local byte buffer.
///
/// `min_width` adds left padding with spaces — needed by the LAT/LON
/// `%10.7f` path which produces ` 5.1815566` for short longitudes.
/// Pass `0` to disable.
pub(crate) fn write_fixed_float<W: std::fmt::Write>(
    w: &mut W,
    v: f64,
    precision: usize,
    min_width: usize,
) -> std::fmt::Result {
    // Fall back to core::fmt for NaN / inf and for values where
    // `v * 10^precision` would exceed the integer-exact range of f64
    // (2^53). That covers the AIS "Altitude unknown" sentinel
    // (~9.2e12) which canboat C formats via `%.*f` directly.
    if !v.is_finite() || v.abs() >= 1e12 {
        return write!(
            w,
            "{:>width$.prec$}",
            v,
            width = min_width,
            prec = precision
        );
    }
    // Preserve the sign of the original value — `-0.0001` with
    // precision 1 rounds to integer 0 but `core::fmt` (and canboat C)
    // still emit it as `-0.0`. Use `is_sign_negative` to catch that
    // along with proper negatives.
    let neg = v.is_sign_negative();
    if precision == 0 {
        // Whole-number path: round half-away-from-zero, integer fmt.
        let rounded = v.round() as i64;
        let mut buf = itoa::Buffer::new();
        let s = buf.format(rounded.unsigned_abs());
        let signed_len = (if neg && rounded != 0 { 1 } else { 0 }) + s.len();
        for _ in 0..min_width.saturating_sub(signed_len) {
            w.write_char(' ')?;
        }
        if neg && rounded != 0 {
            w.write_char('-')?;
        }
        return w.write_str(s);
    }
    // Scale, round to integer, then emit integer + decimal point + zero-
    // padded fractional digits — no float formatter on the hot path.
    let scale = 10f64.powi(precision as i32);
    let scaled = v * scale;
    let rounded = scaled.round() as i128;
    let abs = rounded.unsigned_abs();
    let pow = 10u128.pow(precision as u32);
    let int_part = (abs / pow) as u64;
    let frac_part = (abs % pow) as u64;
    let mut int_buf = itoa::Buffer::new();
    let int_str = int_buf.format(int_part);
    let mut frac_buf = itoa::Buffer::new();
    let frac_str = frac_buf.format(frac_part);
    let total_len = (if neg { 1 } else { 0 }) + int_str.len() + 1 + precision;
    for _ in 0..min_width.saturating_sub(total_len) {
        w.write_char(' ')?;
    }
    if neg {
        w.write_char('-')?;
    }
    w.write_str(int_str)?;
    w.write_char('.')?;
    for _ in 0..precision.saturating_sub(frac_str.len()) {
        w.write_char('0')?;
    }
    w.write_str(frac_str)
}

/// Format days-since-1970-01-01 as `YYYY.MM.DD` (canboat text style).
pub fn format_date(days: u16, w: &mut dyn std::fmt::Write) -> std::fmt::Result {
    let (y, m, d) = days_to_ymd(days as i64);
    write!(w, "{:04}.{:02}.{:02}", y, m, d)
}

/// Format seconds-since-midnight as `HH:MM:SS[.fff]`. Fractional
/// digits follow `precision`. When `trim_zero_fraction` is set and
/// the fractional part is zero, the `.fff` suffix is omitted — this
/// matches canboat's text-mode `fieldPrintTime`, where the JSON path
/// always shows the fraction and the text path skips it when zero.
pub fn format_time(
    seconds: f64,
    precision: usize,
    trim_zero_fraction: bool,
    w: &mut dyn std::fmt::Write,
) -> std::fmt::Result {
    if !seconds.is_finite() {
        return w.write_str("00:00:00");
    }
    // Negative values render as DURATION-style negatives (canboat does
    // the same for DURATION_FIX*_MS-typed fields, e.g. a B&G key-value
    // Race Timer at "-00:05:00.000").
    let neg = seconds < 0.0;
    let abs = seconds.abs();
    let sign = if neg { "-" } else { "" };
    let whole = abs.trunc() as u64;
    let h = whole / 3600;
    let m = (whole / 60) % 60;
    let s = whole % 60;
    if precision == 0 {
        return write!(w, "{sign}{:02}:{:02}:{:02}", h, m, s);
    }
    let frac = (abs - whole as f64) * 10f64.powi(precision as i32);
    let frac_rounded = frac.round() as u64;
    if trim_zero_fraction && frac_rounded == 0 {
        write!(w, "{sign}{:02}:{:02}:{:02}", h, m, s)
    } else {
        write!(
            w,
            "{sign}{:02}:{:02}:{:02}.{:0width$}",
            h,
            m,
            s,
            frac_rounded,
            width = precision
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn precision_matches_canboat() {
        assert_eq!(precision_for(1.0), 0);
        assert_eq!(precision_for(0.1), 1);
        assert_eq!(precision_for(0.01), 2);
        assert_eq!(precision_for(0.001), 3);
        assert_eq!(precision_for(0.0001), 4);
        // Resolutions >= 1 round to 0 decimals.
        assert_eq!(precision_for(10.0), 0);
        // Edge: zero or negative.
        assert_eq!(precision_for(0.0), 0);
        assert_eq!(precision_for(-1.0), 0);
    }

    #[test]
    fn date_round_trips_known_dates() {
        assert_eq!(days_to_ymd(0), (1970, 1, 1));
        assert_eq!(days_to_ymd(1), (1970, 1, 2));
        // 2022-09-10 → 19245 days since epoch.
        assert_eq!(days_to_ymd(19245), (2022, 9, 10));
        // Leap day 2024-02-29.
        assert_eq!(days_to_ymd(19782), (2024, 2, 29));
    }

    #[test]
    fn time_formats() {
        let mut out = String::new();
        format_time(3661.0, 0, false, &mut out).unwrap();
        assert_eq!(out, "01:01:01");
        let mut out = String::new();
        format_time(3661.5, 3, false, &mut out).unwrap();
        assert_eq!(out, "01:01:01.500");
    }
}
