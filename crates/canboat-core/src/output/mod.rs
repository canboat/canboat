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
    // Scaling by 10^precision can land exactly on .5 for a value that is
    // not really a tie: `5 * 0.0001` is 0.000500000000000000010408…, above
    // the tie, but multiplying by 1000 rounds the product to exactly 0.5 and
    // the information is gone. printf never scales — it rounds off the
    // double's own decimal expansion — so PGN 126208's `Air pressure offset`
    // came out 0.000 where canboat prints 0.001.
    //
    // Hand an exact .5 to core::fmt, which is correctly rounded from the
    // original value: a real tie (127489's 1.890625, a dyadic fraction)
    // still goes to even, and a near-tie goes the way its true value points.
    // Rare, so the fast path below keeps the common case.
    fn lands_on_a_tie(v: f64, precision: usize) -> bool {
        let scaled = v * 10f64.powi(precision as i32);
        scaled.fract().abs() == 0.5
    }
    if lands_on_a_tie(v, precision) {
        return write!(
            w,
            "{:>width$.prec$}",
            v,
            width = min_width,
            prec = precision
        );
    }
    if precision == 0 {
        // Whole-number path: integer fmt. Ties go to even, see below.
        let rounded = v.round_ties_even() as i64;
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
    // Ties go to EVEN, not away from zero. canboat prints through printf,
    // which rounds under the current FP mode -- FE_TONEAREST, i.e. half to
    // even. `f64::round` rounds half away from zero, so an exact tie came out
    // one ulp high: 127489's Power factor is 30976/16384 = 1.890625 exactly,
    // scaling to 189062.5 at precision 5, and printed 1.89063 where canboat
    // prints 1.89062.
    let rounded = scaled.round_ties_even() as i128;
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

/// C `printf("%g")` — how canboat prints FLOAT fields
/// (`fieldPrintFloat`, print.c).
///
/// Six significant digits: `%e` when the decimal exponent is below -4
/// or at least 6, `%f` otherwise, then trailing fractional zeros and a
/// bare `.` are stripped. C's exponent always carries a sign and at
/// least two digits, which Rust's `{:e}` does not.
///
/// This is not the same function as Rust's `{}`, which prints the
/// shortest string that round-trips. The two diverge exactly where a
/// FLOAT field is a widened `f32`: Garmin's `Heading to Steer` is
/// `2.7274` under `%g` and `2.7273988723754883` under `{}`.
///
/// Allocates, unlike the rest of this module — but only on the FLOAT
/// path, which a handful of PGNs use.
pub(crate) fn write_c_g<W: std::fmt::Write>(w: &mut W, v: f64) -> std::fmt::Result {
    const PRECISION: i32 = 6;

    if v.is_nan() {
        return w.write_str("nan");
    }
    if v.is_infinite() {
        return w.write_str(if v > 0.0 { "inf" } else { "-inf" });
    }

    // Take the exponent from the rounded `%e` form rather than from
    // `log10`, so a value that rounds up into the next decade (9.9999995
    // → 1e+01) picks the exponent it will actually be printed with.
    let sci = format!("{:.*e}", (PRECISION - 1) as usize, v);
    let (mantissa, exp_str) = sci.split_once('e').expect("{:e} always emits an 'e'");
    let exp: i32 = exp_str
        .parse()
        .expect("{:e} always emits an integer exponent");

    fn trim(s: &str) -> &str {
        match s.split_once('.') {
            Some(_) => s.trim_end_matches('0').trim_end_matches('.'),
            None => s,
        }
    }

    // Spelled as C's own condition rather than a range test, so it
    // reads against the standard's wording for %g.
    #[allow(clippy::manual_range_contains)]
    let scientific = exp < -4 || exp >= PRECISION;
    if scientific {
        w.write_str(trim(mantissa))?;
        write!(w, "e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs())
    } else {
        let decimals = (PRECISION - 1 - exp).max(0) as usize;
        w.write_str(trim(&format!("{v:.decimals$}")))
    }
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

#[cfg(test)]
mod c_g_tests {
    use super::write_c_g;

    fn g(v: f32) -> String {
        let mut s = String::new();
        write_c_g(&mut s, v as f64).unwrap();
        s
    }

    #[test]
    fn matches_c_printf_g() {
        // Expected strings produced by a C program printing `%g` of the
        // same `(double)` promotions, compiled and run on this platform.
        let cases: &[(f32, &str)] = &[
            (2.727_398_9, "2.7274"), // Garmin Heading to Steer
            (0.0, "0"),
            (-0.0, "-0"),
            (1.0, "1"),
            (10.0, "10"),
            (100_000.0, "100000"),  // last %f exponent
            (1_000_000.0, "1e+06"), // first %e exponent
            (1_234_567.0, "1.23457e+06"),
            (0.0001, "0.0001"), // last %f exponent going down
            (0.00001, "1e-05"), // first %e exponent going down
            (9.999_999, "10"),  // rounds up into the next decade
            (-core::f32::consts::PI, "-3.14159"),
            (123_456.0, "123456"),
            (0.5, "0.5"),
            (1e-10, "1e-10"),
            (1e20, "1e+20"),
            (65535.0, "65535"),
            (2.5e-5, "2.5e-05"),
            (999_999.5, "1e+06"),
            (1e6, "1e+06"),
            (0.000_123_456, "0.000123456"),
        ];
        for (v, want) in cases {
            assert_eq!(&g(*v), want, "%g of {v}");
        }
    }

    #[test]
    fn rounds_a_near_tie_the_way_printf_does() {
        use super::write_fixed_float;
        fn f(v: f64, p: usize) -> String {
            let mut s = String::new();
            write_fixed_float(&mut s, v, p, 0).unwrap();
            s
        }
        // 5 * 0.0001 is 0.000500000000000000010408…, just *above* the
        // tie, so printf rounds it up. Scaling by 1000 collapses the
        // product onto exactly 0.5 and loses that. PGN 126208's
        // `Air pressure offset` in samples/scx20-setting-tool-offsets.raw.
        assert_eq!(f(5.0 * 0.0001, 3), "0.001");
        // A genuine tie — 30976/16384 is exactly 1.890625 — still goes
        // to even, as printf does under FE_TONEAREST. PGN 127489.
        assert_eq!(f(30976.0 / 16384.0, 5), "1.89062");
        // …and the even-side tie rounds up to the even digit.
        assert_eq!(f(0.125, 2), "0.12");
        assert_eq!(f(0.375, 2), "0.38");
    }

    #[test]
    fn handles_non_finite() {
        let mut s = String::new();
        write_c_g(&mut s, f64::NAN).unwrap();
        assert_eq!(s, "nan");
        s.clear();
        write_c_g(&mut s, f64::INFINITY).unwrap();
        assert_eq!(s, "inf");
        s.clear();
        write_c_g(&mut s, f64::NEG_INFINITY).unwrap();
        assert_eq!(s, "-inf");
    }
}
