// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Affine conversions between the unit pairs canboat's two schemas use.
//!
//! A [`PgnDatabase`](crate::PgnDatabase) decodes into either SI or
//! practical/Metric units (see [`crate::Units`]), so a decoded field's
//! value and its [`unit`](crate::FieldInfo::unit) always agree. A
//! consumer that needs a *specific* unit — NMEA 0183 is defined in
//! degrees and °C, for instance — asks for it via
//! [`DecodedField::as_f64_in`](crate::DecodedField::as_f64_in) and this
//! module bridges the two, regardless of which schema produced the field.

/// Convert `v` from unit `from` to unit `to`. Returns `v` unchanged when
/// the units match, and `None` when no known affine bridges them.
///
/// Covers exactly the pairs canboat's `fixupUnit` relates: angle
/// `rad`↔`deg`, angular rate `rad/s`↔`deg/s`, temperature `K`↔`C`
/// (Celsius), pressure `Pa`↔`bar`, and charge `C`↔`Ah` (Coulomb). The
/// `C` string is overloaded (Celsius vs Coulomb) but the source/target
/// pair disambiguates: `C↔K` is temperature, `C↔Ah` is charge.
pub fn convert_unit(v: f64, from: &str, to: &str) -> Option<f64> {
    if from == to {
        return Some(v);
    }
    const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;
    Some(match (from, to) {
        ("rad", "deg") => v * RAD_TO_DEG,
        ("deg", "rad") => v / RAD_TO_DEG,
        ("rad/s", "deg/s") => v * RAD_TO_DEG,
        ("deg/s", "rad/s") => v / RAD_TO_DEG,
        ("K", "C") => v - 273.15,
        ("C", "K") => v + 273.15,
        ("Pa", "bar") => v / 100_000.0,
        ("bar", "Pa") => v * 100_000.0,
        ("C", "Ah") => v / 3600.0,
        ("Ah", "C") => v * 3600.0,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::convert_unit;

    #[test]
    fn identity_and_known_pairs() {
        assert_eq!(convert_unit(1.5, "deg", "deg"), Some(1.5));
        assert!((convert_unit(1.6384, "rad", "deg").unwrap() - 93.8734).abs() < 1e-3);
        assert!((convert_unit(93.8734, "deg", "rad").unwrap() - 1.6384).abs() < 1e-4);
        assert!((convert_unit(300.0, "K", "C").unwrap() - 26.85).abs() < 1e-9);
        assert!((convert_unit(100_000.0, "Pa", "bar").unwrap() - 1.0).abs() < 1e-9);
        assert!((convert_unit(3600.0, "C", "Ah").unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn unknown_pair_is_none() {
        assert_eq!(convert_unit(1.0, "m/s", "kn"), None);
        assert_eq!(convert_unit(1.0, "deg", "K"), None);
    }
}
