// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Affine conversions between the unit pairs canboat's two schemas use.
//!
//! A [`PgnDatabase`](crate::engine::PgnDatabase) decodes into either SI or
//! practical/Metric units (see [`crate::engine::Units`]), so a decoded field's
//! value and its [`unit`](crate::engine::FieldInfo::unit) always agree. A
//! consumer that needs a *specific* unit — NMEA 0183 is defined in
//! degrees and °C, for instance — asks for it via
//! [`DecodedField::as_f64_in`](crate::engine::DecodedField::as_f64_in) and this
//! module bridges the two, regardless of which schema produced the field.

/// Convert `v` from unit `from` to unit `to`. Returns `v` unchanged when
/// the units match, and `None` when no known affine bridges them.
///
/// Covers exactly the pairs canboat's `fixupUnit` relates: angle
/// `rad`↔`deg`, angular rate `rad/s`↔`deg/s`, temperature `K`↔`C`
/// (Celsius), pressure `Pa`↔`bar`, charge `C`↔`Ah` (Coulomb),
/// energy `J`↔`kWh`, dimensionless `ratio`↔`%`/`ppm`/`ppt`, and the SI
/// conversions of volume, flow, speed, density, viscosity, pressure rate,
/// rotation (`rpm`↔`Hz`) and GPS `semi-circle` angles. The
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
        ("J", "kWh") => v / 3.6e6,
        ("kWh", "J") => v * 3.6e6,
        ("ratio", "%") => v * 100.0,
        ("%", "ratio") => v / 100.0,
        ("ratio", "ppm") => v * 1e6,
        ("ppm", "ratio") => v / 1e6,
        ("ratio", "ppt") => v * 1000.0,
        ("ppt", "ratio") => v / 1000.0,
        ("m3", "L") => v * 1000.0,
        ("L", "m3") => v / 1000.0,
        ("m3/s", "L/h") => v * 3.6e6,
        ("L/h", "m3/s") => v / 3.6e6,
        ("m/s", "km/h") => v * 3.6,
        ("km/h", "m/s") => v / 3.6,
        ("kg/s", "kg/h") => v * 3600.0,
        ("kg/h", "kg/s") => v / 3600.0,
        ("kg/m3", "g/cm3") => v / 1000.0,
        ("g/cm3", "kg/m3") => v * 1000.0,
        ("Pa.s", "cP") => v * 1000.0,
        ("cP", "Pa.s") => v / 1000.0,
        ("Pa/s", "Pa/hr") => v * 3600.0,
        ("Pa/hr", "Pa/s") => v / 3600.0,
        ("Hz", "rpm") => v * 60.0,
        ("rpm", "Hz") => v / 60.0,
        ("rad", "semi-circle") => v / std::f64::consts::PI,
        ("semi-circle", "rad") => v * std::f64::consts::PI,
        ("rad/s", "semi-circle/s") => v / std::f64::consts::PI,
        ("semi-circle/s", "rad/s") => v * std::f64::consts::PI,
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
        assert!((convert_unit(3.6e6, "J", "kWh").unwrap() - 1.0).abs() < 1e-9);
        assert!((convert_unit(1.0, "kWh", "J").unwrap() - 3.6e6).abs() < 1e-3);
        assert!((convert_unit(-0.5, "ratio", "%").unwrap() + 50.0).abs() < 1e-9);
        assert!((convert_unit(97.536, "%", "ratio").unwrap() - 0.97536).abs() < 1e-9);
        assert!((convert_unit(1800.0, "rpm", "Hz").unwrap() - 30.0).abs() < 1e-9);
        assert!((convert_unit(12.5, "L/h", "m3/s").unwrap() - 12.5 / 3.6e6).abs() < 1e-15);
        assert!(
            (convert_unit(0.5, "semi-circle", "rad").unwrap() - std::f64::consts::FRAC_PI_2).abs()
                < 1e-12
        );
        assert!((convert_unit(350.0, "ppm", "ratio").unwrap() - 0.00035).abs() < 1e-12);
        for (unit, si) in [
            ("L", "m3"),
            ("km/h", "m/s"),
            ("kg/h", "kg/s"),
            ("g/cm3", "kg/m3"),
            ("cP", "Pa.s"),
            ("Pa/hr", "Pa/s"),
            ("ppt", "ratio"),
            ("semi-circle/s", "rad/s"),
        ] {
            let back = convert_unit(convert_unit(7.25, unit, si).unwrap(), si, unit).unwrap();
            assert!((back - 7.25).abs() < 1e-9, "{unit} <-> {si}");
        }
    }

    #[test]
    fn unknown_pair_is_none() {
        assert_eq!(convert_unit(1.0, "m/s", "kn"), None);
        assert_eq!(convert_unit(1.0, "deg", "K"), None);
    }
}
