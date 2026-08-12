// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! NMEA 0183 conversion entry points.
//!
//! [`convert_decoded`] takes an already-decoded [`canboat_core::DecodedPgn`]
//! and delegates to the shared struct-path converter
//! ([`crate::n2kd::decoded::convert_nmea0183`]) — the exact code the live
//! `server` pipeline and `n2kd` run, so every producer emits identical
//! sentences. The per-PGN sentence coverage
//! (RSA/HDG/VHW/DPT/VLW/VTG/GLL/RMC/GSA/MWV/MTW) lives once, in
//! [`crate::n2kd::decoded`]. There is no JSON round-trip on the live
//! path; a test-only `convert` rebuilds a record from a JSON fixture for
//! the unit tests below.
//!
//! What also lives here is [`RateLimiter`] — the per-(src, rate-type)
//! 1 Hz gate and the SOG/COG cache — which both this path and the
//! `decoded` converter share.

use std::sync::OnceLock;
use std::time::{Duration, Instant};

use canboat_core::DecodedPgn;
#[cfg(test)]
use canboat_core::PgnDatabase;

use crate::n2kd::decoded::{self, Handles};

/// Per-(src, rate-type) "this is the last time we let one through"
/// timestamps. Mirrors canboat's `rateLimitPassed[256][RATE_COUNT]`.
///
/// Also carries the single-slot SOG/COG cache that canboat C keeps as
/// `g_sog` / `g_cog` globals — refreshed on PGN 129026 and consumed by
/// the PGN 129029 handler when emitting RMC — and the cycling
/// multi-fragment AIVDM sequence id counter (`gps_ais.c::sequenceId`).
pub struct RateLimiter {
    last_passed: [[Option<Instant>; RATE_COUNT]; 256],
    enabled: bool,
    /// `(sog_ms, cog_deg, captured_at)` — None until we've seen at
    /// least one PGN 129026. Only honoured for ≤ 1s after capture.
    last_sog_cog: Option<(f64, f64, Instant)>,
    /// Cycling 0..9 — the next multi-fragment AIVDM message bumps
    /// this and uses the resulting digit as its sequence id, matching
    /// canboat C's `sequenceId` static in `gps_ais.c::aisToNmea0183`.
    pub ais_seq: u8,
}

/// Number of distinct rate-limited sentence classes — the width of the
/// per-source `last_passed` table. Matches [`crate::n2kd::decoded::Rate`],
/// which owns the PGN → slot mapping the struct-path converter uses.
const RATE_COUNT: usize = 10;

impl RateLimiter {
    pub fn new(enabled: bool) -> Self {
        Self {
            last_passed: [[None; RATE_COUNT]; 256],
            enabled,
            last_sog_cog: None,
            ais_seq: 0,
        }
    }

    /// `true` if rate-limiting is on. Used by the `decoded` module's
    /// `should_drop_fast` shim so it can early-out when disabled
    /// without rebuilding the limiter's clock state.
    #[inline]
    pub(crate) fn enabled(&self) -> bool {
        self.enabled
    }

    /// Direct slot access for [`crate::n2kd::decoded`]. `src` is bounds-
    /// checked to `< 256` by its `u8` type; `rate` is indexed against
    /// `RATE_COUNT` and the caller is expected to pass a valid one.
    #[inline]
    pub(crate) fn last_passed_slot(&mut self, src: usize, rate: usize) -> &mut Option<Instant> {
        &mut self.last_passed[src][rate]
    }

    /// Refresh the single-slot SOG / COG cache that the `position`
    /// handler (when it gets struct-path support in a later phase)
    /// will consume for RMC emission.
    #[inline]
    pub(crate) fn record_sog_cog(&mut self, sog_ms: f64, cog_deg: f64) {
        self.last_sog_cog = Some((sog_ms, cog_deg, Instant::now()));
    }

    /// `(sog_ms, cog_deg)` if [`Self::record_sog_cog`] was called
    /// within the last second.
    #[inline]
    pub(crate) fn recent_sog_cog(&self) -> Option<(f64, f64)> {
        let (sog, cog, ts) = self.last_sog_cog?;
        if ts.elapsed() < Duration::from_secs(1) {
            Some((sog, cog))
        } else {
            None
        }
    }
}

/// **Test-only** JSON → sentence adapter: rebuild a [`DecodedPgn`] from
/// one analyzer name-value JSON line and run the shared struct
/// converter. Production paths (the `server` pipeline and `n2kd`) always
/// start from an already-decoded record, so there is deliberately no
/// JSON round-trip in the live 0183 path — that keeps the analyzer JSON
/// options (`--camel`, SI) from ever affecting 0183. Kept here so the
/// tests can drive the converter straight from a JSON fixture.
#[cfg(test)]
pub fn convert(out: &mut String, msg: &str, rate_limiter: &mut RateLimiter) -> usize {
    let Some(decoded) =
        canboat_core::json_to_decoded(msg, PgnDatabase::embedded(canboat_core::Units::Metric))
    else {
        return 0;
    };
    convert_decoded(out, &decoded, rate_limiter)
}

/// Convert an already-rebuilt [`DecodedPgn`] — the parse-once path both
/// the `server` pipeline and `n2kd` use. Wraps
/// [`crate::n2kd::decoded::convert_nmea0183`] with the shared [`Handles`].
pub fn convert_decoded(
    out: &mut String,
    decoded: &DecodedPgn,
    rate_limiter: &mut RateLimiter,
) -> usize {
    decoded::convert_nmea0183(out, decoded, rate_limiter, handles())
}

/// The pre-resolved [`Handles`] the struct-path converter needs, built
/// once against the embedded schema.
fn handles() -> &'static Handles {
    static HANDLES: OnceLock<Handles> = OnceLock::new();
    HANDLES.get_or_init(Handles::new)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hdg_format() {
        // Analyzer name-value JSON: angle fields already in degrees,
        // the `Reference` lookup as `{"value":N,"name":...}`.
        let msg = r#"{"pgn":127250,"src":7,"fields":{"Heading":90.0,"Reference":{"value":1,"name":"Magnetic"},"Deviation":2.0,"Variation":-3.0}}"#;
        let mut out = String::new();
        let mut rl = RateLimiter::new(false);
        let n = convert(&mut out, msg, &mut rl);
        assert_eq!(n, 1);
        assert!(out.starts_with("$AH"), "got {out}");
        assert!(out.contains("HDG,90.0,2.0,E,3.0,W*"));
        assert!(out.ends_with("\r\n"));
    }

    #[test]
    fn mwv_apparent_wind() {
        let msg = r#"{"pgn":130306,"src":7,"fields":{"Wind Speed":5.0,"Wind Angle":90.0,"Reference":{"value":2,"name":"Apparent"}}}"#;
        let mut out = String::new();
        let mut rl = RateLimiter::new(false);
        convert(&mut out, msg, &mut rl);
        assert!(out.contains("MWV,90.0,R,18.0,K,A*"), "got {out}");
    }

    #[test]
    fn mwv_from_camel_json() {
        // End-to-end: an `analyzer -json -nv -camel` line (record wrapped
        // under the pgn id `windData`, fields keyed by camelCase id) must
        // decode and still produce the MWV sentence — the whole point of
        // n2kd consuming camel input.
        let msg = r#"{"windData":{"pgn":130306,"src":7,"fields":{"windSpeed":5.0,"windAngle":90.0,"reference":{"value":2,"name":"Apparent"}}}}"#;
        let mut out = String::new();
        let mut rl = RateLimiter::new(false);
        convert(&mut out, msg, &mut rl);
        assert!(out.contains("MWV,90.0,R,18.0,K,A*"), "got {out}");
    }

    #[test]
    fn mwv_converts_from_si_radians() {
        // An `analyzer -si` stream carries Wind Angle in radians. Decoded
        // against the SI schema, the MWV sentence must still come out in
        // degrees — the converter asks `as_f64_in("deg")` and the core
        // converts. 1.5708 rad ≈ 90°.
        let msg = r#"{"pgn":130306,"src":7,"fields":{"Wind Speed":5.0,"Wind Angle":1.5708,"Reference":{"value":2,"name":"Apparent"}}}"#;
        let decoded =
            canboat_core::json_to_decoded(msg, PgnDatabase::embedded(canboat_core::Units::Si))
                .expect("decode");
        let mut out = String::new();
        let mut rl = RateLimiter::new(false);
        convert_decoded(&mut out, &decoded, &mut rl);
        assert!(
            out.contains("MWV,90.0,R,"),
            "SI radians should render as 90.0°, got {out}"
        );
    }

    #[test]
    fn dpt_format() {
        let msg = r#"{"pgn":128267,"src":3,"fields":{"Depth":12.3,"Offset":0.5}}"#;
        let mut out = String::new();
        let mut rl = RateLimiter::new(false);
        convert(&mut out, msg, &mut rl);
        assert!(out.contains("DPT,12.3,0.5*"), "got {out}");
    }

    #[test]
    fn vtg_format() {
        // 5.144 m/s ≈ 10 kn; COG already in degrees.
        let msg = r#"{"pgn":129026,"src":3,"fields":{"SOG":5.144,"COG":180.0}}"#;
        let mut out = String::new();
        let mut rl = RateLimiter::new(false);
        convert(&mut out, msg, &mut rl);
        assert!(out.contains("VTG,180.0,T"), "got {out}");
        assert!(out.contains(",10.00,N,"), "got {out}");
    }

    #[test]
    fn rate_limit_drops_second_call_within_a_second() {
        let msg = r#"{"pgn":127250,"src":7,"fields":{"Heading":0.0,"Reference":{"value":1,"name":"Magnetic"}}}"#;
        let mut out = String::new();
        let mut rl = RateLimiter::new(true);
        assert_eq!(convert(&mut out, msg, &mut rl), 1);
        let len_after_first = out.len();
        // Same src + same rate-type within a second → dropped.
        convert(&mut out, msg, &mut rl);
        assert_eq!(out.len(), len_after_first);
    }
}
