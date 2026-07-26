// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Locally-computed magnetic variation (World Magnetic Model 2025).
//!
//! NMEA 2000 devices that report **PGN 127258 Magnetic Variation** carry
//! a *baked-in* World Magnetic Model — usually WMM 2015/2020 — that
//! firmware never updates. The Earth's field drifts fast enough (~50
//! km/yr of pole motion) that a stale model is already off by a
//! meaningful fraction of a degree. canboat already sees position and
//! date on the bus, so with `--quirk wmm` it computes a fresh variation
//! itself using **WMM 2025** and emits its own 127258.
//!
//! Two things happen (both are ordinary quirk `RawFrame` outputs, so
//! they ride the existing bus-write + local-feedback path in
//! [`crate::server::pipeline`]):
//!
//! 1. **Emit** our own PGN 127258 from canboat's own source address,
//!    `Source = WMM 2025`. The *variation* is recomputed only every
//!    [`COMPUTE_INTERVAL`] (it changes very slowly) but the message is
//!    *transmitted* at the PGN's suggested rate ([`EMIT_INTERVAL`]).
//! 2. **Ask older sources to stop**: when another source sends 127258
//!    whose `Source` is an *older WMM epoch* (WMM 2000…2020), send it a
//!    one-shot PGN 126208 Request to stop transmitting 127258 — once per
//!    session per source, so a device that ignores us isn't spammed.
//!
//! Unlike the SCX-20 impersonation quirk, this never spoofs a sensor:
//! the emitted frame is canboat's own node.

use std::collections::HashSet;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use canboat_core::{
    ADDR_GLOBAL, DecodedPgn, FieldRef, PgnDatabase, RawFrame, Units, field, format::days_to_ymd,
    format_iso_ms,
};
use world_magnetic_model::GeomagneticField;
use world_magnetic_model::time::{Date, Month};
use world_magnetic_model::uom::si::angle::{degree, radian};
use world_magnetic_model::uom::si::f32::{Angle, Length};
use world_magnetic_model::uom::si::length::meter;

/// PGNs we observe. Working from the decoded PGN (post-reassembly) means
/// the fast-packet 129029 GNSS Position is available too, not just the
/// single-frame 129025 Rapid Update.
const PGN_POSITION_RAPID: u32 = 129025;
const PGN_GNSS_POSITION: u32 = 129029;
const PGN_SYSTEM_TIME: u32 = 126992;
const PGN_MAGNETIC_VARIATION: u32 = 127258;

/// `MAGNETIC_VARIATION` lookup value for the model we compute with.
/// When WMM 2030 ships (canboat.json adds value 10 and the crate gains
/// the model) this becomes a date→(model, value) mapping.
const SOURCE_WMM_2025: i64 = 9;
/// `MAGNETIC_VARIATION` lookup values for *older* WMM epochs (WMM 2000
/// through WMM 2020) — the sources we ask to stop. Manual / Automatic
/// (0–3) and WMM 2025 (9) are left alone.
const OLDER_WMM: std::ops::RangeInclusive<i64> = 4..=8;

/// How often the variation is recomputed. It changes far slower than
/// this; 10 min keeps the cost negligible on a Pi.
const COMPUTE_INTERVAL: Duration = Duration::from_secs(600);
/// How often we transmit 127258 — the PGN's schema `TransmissionInterval`.
const EMIT_INTERVAL: Duration = Duration::from_millis(1000);

/// Declination (variation) in **radians**, east-positive, at sea level.
/// `None` when the date is outside the model's validity range (a bad or
/// absent clock) — loud, never a silent zero.
pub fn variation_rad(lat_deg: f64, lon_deg: f64, date: Date) -> Option<f64> {
    match GeomagneticField::new(
        Length::new::<meter>(0.0),
        Angle::new::<degree>(lat_deg as f32),
        Angle::new::<degree>(lon_deg as f32),
        date,
    ) {
        Ok(field) => Some(f64::from(field.declination().get::<radian>())),
        Err(e) => {
            log::warn!("quirk(wmm): no variation at {lat_deg:.4},{lon_deg:.4}: {e}");
            None
        }
    }
}

/// Convert an N2K date (16-bit days since 1970-01-01) to a [`Date`].
fn date_from_days(days: u16) -> Option<Date> {
    let (y, mo, d) = days_to_ymd(i64::from(days));
    let month = Month::try_from(mo as u8).ok()?;
    Date::from_calendar_date(y, month, d as u8).ok()
}

/// Days since 1970-01-01 from the host clock — the fallback when the bus
/// hasn't given us a System Time frame yet.
fn host_days() -> u16 {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    (secs / 86_400) as u16
}

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` from the host clock for the emitted frame.
fn now_iso() -> String {
    let ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    format_iso_ms(ms)
}

/// Stateful WMM quirk: observes position / date / other sources' 127258
/// and synthesises our own 127258 (plus one-shot stop-requests).
pub struct WmmQuirk {
    /// SI schema, so `Variation` encodes directly in radians.
    db: &'static PgnDatabase,
    /// Last position seen, degrees (lat, lon).
    last_pos: Option<(f64, f64)>,
    /// Last date seen on the bus (N2K days since 1970); host clock is the
    /// fallback when this is `None`.
    last_days: Option<u16>,
    /// Cached variation in radians; recomputed every [`COMPUTE_INTERVAL`].
    cached_rad: Option<f64>,
    last_compute: Option<Instant>,
    last_emit: Option<Instant>,
    /// Sources already sent a one-shot stop-request this session.
    asked_stop: HashSet<u8>,
}

impl Default for WmmQuirk {
    fn default() -> Self {
        Self::new()
    }
}

impl WmmQuirk {
    pub fn new() -> Self {
        Self {
            db: PgnDatabase::embedded(Units::Si),
            last_pos: None,
            last_days: None,
            cached_rad: None,
            last_compute: None,
            last_emit: None,
            asked_stop: HashSet::new(),
        }
    }

    /// Inspect one decoded PGN; learn from it and maybe synthesise. See
    /// the module docs. `now` is injected so cadence is unit-testable.
    ///
    /// Our own emitted 127258 (fed back through the pipeline) carries
    /// `Source = WMM 2025`, so it is never an older-WMM stop-target, and a
    /// re-emit is blocked by the transmit-interval throttle — so there is
    /// no need to recognise "our own" frame explicitly.
    pub fn process(&mut self, d: &DecodedPgn, now: Instant) -> Vec<RawFrame> {
        let mut out = Vec::new();
        match d.pgn {
            PGN_POSITION_RAPID => self.observe_position(
                d,
                field::position_rapid_update::LATITUDE,
                field::position_rapid_update::LONGITUDE,
            ),
            PGN_GNSS_POSITION => {
                self.observe_position(
                    d,
                    field::gnss_position_data::LATITUDE,
                    field::gnss_position_data::LONGITUDE,
                );
                self.observe_date(d, field::gnss_position_data::DATE);
            }
            PGN_SYSTEM_TIME => self.observe_date(d, field::system_time::DATE),
            PGN_MAGNETIC_VARIATION => {
                if let Some(req) = self.maybe_request_stop(d) {
                    out.push(req);
                }
            }
            _ => {}
        }
        if let Some(f) = self.maybe_emit(now) {
            out.push(f);
        }
        out
    }

    /// Learn lat/lon from a position PGN's latitude/longitude fields
    /// (skips the not-available sentinel).
    fn observe_position(&mut self, d: &DecodedPgn, lat_field: FieldRef, lon_field: FieldRef) {
        let lat = d.field(lat_field).and_then(|f| f.value.as_f64());
        let lon = d.field(lon_field).and_then(|f| f.value.as_f64());
        if let (Some(lat), Some(lon)) = (lat, lon) {
            self.last_pos = Some((lat, lon));
        }
    }

    /// Learn the date (N2K days since 1970) from a PGN's date field.
    fn observe_date(&mut self, d: &DecodedPgn, date_field: FieldRef) {
        if let Some(days) = d.field(date_field).and_then(|f| f.value.as_i64())
            && let Ok(days) = u16::try_from(days)
        {
            self.last_days = Some(days);
        }
    }

    /// If `d` is an older-WMM 127258 from a source we haven't asked yet,
    /// return the one-shot PGN 126208 "stop transmitting 127258".
    fn maybe_request_stop(&mut self, d: &DecodedPgn) -> Option<RawFrame> {
        let source = d
            .field(field::magnetic_variation::SOURCE)
            .and_then(|f| f.value.as_i64())?;
        if !OLDER_WMM.contains(&source) || !self.asked_stop.insert(d.src) {
            return None;
        }
        log::info!(
            "quirk(wmm): asking src={} to stop PGN 127258 (Source={source}, older than WMM 2025)",
            d.src,
        );
        Some(crate::n2kd::overrides::request_interval_frame(
            d.src,
            PGN_MAGNETIC_VARIATION,
            0, // interval 0 = stop transmitting
            None,
            None,
        ))
    }

    /// Emit our 127258 if a position is known and the transmit interval
    /// has elapsed, recomputing the variation at most every
    /// [`COMPUTE_INTERVAL`]. `None` when there's nothing to send yet.
    fn maybe_emit(&mut self, now: Instant) -> Option<RawFrame> {
        let (lat, lon) = self.last_pos?;
        if let Some(last) = self.last_emit
            && now.duration_since(last) < EMIT_INTERVAL
        {
            return None;
        }
        let due = self
            .last_compute
            .is_none_or(|t| now.duration_since(t) >= COMPUTE_INTERVAL);
        if self.cached_rad.is_none() || due {
            let days = self.last_days.unwrap_or_else(host_days);
            // Keep any previously-good value if this recompute fails
            // (e.g. transient bad clock); throttle retries regardless.
            if let Some(rad) = date_from_days(days).and_then(|date| variation_rad(lat, lon, date)) {
                self.cached_rad = Some(rad);
            }
            self.last_compute = Some(now);
        }
        let rad = self.cached_rad?;
        let age_days = self.last_days.unwrap_or_else(host_days);
        let frame = self.build_frame(rad, age_days)?;
        self.last_emit = Some(now);
        Some(frame)
    }

    /// Encode PGN 127258 via the schema-driven message encoder (SI db →
    /// `Variation` in radians). Returns `None` only on an encoder error,
    /// which would be a schema/library bug (logged loudly).
    fn build_frame(&self, variation_rad: f64, age_days: u16) -> Option<RawFrame> {
        let build = || -> Result<RawFrame, canboat_core::encode::EncodeError> {
            let mut b = self
                .db
                .encode("magneticVariation")?
                .priority(7)
                // Emit as "my own node": ADDR_GLOBAL is the sentinel the
                // pipeline rewrites to canboat's live claimed address.
                .source(ADDR_GLOBAL)
                .timestamp(now_iso());
            b.push(field::magnetic_variation::SOURCE, SOURCE_WMM_2025)?;
            b.push(field::magnetic_variation::VARIATION, variation_rad)?;
            b.push(
                field::magnetic_variation::AGE_OF_SERVICE,
                i64::from(age_days),
            )?;
            b.build()
        };
        match build() {
            Ok(frame) => Some(frame),
            Err(e) => {
                log::error!("quirk(wmm): failed to encode PGN 127258: {e}");
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn variation_rad_matches_wmm2025_reference() {
        // Official WMM2025 worked example from the crate/NOAA docs:
        // 37.03°N, 7.91°W, 15th day of 2029 → declination -0.17368°.
        // Their point is at 100 m; fixing altitude at sea level moves it
        // by far less than the 0.01° tolerance here.
        let date = Date::from_ordinal_date(2029, 15).unwrap();
        let deg = variation_rad(37.03, -7.91, date).unwrap().to_degrees();
        assert!((deg - (-0.17368)).abs() < 0.01, "got {deg:.5}°");
    }

    #[test]
    fn variation_rad_out_of_range_date_is_none() {
        // 2015 is before WMM2020/2025 — the crate refuses it.
        let old = Date::from_ordinal_date(2015, 1).unwrap();
        assert!(variation_rad(52.0, 5.0, old).is_none());
    }

    #[test]
    fn date_from_days_round_trips() {
        // 2026-01-01 is 20454 days after 1970-01-01.
        let d = date_from_days(20454).unwrap();
        assert_eq!(d.year(), 2026);
        assert_eq!(d.ordinal(), 1);
    }

    /// Decode a raw frame into the `DecodedPgn` the quirk now consumes.
    fn dec(frame: &RawFrame) -> DecodedPgn {
        PgnDatabase::embedded(Units::Si)
            .decode(frame)
            .expect("decodes")
    }

    fn position_129025(lat_deg: f64, lon_deg: f64) -> DecodedPgn {
        let lat = (lat_deg / 1e-7) as i32;
        let lon = (lon_deg / 1e-7) as i32;
        let mut data = Vec::with_capacity(8);
        data.extend_from_slice(&lat.to_le_bytes());
        data.extend_from_slice(&lon.to_le_bytes());
        dec(&RawFrame::new(None, 2, PGN_POSITION_RAPID, 3, 255, data))
    }

    /// A fast-packet PGN 129029 GNSS Position carrying position + date.
    /// Only the fields WMM reads (SID, date, lat, lon) need be plausible;
    /// the remainder are left at their not-available defaults.
    fn gnss_129029(lat_deg: f64, lon_deg: f64, days: u16) -> DecodedPgn {
        let db = PgnDatabase::embedded(Units::Si);
        let mut b = db.encode("gnssPositionData").unwrap().source(3);
        b.push(field::gnss_position_data::DATE, i64::from(days))
            .unwrap();
        b.push(field::gnss_position_data::LATITUDE, lat_deg)
            .unwrap();
        b.push(field::gnss_position_data::LONGITUDE, lon_deg)
            .unwrap();
        dec(&b.build().unwrap())
    }

    fn system_time_126992(days: u16) -> DecodedPgn {
        let mut data = vec![0xff, 0xf0]; // SID unknown, source 0, reserved
        data.extend_from_slice(&days.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes()); // time
        dec(&RawFrame::new(None, 3, PGN_SYSTEM_TIME, 5, 255, data))
    }

    /// A 127258 from `src` reporting variation `Source` = `source`.
    fn variation_127258(src: u8, source: u8) -> DecodedPgn {
        let db = PgnDatabase::embedded(Units::Si);
        let mut b = db.encode("magneticVariation").unwrap().source(src);
        b.push(field::magnetic_variation::SOURCE, i64::from(source))
            .unwrap();
        b.push(field::magnetic_variation::VARIATION, 0.05f64)
            .unwrap();
        dec(&b.build().unwrap())
    }

    #[test]
    fn no_emit_before_any_position() {
        let mut q = WmmQuirk::new();
        let out = q.process(&system_time_126992(20454), Instant::now());
        assert!(out.is_empty(), "no position yet → nothing to emit");
    }

    #[test]
    fn emits_after_position_then_rate_limited() {
        let mut q = WmmQuirk::new();
        let t0 = Instant::now();
        q.process(&system_time_126992(20454), t0);
        // First position frame → an emit.
        let out = q.process(&position_129025(52.0, 5.0), t0);
        let emitted: Vec<_> = out
            .iter()
            .filter(|f| f.pgn == PGN_MAGNETIC_VARIATION)
            .collect();
        assert_eq!(emitted.len(), 1, "one 127258 emitted");
        assert_eq!(
            emitted[0].src, ADDR_GLOBAL,
            "emitted as the own-node sentinel; the pipeline stamps the real address"
        );
        assert_eq!(emitted[0].data.len(), 8);
        // Read the Source field back: WMM 2025.
        let d = q.db.decode(emitted[0]).unwrap();
        assert_eq!(
            d.field(field::magnetic_variation::SOURCE)
                .and_then(|f| f.value.as_i64()),
            Some(SOURCE_WMM_2025)
        );

        // Another frame 500 ms later: within the transmit interval → no emit.
        let out = q.process(&position_129025(52.0, 5.0), t0 + Duration::from_millis(500));
        assert!(out.iter().all(|f| f.pgn != PGN_MAGNETIC_VARIATION));
        // Past the interval → emits again.
        let out = q.process(
            &position_129025(52.0, 5.0),
            t0 + EMIT_INTERVAL + Duration::from_millis(1),
        );
        assert_eq!(
            out.iter()
                .filter(|f| f.pgn == PGN_MAGNETIC_VARIATION)
                .count(),
            1
        );
    }

    #[test]
    fn gnss_129029_supplies_position_and_date() {
        // A single fast-packet GNSS Position frame carries both the
        // position and the date, so it alone is enough to emit — no
        // separate System Time needed.
        let mut q = WmmQuirk::new();
        let t0 = Instant::now();
        let out = q.process(&gnss_129029(52.0, 5.0, 20454), t0);
        assert_eq!(
            out.iter()
                .filter(|f| f.pgn == PGN_MAGNETIC_VARIATION)
                .count(),
            1,
            "129029 alone yields an emit"
        );
        let (lat, lon) = q.last_pos.expect("position learned");
        assert!((lat - 52.0).abs() < 1e-6 && (lon - 5.0).abs() < 1e-6);
        assert_eq!(q.last_days, Some(20454));
    }

    #[test]
    fn variation_round_trips_within_one_lsb() {
        let mut q = WmmQuirk::new();
        let t0 = Instant::now();
        q.process(&position_129025(52.0, 5.0), t0);
        let out = q.process(&position_129025(52.0, 5.0), t0 + EMIT_INTERVAL);
        // (first frame emitted at t0; this second one is the fresh emit)
        let frame = out
            .iter()
            .find(|f| f.pgn == PGN_MAGNETIC_VARIATION)
            .expect("emit");
        let want = q.cached_rad.unwrap();
        let got =
            q.db.decode(frame)
                .unwrap()
                .field(field::magnetic_variation::VARIATION)
                .and_then(|f| f.value.as_f64())
                .unwrap();
        assert!((got - want).abs() <= 1e-4, "want {want}, got {got}");
    }

    #[test]
    fn older_wmm_source_gets_one_stop_request() {
        let mut q = WmmQuirk::new();
        let t0 = Instant::now();
        // WMM 2020 (value 8) from src 30 → exactly one 126208 stop.
        let out = q.process(&variation_127258(30, 8), t0);
        let stops: Vec<_> = out.iter().filter(|f| f.pgn == 126208).collect();
        assert_eq!(stops.len(), 1);
        assert_eq!(stops[0].dst, 30);
        // A second older-WMM frame from the same src → no further request.
        let out = q.process(&variation_127258(30, 8), t0 + Duration::from_secs(5));
        assert!(out.iter().all(|f| f.pgn != 126208), "once per session");
    }

    #[test]
    fn wmm2025_and_manual_sources_are_left_alone() {
        let mut q = WmmQuirk::new();
        let t0 = Instant::now();
        // Already WMM 2025 (9): leave it.
        assert!(
            q.process(&variation_127258(30, 9), t0)
                .iter()
                .all(|f| f.pgn != 126208)
        );
        // Manual (0): not an older-WMM epoch, leave it.
        assert!(
            q.process(&variation_127258(31, 0), t0)
                .iter()
                .all(|f| f.pgn != 126208)
        );
    }

    #[test]
    fn own_emitted_variation_fed_back_is_a_noop() {
        // Our own emitted 127258 (Source = WMM 2025) is fed back through
        // the pipeline. It must not trigger a stop-request (Source 9 isn't
        // an older epoch) nor an immediate re-emit (transmit throttle).
        let mut q = WmmQuirk::new();
        let t0 = Instant::now();
        q.process(&position_129025(52.0, 5.0), t0); // emits at t0
        let own = variation_127258(21, SOURCE_WMM_2025 as u8);
        let out = q.process(&own, t0 + Duration::from_millis(10));
        assert!(
            out.is_empty(),
            "fed-back own frame: no stop-request, no re-emit within the interval"
        );
    }
}
