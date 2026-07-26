// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Struct-passing converters that take `&DecodedPgn` directly instead
//! of re-parsing a JSON string. Both the live `server` pipeline and
//! `n2kd` run these on an already-decoded record, so no analyzer JSON
//! is involved in producing NMEA 0183 — which is why the analyzer JSON
//! options (`-camel`, `-si`) can't affect the 0183 output.
//!
//! [`convert_nmea0183`] is the single source of truth for which PGNs
//! produce NMEA 0183: it dispatches on the decoded variant's canboat
//! **id** (e.g. `"windData"`), so adding a sentence means adding one
//! match arm and nothing else — there is no parallel PGN list to keep
//! in sync. A record whose id isn't listed simply isn't an NMEA 0183
//! PGN and yields no sentence.
//!
//! The handler bodies mirror `nmea0183.rs` line-for-line — same
//! sentences, same field formats — but pull values via
//! [`canboat_core::FieldRef`] in `O(1)` instead of via substring
//! scans through a JSON line.

use std::time::Instant;

use canboat_core::{DecodedPgn, FieldRef, FieldValue};

use crate::n2kd::nmea0183::RateLimiter;

/// m/s → knots.
const MS_TO_KNOTS: f64 = 1.943_84;
/// m/s → km/h.
const MS_TO_KMH: f64 = 3.6;

/// The [`FieldRef`]s the struct-path reads. Each is a generated
/// compile-time constant (`canboat_core::field::…`), so this is just a
/// named bundle of them — no runtime resolution, no per-record cost.
pub struct Handles {
    // PGN 127250 Vessel Heading.
    vh_heading: FieldRef,
    vh_deviation: FieldRef,
    vh_variation: FieldRef,
    vh_reference: FieldRef,
    // PGN 130306 Wind Data.
    wd_wind_speed: FieldRef,
    wd_wind_angle: FieldRef,
    wd_reference: FieldRef,
    // PGN 129026 SOG / COG.
    sc_sog: FieldRef,
    sc_cog: FieldRef,
    // PGN 127245 Rudder.
    ru_position: FieldRef,
    // PGN 128259 Speed.
    sp_speed_water_referenced: FieldRef,
    // PGN 128267 Water Depth.
    wd_depth: FieldRef,
    wd_offset: FieldRef,
    // PGN 128275 Distance Log.
    dl_log: FieldRef,
    dl_trip_log: FieldRef,
    // PGN 129029 GNSS Position Data.
    gp_lat: FieldRef,
    gp_lon: FieldRef,
    gp_date: FieldRef,
    gp_time: FieldRef,
    // PGN 129539 GNSS DOPs. canboat.json has HDOP/VDOP/TDOP — no
    // PDOP field is defined, so the JSON path's `json::value("PDOP")`
    // always returned None. We model the same here by simply
    // emitting an empty PDOP slot in the GSA sentence.
    dops_actual_mode: FieldRef,
    dops_hdop: FieldRef,
    dops_vdop: FieldRef,
    // PGN 130311 Environmental Parameters.
    env_temp_source: FieldRef,
    env_temp: FieldRef,
}

impl Handles {
    /// Bundle the field refs this module uses. Each [`canboat_core::FieldRef`]
    /// is a compile-checked (PGN, field) constant, so a `canboat.json` rename
    /// breaks the build here instead of returning a runtime `None`.
    pub fn new() -> Self {
        use canboat_core::field as f;
        Self {
            vh_heading: f::vessel_heading::HEADING,
            vh_deviation: f::vessel_heading::DEVIATION,
            vh_variation: f::vessel_heading::VARIATION,
            vh_reference: f::vessel_heading::REFERENCE,
            wd_wind_speed: f::wind_data::WIND_SPEED,
            wd_wind_angle: f::wind_data::WIND_ANGLE,
            wd_reference: f::wind_data::REFERENCE,
            sc_sog: f::cog_sog_rapid_update::SOG,
            sc_cog: f::cog_sog_rapid_update::COG,
            ru_position: f::rudder::POSITION,
            sp_speed_water_referenced: f::speed::SPEED_WATER_REFERENCED,
            wd_depth: f::water_depth::DEPTH,
            wd_offset: f::water_depth::OFFSET,
            dl_log: f::distance_log::LOG,
            dl_trip_log: f::distance_log::TRIP_LOG,
            gp_lat: f::gnss_position_data::LATITUDE,
            gp_lon: f::gnss_position_data::LONGITUDE,
            gp_date: f::gnss_position_data::DATE,
            gp_time: f::gnss_position_data::TIME,
            dops_actual_mode: f::gnss_dops::ACTUAL_MODE,
            dops_hdop: f::gnss_dops::HDOP,
            dops_vdop: f::gnss_dops::VDOP,
            env_temp_source: f::environmental_parameters::TEMPERATURE_SOURCE,
            env_temp: f::environmental_parameters::TEMPERATURE,
        }
    }
}

impl Default for Handles {
    fn default() -> Self {
        Self::new()
    }
}

/// Top-level NMEA 0183 dispatcher, and the single source of truth for
/// which PGNs produce a sentence. Returns the number of NMEA 0183
/// sentences appended to `out` (0 or 1).
///
/// Dispatch keys on the decoded variant's canboat `id` (from
/// canboat.json), not the numeric PGN: a PGN number can carry several
/// variants with different field layouts, and each `id` names exactly
/// one. So this is precise even when the analyzer identifies a specific
/// variant (e.g. from `-camel` output). The keys are the generated
/// [`canboat_core::pgn`] constants rather than string literals, so a
/// renamed id in canboat.json deletes the symbol and breaks this build
/// instead of silently never matching. To add a sentence, add one branch
/// here — its rate-limit class travels in the same branch, so there is
/// no separate PGN or rate table to drift out of sync. An unlisted id
/// isn't an NMEA 0183 PGN and yields nothing.
///
/// This is an `if`/`else if` chain, not a `match`, because a `pgn::X.id`
/// value is a `static`-backed expression, not a `const` pattern (a match
/// pattern would need a `&str` *literal*, defeating the compile-time
/// check). The comparison is exhaustive by the trailing `else`.
pub fn convert_nmea0183(
    out: &mut String,
    decoded: &DecodedPgn,
    rl: &mut RateLimiter,
    h: &Handles,
) -> usize {
    use canboat_core::pgn;
    // AIS PGNs render to `!AIVDM` via the AIS encoder — its own path,
    // deliberately not gated by the `$`-sentence rate limiter. Callers
    // that treat AIS specially (e.g. exempt `!AI…` from the per-device
    // 0183 filter) gate on `is_ais_pgn` themselves. Folding it in here
    // means every producer runs one dispatcher.
    if canboat_core::snapshot::is_ais_pgn(decoded.pgn) {
        return crate::n2kd::ais_decoded::convert(out, decoded, &mut rl.ais_seq);
    }
    let src = decoded.src;
    let id = decoded.id;
    let before = out.len();
    // Each branch: rate-limit gate for that sentence class, then the
    // converter. `rl`/`src` are passed in so the fragment stays hygienic.
    macro_rules! emit {
        ($rl:expr, $src:expr, $rate:expr, $call:expr) => {{
            if $rl.should_drop_fast($src, $rate) {
                return 0;
            }
            $call;
        }};
    }
    if id == pgn::RUDDER.id {
        emit!(rl, src, Rate::Rudder, rudder(out, src, decoded, h));
    } else if id == pgn::VESSEL_HEADING.id {
        emit!(
            rl,
            src,
            Rate::VesselHeading,
            vessel_heading(out, src, decoded, h)
        );
    } else if id == pgn::SPEED.id {
        emit!(rl, src, Rate::WaterSpeed, water_speed(out, src, decoded, h));
    } else if id == pgn::WATER_DEPTH.id {
        emit!(rl, src, Rate::WaterDepth, water_depth(out, src, decoded, h));
    } else if id == pgn::DISTANCE_LOG.id {
        emit!(
            rl,
            src,
            Rate::DistanceLog,
            distance_log(out, src, decoded, h)
        );
    } else if id == pgn::COG_SOG_RAPID_UPDATE.id {
        emit!(rl, src, Rate::GpsSpeed, sog_cog(out, src, decoded, h, rl));
    } else if id == pgn::GNSS_POSITION_DATA.id {
        emit!(
            rl,
            src,
            Rate::GpsPosition,
            position(out, src, decoded, h, rl)
        );
    } else if id == pgn::GNSS_DOPS.id {
        emit!(rl, src, Rate::GpsDop, gps_dop(out, src, decoded, h));
    } else if id == pgn::WIND_DATA.id {
        emit!(rl, src, Rate::WindData, wind_data(out, src, decoded, h));
    } else if id == pgn::ENVIRONMENTAL_PARAMETERS.id {
        emit!(
            rl,
            src,
            Rate::Environmental,
            environmental(out, src, decoded, h)
        );
    } else {
        return 0;
    }
    if out.len() > before { 1 } else { 0 }
}

#[derive(Debug, Clone, Copy)]
pub enum Rate {
    VesselHeading,
    WindData,
    WaterDepth,
    WaterSpeed,
    Rudder,
    GpsSpeed,
    GpsDop,
    GpsPosition,
    Environmental,
    DistanceLog,
}

/// `Rate` → index into `RateLimiter::last_passed_slot`. Must match
/// the discriminant order in `nmea0183.rs` since the two paths
/// share the same limiter state.
fn rate_index(r: Rate) -> usize {
    match r {
        Rate::VesselHeading => 0,
        Rate::WindData => 1,
        Rate::WaterDepth => 2,
        Rate::WaterSpeed => 3,
        Rate::Rudder => 4,
        Rate::GpsSpeed => 5,
        Rate::GpsDop => 6,
        Rate::GpsPosition => 7,
        Rate::Environmental => 8,
        Rate::DistanceLog => 9,
    }
}

/// Thin shim into the existing `RateLimiter` machinery — we
/// duplicate just enough to translate our `Rate` enum into the
/// position used by `nmea0183.rs`. Keeps the limiter's per-source
/// state shared between the two paths.
impl RateLimiter {
    pub(crate) fn should_drop_fast(&mut self, src: u8, rate: Rate) -> bool {
        if !self.enabled() {
            return false;
        }
        let slot = self.last_passed_slot(src as usize, rate_index(rate));
        let now = Instant::now();
        if let Some(prev) = *slot
            && now.duration_since(prev) < std::time::Duration::from_secs(1)
        {
            return true;
        }
        *slot = Some(now);
        false
    }
}

// =====================================================================
// Per-PGN handlers
// =====================================================================

fn vessel_heading(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    // NMEA 0183 is defined in degrees; ask for degrees and let the core
    // convert from the stream's schema unit (deg for Metric, rad for SI).
    let Some(heading) = decoded.field(h.vh_heading).and_then(|f| f.as_f64_in("deg")) else {
        return;
    };
    let reference = decoded
        .field(h.vh_reference)
        .and_then(|f| f.value.lookup_value())
        .map(|v| v as i64)
        .unwrap_or(-1);
    let dev = decoded
        .field(h.vh_deviation)
        .and_then(|f| f.as_f64_in("deg"));
    let var = decoded
        .field(h.vh_variation)
        .and_then(|f| f.as_f64_in("deg"));
    if let (Some(d), Some(v)) = (dev, var)
        && reference == 1
    {
        create(
            out,
            src,
            &format!(
                "HDG,{:.1},{:.1},{},{:.1},{}",
                heading,
                d.abs(),
                if d < 0.0 { 'W' } else { 'E' },
                v.abs(),
                if v < 0.0 { 'W' } else { 'E' },
            ),
        );
        return;
    }
    if reference == 0 {
        create(out, src, &format!("HDT,{:.1},T", heading));
    } else if reference == 1 {
        create(out, src, &format!("HDM,{:.1},M", heading));
    }
}

fn wind_data(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    let Some(speed) = decoded
        .field(h.wd_wind_speed)
        .and_then(|f| f.value.as_f64())
    else {
        return;
    };
    let Some(angle) = decoded
        .field(h.wd_wind_angle)
        .and_then(|f| f.as_f64_in("deg"))
    else {
        return;
    };
    let reference = decoded
        .field(h.wd_reference)
        .and_then(|f| f.value.lookup_value())
        .map(|v| v as i64)
        .unwrap_or(-1);
    let ref_char = match reference {
        2 => 'R', // Apparent
        3 => 'T', // True (boat referenced)
        _ => return,
    };
    create(
        out,
        src,
        &format!("MWV,{:.1},{ref_char},{:.1},K,A", angle, speed * MS_TO_KMH),
    );
}

fn sog_cog(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles, rl: &mut RateLimiter) {
    let Some(sog) = decoded.field(h.sc_sog).and_then(|f| f.value.as_f64()) else {
        return;
    };
    let Some(cog) = decoded.field(h.sc_cog).and_then(|f| f.as_f64_in("deg")) else {
        return;
    };
    // Cache the last (sog, cog) for the position handler's RMC
    // emission — same single-slot global behaviour as the JSON path.
    rl.record_sog_cog(sog, cog);
    create(
        out,
        src,
        &format!(
            "VTG,{:.1},T,,M,{:.2},N,{:.2},K",
            cog,
            sog * MS_TO_KNOTS,
            sog * MS_TO_KMH
        ),
    );
}

fn rudder(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    let Some(pos) = decoded
        .field(h.ru_position)
        .and_then(|f| f.as_f64_in("deg"))
    else {
        return;
    };
    create(out, src, &format!("RSA,{:.1},A,,F", -pos));
}

fn water_speed(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    let Some(s) = decoded
        .field(h.sp_speed_water_referenced)
        .and_then(|f| f.value.as_f64())
    else {
        return;
    };
    create(
        out,
        src,
        &format!("VHW,,T,,M,{:.1},N,{:.1},K", s * MS_TO_KNOTS, s * MS_TO_KMH),
    );
}

fn water_depth(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    let Some(depth) = decoded.field(h.wd_depth).and_then(|f| f.value.as_f64()) else {
        return;
    };
    let offset = decoded.field(h.wd_offset).and_then(|f| f.value.as_f64());
    let body = match offset {
        Some(o) => format!("DPT,{:.1},{:.1}", depth, o),
        None => format!("DPT,{:.1},", depth),
    };
    create(out, src, &body);
}

fn distance_log(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    const M_TO_NM: f64 = 1.0 / 1852.0;
    let Some(log) = decoded.field(h.dl_log).and_then(|f| f.value.as_f64()) else {
        return;
    };
    let Some(trip) = decoded.field(h.dl_trip_log).and_then(|f| f.value.as_f64()) else {
        return;
    };
    create(
        out,
        src,
        &format!("VLW,{:.1},N,{:.1},N", log * M_TO_NM, trip * M_TO_NM),
    );
}

fn environmental(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    // MTW only fires for the Water Temperature sub-type (source == 0).
    let Some(source) = decoded
        .field(h.env_temp_source)
        .and_then(|f| f.value.lookup_value())
    else {
        return;
    };
    if source != 0 {
        return;
    }
    // MTW is defined in Celsius; ask for it and let the core convert from
    // the stream's schema unit (C for Metric, K for SI). This replaces
    // canboat's `t < 173.15` magnitude heuristic, which only existed
    // because the JSON path couldn't tell which unit the number was in —
    // the declared stream units (via the analyzer banner) now settle it.
    let Some(celsius) = decoded.field(h.env_temp).and_then(|f| f.as_f64_in("C")) else {
        return;
    };
    create(out, src, &format!("MTW,{celsius:.1},C"));
}

fn gps_dop(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles) {
    let mode: String = decoded
        .field(h.dops_actual_mode)
        .and_then(|f| f.value.lookup_value())
        .and_then(|n| char::from_digit(n as u32, 10))
        .map(|c| c.to_string())
        .unwrap_or_default();
    // canboat.json's gnssDops doesn't carry a PDOP field — the JSON
    // path's `json::value(msg, "PDOP")` always returned None, so the
    // GSA slot is always empty. Match that.
    let pdop = String::new();
    let hdop = decoded
        .field(h.dops_hdop)
        .and_then(|f| f.value.as_f64())
        .map(|v| format!("{v:.2}"))
        .unwrap_or_default();
    let vdop = decoded
        .field(h.dops_vdop)
        .and_then(|f| f.value.as_f64())
        .map(|v| format!("{v:.2}"))
        .unwrap_or_default();
    create(
        out,
        src,
        &format!("GSA,M,{mode},,,,,,,,,,,,,{pdop},{hdop},{vdop}"),
    );
}

fn position(out: &mut String, src: u8, decoded: &DecodedPgn, h: &Handles, rl: &RateLimiter) {
    let Some(lat) = decoded.field(h.gp_lat).and_then(|f| f.value.as_f64()) else {
        return;
    };
    let Some(lon) = decoded.field(h.gp_lon).and_then(|f| f.value.as_f64()) else {
        return;
    };
    // Round to 7 decimal degrees BEFORE the degree-minute split so
    // we match the JSON path verbatim — that path goes through
    // analyzer's `%.7f` formatter and n2kd then parses the string
    // back, which is equivalent to a `round(v * 1e7) / 1e7` step.
    let lat = (lat * 1e7).round() / 1e7;
    let lon = (lon * 1e7).round() / 1e7;
    let (lat_str, lat_hem) = latlon_to_nmea(lat, true);
    let (lon_str, lon_hem) = latlon_to_nmea(lon, false);
    let time_str = decoded
        .field(h.gp_time)
        .and_then(|f| match &f.value {
            FieldValue::Time { seconds, .. } => Some(*seconds),
            _ => None,
        })
        .map(|s| {
            let mut buf = String::with_capacity(13);
            // 0.0001-s resolution on Time → 4 fractional digits, then
            // canboat strips trailing zeros + bare decimal point.
            canboat_core::output::format_time(s, 4, false, &mut buf).ok();
            cleanup_time(buf)
        })
        .unwrap_or_default();
    let date_str = decoded
        .field(h.gp_date)
        .and_then(|f| match &f.value {
            FieldValue::Date(d) => Some(*d),
            _ => None,
        })
        .map(|d| {
            let mut buf = String::with_capacity(10);
            canboat_core::output::format_date(d, &mut buf).ok();
            nmea_date_string(&buf)
        })
        .unwrap_or_default();

    create(
        out,
        src,
        &format!("GLL,{lat_str},{lat_hem},{lon_str},{lon_hem},{time_str},A"),
    );

    // RMC's sog/cog come from the cached PGN 129026 slot. If we
    // haven't seen one within the last second, leave the fields
    // blank — matches the JSON-path behaviour.
    let (sog_str, cog_str) = match rl.recent_sog_cog() {
        Some((sog, cog)) => (format!("{:.2}", sog * MS_TO_KNOTS), format!("{:.1}", cog)),
        None => (String::new(), String::new()),
    };
    create(
        out,
        src,
        &format!(
            "RMC,{time_str},A,{lat_str},{lat_hem},{lon_str},{lon_hem},{sog_str},{cog_str},{date_str},,,A"
        ),
    );
}

fn cleanup_time(mut s: String) -> String {
    // `12:06:58.0000` → `120658`. Mirror canboat's
    // `cleanupTimeString` in gps_ais.c.
    s.retain(|c| c != ':');
    while s.ends_with('0') {
        s.pop();
    }
    if s.ends_with('.') {
        s.pop();
    }
    s
}

fn nmea_date_string(date: &str) -> String {
    // `2022.09.10` → `100922`. Day-month-year, two digits each.
    let mut parts = date.split('.');
    let y = parts.next().unwrap_or("");
    let mo = parts.next().unwrap_or("");
    let d = parts.next().unwrap_or("");
    if y.is_empty() || mo.is_empty() || d.is_empty() {
        return String::new();
    }
    let y2 = &y[y.len().saturating_sub(2)..];
    format!("{d:0>2}{mo:0>2}{y2:0>2}")
}

fn latlon_to_nmea(decimal: f64, is_lat: bool) -> (String, char) {
    let abs = decimal.abs();
    let deg = abs.floor();
    let combined = deg * 100.0 + (abs - deg) * 60.0;
    let hem = if is_lat {
        if decimal < 0.0 { 'S' } else { 'N' }
    } else if decimal < 0.0 {
        'W'
    } else {
        'E'
    };
    (format!("{combined:.4}"), hem)
}

// =====================================================================
// Local copy of the NMEA wrapping helper. Kept here so we don't have
// to expose `create`/`append_aivdm` from the existing module.
// =====================================================================

fn create(out: &mut String, src: u8, body: &str) {
    let mut first = (b'A' + ((src >> 4) & 0x0f)) as char;
    let second = (b'A' + (src & 0x0f)) as char;
    if first >= 'P' {
        first = (first as u8 + 1) as char;
    }
    let start = out.len();
    out.push('$');
    out.push(first);
    out.push(second);
    out.push_str(body);
    let mut chk = 0u8;
    for b in &out.as_bytes()[start + 1..] {
        chk ^= *b;
    }
    use std::fmt::Write as _;
    let _ = write!(out, "*{chk:02X}\r\n");
}

// `FieldValue::is_not_available` is used implicitly via `as_f64()`
// returning None for sentinel/NotAvailable fields — keep the import
// alive in case downstream code wants to call it directly.
#[allow(dead_code)]
fn _keep_import_alive(v: &FieldValue) -> bool {
    v.is_not_available()
}
