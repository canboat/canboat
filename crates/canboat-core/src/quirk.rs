// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Opt-in corrections for known device misbehaviour, applied at decode
//! time.
//!
//! This is the sans-I/O half of canboat's quirk machinery. The quirks in
//! `canboat-bridge` *emit* synthetic frames onto the bus; the ones here
//! *rewrite* a decoded value in place, so they also apply to a plain
//! `canboat convert` run over a capture file — which is where a broken
//! date is usually first noticed.
//!
//! Every quirk is off by default and switched on explicitly, because
//! every one of them will happily "correct" data that was never wrong.
//!
//! ## GPS week rollover
//!
//! The GPS week number is 10 bits counted from the 1980-01-06 epoch, so
//! it wraps every 1024 weeks (7168 days): 1999-08-22, 2019-04-07, and
//! next on 2038-11-21. A receiver resolves the wrap by carrying a base
//! week from its firmware build date; one that was never updated keeps
//! using a base that is now one (or, for a receiver that also missed
//! 1999, two) epochs stale and reports dates that far in the past. Only
//! the week number wraps, so the time of day is unaffected and the
//! correction is a whole number of epochs.
//!
//! [`corrected_gps_date`] therefore does not just add 7168 days: it
//! snaps the reported date to the epoch nearest a reference day, which
//! is what a receiver's own base-week logic does. That covers a
//! doubly-stale receiver today, and the 2038 rollover without a code
//! change.

use std::sync::atomic::{AtomicU16, Ordering};

use crate::decode::{DecodedField, FieldValue};

/// One GPS rollover epoch: 1024 weeks, in days.
pub const GPS_ROLLOVER_DAYS: u16 = 7168;

/// Largest day count that is still a date rather than a sentinel
/// (0xfffd..=0xffff are Unknown / Out-of-range / Reserved) — 2149-06-03.
const MAX_DATE_DAY: u32 = 0xfffc;

/// Floor for the reference day used by [`enable_gps_rollover`]:
/// 2026-01-01. A boat computer without an RTC comes up believing it is
/// 1970 and gets its clock *from* the GPS we are correcting, so the
/// system clock alone is not a usable reference. Snapping to the nearest
/// epoch tolerates a reference that is off by up to ~9.8 years, so this
/// only ever needs revisiting long after the 2038 rollover.
const MIN_REFERENCE_DAY: u16 = 20454;

/// Reference day for the GPS rollover quirk, as days since 1970-01-01.
/// Zero means the quirk is off — no other value can mean that, since
/// day 0 is 1970-01-01 and every real reference is decades later.
static GPS_ROLLOVER_REFERENCE: AtomicU16 = AtomicU16::new(0);

/// Turn the GPS week rollover quirk on, taking the reference day from
/// the system clock (floored at [`MIN_REFERENCE_DAY`]).
pub fn enable_gps_rollover() {
    enable_gps_rollover_at(today().max(MIN_REFERENCE_DAY));
}

/// Turn the quirk on with an explicit reference day (days since
/// 1970-01-01). Mainly for tests and for a host that knows better than
/// the system clock what day it is.
pub fn enable_gps_rollover_at(reference_day: u16) {
    GPS_ROLLOVER_REFERENCE.store(reference_day, Ordering::Relaxed);
}

/// Turn the quirk off again.
pub fn disable_gps_rollover() {
    GPS_ROLLOVER_REFERENCE.store(0, Ordering::Relaxed);
}

/// The reference day in force, or `None` when the quirk is off.
pub fn gps_rollover_reference_day() -> Option<u16> {
    match GPS_ROLLOVER_REFERENCE.load(Ordering::Relaxed) {
        0 => None,
        d => Some(d),
    }
}

/// Today as days since 1970-01-01, or 0 if the clock predates the epoch.
fn today() -> u16 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| (d.as_secs() / 86400).min(u16::MAX as u64) as u16)
        .unwrap_or(0)
}

/// Snap `days` (days since 1970-01-01) to the GPS rollover epoch nearest
/// `reference_day`.
///
/// Returns `days` unchanged when it is already within half an epoch of
/// the reference, and when the correction would run into the sentinel
/// range.
pub fn corrected_gps_date(days: u16, reference_day: u16) -> u16 {
    let behind = u32::from(reference_day).saturating_sub(u32::from(days));
    let epoch = u32::from(GPS_ROLLOVER_DAYS);
    let epochs = (behind + epoch / 2) / epoch;
    if epochs == 0 {
        return days;
    }
    let corrected = u32::from(days) + epochs * epoch;
    if corrected > MAX_DATE_DAY {
        return days;
    }
    corrected as u16
}

/// PGN 126992 System Time `Source` value for GPS. GLONASS (1) counts
/// weeks from its own epoch and the remaining sources — radio station,
/// local cesium / rubidium / crystal — do not roll over at all.
const SYSTEM_TIME_SOURCE_GPS: u64 = 0;

/// Rewrite the date fields of a decoded PGN in place when the GPS
/// rollover quirk is on.
///
/// Only dates that came from a GNSS receiver on our own bus are
/// touched: 129029 GNSS Position Data, 129033 Time & Date, and 126992
/// System Time when its source is GPS. The AIS reports carry another
/// station's clock, and the remaining DATE fields in the database
/// (Maretron counters, route database entries, tide/current station
/// data) are not receiver clocks at all.
pub(crate) fn apply(pgn: u32, fields: &mut [DecodedField]) {
    if let Some(reference_day) = gps_rollover_reference_day() {
        apply_at(pgn, fields, reference_day);
    }
}

/// [`apply`] with an explicit reference day, independent of the global
/// switch — the testable half.
fn apply_at(pgn: u32, fields: &mut [DecodedField], reference_day: u16) {
    match pgn {
        129029 | 129033 => {}
        126992 => {
            let source_is_gps = fields
                .iter()
                .find(|f| f.id() == "source")
                .and_then(|f| f.value.lookup_value())
                .is_some_and(|v| v == SYSTEM_TIME_SOURCE_GPS);
            if !source_is_gps {
                return;
            }
        }
        _ => return,
    }
    for f in fields.iter_mut() {
        if let FieldValue::Date(days) = f.value {
            f.value = FieldValue::Date(corrected_gps_date(days, reference_day));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Days since 1970-01-01 for a date, the long way round, so the
    /// expectations below read as dates rather than as magic numbers.
    fn day(y: i32, m: u32, d: u32) -> u16 {
        // Days from civil date, Howard Hinnant's algorithm.
        let y = if m <= 2 { y - 1 } else { y };
        let era = y.div_euclid(400);
        let yoe = (y - era * 400) as u32;
        let mp = (m + 9) % 12;
        let doy = (153 * mp + 2) / 5 + d - 1;
        let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        (era as i64 * 146097 + doe as i64 - 719468) as u16
    }

    #[test]
    fn one_epoch_behind_is_corrected() {
        // The example from SignalK/n2k-signalk#271.
        let reported = day(2002, 11, 18);
        let reference = day(2022, 7, 4);
        assert_eq!(corrected_gps_date(reported, reference), day(2022, 7, 4));
    }

    #[test]
    fn two_epochs_behind_is_corrected() {
        // A receiver that missed the 1999 rollover as well as 2019.
        let reference = day(2026, 8, 29);
        let reported = reference - 2 * GPS_ROLLOVER_DAYS;
        assert_eq!(corrected_gps_date(reported, reference), reference);
    }

    #[test]
    fn a_current_date_is_left_alone() {
        let reference = day(2026, 8, 29);
        assert_eq!(corrected_gps_date(reference, reference), reference);
        assert_eq!(
            corrected_gps_date(reference - 30, reference),
            reference - 30
        );
    }

    #[test]
    fn a_merely_old_date_is_left_alone() {
        // Nine years back is not a rollover artefact — it is under half
        // an epoch, so it must not be snapped forward.
        let reference = day(2026, 8, 29);
        let reported = day(2017, 8, 29);
        assert_eq!(corrected_gps_date(reported, reference), reported);
    }

    #[test]
    fn a_stale_reference_still_corrects() {
        // Clockless host: reference stuck at the 2026 floor while the
        // real date is 2030 and the receiver is an epoch behind it.
        let real_today = day(2030, 6, 1);
        let reported = real_today - GPS_ROLLOVER_DAYS;
        assert_eq!(corrected_gps_date(reported, MIN_REFERENCE_DAY), real_today);
    }

    #[test]
    fn the_sentinel_range_is_never_entered() {
        // 2145 is inside the last epoch before the day count would
        // overflow into 0xfffd..0xffff.
        let late = 0xfffcu16 - 100;
        assert_eq!(corrected_gps_date(late, u16::MAX), late);
    }

    /// Decode a PGN 126992 System Time frame with `source` and
    /// `date`, apply the quirk at `reference_day`, and hand back the
    /// resulting raw day count.
    fn system_time_date(source: u8, date: u16, reference_day: u16) -> Option<u16> {
        let d = date.to_le_bytes();
        // SID, Source (low nibble) + Reserved, Date, Time.
        let data = [0x36, 0xf0 | source, d[0], d[1], 0x10, 0x6d, 0xff, 0x19];
        let frame = crate::RawFrame::new(None, 3, 126992, 12, 255, data);
        let mut decoded = crate::PgnDatabase::embedded(crate::Units::Si)
            .decode(&frame)
            .expect("decodes");
        apply_at(frame.pgn, &mut decoded.fields, reference_day);
        decoded.fields.iter().find_map(|f| match f.value {
            FieldValue::Date(d) => Some(d),
            _ => None,
        })
    }

    #[test]
    fn system_time_is_corrected_only_for_a_gps_source() {
        let reference = day(2022, 7, 4);
        let reported = day(2002, 11, 18);
        assert_eq!(system_time_date(0, reported, reference), Some(reference));
        // Local Crystal clock (5): not a GNSS receiver, so the date
        // stands as sent.
        assert_eq!(system_time_date(5, reported, reference), Some(reported));
    }
}
