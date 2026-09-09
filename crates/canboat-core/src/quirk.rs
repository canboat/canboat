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
//!
//! ## Which dates
//!
//! With no device list (`gps-rollover`), only dates that a GNSS
//! receiver produced are touched: PGN 129029, 129033, and 126992 when
//! its Source is GPS. Those are the only DATE fields whose origin the
//! PGN itself tells us.
//!
//! The rot does not stop at the receiver, though: any other device that
//! takes its clock from the bus — a DSC VHF stamping PGN 129808 Date of
//! Receipt, a converter relaying System Time — repeats the wrong date,
//! and nothing in *those* PGNs says where the clock came from. So the
//! quirk also takes a list of [`Device`]s (`gps-rollover=4,1851:491603`,
//! or `gps-rollover=all`): every DATE field in every PGN from a listed
//! device is corrected, on top of the GNSS rule, except the ones a
//! device does not stamp from its own clock — the AIS reports 129793 /
//! 129794, which relay another station's transmission, and 127258 Age
//! of Service, which is the date of the variation model.

use std::str::FromStr;
use std::sync::RwLock;
use std::sync::atomic::{AtomicBool, Ordering};

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

/// Highest source address a device can hold; 252–255 are the null and
/// global addresses, not devices.
const MAX_DEVICE_ADDRESS: u8 = 251;

/// A device whose every date is stamped from a rolled-over clock, as
/// named on the command line. See [`Device::from_str`] for the syntax.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Device {
    /// A source address (0–251). Simple, but a device may claim a
    /// different address after the next power cycle.
    Address(u8),
    /// The Manufacturer Code and Unique Number from the device's ISO
    /// Address Claim — what canboat's own JSON and the Signal K device
    /// list show, so the form people can find. Matched against the
    /// NAME learned from PGN 60928, so it follows the device across
    /// address changes.
    Product { manufacturer: u16, unique: u32 },
    /// The full 64-bit ISO NAME, for the rare case where two devices
    /// share manufacturer and unique number.
    Name(u64),
}

impl FromStr for Device {
    type Err = String;

    /// `4` — a source address; `1851:491603` — manufacturer code and
    /// unique number; `0x00d6c8531b0bc7a3` — a NAME in hex.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.trim();
        if s.is_empty() {
            return Err("empty device".to_string());
        }
        if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
            return u64::from_str_radix(hex, 16)
                .map(Device::Name)
                .map_err(|_| format!("'{s}' is not a hexadecimal ISO NAME"));
        }
        if let Some((m, u)) = s.split_once(':') {
            let manufacturer: u16 = m
                .trim()
                .parse()
                .ok()
                .filter(|&v| v < (1 << 11))
                .ok_or_else(|| format!("'{m}' is not a manufacturer code (0-2047)"))?;
            let unique: u32 = u
                .trim()
                .parse()
                .ok()
                .filter(|&v| v < (1 << 21))
                .ok_or_else(|| format!("'{u}' is not a unique number (0-2097151)"))?;
            return Ok(Device::Product {
                manufacturer,
                unique,
            });
        }
        match s.parse::<u64>() {
            Ok(a) if a <= u64::from(MAX_DEVICE_ADDRESS) => Ok(Device::Address(a as u8)),
            Ok(_) => Err(format!(
                "'{s}' is not a source address (0-{MAX_DEVICE_ADDRESS}); to name a device by \
                 its NAME use <manufacturer>:<unique number> or 0x<hex NAME>"
            )),
            Err(_) => Err(format!(
                "'{s}' is not a device: use a source address, <manufacturer>:<unique \
                 number>, 0x<hex NAME>, or all"
            )),
        }
    }
}

impl Device {
    /// Does this entry name the device that sent from `src`, whose
    /// ISO NAME (if a claim has been seen) is `name`?
    fn matches(self, src: u8, name: Option<u64>) -> bool {
        match self {
            Device::Address(a) => a == src,
            Device::Name(n) => name == Some(n),
            Device::Product {
                manufacturer,
                unique,
            } => name.is_some_and(|n| {
                (n & 0x1f_ffff) as u32 == unique && ((n >> 21) & 0x7ff) as u16 == manufacturer
            }),
        }
    }
}

/// Which dates the GPS rollover quirk corrects.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Target {
    /// Only dates a GNSS receiver produced: PGN 129029, 129033, and
    /// 126992 when its Source is GPS. `gps-rollover` with no argument.
    #[default]
    Gnss,
    /// The GNSS rule, plus every date from the listed devices.
    Devices(Vec<Device>),
    /// Every date on the bus. `gps-rollover=all`.
    All,
}

impl Target {
    /// Parse the part after `gps-rollover=`: `None` for the bare flag,
    /// otherwise `all` or a comma-separated list of [`Device`]s.
    pub fn parse(args: Option<&str>) -> Result<Target, String> {
        let Some(args) = args else {
            return Ok(Target::Gnss);
        };
        if args.trim().eq_ignore_ascii_case("all") {
            return Ok(Target::All);
        }
        let devices = args
            .split(',')
            .map(|d| {
                if d.trim().eq_ignore_ascii_case("all") {
                    Err("'all' cannot be combined with a device list".to_string())
                } else {
                    d.parse::<Device>()
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        if devices.is_empty() {
            return Err("gps-rollover= needs a device list, or 'all'".to_string());
        }
        Ok(Target::Devices(devices))
    }
}

/// The quirk's live configuration. `None` means off.
struct GpsRollover {
    /// Days since 1970-01-01 to snap towards.
    reference_day: u16,
    target: Target,
    /// ISO NAME per source address, learned from PGN 60928 as it goes
    /// by; 0 means no claim seen yet. The NAME is the whole 8-byte
    /// payload, so this needs no field decode.
    names: [u64; 256],
}

/// Set once by [`enable_gps_rollover`] / cleared by
/// [`disable_gps_rollover`]; the one thing the per-frame path reads
/// before deciding to take the lock.
static GPS_ROLLOVER_ON: AtomicBool = AtomicBool::new(false);
static GPS_ROLLOVER: RwLock<Option<GpsRollover>> = RwLock::new(None);

/// Turn the GPS week rollover quirk on for `target`, taking the
/// reference day from the system clock (floored at
/// [`MIN_REFERENCE_DAY`]).
pub fn enable_gps_rollover(target: Target) {
    enable_gps_rollover_at(target, today().max(MIN_REFERENCE_DAY));
}

/// Turn the quirk on with an explicit reference day (days since
/// 1970-01-01). Mainly for tests and for a host that knows better than
/// the system clock what day it is.
pub fn enable_gps_rollover_at(target: Target, reference_day: u16) {
    *GPS_ROLLOVER.write().unwrap_or_else(|e| e.into_inner()) = Some(GpsRollover {
        reference_day,
        target,
        names: [0; 256],
    });
    GPS_ROLLOVER_ON.store(true, Ordering::Release);
}

/// Turn the quirk off again.
pub fn disable_gps_rollover() {
    GPS_ROLLOVER_ON.store(false, Ordering::Release);
    *GPS_ROLLOVER.write().unwrap_or_else(|e| e.into_inner()) = None;
}

/// The reference day in force, or `None` when the quirk is off.
pub fn gps_rollover_reference_day() -> Option<u16> {
    GPS_ROLLOVER
        .read()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|q| q.reference_day)
}

/// The target in force, or `None` when the quirk is off.
pub fn gps_rollover_target() -> Option<Target> {
    GPS_ROLLOVER
        .read()
        .unwrap_or_else(|e| e.into_inner())
        .as_ref()
        .map(|q| q.target.clone())
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

/// Remember the ISO NAME a PGN 60928 frame from `src` just claimed.
/// Called by the decoder for every address claim; a no-op unless the
/// quirk is on.
pub(crate) fn note_address_claim(src: u8, payload: &[u8]) {
    if !GPS_ROLLOVER_ON.load(Ordering::Relaxed) {
        return;
    }
    let Some(bytes) = payload.get(..8) else {
        return;
    };
    let name = u64::from_le_bytes(bytes.try_into().expect("8 bytes"));
    if let Some(q) = GPS_ROLLOVER
        .write()
        .unwrap_or_else(|e| e.into_inner())
        .as_mut()
    {
        q.names[usize::from(src)] = name;
    }
}

/// Rewrite the date fields of a decoded PGN in place when the GPS
/// rollover quirk is on. See the module docs for which dates.
pub(crate) fn apply(pgn: u32, src: u8, fields: &mut [DecodedField]) {
    if !GPS_ROLLOVER_ON.load(Ordering::Relaxed) {
        return;
    }
    let guard = GPS_ROLLOVER.read().unwrap_or_else(|e| e.into_inner());
    if let Some(q) = guard.as_ref() {
        let name = match q.names[usize::from(src)] {
            0 => None,
            n => Some(n),
        };
        apply_at(pgn, src, name, fields, &q.target, q.reference_day);
    }
}

/// [`apply`] with everything explicit, independent of the global
/// switch — the testable half.
fn apply_at(
    pgn: u32,
    src: u8,
    name: Option<u64>,
    fields: &mut [DecodedField],
    target: &Target,
    reference_day: u16,
) {
    let device_is_listed = match target {
        Target::Gnss => false,
        Target::All => true,
        Target::Devices(devices) => devices.iter().any(|d| d.matches(src, name)),
    };
    let correct = if device_is_listed {
        // Everything this device stamps from its own clock — which is
        // every date except the ones it merely relays.
        !matches!(pgn, 129793 | 129794 | 127258)
    } else {
        is_gnss_receiver_date(pgn, fields)
    };
    if !correct {
        return;
    }
    for f in fields.iter_mut() {
        if let FieldValue::Date(days) = f.value {
            f.value = FieldValue::Date(corrected_gps_date(days, reference_day));
        }
    }
}

/// The dates the PGN itself tells us came from a GNSS receiver:
/// 129029 GNSS Position Data, 129033 Time & Date, and 126992 System
/// Time when its Source is GPS. The AIS reports carry another
/// station's clock, and the remaining DATE fields in the database
/// (Maretron counters, route database entries, tide/current station
/// data, DSC receipt stamps) say nothing about where their clock came
/// from — that is what the device list is for.
fn is_gnss_receiver_date(pgn: u32, fields: &[DecodedField]) -> bool {
    match pgn {
        129029 | 129033 => true,
        126992 => fields
            .iter()
            .find(|f| f.id() == "source")
            .and_then(|f| f.value.lookup_value())
            .is_some_and(|v| v == SYSTEM_TIME_SOURCE_GPS),
        _ => false,
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

    // --- device syntax ---------------------------------------------------

    /// The NAME of the DSC VHF from canboat/canboatjs#463: Raymarine
    /// (1851), unique number 491603, function 190 Radiotelephone, class
    /// 70 Communication, marine industry group, arbitrary-address capable.
    const VHF_NAME: u64 = 491603 | 1851 << 21 | 190 << 40 | 70 << 49 | 4 << 60 | 1 << 63;

    #[test]
    fn device_syntax() {
        assert_eq!("4".parse::<Device>(), Ok(Device::Address(4)));
        assert_eq!(" 251 ".parse::<Device>(), Ok(Device::Address(251)));
        assert_eq!(
            "1851:491603".parse::<Device>(),
            Ok(Device::Product {
                manufacturer: 1851,
                unique: 491603
            })
        );
        assert_eq!(
            format!("0x{VHF_NAME:016x}").parse::<Device>(),
            Ok(Device::Name(VHF_NAME))
        );
        assert_eq!("0XFF".parse::<Device>(), Ok(Device::Name(0xff)));
    }

    #[test]
    fn device_syntax_rejects_what_is_not_a_device() {
        for bad in [
            "",
            "252",
            "255",
            "300",
            "-1",
            "4.5",
            "vhf",
            "0x",
            "0xnope",
            "1851",
            "2048:1",
            "1851:2097152",
            "1851:",
            ":5",
        ] {
            assert!(bad.parse::<Device>().is_err(), "{bad:?} should be rejected");
        }
    }

    #[test]
    fn target_syntax() {
        assert_eq!(Target::parse(None), Ok(Target::Gnss));
        assert_eq!(Target::parse(Some("all")), Ok(Target::All));
        assert_eq!(Target::parse(Some("ALL")), Ok(Target::All));
        assert_eq!(
            Target::parse(Some("4, 1851:491603,0x10")),
            Ok(Target::Devices(vec![
                Device::Address(4),
                Device::Product {
                    manufacturer: 1851,
                    unique: 491603
                },
                Device::Name(0x10),
            ]))
        );
        assert!(Target::parse(Some("")).is_err());
        assert!(Target::parse(Some("4,")).is_err());
        assert!(Target::parse(Some("all,4")).is_err());
        assert!(Target::parse(Some("4,all")).is_err());
    }

    #[test]
    fn product_matches_only_its_own_name() {
        let d: Device = "1851:491603".parse().unwrap();
        assert!(d.matches(4, Some(VHF_NAME)));
        assert!(!d.matches(4, None));
        // Same unique number, different manufacturer.
        assert!(!d.matches(4, Some(VHF_NAME ^ (1 << 21))));
        // Same manufacturer, different unique number.
        assert!(!d.matches(4, Some(VHF_NAME ^ 1)));
        // The rest of the NAME does not matter.
        assert!(d.matches(9, Some(VHF_NAME ^ (1 << 40))));
        assert!(Device::Name(VHF_NAME).matches(9, Some(VHF_NAME)));
        assert!(!Device::Name(VHF_NAME).matches(9, Some(VHF_NAME ^ (1 << 40))));
    }

    // --- decoding --------------------------------------------------------

    /// Decode `frame`, apply the quirk with everything explicit, and
    /// hand back the first DATE field as a raw day count.
    fn date_after(
        frame: &crate::RawFrame,
        name: Option<u64>,
        target: &Target,
        reference_day: u16,
    ) -> Option<u16> {
        let mut decoded = crate::PgnDatabase::embedded(crate::Units::Si)
            .decode(frame)
            .expect("decodes");
        apply_at(
            frame.pgn,
            frame.src,
            name,
            &mut decoded.fields,
            target,
            reference_day,
        );
        decoded.fields.iter().find_map(|f| match f.value {
            FieldValue::Date(d) => Some(d),
            _ => None,
        })
    }

    /// A PGN 126992 System Time frame from `src` with `source` and
    /// `date`.
    fn system_time(src: u8, source: u8, date: u16) -> crate::RawFrame {
        let d = date.to_le_bytes();
        // SID, Source (low nibble) + Reserved, Date, Time.
        let data = [0x36, 0xf0 | source, d[0], d[1], 0x10, 0x6d, 0xff, 0x19];
        crate::RawFrame::new(None, 3, 126992, src, 255, data)
    }

    /// The real PGN 129808 DSC Call Information frame from
    /// `samples/pgn129808.raw`, sent by the VHF at source address 4
    /// with Date of Receipt 2007.01.11 — 1024 weeks before the day it
    /// was captured.
    fn dsc_call() -> crate::RawFrame {
        let data: Vec<u8> =
            "74,6c,00,16,29,02,28,64,7e,39,30,30,30,31,36,ff,ff,ff,ff,ff,ff,02,01,ff,ff,ff,\
             7f,ff,ff,ff,7f,ff,ff,ff,ff,ff,ff,ff,ff,ff,7f,fc,ff,ff,ff,ff,ff,ff,ff,ff,ff,ff,\
             ff,ff,40,f2,b5,17,d4,34,00,00"
                .split(',')
                .map(|h| u8::from_str_radix(h, 16).unwrap())
                .collect();
        crate::RawFrame::new(None, 4, 129808, 4, 255, data)
    }

    /// A PGN 129794 AIS Class A Static frame from `src` with an ETA
    /// date of `date`.
    fn ais_static(src: u8, date: u16) -> crate::RawFrame {
        let mut data = vec![0xffu8; 76];
        data[0] = 0x05; // Message ID 5, repeat indicator 0
        data[1..5].copy_from_slice(&244_660_000u32.to_le_bytes());
        data[45..47].copy_from_slice(&date.to_le_bytes());
        crate::RawFrame::new(None, 6, 129794, src, 255, data)
    }

    #[test]
    fn system_time_is_corrected_only_for_a_gps_source() {
        let reference = day(2022, 7, 4);
        let reported = day(2002, 11, 18);
        let gnss = Target::Gnss;
        assert_eq!(
            date_after(&system_time(12, 0, reported), None, &gnss, reference),
            Some(reference)
        );
        // Local Crystal clock (5): not a GNSS receiver, so the date
        // stands as sent.
        assert_eq!(
            date_after(&system_time(12, 5, reported), None, &gnss, reference),
            Some(reported)
        );
    }

    #[test]
    fn a_listed_device_has_every_date_corrected() {
        let reference = day(2026, 8, 27);
        let dsc = dsc_call();
        let rolled = day(2007, 1, 11);
        // Nothing in 129808 says where the clock came from, so the
        // GNSS rule leaves it alone...
        assert_eq!(
            date_after(&dsc, None, &Target::Gnss, reference),
            Some(rolled)
        );
        // ...but naming the VHF, by address or by NAME, corrects it.
        let by_address = Target::Devices(vec![Device::Address(4)]);
        assert_eq!(
            date_after(&dsc, None, &by_address, reference),
            Some(reference)
        );
        let by_product = Target::parse(Some("1851:491603")).unwrap();
        assert_eq!(
            date_after(&dsc, Some(VHF_NAME), &by_product, reference),
            Some(reference)
        );
        assert_eq!(
            date_after(&dsc, None, &Target::All, reference),
            Some(reference)
        );
        // A System Time from a listed device is corrected whatever its
        // Source says.
        assert_eq!(
            date_after(&system_time(4, 5, rolled), None, &by_address, reference),
            Some(reference)
        );
    }

    #[test]
    fn a_name_is_only_matched_once_the_claim_was_seen() {
        let reference = day(2026, 8, 27);
        let by_product = Target::parse(Some("1851:491603")).unwrap();
        // No claim yet: the frame is from an unknown device.
        assert_eq!(
            date_after(&dsc_call(), None, &by_product, reference),
            Some(day(2007, 1, 11))
        );
        // Some other device at that address.
        assert_eq!(
            date_after(&dsc_call(), Some(VHF_NAME ^ 1), &by_product, reference),
            Some(day(2007, 1, 11))
        );
    }

    #[test]
    fn an_unlisted_device_still_gets_the_gnss_rule() {
        let reference = day(2022, 7, 4);
        let reported = day(2002, 11, 18);
        let by_address = Target::Devices(vec![Device::Address(4)]);
        assert_eq!(
            date_after(&system_time(115, 0, reported), None, &by_address, reference),
            Some(reference)
        );
        assert_eq!(
            date_after(&system_time(115, 5, reported), None, &by_address, reference),
            Some(reported)
        );
    }

    #[test]
    fn relayed_ais_dates_are_never_corrected() {
        // A VHF with a built-in AIS receiver: its 129794 ETA is the
        // other vessel's, so it stays even though the VHF is listed.
        let reference = day(2026, 8, 27);
        let eta = day(2007, 1, 11);
        for target in [
            Target::Devices(vec![Device::Address(4)]),
            Target::All,
            Target::Gnss,
        ] {
            assert_eq!(
                date_after(&ais_static(4, eta), None, &target, reference),
                Some(eta),
                "{target:?}"
            );
        }
    }

    #[test]
    fn the_decoder_learns_names_from_address_claims() {
        // Serialised with the bridge's own quirk tests through the
        // process-wide switch; keep every step inside one test.
        let reference = day(2026, 8, 27);
        enable_gps_rollover_at(Target::parse(Some("1851:491603")).unwrap(), reference);
        let db = crate::PgnDatabase::embedded(crate::Units::Si);
        let date_of = |d: crate::DecodedPgn| {
            d.fields.iter().find_map(|f| match f.value {
                FieldValue::Date(d) => Some(d),
                _ => None,
            })
        };
        // Before the claim: not ours.
        assert_eq!(
            date_of(db.decode(&dsc_call()).unwrap()),
            Some(day(2007, 1, 11))
        );
        let claim = crate::RawFrame::new(None, 6, 60928, 4, 255, VHF_NAME.to_le_bytes());
        db.decode(&claim).unwrap();
        assert_eq!(date_of(db.decode(&dsc_call()).unwrap()), Some(reference));
        // The VHF moves to address 9: the old address stops matching,
        // the new one starts.
        let claim = crate::RawFrame::new(None, 6, 60928, 9, 255, VHF_NAME.to_le_bytes());
        db.decode(&claim).unwrap();
        let other = crate::RawFrame::new(None, 6, 60928, 4, 255, (VHF_NAME ^ 1).to_le_bytes());
        db.decode(&other).unwrap();
        assert_eq!(
            date_of(db.decode(&dsc_call()).unwrap()),
            Some(day(2007, 1, 11))
        );
        let mut moved = dsc_call();
        moved.src = 9;
        assert_eq!(date_of(db.decode(&moved).unwrap()), Some(reference));
        disable_gps_rollover();
        assert_eq!(gps_rollover_reference_day(), None);
    }
}
