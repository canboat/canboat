// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `--quirk gps-relay`: put a corrected copy of a rolled-over device's
//! data back on the bus.
//!
//! `gps-rollover=<devices>` fixes the dates canboat *decodes*, which is
//! enough for everything downstream of canboat — n2kd, Signal K, a log.
//! It does nothing for the chartplotter across the cabin that reads the
//! same GPS directly and believes it is 2007. The only complete fix for
//! that is a new receiver. Short of that, canboat can offer itself as an
//! alternative: re-send everything the broken device broadcasts, from
//! canboat's own address, with the dates corrected, so the other
//! devices can select canboat as their position and time source.
//!
//! What is relayed: every broadcast PGN from a device listed in
//! `gps-rollover=`, not just the ones carrying a date. A display will
//! not accept a "GPS" that only sends 129029 once a second; it wants
//! the rapid position and COG/SOG too, so the whole set goes, 1:1 with
//! the original. Network management — address claims, product
//! information, requests, acknowledgements, PGN lists — is the device's
//! own business and is never relayed, and neither is anything addressed
//! to a specific destination.
//!
//! How: the date is patched into a copy of the raw payload at the
//! field's bit offset, rather than re-encoded, so a 129029 with its
//! reference-station set, or a proprietary PGN, goes out byte for byte
//! as it came in. The pipeline stamps canboat's live claimed address
//! on the copy and fragments it if it is a fast packet, as for the
//! other frame-emitting quirks.
//!
//! The copy comes back through the pipeline like any bus frame. Its
//! source is canboat's, which is not a listed device, so it is neither
//! corrected again nor relayed again — and it is skipped explicitly
//! here too, in case someone lists canboat's own address.
//!
//! The original device keeps transmitting; unlike `wmm`, we depend on
//! it and never ask it to stop. Other devices see two sources.

use canboat_core::quirk::gps_rollover_device_listed;
use canboat_core::{ADDR_GLOBAL, DecodedPgn, FieldValue, RawFrame};

/// PGNs a device sends about *itself* or in reply to a request; relaying
/// them would have canboat claim the device's identity.
const NETWORK_MANAGEMENT: &[u32] = &[
    59392,  // ISO Acknowledgement
    59904,  // ISO Request
    60160,  // ISO Transport Protocol, Data Transfer
    60416,  // ISO Transport Protocol, Connection Management
    60928,  // ISO Address Claim
    65240,  // ISO Commanded Address
    126208, // NMEA Request/Command/Acknowledge Group Function
    126464, // PGN List
    126996, // Product Information
    126998, // Configuration Information
];

/// Longest payload the fast-packet fragmenter accepts; anything bigger
/// arrived via ISO Transport Protocol and cannot go back out that way.
const MAX_RELAY_PAYLOAD: usize = 223;

/// Stateless apart from a per-source log guard.
pub struct GpsRelay {
    /// Sources we have announced relaying for, so the log says it once.
    announced: [bool; 256],
}

impl Default for GpsRelay {
    fn default() -> Self {
        Self::new()
    }
}

impl GpsRelay {
    pub fn new() -> Self {
        Self {
            announced: [false; 256],
        }
    }

    /// Relay `d` if it came from a listed device and is the kind of
    /// PGN we relay. `own_addr` is canboat's claimed address, when it
    /// holds one: our own relayed copies come back through here.
    pub fn process(&mut self, d: &DecodedPgn, own_addr: Option<u8>) -> Option<RawFrame> {
        if Some(d.src) == own_addr
            || d.src >= canboat_core::ADDR_NULL
            || d.dst != ADDR_GLOBAL
            || NETWORK_MANAGEMENT.contains(&d.pgn)
            || d.data.len() > MAX_RELAY_PAYLOAD
            || !gps_rollover_device_listed(d.src)
        {
            return None;
        }
        if !std::mem::replace(&mut self.announced[usize::from(d.src)], true) {
            log::info!(
                "quirk(gps-relay): relaying everything from src {} as canboat's own, dates corrected",
                d.src
            );
        }
        Some(RawFrame::new(
            d.timestamp.clone(),
            d.prio,
            d.pgn,
            // "Send as my own node": the pipeline rewrites ADDR_GLOBAL to
            // canboat's live claimed address.
            ADDR_GLOBAL,
            ADDR_GLOBAL,
            corrected_payload(d),
        ))
    }
}

/// The frame's payload with every corrected DATE field written back
/// over the rolled-over one. Fields the decoder left untouched are
/// already equal to the bytes, so this is simply "write what was
/// decoded"; a DATE that is not byte-aligned or not 16 bits is not a
/// DATE the database knows how to lay out, and is left as it came.
fn corrected_payload(d: &DecodedPgn) -> Vec<u8> {
    let mut data = d.data.clone();
    for f in &d.fields {
        let FieldValue::Date(days) = f.value else {
            continue;
        };
        let (Some(bit_offset), Some(16)) = (f.bit_offset, f.bit_length) else {
            continue;
        };
        if bit_offset % 8 != 0 {
            continue;
        }
        let at = bit_offset as usize / 8;
        if let Some(slot) = data.get_mut(at..at + 2) {
            slot.copy_from_slice(&days.to_le_bytes());
        }
    }
    data
}

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::quirk::{Device, Target, disable_gps_rollover, enable_gps_rollover_at};
    use canboat_core::{PgnDatabase, Units};

    /// 2026-08-27 as days since 1970-01-01, and the same day one GPS
    /// epoch earlier: 2007-01-11.
    const REFERENCE: u16 = 20692;
    const ROLLED: u16 = REFERENCE - 7168;

    fn dec(frame: &RawFrame) -> DecodedPgn {
        PgnDatabase::embedded(Units::Si)
            .decode(frame)
            .expect("decodes")
    }

    fn bytes(hex: &str) -> Vec<u8> {
        hex.split(',')
            .map(|h| u8::from_str_radix(h, 16).unwrap())
            .collect()
    }

    /// PGN 126992 System Time from `src`, Source = Local Crystal clock,
    /// date `ROLLED`.
    fn system_time(src: u8, dst: u8) -> RawFrame {
        let d = ROLLED.to_le_bytes();
        RawFrame::new(
            Some("2026-08-27T11:04:00.100Z".into()),
            3,
            126992,
            src,
            dst,
            [0x36, 0xf5, d[0], d[1], 0x10, 0x6d, 0xff, 0x19],
        )
    }

    /// A real 47-byte PGN 129029 GNSS Position Data with one reference
    /// station in its repeating set (analyzer/tests/pgn-test.in), its
    /// date at bytes 1..3 moved back one epoch.
    fn gnss_position(src: u8) -> RawFrame {
        let mut data = bytes(
            "88,a4,50,c0,99,13,10,40,52,84,77,54,17,74,04,00,3d,52,71,eb,12,23,13,c0,c6,2d,00,00,\
             00,00,00,13,fc,0e,40,00,7d,00,0a,0f,00,00,00,ff,ff,ff,ff",
        );
        data[1..3].copy_from_slice(&ROLLED.to_le_bytes());
        RawFrame::new(None, 3, 129029, src, 255, data)
    }

    /// Runs `body` with the quirk enabled for source address 4 and
    /// nothing else. The switch is process-wide, so this serialises
    /// with the other tests that touch it.
    fn with_device_4(body: impl FnOnce()) {
        let _guard = super::super::tests::SWITCH
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        enable_gps_rollover_at(Target::Devices(vec![Device::Address(4)]), REFERENCE);
        body();
        disable_gps_rollover();
    }

    #[test]
    fn relays_a_listed_devices_frame_with_the_date_patched() {
        with_device_4(|| {
            let frame = system_time(4, 255);
            let out = GpsRelay::new()
                .process(&dec(&frame), Some(30))
                .expect("relayed");
            assert_eq!(out.src, ADDR_GLOBAL);
            assert_eq!(out.dst, ADDR_GLOBAL);
            assert_eq!(out.pgn, 126992);
            assert_eq!(out.prio, 3);
            assert_eq!(out.timestamp, frame.timestamp);
            let mut expected = frame.data.to_vec();
            expected[2..4].copy_from_slice(&REFERENCE.to_le_bytes());
            assert_eq!(out.data.to_vec(), expected);
        });
    }

    #[test]
    fn patches_a_fast_packet_in_place() {
        with_device_4(|| {
            let frame = gnss_position(4);
            let out = GpsRelay::new()
                .process(&dec(&frame), None)
                .expect("relayed");
            let mut expected = frame.data.to_vec();
            expected[1..3].copy_from_slice(&REFERENCE.to_le_bytes());
            assert_eq!(out.data.to_vec(), expected, "only the date bytes change");
        });
    }

    #[test]
    fn relays_a_dateless_frame_unchanged() {
        with_device_4(|| {
            // PGN 129025 Position, Rapid Update.
            let frame = RawFrame::new(None, 2, 129025, 4, 255, bytes("40,52,84,77,54,17,74,04"));
            let out = GpsRelay::new()
                .process(&dec(&frame), None)
                .expect("relayed");
            assert_eq!(out.data, frame.data);
            assert_eq!(out.src, ADDR_GLOBAL);
        });
    }

    #[test]
    fn leaves_everything_else_alone() {
        with_device_4(|| {
            let mut relay = GpsRelay::new();
            // Not a listed device.
            assert!(relay.process(&dec(&system_time(5, 255)), None).is_none());
            // Addressed, not broadcast.
            assert!(relay.process(&dec(&system_time(4, 30)), None).is_none());
            // Our own relayed copy coming back.
            assert!(
                relay
                    .process(&dec(&system_time(30, 255)), Some(30))
                    .is_none()
            );
            // The device's address claim: its identity, not its data.
            let claim = RawFrame::new(None, 6, 60928, 4, 255, bytes("53,80,67,e7,00,be,8c,c0"));
            assert!(relay.process(&dec(&claim), None).is_none());
        });
    }

    #[test]
    fn nothing_is_relayed_when_the_quirk_is_off() {
        let _guard = super::super::tests::SWITCH
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        disable_gps_rollover();
        assert!(
            GpsRelay::new()
                .process(&dec(&system_time(4, 255)), None)
                .is_none()
        );
    }
}
