// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Keep or drop the messages of a bus's devices by their manufacturer
//! (canboat#1177): an allow list or a deny list of manufacturer codes.
//!
//! A message goes by the manufacturer of the device that sent it, so every
//! PGN from a source address goes the same way. The ISO Address Claim
//! (PGN 60928) says who that is, and overrides anything learned before. On
//! NMEA 2000 a proprietary PGN is a provisional claim until then: its first
//! 11 bits name whose message format it is, which is usually but not always
//! the sender's own. As soon as an address sends a second code it waits for
//! its Address Claim, because a Navico display sends other Navico brands'
//! codes, and a gateway or MFD sends other makers' to control their devices.
//!
//! Nothing from an address passes until its manufacturer is known, with
//! either list. Those messages are dropped, not held back.

use std::collections::{BTreeSet, HashMap};

use super::codec::SYNTHETIC_PGN_START;
use super::{BusProtocol, LookupTable, PacketType, PgnDatabase, RawFrame, Units};

/// PGN of the ISO Address Claim.
const ADDRESS_CLAIM: u32 = 60928;
/// The source address of a device that has no address yet: never known.
const NO_ADDRESS: u8 = 254;

/// Which manufacturers to keep.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ManufacturerList {
    /// Only keep devices of these manufacturers.
    Allow(BTreeSet<u16>),
    /// Drop devices of these manufacturers.
    Deny(BTreeSet<u16>),
}

impl ManufacturerList {
    /// Whether a device of manufacturer `code` is kept.
    pub fn keeps(&self, code: u16) -> bool {
        match self {
            Self::Allow(codes) => codes.contains(&code),
            Self::Deny(codes) => !codes.contains(&code),
        }
    }
}

/// What is known about the device at one source address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Known {
    /// Its first proprietary PGN's code, until an Address Claim comes.
    Provisional(u16),
    /// Its proprietary PGNs named two codes: only an Address Claim will do.
    AwaitingClaim,
    /// Its Address Claim's manufacturer.
    Claimed(u16),
}

/// What a message says about who sent it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Claim {
    /// An ISO Address Claim of this manufacturer.
    Address(u16),
    /// A proprietary PGN with this manufacturer code.
    Proprietary(u16),
    /// Nothing.
    None,
}

/// The manufacturers of a bus carrying `protocol`, if it has any: NMEA 2000
/// and J1939 share the 11 bits but not the allocation.
pub fn manufacturer_table(protocol: BusProtocol) -> Option<&'static LookupTable> {
    let name = match protocol {
        BusProtocol::Nmea2000 => "MANUFACTURER_CODE",
        BusProtocol::J1939 => "J1939_MANUFACTURER_CODE",
        BusProtocol::Quick => return None,
    };
    protocol.database(Units::Si).lookup(name)
}

/// Whether `pgn` starts with a manufacturer code on NMEA 2000.
pub fn is_proprietary(pgn: u32) -> bool {
    matches!(pgn, 61184 | 65280..=65535 | 126720 | 130816..=131071)
}

/// The per-address state of one [`ManufacturerList`], for a stream of whole
/// messages.
#[derive(Debug, Clone)]
pub struct ManufacturerFilter {
    list: ManufacturerList,
    /// Whether a proprietary PGN is a claim: only on NMEA 2000. J1939's
    /// proprietary PGNs don't start with a manufacturer code.
    proprietary_claims: bool,
    known: HashMap<u8, Known>,
}

impl ManufacturerFilter {
    /// A filter for `list` on a bus carrying `protocol`.
    pub fn new(list: ManufacturerList, protocol: BusProtocol) -> Self {
        Self {
            list,
            proprietary_claims: protocol == BusProtocol::Nmea2000,
            known: HashMap::new(),
        }
    }

    /// The list this filter keeps.
    pub fn list(&self) -> &ManufacturerList {
        &self.list
    }

    /// What the message `pgn` with payload `data` claims.
    pub fn claim_in(&self, pgn: u32, data: &[u8]) -> Claim {
        if pgn == ADDRESS_CLAIM {
            // The NAME, little-endian: the manufacturer is bits 21-31.
            return match data {
                [_, _, b2, b3, ..] => Claim::Address(u16::from(*b2) >> 5 | u16::from(*b3) << 3),
                _ => Claim::None,
            };
        }
        if self.proprietary_claims && is_proprietary(pgn) {
            return match data {
                [b0, b1, ..] => Claim::Proprietary(u16::from_le_bytes([*b0, *b1]) & 0x7ff),
                _ => Claim::None,
            };
        }
        Claim::None
    }

    /// Learn from a whole message, and say whether to keep it.
    pub fn keep_message(&mut self, pgn: u32, src: u8, data: &[u8]) -> bool {
        let claim = self.claim_in(pgn, data);
        self.keep(pgn, src, claim)
    }

    /// Learn `claim` from a message of `pgn` from `src`, and say whether to
    /// keep it. For a source that has already taken the message apart.
    pub fn keep(&mut self, pgn: u32, src: u8, claim: Claim) -> bool {
        // A pseudo PGN comes from the gateway or from canboat, not a device.
        if pgn >= SYNTHETIC_PGN_START {
            return true;
        }
        self.learn(src, claim);
        self.keeps(src)
    }

    /// Whether what is known about `src` keeps its messages.
    fn keeps(&self, src: u8) -> bool {
        match self.known.get(&src) {
            Some(Known::Provisional(code) | Known::Claimed(code)) => self.list.keeps(*code),
            Some(Known::AwaitingClaim) | None => false,
        }
    }

    /// Update what is known about `src` from `claim`, by the module's rules.
    fn learn(&mut self, src: u8, claim: Claim) {
        if src == NO_ADDRESS {
            return;
        }
        match claim {
            Claim::Address(code) => {
                self.known.insert(src, Known::Claimed(code));
            }
            Claim::Proprietary(code) => match self.known.get(&src) {
                None => {
                    self.known.insert(src, Known::Provisional(code));
                }
                Some(Known::Provisional(was)) if *was != code => {
                    self.known.insert(src, Known::AwaitingClaim);
                }
                Some(_) => {}
            },
            Claim::None => {}
        }
    }
}

/// A [`ManufacturerFilter`] for a stream of bus frames, passed on as frames:
/// `convert --to plain`, `interface` and the server.
///
/// A fast-packet's first frame holds its manufacturer code, so that frame
/// decides, and the rest of the message goes the same way: nothing needs
/// joining, and a message is not cut in two by what it teaches.
pub struct FrameManufacturerFilter {
    filter: ManufacturerFilter,
    db: &'static PgnDatabase,
    /// A frame of more than 8 bytes has shown that the source joins
    /// fast-packets itself.
    coalesced: bool,
}

impl FrameManufacturerFilter {
    /// A filter for `list`, on the bus that `db` decodes.
    pub fn new(list: ManufacturerList, protocol: BusProtocol, db: &'static PgnDatabase) -> Self {
        Self {
            filter: ManufacturerFilter::new(list, protocol),
            db,
            coalesced: false,
        }
    }

    /// Tell the filter that the source joins fast-packets itself, as it can
    /// know before any long message has shown it.
    pub fn set_coalesced(&mut self, coalesced: bool) {
        self.coalesced |= coalesced;
    }

    /// Learn from `frame`, and say whether to keep it.
    pub fn keep_frame(&mut self, frame: &RawFrame) -> bool {
        if frame.data.len() > 8 {
            self.coalesced = true;
        }
        let fast = !self.coalesced
            && self
                .db
                .first_pgn(frame.pgn)
                .or_else(|| self.db.fallback_pgn(frame.pgn))
                .is_some_and(|p| matches!(p.packet_type, PacketType::Fast));
        if !fast {
            return self.filter.keep_message(frame.pgn, frame.src, &frame.data);
        }
        // Only a whole first frame: its payload starts after the sequence
        // and length bytes. Bus frames of a fast-packet are always 8 bytes,
        // so a shorter line isn't one, and teaches nothing.
        let claim = match &frame.data[..] {
            [seq, _, payload @ ..] if frame.data.len() == 8 && seq & 0x1f == 0 => {
                self.filter.claim_in(frame.pgn, payload)
            }
            _ => Claim::None,
        };
        self.filter.keep(frame.pgn, frame.src, claim)
    }
}

/// What an analyzer JSON record says about its sender, for a source that
/// reads JSON instead of frames: `n2kd` and the TUI. `record` is the record
/// or its `{"<pgnId>":{...}}` wrapper, with the name, code or `-nv` form of
/// the manufacturer code.
#[cfg(any(feature = "bridge", feature = "cli"))]
pub fn claim_in_record(pgn: u32, record: &serde_json::Value, protocol: BusProtocol) -> Claim {
    use serde_json::Value;
    let proprietary = protocol == BusProtocol::Nmea2000 && is_proprietary(pgn);
    if pgn != ADDRESS_CLAIM && !proprietary {
        return Claim::None;
    }
    let inner = match record.get("fields") {
        Some(_) => record,
        None => match record.as_object().and_then(|o| o.values().next()) {
            Some(inner) => inner,
            None => return Claim::None,
        },
    };
    let Some(fields) = inner.get("fields") else {
        return Claim::None;
    };
    let value = ["manufacturerCode", "Manufacturer Code", "ManufacturerCode"]
        .iter()
        .find_map(|key| fields.get(key));
    let code = match value.map(|v| v.get("value").unwrap_or(v)) {
        Some(Value::Number(n)) => n.as_u64().and_then(|c| u16::try_from(c).ok()),
        Some(Value::String(name)) => manufacturer_table(protocol).and_then(|table| {
            let found = table.values.iter().find(|v| v.name == name)?;
            u16::try_from(found.value).ok()
        }),
        _ => None,
    };
    match code {
        Some(code) if pgn == ADDRESS_CLAIM => Claim::Address(code),
        Some(code) => Claim::Proprietary(code),
        None => Claim::None,
    }
}

/// [`claim_in_record`] of a JSON line, parsed only when its PGN can claim
/// anything.
#[cfg(any(feature = "bridge", feature = "cli"))]
pub fn claim_in_line(pgn: u32, line: &str, protocol: BusProtocol) -> Claim {
    let can_claim =
        pgn == ADDRESS_CLAIM || (protocol == BusProtocol::Nmea2000 && is_proprietary(pgn));
    if !can_claim {
        return Claim::None;
    }
    match serde_json::from_str(line) {
        Ok(record) => claim_in_record(pgn, &record, protocol),
        Err(_) => Claim::None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VICTRON: u16 = 358;
    const BG: u16 = 381;
    const SIMRAD: u16 = 1857;

    fn allow(codes: &[u16]) -> ManufacturerFilter {
        ManufacturerFilter::new(
            ManufacturerList::Allow(codes.iter().copied().collect()),
            BusProtocol::Nmea2000,
        )
    }

    /// A NAME's bytes with this manufacturer.
    fn claim(code: u16) -> Vec<u8> {
        let name = u64::from(code) << 21;
        name.to_le_bytes().to_vec()
    }

    fn proprietary(code: u16) -> Vec<u8> {
        let tag = code | 0x1800 | 4 << 13;
        let mut data = tag.to_le_bytes().to_vec();
        data.extend([0xff; 6]);
        data
    }

    #[test]
    fn the_codes_are_read_from_the_wire_layout() {
        let f = allow(&[]);
        assert_eq!(f.claim_in(60928, &claim(VICTRON)), Claim::Address(VICTRON));
        assert_eq!(f.claim_in(60928, &[1, 2, 3]), Claim::None);
        assert_eq!(f.claim_in(65280, &proprietary(BG)), Claim::Proprietary(BG));
        for pgn in [61184, 65535, 126720, 130816, 131071] {
            assert_eq!(f.claim_in(pgn, &proprietary(BG)), Claim::Proprietary(BG));
        }
        assert_eq!(f.claim_in(127250, &proprietary(BG)), Claim::None);
        assert_eq!(f.claim_in(65280, &[1]), Claim::None);
    }

    #[test]
    fn nothing_passes_until_the_manufacturer_is_known() {
        let mut allowing = allow(&[VICTRON]);
        assert!(!allowing.keep_message(127250, 7, &[0; 8]));
        let mut denying = ManufacturerFilter::new(
            ManufacturerList::Deny([BG].into_iter().collect()),
            BusProtocol::Nmea2000,
        );
        assert!(!denying.keep_message(127250, 7, &[0; 8]));
        // The claim itself goes by what it says.
        assert!(allowing.keep_message(60928, 7, &claim(VICTRON)));
        assert!(allowing.keep_message(127250, 7, &[0; 8]));
        assert!(denying.keep_message(60928, 7, &claim(VICTRON)));
        assert!(!denying.keep_message(60928, 8, &claim(BG)));
        assert!(!denying.keep_message(127250, 8, &[0; 8]));
    }

    #[test]
    fn a_proprietary_pgn_is_a_provisional_claim() {
        let mut f = allow(&[SIMRAD]);
        assert!(f.keep_message(65280, 7, &proprietary(SIMRAD)));
        assert!(f.keep_message(127250, 7, &[0; 8]));
        assert!(!f.keep_message(65280, 8, &proprietary(BG)));
        assert!(!f.keep_message(127250, 8, &[0; 8]));
    }

    #[test]
    fn a_second_code_waits_for_the_address_claim() {
        let mut f = allow(&[SIMRAD]);
        assert!(f.keep_message(65280, 7, &proprietary(SIMRAD)));
        assert!(f.keep_message(65280, 7, &proprietary(SIMRAD)));
        // A B&G display sends a Simrad-coded PGN and then its own.
        assert!(!f.keep_message(65280, 7, &proprietary(BG)));
        assert!(!f.keep_message(127250, 7, &[0; 8]));
        // From then on proprietary PGNs are not used as a claim.
        assert!(!f.keep_message(65280, 7, &proprietary(SIMRAD)));
        assert!(f.keep_message(60928, 7, &claim(SIMRAD)));
        assert!(f.keep_message(127250, 7, &[0; 8]));
    }

    #[test]
    fn an_address_claim_overrides_everything_and_addresses_move() {
        let mut f = allow(&[BG]);
        // Provisional Simrad, then a claim says B&G.
        assert!(!f.keep_message(65280, 7, &proprietary(SIMRAD)));
        assert!(f.keep_message(60928, 7, &claim(BG)));
        // A claimed device's other codes change nothing: an MFD controlling
        // another maker's device.
        assert!(f.keep_message(65280, 7, &proprietary(SIMRAD)));
        assert!(f.keep_message(127250, 7, &[0; 8]));
        // Another device takes the address over.
        assert!(!f.keep_message(60928, 7, &claim(VICTRON)));
        assert!(!f.keep_message(127250, 7, &[0; 8]));
    }

    #[test]
    fn address_254_is_never_known() {
        let mut f = allow(&[BG]);
        assert!(!f.keep_message(60928, 254, &claim(BG)));
        assert!(!f.keep_message(65280, 254, &proprietary(BG)));
        assert!(!f.keep_message(127250, 254, &[0; 8]));
    }

    #[test]
    fn a_pseudo_pgn_is_not_from_a_device() {
        let mut f = allow(&[]);
        assert!(f.keep_message(SYNTHETIC_PGN_START, 0, &[0; 8]));
    }

    #[test]
    fn j1939_proprietary_pgns_are_no_claim() {
        let mut f = ManufacturerFilter::new(
            ManufacturerList::Allow([BG].into_iter().collect()),
            BusProtocol::J1939,
        );
        assert!(!f.keep_message(65280, 7, &proprietary(BG)));
        assert!(f.keep_message(60928, 7, &claim(BG)));
    }

    #[test]
    fn frames_of_a_fast_packet_stay_together() {
        let db = BusProtocol::Nmea2000.database(crate::engine::Units::Si);
        let mut f = FrameManufacturerFilter::new(
            ManufacturerList::Allow([BG].into_iter().collect()),
            BusProtocol::Nmea2000,
            db,
        );
        let frame = |pgn: u32, data: &[u8]| RawFrame::new(None, 7, pgn, 7, 255, data.to_vec());
        let tag = (BG | 0x1800 | 4 << 13).to_le_bytes();
        let first = |seq: u8| frame(130820, &[seq << 5, 9, tag[0], tag[1], 1, 2, 3, 4]);
        let last = |seq: u8| frame(130820, &[seq << 5 | 1, 5, 6, 7, 0xff, 0xff, 0xff, 0xff]);
        // The first message teaches the code, and passes whole.
        assert!(f.keep_frame(&first(1)));
        assert!(f.keep_frame(&last(1)));
        assert!(f.keep_frame(&frame(127250, &[0; 8])));
        // A second code stops the address from its first frame on, and the
        // rest of that message goes the same way.
        let other = (SIMRAD | 0x1800 | 4 << 13).to_le_bytes();
        assert!(!f.keep_frame(&frame(130820, &[3 << 5, 9, other[0], other[1], 1, 2, 3, 4])));
        assert!(!f.keep_frame(&last(3)));
        assert!(!f.keep_frame(&frame(127250, &[0; 8])));
        // A short line on a fast-packet PGN teaches nothing.
        let mut unknown = FrameManufacturerFilter::new(
            ManufacturerList::Allow([BG].into_iter().collect()),
            BusProtocol::Nmea2000,
            db,
        );
        assert!(!unknown.keep_frame(&frame(130820, &[0, 9, tag[0], tag[1]])));
        assert!(!unknown.keep_frame(&frame(127250, &[0; 8])));
    }

    /// A first frame of a denied maker drops the message, whatever the list.
    #[test]
    fn a_first_frame_decides_for_a_denied_maker_too() {
        let db = BusProtocol::Nmea2000.database(crate::engine::Units::Si);
        let mut f = FrameManufacturerFilter::new(
            ManufacturerList::Deny([BG].into_iter().collect()),
            BusProtocol::Nmea2000,
            db,
        );
        let tag = (BG | 0x1800 | 4 << 13).to_le_bytes();
        let frame = |data: [u8; 8]| RawFrame::new(None, 7, 130820, 7, 255, data);
        assert!(!f.keep_frame(&frame([0, 9, tag[0], tag[1], 1, 2, 3, 4])));
        assert!(!f.keep_frame(&frame([1, 5, 6, 7, 0xff, 0xff, 0xff, 0xff])));
    }

    #[test]
    fn a_frame_stream_learns_from_a_claim_and_a_coalesced_message() {
        let db = BusProtocol::Nmea2000.database(crate::engine::Units::Si);
        let mut f = FrameManufacturerFilter::new(
            ManufacturerList::Allow([BG].into_iter().collect()),
            BusProtocol::Nmea2000,
            db,
        );
        let claimed = RawFrame::new(None, 6, 60928, 7, 255, claim(BG));
        assert!(f.keep_frame(&claimed));
        let mut long = proprietary(SIMRAD);
        long.extend([0; 4]);
        let whole = RawFrame::new(None, 7, 130820, 8, 255, long);
        assert!(!f.keep_frame(&whole));
    }

    #[cfg(any(feature = "bridge", feature = "cli"))]
    #[test]
    fn json_lines_give_their_claim_in_every_form() {
        let n2k = BusProtocol::Nmea2000;
        for (pgn, line, want) in [
            (
                60928,
                r#"{"pgn":60928,"fields":{"manufacturerCode":"Victron Energy"}}"#,
                Claim::Address(VICTRON),
            ),
            (
                60928,
                r#"{"pgn":60928,"fields":{"Manufacturer Code":{"value":358,"name":"Victron Energy"}}}"#,
                Claim::Address(VICTRON),
            ),
            (
                65280,
                r#"{"pgn":65280,"fields":{"manufacturerCode":381}}"#,
                Claim::Proprietary(BG),
            ),
            (
                65280,
                r#"{"someId":{"pgn":65280,"fields":{"ManufacturerCode":381}}}"#,
                Claim::Proprietary(BG),
            ),
            (65280, r#"{"pgn":65280,"fields":{"a":1}}"#, Claim::None),
            (
                127250,
                r#"{"fields":{"manufacturerCode":381}}"#,
                Claim::None,
            ),
            (65280, "not json", Claim::None),
        ] {
            assert_eq!(claim_in_line(pgn, line, n2k), want, "{line}");
        }
    }
}
