// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Which application protocol a CAN bus carries.
//!
//! NMEA 2000 and SAE J1939 share the 29-bit ISO 11783 identifier, address
//! claim and ISO Transport Protocol, but not everything on top: they define
//! different PGNs, and only NMEA 2000 has fast-packet framing. A J1939 PGN
//! on data page 1 (0x10000-0x1FFFF) whose payload fits in 8 bytes is one
//! plain frame — reading it as the first frame of a fast-packet loses it.
//! So both the PGN table and the framing follow from the bus.

use std::fmt;
use std::str::FromStr;

use crate::engine::fastpacket;
use crate::engine::{FramePacketType, PgnDatabase, Units};

/// The application protocol on a CAN bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum BusProtocol {
    /// NMEA 2000: fast-packet framing for the PGNs that use it, ISO TP for
    /// the rest of the multi-frame traffic.
    #[default]
    Nmea2000,
    /// SAE J1939 (engines, gensets, transmissions): single frames, and ISO
    /// TP for anything longer than 8 bytes. No fast-packet framing.
    J1939,
}

impl BusProtocol {
    /// Every protocol, in the order `--bus` lists them.
    pub const ALL: [BusProtocol; 2] = [BusProtocol::Nmea2000, BusProtocol::J1939];

    /// The `--bus` spelling: `nmea2000` or `j1939`.
    pub fn as_str(self) -> &'static str {
        match self {
            BusProtocol::Nmea2000 => "nmea2000",
            BusProtocol::J1939 => "j1939",
        }
    }

    /// The embedded PGN table for this bus, in `units`.
    pub fn database(self, units: Units) -> &'static PgnDatabase {
        match self {
            BusProtocol::Nmea2000 => PgnDatabase::embedded(units),
            BusProtocol::J1939 => PgnDatabase::embedded_j1939(units),
        }
    }

    /// How a single CAN frame carrying `pgn` is framed on this bus, for
    /// a device that reassembles before it knows the PGN definition.
    /// ISO TP frames (PGN 60416 / 60160) are reassembled whatever this
    /// says.
    pub fn packet_type(self, pgn: u32) -> FramePacketType {
        match self {
            BusProtocol::Nmea2000 => fastpacket::packet_type(pgn),
            BusProtocol::J1939 => FramePacketType::Single,
        }
    }
}

impl fmt::Display for BusProtocol {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The error from parsing an unknown bus protocol name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownBusProtocol(pub String);

impl fmt::Display for UnknownBusProtocol {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "unknown bus protocol '{}' (expected nmea2000 or j1939)",
            self.0
        )
    }
}

impl std::error::Error for UnknownBusProtocol {}

impl FromStr for BusProtocol {
    type Err = UnknownBusProtocol;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        BusProtocol::ALL
            .into_iter()
            .find(|b| b.as_str().eq_ignore_ascii_case(s))
            .ok_or_else(|| UnknownBusProtocol(s.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_prints_the_cli_spelling() {
        for b in BusProtocol::ALL {
            assert_eq!(b.as_str().parse::<BusProtocol>(), Ok(b));
            assert_eq!(b.to_string(), b.as_str());
        }
        assert_eq!("J1939".parse::<BusProtocol>(), Ok(BusProtocol::J1939));
        assert!("quick".parse::<BusProtocol>().is_err());
    }

    #[test]
    fn j1939_never_uses_fast_packet_framing() {
        // 130816 is a fast-packet PGN on NMEA 2000; on J1939 the same
        // number is an ordinary data page 1 PGN.
        assert_eq!(
            BusProtocol::Nmea2000.packet_type(130816),
            FramePacketType::Fast
        );
        assert_eq!(
            BusProtocol::J1939.packet_type(130816),
            FramePacketType::Single
        );
    }

    #[test]
    fn each_bus_has_its_own_table() {
        let n2k = BusProtocol::Nmea2000.database(Units::Si);
        let j1939 = BusProtocol::J1939.database(Units::Si);
        assert!(!std::ptr::eq(n2k, j1939));
    }
}
