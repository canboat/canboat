// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! canboat-core: sans-I/O NMEA 2000 (canboat) PGN database, parsers,
//! decoder, and output formatters.
//!
//! This crate does no I/O. All functions are sync. Bytes go in, events
//! and bytes come out. The caller drives the I/O — see `canboat-io`
//! (sync) and `canboat-tokio` (async) for adapters. The sole `std::io`
//! touchpoint is the [`FrameSource`] trait (the bring-your-own-transport
//! seam): it names an `io::Result` but performs no I/O itself.

pub mod analyzer_json;
pub mod bits;
pub mod db;
pub mod decode;
pub mod encode;
pub mod format;
pub mod frame;
pub mod from_json;
pub mod os;
pub mod output;
pub mod quirk;
pub mod reassembly;
mod schema_data;
mod schema_data_j1939;
pub mod snapshot;
pub mod source;
pub mod startup;
pub mod types;
pub mod units;

pub use db::{PgnDatabase, Units};
pub use decode::{DecodeError, DecodedField, DecodedPgn, FieldValue};
pub use encode::{EncodeError, EncodeValue, PgnBuilder};
pub use frame::{ADDR_GLOBAL, ADDR_NULL, FASTPACKET_MAX_SIZE, RAWFRAME_MAX_SIZE, RawFrame};
pub use from_json::json_to_decoded;
pub use reassembly::{FramePacketType, Reassembled, Reassembler, ReassemblyError};
pub use schema_data::{COPYRIGHT_ID, SCHEMA_HASH, VERSION as CANBOAT_JSON_VERSION};
pub use schema_data::{field, pgn};
pub use source::{Decoder, FrameSource};
pub use startup::{CANBOAT_BEM, format_iso_ms, parse_iso_ms, startup_record};
pub use types::{
    BitLookupTable, BitLookupValue, FieldInfo, FieldRef, FieldType, IndirectLookupTable,
    IndirectLookupValue, LookupTable, LookupValue, PacketType, PgnInfo,
};

#[cfg(test)]
mod smoke {
    use super::*;

    #[test]
    fn loads_full_database() {
        let db = PgnDatabase::embedded(crate::Units::Metric);
        assert!(!db.version.is_empty());
        assert!(!db.schema_version.is_empty());
        assert!(
            db.pgn_count() >= 500,
            "expected >=500 PGNs, got {}",
            db.pgn_count()
        );
    }

    #[test]
    fn finds_iso_address_claim() {
        let db = PgnDatabase::embedded(crate::Units::Metric);
        let pgn = db.first_pgn(60928).expect("PGN 60928 must exist");
        assert_eq!(pgn.id, "isoAddressClaim");
        assert_eq!(pgn.packet_type, PacketType::Single);
        assert_eq!(pgn.fields.len(), 10);
        let mfr = &pgn.fields[1];
        assert_eq!(mfr.field_type, Some(FieldType::Lookup));
        assert_eq!(mfr.lookup_enumeration, Some("MANUFACTURER_CODE"));
    }

    #[test]
    fn resolves_manufacturer_lookup() {
        let db = PgnDatabase::embedded(crate::Units::Metric);
        let table = db.lookup("MANUFACTURER_CODE").expect("table present");
        assert!(table.values.iter().any(|v| v.name == "Navico"));
    }

    #[test]
    fn field_handle_resolves_and_finds() {
        let db = PgnDatabase::embedded(crate::Units::Metric);
        let h = db
            .field("isoAddressClaim", "uniqueNumber")
            .expect("unique number handle");
        assert_eq!(h.field.order, 1);
        assert!(db.field("isoAddressClaim", "noSuchField").is_none());
        assert!(db.field("noSuchPgn", "uniqueNumber").is_none());
    }

    #[test]
    fn field_handle_indexes_decoded_record() {
        let db = PgnDatabase::embedded(crate::Units::Metric);
        // From canboat/analyzer/tests/pgn-test.in — Unique Number =
        // 1088507, Manufacturer Code = 275 / Navico.
        let mut data: smallvec::SmallVec<[u8; 8]> = smallvec::SmallVec::new();
        for b in [0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0] {
            data.push(b);
        }
        let frame = RawFrame {
            timestamp: None,
            prio: 6,
            pgn: 60928,
            src: 5,
            dst: 255,
            data,
        };
        let dec = db.decode(&frame).expect("decode");
        let h = db.field("isoAddressClaim", "uniqueNumber").expect("handle");
        let f = dec.field(h).expect("field present");
        assert_eq!(f.value.as_i64(), Some(1_088_507));
    }

    #[test]
    fn si_and_metric_schemas_decode_same_frame_in_their_units() {
        // Wind Data (PGN 130306): Wind Angle raw 0x4000 = 16384.
        // SI:     16384 * 0.0001 rad             = 1.6384 rad
        // Metric: 16384 * 0.0001 * 180/π deg     ≈ 93.8734 deg (= 1.6384 rad)
        let mut data: smallvec::SmallVec<[u8; 8]> = smallvec::SmallVec::new();
        // seq=0, wind speed=0x0000, wind angle=0x4000, reference=Apparent(2)
        for b in [0x00, 0x00, 0x00, 0x00, 0x40, 0x02, 0xff, 0xff] {
            data.push(b);
        }
        let frame = RawFrame {
            timestamp: None,
            prio: 2,
            pgn: 130306,
            src: 23,
            dst: 255,
            data,
        };

        let angle = |units| {
            let db = PgnDatabase::embedded(units);
            let dec = db.decode(&frame).expect("decode");
            let h = db.field("windData", "windAngle").expect("handle");
            dec.field(h).expect("field").value.as_f64().expect("f64")
        };

        let si = angle(crate::Units::Si);
        let metric = angle(crate::Units::Metric);
        assert!((si - 1.6384).abs() < 1e-6, "SI radians: {si}");
        assert!((metric - 93.8734).abs() < 1e-3, "Metric degrees: {metric}");
        // Same physical quantity, two presentations.
        assert!((si.to_degrees() - metric).abs() < 1e-3);
    }
}
