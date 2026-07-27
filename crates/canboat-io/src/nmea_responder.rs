// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Shared builders for the standard ISO / NMEA responses an addressable
//! NMEA 2000 node makes: PGN 126996 Product Information, PGN 126464 the
//! Transmit/Receive PGN lists, PGN 59392 ISO Acknowledgement, and PGN
//! 126993 Heartbeat.
//!
//! Both the Linux SocketCAN gateway ([`crate::device::socketcan`]) and the
//! server's synthetic devices (the H5000 Motion-Sensor impersonation quirk)
//! answer bus discovery with these exact PGNs — only the *content* differs
//! (each node's own product code, model, and PGN lists). Keeping the packing
//! here means one implementation, encoded through the schema-driven message
//! builder rather than hand-laid bytes. Pairs with
//! [`crate::address_claim`], which owns the claim state machine itself.

use canboat_core::{ADDR_GLOBAL, PgnDatabase, RawFrame, Units, field};

const PGN_ISO_ACK: u32 = 59392;
const PGN_PGN_LIST: u32 = 126464;
const PGN_HEARTBEAT: u32 = 126993;

/// One node's PGN 126996 Product Information content. Encode it for a given
/// source address with [`ProductInfo::frame`].
pub struct ProductInfo<'a> {
    /// NMEA 2000 database version, raw (res 0.001 → `2100` = "2.100").
    pub db_version: i64,
    /// Manufacturer product code. **The value a Hercules keys its Motion
    /// Source on (`0x53F0`); the SCX-20 discovery likewise keys on it.**
    pub product_code: i64,
    pub model_id: &'a str,
    pub software_version: &'a str,
    pub model_version: &'a str,
    pub model_serial: &'a str,
    pub certification_level: i64,
    pub load_equivalency: i64,
}

impl ProductInfo<'_> {
    /// Encode PGN 126996 from `src` via the schema message builder (fixed
    /// strings 0x00-padded to their field width). `None` only on an encoder
    /// error, which would be a schema/library bug.
    pub fn frame(&self, src: u8) -> Option<RawFrame> {
        use field::product_information as pi;
        let db = PgnDatabase::embedded(Units::Si);
        let build = || -> Result<RawFrame, canboat_core::encode::EncodeError> {
            let mut b = db.encode("productInformation")?.source(src);
            b.push(pi::NMEA2000_VERSION, self.db_version)?;
            b.push(pi::PRODUCT_CODE, self.product_code)?;
            b.push(pi::MODEL_ID, self.model_id)?;
            b.push(pi::SOFTWARE_VERSION_CODE, self.software_version)?;
            b.push(pi::MODEL_VERSION, self.model_version)?;
            b.push(pi::MODEL_SERIAL_CODE, self.model_serial)?;
            b.push(pi::CERTIFICATION_LEVEL, self.certification_level)?;
            b.push(pi::LOAD_EQUIVALENCY, self.load_equivalency)?;
            b.build()
        };
        match build() {
            Ok(f) => Some(f),
            Err(e) => {
                log::error!("PGN 126996 encode failed: {e}");
                None
            }
        }
    }
}

/// PGN 126464 Transmit and Receive PGN lists — one fast-packet frame each
/// (function 0 = transmit, 1 = receive), addressed to `dst`. Each PGN is
/// three little-endian bytes after the function byte.
pub fn pgn_list_frames(src: u8, dst: u8, tx: &[u32], rx: &[u32]) -> Vec<RawFrame> {
    [(0u8, tx), (1u8, rx)]
        .into_iter()
        .map(|(func, list)| {
            let mut data = Vec::with_capacity(1 + 3 * list.len());
            data.push(func);
            for pgn in list {
                data.extend_from_slice(&pgn.to_le_bytes()[..3]);
            }
            RawFrame::new(None, 6, PGN_PGN_LIST, src, dst, data)
        })
        .collect()
}

/// PGN 59392 ISO Acknowledgement. `control` is 0 = ACK, 1 = NAK,
/// 2 = Access Denied, 3 = Cannot Respond.
pub fn iso_ack_frame(src: u8, dst: u8, control: u8, pgn: u32) -> RawFrame {
    let p = pgn.to_le_bytes();
    let data = [control, 0xff, 0xff, 0xff, 0xff, p[0], p[1], p[2]];
    RawFrame::new(None, 6, PGN_ISO_ACK, src, dst, data)
}

/// PGN 126993 Heartbeat. `interval_ms` is the transmit interval this node
/// advertises (field resolution 0.01 s); `seq` is the wrapping sequence
/// counter (0..=252). Controller/equipment status bytes mirror a healthy
/// node (Error-Active / not-available / Operational).
pub fn heartbeat_frame(src: u8, seq: u8, interval_ms: u64) -> RawFrame {
    let offset = (interval_ms / 10) as u16;
    let data = [
        offset as u8,
        (offset >> 8) as u8,
        seq,
        0xCC,
        0xff,
        0xff,
        0xff,
        0xff,
    ];
    RawFrame::new(None, 7, PGN_HEARTBEAT, src, ADDR_GLOBAL, data)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn product_info_round_trips_and_carries_the_code() {
        let pi = ProductInfo {
            db_version: 2100,
            product_code: 21488,
            model_id: "H5000 Motion Sensor",
            software_version: "1.2.4",
            model_version: "Motion",
            model_serial: "123",
            certification_level: 2,
            load_equivalency: 1,
        };
        let f = pi.frame(24).expect("encodes");
        assert_eq!(f.pgn, 126996);
        assert_eq!(f.src, 24);
        assert_eq!(f.data.len(), 134);
        let d = PgnDatabase::embedded(Units::Si).decode(&f).unwrap();
        assert_eq!(
            d.field(field::product_information::PRODUCT_CODE)
                .and_then(|x| x.value.as_i64()),
            Some(21488)
        );
        assert_eq!(
            d.field(field::product_information::MODEL_ID)
                .and_then(|x| x.value.as_str()),
            Some("H5000 Motion Sensor")
        );
    }

    #[test]
    fn pgn_lists_are_two_framed_functions() {
        let out = pgn_list_frames(24, ADDR_GLOBAL, &[60928, 126996], &[59904]);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].data[0], 0); // transmit list
        assert_eq!(&out[0].data[1..4], &[0x00, 0xee, 0x00]); // 60928 LE
        assert_eq!(out[1].data[0], 1); // receive list
        assert!(out.iter().all(|f| f.pgn == PGN_PGN_LIST));
    }

    #[test]
    fn iso_ack_encodes_control_and_pgn() {
        let f = iso_ack_frame(24, 9, 1, 126996);
        assert_eq!(f.pgn, PGN_ISO_ACK);
        assert_eq!(f.data[0], 1); // NAK
        assert_eq!(&f.data[5..8], &[0x14, 0xf0, 0x01]); // 126996 LE
    }
}
