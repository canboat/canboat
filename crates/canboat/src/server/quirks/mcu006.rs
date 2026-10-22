// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Furuno MCU-006 RotoKey remote impersonation — the identity half.
//!
//! The MCU-006 is Furuno's NMEA 2000 remote for TimeZero MFDs: a HID
//! keyboard and pointer tunnelled over proprietary PGN 61184 (`0xEF00`)
//! to one paired MFD. With `--quirk mcu006` canboat claims a distinct
//! virtual node that looks like one on the bus and pairs with the MFD, so
//! a program can drive the MFD through the control port as if it held the
//! remote.
//!
//! The byte layouts come from the MCU-006 firmware 01.03 (the `FUN_…`
//! names are its functions). A TZMAP9 pairs with this remote and acts on
//! every key, the pointer and the knob: `samples/furuno-tzmap9-mcu006.raw`.
//! Nothing has been checked against a real MCU-006.
//!
//! What this half does:
//! - claims an address with the remote's NAME (Furuno 1855, class 110
//!   Human Interface, function 130 Button Interface), preferring 110;
//! - answers ISO requests for 60928, 126996 (model `MCU-006`, product code
//!   21839), 126464 (the firmware's own lists), 65287, 65288 and 61184;
//! - broadcasts the `0xFF07` announce every 5 s, the `0xFF08` backlight
//!   status once, and a 126993 heartbeat every 60 s;
//! - follows the pairing: an `0xFF07` from another node with bit 0 of
//!   byte 2 set makes it the host, and its next address claim gives its
//!   NAME. Then announces it in 130849, as the remote does;
//! - answers the host's `0xEF00` commands the way the firmware does;
//! - logs pairing and the host's settings at `info`; every
//!   Furuno-proprietary frame another node sends, and every frame
//!   addressed to us, at `debug`.
//!
//! The reports themselves are sent by the control port in
//! [`crate::server::mcu006_control`], which reads the remote's address
//! and its host from the shared [`Link`]. Not yet: persisting the pairing
//! across restarts.
//!
//! Timing piggybacks on bus traffic, like the other quirks: the periodic
//! frames go out on the first decoded frame after they fall due. An MFD
//! keeps the bus busy enough for that.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::engine::{ADDR_GLOBAL, DecodedPgn, RawFrame};
use crate::io::address_claim::AddressClaim;
use crate::io::nmea_responder::{self, ProductInfo};

const PGN_ISO_REQUEST: u32 = 59904;
const PGN_ISO_ADDRESS_CLAIM: u32 = 60928;
const PGN_PGN_LIST: u32 = 126464;
const PGN_PRODUCT_INFO: u32 = 126996;
/// Furuno proprietary PGNs the remote uses (hex as in the firmware).
const PGN_HID: u32 = 61184; // 0xEF00, addressed: reports and commands
const PGN_ANNOUNCE: u32 = 65287; // 0xFF07
const PGN_STATUS: u32 = 65288; // 0xFF08
const PGN_HID_INFO: u32 = 130816; // 0x1FF00
const PGN_HOST: u32 = 130849; // 0x1FF21
const PGN_GROUP_FUNCTION: u32 = 126208;
const PGN_MAINTENANCE: u32 = 126720; // 0x1EF00

const MFG_FURUNO: u16 = 1855;
/// Furuno's proprietary header word `0x9F3F`: manufacturer 1855, reserved
/// bits `11`, industry 4. On the wire `3F 9F`.
const FURUNO_HEADER: [u8; 2] = [0x3f, 0x9f];

const DEVICE_FUNCTION: u8 = 130; // Button Interface
const DEVICE_CLASS: u8 = 110; // Human Interface
/// The firmware's reset template leaves both instance bytes at `0xFF`.
const DEVICE_INSTANCE: u8 = 0xff;
const SYSTEM_INSTANCE: u8 = 0x0f;
/// The firmware's default source address.
const PREFERRED_ADDRESS: u8 = 110;

/// PGN 126996 as the firmware builds it (`FUN_08015324`). The software
/// version is `<application>:<bootloader>`; `01.01` is the bootloader
/// version the firmware assumes before reading the unit's own record.
const PRODUCT_CODE: i64 = 21839; // 0x554F; MCU-006H is 5351
const DB_VERSION: i64 = 2100;
const MODEL_ID: &str = "MCU-006";
const SOFTWARE_VERSION: &str = "01.03:01.01";
const MODEL_VERSION: &str = "00.30";
const MODEL_SERIAL: &str = "0000";
const CERTIFICATION_LEVEL: i64 = 2;
const LOAD_EQUIVALENCY: i64 = 2;

/// The firmware's PGN 126464 lists, verbatim.
const TX_PGN_LIST: [u32; 20] = [
    59392, 59904, 60160, 60416, 60928, 61184, 65285, 65287, 65288, 126208, 126464, 126720, 126993,
    126996, 126998, 127502, 130816, 130822, 130823, 130849,
];
const RX_PGN_LIST: [u32; 10] = [
    59392, 59904, 60160, 60416, 60928, 61184, 65240, 65287, 126208, 126720,
];

const PRIO: u8 = 7;
const ANNOUNCE_INTERVAL: Duration = Duration::from_secs(5);
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(60);
/// Backlight preset 0 and 50 %, the firmware's defaults.
const BACKLIGHT_PRESET: u8 = 0;
const BACKLIGHT_PERCENT: u8 = 50;
/// 126208 parameter error code for a value the remote refuses.
const PARAM_ERROR: u8 = 4;

/// Where the remote's reports go, kept up to date by the quirk for the
/// control port. Both are `None` until the claim and the pairing are done.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LinkState {
    /// Our claimed source address.
    pub addr: Option<u8>,
    /// The paired MFD's address, once its NAME is known.
    pub host: Option<u8>,
}

/// The quirk's [`LinkState`], shared with the control port.
pub type Link = Arc<Mutex<LinkState>>;

/// The MFD this remote reports to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Host {
    addr: u8,
    /// Learned from the host's address claim after the pairing trigger.
    name: Option<u64>,
}

/// Stateful MCU-006 impersonation.
pub struct Mcu006 {
    claim: AddressClaim,
    base: Option<Instant>,
    started: bool,
    last_announce: Option<Instant>,
    last_heartbeat: Option<Instant>,
    heartbeat_seq: u8,
    /// Set by the MFD through 126208 on `0xFF08`.
    backlight_preset: u8,
    backlight_percent: u8,
    host: Option<Host>,
    link: Option<Link>,
}

impl Mcu006 {
    pub fn new(state_dir: Option<&std::path::Path>) -> Self {
        Self {
            claim: AddressClaim::new(build_name(state_dir), PREFERRED_ADDRESS, true),
            base: None,
            started: false,
            last_announce: None,
            last_heartbeat: None,
            heartbeat_seq: 0,
            backlight_preset: BACKLIGHT_PRESET,
            backlight_percent: BACKLIGHT_PERCENT,
            host: None,
            link: None,
        }
    }

    /// Share where reports go with the control port.
    pub fn set_link(&mut self, link: Link) {
        self.link = Some(link);
    }

    pub fn process(&mut self, d: &DecodedPgn, now: Instant) -> Vec<RawFrame> {
        let base = *self.base.get_or_insert(now);
        let now_ms = now.saturating_duration_since(base).as_millis() as u64;

        let mut out = Vec::new();
        if !self.started {
            self.started = true;
            log::info!(
                "quirk(mcu006): starting Furuno MCU-006 impersonation \
                 (claiming address, preferred {PREFERRED_ADDRESS}, NAME {:#018x})",
                self.claim.name()
            );
            out.extend(self.claim.start(now_ms));
        }

        let was_claimed = self.claim.is_claimed();
        let own = self.claim.address();
        // The pipeline feeds our own emissions back through; skip them.
        if own.is_some_and(|a| a == d.src) {
            out.extend(self.claim.tick(now_ms));
            return out;
        }

        self.log_interesting(d, own);

        match d.pgn {
            PGN_ISO_ADDRESS_CLAIM => {
                if let Some(their_name) = d.iso_name() {
                    out.extend(self.claim.on_address_claim(now_ms, d.src, their_name));
                    out.extend(self.on_host_claim(d.src, their_name));
                }
            }
            PGN_ISO_REQUEST => out.extend(self.handle_request(d)),
            PGN_ANNOUNCE => out.extend(self.on_announce(d)),
            PGN_HID => out.extend(self.on_hid_command(d)),
            PGN_GROUP_FUNCTION => out.extend(self.on_group_function(d)),
            _ => {}
        }

        out.extend(self.claim.tick(now_ms));

        if !was_claimed && let Some(addr) = self.claim.address() {
            log::info!(
                "quirk(mcu006): claimed address {addr} as a Furuno MCU-006 \
                 (product code {PRODUCT_CODE}); announcing, waiting for an MFD to pair"
            );
            out.extend(self.product_info());
            out.extend(self.status());
        }

        out.extend(self.periodic(now));
        self.update_link();
        out
    }

    fn update_link(&self) {
        let Some(link) = &self.link else { return };
        let state = LinkState {
            addr: self.claim.address(),
            host: self.host.filter(|h| h.name.is_some()).map(|h| h.addr),
        };
        let mut shared = link.lock().unwrap_or_else(|e| e.into_inner());
        if *shared != state {
            *shared = state;
        }
    }

    /// Log what other nodes say that could be about us: any Furuno
    /// proprietary frame, and anything addressed to our address.
    /// The traffic a paired TZMAP9 repeats on its own, its announce every
    /// 5 s and its maintenance poll every 30 s, goes to `trace`; everything
    /// else to `debug`.
    fn log_interesting(&self, d: &DecodedPgn, own: Option<u8>) {
        let to_us = own.is_some_and(|a| a == d.dst);
        let furuno_prop = is_proprietary(d.pgn) && d.data.starts_with(&FURUNO_HEADER);
        if to_us || (furuno_prop && is_remote_pgn(d.pgn)) {
            let from_host = self.host.is_some_and(|h| h.addr == d.src);
            let maintenance_poll = d.pgn == PGN_GROUP_FUNCTION
                && d.data.first() == Some(&0)
                && d.data.get(1..4) == Some(&[0x00, 0xef, 0x01][..]);
            let level = if (from_host && d.pgn == PGN_ANNOUNCE) || maintenance_poll {
                log::Level::Trace
            } else {
                log::Level::Debug
            };
            log::log!(
                level,
                "quirk(mcu006): rx pgn {} ({:#x}) src {} dst {}{}: {}",
                d.pgn,
                d.pgn,
                d.src,
                d.dst,
                if to_us { " [to us]" } else { "" },
                hex(&d.data)
            );
        }
    }

    /// An `0xFF07` from another node. Byte 2 bit 0 set is the pairing
    /// trigger (`FUN_080136e8`); a real remote acts on it only while it
    /// has no host. A TimeZero MFD broadcasts it every 5 s, so repeats
    /// from our own host are only logged at `trace`. Returns the request
    /// for the new host's address claim, which completes the pairing.
    fn on_announce(&mut self, d: &DecodedPgn) -> Option<RawFrame> {
        if !d.data.starts_with(&FURUNO_HEADER) || d.data.get(2).is_none_or(|b| b & 1 == 0) {
            return None;
        }
        if let Some(host) = self.host {
            if host.addr == d.src {
                log::trace!("quirk(mcu006): host src {} announces again", d.src);
            } else {
                log::debug!(
                    "quirk(mcu006): pairing offer from src {} ignored, paired with src {}",
                    d.src,
                    host.addr
                );
            }
            return None;
        }
        log::info!(
            "quirk(mcu006): pairing offer from src {}; taking it as host, \
             requesting its address claim to learn its NAME",
            d.src
        );
        self.host = Some(Host {
            addr: d.src,
            name: None,
        });
        self.claim_request(d.src)
    }

    /// An ISO Request for the address claim of `dst`.
    fn claim_request(&self, dst: u8) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        let pgn = PGN_ISO_ADDRESS_CLAIM;
        Some(RawFrame::new(
            None,
            6,
            PGN_ISO_REQUEST,
            addr,
            dst,
            [pgn as u8, (pgn >> 8) as u8, (pgn >> 16) as u8],
        ))
    }

    /// An address claim: completes a pending pairing, or follows a known
    /// host to a new address.
    fn on_host_claim(&mut self, src: u8, their_name: u64) -> Option<RawFrame> {
        let host = self.host.as_mut()?;
        match host.name {
            None if host.addr == src => {
                host.name = Some(their_name);
                log::info!("quirk(mcu006): paired with host src {src} NAME {their_name:#018x}");
                self.host_frame(ADDR_GLOBAL)
            }
            Some(name) if name == their_name && host.addr != src => {
                log::info!("quirk(mcu006): host moved from {} to {src}", host.addr);
                host.addr = src;
                None
            }
            _ => None,
        }
    }

    /// `0xEF00` addressed to us from the host: `3F 9F 00 <cmd>` with
    /// cmd 0..=2 (`FUN_080133c8` / `FUN_080134a0`).
    fn on_hid_command(&self, d: &DecodedPgn) -> Vec<RawFrame> {
        let Some(addr) = self.claim.address() else {
            return Vec::new();
        };
        if d.dst != addr || !d.data.starts_with(&FURUNO_HEADER) || d.data.get(2) != Some(&0) {
            return Vec::new();
        }
        match d.data.get(3) {
            Some(0) => {
                log::debug!("quirk(mcu006): command 0 from src {}, replying 03", d.src);
                vec![
                    self.hid_reply(addr, d.src, 3),
                    RawFrame::new(
                        None,
                        PRIO,
                        PGN_HID_INFO,
                        addr,
                        ADDR_GLOBAL,
                        [0x3f, 0x9f, 0xf0, 0xff, 0x02, 0x01],
                    ),
                ]
            }
            Some(cmd @ (1 | 2)) => {
                log::debug!(
                    "quirk(mcu006): command {cmd} from src {}, replying 04",
                    d.src
                );
                vec![self.hid_reply(addr, d.src, 4)]
            }
            _ => Vec::new(),
        }
    }

    /// A 126208 Command or Request addressed to us.
    fn on_group_function(&mut self, d: &DecodedPgn) -> Vec<RawFrame> {
        let Some(addr) = self.claim.address() else {
            return Vec::new();
        };
        let data = &d.data;
        let to_us = d.dst == addr;
        if !(to_us || d.dst == ADDR_GLOBAL) || data.len() < 4 {
            return Vec::new();
        }
        let pgn = u32::from_le_bytes([data[1], data[2], data[3], 0]);
        match (data[0], pgn) {
            (1, PGN_STATUS) if to_us => self.on_backlight_command(d, addr),
            (0, PGN_MAINTENANCE) if to_us => self.on_maintenance_request(d, addr),
            (0, PGN_HOST) => self.on_host_request(d, addr, to_us),
            _ => Vec::new(),
        }
    }

    /// 126208 Request for `0x1FF21`, "which MFD are you paired with". A
    /// TZMAP9 broadcasts one to Furuno devices with every step of its
    /// brightness slider. The firmware answers with 130849 to the
    /// requester when the only fields are 1 (manufacturer code, 1855 =
    /// Furuno) and 3 (industry code, 4 = marine); any other field is
    /// refused, with an Acknowledge only when the request
    /// was addressed to the remote.
    fn on_host_request(&self, d: &DecodedPgn, addr: u8, to_us: bool) -> Vec<RawFrame> {
        // Function, PGN, transmission interval (4), interval offset (2).
        let Some(params) = d.data.get(10..).and_then(parse_params) else {
            return Vec::new();
        };
        let codes: Vec<u8> = params
            .iter()
            .map(|&(field, value)| {
                let ok = match field {
                    1 => is_furuno(value),
                    3 => value & 7 == 4,
                    _ => false,
                };
                if ok { 0 } else { PARAM_ERROR }
            })
            .collect();
        if codes.iter().all(|&c| c == 0) {
            log::debug!("quirk(mcu006): src {} asks for our host", d.src);
            return self.host_frame(d.src).into_iter().collect();
        }
        if to_us {
            vec![ack_frame(addr, d.src, &d.data[1..4], &codes)]
        } else {
            Vec::new()
        }
    }

    /// 126208 Command on `0xFF08`. A TZMAP9 sets the backlight right after
    /// pairing (the `0xff08` case of the firmware's command handler): field
    /// 1, the manufacturer code, must be 1855 (Furuno), field 3, the
    /// industry code, 4 (marine); 4 and 5 are ignored, 6 is the
    /// preset and 7 the percentage. Answered with a 126208 Acknowledge
    /// and, when every field passes, the new `0xFF08` status.
    fn on_backlight_command(&mut self, d: &DecodedPgn, addr: u8) -> Vec<RawFrame> {
        let Some(params) = d.data.get(5..).and_then(parse_params) else {
            return Vec::new();
        };
        let (mut preset, mut percent) = (self.backlight_preset, self.backlight_percent);
        let codes: Vec<u8> = params
            .iter()
            .map(|&(field, value)| {
                let ok = match field {
                    1 => is_furuno(value),
                    3 => value & 7 == 4,
                    6 => {
                        preset = value as u8 & 0x0f;
                        true
                    }
                    7 => {
                        percent = value as u8;
                        true
                    }
                    8 => false,
                    _ => true,
                };
                if ok { 0 } else { PARAM_ERROR }
            })
            .collect();
        let mut out = vec![ack_frame(addr, d.src, &d.data[1..4], &codes)];
        if codes.iter().all(|&c| c == 0) {
            log::info!(
                "quirk(mcu006): src {} sets the backlight to preset {preset}, {percent} %",
                d.src
            );
            self.backlight_preset = preset;
            self.backlight_percent = percent;
            out.extend(self.status());
        } else {
            log::info!(
                "quirk(mcu006): refused a backlight command from src {}: codes {codes:?}",
                d.src
            );
        }
        out
    }

    /// 126208 Request for the maintenance PGN `0x1EF00`. The firmware
    /// accepts field 1 (manufacturer code) = 1855, field 3 (industry code)
    /// = 4 and field 4 = `0x0b`, 1 or 2; any
    /// other field is refused. A TZMAP9 asks with field 4 = 5 and a field
    /// 5 every 30 s, which a real remote refuses too: an Acknowledge with
    /// error codes. The replies to the queries it does accept are not
    /// implemented.
    fn on_maintenance_request(&self, d: &DecodedPgn, addr: u8) -> Vec<RawFrame> {
        // Function, PGN, transmission interval (4), interval offset (2).
        let Some(params) = d.data.get(10..).and_then(parse_params) else {
            return Vec::new();
        };
        let codes: Vec<u8> = params
            .iter()
            .map(|&(field, value)| {
                let ok = match field {
                    1 => is_furuno(value),
                    3 => value & 7 == 4,
                    4 => matches!(value, 0x0b | 1 | 2),
                    _ => false,
                };
                if ok { 0 } else { PARAM_ERROR }
            })
            .collect();
        if codes.iter().all(|&c| c == 0) {
            log::warn!(
                "quirk(mcu006): src {} sent a maintenance query the remote accepts \
                 ({params:?}); its reply is not implemented",
                d.src
            );
            return Vec::new();
        }
        log::debug!(
            "quirk(mcu006): refusing maintenance request {params:?} from src {}",
            d.src
        );
        vec![ack_frame(addr, d.src, &d.data[1..4], &codes)]
    }

    fn hid_reply(&self, src: u8, dst: u8, code: u8) -> RawFrame {
        RawFrame::new(
            None,
            PRIO,
            PGN_HID,
            src,
            dst,
            [0x3f, 0x9f, 0x00, code, 0xff, 0xff, 0xff, 0xff],
        )
    }

    fn handle_request(&self, d: &DecodedPgn) -> Vec<RawFrame> {
        let Some(addr) = self.claim.address() else {
            return Vec::new();
        };
        if d.dst != addr && d.dst != ADDR_GLOBAL {
            return Vec::new();
        }
        let Some(requested) = d.data.get(..3) else {
            return Vec::new();
        };
        let requested = u32::from_le_bytes([requested[0], requested[1], requested[2], 0]);
        if d.dst == addr {
            log::debug!(
                "quirk(mcu006): request for pgn {requested} from src {}",
                d.src
            );
        }
        match requested {
            PGN_ISO_ADDRESS_CLAIM => self.claim.respond_to_claim_request().into_iter().collect(),
            PGN_PRODUCT_INFO => self.product_info().into_iter().collect(),
            PGN_PGN_LIST => {
                nmea_responder::pgn_list_frames(addr, d.src, &TX_PGN_LIST, &RX_PGN_LIST)
            }
            PGN_ANNOUNCE => self.announce().into_iter().collect(),
            PGN_STATUS => self.status().into_iter().collect(),
            PGN_HID => vec![self.hid_reply(addr, d.src, 4)],
            PGN_HOST => self.host_frame(d.src).into_iter().collect(),
            _ => Vec::new(),
        }
    }

    /// The 5 s announce and the 60 s heartbeat, once we hold an address.
    fn periodic(&mut self, now: Instant) -> Vec<RawFrame> {
        let mut out = Vec::new();
        let Some(addr) = self.claim.address() else {
            return out;
        };
        if self
            .last_announce
            .is_none_or(|t| now.duration_since(t) >= ANNOUNCE_INTERVAL)
        {
            self.last_announce = Some(now);
            out.extend(self.announce());
            // The firmware asks for claims every 5 s until it knows the
            // host's NAME (`FUN_0800cda4`).
            if let Some(Host {
                addr: host,
                name: None,
            }) = self.host
            {
                out.extend(self.claim_request(host));
            }
        }
        if self
            .last_heartbeat
            .is_none_or(|t| now.duration_since(t) >= HEARTBEAT_INTERVAL)
        {
            self.last_heartbeat = Some(now);
            match nmea_responder::heartbeat_frame(
                addr,
                self.heartbeat_seq,
                HEARTBEAT_INTERVAL.as_millis() as u64,
            ) {
                Ok(f) => out.push(f),
                Err(e) => log::error!("quirk(mcu006): heartbeat encode failed: {e}"),
            }
            self.heartbeat_seq = if self.heartbeat_seq >= 252 {
                0
            } else {
                self.heartbeat_seq + 1
            };
        }
        out
    }

    /// `0xFF07` broadcast: `3F 9F 02 FF FF FF FF FF` (`FUN_0801372c`).
    fn announce(&self) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        Some(RawFrame::new(
            None,
            PRIO,
            PGN_ANNOUNCE,
            addr,
            ADDR_GLOBAL,
            [0x3f, 0x9f, 0x02, 0xff, 0xff, 0xff, 0xff, 0xff],
        ))
    }

    /// `0xFF08` broadcast: `3F 9F 00 (preset<<4|1) percent FF FF FF`
    /// (`FUN_080137f4`).
    fn status(&self) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        Some(RawFrame::new(
            None,
            PRIO,
            PGN_STATUS,
            addr,
            ADDR_GLOBAL,
            [
                0x3f,
                0x9f,
                0x00,
                (self.backlight_preset & 0x0f) << 4 | 1,
                self.backlight_percent,
                0xff,
                0xff,
                0xff,
            ],
        ))
    }

    /// `0x1FF21`: `3F 9F` + the host's NAME (`FUN_08015638`), all ones
    /// while unpaired, as the firmware's stored NAME then is. `None` until
    /// we hold an address.
    fn host_frame(&self, dst: u8) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        let name = self.host.and_then(|h| h.name).unwrap_or(u64::MAX);
        let mut data = FURUNO_HEADER.to_vec();
        data.extend_from_slice(&name.to_le_bytes());
        Some(RawFrame::new(None, PRIO, PGN_HOST, addr, dst, data))
    }

    fn product_info(&self) -> Option<RawFrame> {
        let addr = self.claim.address()?;
        ProductInfo {
            db_version: DB_VERSION,
            product_code: PRODUCT_CODE,
            model_id: MODEL_ID,
            software_version: SOFTWARE_VERSION,
            model_version: MODEL_VERSION,
            model_serial: MODEL_SERIAL,
            certification_level: CERTIFICATION_LEVEL,
            load_equivalency: LOAD_EQUIVALENCY,
        }
        .frame(addr)
    }
}

/// Keyboard report on `0xEF00`: `3F 9F 02` then up to five HID usage IDs,
/// `FF` for an empty slot (`FUN_08013274`).
pub(crate) fn keyboard_report(src: u8, dst: u8, keys: &[u8]) -> RawFrame {
    let mut data = [0x3f, 0x9f, 0x02, 0xff, 0xff, 0xff, 0xff, 0xff];
    for (slot, &key) in data[3..].iter_mut().zip(keys) {
        *slot = key;
    }
    RawFrame::new(None, PRIO, PGN_HID, src, dst, data)
}

/// Pointer report on `0xEF00`: `3F 9F 03 dx dy wheel 01 flags`, bit 0 of
/// `flags` the knob button, bit 3 keypad mode: the CURS/SCROLL button's
/// joystick-as-arrow-keys (`FUN_08013308`).
pub(crate) fn pointer_report(
    src: u8,
    dst: u8,
    dx: i8,
    dy: i8,
    wheel: i8,
    button: bool,
    keypad: bool,
) -> RawFrame {
    let data = [
        0x3f,
        0x9f,
        0x03,
        dx as u8,
        dy as u8,
        wheel as u8,
        0x01,
        button as u8 | (keypad as u8) << 3,
    ];
    RawFrame::new(None, PRIO, PGN_HID, src, dst, data)
}

/// The parameters of a 126208 Command or Request: a count, then `field`
/// and value pairs. Field 1 (the manufacturer code) is two bytes, every
/// other field the remote knows is one. `None` if the payload is short.
fn parse_params(data: &[u8]) -> Option<Vec<(u8, u16)>> {
    let (&count, mut rest) = data.split_first()?;
    let mut params = Vec::with_capacity(count as usize);
    for _ in 0..count {
        let (&field, tail) = rest.split_first()?;
        let size = if field == 1 { 2 } else { 1 };
        let value = tail.get(..size)?;
        params.push((field, value.iter().rev().fold(0, |v, &b| v << 8 | b as u16)));
        rest = &tail[size..];
    }
    Some(params)
}

fn is_furuno(manufacturer_field: u16) -> bool {
    manufacturer_field & 0x7ff == MFG_FURUNO
}

/// A 126208 Acknowledge (`FUN_080138c0`): function 2, the PGN, no PGN or
/// interval error, then one 4-bit code per parameter, two to a byte.
fn ack_frame(src: u8, dst: u8, pgn: &[u8], codes: &[u8]) -> RawFrame {
    let mut data = vec![2, pgn[0], pgn[1], pgn[2], 0, codes.len() as u8];
    for pair in codes.chunks(2) {
        data.push(pair[0] & 0x0f | pair.get(1).map_or(0, |c| c << 4));
    }
    RawFrame::new(None, 3, PGN_GROUP_FUNCTION, src, dst, data)
}

/// The remote's NAME. The unique number is derived from the host machine,
/// as for the Motion Sensor; the manufacturer differs, so the two NAMEs
/// never clash.
fn build_name(state_dir: Option<&std::path::Path>) -> u64 {
    let unique = crate::engine::os::unique_number(state_dir);
    crate::io::name::Name::new(MFG_FURUNO, unique)
        .device_function(DEVICE_FUNCTION)
        .device_class(DEVICE_CLASS)
        .device_instance(DEVICE_INSTANCE)
        .system_instance(SYSTEM_INSTANCE)
        .to_u64()
}

/// The proprietary PGN ranges: 61184, 65280..=65535, 126720, 130816..=131071.
fn is_proprietary(pgn: u32) -> bool {
    matches!(pgn, 61184 | 65280..=65535 | 126720 | 130816..=131071)
}

/// The proprietary PGNs the remote sends or receives. Other Furuno
/// proprietary traffic (heave, 6DOF, …) is not logged.
fn is_remote_pgn(pgn: u32) -> bool {
    matches!(
        pgn,
        61184 | 65285 | 65286 | 65287 | 65288 | 126720 | 130816 | 130822 | 130823 | 130849
    )
}

fn hex(data: &[u8]) -> String {
    data.iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::{PgnDatabase, Units};

    fn si() -> &'static PgnDatabase {
        PgnDatabase::embedded(Units::Si)
    }

    fn dec(f: &RawFrame) -> DecodedPgn {
        si().decode(f).expect("decodes")
    }

    fn request(src: u8, dst: u8, target: u32) -> DecodedPgn {
        let data = [target as u8, (target >> 8) as u8, (target >> 16) as u8];
        dec(&RawFrame::new(None, 6, PGN_ISO_REQUEST, src, dst, data))
    }

    fn claim(src: u8, name: u64) -> DecodedPgn {
        dec(&RawFrame::new(
            None,
            6,
            PGN_ISO_ADDRESS_CLAIM,
            src,
            ADDR_GLOBAL,
            name.to_le_bytes(),
        ))
    }

    fn spare(src: u8) -> DecodedPgn {
        let data = [0x00, 0xff, 0x7f, 0xcd, 0x00, 0x01, 0xfc, 0xff];
        dec(&RawFrame::new(None, 3, 127257, src, ADDR_GLOBAL, data))
    }

    fn drive_to_claimed(m: &mut Mcu006, t0: Instant) -> u8 {
        m.process(&spare(200), t0);
        m.process(&spare(200), t0 + Duration::from_millis(1001));
        m.process(&spare(200), t0 + Duration::from_millis(1300));
        m.claim.address().expect("claimed")
    }

    #[test]
    fn name_matches_the_firmware() {
        let name = build_name(None);
        // Everything above the 21-bit unique number is fixed.
        assert_eq!(name & !0x1f_ffff, 0xcfdc_82ff_e7e0_0000);
    }

    #[test]
    fn claims_110_and_announces_product_info() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        m.process(&spare(200), t0);
        m.process(&spare(200), t0 + Duration::from_millis(1001));
        let out = m.process(&spare(200), t0 + Duration::from_millis(1300));
        assert_eq!(m.claim.address(), Some(PREFERRED_ADDRESS));
        let pi = out
            .iter()
            .find(|f| f.pgn == PGN_PRODUCT_INFO)
            .expect("product info on claim");
        let d = dec(pi);
        use crate::engine::field::product_information as p;
        assert_eq!(
            d.field(p::PRODUCT_CODE).and_then(|f| f.value.as_i64()),
            Some(PRODUCT_CODE)
        );
        assert_eq!(
            d.field(p::MODEL_ID).and_then(|f| f.value.as_str()),
            Some(MODEL_ID)
        );
        let status = out.iter().find(|f| f.pgn == PGN_STATUS).expect("status");
        assert_eq!(
            status.data.as_slice(),
            [0x3f, 0x9f, 0, 1, 50, 0xff, 0xff, 0xff]
        );
        let ann = out
            .iter()
            .find(|f| f.pgn == PGN_ANNOUNCE)
            .expect("announce");
        assert_eq!(
            ann.data.as_slice(),
            [0x3f, 0x9f, 2, 0xff, 0xff, 0xff, 0xff, 0xff]
        );
        assert_eq!(ann.prio, 7);
    }

    #[test]
    fn own_claim_fed_back_keeps_the_address() {
        // The pipeline loops our frames back. With System Instance 15 the
        // echoed NAME must still read as ours, or we walk every address.
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        m.process(&spare(200), t0);
        let out = m.process(&spare(200), t0 + Duration::from_millis(1001));
        let own = out
            .iter()
            .find(|f| f.pgn == PGN_ISO_ADDRESS_CLAIM && f.src == PREFERRED_ADDRESS)
            .expect("claim sent");
        let out = m.process(&dec(own), t0 + Duration::from_millis(1100));
        assert!(
            out.iter().all(|f| f.pgn != PGN_ISO_ADDRESS_CLAIM),
            "{out:?}"
        );
        m.process(&spare(200), t0 + Duration::from_millis(1400));
        assert_eq!(m.claim.address(), Some(PREFERRED_ADDRESS));
    }

    #[test]
    fn announces_every_five_seconds() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        drive_to_claimed(&mut m, t0);
        let count = |out: &[RawFrame]| out.iter().filter(|f| f.pgn == PGN_ANNOUNCE).count();
        assert_eq!(
            count(&m.process(&spare(200), t0 + Duration::from_secs(3))),
            0
        );
        assert_eq!(
            count(&m.process(&spare(200), t0 + Duration::from_secs(7))),
            1
        );
        assert_eq!(
            count(&m.process(&spare(200), t0 + Duration::from_secs(8))),
            0
        );
    }

    #[test]
    fn answers_pgn_list_with_the_firmware_lists() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        let out = m.process(&request(3, addr, PGN_PGN_LIST), t0 + Duration::from_secs(2));
        let lists: Vec<_> = out.iter().filter(|f| f.pgn == PGN_PGN_LIST).collect();
        assert_eq!(lists.len(), 2);
        // Function byte 0 = transmit list, then 3 bytes per PGN.
        assert_eq!(lists[0].data.len(), 1 + 3 * TX_PGN_LIST.len());
        assert_eq!(lists[0].dst, 3);
    }

    #[test]
    fn pairs_on_trigger_then_learns_the_host_name() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        drive_to_claimed(&mut m, t0);
        let trigger = dec(&RawFrame::new(
            None,
            7,
            PGN_ANNOUNCE,
            3,
            ADDR_GLOBAL,
            [0x3f, 0x9f, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff],
        ));
        let out = m.process(&trigger, t0 + Duration::from_secs(2));
        assert_eq!(
            m.host,
            Some(Host {
                addr: 3,
                name: None
            })
        );
        let req = out
            .iter()
            .find(|f| f.pgn == PGN_ISO_REQUEST)
            .expect("asks the host for its claim");
        assert_eq!((req.dst, req.data.as_slice()), (3, &[0x00, 0xee, 0x00][..]));

        let mfd_name = 0xa0a0_0000_e7e1_2345;
        let out = m.process(&claim(3, mfd_name), t0 + Duration::from_secs(3));
        assert_eq!(m.host.unwrap().name, Some(mfd_name));
        let host = out.iter().find(|f| f.pgn == PGN_HOST).expect("130849 sent");
        assert_eq!(&host.data[..2], &FURUNO_HEADER);
        assert_eq!(&host.data[2..], &mfd_name.to_le_bytes());

        // The host moves; we follow it by NAME.
        m.process(&claim(9, mfd_name), t0 + Duration::from_secs(4));
        assert_eq!(m.host.unwrap().addr, 9);
    }

    #[test]
    fn other_remotes_announce_does_not_pair() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        drive_to_claimed(&mut m, t0);
        let other_remote = dec(&RawFrame::new(
            None,
            7,
            PGN_ANNOUNCE,
            111,
            ADDR_GLOBAL,
            [0x3f, 0x9f, 0x02, 0xff, 0xff, 0xff, 0xff, 0xff],
        ));
        m.process(&other_remote, t0 + Duration::from_secs(2));
        assert_eq!(m.host, None);
    }

    #[test]
    fn acknowledges_the_backlight_command() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        // As a TZMAP9 sent it right after pairing: preset 4, 25 %.
        let cmd = dec(&RawFrame::new(
            None,
            3,
            PGN_GROUP_FUNCTION,
            1,
            addr,
            [
                0x01, 0x08, 0xff, 0x00, 0xf8, 0x06, 0x01, 0x3f, 0x07, 0x03, 0x04, 0x04, 0x00, 0x05,
                0x01, 0x06, 0x04, 0x07, 0x19,
            ],
        ));
        let out = m.process(&cmd, t0 + Duration::from_secs(2));
        let ack = out
            .iter()
            .find(|f| f.pgn == PGN_GROUP_FUNCTION)
            .expect("acknowledged");
        assert_eq!((ack.dst, ack.prio), (1, 3));
        assert_eq!(
            ack.data.as_slice(),
            [0x02, 0x08, 0xff, 0x00, 0x00, 0x06, 0, 0, 0]
        );
        let status = out.iter().find(|f| f.pgn == PGN_STATUS).expect("status");
        assert_eq!(
            status.data.as_slice(),
            [0x3f, 0x9f, 0, 0x41, 25, 0xff, 0xff, 0xff]
        );
    }

    #[test]
    fn refuses_the_tzmap_maintenance_request() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        // As a TZMAP9 sends it every 30 s.
        let req = dec(&RawFrame::new(
            None,
            3,
            PGN_GROUP_FUNCTION,
            1,
            addr,
            [
                0x00, 0x00, 0xef, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x04, 0x01, 0x3f, 0x07,
                0x03, 0x04, 0x04, 0x05, 0x05, 0x00,
            ],
        ));
        let out = m.process(&req, t0 + Duration::from_secs(2));
        let ack = out
            .iter()
            .find(|f| f.pgn == PGN_GROUP_FUNCTION)
            .expect("acknowledged");
        assert_eq!(ack.dst, 1);
        assert_eq!(
            ack.data.as_slice(),
            [0x02, 0x00, 0xef, 0x01, 0x00, 0x04, 0x00, 0x44]
        );
    }

    #[test]
    fn link_shows_address_and_host_once_paired() {
        let link = Link::default();
        let mut m = Mcu006::new(None);
        m.set_link(link.clone());
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        assert_eq!(
            *link.lock().unwrap(),
            LinkState {
                addr: Some(addr),
                host: None
            }
        );
        let trigger = dec(&RawFrame::new(
            None,
            7,
            PGN_ANNOUNCE,
            1,
            ADDR_GLOBAL,
            [0x3f, 0x9f, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff],
        ));
        m.process(&trigger, t0 + Duration::from_secs(2));
        assert_eq!(
            link.lock().unwrap().host,
            None,
            "not until the NAME is known"
        );
        m.process(
            &claim(1, 0xc078_af00_e7e4_252c),
            t0 + Duration::from_secs(3),
        );
        assert_eq!(link.lock().unwrap().host, Some(1));
    }

    #[test]
    fn builds_the_reports_that_moved_a_tzmap9() {
        // The frames verified on a TZMAP9 on 2026-10-21.
        let f = pointer_report(110, 1, 16, 0, 0, false, false);
        assert_eq!((f.pgn, f.prio, f.src, f.dst), (PGN_HID, 7, 110, 1));
        assert_eq!(f.data.as_slice(), [0x3f, 0x9f, 0x03, 0x10, 0, 0, 1, 0]);
        let f = pointer_report(110, 1, 0, 0, -1, true, false);
        assert_eq!(f.data.as_slice(), [0x3f, 0x9f, 0x03, 0, 0, 0xff, 1, 1]);
        // Keypad mode is bit 3, next to the button in bit 0.
        let f = pointer_report(110, 1, 0, 0, 0, true, true);
        assert_eq!(f.data.as_slice(), [0x3f, 0x9f, 0x03, 0, 0, 0, 1, 0x09]);
        let f = keyboard_report(110, 1, &[0x52]);
        assert_eq!(
            f.data.as_slice(),
            [0x3f, 0x9f, 0x02, 0x52, 0xff, 0xff, 0xff, 0xff]
        );
        let f = keyboard_report(110, 1, &[]);
        assert_eq!(
            f.data.as_slice(),
            [0x3f, 0x9f, 0x02, 0xff, 0xff, 0xff, 0xff, 0xff]
        );
    }

    #[test]
    fn answers_the_broadcast_host_request() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        drive_to_claimed(&mut m, t0);
        // As a TZMAP9 broadcasts it with each brightness step.
        let req = dec(&RawFrame::new(
            None,
            3,
            PGN_GROUP_FUNCTION,
            1,
            ADDR_GLOBAL,
            [
                0x00, 0x21, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x02, 0x01, 0x3f, 0x07,
                0x03, 0x04,
            ],
        ));
        let out = m.process(&req, t0 + Duration::from_secs(2));
        let host = out.iter().find(|f| f.pgn == PGN_HOST).expect("130849");
        assert_eq!(host.dst, 1);
        assert_eq!(&host.data[2..], &[0xff; 8], "unpaired: all ones");

        // A request with a field the remote does not know: silent when
        // broadcast.
        let odd = dec(&RawFrame::new(
            None,
            3,
            PGN_GROUP_FUNCTION,
            1,
            ADDR_GLOBAL,
            [
                0x00, 0x21, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x04, 0x05,
            ],
        ));
        assert!(m.process(&odd, t0 + Duration::from_secs(2)).is_empty());
    }

    #[test]
    fn answers_host_commands() {
        let mut m = Mcu006::new(None);
        let t0 = Instant::now();
        let addr = drive_to_claimed(&mut m, t0);
        let cmd = |c: u8| {
            dec(&RawFrame::new(
                None,
                7,
                PGN_HID,
                3,
                addr,
                [0x3f, 0x9f, 0x00, c, 0xff, 0xff, 0xff, 0xff],
            ))
        };
        let out = m.process(&cmd(0), t0 + Duration::from_secs(2));
        let reply = out.iter().find(|f| f.pgn == PGN_HID).expect("reply");
        assert_eq!(reply.dst, 3);
        assert_eq!(
            reply.data.as_slice(),
            [0x3f, 0x9f, 0, 3, 0xff, 0xff, 0xff, 0xff]
        );
        assert!(out.iter().any(|f| f.pgn == PGN_HID_INFO));

        let out = m.process(&cmd(2), t0 + Duration::from_secs(2));
        let reply = out.iter().find(|f| f.pgn == PGN_HID).expect("reply");
        assert_eq!(reply.data[3], 4);
        assert!(
            m.process(&cmd(3), t0 + Duration::from_secs(2))
                .iter()
                .all(|f| f.pgn != PGN_HID)
        );
    }
}
