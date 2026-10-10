// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense NGT-1 gateway codec.
//!
//! Wraps [`crate::engine::format::ngt1`]: received bytes go through
//! `Ngt1Decoder`, frames for the bus go out as `N2K_MSG_SEND` (0x94)
//! messages. On connect the codec asks for the gateway's product
//! information, and logs it, then sets its operating mode (BEM Set
//! Operating Mode, [`Config::operating_mode`]); the gateway answers with the
//! mode in force, which is logged, and a mismatch is warned about.
//!
//! There is no keepalive. canboat C's `actisense-serial` long set the mode
//! again every 20 s, added in 2020 against NGT-1 hangs; the Actisense SDK
//! has no keepalive, and the gateway keeps the mode in non-volatile memory.
//! An NGT-1 with firmware 2.690 (2026-10) ran 15 minutes without one,
//! through 5 minutes of nothing written and stalls of up to 60 s in reading
//! with commands written meanwhile, and never hung.
//!
//! Setting the mode is not free: that NGT-1 then drops what is written to
//! it, BEM commands for 50 to 200 ms and N2K messages for 400 to 450 ms. So
//! frames for the bus that [`Codec::send`] is given in the 500 ms after
//! the mode is set come back empty, held, and go out as [`Event::Send`]
//! from [`Codec::receive`] or [`Codec::tick`] once that time is over.
//!
//! The gateway's own BEM messages come out as canboat's `Actisense: …`
//! PGNs (`0x40000` + the BEM id), as canboat C's `actisense-serial` gives
//! them. A Startup Status (the gateway restarted) makes the codec set the
//! operating mode and the transmit list again; an Error Report or a
//! Negative Ack is logged with the Actisense SDK's name for its error. Synthetic PGNs (`>= 0x40000`) are refused, matching
//! canboat's behaviour.
//!
//! Alongside the bus traffic the codec emits the synthetic `NMEA 2000
//! gateway: network status` (PGN 262400), as canboat C's `actisense-serial`
//! does. With [`Config::pgn_lists`] naming Transmit PGNs it also brings the
//! gateway's Transmit PGN Enable list up to date after startup (see
//! [`super::ngt1_tx_list`]).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::engine::RawFrame;
use crate::engine::format::ikonvert::{NetworkStatus, build_network_status};
use crate::engine::format::ngt1::{
    BEM_ERROR_REPORT, BEM_NEGATIVE_ACK, BEM_OPERATING_MODE, BEM_PRODUCT_INFO, BEM_STARTUP_STATUS,
    BEM_SYSTEM_STATUS, BemResponse, Ngt1Decoder, NgtEvent, OperatingMode, ProductInfo,
    describe_error, encode_get_product_info, encode_n2k_send_frame, encode_set_operating_mode,
    startup_status,
};
use crate::engine::pgn_list::{self, PgnListStatus, PgnListSupport, PgnLists, TxGate};

use super::ngt1_tx_list::{NGT_MSG_RECEIVED, TxListRecord, TxListSync};
use super::{Codec, Event, Refused, SYNTHETIC_PGN_START};
use crate::engine::format_iso_ms;

/// The interval of the keepalive the codec no longer sends: the C
/// `actisense-serial` sets the operating mode again every 20 s.
#[deprecated(note = "the NGT-1 codec sends no keepalive; `Ngt1::keepalive` returns `None`")]
pub const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(20);

/// `Actisense: System status` — `ACTISENSE_BEM + 0xf2`. The NGT-1 sends
/// it about once a second, but only when P-codes are enabled for the
/// port, so its two fields may never arrive.
const ACTISENSE_SYSTEM_STATUS_PGN: u32 = ACTISENSE_BEM_PGN + BEM_SYSTEM_STATUS as u32;

/// The canboat PGN a BEM message comes out as, less the BEM id:
/// `Actisense: …`.
const ACTISENSE_BEM_PGN: u32 = SYNTHETIC_PGN_START;

/// How often to emit the synthetic gateway network status.
const NETWORK_STATUS_INTERVAL_MS: u64 = 5_000;

/// NGT-1 settings. `Config::default()` puts the gateway in NGT Transfer Rx
/// All Mode and leaves its lists alone.
#[derive(Debug, Clone)]
pub struct Config {
    /// The operating mode to set: [`OperatingMode::NgTransferRxAll`] (the
    /// default) forwards every received PGN,
    /// [`OperatingMode::NgTransferNormal`] only those on the gateway's
    /// Receive PGN Enable list.
    pub operating_mode: OperatingMode,
    /// PGNs the application sends and reads. The Transmit PGNs are added
    /// to the gateway's Transmit PGN Enable list when missing from it —
    /// the NGT-1 does not transmit a PGN that is not there. The Receive
    /// PGNs are not used: the gateway runs in receive-all mode.
    ///
    /// **⚠️ ONCE `pgn_lists.tx` NAMES ANY PGN, EVERY PGN NOT ON IT IS
    /// REFUSED: THE CODEC DOES NOT SEND IT.** The NGT-1 only transmits the
    /// PGNs in its list, and the codec sets that list once, at startup, so
    /// name every PGN you will send before opening the device.
    /// Network-management PGNs are always sent. With `tx` empty the
    /// gateway's list is left alone and nothing is refused.
    pub pgn_lists: PgnLists,
    /// PGNs canboat itself transmits through the gateway (a quirk's, such
    /// as `wmm`'s 127258). Once `pgn_lists.tx` names a PGN they are enabled
    /// with it and allowed; on their own they neither write the gateway's
    /// list nor make the codec refuse anything.
    pub extra_tx_pgns: Vec<u32>,
    /// What this run has learned about the gateway's transmit list. Share
    /// one across reconnects (clone the `Arc` into each session's
    /// `Config`), so a reconnect does not write again what is known.
    pub tx_list_record: Arc<Mutex<TxListRecord>>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            operating_mode: OperatingMode::NgTransferRxAll,
            pgn_lists: PgnLists::default(),
            extra_tx_pgns: Vec::new(),
            tx_list_record: Arc::default(),
        }
    }
}

/// The Transmit PGNs to enable: the named ones and canboat's own — none
/// when nothing is named, so the gateway's list is left alone.
fn wanted_tx_pgns(named: &[u32], extra: &[u32]) -> (Vec<u32>, Vec<u32>) {
    if named.is_empty() {
        return (Vec::new(), Vec::new());
    }
    pgn_list::merge(&[], &[named, extra].concat())
}

/// How the gateway takes the `named` lists (and canboat's own `extra`
/// PGNs): a named Transmit list is written into it
/// ([`PgnListSupport::Pushed`]), otherwise its list is left
/// ([`PgnListSupport::Untouched`]); there is no known way to set what it
/// advertises as received. `dropped` names the PGNs that do not fit.
pub fn pgn_list_status(named: &PgnLists, extra: &[u32]) -> PgnListStatus {
    PgnListStatus {
        tx: if named.tx.is_empty() {
            PgnListSupport::Untouched
        } else {
            PgnListSupport::Pushed
        },
        rx: if named.rx.is_empty() {
            PgnListSupport::Untouched
        } else {
            PgnListSupport::Unsupported
        },
        dropped: wanted_tx_pgns(&named.tx, extra).1,
    }
}

/// The NGT-1 protocol; see the [module docs](self).
pub struct Ngt1 {
    inner: Ngt1Decoder,
    /// State for the synthetic network status, so a consumer sees the same
    /// per-gateway record whichever gateway is feeding it.
    net: NetworkStatusState,
    /// Brings the gateway's Transmit PGN Enable list up to date.
    tx_list: TxListSync,
    gate: Option<TxGate>,
    /// The operating mode to set.
    mode: OperatingMode,
    /// The mode the gateway last reported, to log only a change.
    reported_mode: Option<OperatingMode>,
    /// The gateway's product information as its parts arrive.
    product_info: ProductInfo,
    /// Whether frames for the bus wait for a Set Operating Mode to settle.
    settle: Settle,
    /// Frames for the bus held meanwhile, encoded.
    held: Vec<Vec<u8>>,
}

/// How long after Set Operating Mode frames for the bus are held: an NGT-1
/// (firmware 2.690) drops everything written in the 400 to 450 ms after
/// it, N2K messages and BEM commands alike.
const MODE_SETTLE_MS: u64 = 500;

/// Holding frames for the bus after a Set Operating Mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Settle {
    /// Nothing held.
    Settled,
    /// Set Operating Mode was written by [`Codec::open`], which has no
    /// clock; the hold runs from the first time the codec is given.
    Opened,
    /// Held until then.
    Until(u64),
}

struct NetworkStatusState {
    /// When the codec first saw the time: the uptime's zero, and the
    /// wall-clock status timer's base. `None` until then.
    start_ms: Option<u64>,
    next_ms: u64,
    /// Distinct source addresses seen, for the device count.
    seen: [bool; 256],
    /// Ch1 Rx Load and Error ID, cached from the last System Status.
    /// Stay `None` — and ride the canboat sentinel — until one arrives.
    load_pct: Option<u8>,
    errors: Option<u32>,
}

impl Ngt1 {
    /// A codec with [`Config`].
    pub fn new(config: Config) -> Self {
        let (tx_pgns, dropped) = wanted_tx_pgns(&config.pgn_lists.tx, &config.extra_tx_pgns);
        if !dropped.is_empty() {
            log::warn!(
                "ngt1: not enabling transmit PGNs {dropped:?} (invalid, or the list is full)"
            );
        }
        Self {
            inner: Ngt1Decoder::new(),
            net: NetworkStatusState {
                start_ms: None,
                next_ms: 0,
                seen: [false; 256],
                load_pct: None,
                errors: None,
            },
            tx_list: TxListSync::new(tx_pgns, config.tx_list_record.clone()),
            gate: TxGate::new("ngt1", &config.pgn_lists.tx, &config.extra_tx_pgns),
            mode: config.operating_mode,
            reported_mode: None,
            product_info: ProductInfo::default(),
            settle: Settle::Settled,
            held: Vec::new(),
        }
    }

    /// Start or end holding frames for the bus, as the time goes by.
    fn settle(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        match self.settle {
            Settle::Opened => self.settle = Settle::Until(now_ms + MODE_SETTLE_MS),
            Settle::Until(until) if now_ms >= until => {
                self.settle = Settle::Settled;
                if !self.held.is_empty() {
                    log::debug!(
                        "ngt1: sending {} frames held after Set Operating Mode",
                        self.held.len()
                    );
                }
                send_all(events, std::mem::take(&mut self.held));
            }
            _ => {}
        }
    }

    /// Act on a BEM message from the gateway: pass it on as an `Actisense:
    /// …` PGN, and log or answer it.
    fn on_bem(&mut self, payload: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        let Some(response) = BemResponse::parse(payload) else {
            return;
        };
        match response.bem {
            BEM_OPERATING_MODE => self.on_operating_mode(&response),
            BEM_PRODUCT_INFO => {
                if self.product_info.add(&response) {
                    let info = &self.product_info;
                    log::info!(
                        "ngt1: gateway {} serial {}, software {}, hardware {}, product code {}, NMEA 2000 {}.{:03}",
                        info.model,
                        info.serial_number,
                        info.software_version,
                        info.hardware_version,
                        info.product_code,
                        info.nmea2000_version / 1000,
                        info.nmea2000_version % 1000
                    );
                    if let Some(firmware) = info.firmware() {
                        self.tx_list.set_firmware(firmware);
                    }
                }
            }
            BEM_STARTUP_STATUS => {
                if let Some((firmware, reset)) = startup_status(&response) {
                    log::warn!(
                        "ngt1: the gateway restarted (firmware {}.{:03}, reset status {reset:#x}); setting it up again",
                        firmware / 1000,
                        firmware % 1000
                    );
                }
                self.reported_mode = None;
                self.product_info = ProductInfo::default();
                self.tx_list.restart();
                // Product Info first, as in `open`.
                events.push(Event::Send(encode_get_product_info()));
                events.push(Event::Send(encode_set_operating_mode(self.mode)));
                // Frames already held keep their place, ahead of new ones.
                self.settle = Settle::Until(now_ms + MODE_SETTLE_MS);
            }
            BEM_ERROR_REPORT => log::warn!(
                "ngt1: the gateway reports error {}",
                describe_error(response.error)
            ),
            BEM_NEGATIVE_ACK => log::warn!(
                "ngt1: the gateway refused a command: error {}",
                describe_error(response.error)
            ),
            _ => {}
        }
        let frame = RawFrame::new(
            Some(format_iso_ms(now_ms)),
            0,
            ACTISENSE_BEM_PGN + u32::from(response.bem),
            0,
            0,
            payload[1..].iter().copied(),
        );
        self.note_frame(frame, now_ms, events);
    }

    /// Check the gateway's answer to Set Operating Mode: log the mode it
    /// reports when that changes, and warn when it is not the one set.
    fn on_operating_mode(&mut self, response: &BemResponse<'_>) {
        let Some(mode) = response.operating_mode() else {
            return;
        };
        if response.error != 0 {
            log::warn!(
                "ngt1: the gateway refused {} (error {}); it stays in {mode}",
                self.mode,
                describe_error(response.error)
            );
        }
        if self.reported_mode == Some(mode) {
            return;
        }
        self.reported_mode = Some(mode);
        if mode == self.mode {
            log::info!(
                "ngt1: gateway model {:#06x}, serial {}, in {mode}",
                response.model_id,
                response.serial
            );
        } else {
            log::warn!(
                "ngt1: gateway model {:#06x}, serial {}, is in {mode}, not {}",
                response.model_id,
                response.serial,
                self.mode
            );
        }
    }

    /// Start the clock on the first time seen.
    fn started(&mut self, now_ms: u64) {
        if self.net.start_ms.is_none() {
            self.net.start_ms = Some(now_ms);
            self.net.next_ms = now_ms + NETWORK_STATUS_INTERVAL_MS;
        }
    }

    /// Per-frame bookkeeping for the synthetic network status: note the
    /// source address, and harvest the two fields `Actisense: System
    /// status` carries. Pushes the frame itself, plus a fresh status
    /// record when that message is what arrived.
    fn note_frame(&mut self, frame: RawFrame, now_ms: u64, events: &mut Vec<Event>) {
        // The gateway's own messages come from no bus device.
        if frame.pgn < SYNTHETIC_PGN_START {
            self.net.seen[frame.src as usize] = true;
        }
        // Frame data drops the subcommand byte, so the C's msg[8..11]
        // and msg[14] are data[7..10] and data[13] here.
        if frame.pgn == ACTISENSE_SYSTEM_STATUS_PGN {
            if frame.data.len() >= 11 {
                self.net.errors = Some(u32::from_le_bytes([
                    frame.data[7],
                    frame.data[8],
                    frame.data[9],
                    frame.data[10],
                ]));
            }
            if frame.data.len() >= 14 {
                self.net.load_pct = Some(frame.data[13]);
            }
            events.push(Event::Frame(frame));
            // A System Status arrival is the authoritative trigger for
            // a fresh record, as it is in the C — which also makes this
            // work during a file replay, where the wall-clock fallback
            // never fires.
            self.emit_network_status(now_ms, events);
            return;
        }
        events.push(Event::Frame(frame));
    }

    /// Emit the synthetic PGN 262400 record and re-arm the timer.
    fn emit_network_status(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        self.started(now_ms);
        let start_ms = self.net.start_ms.unwrap_or(now_ms);
        let device_count = self.net.seen.iter().filter(|s| **s).count().min(255) as u8;
        let frame = build_network_status(
            NetworkStatus {
                load_pct: self.net.load_pct,
                errors: self.net.errors,
                device_count: Some(device_count),
                uptime_s: Some(now_ms.saturating_sub(start_ms).div_euclid(1000) as u32),
                // The NGT-1 reports neither its own address nor any
                // rejected-TX count, so both stay at the canboat
                // "no data" sentinel.
                gateway_addr: 0xff,
                rejected_tx: None,
            },
            Some(format_iso_ms(now_ms)),
        );
        events.push(Event::Frame(frame));
        self.net.next_ms = now_ms + NETWORK_STATUS_INTERVAL_MS;
    }
}

impl Default for Ngt1 {
    fn default() -> Self {
        Self::new(Config::default())
    }
}

/// Queue each command for the gateway.
fn send_all(events: &mut Vec<Event>, commands: Vec<Vec<u8>>) {
    events.extend(commands.into_iter().map(Event::Send));
}

impl Codec for Ngt1 {
    /// Product Info goes first: an NGT-1 (firmware 2.690) ignores what
    /// follows Set Operating Mode for a while. For the same reason frames
    /// for the bus are held until [`MODE_SETTLE_MS`] after the first
    /// [`Codec::receive`] or [`Codec::tick`].
    fn open(&mut self) -> Vec<u8> {
        self.settle = Settle::Opened;
        let mut out = encode_get_product_info();
        out.extend(encode_set_operating_mode(self.mode));
        out
    }

    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        self.tick(now_ms, events);
        for ev in self.inner.push_bytes(bytes) {
            match ev {
                NgtEvent::Message(msg) if msg.command == NGT_MSG_RECEIVED => {
                    send_all(events, self.tx_list.on_message(&msg.payload, now_ms));
                    self.on_bem(&msg.payload, now_ms, events);
                }
                NgtEvent::Message(msg) => {
                    if let Some(mut frame) = msg.to_raw_frame() {
                        // The message carries the NGT-1's ms-since-power-up
                        // clock; a live stream wants wall-clock time, as
                        // canboat C's actisense-serial and the iKonvert
                        // codec give it.
                        frame.timestamp = Some(format_iso_ms(now_ms));
                        self.note_frame(frame, now_ms, events);
                    }
                }
                NgtEvent::Error(e) => events.push(Event::Error(e.to_string())),
            }
        }
    }

    /// The codec's deadlines, run on every receive too, so they advance
    /// on a quiet bus.
    fn tick(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        self.started(now_ms);
        self.settle(now_ms, events);
        // Wall-clock fallback, for a gateway whose P-codes are off and
        // so never sends a System Status to trigger on.
        if now_ms >= self.net.next_ms {
            self.emit_network_status(now_ms, events);
        }
        if !self.tx_list.is_done() {
            send_all(events, self.tx_list.on_tick(now_ms));
        }
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        if frame.pgn >= SYNTHETIC_PGN_START {
            log::debug!("ngt1: skipping synthetic PGN {}", frame.pgn);
            return Err(Refused::Synthetic);
        }
        if let Some(gate) = &self.gate
            && !gate.allows(frame.pgn)
        {
            return Err(Refused::NotOnTxList);
        }
        // Six header bytes + data must fit the one-byte message length.
        if 6 + frame.data.len() > u8::MAX as usize {
            return Err(Refused::TooLarge);
        }
        let bytes = encode_n2k_send_frame(frame);
        if self.settle == Settle::Settled {
            Ok(bytes)
        } else {
            // Written now it would be lost; `tick` sends it later.
            self.held.push(bytes);
            Ok(Vec::new())
        }
    }

    /// None: the NGT-1 needs no keepalive (see the module documentation).
    fn keepalive(&self) -> Option<(Duration, Vec<u8>)> {
        None
    }
}

#[cfg(test)]
mod settle_tests {
    use super::*;

    const NOW: u64 = 1_780_082_164_826;

    fn frame(pgn: u32) -> RawFrame {
        RawFrame::new(None, 6, pgn, 0, 21, vec![0x14, 0xf0, 0x01])
    }

    fn sent(events: &[Event]) -> Vec<&Vec<u8>> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Send(b) => Some(b),
                _ => None,
            })
            .collect()
    }

    /// Without `open` nothing was set, so nothing is held.
    #[test]
    fn frames_go_straight_out_without_open() {
        let mut d = Ngt1::default();
        assert_eq!(
            d.send(&frame(59904)),
            Ok(encode_n2k_send_frame(&frame(59904)))
        );
    }

    /// Frames given before and in the 500 ms after the first tick are
    /// held, then sent in order.
    #[test]
    fn frames_after_open_are_held_until_the_mode_settles() {
        let mut d = Ngt1::default();
        d.open();
        assert_eq!(d.send(&frame(59904)), Ok(Vec::new()));
        let mut events = Vec::new();
        d.tick(NOW, &mut events);
        assert!(sent(&events).is_empty());
        assert_eq!(d.send(&frame(126208)), Ok(Vec::new()));
        d.tick(NOW + MODE_SETTLE_MS - 1, &mut events);
        assert!(sent(&events).is_empty(), "still settling");
        d.tick(NOW + MODE_SETTLE_MS, &mut events);
        assert_eq!(
            sent(&events),
            [
                &encode_n2k_send_frame(&frame(59904)),
                &encode_n2k_send_frame(&frame(126208))
            ]
        );
        assert_eq!(
            d.send(&frame(59904)),
            Ok(encode_n2k_send_frame(&frame(59904)))
        );
    }

    /// A refused frame is refused at once, not held.
    #[test]
    fn a_refused_frame_is_not_held() {
        let mut d = Ngt1::default();
        d.open();
        assert_eq!(d.send(&frame(SYNTHETIC_PGN_START)), Err(Refused::Synthetic));
        assert!(d.held.is_empty());
    }

    /// The gateway restarting sets the mode again, and holds frames again.
    #[test]
    fn a_startup_status_holds_frames_again() {
        let mut d = Ngt1::default();
        let mut payload = vec![BEM_STARTUP_STATUS, 1, 0x0e, 0];
        payload.extend_from_slice(&[0; 8]);
        payload.extend_from_slice(&[0x8e, 0x0a, 0]);
        let mut wire = Vec::new();
        crate::engine::format::ngt1::encode_ngt_message(NGT_MSG_RECEIVED, &payload, &mut wire);
        let mut events = Vec::new();
        d.receive(&wire, NOW, &mut events);
        assert_eq!(d.send(&frame(59904)), Ok(Vec::new()));
        events.clear();
        d.tick(NOW + MODE_SETTLE_MS, &mut events);
        assert_eq!(sent(&events), [&encode_n2k_send_frame(&frame(59904))]);
    }
}

#[cfg(test)]
mod network_status_tests {
    use super::*;

    const NOW: u64 = 1_780_082_164_826;

    /// The NGT-1 codec must produce the same synthetic PGN 262400
    /// record the iKonvert and SocketCAN drivers do — canboat C's
    /// `actisense-serial` emits it, and a consumer should see one
    /// uniform per-gateway status whichever gateway is feeding it.
    #[test]
    fn system_status_yields_a_network_status_record() {
        let mut d = Ngt1::default();
        let mut events = Vec::new();

        // `Actisense: System status` (ACTISENSE_BEM + 0xf2) as the
        // decoder hands it over: the subcommand byte is stripped, so
        // Error ID sits at data[7..10] and Ch1 Rx Load at data[13].
        let mut data = vec![0u8; 16];
        data[7..11].copy_from_slice(&1234u32.to_le_bytes());
        data[13] = 42; // 42 % bus load
        d.net.seen[3] = true;
        d.net.seen[9] = true;
        d.note_frame(
            RawFrame {
                timestamp: None,
                prio: 0,
                pgn: ACTISENSE_SYSTEM_STATUS_PGN,
                src: 0,
                dst: 0,
                data: data.into(),
            },
            NOW,
            &mut events,
        );

        let status = events
            .iter()
            .filter_map(|e| match e {
                Event::Frame(f) if f.pgn == 0x40100 => Some(f),
                _ => None,
            })
            .next()
            .expect("a network status frame");
        assert_eq!(status.data.len(), 15);
        assert_eq!(status.data[0], 42, "Ch1 Rx Load");
        assert_eq!(
            u32::from_le_bytes([
                status.data[1],
                status.data[2],
                status.data[3],
                status.data[4]
            ]),
            1234,
            "Error ID"
        );
        // src 3 and 9 seen by now; the gateway's own message is no device.
        assert_eq!(status.data[5], 2, "device count");
        // The NGT-1 knows neither of these.
        assert_eq!(status.data[10], 0xff, "gateway address sentinel");
        assert_eq!(&status.data[11..15], &[0xff; 4], "rejected TX sentinel");
        assert_eq!(
            status.timestamp.as_deref(),
            Some("2026-05-29T19:16:04.826Z")
        );
    }

    /// The status timer and the uptime run on the caller's clock.
    #[test]
    fn network_status_runs_on_the_callers_clock() {
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        d.tick(NOW, &mut events);
        assert!(events.is_empty(), "the first status is 5 s out");
        d.tick(NOW + 4_999, &mut events);
        assert!(events.is_empty());
        d.tick(NOW + 7_000, &mut events);
        let [Event::Frame(f)] = &events[..] else {
            panic!("expected one status frame, got {events:?}")
        };
        assert_eq!(f.pgn, 0x40100);
        assert_eq!(
            u32::from_le_bytes([f.data[6], f.data[7], f.data[8], f.data[9]]),
            7,
            "uptime in seconds"
        );
    }

    /// A received bus frame is stamped with the caller's time, not the
    /// NGT-1's ms-since-power-up clock that rides in the message.
    #[test]
    fn received_frames_carry_the_callers_time() {
        let frame = RawFrame::new(
            None,
            2,
            127250,
            52,
            255,
            [0xff, 0xac, 0xc9, 0xff, 0x7f, 0xff, 0x7f, 0xfd],
        );
        let wire =
            crate::engine::format::ngt1::encode_n2k_received_frame(&frame, 148).expect("fits");
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        d.receive(&wire, NOW, &mut events);
        let [Event::Frame(f)] = &events[..] else {
            panic!("expected one frame, got {events:?}")
        };
        assert_eq!(f.pgn, 127250);
        assert_eq!(f.timestamp.as_deref(), Some("2026-05-29T19:16:04.826Z"));
    }

    /// The gateway's own answers (`NGT_MSG_RECEIVED`) feed the transmit
    /// list sync, and come out as `Actisense: …` PGNs, never as bus frames.
    #[test]
    fn gateway_answers_are_actisense_pgns() {
        let mut d = Ngt1::new(Config {
            pgn_lists: PgnLists {
                tx: vec![127508],
                rx: vec![],
            },
            ..Default::default()
        });
        let mut payload = vec![0x11, 1, 0x0e, 0];
        payload.extend_from_slice(&[0; 8]);
        payload.extend_from_slice(&[2, 0]);
        let mut wire = Vec::new();
        crate::engine::format::ngt1::encode_ngt_message(NGT_MSG_RECEIVED, &payload, &mut wire);
        let mut events = Vec::new();
        d.receive(&wire, NOW, &mut events);
        let pgns: Vec<u32> = events
            .iter()
            .filter_map(|e| match e {
                Event::Frame(f) => Some(f.pgn),
                _ => None,
            })
            .collect();
        assert_eq!(pgns, [262161], "Actisense: Operating mode");
        assert!(!d.tx_list.is_done(), "startup confirmed, list read pending");
    }

    /// canboat's own PGNs alone (the wmm quirk) do not write the gateway's
    /// list, and a status says so.
    #[test]
    fn extra_pgns_alone_leave_the_list_alone() {
        let d = Ngt1::new(Config {
            extra_tx_pgns: vec![127258],
            ..Default::default()
        });
        assert!(d.tx_list.is_done(), "nothing to write");
        let status = pgn_list_status(&PgnLists::default(), &[127258]);
        assert_eq!(status.tx, PgnListSupport::Untouched);
        let status = pgn_list_status(
            &PgnLists {
                tx: vec![],
                rx: vec![129025],
            },
            &[],
        );
        assert_eq!(status.tx, PgnListSupport::Untouched);
        assert_eq!(status.rx, PgnListSupport::Unsupported);
    }

    /// With a named transmit list, only its PGNs, canboat's own and the
    /// network-management PGNs go out; with only canboat's own, nothing is
    /// refused.
    #[test]
    fn a_named_transmit_list_refuses_the_rest() {
        let frame = |pgn| RawFrame {
            timestamp: None,
            prio: 6,
            pgn,
            src: 0,
            dst: 255,
            data: vec![0; 8].into(),
        };
        let mut strict = Ngt1::new(Config {
            pgn_lists: PgnLists {
                tx: vec![127508],
                rx: vec![],
            },
            extra_tx_pgns: vec![127258],
            ..Default::default()
        });
        for pgn in [127508, 127258, 59904] {
            assert!(strict.send(&frame(pgn)).is_ok(), "{pgn}");
        }
        assert_eq!(strict.send(&frame(127506)), Err(Refused::NotOnTxList));
        assert_eq!(strict.send(&frame(0x40100)), Err(Refused::Synthetic));
        let mut long = frame(127508);
        long.data = std::iter::repeat_n(0, 250).collect();
        assert_eq!(strict.send(&long), Err(Refused::TooLarge));
        long.data.pop();
        assert!(strict.send(&long).is_ok(), "249 bytes of data fit");
        let mut open = Ngt1::new(Config {
            extra_tx_pgns: vec![127258],
            ..Default::default()
        });
        assert!(open.send(&frame(127506)).is_ok());
    }

    /// With no System Status ever seen — P-codes off — the two fields
    /// it carries must ride the sentinel rather than read as zero.
    #[test]
    fn without_system_status_load_and_errors_are_sentinel() {
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        d.emit_network_status(NOW, &mut events);
        let Event::Frame(f) = &events[0] else {
            panic!("expected a frame")
        };
        assert_eq!(f.data[0], 0xff, "load sentinel");
        assert_eq!(&f.data[1..5], &[0xff; 4], "errors sentinel");
    }
}

#[cfg(test)]
mod operating_mode_tests {
    use super::*;
    use crate::engine::format::ngt1::{NGT_MSG_SEND, encode_ngt_message};

    /// The gateway's answer to Operating Mode, reporting `mode`.
    fn answer(mode: u16, error: u32) -> Vec<u8> {
        let mut payload = vec![BEM_OPERATING_MODE, 0, 0x0e, 0x00];
        payload.extend_from_slice(&1234u32.to_le_bytes());
        payload.extend_from_slice(&error.to_le_bytes());
        payload.extend_from_slice(&mode.to_le_bytes());
        let mut wire = Vec::new();
        encode_ngt_message(NGT_MSG_RECEIVED, &payload, &mut wire);
        wire
    }

    #[test]
    fn open_sets_the_configured_mode_and_nothing_keeps_it_alive() {
        let mut d = Ngt1::default();
        let mut open = encode_get_product_info();
        open.extend(encode_set_operating_mode(OperatingMode::NgTransferRxAll));
        assert_eq!(d.open(), open);

        let mut d = Ngt1::new(Config {
            operating_mode: OperatingMode::NgTransferNormal,
            ..Config::default()
        });
        let mut wire = Vec::new();
        encode_ngt_message(NGT_MSG_SEND, &[BEM_OPERATING_MODE, 1, 0], &mut wire);
        assert!(d.open().ends_with(&wire));
        assert_eq!(d.keepalive(), None);
    }

    #[test]
    fn the_reported_mode_is_noted_and_never_a_frame() {
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        d.receive(&answer(2, 0), NOW, &mut events);
        assert_eq!(d.reported_mode, Some(OperatingMode::NgTransferRxAll));
        d.receive(&answer(1, 0x8000_0001), NOW, &mut events);
        assert_eq!(d.reported_mode, Some(OperatingMode::NgTransferNormal));
        assert!(
            !events
                .iter()
                .any(|e| matches!(e, Event::Frame(f) if f.pgn < SYNTHETIC_PGN_START)),
            "{events:?}"
        );
    }

    /// A BEM message from the gateway: `bem`, part `sequence`, `data`.
    fn bem(bem: u8, sequence: u8, error: i32, data: &[u8]) -> Vec<u8> {
        let mut payload = vec![bem, sequence, 0x0e, 0x00];
        payload.extend_from_slice(&1234u32.to_le_bytes());
        payload.extend_from_slice(&error.to_le_bytes());
        payload.extend_from_slice(data);
        let mut wire = Vec::new();
        encode_ngt_message(NGT_MSG_RECEIVED, &payload, &mut wire);
        wire
    }

    fn padded(text: &str) -> Vec<u8> {
        let mut v = text.as_bytes().to_vec();
        v.resize(32, 0xff);
        v
    }

    /// Before, every BEM message was dropped, so a live NGT-1's System
    /// Status never reached the network status.
    #[test]
    fn a_live_system_status_reaches_the_network_status() {
        let mut d = Ngt1::default();
        let mut data = vec![0u8; 4];
        data[2] = 37; // Ch1 Rx Load, after channel count and bandwidth
        let mut events = Vec::new();
        d.receive(&bem(BEM_SYSTEM_STATUS, 0, 0, &data), NOW, &mut events);
        assert_eq!(d.net.load_pct, Some(37));
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Frame(f) if f.pgn == 262386))
        );
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Frame(f) if f.pgn == 0x40100))
        );
    }

    #[test]
    fn product_info_in_five_parts() {
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        let mut main = 2100u16.to_le_bytes().to_vec();
        main.extend_from_slice(&1234u16.to_le_bytes());
        main.extend_from_slice(&[0, 1]);
        d.receive(&bem(BEM_PRODUCT_INFO, 1, 0, &main), NOW, &mut events);
        for (part, text) in [(2, "NGT-1"), (3, "2.190"), (4, "Rev B"), (5, "110763")] {
            d.receive(
                &bem(BEM_PRODUCT_INFO, part, 0, &padded(text)),
                NOW,
                &mut events,
            );
        }
        assert_eq!(d.product_info.model, "NGT-1");
        assert_eq!(d.product_info.software_version, "2.190");
        assert_eq!(d.product_info.serial_number, "110763");
        assert_eq!(d.product_info.nmea2000_version, 2100);
        let mut probe = d.product_info.clone();
        let header = [BEM_PRODUCT_INFO, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let mut payload = header.to_vec();
        payload.extend(padded("x"));
        assert!(
            probe.add(&BemResponse::parse(&payload).unwrap()),
            "complete"
        );
    }

    #[test]
    fn product_info_in_one_message() {
        let mut data = 17u32.to_le_bytes().to_vec();
        data.extend_from_slice(&2100u16.to_le_bytes());
        data.extend_from_slice(&1234u16.to_le_bytes());
        for text in ["W2K-1", "1.010", "B", "123456"] {
            data.extend(padded(text));
        }
        data.extend_from_slice(&[0, 2]);
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        d.receive(&bem(BEM_PRODUCT_INFO, 6, 0, &data), NOW, &mut events);
        assert_eq!(d.product_info.model, "W2K-1");
        assert_eq!(d.product_info.hardware_version, "B");
        assert_eq!(d.product_info.product_code, 1234);
    }

    /// A gateway that restarted is set up again: the operating mode, the
    /// product information and the transmit list.
    #[test]
    fn a_startup_status_sets_the_gateway_up_again() {
        let mut d = Ngt1::new(Config {
            pgn_lists: PgnLists {
                tx: vec![127508],
                rx: vec![],
            },
            ..Default::default()
        });
        d.tx_list.on_message(&[BEM_OPERATING_MODE, 1], NOW);
        let mut events = Vec::new();
        d.receive(
            &bem(BEM_STARTUP_STATUS, 0, 0, &[0x8e, 0x08, 0]),
            NOW,
            &mut events,
        );
        let sent: Vec<&Vec<u8>> = events
            .iter()
            .filter_map(|e| match e {
                Event::Send(b) => Some(b),
                _ => None,
            })
            .collect();
        assert_eq!(
            sent,
            [
                &encode_get_product_info(),
                &encode_set_operating_mode(OperatingMode::NgTransferRxAll)
            ]
        );
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Frame(f) if f.pgn == 262384))
        );
        assert!(!d.tx_list.is_done());
    }

    #[test]
    fn errors_have_the_sdk_names() {
        assert_eq!(describe_error(-1158), "-1158 (command timeout)");
        assert_eq!(describe_error(-697), "-697");
        let mut d = Ngt1::default();
        let mut events = Vec::new();
        d.receive(
            &bem(BEM_NEGATIVE_ACK, 0, -1158, &[1, 0, 0, 0]),
            NOW,
            &mut events,
        );
        d.receive(
            &bem(BEM_ERROR_REPORT, 0, -1497, &[4, 1, 0, 0, 0]),
            NOW,
            &mut events,
        );
        let pgns: Vec<u32> = events
            .iter()
            .filter_map(|e| match e {
                Event::Frame(f) => Some(f.pgn),
                _ => None,
            })
            .collect();
        assert_eq!(pgns, [262388, 262385]);
    }

    const NOW: u64 = 1_780_082_164_826;
}
