// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense NGT-1 gateway codec.
//!
//! Wraps [`crate::engine::format::ngt1`]: received bytes go through
//! `Ngt1Decoder`, frames for the bus go out as `N2K_MSG_SEND` (0x94)
//! messages, and a 20-second keepalive resends the NGT-1 startup ping while
//! the link is quiet. Synthetic PGNs (`>= 0x40000`) are refused, matching
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
use crate::engine::format::{
    encode_n2k_send_frame, encode_startup_ping,
    ngt1::{Ngt1Decoder, NgtEvent},
};
use crate::engine::pgn_list::{self, PgnListStatus, PgnListSupport, PgnLists, TxGate};

use super::ngt1_tx_list::{NGT_MSG_RECEIVED, TxListRecord, TxListSync};
use super::{Codec, Event, Refused, SYNTHETIC_PGN_START};
use crate::engine::format_iso_ms;

/// Re-ping the NGT-1 startup sequence every 20 s — matches the C
/// `actisense-serial` keepalive.
pub const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(20);

/// `Actisense: System status` — `ACTISENSE_BEM + 0xf2`. The NGT-1 sends
/// it about once a second, but only when P-codes are enabled for the
/// port, so its two fields may never arrive.
const ACTISENSE_SYSTEM_STATUS_PGN: u32 = SYNTHETIC_PGN_START + 0xf2;

/// How often to emit the synthetic gateway network status.
const NETWORK_STATUS_INTERVAL_MS: u64 = 5_000;

/// NGT-1 settings. `Config::default()` leaves the gateway's lists alone.
#[derive(Debug, Clone, Default)]
pub struct Config {
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
        self.net.seen[frame.src as usize] = true;
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
    fn open(&mut self) -> Vec<u8> {
        encode_startup_ping()
    }

    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        self.tick(now_ms, events);
        for ev in self.inner.push_bytes(bytes) {
            match ev {
                NgtEvent::Message(msg) if msg.command == NGT_MSG_RECEIVED => {
                    send_all(events, self.tx_list.on_message(&msg.payload, now_ms));
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
                // EBL header records (timestamp etc.) only appear when
                // the decoder is in EBL mode, which a live link never
                // enables.
                NgtEvent::Header(_) => {}
            }
        }
    }

    /// The codec's deadlines, run on every receive too, so they advance
    /// on a quiet bus.
    fn tick(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        self.started(now_ms);
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
        Ok(encode_n2k_send_frame(frame))
    }

    fn keepalive(&self) -> Option<(Duration, Vec<u8>)> {
        Some((KEEPALIVE_INTERVAL, encode_startup_ping()))
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
        // src 0, 3 and 9 seen by now.
        assert_eq!(status.data[5], 3, "device count");
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
    /// list sync and never surface as bus frames.
    #[test]
    fn gateway_answers_are_not_frames() {
        let mut d = Ngt1::new(Config {
            pgn_lists: PgnLists {
                tx: vec![127508],
                rx: vec![],
            },
            ..Default::default()
        });
        let mut wire = Vec::new();
        crate::engine::format::ngt1::encode_ngt_message(NGT_MSG_RECEIVED, &[0x11, 1], &mut wire);
        let mut events = Vec::new();
        d.receive(&wire, NOW, &mut events);
        assert!(
            !events
                .iter()
                .any(|e| matches!(e, Event::Frame(f) if f.pgn != 0x40100)),
            "{events:?}"
        );
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
