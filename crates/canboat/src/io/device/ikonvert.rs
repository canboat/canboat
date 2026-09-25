// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Digital Yacht iKonvert codec adapter for [`super::run`].
//!
//! Wraps [`crate::engine::format::ikonvert`]. iKonvert is line-based
//! ASCII (`$PDGY,...` control sentences and `!PDGY,...` frame
//! sentences). The decoder buffers a partial line across reads, so
//! the partial-frame state lives in [`Decoder`].
//!
//! Init handshake — mirrors `canboat/ikonvert-serial/ikonvert-serial.c`
//! exactly, because the iKonvert ignores commands that arrive while
//! it's still processing the previous one. The boot sequence is
//! **ACK-driven**: every command is followed by a `$PDGY,ACK,...`
//! confirmation, and the next command may only be written once the
//! previous one is acknowledged. Anything blasted as a single chunk
//! puts the device in a state where it stops emitting frames.
//!
//! Step-by-step (per canboat C `sendNextInitCommand`):
//!
//!   1. `$PDGY,N2NET_OFFLINE`         → wait ACK (returned as TEXT banner)
//!   2. `$PDGY,N2NET_RESET`           → wait ACK   (unconditional — wipes
//!      any RX/TX filter stored in the device's NVRAM)
//!   3. `$PDGY,RX_LIST,<pgns>`        → wait ACK   (only if rx list set)
//!   4. `$PDGY,TX_LIST,<pgns>`        → wait ACK   (only if tx list set)
//!   5. `$PDGY,N2NET_INIT,{ALL|NORMAL}` → wait ACK
//!   6. `$PDGY,TX_LIMIT,OFF`          (no ACK)     (only if `rate_limit_off`)
//!
//! [`Encoder::init_bytes`] writes only step 1; the rest flow back
//! via [`super::DeviceEvent::SendBytes`] each time the decoder sees
//! an ACK.
//!
//! The two lists do more than filter. Per Digital Yacht's *iKonvert
//! Gateway Developer's Guide* (V1.04, §3.2, §4.5–4.8) the gateway stores
//! them in non-volatile memory, answers other devices' PGN 126464
//! requests with them, and refuses to transmit a PGN missing from the TX
//! list (`$PDGY,NAK,PGN_NOT_IN_TX_LIST`). Only `N2NET_INIT,NORMAL` makes
//! the RX list a receive filter; in `ALL` mode everything is passed on.
//! So [`Config::pgn_lists`] goes into both lists but keeps `ALL` mode,
//! and only the legacy [`Config::rx_list`] asks for `NORMAL`.

use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::engine::RawFrame;
use crate::engine::format::ikonvert::{
    self, IkonvertLine, TX_LIMIT_OFF, TX_OFFLINE, TX_ONLINE_ALL, TX_ONLINE_NORMAL,
    synthesize_network_status,
};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};
use crate::io::pgn_list::{self, PgnListStatus, PgnListSupport, PgnLists, TxGate};

/// Synthetic-PGN marker. iKonvert silently drops `>= 0x40000` PGNs
/// the same way actisense-serial does.
pub const IKONVERT_SYNTHETIC_PGN: u32 = 0x40000;

/// iKonvert initialisation parameters. All fields are optional —
/// `Config::default()` brings the bus online in `ALL` mode with no
/// rate-limit override.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// Comma-separated PGN filter for receive. If set, init enters
    /// `NORMAL` mode instead of `ALL`, so the gateway passes on only
    /// these PGNs (and [`Self::pgn_lists`]'s receive PGNs).
    /// `N2NET_RESET` is always sent during init regardless of this
    /// field — see the module-level handshake docs.
    pub rx_list: Option<String>,
    /// Comma-separated PGN filter for transmit. Triggers the
    /// `TX_LIST` step (the `N2NET_RESET` step always runs).
    pub tx_list: Option<String>,
    /// PGNs the application sends and reads, written into the gateway's
    /// TX and RX lists (after the two strings above) so it advertises
    /// them in PGN 126464 and agrees to transmit them. Receive PGNs here
    /// do not turn on `NORMAL` mode.
    ///
    /// **⚠️ ONCE A TRANSMIT LIST IS NAMED — HERE OR IN `tx_list` — ANY PGN
    /// NOT ON IT IS REFUSED: THE DRIVER DOES NOT SEND IT.** The gateway
    /// would reject it anyway (`NAK,PGN_NOT_IN_TX_LIST`), and its list
    /// cannot change while it is on the bus. Name every PGN you will send
    /// before opening the device. Network-management PGNs are always sent.
    pub pgn_lists: PgnLists,
    /// PGNs canboat itself transmits through the gateway (a quirk's, such
    /// as `wmm`'s 127258). Added to the TX list and allowed; unlike
    /// `pgn_lists.tx` they never on their own make the driver refuse
    /// other PGNs.
    pub extra_tx_pgns: Vec<u32>,
    /// Disable the iKonvert TX rate limit. Off by default.
    pub rate_limit_off: bool,
    /// Skip the init handshake entirely. Useful when the "writer
    /// side" is `/dev/null` (replay-from-file mode), since there's
    /// no device on the other end to ACK the commands.
    pub skip_init: bool,
}

/// How the gateway takes `lists`: it stores each non-empty list in its own
/// memory and answers PGN 126464 from it ([`PgnListSupport::Pushed`]); an
/// empty one is not written ([`PgnListSupport::Untouched`]). `dropped`
/// names the PGNs that do not fit.
pub fn pgn_list_status(lists: &PgnLists) -> PgnListStatus {
    let (_, mut dropped) = pgn_list::merge(&[], &lists.tx);
    for pgn in pgn_list::merge(&[], &lists.rx).1 {
        if !dropped.contains(&pgn) {
            dropped.push(pgn);
        }
    }
    let written = |list: &[u32]| {
        if list.is_empty() {
            PgnListSupport::Untouched
        } else {
            PgnListSupport::Pushed
        }
    };
    PgnListStatus {
        tx: written(&lists.tx),
        rx: written(&lists.rx),
        dropped,
    }
}

/// The init handshake's settings, resolved once from a [`Config`].
struct Init {
    rx_list: Option<String>,
    tx_list: Option<String>,
    /// `N2NET_INIT,NORMAL` (the RX list filters) rather than `ALL`.
    normal_mode: bool,
    rate_limit_off: bool,
}

impl Init {
    fn new(config: &Config) -> Self {
        Self {
            rx_list: list_sentence(config.rx_list.as_deref(), &config.pgn_lists.rx, "RX"),
            tx_list: list_sentence(
                config.tx_list.as_deref(),
                &[config.pgn_lists.tx.as_slice(), &config.extra_tx_pgns].concat(),
                "TX",
            ),
            normal_mode: config.rx_list.is_some(),
            rate_limit_off: config.rate_limit_off,
        }
    }
}

/// The PGNs for one `$PDGY,…_LIST` sentence: the legacy comma-separated
/// `list` first, then `extra`'s not already in it; `None` when both are
/// empty.
fn list_sentence(list: Option<&str>, extra: &[u32], which: &str) -> Option<String> {
    let mut pgns: Vec<u32> = Vec::new();
    for item in list.unwrap_or("").split(',').map(str::trim) {
        match item.parse() {
            Ok(pgn) => pgns.push(pgn),
            Err(_) if item.is_empty() => {}
            Err(_) => log::warn!("ikonvert: ignoring {which} list entry {item:?}, not a PGN"),
        }
    }
    pgns.extend_from_slice(extra);
    let (pgns, dropped) = pgn_list::merge(&[], &pgns);
    if !dropped.is_empty() {
        log::warn!("ikonvert: leaving {which} PGNs {dropped:?} out (invalid, or the list is full)");
    }
    if pgns.is_empty() {
        return None;
    }
    Some(
        pgns.iter()
            .map(u32::to_string)
            .collect::<Vec<_>>()
            .join(","),
    )
}

/// Start the iKonvert reader/writer threads.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    let encoder = Encoder {
        skip_init: config.skip_init,
        refusal: tx_gate(&config),
    };
    super::run(Decoder::new(config), encoder, reader, writer)
}

/// Decoder buffers a partial line across reads and owns the
/// ACK-driven init state machine.
pub struct Decoder {
    acc: String,
    init: Init,
    /// Sequence counter that mirrors C's `sendInitState`. Even values
    /// are "ready to send the next command"; odd values are "waiting
    /// for the ACK of the command we just sent". `0` means init is
    /// complete.
    init_state: Arc<Mutex<u32>>,
}

impl Decoder {
    pub fn new(config: Config) -> Self {
        let start = if config.skip_init {
            STATE_DONE
        } else {
            // After `init_bytes()` sends `N2NET_OFFLINE`, the writer
            // thread is waiting for that ACK — start at the
            // corresponding odd state.
            STATE_WAIT_OFFLINE_ACK
        };
        Self {
            acc: String::with_capacity(1024),
            init: Init::new(&config),
            init_state: Arc::new(Mutex::new(start)),
        }
    }
}

impl DeviceDecoder for Decoder {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        self.acc.push_str(&String::from_utf8_lossy(bytes));
        while let Some(eol) = self.acc.find('\n') {
            let line: String = self.acc.drain(..=eol).collect();
            let trimmed = line.trim_end_matches(['\r', '\n']);
            if trimmed.is_empty() {
                continue;
            }
            match ikonvert::parse_line(trimmed) {
                Ok(IkonvertLine::Frame(mut f)) => {
                    // canboat C overrides the iKonvert's
                    // seconds-since-boot timestamp with host time
                    // (see `computeIKonvertTime`). The device clock
                    // is known to drift, and downstream tools (the
                    // analyzer, n2kd) expect a wall-clock timestamp.
                    f.timestamp = Some(now_iso_ms());
                    events.push(DeviceEvent::Frame(f));
                }
                Ok(IkonvertLine::Control(c)) => self.handle_control(&c, events),
                Ok(IkonvertLine::Other) => {}
                Err(e) => {
                    // When the iKonvert reboots mid-stream (e.g. on
                    // a bus condition) it can splice its boot banner
                    // straight onto the tail of an unterminated frame
                    // line — no `\r\n` separator. The line then fails
                    // to parse as a frame. Recover the TEXT signal
                    // out of the rubble; it's the only signal we have
                    // that the device just reset.
                    if let Some(idx) = trimmed.find("$PDGY,TEXT,") {
                        let banner = &trimmed[idx + "$PDGY,".len()..];
                        log::warn!("ikonvert: corrupt line, but found TEXT banner");
                        self.handle_control(banner, events);
                    } else {
                        events.push(DeviceEvent::Error(format!("ikonvert: {e}")));
                    }
                }
            }
        }
    }
}

impl Decoder {
    fn handle_control(&self, body: &str, events: &mut Vec<DeviceEvent>) {
        // `body` is everything after the `$PDGY,` prefix. The C tool
        // dispatches on the head before the first comma — so do we.
        //
        // Quirk: the iKonvert does *not* acknowledge `N2NET_OFFLINE`
        // with an `ACK,…` line. Instead it emits its device-info
        // banner (`TEXT,Digital_Yacht_iKonvert_v2_…`). All later
        // commands (RESET / RX_LIST / TX_LIST / INIT) are followed by
        // a normal `ACK,…`. canboat C handles this by matching TEXT
        // explicitly at state 13 and matching ACK at the other odd
        // states.
        if let Some(rest) = body.strip_prefix("TEXT,") {
            log::info!("ikonvert: connected to {rest}");
            self.advance_after_text(events);
            return;
        }
        if let Some(rest) = body.strip_prefix("ACK,") {
            log::debug!("ikonvert: ACK ({rest})");
            self.advance_after_ack(events);
            return;
        }
        if let Some(rest) = body.strip_prefix("NAK,") {
            log::warn!("ikonvert: NAK ({rest})");
            // Stay at the current state — canboat C does the same;
            // a typical NAK is "already offline" which is harmless.
            return;
        }
        // `$PDGY,000000,…` — network-status heartbeat. The populated
        // form is surfaced as a synthesized IKONVERT_BEM frame (PGN
        // 262400); the empty keep-alive form (`,,,,,,`) is logged
        // only. Either way the heartbeat still drives the during-init
        // resend, mirroring C's unconditional `sendNextInitCommand`
        // call after each handled ASCII line.
        if body.starts_with("000000,") {
            match synthesize_network_status(body, now_iso_ms()) {
                Some(frame) => {
                    log::debug!(
                        "ikonvert: synthesized network status (load={} count={})",
                        frame.data[0],
                        frame.data[5],
                    );
                    events.push(DeviceEvent::Frame(frame));
                }
                None => log::debug!("ikonvert: keep-alive heartbeat"),
            }
            self.maybe_resend_offline(events);
            return;
        }
        // Any other `$PDGY,…` line that arrives while we're waiting
        // for OFFLINE's TEXT reply means the device missed the
        // command (or replied earlier than we started listening). C
        // canboat resends; do the same.
        log::debug!("ikonvert control: {body}");
        self.maybe_resend_offline(events);
    }

    /// Handle a `$PDGY,TEXT,…` banner.
    ///
    /// * While we're waiting for the OFFLINE response (state 13),
    ///   advance into the rest of the handshake.
    /// * After init is finished, TEXT means the device just rebooted
    ///   itself (bus condition, watchdog, whatever) and is back in
    ///   the pre-init OFFLINE state. Restart the handshake from
    ///   `N2NET_RESET` — no need to re-send OFFLINE, the device is
    ///   already there.
    /// * Mid-handshake TEXT is unusual; treat it as a soft reset too.
    fn advance_after_text(&self, events: &mut Vec<DeviceEvent>) {
        let mut state = self.init_state.lock().expect("ikonvert state poisoned");
        match *state {
            STATE_WAIT_OFFLINE_ACK => {
                *state = STATE_SEND_RESET;
                if let Some(cmd) = next_init_command(&mut state, &self.init) {
                    events.push(DeviceEvent::SendBytes(cmd.into_bytes()));
                } else {
                    log::info!("ikonvert: initialization complete");
                }
            }
            STATE_DONE => {
                log::warn!("ikonvert: device reset detected, re-initialising");
                // Skip OFFLINE since the device is already offline
                // post-reboot; resume with the unconditional RESET.
                *state = STATE_SEND_RESET;
                if let Some(cmd) = next_init_command(&mut state, &self.init) {
                    events.push(DeviceEvent::SendBytes(cmd.into_bytes()));
                }
            }
            _ => {
                log::debug!("ikonvert: TEXT during init at state {}", *state);
            }
        }
    }

    fn advance_after_ack(&self, events: &mut Vec<DeviceEvent>) {
        let mut state = self.init_state.lock().expect("ikonvert state poisoned");
        // ACK only advances at odd states *other* than the OFFLINE
        // wait — that one needs TEXT.
        if *state == STATE_DONE || *state == STATE_WAIT_OFFLINE_ACK || (*state).is_multiple_of(2) {
            return;
        }
        *state -= 1;
        if let Some(cmd) = next_init_command(&mut state, &self.init) {
            events.push(DeviceEvent::SendBytes(cmd.into_bytes()));
        } else {
            log::info!("ikonvert: initialization complete");
        }
    }

    /// If we're at `STATE_WAIT_OFFLINE_ACK` and a non-TEXT line just
    /// arrived, the device may have missed our OFFLINE command. Push
    /// it again so we don't sit waiting on the heartbeat stream
    /// forever. Mirrors canboat C's `sendInitState == 13 → ++` logic.
    fn maybe_resend_offline(&self, events: &mut Vec<DeviceEvent>) {
        let state = self.init_state.lock().expect("ikonvert state poisoned");
        if *state != STATE_WAIT_OFFLINE_ACK {
            return;
        }
        drop(state);
        log::debug!("ikonvert: resending N2NET_OFFLINE");
        events.push(DeviceEvent::SendBytes(TX_OFFLINE.as_bytes().to_vec()));
    }
}

pub struct Encoder {
    skip_init: bool,
    refusal: Option<TxGate>,
}

/// The refusal of unlisted PGNs, when the client named a transmit list;
/// see [`Config::pgn_lists`]. None with the handshake skipped: nothing
/// sets the gateway's list then.
fn tx_gate(config: &Config) -> Option<TxGate> {
    if config.skip_init {
        return None;
    }
    let mut named = pgns_in(config.tx_list.as_deref());
    named.extend_from_slice(&config.pgn_lists.tx);
    TxGate::new("ikonvert", &named, &config.extra_tx_pgns)
}

/// The PGNs in a legacy comma-separated list.
fn pgns_in(list: Option<&str>) -> Vec<u32> {
    list.unwrap_or("")
        .split(',')
        .filter_map(|p| p.trim().parse().ok())
        .collect()
}

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        if self.skip_init {
            return Vec::new();
        }
        // Only the first command goes out unsolicited. Everything
        // else is gated on the corresponding ACK arriving back from
        // the device.
        log::info!("ikonvert: initialization start");
        TX_OFFLINE.as_bytes().to_vec()
    }

    /// The gateway stays on the bus after the application goes away —
    /// still answering address-claim, product-information and PGN-list
    /// requests with nothing driving it. `N2NET_OFFLINE` takes it off (it
    /// reboots, and waits to be initialised again).
    fn shutdown_bytes(&self) -> Vec<u8> {
        if self.skip_init {
            return Vec::new();
        }
        log::info!("ikonvert: send N2NET_OFFLINE");
        TX_OFFLINE.as_bytes().to_vec()
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.pgn >= IKONVERT_SYNTHETIC_PGN {
            log::debug!("ikonvert: skipping synthetic PGN {}", frame.pgn);
            return None;
        }
        if let Some(gate) = &self.refusal
            && !gate.allows(frame.pgn)
        {
            return None;
        }
        Some(ikonvert::encode_tx_frame(frame).into_bytes())
    }
}

// State values mirror C's `sendInitState`. Even = "ready to evaluate
// case N and possibly emit". Odd = "waiting for the ACK that follows
// the command emitted at state N+1".
const STATE_DONE: u32 = 0;
const STATE_MAYBE_LIMIT_OFF: u32 = 2;
const STATE_WAIT_INIT_ACK: u32 = 3;
const STATE_SEND_INIT: u32 = 4;
const STATE_MAYBE_SHOWLISTS: u32 = 6;
const STATE_MAYBE_TX_LIST: u32 = 8;
const STATE_WAIT_TX_LIST_ACK: u32 = 7;
const STATE_MAYBE_RX_LIST: u32 = 10;
const STATE_WAIT_RX_LIST_ACK: u32 = 9;
const STATE_SEND_RESET: u32 = 12;
const STATE_WAIT_RESET_ACK: u32 = 11;
const STATE_WAIT_OFFLINE_ACK: u32 = 13;

/// Drive the state machine forward starting from an *even* state.
/// Returns the bytes of whatever command should go on the wire next,
/// or `None` when init is complete.
fn next_init_command(state: &mut u32, init: &Init) -> Option<String> {
    loop {
        match *state {
            STATE_SEND_RESET => {
                // Always issue N2NET_RESET so any RX/TX filter the
                // device may have stored in NVRAM from a previous
                // session is wiped before we (optionally) set our own.
                *state = STATE_WAIT_RESET_ACK;
                log::info!("ikonvert: send N2NET_RESET");
                return Some("$PDGY,N2NET_RESET\r\n".to_string());
            }
            STATE_MAYBE_RX_LIST => {
                if let Some(list) = &init.rx_list {
                    *state = STATE_WAIT_RX_LIST_ACK;
                    log::info!("ikonvert: send RX_LIST {list}");
                    return Some(format!("$PDGY,RX_LIST,{list}\r\n"));
                }
                *state = STATE_MAYBE_TX_LIST;
            }
            STATE_MAYBE_TX_LIST => {
                if let Some(list) = &init.tx_list {
                    *state = STATE_WAIT_TX_LIST_ACK;
                    log::info!("ikonvert: send TX_LIST {list}");
                    return Some(format!("$PDGY,TX_LIST,{list}\r\n"));
                }
                *state = STATE_MAYBE_SHOWLISTS;
            }
            STATE_MAYBE_SHOWLISTS => {
                // C only emits SHOW_LISTS in verbose / log-debug mode;
                // we skip it unconditionally — no functional effect.
                *state = STATE_SEND_INIT;
            }
            STATE_SEND_INIT => {
                *state = STATE_WAIT_INIT_ACK;
                let cmd = if init.normal_mode {
                    TX_ONLINE_NORMAL
                } else {
                    TX_ONLINE_ALL
                };
                log::info!("ikonvert: send N2NET_INIT");
                return Some(cmd.to_string());
            }
            STATE_MAYBE_LIMIT_OFF => {
                // TX_LIMIT,OFF has no ACK; we move straight to DONE.
                let cmd = if init.rate_limit_off {
                    log::info!("ikonvert: send TX_LIMIT,OFF");
                    Some(TX_LIMIT_OFF.to_string())
                } else {
                    None
                };
                *state = STATE_DONE;
                return cmd;
            }
            STATE_DONE => return None,
            // Odd states: caller drove us into "waiting for ACK"
            // territory without an ACK arriving — bail.
            _ => return None,
        }
    }
}

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` from the host clock in UTC, the shape
/// every canboat gateway driver stamps and canboat C prints.
fn now_iso_ms() -> String {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    crate::engine::format_iso_ms(dur.as_millis() as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Walk the state machine the way an iKonvert would: send the
    /// first command via `init_bytes`, then feed the first response
    /// as TEXT (the OFFLINE-completion banner), and every subsequent
    /// as ACK. Collects the bytes the codec wants written back.
    fn run_init(config: Config) -> Vec<String> {
        let encoder = Encoder {
            skip_init: false,
            refusal: None,
        };
        let first = String::from_utf8(encoder.init_bytes()).unwrap();
        let mut commands = vec![first];

        let decoder = Decoder::new(config);
        let mut first_response = true;
        loop {
            let mut events = Vec::new();
            if first_response {
                decoder.handle_control("TEXT,iKonvert_simulated", &mut events);
                first_response = false;
            } else {
                decoder.handle_control("ACK,test", &mut events);
            }
            let mut emitted_one = false;
            for ev in events {
                if let DeviceEvent::SendBytes(b) = ev {
                    commands.push(String::from_utf8(b).unwrap());
                    emitted_one = true;
                }
            }
            if !emitted_one {
                break;
            }
        }
        commands
    }

    #[test]
    fn default_init_sends_offline_reset_init_all() {
        let cmds = run_init(Config::default());
        assert_eq!(
            cmds,
            vec![
                "$PDGY,N2NET_OFFLINE\r\n".to_string(),
                "$PDGY,N2NET_RESET\r\n".to_string(),
                "$PDGY,N2NET_INIT,ALL\r\n".to_string(),
            ],
            "default init should be OFFLINE → RESET → INIT,ALL"
        );
    }

    #[test]
    fn rx_list_triggers_reset_rxlist_normal() {
        let cfg = Config {
            rx_list: Some("129025,129026".to_string()),
            ..Config::default()
        };
        let cmds = run_init(cfg);
        assert_eq!(
            cmds,
            vec![
                "$PDGY,N2NET_OFFLINE\r\n".to_string(),
                "$PDGY,N2NET_RESET\r\n".to_string(),
                "$PDGY,RX_LIST,129025,129026\r\n".to_string(),
                "$PDGY,N2NET_INIT,NORMAL\r\n".to_string(),
            ]
        );
    }

    /// The application's PGNs fill both lists but leave the gateway in
    /// `ALL` mode: advertising what we read must not filter it.
    #[test]
    fn pgn_lists_fill_both_lists_without_filtering() {
        let cfg = Config {
            pgn_lists: PgnLists {
                tx: vec![127508, 127506],
                rx: vec![129025],
            },
            ..Config::default()
        };
        assert_eq!(
            run_init(cfg),
            vec![
                "$PDGY,N2NET_OFFLINE\r\n".to_string(),
                "$PDGY,N2NET_RESET\r\n".to_string(),
                "$PDGY,RX_LIST,129025\r\n".to_string(),
                "$PDGY,TX_LIST,127508,127506\r\n".to_string(),
                "$PDGY,N2NET_INIT,ALL\r\n".to_string(),
            ]
        );
    }

    /// With the legacy filter strings too, the application's PGNs follow
    /// them without duplicates, and the RX filter keeps `NORMAL` mode.
    #[test]
    fn pgn_lists_extend_the_legacy_filters() {
        let cfg = Config {
            rx_list: Some("129025, 129026".to_string()),
            tx_list: Some("127508".to_string()),
            pgn_lists: PgnLists {
                tx: vec![127508, 127506, 0x40000],
                rx: vec![129026, 130306],
            },
            ..Config::default()
        };
        assert_eq!(
            run_init(cfg),
            vec![
                "$PDGY,N2NET_OFFLINE\r\n".to_string(),
                "$PDGY,N2NET_RESET\r\n".to_string(),
                "$PDGY,RX_LIST,129025,129026,130306\r\n".to_string(),
                "$PDGY,TX_LIST,127508,127506\r\n".to_string(),
                "$PDGY,N2NET_INIT,NORMAL\r\n".to_string(),
            ]
        );
    }

    fn frame(pgn: u32) -> RawFrame {
        RawFrame {
            timestamp: None,
            prio: 6,
            pgn,
            src: 0,
            dst: 255,
            data: vec![0; 8].into(),
        }
    }

    fn encoder_for(config: &Config) -> Encoder {
        Encoder {
            skip_init: config.skip_init,
            refusal: tx_gate(config),
        }
    }

    /// With a named transmit list, only its PGNs, canboat's own and the
    /// network-management PGNs go out; the rest is refused.
    #[test]
    fn a_named_transmit_list_refuses_the_rest() {
        let encoder = encoder_for(&Config {
            tx_list: Some("130306".to_string()),
            pgn_lists: PgnLists {
                tx: vec![127508],
                rx: vec![],
            },
            extra_tx_pgns: vec![127258],
            ..Config::default()
        });
        for pgn in [127508, 130306, 127258, 59904, 126996] {
            assert!(encoder.encode_frame(&frame(pgn)).is_some(), "{pgn} allowed");
        }
        assert!(encoder.encode_frame(&frame(127506)).is_none(), "not listed");
    }

    /// Nothing named (a quirk's PGN alone does not count), or a replay
    /// with no handshake: nothing is refused.
    #[test]
    fn without_a_named_list_nothing_is_refused() {
        for config in [
            Config {
                extra_tx_pgns: vec![127258],
                ..Config::default()
            },
            Config {
                pgn_lists: PgnLists {
                    tx: vec![127508],
                    rx: vec![],
                },
                skip_init: true,
                ..Config::default()
            },
        ] {
            assert!(encoder_for(&config).encode_frame(&frame(127506)).is_some());
        }
    }

    /// canboat's own PGNs join the TX list without being named.
    #[test]
    fn extra_pgns_join_the_tx_list() {
        let cmds = run_init(Config {
            pgn_lists: PgnLists {
                tx: vec![127508],
                rx: vec![],
            },
            extra_tx_pgns: vec![127258],
            ..Config::default()
        });
        assert!(
            cmds.contains(&"$PDGY,TX_LIST,127508,127258\r\n".to_string()),
            "{cmds:?}"
        );
    }

    #[test]
    fn the_gateway_holds_both_lists() {
        let status = pgn_list_status(&PgnLists {
            tx: vec![127508, 0x40000],
            rx: vec![],
        });
        assert_eq!(status.tx, PgnListSupport::Pushed);
        assert_eq!(status.rx, PgnListSupport::Untouched, "nothing named");
        assert_eq!(status.dropped, [0x40000]);
    }

    #[test]
    fn rate_limit_off_sends_after_init_with_no_ack() {
        let cfg = Config {
            rate_limit_off: true,
            ..Config::default()
        };
        let cmds = run_init(cfg);
        assert_eq!(
            cmds,
            vec![
                "$PDGY,N2NET_OFFLINE\r\n".to_string(),
                "$PDGY,N2NET_RESET\r\n".to_string(),
                "$PDGY,N2NET_INIT,ALL\r\n".to_string(),
                "$PDGY,TX_LIMIT,OFF\r\n".to_string(),
            ]
        );
    }

    #[test]
    fn extra_ack_after_done_is_ignored() {
        let decoder = Decoder::new(Config::default());
        let mut events = Vec::new();
        decoder.handle_control("TEXT,welcome", &mut events); // → RESET
        events.clear();
        decoder.handle_control("ACK,reset", &mut events); // → INIT,ALL
        events.clear();
        decoder.handle_control("ACK,init", &mut events); // → done
        events.clear();
        decoder.handle_control("ACK,stray", &mut events); // ignored
        assert!(events.is_empty(), "post-init ACKs must not emit commands");
    }

    /// Closing takes the gateway off the bus, which it otherwise stays on;
    /// with no handshake there is nothing to undo.
    #[test]
    fn closing_sends_offline() {
        let encoder = Encoder {
            skip_init: false,
            refusal: None,
        };
        assert_eq!(encoder.shutdown_bytes(), b"$PDGY,N2NET_OFFLINE\r\n");
        let replay = Encoder {
            skip_init: true,
            refusal: None,
        };
        assert!(replay.shutdown_bytes().is_empty());
    }

    #[test]
    fn heartbeat_at_offline_wait_resends_offline() {
        // While we're waiting for the TEXT banner, the iKonvert may
        // emit any number of `$PDGY,000000,,,,,,` heartbeats. Each
        // should trigger a retry of the OFFLINE command.
        let decoder = Decoder::new(Config::default());
        let mut events = Vec::new();
        decoder.handle_control("000000,,,,,,", &mut events);
        assert_eq!(events.len(), 1, "heartbeat should trigger one resend");
        match &events[0] {
            DeviceEvent::SendBytes(b) => {
                assert_eq!(String::from_utf8_lossy(b), "$PDGY,N2NET_OFFLINE\r\n");
            }
            _ => panic!("expected SendBytes"),
        }
    }

    #[test]
    fn populated_heartbeat_emits_synthetic_frame() {
        // Drive past the init handshake so the resend path is inert
        // and the only event we should see is the synthesized frame.
        let decoder = Decoder::new(Config::default());
        let mut events = Vec::new();
        decoder.handle_control("TEXT,welcome", &mut events); // → RESET
        events.clear();
        decoder.handle_control("ACK,reset", &mut events); // → INIT,ALL
        events.clear();
        decoder.handle_control("ACK,init", &mut events); // → done
        events.clear();

        decoder.handle_control("000000,38,1,38,753,2,0", &mut events);
        assert_eq!(events.len(), 1, "data heartbeat should emit one frame");
        match &events[0] {
            DeviceEvent::Frame(f) => {
                assert_eq!(f.pgn, 0x40100); // IKONVERT_BEM
                assert_eq!(f.prio, 7);
                assert_eq!(f.src, 2, "src = gateway address from heartbeat");
                assert_eq!(f.dst, 255);
                assert_eq!(f.data.len(), 15);
                // Spot-check a couple of bytes; full coverage lives
                // in the engine unit test.
                assert_eq!(f.data[0], 38, "CAN load");
                assert_eq!(&f.data[6..10], &[0xf1, 0x02, 0, 0], "uptime LE = 753");
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn text_after_init_triggers_reinit() {
        let decoder = Decoder::new(Config::default());
        let mut events = Vec::new();
        decoder.handle_control("TEXT,banner", &mut events); // → RESET
        events.clear();
        decoder.handle_control("ACK,reset", &mut events); // → INIT,ALL
        events.clear();
        decoder.handle_control("ACK,init", &mut events); // → done
        events.clear();
        // Simulate the device rebooting and sending its banner again.
        decoder.handle_control("TEXT,banner2", &mut events);
        assert_eq!(events.len(), 1, "post-init TEXT should re-init");
        match &events[0] {
            DeviceEvent::SendBytes(b) => {
                let s = String::from_utf8_lossy(b);
                assert!(
                    s.starts_with("$PDGY,N2NET_RESET"),
                    "expected re-init to start with RESET, got {s:?}"
                );
            }
            _ => panic!("expected SendBytes"),
        }
    }

    #[test]
    fn corrupt_line_with_embedded_text_still_resets() {
        // Mid-frame device reboot: the iKonvert splices its banner
        // onto an unterminated `!PDGY,…` line. The decoder must
        // recover the TEXT signal even though parse_line errors.
        let mut decoder = Decoder::new(Config::default());
        // Drive to STATE_DONE first.
        let mut events = Vec::new();
        decoder.handle_control("TEXT,init", &mut events); // → RESET
        events.clear();
        decoder.handle_control("ACK,reset", &mut events); // → INIT,ALL
        events.clear();
        decoder.handle_control("ACK,init", &mut events); // → done
        events.clear();

        // Feed the actual garbage line observed on real hardware.
        let bad = b"!PDGY,65350,2,27,\xff$PDGY,TEXT,Digital_Yacht_iKonvert_v2_49_x\r\n";
        decoder.decode(bad, &mut events);

        // Expect: no `Error` event (TEXT was recovered) and exactly
        // one `SendBytes` carrying the re-init command.
        let send_count = events
            .iter()
            .filter(|e| matches!(e, DeviceEvent::SendBytes(_)))
            .count();
        let err_count = events
            .iter()
            .filter(|e| matches!(e, DeviceEvent::Error(_)))
            .count();
        assert_eq!(err_count, 0, "TEXT salvage should suppress the parse error");
        assert_eq!(send_count, 1, "device reset should re-init");
    }
}
