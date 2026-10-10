// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! An ISO 11783 bus participant — an NMEA 2000 or J1939 node — on a raw
//! CAN link, whatever carries the frames: the ISO 11783-5 address claim,
//! ISO Request and ISO TP, which both buses share, and on NMEA 2000 the
//! Heartbeat, Product Information, PGN list and Group Function
//! responders, fast-packet framing and the synthetic network-status PGN.
//! On a Quick bus, which has no addresses, it only relays 11-bit frames.
//!
//! The transports — [`super::socketcan`] on Linux and
//! [`super::canalyst`] over USB — own the link and a worker thread.
//! They feed every received frame to [`handle_frame`], every
//! application command to [`dispatch_cmd`], call [`NmeaDevice::tick`]
//! for the timers, and write out what lands in [`TxBuffer::queue`].
//!
//! `src` convention on outbound frames sent via `DeviceHandle::send_frame`:
//! `frame.src == 0` is treated as "use my claim address" and rewritten to
//! the currently-claimed source; any other value is sent on the wire as
//! given. While no address is claimed such a frame is dropped: ISO
//! 11783-5 allows only address claims (and requests for them) from the
//! null address (254), so an explicit `src == 254` is dropped otherwise.

use std::collections::{HashMap, VecDeque};
use std::sync::mpsc;
use std::time::{SystemTime, UNIX_EPOCH};

use smallvec::SmallVec;

use crate::engine::fastpacket;
use crate::engine::format::ikonvert::{NetworkStatus, build_network_status};
use crate::engine::format::{days_to_ymd, iso11783_compose, iso11783_decompose};
use crate::engine::frame::RawFrame;
use crate::engine::iso_tp::{PGN_TP_CM, TpSender};
use crate::engine::pgn_list::{self, PgnLists};
use crate::engine::{
    ADDR_GLOBAL, ADDR_NULL, BusProtocol, FramePacketType, Reassembled, Reassembler,
};
use crate::io::address_claim::{AddressClaim, ClaimState};
use crate::io::device::WriterCmd;
use crate::io::nmea_responder::{self, ProductInfo};

use super::socketcan::{BUILTIN_RX_PGNS as RX_PGN_LIST, BUILTIN_TX_PGNS as TX_PGN_LIST};

pub(super) const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
pub(super) const CAN_SFF_MASK: u32 = 0x7FF;

/// Bus-participant configuration. All fields have sensible defaults
/// via [`Config::default`]; tweak only what differs from canboat C
/// `socketcan-serial`'s built-in defaults.
#[derive(Debug, Clone)]
pub struct Config {
    /// Preferred source address to claim. Defaults to 0.
    pub address: u8,
    /// Unique number for the ISO NAME's identity field. 0 = derive
    /// from the machine id (stable per-host across restarts), or, if
    /// the machine can't be identified, a random number stored in
    /// `state_dir`.
    pub unique: u32,
    /// Manufacturer code for the ISO NAME. Defaults to 999 (Signal K).
    pub manufacturer: u16,
    /// System Instance (4 bits). Default 15 = maximum, so our NAME
    /// loses arbitration against any real sensor that leaves this
    /// at 0 — we yield rather than steal an address.
    pub system_instance: u8,
    /// Heartbeat (PGN 126993) interval in ms; 0 disables.
    pub heartbeat_ms: u64,
    /// Skip the ISO address-claim handshake entirely (passive sniff).
    pub no_claim: bool,
    /// Quit if no frame is received for this many seconds (0 disables).
    pub timeout_secs: u64,
    /// Override the PGN 126996 Model Version string. `None` defaults
    /// to `"socketcan-serial-rs"`; `canboat-pipeline` sets this to
    /// `"canboat-pipeline-rs"` so a downstream display can tell which
    /// canboat-rs binary is driving the bus.
    pub model_version: Option<&'static str>,
    /// When `true`, the driver owns the interface's link lifecycle: before
    /// opening the socket it brings the link down, sets `bitrate` with
    /// bus-off auto-recovery, and brings
    /// it up — retrying to ride out controllers that intermittently fail
    /// to configure at boot (notably the MCP2515's "didn't enter config
    /// mode"). `false` leaves the interface untouched, assuming it was
    /// configured externally (e.g. a systemd `ip link set … up` unit).
    // Read by the SocketCAN driver, which is Linux-only.
    #[cfg_attr(not(target_os = "linux"), allow(dead_code))]
    pub configure_link: bool,
    /// PGNs the application sends and receives through this gateway,
    /// advertised in the PGN 126464 lists after the gateway's own
    /// ISO housekeeping PGNs. The Transmit list should name every PGN
    /// the application originates.
    pub pgn_lists: PgnLists,
    /// Add each PGN the application transmits from the gateway's own
    /// address to the advertised Transmit list, as it goes out. On by
    /// default. It covers PGNs `pgn_lists` misses, but only from their
    /// first transmission on: a display that asked for the list before
    /// then is not told, so name the PGNs up front as well.
    pub learn_tx_pgns: bool,
    /// What the bus carries. NMEA 2000 (the default) makes the gateway
    /// a full NMEA 2000 node: fast-packet framing, Heartbeat, Product
    /// Information, PGN lists and Group Function. J1939 keeps only
    /// what the two share — address claim, ISO Request and ISO TP —
    /// frames everything as single frames, and claims a NAME in the
    /// Global industry group. Quick PCS makes it a passive bridge for
    /// 11-bit frames: no address claim and nothing of its own on the bus.
    pub protocol: BusProtocol,
    /// Bit rate for the managed bring-up (`configure_link`). NMEA 2000
    /// is always 250 kbit/s; J1939 is 250 kbit/s (J1939-11/-15) or
    /// 500 kbit/s (J1939-14).
    pub bitrate: u32,
    /// Where a random unique number is stored when `unique` is 0 and
    /// the machine can't be identified. `None`: it holds for this run.
    pub state_dir: Option<std::path::PathBuf>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            address: 0,
            unique: 0,
            manufacturer: 999,
            system_instance: 15,
            heartbeat_ms: 60_000,
            no_claim: false,
            timeout_secs: 0,
            model_version: None,
            configure_link: false,
            pgn_lists: PgnLists::default(),
            learn_tx_pgns: true,
            protocol: BusProtocol::Nmea2000,
            bitrate: 250_000,
            state_dir: None,
        }
    }
}

/// One CAN frame waiting to go out.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct TxFrame {
    /// The identifier: 29 bits, or 11 when `standard`.
    pub(super) id: u32,
    /// An 11-bit (standard) frame, for a Quick bus.
    pub(super) standard: bool,
    pub(super) data: SmallVec<[u8; 8]>,
}

/// What the network-status PGN reports about the link itself. Every
/// figure is optional: a link that cannot tell leaves canboat's
/// "unknown" in the PGN.
pub(super) trait LinkStats: Send {
    /// The bus bit rate.
    fn bitrate(&self) -> Option<u32> {
        None
    }
    /// Receive errors so far.
    fn errors(&self) -> Option<u32> {
        None
    }
    /// Frames that could not be sent so far.
    fn rejected_tx(&self) -> Option<u32> {
        None
    }
    /// Bytes and frames each way so far, at `now_ms`.
    fn load_sample(&self, _now_ms: u64) -> Option<LoadSample> {
        None
    }
}

/// A link that reports nothing about itself.
#[cfg(test)]
struct NoStats;

#[cfg(test)]
impl LinkStats for NoStats {}

/// How often to emit the synthetic `NMEA 2000 gateway: network
/// status` PGN (262400) into the upstream `frames_tx` channel.
/// Mirrors the ~1 Hz cadence the iKonvert pushes its `$PDGY`
/// heartbeat at; 5 s is plenty for human-visible status without
/// drowning the snapshot port in identical rows.
const NETWORK_STATUS_INTERVAL_MS: u64 = 5_000;

/// Per-frame protocol overhead in bits for CAN 2.0B extended frames
/// (NMEA 2000 is always 29-bit). Sum of SOF (1) + Arbitration
/// (11+SRR+IDE+18+RTR = 32) + Control (6) + CRC (16) + ACK (2) +
/// EOF (7) + IFS (3). Excludes the data bytes themselves and the
/// bit-stuffing inflation (applied as a scalar below).
const CAN_EFF_OVERHEAD_BITS: u64 = 67;

/// Average bit-stuffing inflation across arbitration / control /
/// data / CRC. ~20% is the canonical estimate for NMEA 2000
/// traffic; close enough for a single-byte load percentage.
const STUFFING_NUMER: u64 = 120;
const STUFFING_DENOM: u64 = 100;

/// One read of the kernel CAN counters we need for load
/// computation. Sampled `prev` → `curr` deltas over the time
/// between consecutive `emit_network_status` calls.
#[derive(Debug, Clone, Copy)]
pub(super) struct LoadSample {
    pub(super) rx_bytes: u64,
    pub(super) tx_bytes: u64,
    pub(super) rx_packets: u64,
    pub(super) tx_packets: u64,
    pub(super) at_ms: u64,
}

const PGN_ISO_REQUEST: u32 = 59904;
const PGN_ISO_ADDRESS_CLAIM: u32 = 60928;
const PGN_GROUP_FUNCTION: u32 = 126208;
const PGN_PGN_LIST: u32 = 126464;
const PGN_HEARTBEAT: u32 = 126993;
const PGN_PRODUCT_INFO: u32 = 126996;

/// Synthetic-PGN range — never on the wire, never sent to the bus.
const CANBOAT_PGN_START: u32 = 0x40000;

// Product Information (PGN 126996) content for the gateway itself.
const N2K_DB_VERSION: u16 = 2100; // 2.100 at 0.001 resolution
const PRODUCT_CODE: u16 = 1;
const CERTIFICATION_LEVEL: u8 = 0; // Level A
const LOAD_EQUIVALENCY: u8 = 1; // 1 LEN = 50 mA
// PGN 126996 "Model ID" carries our brand; the per-binary name
// ("socketcan-serial-rs" / "canboat-pipeline-rs") goes in
// "Model Version" via `Config::model_version`.
const MODEL_ID: &str = "CANboat";
const DEFAULT_MODEL_VERSION: &str = "socketcan-serial-rs";

// Group Function (PGN 126208) function codes and DURATION sentinels.
const GROUP_FUNCTION_REQUEST: u8 = 0;
const GROUP_FUNCTION_ACK: u8 = 2;
const TX_INTERVAL_NO_CHANGE: u32 = 0xffff_ffff;
const TX_INTERVAL_RESTORE_DEFAULT: u32 = 0xffff_fffe;

/// Hold outbound frames until the kernel CAN qdisc has room. The
/// worker thread drains one frame per POLLOUT wakeup so a single
/// fast-packet burst never starves RX or the claim timers. ~1 s of
/// bus time at 250 kbit/s with the default txqueuelen.
pub(super) const TX_BUFFER_CAPACITY: usize = 1024;

/// Worst-case latency for a user-side `send_frame` to reach the bus.
/// The worker drains `cmd_rx` after each `poll()` wakeup; a 100 ms
/// cap keeps stdin / quirk-synthesiser sends prompt without burning
/// CPU on idle.
pub(super) const MAX_POLL_MS: u64 = 100;

pub(super) fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// `YYYY-MM-DDTHH:MM:SS.mmmZ` from epoch milliseconds; matches
/// canboat C's `fmtTimestamp`.
pub(super) fn format_iso(ms: u64) -> String {
    let secs = (ms / 1000) as i64;
    let millis = ms % 1000;
    let days = secs.div_euclid(86_400);
    let tod = secs.rem_euclid(86_400);
    let (y, mo, d) = days_to_ymd(days);
    let (h, mi, s) = (tod / 3600, (tod % 3600) / 60, tod % 60);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}.{millis:03}Z")
}

/// One PGN 126464 list: the gateway's `builtin` PGNs plus the
/// application's, warning about any that cannot be advertised.
fn advertised_pgns(builtin: &[u32], extra: &[u32], which: &str) -> Vec<u32> {
    let (list, dropped) = pgn_list::merge(builtin, extra);
    if !dropped.is_empty() {
        log::warn!("CAN: not advertising {which} PGNs {dropped:?} (invalid, or the list is full)");
    }
    list
}

/// Build the 64-bit ISO NAME from a [`Config`].
fn build_name(config: &Config) -> u64 {
    let unique = if config.unique != 0 {
        config.unique
    } else {
        // Per-machine, stable across restarts. Two CANboat gateways
        // on the same host would collide; pass `--unique N` (or
        // `Config.unique`) to disambiguate.
        crate::engine::os::unique_number(config.state_dir.as_deref())
    };
    // PC Gateway (130) / Inter-Intranetwork Device (25); arbitrary-
    // address-capable is the builder's default. Marine industry group
    // on NMEA 2000, Global on J1939.
    let industry_group = match config.protocol {
        BusProtocol::J1939 => 0,
        _ => 4,
    };
    crate::io::name::Name::new(config.manufacturer, unique)
        .device_function(130)
        .device_class(25)
        .industry_group(industry_group)
        .system_instance(config.system_instance)
        .to_u64()
}

/// Outbound ring drained one frame per writability wakeup. Lives on
/// the worker thread; not shared, no locking.
pub(super) struct TxBuffer {
    pub(super) queue: VecDeque<TxFrame>,
    pub(super) overflowed: usize,
    /// Per-(pgn, src) fast-packet sequence counter, mod 8. The
    /// upper 3 bits of every fast-packet `frame[0]` carry this
    /// value; a strict receiver uses it to distinguish back-to-
    /// back instances of the same PGN from continuations of an
    /// in-flight reassembly. We previously left it at 0, so a
    /// receiver could fold two consecutive Product-Info responses
    /// into one corrupted reassembly.
    fast_seq: HashMap<(u32, u8), u8>,
    /// Decides the framing of every outbound PGN.
    protocol: BusProtocol,
    /// J1939 messages over 8 bytes, going out over ISO TP.
    pub(super) tp: TpSender,
}

impl TxBuffer {
    #[cfg(test)]
    fn new() -> Self {
        Self::with_protocol(BusProtocol::Nmea2000)
    }

    pub(super) fn with_protocol(protocol: BusProtocol) -> Self {
        Self {
            queue: VecDeque::with_capacity(64),
            overflowed: 0,
            fast_seq: HashMap::new(),
            protocol,
            tp: TpSender::new(),
        }
    }

    /// Queue a frame that already carries its own header.
    pub(super) fn push_frame(&mut self, f: &RawFrame) {
        self.push(iso11783_compose(f.prio, f.pgn, f.src, f.dst), &f.data);
    }

    /// Queue whatever ISO TP has due at `now`.
    pub(super) fn poll_tp(&mut self, now: u64) {
        for f in self.tp.poll(now) {
            self.push_frame(&f);
        }
    }

    /// Returns the next fast-packet sequence number for
    /// `(pgn, src)` and advances the counter.
    fn next_fast_seq(&mut self, pgn: u32, src: u8) -> u8 {
        let slot = self.fast_seq.entry((pgn, src)).or_insert(0);
        let s = *slot;
        *slot = (s + 1) & 0x07;
        s
    }

    fn push(&mut self, can_id: u32, data: &[u8]) {
        if data.len() > 8 {
            log::error!("could not build CAN frame for id {can_id:#x}");
            return;
        }
        self.enqueue(TxFrame {
            id: can_id & CAN_EFF_MASK,
            standard: false,
            data: SmallVec::from_slice(data),
        });
    }

    /// Queue an 11-bit (standard) frame, for a Quick bus.
    fn push_standard(&mut self, id: u32, data: &[u8]) {
        if id > CAN_SFF_MASK {
            log::warn!("CAN: not sending {id:#x}: not an 11-bit identifier");
            return;
        }
        if data.len() > 8 {
            log::warn!(
                "CAN: not sending {id:#x}: {} bytes do not fit one frame",
                data.len()
            );
            return;
        }
        self.enqueue(TxFrame {
            id,
            standard: true,
            data: SmallVec::from_slice(data),
        });
    }

    fn enqueue(&mut self, frame: TxFrame) {
        if self.queue.len() >= TX_BUFFER_CAPACITY {
            if self.overflowed == 0 {
                log::error!(
                    "CAN TX buffer full ({TX_BUFFER_CAPACITY} frames), dropping outbound frame"
                );
            }
            self.overflowed += 1;
            return;
        }
        self.queue.push_back(frame);
    }

    /// Take the oldest frame off the queue: it went out, or it never
    /// will. Clears the overflow once the queue has drained to half.
    pub(super) fn pop_sent(&mut self) {
        self.queue.pop_front();
        if self.overflowed > 0 && self.queue.len() < TX_BUFFER_CAPACITY / 2 {
            log::info!(
                "CAN TX buffer recovered ({} frames had been dropped)",
                self.overflowed
            );
            self.overflowed = 0;
        }
    }

    pub(super) fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

/// Bundles the resources every send-path needs: the socket, the TX
/// ring, and the consumer-side channel that receives a `RawFrame`
/// echo of every self-generated outbound. Passed by `&mut` through
/// the NmeaDevice methods so they stay free-functions in spirit.
pub(super) struct Bus<'a> {
    pub(super) tx_buf: &'a mut TxBuffer,
    pub(super) frames_tx: &'a mpsc::Sender<RawFrame>,
}

impl<'a> Bus<'a> {
    /// Enqueue a self-generated outbound PGN. On NMEA 2000, splits
    /// fast-packet PGNs into fast-packet chunks for the wire; when `emit`
    /// is true the consumer also sees one coalesced `RawFrame` on
    /// `frames_tx` (same contract as inbound — never split).
    fn send_pgn(&mut self, prio: u8, pgn: u32, src: u8, dst: u8, data: &[u8], emit: bool) {
        let can_id = iso11783_compose(prio, pgn, src, dst);
        // Single vs fast-packet is decided strictly from the bus and,
        // on NMEA 2000, the PGN type table generated at build time
        // from canboat.json.
        // **Never** infer from `data.len()` alone: a single-frame
        // PGN whose payload happens to be 8 bytes (PGN 127508
        // Battery Status is the canonical example) must NOT be
        // wrapped in a fast-packet shell — the receiver's schema
        // would decode the seq/len header bytes as the first two
        // payload fields and produce wildly wrong values.
        // Conversely, a fast-packet PGN with ≤8 bytes of payload
        // (PGN 126208 Group Function ACK, 6 bytes) still has to go
        // out with the fast-packet wrapper because strict
        // receivers key on the PGN type, not the byte count.
        let is_fast = self.tx_buf.protocol.packet_type(pgn) == FramePacketType::Fast;
        if !is_fast && data.len() > 8 {
            if self.tx_buf.protocol == BusProtocol::Nmea2000 {
                // A single-frame PGN cannot carry more; never
                // truncate it.
                log::warn!(
                    "CAN: not sending PGN {pgn}: {} bytes do not fit one frame",
                    data.len()
                );
                return;
            }
            // J1939: ISO TP, a BAM to global or RTS/CTS to one node.
            match self.tx_buf.tp.send(now_ms(), pgn, src, dst, data) {
                Ok(frames) => {
                    for f in &frames {
                        self.tx_buf.push_frame(f);
                    }
                }
                Err(e) => {
                    log::warn!("CAN: not sending PGN {pgn} over ISO TP: {e}");
                    return;
                }
            }
        } else if !is_fast {
            self.tx_buf.push(can_id, data);
        } else {
            let seq = self.tx_buf.next_fast_seq(pgn, src);
            let Some(frames) = fastpacket::fragment(seq, data) else {
                log::warn!(
                    "CAN: not sending PGN {pgn}: {} bytes exceed one fast-packet",
                    data.len()
                );
                return;
            };
            for frame in frames {
                self.tx_buf.push(can_id, &frame);
            }
        }
        if emit {
            let f = RawFrame::new(
                Some(format_iso(now_ms())),
                prio,
                pgn,
                src,
                dst,
                data.iter().copied(),
            );
            let _ = self.frames_tx.send(f);
        }
    }
}

/// A full NMEA 2000 bus participant: the ISO 11783-5 address-claim
/// state machine ([`AddressClaim`]) plus this gateway's device-
/// specific responders — Product Information, PGN lists, heartbeat,
/// Group Function, and the synthetic network-status PGN. The claim
/// handshake itself lives in [`crate::io::address_claim`] and is shared
/// with the server's Motion-Sensor impersonation quirk.
pub(super) struct NmeaDevice {
    /// The shared address-claim state machine (owns NAME, address,
    /// state, and the used-address table).
    pub(super) claim: AddressClaim,
    /// `Config::protocol`. The NMEA 2000-only responders (Heartbeat,
    /// Product Information, PGN lists, Group Function) stay silent
    /// on J1939.
    pub(super) protocol: BusProtocol,
    pub(super) heartbeat_interval: u64, // ms, 0 disables
    heartbeat_seq: u8,
    pub(super) next_heartbeat: u64, // ms
    last_product_info: u64,         // ms; rate-limit broadcast bursts
    model_version: &'static str,
    /// PGN 126464 lists: the housekeeping PGNs plus `Config::pgn_lists`.
    tx_pgns: Vec<u32>,
    rx_pgns: Vec<u32>,
    /// `Config::learn_tx_pgns`.
    learn_tx_pgns: bool,
    /// Whether the "Transmit list is full" warning has been given.
    tx_pgns_full_warned: bool,
    // -- NMEA 2000 gateway: network status (PGN 262400) emission --
    /// Where the network-status tick reads the link's counters: the
    /// kernel's for SocketCAN, the node's own count for a USB adapter.
    stats: Box<dyn LinkStats>,
    /// Wall-clock ms at worker construction. The `uptime_s` field
    /// is `(now - start_ms) / 1000`.
    start_ms: u64,
    /// Next wall-clock ms at which to emit the network-status PGN.
    /// Set on first claim; reset on each emission.
    next_network_status: u64,
    /// Distinct N2K source addresses we have ever received a
    /// (non-error, extended) frame from. Strictly bigger than the
    /// claim state machine's used-address table, which is only
    /// updated on PGN 60928 claims — many real devices never
    /// re-announce, so the claim table undercounts the bus.
    seen_addrs: [bool; 256],
    /// Bus bitrate (bits/s), read once at construction from
    /// [`LinkStats::bitrate`], or `Config::bitrate` if the link does
    /// not know it.
    bitrate_bps: u32,
    /// Most recent `LoadSample`, or `None` before the first
    /// network-status emission. Used to compute the bytes/packets
    /// delta over the time between the previous and current emit.
    /// First emit reports `load_pct = None` (no baseline yet).
    prev_load_sample: Option<LoadSample>,
    /// Frames [`dispatch_cmd`] dropped because we had no address to
    /// send them from, since we last had one. Logged once per spell.
    dropped_unclaimed: u64,
}

/// Send each frame the address-claim state machine produced onto the
/// bus (with a local echo — same contract as `send_pgn(..., true)`).
fn emit(bus: &mut Bus<'_>, frames: Vec<RawFrame>) {
    for f in frames {
        bus.send_pgn(f.prio, f.pgn, f.src, f.dst, &f.data, true);
    }
}

impl NmeaDevice {
    pub(super) fn new(config: &Config, stats: Box<dyn LinkStats>) -> Self {
        let name = build_name(config);
        // A Quick bus has no addresses to claim: the gateway only
        // relays its 11-bit frames.
        let claim = if config.no_claim || config.protocol == BusProtocol::Quick {
            AddressClaim::disabled(name)
        } else {
            // Our NAME is arbitrary-address-capable (`build_name` sets
            // the bit), so we yield and move on a lost conflict.
            AddressClaim::new(name, config.address, true)
        };
        let nmea2000 = config.protocol == BusProtocol::Nmea2000;
        Self {
            claim,
            protocol: config.protocol,
            heartbeat_interval: if nmea2000 { config.heartbeat_ms } else { 0 },
            heartbeat_seq: 0,
            next_heartbeat: 0,
            last_product_info: 0,
            model_version: config.model_version.unwrap_or(DEFAULT_MODEL_VERSION),
            tx_pgns: advertised_pgns(&TX_PGN_LIST, &config.pgn_lists.tx, "transmit"),
            rx_pgns: advertised_pgns(&RX_PGN_LIST, &config.pgn_lists.rx, "receive"),
            learn_tx_pgns: config.learn_tx_pgns && nmea2000,
            tx_pgns_full_warned: false,
            start_ms: now_ms(),
            next_network_status: 0,
            seen_addrs: [false; 256],
            bitrate_bps: stats.bitrate().unwrap_or(config.bitrate),
            stats,
            prev_load_sample: None,
            dropped_unclaimed: 0,
        }
    }

    /// The address we send from ([`AddressClaim::send_address`]), or
    /// [`ADDR_NULL`] when we have none. Device responders below only
    /// run once claimed, and [`dispatch_cmd`] drops what would go out
    /// from the null address, so nothing but a claim is sent from it.
    fn addr(&self) -> u8 {
        self.claim.send_address().unwrap_or(ADDR_NULL)
    }

    /// Our 64-bit ISO NAME.
    fn name(&self) -> u64 {
        self.claim.name()
    }

    /// Kick off the claim handshake (scan → claim).
    pub(super) fn start(&mut self, bus: &mut Bus<'_>) {
        emit(bus, self.claim.start(now_ms()));
    }

    /// Feed an inbound PGN 60928 Address Claim to the state machine.
    fn on_claim(&mut self, bus: &mut Bus<'_>, src: u8, data: &[u8]) {
        if data.len() < 8 {
            return;
        }
        let their_name = u64::from_le_bytes(data[..8].try_into().unwrap());
        emit(bus, self.claim.on_address_claim(now_ms(), src, their_name));
    }

    fn on_request(&mut self, bus: &mut Bus<'_>, src: u8, dst: u8, data: &[u8]) {
        if data.len() < 3 {
            return;
        }
        let requested = data[0] as u32 | (data[1] as u32) << 8 | (data[2] as u32) << 16;
        let addressed = dst == self.addr();
        if !addressed && dst != ADDR_GLOBAL {
            return; // request is for some other node
        }

        // The address claim must be answerable even before fully claimed.
        if requested == PGN_ISO_ADDRESS_CLAIM {
            if let Some(f) = self.claim.respond_to_claim_request() {
                emit(bus, vec![f]);
            }
            return;
        }

        if !self.claim.is_claimed() {
            return; // need a claimed address to answer from
        }

        let nmea2000 = self.protocol == BusProtocol::Nmea2000;
        match requested {
            PGN_PRODUCT_INFO if nmea2000 => self.send_product_info(bus),
            PGN_PGN_LIST if nmea2000 => self.send_pgn_list(bus, src),
            PGN_HEARTBEAT if nmea2000 => self.send_heartbeat(bus),
            _ => {
                // ISO 11783-3: NAK an addressed request for a PGN we do
                // not send; silently ignore an unsupported global request.
                if addressed {
                    self.send_iso_ack(bus, src, 1 /* NAK */, requested);
                }
            }
        }
    }

    // Product Information, PGN 126996. Broadcast (PDU2), so one reply
    // answers every requester; rate-limited to collapse discovery bursts.
    fn send_product_info(&mut self, bus: &mut Bus<'_>) {
        let now = now_ms();
        if now - self.last_product_info < 1000 {
            return;
        }
        self.last_product_info = now;

        let serial = (self.name() & 0x1fffff).to_string();
        let pi = ProductInfo {
            db_version: i64::from(N2K_DB_VERSION),
            product_code: i64::from(PRODUCT_CODE),
            model_id: MODEL_ID,
            software_version: env!("CARGO_PKG_VERSION"),
            model_version: self.model_version,
            model_serial: &serial,
            certification_level: i64::from(CERTIFICATION_LEVEL),
            load_equivalency: i64::from(LOAD_EQUIVALENCY),
        };
        emit(bus, pi.frame(self.addr()).into_iter().collect());
    }

    /// Advertise `pgn` in the Transmit list from now on: the
    /// application just sent it from our address.
    fn learn_tx_pgn(&mut self, pgn: u32) {
        // A frame can carry a PGN the extended CAN id encodes but NMEA
        // 2000 does not define; never advertise one.
        if pgn > pgn_list::MAX_PGN || !self.learn_tx_pgns || self.tx_pgns.contains(&pgn) {
            return;
        }
        if self.tx_pgns.len() >= pgn_list::MAX_PGN_LIST_LEN {
            if !self.tx_pgns_full_warned {
                log::warn!("CAN: transmit PGN list is full; not advertising PGN {pgn}");
                self.tx_pgns_full_warned = true;
            }
            return;
        }
        log::info!("CAN: advertising transmit PGN {pgn}, first sent now");
        self.tx_pgns.push(pgn);
    }

    // PGN List (Transmit and Receive), PGN 126464: one message per list.
    fn send_pgn_list(&self, bus: &mut Bus<'_>, dst: u8) {
        emit(
            bus,
            nmea_responder::pgn_list_frames(self.addr(), dst, &self.tx_pgns, &self.rx_pgns),
        );
    }

    // ISO Acknowledgement, PGN 59392.
    fn send_iso_ack(&self, bus: &mut Bus<'_>, dst: u8, control: u8, pgn: u32) {
        emit(
            bus,
            vec![nmea_responder::iso_ack_frame(
                self.addr(),
                dst,
                control,
                pgn,
            )],
        );
    }

    // Acknowledge Group Function, PGN 126208 function 2.
    fn send_ack_group_function(
        &self,
        bus: &mut Bus<'_>,
        dst: u8,
        pgn: u32,
        pgn_err: u8,
        param_err: u8,
    ) {
        let p = pgn.to_le_bytes();
        let data = [
            GROUP_FUNCTION_ACK,
            p[0],
            p[1],
            p[2],
            (pgn_err & 0x0f) | ((param_err & 0x0f) << 4),
            0, // number of parameters
        ];
        bus.send_pgn(6, PGN_GROUP_FUNCTION, self.addr(), dst, &data, true);
    }

    // NMEA Request Group Function (PGN 126208, function 0). We act only on
    // a request targeting our Heartbeat (PGN 126993): it sets the transmit
    // interval (or disables it), then we reply with an Acknowledge.
    fn handle_group_function(&mut self, bus: &mut Bus<'_>, src: u8, data: &[u8]) {
        if data.len() < 8 || data[0] != GROUP_FUNCTION_REQUEST || !self.claim.is_claimed() {
            return;
        }
        let target = data[1] as u32 | (data[2] as u32) << 8 | (data[3] as u32) << 16;
        if target != PGN_HEARTBEAT {
            return;
        }
        // Transmission interval: 32-bit, 0.001s resolution => ms.
        let interval = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let mut param_err = 0u8;
        match interval {
            TX_INTERVAL_NO_CHANGE => {}
            TX_INTERVAL_RESTORE_DEFAULT => {
                self.heartbeat_interval = 60000;
                self.next_heartbeat = now_ms() + self.heartbeat_interval;
                log::info!("Heartbeat interval restored to default 60000 ms");
            }
            0 => {
                self.heartbeat_interval = 0;
                log::info!("Heartbeat disabled by group function");
            }
            1000..=60000 => {
                self.heartbeat_interval = interval as u64;
                self.next_heartbeat = now_ms() + self.heartbeat_interval;
                log::info!("Heartbeat interval set to {interval} ms by group function");
            }
            _ => {
                param_err = 2; // transmission interval out of range
                log::error!("Requested heartbeat interval {interval} ms out of range");
            }
        }
        self.send_ack_group_function(bus, src, PGN_HEARTBEAT, 0, param_err);
    }

    pub(super) fn tick(&mut self, bus: &mut Bus<'_>, now: u64) {
        // Advance the shared claim state machine and put any claim
        // frames it produced on the wire.
        let was_claimed = self.claim.is_claimed();
        emit(bus, self.claim.tick(now));
        // The moment we take ownership: announce ourselves once and
        // arm the heartbeat / network-status timers.
        if !was_claimed && self.claim.is_claimed() {
            log::info!("Address {} claimed", self.addr());
            if self.protocol == BusProtocol::Nmea2000 {
                self.send_product_info(bus);
            }
            if self.heartbeat_interval > 0 {
                self.next_heartbeat = now + self.heartbeat_interval;
            }
            // First network-status drop one interval after claim,
            // so a downstream snapshot client has the data even
            // before the first bus traffic.
            self.next_network_status = now + NETWORK_STATUS_INTERVAL_MS;
        }
        if self.claim.is_claimed() && self.heartbeat_interval > 0 && now >= self.next_heartbeat {
            self.send_heartbeat(bus);
            self.next_heartbeat = now + self.heartbeat_interval;
        }
        if self.claim.is_claimed() && now >= self.next_network_status {
            self.emit_network_status(bus, now);
            self.next_network_status = now + NETWORK_STATUS_INTERVAL_MS;
        }
    }

    /// Note that we received a frame from `src`. Used by the
    /// network-status emitter's device-count field — the claim
    /// state machine's used-address table only tracks PGN 60928
    /// announcers (many real devices never re-announce), so this
    /// captures everything the wire shows.
    fn note_seen(&mut self, src: u8) {
        if (src as usize) < self.seen_addrs.len() {
            self.seen_addrs[src as usize] = true;
        }
    }

    /// Build and push a synthetic `NMEA 2000 gateway: network
    /// status` (PGN 262400) frame into `frames_tx`. Never goes on
    /// the wire — it's a BEM PGN by design. Errors, rejected TX
    /// and load come from [`LinkStats`]; everything else from
    /// in-process counters.
    fn emit_network_status(&mut self, bus: &mut Bus<'_>, now: u64) {
        let uptime_s = ((now - self.start_ms) / 1000) as u32;
        let device_count = self
            .seen_addrs
            .iter()
            .filter(|seen| **seen)
            .count()
            .min(u8::MAX as usize) as u8;
        let errors = self.stats.errors();
        let rejected_tx = self.stats.rejected_tx();
        // Load: delta against the previous sample. First emit has
        // no baseline, so load_pct stays None and the canboat
        // sentinel rides through; baseline shifts forward each
        // call.
        let curr_sample = self.stats.load_sample(now);
        let load_pct = match (self.prev_load_sample, curr_sample) {
            (Some(prev), Some(curr)) => compute_load_pct(prev, curr, self.bitrate_bps),
            _ => None,
        };
        if let Some(curr) = curr_sample {
            self.prev_load_sample = Some(curr);
        }
        let frame = build_network_status(
            NetworkStatus {
                load_pct,
                errors,
                device_count: Some(device_count),
                uptime_s: Some(uptime_s),
                gateway_addr: self.addr(),
                rejected_tx,
            },
            Some(format_iso(now)),
        );
        let _ = bus.frames_tx.send(frame);
    }

    // NMEA 2000 Heartbeat, PGN 126993. Sent every heartbeat_interval ms
    // once we own an address so other nodes know we are alive.
    // An interval the frame cannot carry stops the heartbeat.
    fn send_heartbeat(&mut self, bus: &mut Bus<'_>) {
        match nmea_responder::heartbeat_frame(
            self.addr(),
            self.heartbeat_seq,
            self.heartbeat_interval,
        ) {
            Ok(frame) => emit(bus, vec![frame]),
            Err(e) => {
                log::error!("Heartbeat disabled: {e}");
                self.heartbeat_interval = 0;
                return;
            }
        }
        self.heartbeat_seq = if self.heartbeat_seq >= 252 {
            0
        } else {
            self.heartbeat_seq + 1
        };
    }
}

/// CAN bus load percentage from two samples and the bus bitrate.
/// Accounts for the per-frame CAN protocol overhead and a flat
/// 20% bit-stuffing inflation.
pub(super) fn compute_load_pct(prev: LoadSample, curr: LoadSample, bitrate_bps: u32) -> Option<u8> {
    let dt_ms = curr.at_ms.saturating_sub(prev.at_ms);
    if dt_ms == 0 || bitrate_bps == 0 {
        return None;
    }
    // The counters are the kernel's monotonic u64, read at full width
    // by `read_sysfs_counter_u64`, so they only ever go backwards if
    // the interface was reset under us (an `ip link set down/up`).
    // There is no meaningful load for that interval — the traffic
    // since the last sample is simply unknown — so say so rather than
    // subtract across the reset. Reporting a number here would mean
    // reporting a wrong one: the deltas would come out near u64::MAX
    // and saturate to a flat 100%.
    //
    // The caller stores `curr` as the next baseline whether or not
    // this returns a figure, so one interval is lost, not the series.
    if curr.rx_bytes < prev.rx_bytes
        || curr.tx_bytes < prev.tx_bytes
        || curr.rx_packets < prev.rx_packets
        || curr.tx_packets < prev.tx_packets
    {
        return None;
    }
    let d_bytes = (curr.rx_bytes - prev.rx_bytes) + (curr.tx_bytes - prev.tx_bytes);
    let d_packets = (curr.rx_packets - prev.rx_packets) + (curr.tx_packets - prev.tx_packets);
    // bits_raw = data bytes * 8 + packets * (SOF + arb + ctrl +
    // CRC + ACK + EOF + IFS). bits_on_wire scales by the
    // stuffing factor, then load_pct = bits / (bitrate * Δt).
    let bits_raw = d_bytes
        .saturating_mul(8)
        .saturating_add(d_packets.saturating_mul(CAN_EFF_OVERHEAD_BITS));
    let bits_on_wire = bits_raw
        .saturating_mul(STUFFING_NUMER)
        .saturating_div(STUFFING_DENOM);
    // pct = bits * 100 / (bitrate * dt_seconds)
    //     = bits * 100_000 / (bitrate * dt_ms)
    let denom = (bitrate_bps as u64).saturating_mul(dt_ms);
    if denom == 0 {
        return None;
    }
    let pct = bits_on_wire.saturating_mul(100_000) / denom;
    Some(pct.min(100) as u8)
}

/// Translate an incoming single-frame CAN message. Drives the
/// claim state machine immediately on the raw single frame (its
/// PGNs — 60928, 59904 — are all single-frame anyway), then
/// pushes through the reassembler. On a coalesced output we (a)
/// invoke the Group Function handler if it's PGN 126208 and (b)
/// forward the *coalesced* `RawFrame` to `frames_tx` so the
/// consumer (canboat-pipeline, socketcan-serial stdout, …) sees a
/// complete PGN, matching the NGT-1 / iKonvert adapter contract.
pub(super) fn handle_frame(
    bus: &mut Bus<'_>,
    claimer: &mut NmeaDevice,
    reasm: &mut Reassembler,
    can_id: u32,
    data: &[u8],
    when: u64,
) {
    let (prio, pgn, src, dst) = iso11783_decompose(can_id);
    let single_frame = RawFrame::new(
        Some(format_iso(when)),
        prio,
        pgn,
        src,
        dst,
        data.iter().copied(),
    );

    // A CTS, EOMA or Abort for one of our ISO TP transfers.
    if claimer.protocol == BusProtocol::J1939 && pgn == PGN_TP_CM {
        for f in bus.tx_buf.tp.on_frame(when, &single_frame) {
            bus.tx_buf.push_frame(&f);
        }
    }

    if claimer.claim.state() != ClaimState::Disabled {
        if pgn == PGN_ISO_ADDRESS_CLAIM {
            claimer.on_claim(bus, src, data);
        } else if pgn == PGN_ISO_REQUEST {
            claimer.on_request(bus, src, dst, data);
        }
    }

    // Track every distinct src for the network-status PGN's
    // device-count field. `claimer.used` only fires on PGN 60928
    // (ISO Address Claim), and many real devices don't
    // re-announce after we attach, so a separate tracker is
    // required for a realistic per-snapshot count.
    if src != ADDR_NULL && src != ADDR_GLOBAL {
        claimer.note_seen(src);
    }

    // Classify by the bus (and, on NMEA 2000, the build-time
    // fastpacket table), push through the reassembler, and forward
    // the coalesced result. A real single-frame PGN takes the
    // `PassThrough` branch unchanged; a fast-packet PGN accumulates
    // until `Complete`; ISO TP is reassembled on either bus.
    let pt = claimer.protocol.packet_type(pgn);
    match reasm.push(single_frame, pt) {
        Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
            if claimer.claim.state() != ClaimState::Disabled
                && claimer.protocol == BusProtocol::Nmea2000
                && f.pgn == PGN_GROUP_FUNCTION
            {
                claimer.handle_group_function(bus, src, &f.data);
            }
            let _ = bus.frames_tx.send(f);
        }
        Reassembled::Partial => {}
        Reassembled::Error(e) => log::debug!("reassembly: {e}"),
    }
}

/// Apply an outbound `WriterCmd` from the public `DeviceHandle` API.
/// `src == 0` (PC-gateway / unset default) and `src == 255`
/// (broadcast — never a valid source on the bus) are both
/// interpreted as "use my claim address", and dropped while we have
/// none; any other value is forwarded unchanged so quirk synthesisers can impersonate other
/// nodes on the wire (e.g. the SCX-20 quirk uses `src = 52`).
pub(super) fn dispatch_cmd(bus: &mut Bus<'_>, claimer: &mut NmeaDevice, cmd: WriterCmd) {
    match cmd {
        WriterCmd::Frame(f) => {
            if f.pgn >= CANBOAT_PGN_START {
                return; // synthetic, never goes on the bus
            }
            // Quick: the message type is the 11-bit identifier, and
            // one frame is the whole message.
            if claimer.protocol == BusProtocol::Quick {
                bus.tx_buf.push_standard(f.pgn, &f.data);
                return;
            }
            // ISO 11783-5: from the null address only an address
            // claim ("cannot claim") or a Request for Address Claimed
            // may go out, whoever's frame it is.
            if f.src == ADDR_NULL && !may_send_from_null(f.pgn, &f.data) {
                log::debug!("CAN: dropping PGN {} from the null address", f.pgn);
                return;
            }
            let own = claimer.claim.send_address();
            let src = if f.src == 0 || f.src == ADDR_GLOBAL {
                // ISO 11783-5: only an address claim may go out from
                // the null address, so without an address of our own
                // (scanning, settling a new claim, or none to be had)
                // there is nothing to send this from.
                let Some(own) = own else {
                    if claimer.dropped_unclaimed == 0 {
                        log::warn!(
                            "CAN: no source address claimed yet; dropping outbound \
                             frames until one is"
                        );
                    }
                    claimer.dropped_unclaimed += 1;
                    return;
                };
                own
            } else {
                f.src
            };
            if own.is_some() && claimer.dropped_unclaimed > 0 {
                log::info!(
                    "CAN: sending from address {} again; dropped {} frames without one",
                    claimer.addr(),
                    claimer.dropped_unclaimed
                );
                claimer.dropped_unclaimed = 0;
            }
            // Only what goes out as us: a quirk impersonating another
            // device sends from that device's address.
            if Some(src) == own {
                claimer.learn_tx_pgn(f.pgn);
            }
            // emit=false: this is a user-initiated send; the caller
            // already has visibility into what they sent. (For the
            // SCX-20 quirk we want the synthetic 126996 to be
            // visible in the local pipeline too — the caller injects
            // it into the analyzer side directly.)
            bus.send_pgn(f.prio, f.pgn, src, f.dst, &f.data, false);
        }
        // Handled by the worker loop, which owns the socket.
        WriterCmd::Shutdown(_) => {}
        WriterCmd::Bytes(_) => {
            // The SocketCAN backend has no concept of raw "bytes"
            // since the wire format is frame-based. Silently drop;
            // log so a misuse is at least visible.
            log::debug!("WriterCmd::Bytes ignored by the CAN node");
        }
    }
}

/// Whether a frame may go out from the null address (254): an Address
/// Claim, or an ISO Request for one.
fn may_send_from_null(pgn: u32, data: &[u8]) -> bool {
    pgn == PGN_ISO_ADDRESS_CLAIM
        || (pgn == PGN_ISO_REQUEST
            && data.len() >= 3
            && u32::from_le_bytes([data[0], data[1], data[2], 0]) == PGN_ISO_ADDRESS_CLAIM)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Run one `send_pgn` against a fresh (or provided) TX ring and
    /// return every queued CAN payload, so the tests pin the exact
    /// wire bytes the chunking produces.
    fn send(tx_buf: &mut TxBuffer, pgn: u32, data: &[u8]) -> Vec<Vec<u8>> {
        let before = tx_buf.queue.len();
        let (frames_tx, _frames_rx) = mpsc::channel();
        let mut bus = Bus {
            tx_buf,
            frames_tx: &frames_tx,
        };
        bus.send_pgn(6, pgn, 42, ADDR_GLOBAL, data, false);
        bus.tx_buf
            .queue
            .iter()
            .skip(before)
            .map(|f| f.data.to_vec())
            .collect()
    }

    /// Build a load sample; only the fields a test varies matter.
    fn sample(at_ms: u64, rx_bytes: u64, rx_packets: u64) -> LoadSample {
        LoadSample {
            rx_bytes,
            tx_bytes: 0,
            rx_packets,
            tx_packets: 0,
            at_ms,
        }
    }

    /// A quiet bus reads as 0%, not as `None`: two samples a second
    /// apart with no traffic between them is a real measurement.
    #[test]
    fn an_idle_bus_reads_zero_percent() {
        let pct = compute_load_pct(sample(0, 0, 0), sample(1000, 0, 0), 250_000);
        assert_eq!(pct, Some(0));
    }

    /// Load counts data bits plus per-frame CAN overhead, inflated
    /// 20% for bit stuffing. 100 frames of 8 bytes in one second at
    /// 250 kbit/s: (100*8*8 + 100*67) * 1.2 = 15_720 bits -> 6%.
    #[test]
    fn load_includes_frame_overhead_and_stuffing() {
        let pct = compute_load_pct(sample(0, 0, 0), sample(1000, 800, 100), 250_000);
        assert_eq!(pct, Some(6));
    }

    /// Both directions count toward bus load — the wire carries TX
    /// and RX alike.
    #[test]
    fn tx_and_rx_both_count() {
        let prev = sample(0, 0, 0);
        let mut curr = sample(1000, 800, 100);
        curr.tx_bytes = 800;
        curr.tx_packets = 100;
        let both = compute_load_pct(prev, curr, 250_000).unwrap();
        let rx_only = compute_load_pct(prev, sample(1000, 800, 100), 250_000).unwrap();
        assert_eq!(both, rx_only * 2);
    }

    /// A bus busier than the bitrate allows still reports a valid
    /// percentage rather than overflowing the byte.
    #[test]
    fn load_saturates_at_one_hundred() {
        let pct = compute_load_pct(sample(0, 0, 0), sample(1000, 10_000_000, 1), 250_000);
        assert_eq!(pct, Some(100));
    }

    /// The counters are read at the kernel's full u64 width, so a
    /// byte total past 4 GiB is ordinary traffic, not a wrap: the
    /// delta across that boundary reads as the 800 bytes it is.
    /// (Truncating to u32 first would make this 100%.)
    #[test]
    fn a_counter_past_four_gigabytes_is_not_a_wrap() {
        let prev = sample(0, u32::MAX as u64 - 399, 50);
        let curr = sample(1000, u32::MAX as u64 + 401, 150);
        assert_eq!(compute_load_pct(prev, curr, 250_000), Some(6));
    }

    /// An `ip link set down/up` resets the interface counters, so the
    /// traffic over that interval is unknown. Report nothing — the
    /// canboat sentinel rides through — rather than a figure that
    /// would be a flat, wrong 100%.
    #[test]
    fn a_counter_reset_reports_no_load() {
        let prev = sample(0, 5_000_000, 100_000);
        let curr = sample(1000, 0, 0);
        assert_eq!(compute_load_pct(prev, curr, 250_000), None);
    }

    /// A reset costs one interval, not the series: the next pair of
    /// samples measures normally against the new baseline.
    #[test]
    fn the_interval_after_a_reset_measures_normally() {
        let after_reset = sample(1000, 0, 0);
        let next = sample(2000, 800, 100);
        assert_eq!(compute_load_pct(after_reset, next, 250_000), Some(6));
    }

    /// Two samples at the same instant give no interval to divide
    /// by, so there is no measurement to report.
    #[test]
    fn a_zero_interval_has_no_load() {
        assert_eq!(
            compute_load_pct(sample(5, 0, 0), sample(5, 800, 100), 250_000),
            None
        );
    }

    /// A zero bitrate would divide by zero; report nothing instead.
    #[test]
    fn a_zero_bitrate_has_no_load() {
        assert_eq!(
            compute_load_pct(sample(0, 0, 0), sample(1000, 800, 100), 0),
            None
        );
    }

    /// The same traffic is a heavier load on a slower bus.
    #[test]
    fn a_slower_bus_reads_a_higher_load() {
        let prev = sample(0, 0, 0);
        let curr = sample(1000, 800, 100);
        let slow = compute_load_pct(prev, curr, 125_000).unwrap();
        let fast = compute_load_pct(prev, curr, 250_000).unwrap();
        assert!(slow > fast, "{slow} should exceed {fast}");
    }

    /// A link that does not know its bit rate falls back to the
    /// configured one, so a 500 kbit/s J1939-14 bus does not read twice
    /// the load it carries.
    #[test]
    fn a_link_without_a_bitrate_falls_back_to_the_configured_one() {
        let config = Config {
            bitrate: 500_000,
            ..j1939_config()
        };
        let dev = NmeaDevice::new(&config, Box::new(NoStats));
        assert_eq!(dev.bitrate_bps, 500_000);
    }

    /// The timestamp matches canboat C's `fmtTimestamp`, including
    /// the day rollover and the Zulu suffix.
    #[test]
    fn iso_timestamps_match_canboat_c() {
        assert_eq!(format_iso(0), "1970-01-01T00:00:00.000Z");
        assert_eq!(format_iso(90_061_500), "1970-01-02T01:01:01.500Z");
        assert_eq!(format_iso(1_764_500_000_123), "2025-11-30T10:53:20.123Z");
    }

    /// The hoisted housekeeping lists name the same PGNs as the
    /// driver's own constants, in the order they were always sent.
    #[test]
    fn the_builtin_lists_are_the_housekeeping_pgns() {
        assert_eq!(
            TX_PGN_LIST,
            [
                59392, // ISO Acknowledgement
                PGN_ISO_REQUEST,
                PGN_ISO_ADDRESS_CLAIM,
                PGN_GROUP_FUNCTION,
                PGN_PGN_LIST,
                PGN_HEARTBEAT,
                PGN_PRODUCT_INFO,
            ]
        );
        assert_eq!(
            RX_PGN_LIST,
            [PGN_ISO_REQUEST, PGN_ISO_ADDRESS_CLAIM, PGN_GROUP_FUNCTION]
        );
    }

    /// The application's PGNs follow the housekeeping ones in the
    /// 126464 answer, and the status reports what did not fit.
    #[test]
    fn the_pgn_lists_carry_the_application_pgns() {
        use crate::engine::pgn_list::{PgnListSupport, PgnLists};
        let config = Config {
            pgn_lists: PgnLists {
                tx: vec![127508, 127506, 0x40000],
                rx: vec![127245],
            },
            ..Default::default()
        };
        let dev = NmeaDevice::new(&config, Box::new(NoStats));
        assert_eq!(&dev.tx_pgns[..7], &TX_PGN_LIST);
        assert_eq!(&dev.tx_pgns[7..], [127508, 127506]);
        assert_eq!(&dev.rx_pgns[3..], [127245]);
        let status = super::super::socketcan::pgn_list_status(&config.pgn_lists);
        assert_eq!(status.tx, PgnListSupport::Answered);
        assert_eq!(status.dropped, [0x40000]);
    }

    /// A PGN the application sends from our address joins the
    /// Transmit list; one sent as another device, one dropped for
    /// want of an address, or with learning off, does not.
    #[test]
    fn transmitted_pgns_are_learned() {
        fn send(dev: &mut NmeaDevice, pgn: u32, src: u8) {
            let mut tx_buf = TxBuffer::new();
            let (frames_tx, _frames_rx) = mpsc::channel();
            let mut bus = Bus {
                tx_buf: &mut tx_buf,
                frames_tx: &frames_tx,
            };
            let frame = RawFrame {
                timestamp: None,
                prio: 6,
                pgn,
                src,
                dst: ADDR_GLOBAL,
                data: vec![0; 8].into(),
            };
            dispatch_cmd(&mut bus, dev, WriterCmd::Frame(frame));
        }
        // Only frames that go out as us are learned, which needs an
        // address of our own: scan, then claim, each to its deadline.
        fn claimed(config: &Config) -> NmeaDevice {
            let mut dev = NmeaDevice::new(config, Box::new(NoStats));
            let mut tx_buf = TxBuffer::new();
            let (frames_tx, _frames_rx) = mpsc::channel();
            let mut bus = Bus {
                tx_buf: &mut tx_buf,
                frames_tx: &frames_tx,
            };
            dev.start(&mut bus);
            for _ in 0..2 {
                let deadline = dev.claim.deadline();
                dev.tick(&mut bus, deadline);
            }
            assert!(dev.claim.is_claimed());
            dev
        }

        let mut dev = NmeaDevice::new(&Config::default(), Box::new(NoStats));
        send(&mut dev, 127508, 0);
        assert_eq!(dev.tx_pgns, TX_PGN_LIST, "dropped while unclaimed");

        let mut dev = claimed(&Config::default());
        send(&mut dev, 127508, 0);
        send(&mut dev, 127508, 0);
        send(&mut dev, 130824, 24); // as the impersonated H5000
        send(&mut dev, 0x40100, 0); // synthetic, never on the wire
        send(&mut dev, 0x2_0000, 0); // beyond the 17-bit PGN range
        assert_eq!(&dev.tx_pgns[TX_PGN_LIST.len()..], [127508]);

        let config = Config {
            learn_tx_pgns: false,
            ..Default::default()
        };
        let mut dev = claimed(&config);
        send(&mut dev, 127508, 0);
        assert_eq!(dev.tx_pgns, TX_PGN_LIST);
    }

    /// The gateway announces itself as a PC Gateway (130) in the
    /// Inter-Intranetwork Device class (25), with the configured
    /// manufacturer and unique id packed into the ISO NAME.
    #[test]
    fn the_iso_name_carries_function_class_and_unique() {
        let config = Config {
            unique: 0x12345,
            manufacturer: 717,
            system_instance: 3,
            ..Default::default()
        };
        let name = build_name(&config);
        assert_eq!((name & 0x1F_FFFF) as u32, 0x12345, "unique id");
        assert_eq!(((name >> 21) & 0x7FF) as u16, 717, "manufacturer");
        assert_eq!(((name >> 40) & 0xFF) as u8, 130, "device function");
        assert_eq!(((name >> 49) & 0x7F) as u8, 25, "device class");
        assert_eq!(((name >> 56) & 0x0F) as u8, 3, "system instance");
        assert_eq!(((name >> 60) & 0x07) as u8, 4, "marine industry group");
        assert_eq!(name >> 63, 1, "arbitrary-address-capable");
    }

    /// `unique == 0` means "derive one", so two default configs on
    /// one host agree — and neither claims unique id 0.
    #[test]
    fn a_zero_unique_is_derived_from_the_machine() {
        let a = build_name(&Config::default());
        let b = build_name(&Config::default());
        assert_eq!(a, b, "stable across calls on one host");
        assert_ne!(a & 0x1F_FFFF, 0, "a derived id is not zero");
    }

    /// An explicit `unique` overrides the derived one, which is how
    /// two gateways on one host avoid colliding.
    #[test]
    fn an_explicit_unique_overrides_the_derived_one() {
        let derived = build_name(&Config::default());
        let explicit = build_name(&Config {
            unique: 7,
            ..Default::default()
        });
        assert_eq!(explicit & 0x1F_FFFF, 7);
        assert_ne!(derived, explicit);
    }

    /// A payload at the fast-packet ceiling chunks into 32 frames:
    /// 6 bytes in the first, 7 in each of the other 31.
    #[test]
    fn a_maximum_fast_packet_payload_chunks_completely() {
        let data: Vec<u8> = (0..=255u8).cycle().take(223).collect();
        let frames = send(&mut TxBuffer::new(), 126996, &data);
        assert_eq!(frames.len(), 32);
        let carried: Vec<u8> = frames
            .iter()
            .enumerate()
            .flat_map(|(i, f)| if i == 0 { &f[2..] } else { &f[1..] }.to_vec())
            .take(223)
            .collect();
        assert_eq!(carried, data);
    }

    /// The 3-bit sequence counter wraps after eight sends rather
    /// than bleeding into the frame-index nibble.
    #[test]
    fn the_fast_packet_sequence_wraps_after_eight() {
        let mut tx_buf = TxBuffer::new();
        let seqs: Vec<u8> = (0..9)
            .map(|_| send(&mut tx_buf, 126996, &[0xaa; 10])[0][0])
            .collect();
        assert_eq!(
            seqs,
            vec![0x00, 0x20, 0x40, 0x60, 0x80, 0xa0, 0xc0, 0xe0, 0x00]
        );
    }

    /// Sequence counters are per (PGN, src), so interleaved sends of
    /// different PGNs don't advance each other's.
    #[test]
    fn sequence_counters_are_per_pgn() {
        let mut tx_buf = TxBuffer::new();
        send(&mut tx_buf, 126996, &[0xaa; 10]);
        send(&mut tx_buf, 126998, &[0xbb; 10]);
        let second_126996 = send(&mut tx_buf, 126996, &[0xaa; 10]);
        assert_eq!(second_126996[0][0], 0x20, "126998 must not advance 126996");
    }

    /// PGN 127508 (Battery Status) is single-frame even at exactly
    /// 8 bytes of payload: no fast-packet shell.
    #[test]
    fn single_frame_pgn_goes_out_verbatim() {
        let data: Vec<u8> = (1..=8).collect();
        let frames = send(&mut TxBuffer::new(), 127508, &data);
        assert_eq!(frames, vec![data]);
    }

    /// A fast-packet payload splits per ISO 11783-3: 6 bytes after
    /// the (seq|index, length) header, then 7 per continuation,
    /// last frame padded to 8 with 0xff.
    #[test]
    fn fast_packet_frame_layout_is_pinned() {
        let data: Vec<u8> = (1..=17).collect();
        let frames = send(&mut TxBuffer::new(), 126996, &data);
        assert_eq!(
            frames,
            vec![
                vec![0x00, 17, 1, 2, 3, 4, 5, 6],
                vec![0x01, 7, 8, 9, 10, 11, 12, 13],
                vec![0x02, 14, 15, 16, 17, 0xff, 0xff, 0xff],
            ]
        );
    }

    /// A fast-packet PGN keeps its shell even when the payload fits
    /// a single CAN frame: receivers key framing on the PGN type.
    #[test]
    fn short_fast_packet_payload_keeps_the_shell() {
        let frames = send(&mut TxBuffer::new(), 126208, &[9, 8, 7, 6, 5, 4]);
        assert_eq!(frames, vec![vec![0x00, 6, 9, 8, 7, 6, 5, 4]]);
    }

    /// Consecutive sends of the same (pgn, src) advance the 3-bit
    /// sequence counter in the upper bits of byte 0.
    #[test]
    fn fast_seq_advances_between_sends() {
        let mut tx_buf = TxBuffer::new();
        let first = send(&mut tx_buf, 126996, &[0xaa; 10]);
        let second = send(&mut tx_buf, 126996, &[0xaa; 10]);
        assert_eq!(first[0][0], 0x00);
        assert_eq!(second[0][0], 0x20);
    }

    fn j1939_config() -> Config {
        Config {
            protocol: BusProtocol::J1939,
            ..Default::default()
        }
    }

    /// On J1939, a data page 1 PGN that is fast-packet on NMEA 2000
    /// (130816) goes out as one plain frame, and nothing is ever
    /// fast-packet framed.
    #[test]
    fn j1939_sends_single_frames_only() {
        let mut tx_buf = TxBuffer::with_protocol(BusProtocol::J1939);
        let data: Vec<u8> = (1..=8).collect();
        assert_eq!(send(&mut tx_buf, 130816, &data), vec![data]);
        assert_eq!(
            send(&mut tx_buf, 126208, &[9, 8, 7, 6, 5, 4]),
            vec![vec![9, 8, 7, 6, 5, 4]],
            "no fast-packet shell on J1939"
        );
    }

    /// A J1939 payload over 8 bytes to global goes out as an ISO TP
    /// BAM: the announcement at once, the data packets as the
    /// worker polls the sender.
    #[test]
    fn j1939_sends_long_messages_over_iso_tp() {
        let mut tx_buf = TxBuffer::with_protocol(BusProtocol::J1939);
        let data: Vec<u8> = (1..=12).collect();
        let announced = send(&mut tx_buf, 130816, &data);
        assert_eq!(
            announced,
            vec![vec![32, 12, 0, 2, 0xFF, 0x00, 0xFF, 0x01]],
            "BAM for PGN 130816, 12 bytes in 2 packets"
        );
        let id = tx_buf.queue.back().unwrap().id;
        assert_eq!((id >> 8) & 0x3FFFF, 0xECFF, "TP.CM to global");
        let at = tx_buf.tp.next_deadline().unwrap();
        let before = tx_buf.queue.len();
        tx_buf.poll_tp(at);
        tx_buf.poll_tp(at + crate::engine::iso_tp::BAM_GAP_MS);
        let packets: Vec<Vec<u8>> = tx_buf
            .queue
            .iter()
            .skip(before)
            .map(|f| f.data.to_vec())
            .collect();
        assert_eq!(
            packets,
            vec![
                vec![1, 1, 2, 3, 4, 5, 6, 7],
                vec![2, 8, 9, 10, 11, 12, 0xFF, 0xFF]
            ]
        );
        assert!(tx_buf.tp.is_idle());
    }

    /// A J1939 node claims in the Global industry group and runs none
    /// of the NMEA 2000 housekeeping: no Heartbeat, no learned
    /// Transmit list.
    #[test]
    fn j1939_node_has_no_nmea2000_housekeeping() {
        let name = build_name(&j1939_config());
        assert_eq!(((name >> 60) & 0x07) as u8, 0, "global industry group");
        let dev = NmeaDevice::new(&j1939_config(), Box::new(NoStats));
        assert_eq!(dev.heartbeat_interval, 0);
        assert!(!dev.learn_tx_pgns);
    }

    /// An addressed ISO Request for Product Information is NAKed on
    /// J1939 (it is an NMEA 2000 PGN), where NMEA 2000 answers it.
    #[test]
    fn j1939_naks_a_product_information_request() {
        let mut dev = NmeaDevice::new(&j1939_config(), Box::new(NoStats));
        let mut tx_buf = TxBuffer::with_protocol(BusProtocol::J1939);
        let (frames_tx, frames_rx) = mpsc::channel();
        let mut bus = Bus {
            tx_buf: &mut tx_buf,
            frames_tx: &frames_tx,
        };
        dev.start(&mut bus);
        // Scan window, then the claim window, each run to its deadline.
        for _ in 0..2 {
            let deadline = dev.claim.deadline();
            dev.tick(&mut bus, deadline);
        }
        assert!(dev.claim.is_claimed());
        while frames_rx.try_recv().is_ok() {}

        let p = PGN_PRODUCT_INFO.to_le_bytes();
        dev.on_request(&mut bus, 0x21, dev.addr(), &[p[0], p[1], p[2]]);
        let sent: Vec<RawFrame> = frames_rx.try_iter().collect();
        assert_eq!(sent.len(), 1, "{sent:?}");
        assert_eq!(sent[0].pgn, 59392, "ISO Acknowledgement");
        assert_eq!(sent[0].data[0], 1, "NAK");
    }

    /// ISO 11783-5 allows only address claims from the null address:
    /// a frame sent as us (src 0) before we own an address is dropped,
    /// not put on the wire from 254. One impersonating another device
    /// still goes out, and an explicit 254 only for a claim.
    #[test]
    fn drops_own_frames_until_an_address_is_claimed() {
        let config = Config::default();
        let mut dev = NmeaDevice::new(&config, Box::new(NoStats));
        let mut tx_buf = TxBuffer::new();
        let (frames_tx, _frames_rx) = mpsc::channel();
        let mut bus = Bus {
            tx_buf: &mut tx_buf,
            frames_tx: &frames_tx,
        };
        let frame = |src| WriterCmd::Frame(RawFrame::new(None, 2, 127250, src, 255, [0; 8]));
        let sources = |bus: &mut Bus<'_>| -> Vec<u8> {
            bus.tx_buf
                .queue
                .drain(..)
                .map(|f| (f.id & 0xff) as u8)
                .collect()
        };

        dev.start(&mut bus);
        sources(&mut bus);
        dispatch_cmd(&mut bus, &mut dev, frame(0));
        dispatch_cmd(&mut bus, &mut dev, frame(ADDR_GLOBAL));
        assert_eq!(sources(&mut bus), Vec::<u8>::new(), "scanning");

        let deadline = dev.claim.deadline();
        dev.tick(&mut bus, deadline);
        sources(&mut bus);
        dispatch_cmd(&mut bus, &mut dev, frame(0));
        assert_eq!(sources(&mut bus), Vec::<u8>::new(), "claim settling");
        dispatch_cmd(&mut bus, &mut dev, frame(52));
        assert_eq!(sources(&mut bus), vec![52], "impersonation goes out");
        // An explicit null source: only a claim, or a request for one.
        dispatch_cmd(&mut bus, &mut dev, frame(ADDR_NULL));
        assert_eq!(sources(&mut bus), Vec::<u8>::new(), "not from 254");
        let claim = RawFrame::new(None, 6, PGN_ISO_ADDRESS_CLAIM, ADDR_NULL, 255, [0xff; 8]);
        dispatch_cmd(&mut bus, &mut dev, WriterCmd::Frame(claim));
        let p = PGN_ISO_ADDRESS_CLAIM.to_le_bytes();
        let request = RawFrame::new(None, 6, PGN_ISO_REQUEST, ADDR_NULL, 255, [p[0], p[1], p[2]]);
        dispatch_cmd(&mut bus, &mut dev, WriterCmd::Frame(request));
        assert_eq!(sources(&mut bus), vec![ADDR_NULL, ADDR_NULL]);

        let deadline = dev.claim.deadline();
        dev.tick(&mut bus, deadline);
        assert!(dev.claim.is_claimed());
        sources(&mut bus);
        dispatch_cmd(&mut bus, &mut dev, frame(0));
        assert_eq!(sources(&mut bus), vec![dev.addr()]);
        assert_eq!(dev.dropped_unclaimed, 0, "reset once sending again");
    }
}
