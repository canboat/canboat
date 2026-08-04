// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Linux SocketCAN device adapter — a full NMEA 2000 bus participant.
//!
//! Wraps a raw `CAN_RAW` socket on a Linux SocketCAN interface and runs
//! the ISO 11783-5 address-claim handshake, heartbeat scheduler, and
//! ISO Request / Group Function responders, all on a single internal
//! worker thread. Outbound writes go through a buffered ring drained
//! one frame per `POLLOUT` wakeup so a shallow netdev qdisc never
//! stalls RX or the claim timers.
//!
//! Mirrors the bus-protocol layer of `canboat/socketcan-serial/socketcan-serial.c`.
//! Both the standalone `socketcan-serial` binary and the integrated
//! `canboat-pipeline` use this module via [`run`], which returns a
//! standard [`super::DeviceHandle`].
//!
//! `src` convention on outbound frames sent via `DeviceHandle::send_frame`:
//! `frame.src == 0` is treated as "use my claim address" and rewritten to
//! the currently-claimed source; any other value is sent on the wire as
//! given. This matches canboat C's stdin pump and lets quirk
//! synthesisers (e.g. the SCX-20 PGN 126996 fabrication) impersonate
//! other nodes by passing the impersonated `src` explicitly.
//!
//! Linux-only; on every other platform `run` returns an
//! `io::ErrorKind::Unsupported` error and `Config` is a placeholder.

#[cfg(target_os = "linux")]
pub use imp::run;

pub use config::Config;

/// Sentinel value stored in the claim-address atom when the gateway
/// hasn't successfully claimed an address yet. Callers (e.g.
/// `canboat-pipeline`'s CSV-port injector) treat this as "no rewrite
/// available, leave the caller's `src` alone".
pub const CLAIM_UNCLAIMED: u8 = 254;

mod config {
    /// Bus-participant configuration. All fields have sensible defaults
    /// via [`Config::default`]; tweak only what differs from canboat C
    /// `socketcan-serial`'s built-in defaults.
    #[derive(Debug, Clone)]
    pub struct Config {
        /// Preferred source address to claim. Defaults to 0.
        pub address: u8,
        /// Unique number for the ISO NAME's identity field. 0 = derive
        /// from the machine id (stable per-host across restarts).
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
        /// opening the socket it brings the link down, sets the NMEA 2000
        /// bitrate (always 250 kbit/s) with bus-off auto-recovery, and brings
        /// it up — retrying to ride out controllers that intermittently fail
        /// to configure at boot (notably the MCP2515's "didn't enter config
        /// mode"). `false` leaves the interface untouched, assuming it was
        /// configured externally (e.g. a systemd `ip link set … up` unit).
        pub configure_link: bool,
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
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn run(
    _iface: &str,
    _config: Config,
    _claim_addr: std::sync::Arc<std::sync::atomic::AtomicU8>,
) -> std::io::Result<super::DeviceHandle> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "SocketCAN is only available on Linux",
    ))
}

#[cfg(target_os = "linux")]
mod imp {
    use std::collections::{HashMap, VecDeque};
    use std::os::fd::AsRawFd;
    use std::sync::atomic::{AtomicU8, Ordering};
    use std::sync::{Arc, mpsc};
    use std::thread;
    use std::time::{SystemTime, UNIX_EPOCH};

    use canboat_core::format::ikonvert::{NetworkStatus, build_network_status};
    use canboat_core::format::{days_to_ymd, iso11783_compose, iso11783_decompose};
    use canboat_core::frame::RawFrame;
    use canboat_core::{ADDR_GLOBAL, ADDR_NULL, FramePacketType, Reassembled, Reassembler};
    use socketcan::{CanInterface, CanSocket, EmbeddedFrame, ExtendedId, Socket};

    use super::config::Config;
    use crate::address_claim::{AddressClaim, ClaimState};
    use crate::device::{DeviceHandle, WriterCmd, from_parts};
    use crate::fastpacket;
    use crate::nmea_responder::{self, ProductInfo};

    const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
    const CAN_ERR_FLAG: u32 = 0x2000_0000;
    const CAN_EFF_FLAG: u32 = 0x8000_0000;

    /// How often to emit the synthetic `NMEA 2000 gateway: network
    /// status` PGN (262400) into the upstream `frames_tx` channel.
    /// Mirrors the ~1 Hz cadence the iKonvert pushes its `$PDGY`
    /// heartbeat at; 5 s is plenty for human-visible status without
    /// drowning the snapshot port in identical rows.
    const NETWORK_STATUS_INTERVAL_MS: u64 = 5_000;

    /// Fallback CAN bitrate when `/sys/class/net/<iface>/can_bittiming/
    /// bitrate` isn't readable. NMEA 2000 is fixed at 250 kbit/s so
    /// this is the right default for ~every real bus.
    const FALLBACK_BITRATE_BPS: u32 = 250_000;

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
    struct LoadSample {
        rx_bytes: u64,
        tx_bytes: u64,
        rx_packets: u64,
        tx_packets: u64,
        at_ms: u64,
    }

    const PGN_ISO_ACK: u32 = 59392;
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

    // PGNs we originate / consume, reported via PGN 126464 on request.
    const TX_PGN_LIST: [u32; 7] = [
        PGN_ISO_ACK,
        PGN_ISO_REQUEST,
        PGN_ISO_ADDRESS_CLAIM,
        PGN_GROUP_FUNCTION,
        PGN_PGN_LIST,
        PGN_HEARTBEAT,
        PGN_PRODUCT_INFO,
    ];
    const RX_PGN_LIST: [u32; 3] = [PGN_ISO_REQUEST, PGN_ISO_ADDRESS_CLAIM, PGN_GROUP_FUNCTION];

    /// Hold outbound frames until the kernel CAN qdisc has room. The
    /// worker thread drains one frame per POLLOUT wakeup so a single
    /// fast-packet burst never starves RX or the claim timers. ~1 s of
    /// bus time at 250 kbit/s with the default txqueuelen.
    const TX_BUFFER_CAPACITY: usize = 1024;

    /// Worst-case latency for a user-side `send_frame` to reach the bus.
    /// The worker drains `cmd_rx` after each `poll()` wakeup; a 100 ms
    /// cap keeps stdin / quirk-synthesiser sends prompt without burning
    /// CPU on idle.
    const MAX_POLL_MS: u64 = 100;

    fn now_ms() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0)
    }

    /// `YYYY-MM-DDTHH:MM:SS.mmmZ` from epoch milliseconds; matches
    /// canboat C's `fmtTimestamp`.
    fn format_iso(ms: u64) -> String {
        let secs = (ms / 1000) as i64;
        let millis = ms % 1000;
        let days = secs.div_euclid(86_400);
        let tod = secs.rem_euclid(86_400);
        let (y, mo, d) = days_to_ymd(days);
        let (h, mi, s) = (tod / 3600, (tod % 3600) / 60, tod % 60);
        format!("{y:04}-{mo:02}-{d:02}T{h:02}:{mi:02}:{s:02}.{millis:03}Z")
    }

    /// Build the 64-bit ISO NAME from a [`Config`].
    fn build_name(config: &Config) -> u64 {
        let unique = if config.unique != 0 {
            config.unique
        } else {
            // Per-machine, stable across restarts. Two CANboat gateways
            // on the same host would collide; pass `--unique N` (or
            // `Config.unique`) to disambiguate.
            canboat_core::os::get_machine_id() as u32
        };
        // PC Gateway (130) / Inter-Intranetwork Device (25); Marine industry
        // group and arbitrary-address-capable are the builder's defaults.
        crate::name::Name::new(config.manufacturer, unique)
            .device_function(130)
            .device_class(25)
            .system_instance(config.system_instance)
            .to_u64()
    }

    /// Outbound ring drained one frame per writability wakeup. Lives on
    /// the worker thread; not shared, no locking.
    struct TxBuffer {
        queue: VecDeque<socketcan::CanFrame>,
        overflowed: usize,
        /// Per-(pgn, src) fast-packet sequence counter, mod 8. The
        /// upper 3 bits of every fast-packet `frame[0]` carry this
        /// value; a strict receiver uses it to distinguish back-to-
        /// back instances of the same PGN from continuations of an
        /// in-flight reassembly. We previously left it at 0, so a
        /// receiver could fold two consecutive Product-Info responses
        /// into one corrupted reassembly.
        fast_seq: HashMap<(u32, u8), u8>,
    }

    impl TxBuffer {
        fn new() -> Self {
            Self {
                queue: VecDeque::with_capacity(64),
                overflowed: 0,
                fast_seq: HashMap::new(),
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
            if self.queue.len() >= TX_BUFFER_CAPACITY {
                if self.overflowed == 0 {
                    log::error!(
                        "CAN TX buffer full ({TX_BUFFER_CAPACITY} frames), dropping outbound frame"
                    );
                }
                self.overflowed += 1;
                return;
            }
            let Some(id) = ExtendedId::new(can_id & CAN_EFF_MASK) else {
                return;
            };
            let Some(frame) = socketcan::CanFrame::new(socketcan::Id::Extended(id), data) else {
                log::error!("could not build CAN frame for id {can_id:#x}");
                return;
            };
            self.queue.push_back(frame);
        }

        fn is_empty(&self) -> bool {
            self.queue.is_empty()
        }
    }

    /// Bundles the resources every send-path needs: the socket, the TX
    /// ring, and the consumer-side channel that receives a `RawFrame`
    /// echo of every self-generated outbound. Passed by `&mut` through
    /// the NmeaDevice methods so they stay free-functions in spirit.
    struct Bus<'a> {
        tx_buf: &'a mut TxBuffer,
        frames_tx: &'a mpsc::Sender<RawFrame>,
    }

    impl<'a> Bus<'a> {
        /// Enqueue a self-generated outbound PGN. Splits >8-byte
        /// payloads into fast-packet chunks for the wire; when `emit`
        /// is true the consumer also sees one coalesced `RawFrame` on
        /// `frames_tx` (same contract as inbound — never split).
        fn send_pgn(&mut self, prio: u8, pgn: u32, src: u8, dst: u8, data: &[u8], emit: bool) {
            let can_id = iso11783_compose(prio, pgn, src, dst);
            // Single vs fast-packet is decided strictly from the PGN
            // type table generated at build time from canboat.json.
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
            let is_fast = fastpacket::packet_type(pgn) == FramePacketType::Fast;
            if !is_fast {
                self.tx_buf.push(can_id, data);
            } else {
                // Per ISO 11783-3: every fast-packet CAN frame is
                // exactly 8 bytes; the last chunk is padded with 0xff.
                // The first byte is `(seq << 5) | index`, with `seq`
                // incrementing per-(pgn, src) instance so consecutive
                // first frames are distinguishable from continuations.
                let seq = self.tx_buf.next_fast_seq(pgn, src);
                let mut index: u8 = 0;
                let mut remaining = data.len();
                let mut offset = 0;
                while remaining > 0 {
                    let mut frame = [0xffu8; 8];
                    frame[0] = (seq << 5) | (index & 0x1f);
                    let chunk;
                    let payload_start;
                    if index == 0 {
                        frame[1] = data.len() as u8;
                        chunk = remaining.min(6);
                        payload_start = 2;
                    } else {
                        chunk = remaining.min(7);
                        payload_start = 1;
                    }
                    frame[payload_start..payload_start + chunk]
                        .copy_from_slice(&data[offset..offset + chunk]);
                    self.tx_buf.push(can_id, &frame[..]);
                    offset += chunk;
                    remaining -= chunk;
                    index = index.wrapping_add(1);
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
    /// handshake itself lives in [`crate::address_claim`] and is shared
    /// with the server's Motion-Sensor impersonation quirk.
    struct NmeaDevice {
        /// The shared address-claim state machine (owns NAME, address,
        /// state, and the used-address table).
        claim: AddressClaim,
        heartbeat_interval: u64, // ms, 0 disables
        heartbeat_seq: u8,
        next_heartbeat: u64,    // ms
        last_product_info: u64, // ms; rate-limit broadcast bursts
        model_version: &'static str,
        // -- NMEA 2000 gateway: network status (PGN 262400) emission --
        /// Interface name passed to `socketcan::CanSocket::open`, kept
        /// so the network-status tick can read kernel CAN counters via
        /// `/sys/class/net/<iface>/statistics/`.
        iface: String,
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
        /// `/sys/class/net/<iface>/can_bittiming/bitrate`. NMEA 2000
        /// is fixed at 250 kbit/s so the fallback covers the common
        /// case if the kernel hasn't filled in the bittiming yet.
        bitrate_bps: u32,
        /// Most recent `LoadSample`, or `None` before the first
        /// network-status emission. Used to compute the bytes/packets
        /// delta over the time between the previous and current emit.
        /// First emit reports `load_pct = None` (no baseline yet).
        prev_load_sample: Option<LoadSample>,
    }

    /// Send each frame the address-claim state machine produced onto the
    /// bus (with a local echo — same contract as `send_pgn(..., true)`).
    fn emit(bus: &mut Bus<'_>, frames: Vec<RawFrame>) {
        for f in frames {
            bus.send_pgn(f.prio, f.pgn, f.src, f.dst, &f.data, true);
        }
    }

    impl NmeaDevice {
        fn new(config: &Config, iface: &str) -> Self {
            let name = build_name(config);
            let claim = if config.no_claim {
                AddressClaim::disabled(name)
            } else {
                // Our NAME is arbitrary-address-capable (`build_name` sets
                // the bit), so we yield and move on a lost conflict.
                AddressClaim::new(name, config.address, true)
            };
            Self {
                claim,
                heartbeat_interval: config.heartbeat_ms,
                heartbeat_seq: 0,
                next_heartbeat: 0,
                last_product_info: 0,
                model_version: config.model_version.unwrap_or(DEFAULT_MODEL_VERSION),
                iface: iface.to_string(),
                start_ms: now_ms(),
                next_network_status: 0,
                seen_addrs: [false; 256],
                bitrate_bps: read_bitrate_bps(iface),
                prev_load_sample: None,
            }
        }

        /// Our claimed source address, or [`ADDR_NULL`] before we own one.
        /// Device responders below only run once claimed, so this resolves
        /// to the real address for them.
        fn addr(&self) -> u8 {
            self.claim.address().unwrap_or(ADDR_NULL)
        }

        /// Our 64-bit ISO NAME.
        fn name(&self) -> u64 {
            self.claim.name()
        }

        /// Kick off the claim handshake (scan → claim).
        fn start(&mut self, bus: &mut Bus<'_>) {
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

            match requested {
                PGN_PRODUCT_INFO => self.send_product_info(bus),
                PGN_PGN_LIST => self.send_pgn_list(bus, src),
                PGN_HEARTBEAT => self.send_heartbeat(bus),
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

        // PGN List (Transmit and Receive), PGN 126464: one message per list.
        fn send_pgn_list(&self, bus: &mut Bus<'_>, dst: u8) {
            emit(
                bus,
                nmea_responder::pgn_list_frames(self.addr(), dst, &TX_PGN_LIST, &RX_PGN_LIST),
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

        fn tick(&mut self, bus: &mut Bus<'_>, now: u64) {
            // Advance the shared claim state machine and put any claim
            // frames it produced on the wire.
            let was_claimed = self.claim.is_claimed();
            emit(bus, self.claim.tick(now));
            // The moment we take ownership: announce ourselves once and
            // arm the heartbeat / network-status timers.
            if !was_claimed && self.claim.is_claimed() {
                log::info!("Address {} claimed", self.addr());
                self.send_product_info(bus);
                if self.heartbeat_interval > 0 {
                    self.next_heartbeat = now + self.heartbeat_interval;
                }
                // First network-status drop one interval after claim,
                // so a downstream snapshot client has the data even
                // before the first bus traffic.
                self.next_network_status = now + NETWORK_STATUS_INTERVAL_MS;
            }
            if self.claim.is_claimed() && self.heartbeat_interval > 0 && now >= self.next_heartbeat
            {
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
        /// the wire — it's a BEM PGN by design. Errors / rejected-TX
        /// come from `/sys/class/net/<iface>/statistics/`; everything
        /// else from in-process counters.
        fn emit_network_status(&mut self, bus: &mut Bus<'_>, now: u64) {
            let uptime_s = ((now - self.start_ms) / 1000) as u32;
            let device_count = self
                .seen_addrs
                .iter()
                .filter(|seen| **seen)
                .count()
                .min(u8::MAX as usize) as u8;
            let errors = read_sysfs_counter(&self.iface, "rx_errors");
            let rejected_tx = read_sysfs_counter(&self.iface, "tx_dropped");
            // Load: delta against the previous sample. First emit has
            // no baseline, so load_pct stays None and the canboat
            // sentinel rides through; baseline shifts forward each
            // call.
            let curr_sample = read_load_sample(&self.iface, now);
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
        fn send_heartbeat(&mut self, bus: &mut Bus<'_>) {
            emit(
                bus,
                vec![nmea_responder::heartbeat_frame(
                    self.addr(),
                    self.heartbeat_seq,
                    self.heartbeat_interval,
                )],
            );
            self.heartbeat_seq = if self.heartbeat_seq >= 252 {
                0
            } else {
                self.heartbeat_seq + 1
            };
        }
    }

    /// Read a kernel-maintained counter from
    /// `/sys/class/net/<iface>/statistics/<name>`. Returns `None` on
    /// any error (interface gone, file missing, parse fail) so the
    /// network-status emitter degrades to the "no data" sentinel
    /// rather than failing loudly. Cheap: sysfs is a virtual fs and
    /// each read is a handful of bytes; called once per emission
    /// interval (default 5 s).
    fn read_sysfs_counter(iface: &str, name: &str) -> Option<u32> {
        let path = format!("/sys/class/net/{iface}/statistics/{name}");
        let raw = std::fs::read_to_string(path).ok()?;
        raw.trim().parse::<u64>().ok().map(|v| v as u32)
    }

    /// Read the CAN bus bitrate (bits/s) from
    /// `/sys/class/net/<iface>/can_bittiming/bitrate`. The file
    /// exists for every SocketCAN device and contains a decimal
    /// integer (e.g. `250000`). Returns `FALLBACK_BITRATE_BPS` when
    /// missing, unreadable, or 0 — most production NMEA 2000 buses
    /// are fixed at 250 kbit/s.
    fn read_bitrate_bps(iface: &str) -> u32 {
        let path = format!("/sys/class/net/{iface}/can_bittiming/bitrate");
        let raw = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(_) => return FALLBACK_BITRATE_BPS,
        };
        match raw.trim().parse::<u32>() {
            Ok(0) | Err(_) => FALLBACK_BITRATE_BPS,
            Ok(v) => v,
        }
    }

    /// Take one read of `(rx_bytes, tx_bytes, rx_packets, tx_packets)`
    /// from sysfs, timestamped with `now_ms`. Returns `None` if any
    /// of the counters are unreadable; the emitter then reports
    /// `load_pct = None` and the canboat sentinel survives.
    fn read_load_sample(iface: &str, now_ms: u64) -> Option<LoadSample> {
        Some(LoadSample {
            rx_bytes: read_sysfs_counter(iface, "rx_bytes")? as u64,
            tx_bytes: read_sysfs_counter(iface, "tx_bytes")? as u64,
            rx_packets: read_sysfs_counter(iface, "rx_packets")? as u64,
            tx_packets: read_sysfs_counter(iface, "tx_packets")? as u64,
            at_ms: now_ms,
        })
    }

    /// CAN bus load percentage from two samples and the bus bitrate.
    /// Accounts for the per-frame CAN protocol overhead and a flat
    /// 20% bit-stuffing inflation.
    fn compute_load_pct(prev: LoadSample, curr: LoadSample, bitrate_bps: u32) -> Option<u8> {
        let dt_ms = curr.at_ms.saturating_sub(prev.at_ms);
        if dt_ms == 0 || bitrate_bps == 0 {
            return None;
        }
        // Kernel counters are monotonic u64 internally — even though
        // sysfs widens to u32 here, wrapping_sub keeps a wrap from
        // showing up as 4 GiB of phantom traffic on a long-running
        // gateway.
        let d_bytes = curr
            .rx_bytes
            .wrapping_sub(prev.rx_bytes)
            .saturating_add(curr.tx_bytes.wrapping_sub(prev.tx_bytes));
        let d_packets = curr
            .rx_packets
            .wrapping_sub(prev.rx_packets)
            .saturating_add(curr.tx_packets.wrapping_sub(prev.tx_packets));
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

    /// Try to write the oldest queued frame. Returns true if a frame was
    /// actually delivered; false on empty queue or kernel backpressure
    /// (the frame stays queued, retried on the next wakeup).
    fn tx_drain_one(sock: &CanSocket, tx_buf: &mut TxBuffer) -> bool {
        let Some(frame) = tx_buf.queue.front().cloned() else {
            return false;
        };
        match sock.write_frame(&frame) {
            Ok(()) => {
                tx_buf.queue.pop_front();
                if tx_buf.overflowed > 0 && tx_buf.queue.len() < TX_BUFFER_CAPACITY / 2 {
                    log::info!(
                        "CAN TX buffer recovered ({} frames had been dropped)",
                        tx_buf.overflowed
                    );
                    tx_buf.overflowed = 0;
                }
                true
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.raw_os_error() == Some(libc::ENOBUFS) =>
            {
                false
            }
            Err(e) => {
                log::error!("write to CAN: {e} (dropping frame)");
                tx_buf.queue.pop_front();
                false
            }
        }
    }

    /// One inbound CAN frame plus its kernel SO_TIMESTAMP: extended
    /// id, payload length, payload bytes, and the timestamp in ms
    /// since the epoch (0 if the cmsg wasn't present).
    struct RecvFrame {
        id: u32,
        dlc: usize,
        data: [u8; 8],
        when_ms: u64,
    }

    /// How many CAN frames one `recvmmsg` syscall reads at most. A busy
    /// NMEA 2000 bus delivers fast-packet PGNs in back-to-back bursts;
    /// draining a whole burst per syscall (instead of one `recvmsg` per
    /// frame) is where the RX-path syscall savings come from.
    const RX_BATCH: usize = 64;

    /// Reusable `recvmmsg` scratch: `RX_BATCH` CAN-frame + control-buffer
    /// slots, plus the `iovec`/`mmsghdr` arrays that point into them.
    /// Boxed and never moved after construction so those interior raw
    /// pointers stay valid across calls; [`RxBatch::prepare`] re-wires
    /// them (and resets the kernel-written lengths) before every call.
    struct RxBatch {
        frames: [[u8; 16]; RX_BATCH], // struct can_frame per slot
        ctrls: [[u8; 64]; RX_BATCH],  // SO_TIMESTAMP cmsg per slot
        iovecs: [libc::iovec; RX_BATCH],
        msgs: [libc::mmsghdr; RX_BATCH],
    }

    impl RxBatch {
        fn new() -> Box<RxBatch> {
            // SAFETY: `iovec`/`mmsghdr` are C POD; an all-zero state is
            // valid (null pointers), and `prepare` wires every pointer
            // and length before the buffer is ever handed to the kernel.
            unsafe { Box::new(std::mem::zeroed()) }
        }

        /// Point each `mmsghdr` at its slot's frame + control buffers and
        /// reset the fields the kernel overwrites (`msg_controllen`,
        /// `msg_flags`, `msg_len`). Must run before each `recvmmsg`.
        fn prepare(&mut self) {
            for i in 0..RX_BATCH {
                self.iovecs[i] = libc::iovec {
                    iov_base: self.frames[i].as_mut_ptr() as *mut libc::c_void,
                    iov_len: self.frames[i].len(),
                };
                let hdr = &mut self.msgs[i].msg_hdr;
                hdr.msg_name = std::ptr::null_mut();
                hdr.msg_namelen = 0;
                hdr.msg_iov = &mut self.iovecs[i];
                hdr.msg_iovlen = 1;
                hdr.msg_control = self.ctrls[i].as_mut_ptr() as *mut libc::c_void;
                hdr.msg_controllen = self.ctrls[i].len() as _;
                hdr.msg_flags = 0;
                self.msgs[i].msg_len = 0;
            }
        }

        /// Decode slot `i` (valid only for `i < ` the count a preceding
        /// `recv_batch` returned) into a [`RecvFrame`]. `None` on a short
        /// datagram — anything below a full `struct can_frame`.
        fn parse(&self, i: usize) -> Option<RecvFrame> {
            if (self.msgs[i].msg_len as usize) < 16 {
                return None;
            }
            let raw = &self.frames[i];
            let mut when = 0u64;
            // SAFETY: `msg_hdr` describes the control buffer the kernel
            // just filled for this slot; walk it for the RX timestamp.
            unsafe {
                let msg: *const libc::msghdr = &self.msgs[i].msg_hdr;
                let mut cmsg = libc::CMSG_FIRSTHDR(msg);
                while !cmsg.is_null() {
                    let c = &*cmsg;
                    if c.cmsg_level == libc::SOL_SOCKET && c.cmsg_type == libc::SCM_TIMESTAMP {
                        let mut tv: libc::timeval = std::mem::zeroed();
                        std::ptr::copy_nonoverlapping(
                            libc::CMSG_DATA(cmsg),
                            &mut tv as *mut _ as *mut u8,
                            std::mem::size_of::<libc::timeval>(),
                        );
                        when = tv.tv_sec as u64 * 1000 + (tv.tv_usec as u64) / 1000;
                    }
                    cmsg = libc::CMSG_NXTHDR(msg, cmsg);
                }
            }
            let id = u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]);
            let dlc = raw[4] as usize;
            let mut data = [0u8; 8];
            data.copy_from_slice(&raw[8..16]);
            Some(RecvFrame {
                id,
                dlc: dlc.min(8),
                data,
                when_ms: when,
            })
        }
    }

    /// One `recvmmsg` call, filling `batch` with up to `RX_BATCH` frames
    /// (each with its kernel SO_TIMESTAMP). Returns the number of frames
    /// read; `Ok(0)` means the socket is drained right now (EAGAIN).
    /// Non-blocking via `MSG_DONTWAIT` so it never stalls the claim
    /// timers even though `poll` already reported readability.
    fn recv_batch(fd: i32, batch: &mut RxBatch) -> std::io::Result<usize> {
        batch.prepare();
        // SAFETY: `msgs` is a live array of `RX_BATCH` wired-up mmsghdrs;
        // the kernel writes only within the buffers `prepare` pointed at.
        let n = unsafe {
            libc::recvmmsg(
                fd,
                batch.msgs.as_mut_ptr(),
                RX_BATCH as libc::c_uint,
                // The flags parameter is c_int on glibc but c_uint on musl;
                // `as _` lets the same call compile for both libc targets
                // (the static-musl release build needs this).
                libc::MSG_DONTWAIT as _,
                std::ptr::null_mut(),
            )
        };
        if n < 0 {
            let e = std::io::Error::last_os_error();
            return match e.raw_os_error() {
                Some(code) if code == libc::EAGAIN || code == libc::EWOULDBLOCK => Ok(0),
                Some(code) if code == libc::EINTR => Ok(0),
                _ => Err(e),
            };
        }
        Ok(n as usize)
    }

    /// Translate an incoming single-frame CAN message. Drives the
    /// claim state machine immediately on the raw single frame (its
    /// PGNs — 60928, 59904 — are all single-frame anyway), then
    /// pushes through the reassembler. On a coalesced output we (a)
    /// invoke the Group Function handler if it's PGN 126208 and (b)
    /// forward the *coalesced* `RawFrame` to `frames_tx` so the
    /// consumer (canboat-pipeline, socketcan-serial stdout, …) sees a
    /// complete PGN, matching the NGT-1 / iKonvert adapter contract.
    fn handle_frame(
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

        // Classify with the build-time fastpacket table, push through
        // the reassembler, and forward the coalesced result. A real
        // single-frame PGN takes the `PassThrough` branch unchanged;
        // a fast-packet PGN accumulates until `Complete`.
        let pt = fastpacket::packet_type(pgn);
        match reasm.push(single_frame, pt) {
            Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
                if claimer.claim.state() != ClaimState::Disabled && f.pgn == PGN_GROUP_FUNCTION {
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
    /// interpreted as "use my claim address"; any other value is
    /// forwarded unchanged so quirk synthesisers can impersonate other
    /// nodes on the wire (e.g. the SCX-20 quirk uses `src = 52`).
    fn dispatch_cmd(bus: &mut Bus<'_>, claimer: &NmeaDevice, cmd: WriterCmd) {
        match cmd {
            WriterCmd::Frame(f) => {
                if f.pgn >= CANBOAT_PGN_START {
                    return; // synthetic, never goes on the bus
                }
                let src = if f.src == 0 || f.src == ADDR_GLOBAL {
                    claimer.addr()
                } else {
                    f.src
                };
                // emit=false: this is a user-initiated send; the caller
                // already has visibility into what they sent. (For the
                // SCX-20 quirk we want the synthetic 126996 to be
                // visible in the local pipeline too — the caller injects
                // it into the analyzer side directly.)
                bus.send_pgn(f.prio, f.pgn, src, f.dst, &f.data, false);
            }
            WriterCmd::Bytes(_) => {
                // The SocketCAN backend has no concept of raw "bytes"
                // since the wire format is frame-based. Silently drop;
                // log so a misuse is at least visible.
                log::debug!("WriterCmd::Bytes ignored by socketcan adapter");
            }
        }
    }

    /// Open the SocketCAN interface and spawn the bus-participant
    /// worker thread. Returns a standard [`DeviceHandle`] for the
    /// caller to drain RX from and push TX into.
    ///
    /// `claim_addr` is shared with the worker: it starts at
    /// [`super::CLAIM_UNCLAIMED`] (254) and is updated to our
    /// currently-claimed source address every time the worker decides
    /// it. Callers that need to rewrite an outbound `src` to match the
    /// gateway's live address (e.g. canboat-pipeline's CSV-port
    /// injector, so the in-process loopback shows the same `src` the
    /// rewritten frame will reach the bus with) read it from this
    /// atom. The supervisor reuses the same atom across reconnects.
    /// NMEA 2000 runs at a fixed 250 kbit/s — the standard never varies, so
    /// the managed bring-up hard-codes it rather than exposing a knob.
    const NMEA2000_BITRATE: u32 = 250_000;
    /// Bus-off auto-recovery delay for the managed bring-up (matches the
    /// historical `ip link … restart-ms 100`).
    const NMEA2000_RESTART_MS: u32 = 100;

    /// Configure and bring up a SocketCAN link via netlink (RTM_NEWLINK),
    /// replacing an external `ip link set … up type can bitrate 250000 …`
    /// unit. A CAN controller must be *down* to set its bit timing, so we
    /// always cycle down → set bitrate + restart-ms → up. Retried a few times
    /// because some controllers (notably the MCP2515) intermittently fail to
    /// enter config mode on the first bring-up after boot, and a fresh down→up
    /// often clears it. Best-effort: on give-up it logs and returns rather
    /// than aborting the device session, so a later supervisor reconnect can
    /// try again. PRIVILEGED — requires the process to run as root.
    fn configure_can_link(iface: &str) {
        let ci = match CanInterface::open(iface) {
            Ok(ci) => ci,
            Err(e) => {
                log::error!("CAN {iface}: cannot open netlink handle to configure link: {e}");
                return;
            }
        };
        const ATTEMPTS: u32 = 5;
        for attempt in 1..=ATTEMPTS {
            // Down first: bit timing can only be set while down, and it also
            // resets a controller left half-configured by a prior failed
            // bring-up. An already-down interface makes this a no-op.
            let _ = ci.bring_down();
            let result = ci
                .set_bitrate(NMEA2000_BITRATE, None::<u32>)
                .and_then(|()| ci.set_restart_ms(NMEA2000_RESTART_MS))
                .and_then(|()| ci.bring_up());
            match result {
                Ok(()) => {
                    log::info!(
                        "CAN {iface}: configured {NMEA2000_BITRATE} bit/s, \
                         restart-ms {NMEA2000_RESTART_MS}, link up (attempt {attempt})"
                    );
                    return;
                }
                Err(e) => {
                    log::warn!("CAN {iface}: bring-up attempt {attempt}/{ATTEMPTS} failed: {e}");
                    thread::sleep(std::time::Duration::from_millis(500));
                }
            }
        }
        log::error!(
            "CAN {iface}: could not bring link up after {ATTEMPTS} attempts; opening socket \
             anyway (TX will fail with ENETDOWN until the controller recovers)"
        );
    }

    pub fn run(
        iface: &str,
        config: Config,
        claim_addr: Arc<AtomicU8>,
    ) -> std::io::Result<DeviceHandle> {
        // Own the link's lifecycle here instead of relying on an external
        // bring-up (systemd/ip) that may not be ordered before us — the
        // historical failure mode where the N2K interface was left DOWN
        // because its bring-up unit no longer ran first. Best-effort: `run()`
        // proceeds even if it can't come up, so the supervisor keeps retrying.
        if config.configure_link {
            configure_can_link(iface);
        }
        let sock = CanSocket::open(iface).map_err(std::io::Error::other)?;
        sock.set_nonblocking(true)?;
        let fd = sock.as_raw_fd();

        // Kernel RX timestamps so emitted frames carry the wire time
        // rather than when we got round to draining the kernel queue.
        let on: libc::c_int = 1;
        // SAFETY: setsockopt with a valid fd and an int-sized option.
        let rc = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMP,
                &on as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as _,
            )
        };
        if rc < 0 {
            log::warn!(
                "SO_TIMESTAMP: {} (continuing without kernel timestamps)",
                std::io::Error::last_os_error()
            );
        }

        let (frames_tx, frames_rx) = mpsc::channel::<RawFrame>();
        let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd>();

        let iface_owned = iface.to_string();
        let join = thread::Builder::new()
            .name("socketcan-worker".into())
            .spawn(move || worker(sock, fd, iface_owned, config, frames_tx, cmd_rx, claim_addr))?;

        Ok(from_parts(frames_rx, cmd_tx, vec![join]))
    }

    /// The single worker thread that owns the socket and runs the poll
    /// loop. Polls the CAN fd, drains `cmd_rx` via `try_recv`, runs the
    /// claim/heartbeat timers, and drains the TX buffer one frame per
    /// writability wakeup.
    fn worker(
        sock: CanSocket,
        fd: i32,
        iface: String,
        config: Config,
        frames_tx: mpsc::Sender<RawFrame>,
        cmd_rx: mpsc::Receiver<WriterCmd>,
        claim_addr: Arc<AtomicU8>,
    ) {
        let mut claimer = NmeaDevice::new(&config, &iface);
        // Reset the claim atom to "unclaimed" so a reconnect resumes
        // with no stale value visible to consumers.
        claim_addr.store(super::CLAIM_UNCLAIMED, Ordering::Relaxed);
        let mut tx_buf = TxBuffer::new();
        let mut last_published_addr: u8 = super::CLAIM_UNCLAIMED;
        // Fast-packet reassembler driven by the build-time
        // `fastpacket` table. The library hands fully coalesced
        // `RawFrame`s to `frames_tx`, matching the NGT-1 / iKonvert
        // adapter contract; both the standalone binary and
        // canboat-pipeline can then run with `pre_coalesced = true`
        // and skip a second reassembly pass.
        let mut reasm = Reassembler::new();
        // Reused across the whole thread lifetime: one `recvmmsg` scratch
        // buffer so the RX drain allocates nothing per wakeup.
        let mut rx_batch = RxBatch::new();
        let mut last_frame = now_ms();
        let timeout_ms = config.timeout_secs.saturating_mul(1000);

        // Kick off the claim handshake if enabled.
        if claimer.claim.state() != ClaimState::Disabled {
            let mut bus = Bus {
                tx_buf: &mut tx_buf,
                frames_tx: &frames_tx,
            };
            claimer.start(&mut bus);
        }

        loop {
            let now = now_ms();

            // 1. Drain any user-side sends into the TX ring.
            loop {
                match cmd_rx.try_recv() {
                    Ok(cmd) => {
                        let mut bus = Bus {
                            tx_buf: &mut tx_buf,
                            frames_tx: &frames_tx,
                        };
                        dispatch_cmd(&mut bus, &claimer, cmd);
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => return,
                }
            }

            // 2. Pick a poll timeout: soonest of claim deadline,
            //    heartbeat, MAX_POLL_MS, and (when TX is backed up) 5 ms
            //    as a safety net in case POLLOUT lags qdisc availability.
            let mut wait: u64 = if claimer.claim.is_timing() && claimer.claim.deadline() > now {
                claimer.claim.deadline() - now
            } else if claimer.claim.is_claimed() && claimer.heartbeat_interval > 0 {
                claimer.next_heartbeat.saturating_sub(now)
            } else {
                MAX_POLL_MS
            };
            if !tx_buf.is_empty() && wait > 5 {
                wait = 5;
            }
            if wait > MAX_POLL_MS {
                wait = MAX_POLL_MS;
            }
            let timeout_arg = wait.min(i32::MAX as u64) as i32;

            // 3. poll() — POLLIN always; POLLOUT only while TX is pending.
            let events = if tx_buf.is_empty() {
                libc::POLLIN
            } else {
                libc::POLLIN | libc::POLLOUT
            };
            let mut pfd = libc::pollfd {
                fd,
                events,
                revents: 0,
            };
            // SAFETY: poll over a single live pollfd.
            let r = unsafe { libc::poll(&mut pfd, 1, timeout_arg) };
            if r < 0 {
                let e = std::io::Error::last_os_error();
                if e.raw_os_error() == Some(libc::EINTR) {
                    continue;
                }
                log::error!("socketcan poll: {e}");
                return;
            }

            // 4. Idle-timeout check.
            if timeout_ms > 0 && now_ms().saturating_sub(last_frame) >= timeout_ms {
                log::error!("Timeout {} seconds; no data received", config.timeout_secs);
                return;
            }

            // 5. Drain one TX frame per writability wakeup (or per
            //    safety-net timeout).
            if !tx_buf.is_empty() {
                tx_drain_one(&sock, &mut tx_buf);
            }

            // 6. Drain RX in `recvmmsg` batches up to a per-wakeup budget;
            //    yield then so the claim timers keep advancing on a busy
            //    bus. Reading a whole batch per syscall also lets the
            //    downstream relay coalesce its wakeups: the batch's frames
            //    land in `frames_tx` back-to-back, so a greedy-draining
            //    consumer parks once per batch rather than once per frame.
            if pfd.revents & libc::POLLIN != 0 {
                last_frame = now_ms();
                let mut budget: i64 = 4096;
                loop {
                    let n = match recv_batch(fd, &mut rx_batch) {
                        Ok(0) => break,
                        Ok(n) => n,
                        Err(e) => {
                            log::error!("socketcan recvmmsg: {e}");
                            return;
                        }
                    };
                    for i in 0..n {
                        let Some(rx) = rx_batch.parse(i) else {
                            continue;
                        };
                        if rx.id & CAN_ERR_FLAG != 0 {
                            continue;
                        }
                        // NMEA 2000 is always 29-bit extended. CAN 1.0
                        // standard frames (11-bit, no EFF flag) cannot
                        // be N2K, so skip them rather than reinterpret
                        // the low 11 bits as a degenerate 29-bit id.
                        if rx.id & CAN_EFF_FLAG == 0 {
                            continue;
                        }
                        let mut bus = Bus {
                            tx_buf: &mut tx_buf,
                            frames_tx: &frames_tx,
                        };
                        handle_frame(
                            &mut bus,
                            &mut claimer,
                            &mut reasm,
                            rx.id & CAN_EFF_MASK,
                            &rx.data[..rx.dlc],
                            if rx.when_ms != 0 {
                                rx.when_ms
                            } else {
                                now_ms()
                            },
                        );
                    }
                    budget -= n as i64;
                    // Socket drained (short batch) or budget spent: let the
                    // loop re-poll so claim/heartbeat timers advance.
                    if n < RX_BATCH || budget <= 0 {
                        break;
                    }
                }
            }

            // 7. Claim / heartbeat timers.
            let mut bus = Bus {
                tx_buf: &mut tx_buf,
                frames_tx: &frames_tx,
            };
            claimer.tick(&mut bus, now_ms());

            // 8. Publish the live claim to the shared atom so external
            //    consumers (e.g. the canboat-pipeline CSV injector,
            //    which needs to rewrite an incoming src=0/255 to our
            //    address before mirroring the frame onto the in-process
            //    pipeline) always see what's on the wire. Only update
            //    when actually claimed; while scanning/pending/failed
            //    leave it at CLAIM_UNCLAIMED so the caller knows not
            //    to rewrite yet.
            let live = claimer.claim.address().unwrap_or(super::CLAIM_UNCLAIMED);
            if live != last_published_addr {
                claim_addr.store(live, Ordering::Relaxed);
                last_published_addr = live;
            }
        }
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
                .map(|f| f.data().to_vec())
                .collect()
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
    }
}
