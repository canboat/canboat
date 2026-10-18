// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! [`Device`]: the one way to open a gateway, by its [`Kind`] and where it
//! is, for whole NMEA 2000 messages ([`Device::open`]) or single CAN
//! frames ([`Device::open_raw`]).

use std::io;
use std::path::PathBuf;
use std::sync::atomic::AtomicU8;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::engine::BusProtocol;
use crate::engine::codec::ngt1_tx_list::TxListRecord;
use crate::engine::codec::raw::{self as raw_codec, CanFrame, RawCodec};
use crate::engine::format::ngt1::OperatingMode;
use crate::engine::pgn_list::PgnLists;
#[cfg(feature = "io")]
use crate::engine::pgn_list::{PgnListStatus, PgnListSupport};

use super::kind::{ActisenseFormat, Bitrates, Capabilities, Kind, ListUse, Setting, Transport};
use super::supervisor::{Named, supervise};
use super::{CLAIM_UNCLAIMED, DeviceHandle};

/// A gateway to open: its [`Kind`], where it is, and how to run it.
///
/// ```no_run
/// use canboat::bus::{BusProtocol, Device, Kind};
///
/// let dev = Device::new(Kind::YachtDevices, "tcp://ydwg");
/// assert!(dev.capabilities().raw);
/// let link = dev.open(BusProtocol::Nmea2000)?;
/// for frame in link.frames_rx.iter().take(10) {
///     println!("{} from {}", frame.pgn, frame.src);
/// }
/// # Ok::<(), std::io::Error>(())
/// ```
///
/// The address is a serial port (`/dev/ttyUSB0`, `COM3`), `usb` or
/// `usb:SERIAL`, `tcp://host[:port]`, `udp://[host:]port` or a network
/// interface (`can0`), as the kind takes: see [`Kind::transports`], which
/// also gives the port and serial speed when the address names none.
///
/// A setting the kind does not have ([`Kind::settings`]), a bit rate it
/// cannot run at, or an address it cannot be reached at, makes
/// [`open`](Self::open) and [`open_raw`](Self::open_raw) fail with
/// [`io::ErrorKind::InvalidInput`]: no setting is ever ignored.
///
/// Without [`reconnect`](Self::reconnect), what `open` returns lasts one
/// session: when the gateway goes, its `frames_rx` closes and
/// [`lost`](DeviceHandle::lost) says why. With it, canboat opens the
/// gateway again whenever it goes, behind the same handle.
#[derive(Debug, Clone)]
pub struct Device {
    kind: Kind,
    address: String,
    /// The settings given, to check against the kind's.
    given: Vec<Setting>,
    baud: Option<u32>,
    bitrate: Option<u32>,
    configure_link: bool,
    channel: u8,
    password: String,
    mode: OperatingMode,
    no_tx_rate_limit: bool,
    ignore_checksum: bool,
    rx_filter: Vec<u32>,
    pgn_lists: PgnLists,
    extra_tx_pgns: Vec<u32>,
    node: Node,
    reconnect: bool,
    /// What every session of this device shares, also across clones.
    shared: Shared,
}

/// canboat's node, where it runs one.
#[derive(Debug, Clone)]
struct Node {
    address: u8,
    unique: u32,
    manufacturer: u16,
    system_instance: u8,
    heartbeat_ms: u64,
    no_claim: bool,
    learn_tx_pgns: bool,
    model_version: Option<&'static str>,
    state_dir: Option<PathBuf>,
    timeout: Duration,
}

impl Default for Node {
    fn default() -> Self {
        Node {
            address: 0,
            unique: 0,
            manufacturer: 999,
            system_instance: 15,
            heartbeat_ms: 60_000,
            no_claim: false,
            learn_tx_pgns: true,
            model_version: None,
            state_dir: None,
            timeout: Duration::ZERO,
        }
    }
}

#[derive(Debug, Clone)]
struct Shared {
    /// The address the node holds, kept across reconnects for what stamps
    /// it on frames it sends.
    claim: Arc<AtomicU8>,
    /// The NGT-1's: a PGN put on its transmit list once is not written
    /// again after a reconnect.
    tx_list_record: Arc<Mutex<TxListRecord>>,
}

/// The bit rate of an NMEA 2000 bus, where canboat sets one and none is
/// given.
const NMEA_2000_BITRATE: u32 = 250_000;

fn invalid(why: String) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, why)
}

impl Device {
    /// The `kind` gateway at `address`.
    pub fn new(kind: Kind, address: impl Into<String>) -> Self {
        Device {
            kind,
            address: address.into(),
            given: Vec::new(),
            baud: None,
            bitrate: None,
            configure_link: false,
            channel: 0,
            password: String::new(),
            mode: OperatingMode::NgTransferRxAll,
            no_tx_rate_limit: false,
            ignore_checksum: false,
            rx_filter: Vec::new(),
            pgn_lists: PgnLists::default(),
            extra_tx_pgns: Vec::new(),
            node: Node::default(),
            reconnect: false,
            shared: Shared {
                claim: Arc::new(AtomicU8::new(CLAIM_UNCLAIMED)),
                tx_list_record: Arc::default(),
            },
        }
    }

    fn with(mut self, setting: Setting) -> Self {
        if !self.given.contains(&setting) {
            self.given.push(setting);
        }
        self
    }

    /// Its kind.
    pub fn kind(&self) -> Kind {
        self.kind
    }

    /// Where it is.
    pub fn address(&self) -> &str {
        &self.address
    }

    /// The serial speed, for a gateway on a serial port. Defaults to the
    /// kind's ([`Kind::default_baud`]).
    pub fn baud(mut self, baud: u32) -> Self {
        self.baud = Some(baud);
        self.with(Setting::Baud)
    }

    /// The CAN bus bit rate, where canboat is the CAN controller: one of
    /// [`Capabilities::bitrates`]. Defaults to 250 000, NMEA 2000's. On
    /// SocketCAN it also configures the link, as
    /// [`configure_link`](Self::configure_link).
    pub fn bitrate(mut self, bitrate: u32) -> Self {
        self.bitrate = Some(bitrate);
        self.with(Setting::Bitrate)
    }

    /// SocketCAN: bring the interface down, set its bit rate (250 000
    /// unless [`bitrate`](Self::bitrate) says otherwise) and bring it up
    /// again, which needs root or CAP_NET_ADMIN. Without it the interface
    /// is used as it was set up.
    pub fn configure_link(mut self, configure: bool) -> Self {
        self.configure_link = configure;
        if configure {
            self.with(Setting::ConfigureLink)
        } else {
            self
        }
    }

    /// The CAN channel of a CANalyst-II, 0 or 1. Defaults to 0.
    pub fn channel(mut self, channel: u8) -> Self {
        self.channel = channel;
        self.with(Setting::Channel)
    }

    /// The Maretron IPG's login password. Defaults to none.
    pub fn password(mut self, password: impl Into<String>) -> Self {
        self.password = password.into();
        self.with(Setting::Password)
    }

    /// The NGT-1's operating mode. Defaults to Rx All, which passes on
    /// every PGN received.
    pub fn mode(mut self, mode: OperatingMode) -> Self {
        self.mode = mode;
        self.with(Setting::Mode)
    }

    /// iKonvert: turn off the gateway's transmit rate limit, at your own
    /// risk.
    pub fn no_tx_rate_limit(mut self) -> Self {
        self.no_tx_rate_limit = true;
        self.with(Setting::NoTxRateLimit)
    }

    /// MiniPlex and SeaSmart: accept sentences whose checksum is wrong.
    pub fn ignore_checksum(mut self) -> Self {
        self.ignore_checksum = true;
        self.with(Setting::IgnoreChecksum)
    }

    /// iKonvert: have the gateway pass on only these PGNs (and network
    /// management). The other kinds have no such filter; drop what is not
    /// wanted as it arrives instead.
    pub fn rx_filter(mut self, pgns: Vec<u32>) -> Self {
        self.rx_filter = pgns;
        self.with(Setting::RxFilter)
    }

    /// The PGNs the application receives and transmits: what the gateway
    /// does with them is [`Kind::pgn_lists`], and
    /// [`pgn_list_status`](Self::pgn_list_status) says how it went. Every
    /// kind takes them; one that has no such list leaves them be.
    ///
    /// On an NGT-1 or iKonvert, once the transmit list names a PGN, every
    /// PGN not on it is refused (network management apart).
    pub fn pgn_lists(mut self, lists: PgnLists) -> Self {
        self.pgn_lists = lists;
        self
    }

    /// PGNs transmitted that the application did not name in
    /// [`pgn_lists`](Self::pgn_lists), such as those a quirk sends: put on
    /// the gateway's transmit list too, but not reported as asked for.
    pub fn extra_tx_pgns(mut self, pgns: Vec<u32>) -> Self {
        self.extra_tx_pgns = pgns;
        self
    }

    /// The source address canboat's node claims; a free one if it is
    /// taken. Defaults to 0.
    pub fn source_address(mut self, address: u8) -> Self {
        self.node.address = address;
        self.with(Setting::Node)
    }

    /// The ISO NAME unique number. Defaults to one derived from the
    /// machine id, stable across restarts.
    pub fn unique(mut self, unique: u32) -> Self {
        self.node.unique = unique;
        self.with(Setting::Node)
    }

    /// The ISO NAME manufacturer code. Defaults to 999, Signal K.
    pub fn manufacturer(mut self, manufacturer: u16) -> Self {
        self.node.manufacturer = manufacturer;
        self.with(Setting::Node)
    }

    /// The ISO NAME System Instance, 0 to 15. Defaults to 15, which makes
    /// canboat's NAME lose to real hardware's.
    pub fn system_instance(mut self, instance: u8) -> Self {
        self.node.system_instance = instance;
        self.with(Setting::Node)
    }

    /// The Heartbeat (PGN 126993) interval in ms; 0 for none. Defaults to
    /// 60 000.
    pub fn heartbeat_ms(mut self, ms: u64) -> Self {
        self.node.heartbeat_ms = ms;
        self.with(Setting::Node)
    }

    /// Whether the node adds PGNs to the Transmit list it advertises as
    /// it first sends them. Defaults to true.
    pub fn learn_tx_pgns(mut self, learn: bool) -> Self {
        self.node.learn_tx_pgns = learn;
        self.with(Setting::Node)
    }

    /// End the session when nothing has been received for this long; zero
    /// (the default) for never.
    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.node.timeout = timeout;
        self.with(Setting::Node)
    }

    /// Run canboat's node without claiming an address: listen only, or on
    /// an AOS simulator send from the simulator's own.
    pub fn no_claim(mut self) -> Self {
        self.node.no_claim = true;
        self.with(Setting::NoClaim)
    }

    /// The model version canboat's node gives in its Product Information,
    /// where it runs one.
    pub fn model_version(mut self, version: &'static str) -> Self {
        self.node.model_version = Some(version);
        self
    }

    /// Where canboat's node keeps a random unique number, on a machine
    /// that cannot be identified, where it runs one.
    pub fn state_dir(mut self, dir: PathBuf) -> Self {
        self.node.state_dir = Some(dir);
        self
    }

    /// [`model_version`](Self::model_version) and
    /// [`state_dir`](Self::state_dir), where none was given: `canboat
    /// server`'s.
    #[cfg_attr(not(feature = "bridge"), allow(dead_code))]
    pub(crate) fn node_defaults(mut self, version: &'static str, dir: Option<PathBuf>) -> Self {
        self.node.model_version = self.node.model_version.or(Some(version));
        self.node.state_dir = self.node.state_dir.or(dir);
        self
    }

    /// Open the gateway again whenever it goes, with backoff, behind one
    /// long-lived handle. Defaults to false.
    pub fn reconnect(mut self, reconnect: bool) -> Self {
        self.reconnect = reconnect;
        self
    }

    /// What it can do, at this address: a `udp://port` only receives (as
    /// does a `udp://` with an address of this host, which is only known
    /// once it is opened).
    pub fn capabilities(&self) -> Capabilities {
        let mut caps = self.kind.capabilities();
        if self.receive_only() {
            caps.sends = false;
            caps.raw_sends = false;
        }
        caps
    }

    /// Whether the address only receives.
    fn receive_only(&self) -> bool {
        let Some(rest) = self.address.strip_prefix("udp://") else {
            return false;
        };
        // A NavLink2 takes nothing over UDP.
        self.kind == Kind::Ikonvert || !rest.contains(':')
    }

    fn is_network(&self) -> bool {
        self.address.starts_with("tcp://") || self.address.starts_with("udp://")
    }

    /// The kind's TCP port, for an address that names none.
    fn tcp_port(&self) -> u16 {
        self.kind
            .transports()
            .iter()
            .find_map(|t| match *t {
                Transport::Tcp { port } => Some(port),
                _ => None,
            })
            .unwrap_or(0)
    }

    /// The serial speed: as given, or the kind's.
    fn serial_baud(&self) -> u32 {
        self.baud.or(self.kind.default_baud()).unwrap_or(0)
    }

    /// Refuse what the kind does not have: a setting, a bit rate, an
    /// address it cannot be reached at, and `protocol`, when given.
    pub(crate) fn check(&self, protocol: Option<BusProtocol>) -> io::Result<()> {
        let kind = self.kind;
        let caps = kind.capabilities();
        if let Some(&setting) = self.given.iter().find(|&&s| !kind.takes(s)) {
            return Err(invalid(format!(
                "{kind}: has no {} setting",
                setting.name()
            )));
        }
        if let Some(bitrate) = self.bitrate
            && !caps.bitrates.allows(bitrate)
        {
            let runs_at = match caps.bitrates {
                Bitrates::Only(rates) => rates
                    .iter()
                    .map(u32::to_string)
                    .collect::<Vec<_>>()
                    .join(", "),
                _ => "a rate its driver takes".into(),
            };
            return Err(invalid(format!(
                "{kind}: does not run at {bitrate} bit/s; it runs at {runs_at}"
            )));
        }
        if let Some(protocol) = protocol
            && !caps.protocols.contains(&protocol)
        {
            let why = if caps.raw {
                "it carries only 29-bit frames"
            } else {
                "it does NMEA 2000 framing itself"
            };
            return Err(invalid(format!("{kind}: cannot carry {protocol}: {why}")));
        }
        let transports = kind.transports();
        let has = |want: fn(&Transport) -> bool| transports.iter().any(want);
        let fits = if self.address.starts_with("udp://") {
            has(|t| matches!(t, Transport::Udp { .. }))
        } else if self.address.starts_with("tcp://") {
            has(|t| matches!(t, Transport::Tcp { .. }))
        } else if self.address == "usb" || self.address.starts_with("usb:") {
            has(|t| matches!(t, Transport::Usb))
        } else {
            // A serial port or an interface, or a host for a gateway
            // reached only over TCP.
            !has(|t| matches!(t, Transport::Usb)) || transports.len() > 1
        };
        if !fits {
            return Err(invalid(format!(
                "{kind}: cannot be reached at {}; it takes {}",
                self.address,
                describe_transports(transports)
            )));
        }
        Ok(())
    }

    /// Open it for whole NMEA 2000 messages (or J1939 or Quick ones, on a
    /// gateway that passes raw frames: [`Capabilities::protocols`]), with
    /// fast-packets joined and split and, where canboat is the node, its
    /// address claimed and ISO Requests answered.
    #[cfg(feature = "io")]
    pub fn open(&self, protocol: BusProtocol) -> io::Result<DeviceHandle> {
        self.check(Some(protocol))?;
        self.warn_about_pgn_lists();
        let handle = if self.reconnect {
            let dev = self.clone();
            supervise(Named {
                name: self.kind.to_string(),
                open: move || dev.open_session(protocol),
            })
        } else {
            self.open_session(protocol)?
        };
        Ok(if self.kind.capabilities().node {
            handle.with_claim(self.shared.claim.clone())
        } else {
            handle
        })
    }

    /// Open it for single CAN frames, with nothing of NMEA 2000 in
    /// between: nothing joined, split or claimed, nothing sent but what
    /// the application sends. Fails with [`io::ErrorKind::Unsupported`] for
    /// a kind that does NMEA 2000 framing itself
    /// ([`Capabilities::raw`]).
    pub fn open_raw(&self) -> io::Result<DeviceHandle<CanFrame>> {
        if !self.kind.capabilities().raw {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                format!(
                    "{}: does NMEA 2000 framing itself, and passes no single CAN frames",
                    self.kind
                ),
            ));
        }
        self.check(None)?;
        if self.reconnect {
            let dev = self.clone();
            Ok(supervise(Named {
                name: format!("{} raw", self.kind),
                open: move || dev.open_raw_session(),
            }))
        } else {
            self.open_raw_session()
        }
    }

    /// What became of the PGN lists: `None` when none were given.
    #[cfg(feature = "io")]
    pub fn pgn_list_status(&self) -> Option<PgnListStatus> {
        let effective = self.effective_pgn_lists();
        if effective.is_empty() {
            return None;
        }
        let uses = self.kind.pgn_lists();
        Some(if self.receive_only() || uses.tx == ListUse::Ignored {
            PgnListStatus {
                tx: PgnListSupport::Unsupported,
                rx: PgnListSupport::Unsupported,
                dropped: Vec::new(),
            }
        } else {
            match self.kind {
                Kind::NGT1 => super::ngt1::pgn_list_status(&self.pgn_lists, &self.extra_tx_pgns),
                Kind::Ikonvert => super::ikonvert::pgn_list_status(&effective),
                _ => super::socketcan::pgn_list_status(&effective),
            }
        })
    }

    /// The application's lists, with the extra transmit PGNs.
    fn effective_pgn_lists(&self) -> PgnLists {
        let mut lists = self.pgn_lists.clone();
        for &pgn in &self.extra_tx_pgns {
            if !lists.tx.contains(&pgn) {
                lists.tx.push(pgn);
            }
        }
        lists
    }

    /// Say so, once, when a list goes unused.
    #[cfg(feature = "io")]
    fn warn_about_pgn_lists(&self) {
        let (lists, uses) = (&self.pgn_lists, self.kind.pgn_lists());
        let unused = |what: &str, pgns: &[u32], used: ListUse| {
            if !pgns.is_empty() && (used == ListUse::Ignored || self.receive_only()) {
                log::warn!(
                    "{}: has no {what} PGN list here; ignoring {pgns:?}",
                    self.kind
                );
            }
        };
        unused("receive", &lists.rx, uses.rx);
        unused("transmit", &lists.tx, uses.tx);
        if self.no_tx_rate_limit && self.receive_only() {
            log::warn!(
                "{}: sends nothing over udp://; no rate limit to turn off",
                self.kind
            );
        }
    }

    /// The serial port or TCP connection of a gateway reached either way.
    #[cfg(feature = "io")]
    fn serial_or_tcp(&self) -> io::Result<crate::io::net::Pair> {
        if self.address.starts_with("tcp://") {
            return crate::io::net::open_tcp(&self.address, self.tcp_port());
        }
        crate::io::open_serial_rw(&self.address, self.serial_baud()).map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("opening serial port {}: {e}", self.address),
            )
        })
    }

    /// The network endpoint or serial port of a line gateway, and whether
    /// what is written reaches it.
    fn line_endpoint(&self) -> io::Result<(crate::io::net::Pair, bool)> {
        if self.is_network() {
            crate::io::net::open_line_endpoint_sends(&self.address, self.tcp_port())
        } else {
            let pair = crate::io::open_serial_rw(&self.address, self.serial_baud())?;
            Ok((pair, true))
        }
    }

    /// The node's configuration.
    #[cfg(feature = "io")]
    fn node_config(&self, protocol: BusProtocol) -> super::socketcan::Config {
        let model_version = self.node.model_version.or(match self.kind {
            Kind::Aos => Some("canboat-aos"),
            Kind::Canalyst => Some("canboat-canalyst"),
            Kind::UsbCanA => Some("canboat-usb-can-a"),
            _ => None,
        });
        super::socketcan::Config {
            address: self.node.address,
            unique: self.node.unique,
            manufacturer: self.node.manufacturer,
            system_instance: self.node.system_instance,
            heartbeat_ms: self.node.heartbeat_ms,
            no_claim: self.node.no_claim,
            timeout_secs: self.node.timeout.as_secs(),
            model_version,
            configure_link: self.configure_link || self.bitrate.is_some(),
            pgn_lists: self.effective_pgn_lists(),
            learn_tx_pgns: self.node.learn_tx_pgns,
            protocol,
            bitrate: self.bitrate.unwrap_or(NMEA_2000_BITRATE),
            state_dir: self.node.state_dir.clone(),
        }
    }

    /// One session of [`open`](Self::open).
    #[cfg(feature = "io")]
    fn open_session(&self, protocol: BusProtocol) -> io::Result<DeviceHandle> {
        use super::line_gateway::{self, LineFormat};
        let claim = || self.shared.claim.clone();
        let line = |format: LineFormat, init: &[u8]| -> io::Result<DeviceHandle> {
            let ((reader, writer), _) = self.line_endpoint()?;
            Ok(line_gateway::run(
                reader,
                writer,
                format,
                protocol,
                init,
                self.ignore_checksum,
            ))
        };
        Ok(match self.kind {
            Kind::NGT1 => {
                let (reader, writer) = self.serial_or_tcp()?;
                let config = super::ngt1::Config {
                    operating_mode: self.mode,
                    pgn_lists: self.pgn_lists.clone(),
                    extra_tx_pgns: self.extra_tx_pgns.clone(),
                    tx_list_record: self.shared.tx_list_record.clone(),
                };
                // An AOS simulator is Kind::Aos: this gives up on one.
                super::ngt1::run_ngt1(reader, writer, config)
            }
            Kind::Aos => {
                let (reader, writer) = self.serial_or_tcp()?;
                super::ngt1::run_aos(reader, writer, self.node_config(protocol), claim())
            }
            Kind::Ikonvert => {
                let (reader, writer, receive_only) =
                    super::ikonvert::open(&self.address, self.serial_baud())?;
                let config = if receive_only {
                    // Nothing reaches the gateway: its lists and rate limit
                    // stay as they are.
                    super::ikonvert::Config {
                        skip_init: true,
                        ..Default::default()
                    }
                } else {
                    super::ikonvert::Config {
                        rx_list: (!self.rx_filter.is_empty()).then(|| {
                            self.rx_filter
                                .iter()
                                .map(u32::to_string)
                                .collect::<Vec<_>>()
                                .join(",")
                        }),
                        pgn_lists: self.pgn_lists.clone(),
                        extra_tx_pgns: self.extra_tx_pgns.clone(),
                        rate_limit_off: self.no_tx_rate_limit,
                        ..Default::default()
                    }
                };
                super::ikonvert::run(reader, writer, config)
            }
            Kind::Maretron => {
                // `host[:port]` as well as `tcp://host[:port]`: the C
                // maretron-ipg's syntax, which signalk-server spawns it with.
                let (reader, writer) = crate::io::net::open_tcp(&self.address, self.tcp_port())?;
                let config = super::maretron::Config {
                    password: self.password.clone(),
                    fixtime: None,
                };
                super::maretron::run(reader, writer, config)
            }
            Kind::YachtDevices => {
                // A network endpoint is a YDWG-02 / YDEN; anything else is
                // the serial port of a YDNU-02, which is first put in RAW
                // mode.
                let init = if self.is_network() {
                    &[][..]
                } else {
                    line_gateway::YDNU_RAW_MODE
                };
                line(LineFormat::YdwgRaw, init)?
            }
            Kind::Miniplex => {
                let ((reader, writer), sends) = self.line_endpoint()?;
                if !sends {
                    log::warn!(
                        "canboat can't set a MiniPlex it can't send to: set it to send \
                         every PGN as $MXPGN yourself ($PSMDNO options 30 and 31, \
                         mode 2), or give its address, udp://miniplex:port"
                    );
                }
                line_gateway::run(
                    reader,
                    writer,
                    LineFormat::Mxpgn,
                    protocol,
                    &line_gateway::miniplex_init(),
                    self.ignore_checksum,
                )
            }
            Kind::SeaSmart => line(LineFormat::SeaSmart, &[])?,
            Kind::Actisense(ActisenseFormat::RawAscii) => line(LineFormat::YdwgRaw, &[])?,
            Kind::Actisense(ActisenseFormat::N2kAscii) => line(LineFormat::N2kAscii, &[])?,
            Kind::Actisense(ActisenseFormat::Mxpgn) => line(LineFormat::ActisenseMxpgn, &[])?,
            Kind::Actisense(ActisenseFormat::N2kActisense) => {
                let (reader, writer) = crate::io::net::open_tcp(&self.address, self.tcp_port())?;
                super::bst_d0::run(reader, writer)
            }
            Kind::Actisense(ActisenseFormat::RawActisense) => {
                // A network endpoint is a PRO-NDC-1E2K / W2K-1; anything
                // else is the serial port of an NGX.
                let (reader, writer) = self.serial_or_tcp()?;
                super::bst95::run(reader, writer, protocol)
            }
            Kind::SocketCan => {
                super::socketcan::run(&self.address, self.node_config(protocol), claim()).map_err(
                    |e| {
                        io::Error::new(
                            e.kind(),
                            format!("opening SocketCAN interface {}: {e}", self.address),
                        )
                    },
                )?
            }
            Kind::Canalyst => {
                #[cfg(feature = "raw-usb")]
                {
                    let config = self.node_config(protocol);
                    let (reader, writer) = self.canalyst_rw(config.bitrate)?;
                    super::canalyst::run(reader, writer, config, claim())?
                }
                #[cfg(not(feature = "raw-usb"))]
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "a CANalyst-II needs canboat built with the raw-usb feature",
                ));
            }
            Kind::UsbCanA => {
                let (reader, writer) = super::usb_can_a::open_rw(&self.address, self.baud)
                    .map_err(|e| {
                        io::Error::new(e.kind(), format!("opening USB-CAN-A {}: {e}", self.address))
                    })?;
                super::usb_can_a::run(reader, writer, self.node_config(protocol), claim())?
            }
        })
    }

    /// A CANalyst-II's message endpoints, its channel opened at `bitrate`.
    #[cfg(feature = "raw-usb")]
    fn canalyst_rw(&self, bitrate: u32) -> io::Result<crate::io::net::Pair> {
        let selector = crate::io::usb::Selector::parse(&self.address).unwrap_or_else(|| {
            Err(invalid(format!(
                "{}: a CANalyst-II is usb or usb:SERIAL",
                self.address
            )))
        })?;
        crate::io::canalyst::open_rw(&selector, self.channel, bitrate).map_err(|e| {
            io::Error::new(
                e.kind(),
                format!("opening CANalyst-II {}: {e}", self.address),
            )
        })
    }

    /// The codec [`open_raw`](Self::open_raw) runs: `None` for SocketCAN,
    /// which needs none, and for a kind that passes no single frames.
    pub(crate) fn raw_codec(&self) -> Option<Box<dyn RawCodec + Send>> {
        Some(match self.kind {
            Kind::YachtDevices if self.is_network() => Box::new(raw_codec::Ydwg::new()),
            Kind::YachtDevices => Box::new(raw_codec::Ydwg::ydnu()),
            Kind::Actisense(ActisenseFormat::RawAscii) => Box::new(raw_codec::Ydwg::new()),
            Kind::Miniplex => Box::new(raw_codec::Mxpgn::miniplex()),
            Kind::Actisense(ActisenseFormat::Mxpgn) => Box::new(raw_codec::Mxpgn::actisense()),
            Kind::Actisense(ActisenseFormat::RawActisense) => Box::new(raw_codec::Bst95::new()),
            Kind::Canalyst => Box::new(raw_codec::Canalyst::new()),
            Kind::UsbCanA => Box::new(raw_codec::UsbCanA::new(
                self.bitrate.unwrap_or(NMEA_2000_BITRATE),
            )?),
            _ => return None,
        })
    }

    /// One session of [`open_raw`](Self::open_raw).
    fn open_raw_session(&self) -> io::Result<DeviceHandle<CanFrame>> {
        if self.kind == Kind::SocketCan {
            if self.configure_link || self.bitrate.is_some() {
                #[cfg(all(feature = "io", target_os = "linux"))]
                super::socketcan::configure_link(
                    &self.address,
                    self.bitrate.unwrap_or(NMEA_2000_BITRATE),
                );
                #[cfg(not(all(feature = "io", target_os = "linux")))]
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "configuring a SocketCAN link needs Linux and canboat's io feature",
                ));
            }
            return super::raw::open_socketcan(&self.address);
        }
        let codec = self
            .raw_codec()
            .expect("open_raw checks the kind passes raw frames");
        let (reader, writer) = match self.kind {
            Kind::Canalyst => {
                #[cfg(feature = "raw-usb")]
                {
                    self.canalyst_rw(self.bitrate.unwrap_or(NMEA_2000_BITRATE))?
                }
                #[cfg(not(feature = "raw-usb"))]
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "a CANalyst-II needs canboat built with the raw-usb feature",
                ));
            }
            Kind::UsbCanA => crate::io::open_serial_rw(&self.address, self.serial_baud())?,
            Kind::Actisense(ActisenseFormat::RawActisense) if !self.is_network() => {
                crate::io::open_serial_rw(&self.address, self.serial_baud())?
            }
            Kind::Actisense(ActisenseFormat::RawActisense) => {
                crate::io::net::open_tcp(&self.address, self.tcp_port())?
            }
            _ => self.line_endpoint()?.0,
        };
        Ok(super::raw::run(codec, reader, writer))
    }
}

/// `tcp://host[:port] (port 1457), udp://…, a serial port (38400 baud)`.
fn describe_transports(transports: &[Transport]) -> String {
    transports
        .iter()
        .map(|t| match *t {
            Transport::Serial { baud } => format!("a serial port ({baud} baud)"),
            Transport::Usb => "usb or usb:SERIAL".into(),
            Transport::Tcp { port } => format!("tcp://host[:port] (port {port})"),
            Transport::Udp { port: Some(port) } => format!("udp://[host:]port (port {port})"),
            Transport::Udp { port: None } => "udp://[host:]port".into(),
            Transport::Interface => "a network interface (can0)".into(),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::codec::Refused;

    /// An address `kind` is reached at.
    fn reachable(kind: Kind) -> &'static str {
        match kind.transports()[0] {
            Transport::Serial { .. } => "/dev/ttyUSB0",
            Transport::Usb => "usb",
            Transport::Tcp { .. } => "tcp://192.0.2.1",
            Transport::Udp { .. } => "udp://192.0.2.1:1",
            Transport::Interface => "can0",
        }
    }

    fn frames() -> (CanFrame, CanFrame) {
        (
            CanFrame::extended(0x09F1_1234, &[1, 2, 3]).unwrap(),
            CanFrame::standard(0x123, &[1, 2, 3]).unwrap(),
        )
    }

    /// The issue's own test: what each kind says it can send single
    /// frames of, its codec sends, and what it says it cannot, its codec
    /// refuses.
    #[test]
    fn every_kind_s_capabilities_match_what_its_codec_refuses() {
        let (extended, standard) = frames();
        for &kind in Kind::ALL {
            let caps = kind.capabilities();
            let dev = Device::new(kind, "tcp://192.0.2.1");
            let Some(mut codec) = dev.raw_codec() else {
                // No codec: SocketCAN passes frames as they are, and the
                // others pass none, which open_raw says before opening.
                assert_eq!(caps.raw, kind == Kind::SocketCan, "{kind}");
                if !caps.raw {
                    let Err(e) = Device::new(kind, reachable(kind)).open_raw() else {
                        panic!("{kind}: open_raw opened");
                    };
                    assert_eq!(e.kind(), io::ErrorKind::Unsupported, "{kind}");
                }
                continue;
            };
            assert!(caps.raw, "{kind}");
            let codec_caps = codec.capabilities();
            assert_eq!(codec_caps.sends, caps.raw_sends, "{kind}");
            assert_eq!(codec_caps.standard_frames, caps.standard_frames, "{kind}");
            let refused = |r: Result<Vec<u8>, Refused>| r.err();
            assert_eq!(codec.send(&extended).is_ok(), caps.raw_sends, "{kind}");
            assert_eq!(
                codec.send(&standard).is_ok(),
                caps.raw_sends && caps.standard_frames,
                "{kind}"
            );
            if !caps.raw_sends {
                assert_eq!(
                    refused(codec.send(&extended)),
                    Some(Refused::ReceiveOnly),
                    "{kind}"
                );
            }
        }
        // A YDNU-02 on its serial port, as a YDWG-02.
        let ydnu = Device::new(Kind::YachtDevices, "/dev/ttyUSB0");
        assert!(ydnu.raw_codec().unwrap().send(&extended).is_err());
    }

    #[test]
    fn a_kind_refuses_the_protocols_it_cannot_carry() {
        for &kind in Kind::ALL {
            let caps = kind.capabilities();
            for &protocol in BusProtocol::ALL {
                let refused = Device::new(kind, reachable(kind))
                    .check(Some(protocol))
                    .is_err_and(|e| e.kind() == io::ErrorKind::InvalidInput);
                assert_eq!(
                    refused,
                    !caps.protocols.contains(&protocol),
                    "{kind} {protocol}"
                );
            }
        }
    }

    #[test]
    fn a_setting_the_kind_lacks_is_refused() {
        let refused = |dev: Device| dev.check(None).expect_err("refused").to_string();
        let why = refused(Device::new(Kind::Canalyst, "usb").mode(OperatingMode::NgTransferNormal));
        assert!(why.contains("canalyst") && why.contains("mode"), "{why}");
        assert!(
            refused(Device::new(Kind::YachtDevices, "tcp://ydwg").bitrate(250_000))
                .contains("bitrate")
        );
        assert!(
            refused(Device::new(Kind::NGT1, "/dev/ttyUSB0").source_address(3)).contains("node")
        );
        assert!(
            Device::new(Kind::Aos, "/dev/ttyUSB0")
                .source_address(3)
                .check(None)
                .is_ok()
        );
        // Every kind takes the PGN lists, and what a quirk sends.
        for &kind in Kind::ALL {
            let dev = Device::new(kind, "x")
                .pgn_lists(PgnLists {
                    tx: vec![127250],
                    rx: vec![],
                })
                .extra_tx_pgns(vec![127258]);
            assert!(!dev.given.iter().any(|&s| !kind.takes(s)), "{kind}");
        }
    }

    #[test]
    fn a_bit_rate_the_adapter_lacks_is_refused() {
        let why = Device::new(Kind::UsbCanA, "/dev/ttyUSB0")
            .bitrate(300_000)
            .check(None)
            .expect_err("refused")
            .to_string();
        assert!(why.contains("300000") && why.contains("250000"), "{why}");
        assert!(
            Device::new(Kind::Canalyst, "usb")
                .bitrate(500_000)
                .check(None)
                .is_ok()
        );
        assert!(
            Device::new(Kind::Canalyst, "usb")
                .bitrate(300_000)
                .check(None)
                .is_err()
        );
        assert!(
            Device::new(Kind::SocketCan, "can0")
                .bitrate(300_000)
                .check(None)
                .is_ok()
        );
    }

    #[test]
    fn a_binary_gateway_is_not_opened_over_udp() {
        for kind in [
            Kind::Actisense(ActisenseFormat::N2kActisense),
            Kind::Actisense(ActisenseFormat::RawActisense),
            Kind::NGT1,
            Kind::Maretron,
        ] {
            let e = Device::new(kind, "udp://192.0.2.1:60002")
                .check(Some(BusProtocol::Nmea2000))
                .expect_err("refused");
            assert_eq!(e.kind(), io::ErrorKind::InvalidInput, "{e}");
        }
        assert!(
            Device::new(Kind::Canalyst, "/dev/ttyUSB0")
                .check(None)
                .is_err()
        );
        assert!(Device::new(Kind::SocketCan, "tcp://x").check(None).is_err());
        assert!(Device::new(Kind::Maretron, "ipg:6543").check(None).is_ok());
    }

    #[test]
    fn a_bare_udp_port_only_receives() {
        let caps = |kind, address| Device::new(kind, address).capabilities();
        assert!(!caps(Kind::YachtDevices, "udp://1457").sends);
        assert!(caps(Kind::YachtDevices, "udp://ydwg:1457").sends);
        assert!(caps(Kind::YachtDevices, "tcp://ydwg").sends);
        assert!(!caps(Kind::Ikonvert, "udp://navlink:2000").sends);
        assert!(!caps(Kind::Actisense(ActisenseFormat::RawActisense), "udp://1").raw_sends);
    }

    #[cfg(feature = "io")]
    #[test]
    fn the_node_defaults_are_the_node_s() {
        let ours = Device::new(Kind::SocketCan, "can0").node_config(BusProtocol::Nmea2000);
        let theirs = super::super::socketcan::Config::default();
        assert_eq!(
            (
                ours.address,
                ours.unique,
                ours.manufacturer,
                ours.system_instance,
                ours.heartbeat_ms,
                ours.no_claim,
                ours.learn_tx_pgns,
                ours.bitrate,
                ours.configure_link,
            ),
            (
                theirs.address,
                theirs.unique,
                theirs.manufacturer,
                theirs.system_instance,
                theirs.heartbeat_ms,
                theirs.no_claim,
                theirs.learn_tx_pgns,
                theirs.bitrate,
                theirs.configure_link,
            )
        );
    }

    #[cfg(feature = "io")]
    #[test]
    fn the_pgn_lists_are_reported_as_the_gateway_handles_them() {
        let lists = PgnLists {
            tx: vec![127250],
            rx: vec![129025],
        };
        let status = |kind, address| {
            Device::new(kind, address)
                .pgn_lists(lists.clone())
                .pgn_list_status()
                .expect("lists were given")
        };
        assert_eq!(status(Kind::SocketCan, "can0").tx, PgnListSupport::Answered);
        assert_eq!(
            status(Kind::Ikonvert, "/dev/ttyUSB0").tx,
            PgnListSupport::Pushed
        );
        assert_eq!(
            status(Kind::Ikonvert, "udp://2000").tx,
            PgnListSupport::Unsupported
        );
        assert_eq!(
            status(Kind::NGT1, "/dev/ttyUSB0").rx,
            PgnListSupport::Unsupported
        );
        assert_eq!(
            status(Kind::Maretron, "ipg").tx,
            PgnListSupport::Unsupported
        );
        assert!(
            Device::new(Kind::SocketCan, "can0")
                .pgn_list_status()
                .is_none()
        );
    }
}
