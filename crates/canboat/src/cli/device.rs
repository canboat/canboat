// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The gateway options `canboat interface` and `canboat server` share:
//! which kind of gateway (`--kind`), and how to run it. One set of flags,
//! so the two commands cannot drift apart. An option that only some
//! gateways have says which in its help, and is refused for the others.

use anyhow::{Result, bail};

use crate::engine::BusProtocol;
use crate::engine::format::ngt1::OperatingMode;
use crate::io::device::gateway::{ActisenseFormat, GatewayKind};

/// The gateway to talk to, as `--kind` and `--data-format` resolve it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Actisense NGT-1, in NGT Actisense (BST-93/94).
    Ngt1,
    /// The AOS NMEA 2000/0183 Simulator, in NGT Actisense, with canboat's
    /// node.
    Aos,
    Ikonvert,
    Maretron,
    Socketcan,
    Canalyst,
    /// Waveshare USB-CAN-A, on its serial port.
    UsbCanA,
    /// Yacht Devices, or an Actisense in another data format: the gateways
    /// both commands open the same way.
    Gateway(GatewayKind),
}

impl Kind {
    /// Default serial baud of the NGT-1 and the iKonvert.
    pub fn default_baud(self) -> u32 {
        match self {
            Kind::Ngt1 | Kind::Aos => 115_200,
            Kind::Ikonvert => 230_400,
            Kind::UsbCanA => crate::io::device::usb_can_a::BAUD,
            _ => self.gateway().map_or(0, GatewayKind::default_baud),
        }
    }

    /// The line and network gateways, which both commands open the same way.
    pub fn gateway(self) -> Option<GatewayKind> {
        match self {
            Kind::Gateway(kind) => Some(kind),
            _ => None,
        }
    }

    /// The gateways canboat drives as the CAN controller, running its own
    /// node on them.
    pub fn is_can_controller(self) -> bool {
        matches!(self, Kind::Socketcan | Kind::Canalyst | Kind::UsbCanA)
    }

    /// Whether the gateway passes raw CAN frames, so it can carry a bus
    /// other than NMEA 2000.
    pub fn passes_raw_frames(self) -> bool {
        self.is_can_controller() || self.gateway().is_some_and(GatewayKind::passes_raw_frames)
    }

    /// How the command line names it, for messages.
    pub fn name(self) -> &'static str {
        match self {
            Kind::Ngt1 => "actisense (ngt-actisense)",
            Kind::Aos => "aos",
            Kind::Ikonvert => "ikonvert",
            Kind::Maretron => "maretron",
            Kind::Socketcan => "socketcan",
            Kind::Gateway(GatewayKind::YachtDevices) => "yachtd",
            Kind::Gateway(GatewayKind::Miniplex) => "miniplex",
            Kind::Gateway(GatewayKind::SeaSmart) => "seasmart",
            Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kAscii)) => {
                "actisense (n2k-ascii)"
            }
            Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kActisense)) => {
                "actisense (n2k-actisense)"
            }
            Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::RawActisense)) => {
                "actisense (raw-actisense)"
            }
            Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::RawAscii)) => {
                "actisense (raw-ascii)"
            }
            Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::Mxpgn)) => "actisense (mxpgn)",
            Kind::Canalyst => "canalyst",
            Kind::UsbCanA => "usb-can-a",
        }
    }
}

/// What `--kind` takes: a family of gateways, or one of the old names,
/// which also implies a data format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KindArg {
    name: &'static str,
    family: Family,
    format: Option<DataFormat>,
    /// One of the old names: parsed, but not listed.
    old: bool,
    help: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Family {
    Actisense,
    Yachtd,
    Miniplex,
    SeaSmart,
    Ikonvert,
    Maretron,
    Socketcan,
    Canalyst,
    UsbCanA,
    Aos,
}

const fn kind_arg(name: &'static str, family: Family, help: &'static str) -> KindArg {
    KindArg {
        name,
        family,
        format: None,
        old: false,
        help,
    }
}

const fn old_kind(name: &'static str, family: Family, format: Option<DataFormat>) -> KindArg {
    KindArg {
        name,
        family,
        format,
        old: true,
        help: "",
    }
}

const KINDS: &[KindArg] = &[
    kind_arg(
        "actisense",
        Family::Actisense,
        "Actisense NGT-1, W2K-1, PRO-NDC-1E2K, NGX; see --data-format",
    ),
    kind_arg(
        "yachtd",
        Family::Yachtd,
        "Yacht Devices YDWG-02, YDEN-02, YDNR-02, YDNU-02, in RAW ASCII",
    ),
    kind_arg(
        "miniplex",
        Family::Miniplex,
        "ShipModul MiniPlex-3 with NMEA 2000, in $MXPGN",
    ),
    kind_arg(
        "seasmart",
        Family::SeaSmart,
        "Chetco SeaSmart, Quark-elec, Yacht Devices in NMEA 0183, in $PCDIN",
    ),
    kind_arg("ikonvert", Family::Ikonvert, "Digital Yacht iKonvert"),
    kind_arg("maretron", Family::Maretron, "Maretron IPG100/200"),
    kind_arg(
        "socketcan",
        Family::Socketcan,
        "Linux SocketCAN interface; canboat is a full NMEA 2000 node on it",
    ),
    kind_arg(
        "canalyst",
        Family::Canalyst,
        "CANalyst-II (Waveshare USB-CAN-B) over usb; canboat is the node",
    ),
    kind_arg(
        "usb-can-a",
        Family::UsbCanA,
        "Waveshare USB-CAN-A on its serial port; canboat is the node",
    ),
    kind_arg(
        "aos",
        Family::Aos,
        "AOS NMEA 2000/0183 Simulator, in NGT Actisense; canboat is the node",
    ),
    // The names canboat 8.3.0 took.
    old_kind("ngt1", Family::Actisense, Some(DataFormat::NgtActisense)),
    old_kind("w2k-ascii", Family::Actisense, Some(DataFormat::N2kAscii)),
    old_kind("ydwg", Family::Yachtd, Some(DataFormat::RawAscii)),
];

impl clap::ValueEnum for KindArg {
    fn value_variants<'a>() -> &'a [Self] {
        KINDS
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        let value = clap::builder::PossibleValue::new(self.name).hide(self.old);
        Some(if self.help.is_empty() {
            value
        } else {
            value.help(self.help)
        })
    }
}

impl KindArg {
    /// The gateway this and `--data-format` name.
    pub fn resolve(self, data_format: Option<DataFormat>) -> Result<Kind> {
        let format = match (self.format, data_format) {
            (Some(implied), Some(given)) if implied != given => bail!(
                "--kind {} is --data-format {}, not {}",
                self.name,
                implied.name(),
                given.name()
            ),
            (implied, given) => implied.or(given),
        };
        let plain = |kind: Kind| match format {
            Some(f) => bail!(
                "--data-format {}: --kind {} has no data formats",
                f.name(),
                self.name
            ),
            None => Ok(kind),
        };
        match self.family {
            Family::Actisense => Ok(match format.unwrap_or(DataFormat::NgtActisense) {
                DataFormat::NgtActisense => Kind::Ngt1,
                DataFormat::N2kAscii => {
                    Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kAscii))
                }
                DataFormat::N2kActisense => {
                    Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kActisense))
                }
                DataFormat::RawAscii => {
                    Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::RawAscii))
                }
                DataFormat::RawActisense => {
                    Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::RawActisense))
                }
                DataFormat::Mxpgn => Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::Mxpgn)),
            }),
            Family::Yachtd => match format {
                None | Some(DataFormat::RawAscii) => Ok(Kind::Gateway(GatewayKind::YachtDevices)),
                Some(f) => bail!(
                    "--data-format {}: canboat speaks only raw-ascii to Yacht Devices gateways",
                    f.name()
                ),
            },
            Family::Miniplex => plain(Kind::Gateway(GatewayKind::Miniplex)),
            Family::SeaSmart => plain(Kind::Gateway(GatewayKind::SeaSmart)),
            Family::Ikonvert => plain(Kind::Ikonvert),
            Family::Maretron => plain(Kind::Maretron),
            Family::Socketcan => plain(Kind::Socketcan),
            Family::Canalyst => plain(Kind::Canalyst),
            Family::UsbCanA => plain(Kind::UsbCanA),
            Family::Aos => plain(Kind::Aos),
        }
    }
}

/// The data formats an Actisense gateway sends and takes, by Actisense's
/// names. A Yacht Devices gateway speaks raw-ascii.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum DataFormat {
    /// NGT Actisense: the NGT-1's binary format (BST-93/94). The default.
    NgtActisense,
    /// N2K ASCII: one reassembled NMEA 2000 message per line.
    N2kAscii,
    /// N2K Actisense (BST-D0): reassembled NMEA 2000 messages, binary.
    N2kActisense,
    /// RAW ASCII: one CAN frame per line, as Yacht Devices sends it.
    RawAscii,
    /// RAW Actisense (BST-95): CAN frames, binary.
    RawActisense,
    /// NMEA 0183 with `$MXPGN`: CAN frames, the data in bus order, as a
    /// W2K-2's CANDIRECT data server sends them.
    Mxpgn,
}

impl DataFormat {
    /// The name `--data-format` takes.
    pub fn name(self) -> &'static str {
        match self {
            DataFormat::NgtActisense => "ngt-actisense",
            DataFormat::N2kAscii => "n2k-ascii",
            DataFormat::N2kActisense => "n2k-actisense",
            DataFormat::RawAscii => "raw-ascii",
            DataFormat::RawActisense => "raw-actisense",
            DataFormat::Mxpgn => "mxpgn",
        }
    }
}

/// The NGT-1 operating modes `--mode` offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum Ngt1Mode {
    RxAll,
    Normal,
}

impl From<Ngt1Mode> for OperatingMode {
    fn from(mode: Ngt1Mode) -> Self {
        match mode {
            Ngt1Mode::RxAll => OperatingMode::NgTransferRxAll,
            Ngt1Mode::Normal => OperatingMode::NgTransferNormal,
        }
    }
}

/// Help headings, in the order they appear.
pub const GATEWAY: &str = "Gateway";
pub const CAN: &str = "CAN controller (socketcan, canalyst, usb-can-a)";
pub const SPECIFIC: &str = "Gateway-specific";
pub const LISTS: &str = "PGN lists";
pub const NODE: &str = "canboat's node (socketcan, canalyst, usb-can-a, aos)";
pub const BUS: &str = "Bus";

/// The end of `--help` for both commands: the gateways and their data
/// formats, what canboat does with each, which options only some take,
/// and what each does with the PGN lists. Kept under 80 columns.
pub const GATEWAY_HELP: &str = "\
Gateways (--kind) and their data formats (--data-format):

  KIND       --data-format   DEVICE                       R/W  J1939  11-bit
  actisense  ngt-actisense*  /dev/ttyUSB0, usb, tcp:6001  rw   -      -
             n2k-ascii       tcp:60002, udp               rw   -      -
             n2k-actisense   tcp:60002                    rw   -      -
             raw-ascii       tcp:60002, udp               rw   yes    -
             raw-actisense   tcp:60002, /dev/tty… (NGX)   rw   yes    -
             mxpgn           tcp:20000, udp               rw   yes    -
  yachtd     raw-ascii*      tcp:1457, udp, /dev/tty…     rw   yes    -
  miniplex                   tcp:10110, udp, /dev/tty…    rw   yes    -
  seasmart                   tcp:10001, udp, /dev/tty…    rw   -      -
  ikonvert                   /dev/ttyUSB0, tcp:2000, udp  rw   -      -
  maretron                   host[:6543]                  rw   -      -
  aos                        /dev/ttyUSB0, usb, tcp:6001  rw   -      -
  socketcan                  can0                         rw   yes    yes
  canalyst                   usb[:VVVV:PPPP]              rw   yes    yes
  usb-can-a                  /dev/ttyUSB0                 rw   yes    yes

  * the default. rw: canboat reads from and writes to the bus; a
  udp://[bind:]port only reads. J1939 (--protocol j1939): where it says
  yes; ISO TP messages are received there, but sent only on socketcan,
  canalyst and usb-can-a, where canboat is the node.
  11-bit: Quick PCS (--protocol quick). The others carry NMEA 2000 only.
  A W2K-1 sends ngt-actisense but does not take it; canboat does not
  know that yet. seasmart ($PCDIN) has no priority or destination, and
  not every gateway takes it to send: Yacht Devices only a few PGNs.
  actisense mxpgn: $MXPGN with the data in bus order, unlike a
  MiniPlex's (--kind miniplex). A W2K-2 sends it only on its hidden
  CANDIRECT data server (port 20000) set to NMEA 0183; its own data
  servers' NMEA 0183 has no $MXPGN. A W2K-1 is not known to send it.
  ikonvert: tcp:// is the better mode for a NavLink2. Over
  udp://[bind:][port] (its UDP output, port 2000) canboat only
  receives: it expects the gateway to be on the bus already, put there
  by another client, and never takes it off. Its PGN lists and rate
  limit stay as they are.

  aos is the Across Ocean Systems simulator, which speaks the NGT-1's
  format but is no node: canboat claims an address for it. actisense
  (ngt-actisense) and aos each give up on a gateway that turns out to be
  the other. The names of canboat 8.3.0 still work: --kind ngt1,
  w2k-ascii, ydwg.

Options only some gateways take:

  --baud              a gateway on a serial port
  --mode              actisense (ngt-actisense)
  --password          maretron
  --no-tx-rate-limit  ikonvert
  --ignore-checksum   seasmart, miniplex
  --bitrate           socketcan, canalyst, usb-can-a
  --channel           canalyst
  node options        socketcan, canalyst, usb-can-a, aos

What each gateway does with the PGN lists:

                   --rx-pgn             --tx-pgn
  ikonvert         stored, advertised   stored, advertised; only these
                                        are sent
  actisense        ignored              Transmit PGN Enable list; only
  (ngt-actisense)                       these are sent
  aos, socketcan,  advertised           advertised
  canalyst,
  usb-can-a
  others           ignored              ignored

  --rx-pgn never filters; --rx-filter does, in the gateway on ikonvert
  and in canboat elsewhere. Network management PGNs always pass, and are
  always sent.";

/// How to run the gateway. Options that have a default show it in their
/// help but are `None` when not given, so one given for a gateway that
/// does not have it can be refused.
#[derive(Debug, Clone, clap::Args)]
pub struct DeviceArgs {
    /// actisense: the data format it is set to [default: ngt-actisense].
    ///
    /// By Actisense's names; see the table at the end. A Yacht Devices
    /// gateway speaks raw-ascii.
    #[arg(long, value_enum, value_name = "FORMAT", help_heading = GATEWAY)]
    pub data_format: Option<DataFormat>,

    /// Serial baud rate [default: 115200, ikonvert 230400, YDNU-02 38400].
    // `-s` is the C actisense-serial/ikonvert-serial spelling; the argv[0]
    // shims must stay drop-in compatible with it.
    #[arg(short = 'b', long, short_alias = 's', value_name = "BAUD", help_heading = GATEWAY)]
    pub baud: Option<u32>,

    /// The CAN bus bit rate [default: canalyst, usb-can-a 250000; socketcan as configured].
    ///
    /// On socketcan this brings the interface down, sets the rate and brings
    /// it up again, which needs root or CAP_NET_ADMIN. 250000 for NMEA 2000,
    /// 500000 for a J1939-14 bus. Not the serial speed to a gateway such as
    /// the NGT-1: that is --baud.
    #[arg(long, value_name = "BIT/S", help_heading = CAN)]
    pub bitrate: Option<u32>,

    /// canalyst: the CAN channel, 0 or 1 [default: 0].
    #[arg(long, value_name = "N", help_heading = CAN)]
    pub channel: Option<u8>,

    /// maretron: the IPG login password [default: empty].
    #[arg(long, alias = "maretron-password", value_name = "PASSWORD", help_heading = SPECIFIC)]
    pub password: Option<String>,

    /// ngt1: the operating mode [default: rx-all].
    ///
    /// `rx-all` ("NGT Transfer Rx All Mode") passes on every PGN received;
    /// `normal` ("NGT Transfer Normal Mode") only those on the Receive PGN
    /// Enable list the gateway holds.
    #[arg(long, value_enum, help_heading = SPECIFIC)]
    pub mode: Option<Ngt1Mode>,

    /// ikonvert: turn off the gateway's transmit rate limit, at your own risk.
    #[arg(
        long,
        visible_alias = "rate-limit-off",
        alias = "ikonvert-rate-limit-off",
        help_heading = SPECIFIC
    )]
    pub no_tx_rate_limit: bool,

    /// seasmart, miniplex: accept $PCDIN or $MXPGN sentences whose checksum
    /// is wrong, as some gateways may send them.
    #[arg(long, help_heading = SPECIFIC)]
    pub ignore_checksum: bool,

    /// PGNs the application receives, to advertise; never a filter.
    ///
    /// Repeatable, or comma-separated. Answered in PGN 126464 by canboat's
    /// node or stored in an iKonvert; see the table at the end.
    #[arg(long = "rx-pgn", value_name = "PGN", value_delimiter = ',', help_heading = LISTS)]
    pub rx_pgn: Vec<u32>,

    /// Pass on only these PGNs (and network management); drop the rest.
    ///
    /// Repeatable, or comma-separated. An iKonvert filters itself (it runs
    /// in `NORMAL` mode, and the list is advertised as with --rx-pgn); for
    /// every other gateway canboat drops the rest as they arrive. The
    /// network-management PGNs (address claims, requests, product
    /// information, …) and canboat's own gateway status always pass.
    #[arg(
        long = "rx-filter",
        alias = "rx",
        alias = "ikonvert-rx",
        value_name = "PGN",
        value_delimiter = ',',
        help_heading = LISTS
    )]
    pub rx_filter: Vec<u32>,

    /// PGNs the application transmits; on ikonvert and ngt1 the only ones sent.
    ///
    /// Repeatable, or comma-separated. See the table at the end.
    ///
    /// WARNING — WITH ikonvert OR ngt1, THIS IS THE WHOLE TRANSMIT
    /// LIST: ONCE ANY --tx-pgn IS GIVEN, EVERY OTHER PGN IS REFUSED (NOT
    /// SENT), apart from network management. List every PGN you will send.
    #[arg(
        long = "tx-pgn",
        alias = "tx",
        alias = "ikonvert-tx",
        value_name = "PGN",
        value_delimiter = ',',
        help_heading = LISTS
    )]
    pub tx_pgn: Vec<u32>,

    /// The source address to claim; a free one if it is taken [default: 0].
    #[arg(short = 'a', long, alias = "socketcan-address", value_name = "ADDR", help_heading = NODE)]
    pub address: Option<u8>,

    /// Claim no address: listen only, or on aos send from the simulator's own.
    ///
    /// On socketcan and canalyst a passive sniff. On aos canboat runs no
    /// node, and the simulator sends from its fixed address without
    /// claiming it.
    #[arg(short = 'n', long, help_heading = NODE)]
    pub no_claim: bool,

    /// ISO NAME unique number [default: derived from the machine id].
    ///
    /// The default is stable across restarts; on a machine that can't be
    /// identified it is a random one, stored with the state files.
    #[arg(short = 'u', long, value_name = "N", help_heading = NODE)]
    pub unique: Option<u32>,

    /// ISO NAME manufacturer code [default: 999, Signal K].
    #[arg(short = 'm', long, value_name = "N", help_heading = NODE)]
    pub manufacturer: Option<u16>,

    /// ISO NAME System Instance, 0..15 [default: 15].
    ///
    /// 15 makes canboat's NAME lose to real hardware's, so it yields an
    /// address rather than steal one.
    #[arg(
        long,
        value_name = "N",
        value_parser = clap::value_parser!(u8).range(0..=15),
        help_heading = NODE
    )]
    pub system_instance: Option<u8>,

    /// Heartbeat (PGN 126993) interval in ms, up to 65532; 0: none [default: 60000].
    #[arg(
        long,
        alias = "hb",
        value_name = "MS",
        value_parser = clap::value_parser!(u64).range(0..=65_532),
        help_heading = NODE
    )]
    pub heartbeat: Option<u64>,

    /// Do not add PGNs to the advertised Transmit list as they are first sent.
    #[arg(long, help_heading = NODE)]
    pub no_learn_tx_pgns: bool,

    #[command(flatten, next_help_heading = BUS)]
    pub protocol: crate::cli::protocol::ProtocolArgs,
}

impl DeviceArgs {
    /// Refuse what does not apply to a `kind` gateway, and a protocol it
    /// cannot carry. `kind` is `None` for a source that is not a gateway
    /// (a `canboat server` on stdin, a replay or another server): then
    /// only the protocol is checked.
    pub fn check(&self, kind: Option<Kind>) -> Result<()> {
        let protocol = self.protocol.resolve()?;
        let Some(kind) = kind else {
            return self.refuse_all_but(&[], "the source is not a gateway");
        };
        let name = kind.name();
        if protocol == BusProtocol::Quick && !kind.is_can_controller() {
            bail!(
                "--protocol quick needs socketcan, canalyst or usb-can-a: the other gateways pass only 29-bit frames"
            );
        }
        if protocol != BusProtocol::Nmea2000 && !kind.passes_raw_frames() {
            bail!(
                "--protocol {protocol} needs a gateway that passes raw CAN frames \
                 (socketcan, canalyst, usb-can-a, yachtd, miniplex, or actisense in raw-ascii or raw-actisense); \
                 {name} does NMEA 2000 framing itself"
            );
        }
        let allowed: &[&str] = match kind {
            Kind::Ngt1 => &["mode"],
            Kind::Aos => &["node", "no-claim"],
            Kind::Ikonvert => &["no-tx-rate-limit"],
            Kind::Maretron => &["password"],
            Kind::Socketcan => &["bitrate", "node", "no-claim"],
            Kind::Canalyst => &["bitrate", "channel", "node", "no-claim"],
            Kind::UsbCanA => &["bitrate", "node", "no-claim"],
            Kind::Gateway(GatewayKind::SeaSmart | GatewayKind::Miniplex) => &["ignore-checksum"],
            _ => &[],
        };
        let why = if kind == Kind::Ngt1 {
            // The node options were the AOS simulator's, on ngt1.
            format!("--kind {name} does not have it (an AOS simulator is --kind aos)")
        } else {
            format!("--kind {name} does not have it")
        };
        self.refuse_all_but(allowed, &why)
    }

    /// Bail on the first option given that is not in `allowed`; `node`
    /// stands for the ISO NAME options.
    fn refuse_all_but(&self, allowed: &[&str], why: &str) -> Result<()> {
        let given = [
            ("bitrate", "--bitrate", self.bitrate.is_some()),
            ("channel", "--channel", self.channel.is_some()),
            ("password", "--password", self.password.is_some()),
            ("mode", "--mode", self.mode.is_some()),
            (
                "no-tx-rate-limit",
                "--no-tx-rate-limit",
                self.no_tx_rate_limit,
            ),
            ("ignore-checksum", "--ignore-checksum", self.ignore_checksum),
            ("node", "--address", self.address.is_some()),
            ("no-claim", "--no-claim", self.no_claim),
            ("node", "--unique", self.unique.is_some()),
            ("node", "--manufacturer", self.manufacturer.is_some()),
            ("node", "--system-instance", self.system_instance.is_some()),
            ("node", "--heartbeat", self.heartbeat.is_some()),
            ("node", "--no-learn-tx-pgns", self.no_learn_tx_pgns),
        ];
        for (group, flag, set) in given {
            if set && !allowed.contains(&group) {
                if flag == "--bitrate" {
                    bail!(
                        "--bitrate sets the CAN bus rate of socketcan, canalyst or usb-can-a, which canboat \
                         drives as the CAN controller ({why}; for the serial speed to a gateway, \
                         use --baud)"
                    );
                }
                bail!("{flag}: {why}");
            }
        }
        Ok(())
    }

    /// What the bus carries. Call [`Self::check`] first.
    pub fn protocol(&self) -> BusProtocol {
        self.protocol
            .resolve()
            .expect("DeviceArgs::check rejects a --protocol this cannot convert")
    }

    /// The node of a socketcan, canalyst or usb-can-a gateway, or of an AOS simulator
    /// on ngt1, without the per-gateway parts (model version, bit rate).
    pub fn node_config(&self) -> crate::io::device::socketcan::Config {
        let defaults = crate::io::device::socketcan::Config::default();
        crate::io::device::socketcan::Config {
            address: self.address.unwrap_or(defaults.address),
            unique: self.unique.unwrap_or(defaults.unique),
            manufacturer: self.manufacturer.unwrap_or(defaults.manufacturer),
            system_instance: self.system_instance.unwrap_or(defaults.system_instance),
            heartbeat_ms: self.heartbeat.unwrap_or(defaults.heartbeat_ms),
            no_claim: self.no_claim,
            learn_tx_pgns: !self.no_learn_tx_pgns,
            pgn_lists: self.pgn_lists(),
            protocol: self.protocol(),
            ..defaults
        }
    }

    /// `--rx-pgn` and `--tx-pgn`.
    pub fn pgn_lists(&self) -> crate::engine::pgn_list::PgnLists {
        crate::engine::pgn_list::PgnLists {
            tx: self.tx_pgn.clone(),
            rx: self.rx_pgn.clone(),
        }
    }

    /// The iKonvert receive filter, `--rx-filter`, which puts the gateway
    /// in `NORMAL` mode.
    pub fn ikonvert_rx_list(&self) -> Option<String> {
        (!self.rx_filter.is_empty()).then(|| {
            self.rx_filter
                .iter()
                .map(u32::to_string)
                .collect::<Vec<_>>()
                .join(",")
        })
    }

    /// The NGT-1 operating mode.
    pub fn operating_mode(&self) -> OperatingMode {
        self.mode.unwrap_or(Ngt1Mode::RxAll).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[derive(Parser)]
    struct Cli {
        #[command(flatten)]
        device: DeviceArgs,
    }

    fn parse(args: &[&str]) -> DeviceArgs {
        Cli::try_parse_from([&["x"], args].concat()).unwrap().device
    }

    #[test]
    fn the_old_spellings_still_parse() {
        let d = parse(&[
            "--maretron-password",
            "pw",
            "--ikonvert-rx",
            "129025,129026",
            "--tx",
            "127508",
            "--rate-limit-off",
            "--socketcan-address",
            "7",
        ]);
        assert_eq!(d.password.as_deref(), Some("pw"));
        assert_eq!(d.rx_filter, [129025, 129026]);
        assert!(d.rx_pgn.is_empty());
        assert_eq!(d.tx_pgn, [127508]);
        assert!(d.no_tx_rate_limit);
        assert_eq!(d.address, Some(7));
        assert_eq!(d.ikonvert_rx_list().as_deref(), Some("129025,129026"));
    }

    #[test]
    fn an_option_the_gateway_lacks_is_refused() {
        assert!(
            parse(&["--bitrate", "500000"])
                .check(Some(Kind::Socketcan))
                .is_ok()
        );
        let err = parse(&["--bitrate", "500000"])
            .check(Some(Kind::Ngt1))
            .unwrap_err();
        assert!(err.to_string().contains("--baud"), "{err}");
        assert!(
            parse(&["--password", "x"])
                .check(Some(Kind::Ikonvert))
                .is_err()
        );
        assert!(
            parse(&["--channel", "1"])
                .check(Some(Kind::Socketcan))
                .is_err()
        );
        for kind in [GatewayKind::SeaSmart, GatewayKind::Miniplex] {
            assert!(
                parse(&["--ignore-checksum"])
                    .check(Some(Kind::Gateway(kind)))
                    .is_ok()
            );
        }
        assert!(
            parse(&["--ignore-checksum"])
                .check(Some(Kind::Gateway(GatewayKind::YachtDevices)))
                .is_err()
        );
        assert!(parse(&["--mode", "normal"]).check(Some(Kind::Ngt1)).is_ok());
        assert!(
            parse(&["--mode", "normal"])
                .check(Some(Kind::Gateway(GatewayKind::YachtDevices)))
                .is_err()
        );
        // The node options: the CAN controllers and the AOS simulator.
        let err = parse(&["-a", "9"]).check(Some(Kind::Ngt1)).unwrap_err();
        assert!(err.to_string().contains("--kind aos"), "{err}");
        for kind in [Kind::Socketcan, Kind::Canalyst, Kind::UsbCanA, Kind::Aos] {
            assert!(
                parse(&["-a", "9", "-u", "1", "-n"])
                    .check(Some(kind))
                    .is_ok()
            );
        }
        assert!(parse(&["-a", "9"]).check(Some(Kind::Ikonvert)).is_err());
        assert!(parse(&["-a", "9"]).check(None).is_err());
        for kind in [Kind::Ikonvert, Kind::Socketcan, Kind::Ngt1] {
            assert!(parse(&["--rx", "129025"]).check(Some(kind)).is_ok());
        }
        assert!(parse(&["--rx-filter", "129025"]).check(None).is_ok());
        // The PGN lists apply everywhere; a gateway without them warns.
        assert!(
            parse(&["--tx-pgn", "127508"])
                .check(Some(Kind::Maretron))
                .is_ok()
        );
    }

    #[test]
    fn kind_and_data_format_resolve() {
        use clap::ValueEnum;
        let kind = |name: &str| KindArg::from_str(name, false).unwrap();
        use DataFormat::*;
        for (name, format, want) in [
            ("actisense", None, Kind::Ngt1),
            ("actisense", Some(NgtActisense), Kind::Ngt1),
            (
                "actisense",
                Some(N2kAscii),
                Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kAscii)),
            ),
            (
                "actisense",
                Some(N2kActisense),
                Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kActisense)),
            ),
            (
                "actisense",
                Some(RawAscii),
                Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::RawAscii)),
            ),
            (
                "actisense",
                Some(RawActisense),
                Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::RawActisense)),
            ),
            ("yachtd", None, Kind::Gateway(GatewayKind::YachtDevices)),
            (
                "yachtd",
                Some(RawAscii),
                Kind::Gateway(GatewayKind::YachtDevices),
            ),
            ("seasmart", None, Kind::Gateway(GatewayKind::SeaSmart)),
            ("aos", None, Kind::Aos),
            // The old names imply their format.
            ("ngt1", None, Kind::Ngt1),
            (
                "w2k-ascii",
                Some(N2kAscii),
                Kind::Gateway(GatewayKind::Actisense(ActisenseFormat::N2kAscii)),
            ),
            ("ydwg", None, Kind::Gateway(GatewayKind::YachtDevices)),
        ] {
            assert_eq!(
                kind(name).resolve(format).unwrap(),
                want,
                "{name} {format:?}"
            );
        }
        assert!(kind("w2k-ascii").resolve(Some(RawAscii)).is_err());
        assert!(kind("yachtd").resolve(Some(N2kAscii)).is_err());
        assert!(kind("socketcan").resolve(Some(RawAscii)).is_err());
        // Only the new names are listed.
        let listed: Vec<_> = KindArg::value_variants()
            .iter()
            .filter_map(|k| k.to_possible_value())
            .filter(|v| !v.is_hide_set())
            .map(|v| v.get_name().to_owned())
            .collect();
        assert_eq!(
            listed,
            [
                "actisense",
                "yachtd",
                "miniplex",
                "seasmart",
                "ikonvert",
                "maretron",
                "socketcan",
                "canalyst",
                "usb-can-a",
                "aos"
            ]
        );
    }

    #[test]
    fn only_raw_frame_gateways_carry_j1939_and_quick() {
        let j1939 = parse(&["--protocol", "j1939"]);
        assert!(
            j1939
                .check(Some(Kind::Gateway(GatewayKind::YachtDevices)))
                .is_ok()
        );
        assert!(j1939.check(Some(Kind::Ngt1)).is_err());
        let quick = parse(&["--protocol", "quick"]);
        assert!(quick.check(Some(Kind::Canalyst)).is_ok());
        assert!(quick.check(Some(Kind::UsbCanA)).is_ok());
        assert!(j1939.check(Some(Kind::UsbCanA)).is_ok());
        assert!(
            quick
                .check(Some(Kind::Gateway(GatewayKind::Actisense(
                    ActisenseFormat::RawActisense
                ))))
                .is_err()
        );
    }
}
