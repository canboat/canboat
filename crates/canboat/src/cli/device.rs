// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The gateway options `canboat interface` and `canboat server` share:
//! which kind of gateway (`--kind`), and how to run it. One set of flags,
//! so the two commands cannot drift apart. An option that only some
//! gateways have says which in its help, and is refused for the others.

use anyhow::{Result, bail};

use crate::engine::BusProtocol;
use crate::engine::format::ngt1::OperatingMode;
use crate::io::device::gateway::GatewayKind;

/// Which gateway to talk to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
pub enum Kind {
    /// Actisense NGT-1 (NGT binary, BST-93/94); serial, usb, or tcp (port 6001).
    #[value(alias = "actisense")]
    Ngt1,
    /// AOS NMEA 2000/0183 Simulator, with canboat as the node; as ngt1. Not an NGT-1.
    Aos,
    /// Digital Yacht iKonvert; serial, or tcp (a NavLink2, port 6001).
    Ikonvert,
    /// Maretron IPG100/200; tcp (port 6543).
    Maretron,
    /// Linux SocketCAN interface; canboat is a full NMEA 2000 node on it.
    Socketcan,
    /// RAW ASCII: Yacht Devices YDWG-02/YDEN-02/YDNU-02; tcp (port 1457), udp, serial.
    #[value(name = "raw-ascii", alias = "ydwg")]
    Ydwg,
    /// Actisense N2K ASCII: W2K-1, PRO-NDC-1E2K; tcp (port 60002), udp.
    #[value(name = "n2k-ascii", alias = "w2k-ascii")]
    W2kAscii,
    /// Actisense BST-D0: W2K-1, PRO-NDC-1E2K in Actisense mode; tcp (port 60002).
    #[value(name = "bst-d0", alias = "w2k-actisense")]
    BstD0,
    /// Actisense BST-95 raw CAN frames: W2K-1, PRO-NDC-1E2K; tcp (60002); NGX serial.
    #[value(name = "bst-95")]
    Bst95,
    /// CANalyst-II (Waveshare USB-CAN-B) over usb; a full node, on every platform.
    #[value(name = "canalyst", alias = "usb-can-b")]
    Canalyst,
}

impl Kind {
    /// Default serial baud of the NGT-1 and the iKonvert.
    pub fn default_baud(self) -> u32 {
        match self {
            Kind::Ngt1 | Kind::Aos => 115_200,
            Kind::Ikonvert => 230_400,
            _ => self.gateway().map_or(0, GatewayKind::default_baud),
        }
    }

    /// The line and network gateways, which both commands open the same way.
    pub fn gateway(self) -> Option<GatewayKind> {
        match self {
            Kind::Ydwg => Some(GatewayKind::Ydwg),
            Kind::W2kAscii => Some(GatewayKind::W2kAscii),
            Kind::BstD0 => Some(GatewayKind::BstD0),
            Kind::Bst95 => Some(GatewayKind::Bst95),
            _ => None,
        }
    }

    /// The gateways canboat drives as the CAN controller, running its own
    /// node on them.
    pub fn is_can_controller(self) -> bool {
        matches!(self, Kind::Socketcan | Kind::Canalyst)
    }

    /// Whether the gateway passes raw CAN frames, so it can carry a bus
    /// other than NMEA 2000.
    pub fn passes_raw_frames(self) -> bool {
        self.is_can_controller() || self.gateway().is_some_and(GatewayKind::passes_raw_frames)
    }

    /// The name `--kind` takes.
    pub fn name(self) -> &'static str {
        match self {
            Kind::Ngt1 => "ngt1",
            Kind::Aos => "aos",
            Kind::Ikonvert => "ikonvert",
            Kind::Maretron => "maretron",
            Kind::Socketcan => "socketcan",
            Kind::Ydwg => "raw-ascii",
            Kind::W2kAscii => "n2k-ascii",
            Kind::BstD0 => "bst-d0",
            Kind::Bst95 => "bst-95",
            Kind::Canalyst => "canalyst",
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
pub const CAN: &str = "CAN controller (socketcan, canalyst)";
pub const SPECIFIC: &str = "Gateway-specific";
pub const LISTS: &str = "PGN lists";
pub const NODE: &str = "canboat's node (socketcan, canalyst, aos)";
pub const BUS: &str = "Bus";

/// The end of `--help` for both commands: which options each gateway
/// takes, and what it does with the PGN lists. Kept under 80 columns.
pub const GATEWAY_HELP: &str = "\
Gateways (--kind), where they are, and the options only they take:

  ngt1       /dev/ttyUSB0, usb[:…], tcp://…  --baud --mode
  aos        /dev/ttyUSB0, usb[:…], tcp://…  --baud, node options
  ikonvert   /dev/ttyUSB0, tcp://…           --baud --rx-filter
                                             --no-tx-rate-limit
  maretron   host[:port]                     --password
  socketcan  can0                            --bitrate, node options
  canalyst   usb[:VVVV:PPPP]                 --bitrate --channel,
                                             node options
  raw-ascii  tcp://…, udp://…, /dev/tty…     --baud (a YDNU-02)
  n2k-ascii  tcp://…, udp://…
  bst-d0     tcp://…
  bst-95     tcp://…, /dev/tty…              --baud (an NGX)

  aos is the Across Ocean Systems simulator, which speaks the NGT-1's
  framing but is no node: canboat claims an address for it. It gives up
  on a gateway that turns out to be an NGT-1.
  The old names still work: --kind ydwg, w2k-ascii, actisense.

What each gateway does with the PGN lists:

                   --rx-pgn             --tx-pgn
  ikonvert         stored, advertised   stored, advertised; only these
                                        are sent
  ngt1             ignored              Transmit PGN Enable list; only
                                        these are sent
  aos, socketcan,  advertised           advertised
  canalyst
  others           ignored              ignored

  --rx-pgn never filters; --rx-filter (ikonvert) does. Network
  management PGNs are always sent.";

/// How to run the gateway. Options that have a default show it in their
/// help but are `None` when not given, so one given for a gateway that
/// does not have it can be refused.
#[derive(Debug, Clone, clap::Args)]
pub struct DeviceArgs {
    /// Serial baud rate [default: 115200, ikonvert 230400, YDNU-02 38400].
    // `-s` is the C actisense-serial/ikonvert-serial spelling; the argv[0]
    // shims must stay drop-in compatible with it.
    #[arg(short = 'b', long, short_alias = 's', value_name = "BAUD", help_heading = GATEWAY)]
    pub baud: Option<u32>,

    /// The CAN bus bit rate [default: canalyst 250000; socketcan as configured].
    ///
    /// On socketcan this brings the interface down, sets the rate and brings
    /// it up again, which needs root or CAP_NET_ADMIN. 250000 for NMEA 2000,
    /// 500000 for a J1939-14 bus. Not the serial speed to a gateway such as
    /// the NGT-1: that is --baud.
    #[arg(long, value_name = "BIT/S", help_heading = CAN)]
    pub bitrate: Option<u32>,

    /// canalyst: the CAN channel, 0 or 1 [default: 0].
    #[arg(long, alias = "canalyst-channel", value_name = "N", help_heading = CAN)]
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

    /// PGNs the application receives, to advertise; never a filter.
    ///
    /// Repeatable, or comma-separated. Answered in PGN 126464 by canboat's
    /// node or stored in an iKonvert; see the table at the end.
    #[arg(long = "rx-pgn", value_name = "PGN", value_delimiter = ',', help_heading = LISTS)]
    pub rx_pgn: Vec<u32>,

    /// ikonvert: pass on only these PGNs.
    ///
    /// Repeatable, or comma-separated. The gateway runs in `NORMAL` mode and
    /// drops every other PGN; the list is advertised as with --rx-pgn. (An
    /// NGT-1 filters with --mode normal, on the list it already holds.)
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
                "--protocol quick needs socketcan or canalyst: the other gateways pass only 29-bit frames"
            );
        }
        if protocol != BusProtocol::Nmea2000 && !kind.passes_raw_frames() {
            bail!(
                "--protocol {protocol} needs a gateway that passes raw CAN frames \
                 (socketcan, canalyst, raw-ascii or bst-95); {name} does NMEA 2000 framing itself"
            );
        }
        let allowed: &[&str] = match kind {
            Kind::Ngt1 => &["mode"],
            Kind::Aos => &["node", "no-claim"],
            Kind::Ikonvert => &["no-tx-rate-limit", "rx-filter"],
            Kind::Maretron => &["password"],
            Kind::Socketcan => &["bitrate", "node", "no-claim"],
            Kind::Canalyst => &["bitrate", "channel", "node", "no-claim"],
            _ => &[],
        };
        let why = if kind == Kind::Ngt1 {
            // The node options were the AOS simulator's, on ngt1.
            "--kind ngt1 does not have it (an AOS simulator is --kind aos)".to_string()
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
            ("rx-filter", "--rx-filter", !self.rx_filter.is_empty()),
            (
                "no-tx-rate-limit",
                "--no-tx-rate-limit",
                self.no_tx_rate_limit,
            ),
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
                        "--bitrate sets the CAN bus rate of socketcan or canalyst, which canboat \
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

    /// The node of a socketcan or canalyst gateway, or of an AOS simulator
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
            "--canalyst-channel",
            "1",
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
        assert_eq!(d.channel, Some(1));
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
        assert!(parse(&["--mode", "normal"]).check(Some(Kind::Ngt1)).is_ok());
        assert!(
            parse(&["--mode", "normal"])
                .check(Some(Kind::Ydwg))
                .is_err()
        );
        // The node options: the CAN controllers and the AOS simulator.
        let err = parse(&["-a", "9"]).check(Some(Kind::Ngt1)).unwrap_err();
        assert!(err.to_string().contains("--kind aos"), "{err}");
        for kind in [Kind::Socketcan, Kind::Canalyst, Kind::Aos] {
            assert!(
                parse(&["-a", "9", "-u", "1", "-n"])
                    .check(Some(kind))
                    .is_ok()
            );
        }
        assert!(parse(&["-a", "9"]).check(Some(Kind::Ikonvert)).is_err());
        assert!(parse(&["-a", "9"]).check(None).is_err());
        assert!(
            parse(&["--rx", "129025"])
                .check(Some(Kind::Ikonvert))
                .is_ok()
        );
        assert!(
            parse(&["--rx-filter", "129025"])
                .check(Some(Kind::Socketcan))
                .is_err()
        );
        // The PGN lists apply everywhere; a gateway without them warns.
        assert!(
            parse(&["--tx-pgn", "127508"])
                .check(Some(Kind::Maretron))
                .is_ok()
        );
    }

    #[test]
    fn names_match_what_kind_takes() {
        use clap::ValueEnum;
        for kind in Kind::value_variants() {
            assert_eq!(kind.to_possible_value().unwrap().get_name(), kind.name());
        }
    }

    #[test]
    fn only_raw_frame_gateways_carry_j1939_and_quick() {
        let j1939 = parse(&["--protocol", "j1939"]);
        assert!(j1939.check(Some(Kind::Ydwg)).is_ok());
        assert!(j1939.check(Some(Kind::Ngt1)).is_err());
        let quick = parse(&["--protocol", "quick"]);
        assert!(quick.check(Some(Kind::Canalyst)).is_ok());
        assert!(quick.check(Some(Kind::Bst95)).is_err());
    }
}
