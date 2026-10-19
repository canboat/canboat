// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The gateway options `canboat interface` and `canboat server` share:
//! which kind of gateway (`--kind`), and how to run it. One set of flags,
//! so the two commands cannot drift apart. An option that only some
//! gateways have says which in its help, and is refused for the others.

use std::sync::LazyLock;

use anyhow::{Result, bail};

use crate::engine::BusProtocol;
use crate::engine::format::ngt1::OperatingMode;
use crate::io::device::kind::{ActisenseFormat, Kind, Setting};
use crate::io::device::open::Device;

use super::help;

/// What `--kind` takes: a kind's name, the first of its kinds standing
/// for them all (an Actisense gateway is `actisense`, whatever its
/// `--data-format`), or one of canboat 8.3.0's names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KindArg {
    name: &'static str,
    kind: Kind,
    /// One of the old names: parsed, but not listed.
    old: bool,
}

/// Every `--kind`, in [`Kind::ALL`]'s order, then the old names.
static KINDS: LazyLock<Vec<KindArg>> = LazyLock::new(|| {
    let mut kinds: Vec<KindArg> = Vec::new();
    for &kind in Kind::ALL {
        if !kinds.iter().any(|k| k.name == kind.name()) {
            kinds.push(KindArg {
                name: kind.name(),
                kind,
                old: false,
            });
        }
    }
    kinds.extend(Kind::OLD_NAMES.iter().map(|&(name, kind)| KindArg {
        name,
        kind,
        old: true,
    }));
    kinds
});

impl clap::ValueEnum for KindArg {
    fn value_variants<'a>() -> &'a [Self] {
        &KINDS
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        let value = clap::builder::PossibleValue::new(self.name).hide(self.old);
        Some(if self.old {
            value
        } else {
            value.help(crate::cli::help::kind_help(self.kind))
        })
    }
}

impl KindArg {
    /// The data format an old name implies.
    fn implied_format(self) -> Option<ActisenseFormat> {
        match self.kind {
            Kind::Actisense(format) if self.old => Some(format),
            Kind::YachtDevices if self.old => Some(ActisenseFormat::RawAscii),
            _ => None,
        }
    }

    /// The gateway this and `--data-format` name.
    pub fn resolve(self, data_format: Option<DataFormat>) -> Result<Kind> {
        let given = data_format.map(|f| f.0);
        if let (Some(implied), Some(given)) = (self.implied_format(), given)
            && implied != given
        {
            bail!(
                "--kind {} is --data-format {}, not {}",
                self.name,
                implied.name(),
                given.name()
            );
        }
        let Some(format) = given else {
            return Ok(self.kind);
        };
        match self.kind {
            Kind::Actisense(_) => Ok(Kind::Actisense(format)),
            Kind::YachtDevices if format == ActisenseFormat::RawAscii => Ok(self.kind),
            Kind::YachtDevices => bail!(
                "--data-format {}: canboat speaks only raw-ascii to Yacht Devices gateways",
                format.name()
            ),
            _ => bail!(
                "--data-format {}: --kind {} has no data formats",
                format.name(),
                self.name
            ),
        }
    }
}

/// What `--data-format` takes: the data formats of an Actisense gateway,
/// by Actisense's names. A Yacht Devices gateway speaks raw-ascii.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataFormat(pub ActisenseFormat);

static DATA_FORMATS: LazyLock<Vec<DataFormat>> = LazyLock::new(|| {
    ActisenseFormat::ALL
        .iter()
        .copied()
        .map(DataFormat)
        .collect()
});

impl DataFormat {
    /// The name `--data-format` takes.
    pub fn name(self) -> &'static str {
        self.0.name()
    }
}

impl clap::ValueEnum for DataFormat {
    fn value_variants<'a>() -> &'a [Self] {
        &DATA_FORMATS
    }

    fn to_possible_value(&self) -> Option<clap::builder::PossibleValue> {
        Some(clap::builder::PossibleValue::new(self.0.name()).help(self.0.describe()))
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

/// Help headings, in the order they appear. Those that name gateways are
/// [`crate::cli::help`]'s, written from what each kind takes.
pub const GATEWAY: &str = "Gateway";
pub const SPECIFIC: &str = "Gateway-specific";
pub const LISTS: &str = "PGN lists";
pub const BUS: &str = "Bus";

/// How to run the gateway. Options that have a default show it in their
/// help but are `None` when not given, so one given for a gateway that
/// does not have it can be refused.
#[derive(Debug, Clone, clap::Args)]
pub struct DeviceArgs {
    /// The data format an Actisense gateway is set to, by Actisense's
    /// names; see the table at the end. A Yacht Devices gateway speaks
    /// raw-ascii.
    #[arg(
        long,
        value_enum,
        value_name = "FORMAT",
        help = help::data_format_help(),
        help_heading = GATEWAY
    )]
    pub data_format: Option<DataFormat>,

    /// The serial speed, for a gateway on a serial port.
    // `-s` is the C actisense-serial/ikonvert-serial spelling; the argv[0]
    // shims must stay drop-in compatible with it.
    #[arg(
        short = 'b',
        long,
        short_alias = 's',
        value_name = "BAUD",
        help = help::baud_help(),
        help_heading = GATEWAY
    )]
    pub baud: Option<u32>,

    /// The CAN bus bit rate, where canboat is the CAN controller.
    #[arg(
        long,
        value_name = "BIT/S",
        help = help::bitrate_help(),
        long_help = help::bitrate_long_help(),
        help_heading = help::can_heading()
    )]
    pub bitrate: Option<u32>,

    /// The CAN channel of a two-channel adapter.
    #[arg(
        long,
        value_name = "N",
        help = help::only_help(Setting::Channel, "the CAN channel, 0 or 1 [default: 0]."),
        help_heading = help::can_heading()
    )]
    pub channel: Option<u8>,

    /// The gateway's login password.
    #[arg(
        long,
        alias = "maretron-password",
        value_name = "PASSWORD",
        help = help::only_help(Setting::Password, "the login password [default: empty]."),
        help_heading = SPECIFIC
    )]
    pub password: Option<String>,

    /// The NGT-1's operating mode.
    #[arg(
        long,
        value_enum,
        help = help::only_help(Setting::Mode, "the operating mode [default: rx-all]."),
        long_help = format!(
            "{}\n\n`rx-all` (\"NGT Transfer Rx All Mode\") passes on every PGN received; \
             `normal` (\"NGT Transfer Normal Mode\") only those on the Receive PGN Enable \
             list the gateway holds.",
            help::only_help(Setting::Mode, "the operating mode [default: rx-all].")
        ),
        help_heading = SPECIFIC
    )]
    pub mode: Option<Ngt1Mode>,

    /// Turn off the gateway's transmit rate limit.
    #[arg(
        long,
        visible_alias = "rate-limit-off",
        alias = "ikonvert-rate-limit-off",
        help = help::only_help(
            Setting::NoTxRateLimit,
            "turn off the gateway's transmit rate limit, at your own risk."
        ),
        help_heading = SPECIFIC
    )]
    pub no_tx_rate_limit: bool,

    /// Accept sentences whose checksum is wrong.
    #[arg(
        long,
        help = help::only_help(
            Setting::IgnoreChecksum,
            "accept $PCDIN or $MXPGN sentences whose checksum is wrong, as some gateways \
             may send them."
        ),
        help_heading = SPECIFIC
    )]
    pub ignore_checksum: bool,

    /// PGNs the application receives, to advertise; never a filter.
    ///
    /// Repeatable, or comma-separated. Answered in PGN 126464 by canboat's
    /// node or stored in an iKonvert; see the table at the end.
    #[arg(long = "rx-pgn", value_name = "PGN", value_delimiter = ',', help_heading = LISTS)]
    pub rx_pgn: Vec<u32>,

    /// Pass on only these PGNs (and network management); drop the rest.
    #[arg(
        long = "rx-filter",
        alias = "rx",
        alias = "ikonvert-rx",
        value_name = "PGN",
        value_delimiter = ',',
        long_help = help::rx_filter_long_help(),
        help_heading = LISTS
    )]
    pub rx_filter: Vec<u32>,

    /// PGNs the application transmits.
    #[arg(
        long = "tx-pgn",
        alias = "tx",
        alias = "ikonvert-tx",
        value_name = "PGN",
        value_delimiter = ',',
        help = help::tx_pgn_help(),
        long_help = help::tx_pgn_long_help(),
        help_heading = LISTS
    )]
    pub tx_pgn: Vec<u32>,

    /// The source address to claim; a free one if it is taken [default: 0].
    #[arg(short = 'a', long, alias = "socketcan-address", value_name = "ADDR", help_heading = help::node_heading())]
    pub address: Option<u8>,

    /// Run canboat's node on a gateway that passes frames through.
    #[arg(
        long,
        help = help::only_help(
            Setting::RunNode,
            "run canboat's node on the gateway: claim an address and answer ISO \
             Requests; without, frames go out as they are given."
        ),
        help_heading = help::node_heading()
    )]
    pub node: bool,

    /// Claim no address: listen only, or on aos send from the simulator's own.
    ///
    /// On socketcan and canalyst a passive sniff. On aos canboat runs no
    /// node, and the simulator sends from its fixed address without
    /// claiming it.
    #[arg(short = 'n', long, help_heading = help::node_heading())]
    pub no_claim: bool,

    /// ISO NAME unique number [default: derived from the machine id].
    ///
    /// The default is stable across restarts; on a machine that can't be
    /// identified it is a random one, stored with the state files.
    #[arg(short = 'u', long, value_name = "N", help_heading = help::node_heading())]
    pub unique: Option<u32>,

    /// ISO NAME manufacturer code [default: 999, Signal K].
    #[arg(short = 'm', long, value_name = "N", help_heading = help::node_heading())]
    pub manufacturer: Option<u16>,

    /// ISO NAME System Instance, 0..15 [default: 15].
    ///
    /// 15 makes canboat's NAME lose to real hardware's, so it yields an
    /// address rather than steal one.
    #[arg(
        long,
        value_name = "N",
        value_parser = clap::value_parser!(u8).range(0..=15),
        help_heading = help::node_heading()
    )]
    pub system_instance: Option<u8>,

    /// Heartbeat (PGN 126993) interval in ms, up to 65532; 0: none [default: 60000].
    #[arg(
        long,
        alias = "hb",
        value_name = "MS",
        value_parser = clap::value_parser!(u64).range(0..=65_532),
        help_heading = help::node_heading()
    )]
    pub heartbeat: Option<u64>,

    /// Do not add PGNs to the advertised Transmit list as they are first sent.
    #[arg(long, help_heading = help::node_heading())]
    pub no_learn_tx_pgns: bool,

    #[command(flatten, next_help_heading = BUS)]
    pub protocol: crate::cli::protocol::ProtocolArgs,
}

impl DeviceArgs {
    /// The flags given that only some kinds of gateway have, with the
    /// [`Setting`] each is. `--rx-filter` is not among them: canboat
    /// filters for every gateway, and an iKonvert also filters itself.
    fn settings_given(&self) -> Vec<(&'static str, Setting)> {
        [
            ("--baud", Setting::Baud, self.baud.is_some()),
            ("--bitrate", Setting::Bitrate, self.bitrate.is_some()),
            ("--channel", Setting::Channel, self.channel.is_some()),
            ("--password", Setting::Password, self.password.is_some()),
            ("--mode", Setting::Mode, self.mode.is_some()),
            (
                "--no-tx-rate-limit",
                Setting::NoTxRateLimit,
                self.no_tx_rate_limit,
            ),
            (
                "--ignore-checksum",
                Setting::IgnoreChecksum,
                self.ignore_checksum,
            ),
            ("--node", Setting::RunNode, self.node),
            ("--address", Setting::Node, self.address.is_some()),
            ("--no-claim", Setting::NoClaim, self.no_claim),
            ("--unique", Setting::Node, self.unique.is_some()),
            ("--manufacturer", Setting::Node, self.manufacturer.is_some()),
            (
                "--system-instance",
                Setting::Node,
                self.system_instance.is_some(),
            ),
            ("--heartbeat", Setting::Node, self.heartbeat.is_some()),
            ("--no-learn-tx-pgns", Setting::Node, self.no_learn_tx_pgns),
        ]
        .into_iter()
        .filter(|&(_, _, given)| given)
        .map(|(flag, setting, _)| (flag, setting))
        .collect()
    }

    /// Refuse what does not apply to a `kind` gateway, and a protocol it
    /// cannot carry. `kind` is `None` for a source that is not a gateway
    /// (a `canboat server` on stdin, a replay or another server): then
    /// only the protocol is checked.
    pub fn check(&self, kind: Option<Kind>) -> Result<()> {
        let protocol = self.protocol.resolve()?;
        let Some(kind) = kind else {
            if let Some((flag, _)) = self.settings_given().first() {
                bail!("{flag}: the source is not a gateway");
            }
            return Ok(());
        };
        if !kind.capabilities().protocols.contains(&protocol) {
            let carriers = crate::cli::help::kinds_carrying(protocol);
            if protocol.standard_frames() {
                bail!(
                    "--protocol {protocol} needs {carriers}: the other gateways pass only 29-bit frames"
                );
            }
            bail!(
                "--protocol {protocol} needs a gateway that passes raw CAN frames ({carriers}); \
                 {kind} does NMEA 2000 framing itself"
            );
        }
        for (flag, setting) in self.settings_given() {
            if kind.takes(setting) {
                continue;
            }
            if setting == Setting::Bitrate {
                bail!(
                    "--bitrate sets the CAN bus rate of {}, which canboat drives as the CAN \
                     controller (--kind {kind} does not have it; for the serial speed to a \
                     gateway, use --baud)",
                    crate::cli::help::kinds_taking(Setting::Bitrate)
                );
            }
            if kind == Kind::NGT1 && Kind::Aos.takes(setting) {
                // The node options were the AOS simulator's, on ngt1.
                bail!("{flag}: --kind {kind} does not have it (an AOS simulator is --kind aos)");
            }
            bail!("{flag}: --kind {kind} does not have it");
        }
        if kind.takes(Setting::RunNode)
            && !self.node
            && let Some((flag, _)) = self
                .settings_given()
                .into_iter()
                .find(|&(_, s)| matches!(s, Setting::Node | Setting::NoClaim))
        {
            bail!("{flag}: --kind {kind} runs canboat's node only with --node");
        }
        Ok(())
    }

    /// What the bus carries. Call [`Self::check`] first.
    pub fn protocol(&self) -> BusProtocol {
        self.protocol
            .resolve()
            .expect("DeviceArgs::check rejects a --protocol this cannot convert")
    }

    /// The `kind` gateway at `address`, with the options given. Call
    /// [`Self::check`] first: what it refuses, `Device` would too, with
    /// the library's words rather than the flags'.
    pub fn device(&self, kind: Kind, address: &str) -> Device {
        let mut dev = Device::new(kind, address).pgn_lists(self.pgn_lists());
        if let Some(baud) = self.baud {
            dev = dev.baud(baud);
        }
        if let Some(bitrate) = self.bitrate {
            dev = dev.bitrate(bitrate);
        }
        if let Some(channel) = self.channel {
            dev = dev.channel(channel);
        }
        if let Some(password) = &self.password {
            dev = dev.password(password.clone());
        }
        if let Some(mode) = self.mode {
            dev = dev.mode(mode.into());
        }
        if self.no_tx_rate_limit {
            dev = dev.no_tx_rate_limit();
        }
        if self.ignore_checksum {
            dev = dev.ignore_checksum();
        }
        if !self.rx_filter.is_empty() && kind.takes(Setting::RxFilter) {
            dev = dev.rx_filter(self.rx_filter.clone());
        }
        if let Some(address) = self.address {
            dev = dev.source_address(address);
        }
        if let Some(unique) = self.unique {
            dev = dev.unique(unique);
        }
        if let Some(manufacturer) = self.manufacturer {
            dev = dev.manufacturer(manufacturer);
        }
        if let Some(instance) = self.system_instance {
            dev = dev.system_instance(instance);
        }
        if let Some(ms) = self.heartbeat {
            dev = dev.heartbeat_ms(ms);
        }
        if self.no_learn_tx_pgns {
            dev = dev.learn_tx_pgns(false);
        }
        if self.no_claim {
            dev = dev.no_claim();
        }
        if self.node {
            dev = dev.run_node(true);
        }
        dev
    }

    /// `--rx-pgn` and `--tx-pgn`.
    pub fn pgn_lists(&self) -> crate::engine::pgn_list::PgnLists {
        crate::engine::pgn_list::PgnLists {
            tx: self.tx_pgn.clone(),
            rx: self.rx_pgn.clone(),
        }
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

    const YACHTD: Kind = Kind::YachtDevices;

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
    }

    #[test]
    fn an_option_the_gateway_lacks_is_refused() {
        assert!(
            parse(&["--bitrate", "500000"])
                .check(Some(Kind::SocketCan))
                .is_ok()
        );
        let err = parse(&["--bitrate", "500000"])
            .check(Some(Kind::NGT1))
            .unwrap_err();
        assert!(err.to_string().contains("--baud"), "{err}");
        assert!(
            parse(&["--password", "x"])
                .check(Some(Kind::Ikonvert))
                .is_err()
        );
        assert!(
            parse(&["--channel", "1"])
                .check(Some(Kind::SocketCan))
                .is_err()
        );
        for kind in [Kind::SeaSmart, Kind::Miniplex] {
            assert!(parse(&["--ignore-checksum"]).check(Some(kind)).is_ok());
        }
        assert!(parse(&["--ignore-checksum"]).check(Some(YACHTD)).is_err());
        assert!(parse(&["--mode", "normal"]).check(Some(Kind::NGT1)).is_ok());
        assert!(parse(&["--mode", "normal"]).check(Some(YACHTD)).is_err());
        // The node options: the CAN controllers and the AOS simulator.
        let err = parse(&["-a", "9"]).check(Some(Kind::NGT1)).unwrap_err();
        assert!(err.to_string().contains("--kind aos"), "{err}");
        for kind in [Kind::SocketCan, Kind::Canalyst, Kind::UsbCanA, Kind::Aos] {
            assert!(
                parse(&["-a", "9", "-u", "1", "-n"])
                    .check(Some(kind))
                    .is_ok()
            );
        }
        assert!(parse(&["-a", "9"]).check(Some(Kind::Ikonvert)).is_err());
        assert!(parse(&["-a", "9"]).check(None).is_err());
        for kind in [Kind::Ikonvert, Kind::SocketCan, Kind::NGT1] {
            assert!(parse(&["--rx", "129025"]).check(Some(kind)).is_ok());
        }
        assert!(parse(&["--rx-filter", "129025"]).check(None).is_ok());
        // The PGN lists apply everywhere; a gateway without them warns.
        assert!(
            parse(&["--tx-pgn", "127508"])
                .check(Some(Kind::Maretron))
                .is_ok()
        );
        // A serial speed for what has no serial port.
        assert!(
            parse(&["--baud", "9600"])
                .check(Some(Kind::SocketCan))
                .is_err()
        );
        assert!(parse(&["-s", "115200"]).check(Some(Kind::NGT1)).is_ok());
    }

    /// What `check` lets through, `Device` takes too: the CLI's checks are
    /// the library's.
    #[test]
    fn what_check_allows_the_device_takes() {
        for &kind in Kind::ALL {
            for args in [
                &["--baud", "9600"][..],
                &["--bitrate", "250000"],
                &["--channel", "1"],
                &["--password", "x"],
                &["--mode", "normal"],
                &["--no-tx-rate-limit"],
                &["--ignore-checksum"],
                &["-a", "9", "-u", "1", "--heartbeat", "0"],
                &["-n"],
                &["--rx-filter", "129025"],
            ] {
                let d = parse(args);
                if d.check(Some(kind)).is_ok() {
                    use crate::io::device::kind::Transport;
                    let address = match kind.transports()[0] {
                        Transport::Serial { .. } => "/dev/ttyUSB0",
                        Transport::Usb => "usb",
                        Transport::Interface => "can0",
                        _ => "tcp://192.0.2.1",
                    };
                    let dev = d.device(kind, address);
                    assert!(dev.check(None).is_ok(), "{kind} {args:?}");
                }
            }
        }
    }

    #[test]
    fn candump_takes_the_node_options_only_with_node() {
        assert!(parse(&["-a", "9"]).check(Some(Kind::Candump)).is_err());
        assert!(
            parse(&["--node", "-a", "9"])
                .check(Some(Kind::Candump))
                .is_ok()
        );
        assert!(parse(&["--node"]).check(Some(Kind::SocketCan)).is_err());
        let dev = parse(&["--node", "-a", "9"]).device(Kind::Candump, "tcp://sensesp");
        assert!(dev.capabilities().node);
        assert!(
            !parse(&[])
                .device(Kind::Candump, "tcp://x")
                .capabilities()
                .node
        );
    }

    #[test]
    fn kind_and_data_format_resolve() {
        use clap::ValueEnum;
        let kind = |name: &str| KindArg::from_str(name, false).unwrap();
        let format = |f: ActisenseFormat| Some(DataFormat(f));
        use ActisenseFormat::*;
        for (name, given, want) in [
            ("actisense", None, Kind::NGT1),
            ("actisense", format(NgtActisense), Kind::NGT1),
            ("actisense", format(N2kAscii), Kind::Actisense(N2kAscii)),
            (
                "actisense",
                format(N2kActisense),
                Kind::Actisense(N2kActisense),
            ),
            ("actisense", format(RawAscii), Kind::Actisense(RawAscii)),
            (
                "actisense",
                format(RawActisense),
                Kind::Actisense(RawActisense),
            ),
            ("actisense", format(Mxpgn), Kind::Actisense(Mxpgn)),
            ("yachtd", None, YACHTD),
            ("yachtd", format(RawAscii), YACHTD),
            ("seasmart", None, Kind::SeaSmart),
            ("aos", None, Kind::Aos),
            // The old names imply their format.
            ("ngt1", None, Kind::NGT1),
            ("w2k-ascii", format(N2kAscii), Kind::Actisense(N2kAscii)),
            ("ydwg", None, YACHTD),
        ] {
            assert_eq!(kind(name).resolve(given).unwrap(), want, "{name} {given:?}");
        }
        assert!(kind("w2k-ascii").resolve(format(RawAscii)).is_err());
        assert!(kind("yachtd").resolve(format(N2kAscii)).is_err());
        assert!(kind("socketcan").resolve(format(RawAscii)).is_err());
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
                "candump",
                "plain",
                "ikonvert",
                "maretron",
                "aos",
                "socketcan",
                "canalyst",
                "usb-can-a",
            ]
        );
    }

    #[test]
    fn only_raw_frame_gateways_carry_j1939_and_quick() {
        let j1939 = parse(&["--protocol", "j1939"]);
        assert!(j1939.check(Some(YACHTD)).is_ok());
        assert!(
            j1939
                .check(Some(Kind::Actisense(ActisenseFormat::Mxpgn)))
                .is_ok()
        );
        let err = j1939.check(Some(Kind::NGT1)).unwrap_err().to_string();
        assert!(err.contains("actisense (mxpgn)"), "{err}");
        let quick = parse(&["--protocol", "quick"]);
        assert!(quick.check(Some(Kind::Canalyst)).is_ok());
        assert!(quick.check(Some(Kind::UsbCanA)).is_ok());
        assert!(j1939.check(Some(Kind::UsbCanA)).is_ok());
        assert!(
            quick
                .check(Some(Kind::Actisense(ActisenseFormat::RawActisense)))
                .is_err()
        );
    }
}
