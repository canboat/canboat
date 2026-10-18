// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Which gateway, and what it can do. This is the one place canboat says
//! so: [`Device`](super::open::Device) opens a gateway by its [`Kind`] and
//! checks its settings against it, and the command line parses `--kind`
//! with it and writes its `--help`, and the tables in `doc/gateways.md`,
//! from it.

use std::fmt;
use std::str::FromStr;

use crate::engine::BusProtocol;

/// A kind of gateway: the hardware, and the data format it is set to.
///
/// [`Kind::ALL`] lists them, and [`FromStr`] takes the names
/// `canboat interface --kind` takes: `actisense` (the NGT-1),
/// `actisense:raw-ascii` and the other data formats of an Actisense
/// gateway, `yachtd`, `miniplex`, `seasmart`, `ikonvert`, `maretron`,
/// `aos`, `socketcan`, `canalyst` and `usb-can-a`, as well as canboat
/// 8.3.0's `ngt1`, `w2k-ascii` and `ydwg`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Kind {
    /// An Actisense gateway in one of its data formats:
    /// [`ActisenseFormat::NgtActisense`] is the NGT-1.
    Actisense(ActisenseFormat),
    /// The Across Ocean Systems NMEA 2000/0183 Simulator. It speaks the
    /// NGT-1's format but is no node: canboat claims an address for it.
    Aos,
    /// A Digital Yacht iKonvert or NavLink2.
    Ikonvert,
    /// A Maretron IPG100 or IPG200.
    Maretron,
    /// A Yacht Devices gateway in RAW ASCII.
    YachtDevices,
    /// A ShipModul MiniPlex-3 with NMEA 2000, in `$MXPGN`.
    Miniplex,
    /// A gateway in Chetco's SeaSmart.Net `$PCDIN`.
    SeaSmart,
    /// A Linux SocketCAN interface. canboat is the node on it.
    SocketCan,
    /// A CANalyst-II (Waveshare USB-CAN-B) over USB. canboat is the node.
    Canalyst,
    /// A Waveshare USB-CAN-A on its serial port. canboat is the node.
    UsbCanA,
}

/// The data formats of an Actisense gateway, by Actisense's names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ActisenseFormat {
    /// NGT Actisense (BST-93/94): the NGT-1's binary format.
    NgtActisense,
    /// N2K ASCII: one reassembled NMEA 2000 message per line.
    N2kAscii,
    /// N2K Actisense (BST-D0): reassembled NMEA 2000 messages, binary.
    N2kActisense,
    /// RAW ASCII: one CAN frame per line, as Yacht Devices sends it.
    RawAscii,
    /// RAW Actisense (BST-95): CAN frames, binary.
    RawActisense,
    /// NMEA 0183 with `$MXPGN`: CAN frames with the data in bus order, as
    /// a W2K-2's hidden CANDIRECT data server sends them.
    Mxpgn,
}

impl ActisenseFormat {
    /// Every format, in the order `--data-format` lists them.
    pub const ALL: &'static [ActisenseFormat] = &[
        ActisenseFormat::NgtActisense,
        ActisenseFormat::N2kAscii,
        ActisenseFormat::N2kActisense,
        ActisenseFormat::RawAscii,
        ActisenseFormat::RawActisense,
        ActisenseFormat::Mxpgn,
    ];

    /// What it is, in a line.
    pub fn describe(self) -> &'static str {
        match self {
            ActisenseFormat::NgtActisense => "NGT Actisense: the NGT-1's binary format (BST-93/94)",
            ActisenseFormat::N2kAscii => "N2K ASCII: one reassembled NMEA 2000 message per line",
            ActisenseFormat::N2kActisense => {
                "N2K Actisense (BST-D0): reassembled NMEA 2000 messages, binary"
            }
            ActisenseFormat::RawAscii => {
                "RAW ASCII: one CAN frame per line, as Yacht Devices sends it"
            }
            ActisenseFormat::RawActisense => "RAW Actisense (BST-95): CAN frames, binary",
            ActisenseFormat::Mxpgn => {
                "NMEA 0183 with $MXPGN: CAN frames, the data in bus order, as a W2K-2's \
                 CANDIRECT data server sends them"
            }
        }
    }

    /// The name `--data-format` takes.
    pub fn name(self) -> &'static str {
        match self {
            ActisenseFormat::NgtActisense => "ngt-actisense",
            ActisenseFormat::N2kAscii => "n2k-ascii",
            ActisenseFormat::N2kActisense => "n2k-actisense",
            ActisenseFormat::RawAscii => "raw-ascii",
            ActisenseFormat::RawActisense => "raw-actisense",
            ActisenseFormat::Mxpgn => "mxpgn",
        }
    }
}

/// How a gateway is reached, with the port or speed canboat uses when the
/// address gives none.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Transport {
    /// A serial port, `/dev/ttyUSB0` or `COM3`, at `baud` unless
    /// [`Device::baud`](super::open::Device::baud) says otherwise.
    Serial { baud: u32 },
    /// Straight over USB: `usb`, or `usb:SERIAL` to pick one adapter of
    /// several.
    Usb,
    /// `tcp://host[:port]`.
    Tcp { port: u16 },
    /// `udp://[host:]port`, or `udp://` alone where there is a `port`.
    /// Only receives, unless the address is the gateway's.
    Udp { port: Option<u16> },
    /// A network interface of this host, `can0`.
    Interface,
}

/// A setting of [`Device`](super::open::Device) that only some kinds of
/// gateway have. One given for a kind that does not have it fails at
/// `open`: it is never ignored.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Setting {
    /// The serial speed: a gateway on a serial port.
    Baud,
    /// The CAN bus bit rate: where canboat is the CAN controller.
    Bitrate,
    /// Bring the interface down, set its bit rate and bring it up again.
    ConfigureLink,
    /// The CAN channel of a two-channel adapter.
    Channel,
    /// The gateway's login password.
    Password,
    /// The NGT-1's operating mode.
    Mode,
    /// Turn off the gateway's own transmit rate limit.
    NoTxRateLimit,
    /// Accept sentences whose checksum is wrong.
    IgnoreChecksum,
    /// Have the gateway itself pass on only some PGNs.
    RxFilter,
    /// canboat's node: its address, ISO NAME, heartbeat and the rest.
    Node,
    /// Run canboat's node without claiming an address.
    NoClaim,
}

impl Setting {
    /// Every setting.
    pub const ALL: &'static [Setting] = &[
        Setting::Baud,
        Setting::Bitrate,
        Setting::ConfigureLink,
        Setting::Channel,
        Setting::Password,
        Setting::Mode,
        Setting::NoTxRateLimit,
        Setting::IgnoreChecksum,
        Setting::RxFilter,
        Setting::Node,
        Setting::NoClaim,
    ];

    /// Its name, for messages.
    pub fn name(self) -> &'static str {
        match self {
            Setting::Baud => "baud",
            Setting::Bitrate => "bitrate",
            Setting::ConfigureLink => "configure_link",
            Setting::Channel => "channel",
            Setting::Password => "password",
            Setting::Mode => "mode",
            Setting::NoTxRateLimit => "no_tx_rate_limit",
            Setting::IgnoreChecksum => "ignore_checksum",
            Setting::RxFilter => "rx_filter",
            Setting::Node => "node",
            Setting::NoClaim => "no_claim",
        }
    }

    /// The kinds that have it, in [`Kind::ALL`]'s order.
    pub fn kinds(self) -> impl Iterator<Item = Kind> {
        Kind::ALL.iter().copied().filter(move |k| k.takes(self))
    }
}

/// What a gateway does with one of the PGN lists an application gives
/// ([`Device::pgn_lists`](super::open::Device::pgn_lists)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ListUse {
    /// Nothing: the gateway has no such list.
    Ignored,
    /// canboat's node answers with it, in PGN 126464.
    Advertised,
    /// Stored in the gateway, which advertises it.
    Stored,
    /// Stored in the gateway, which advertises it and sends only these.
    StoredExclusive,
    /// The gateway's Transmit PGN Enable list: it sends only these.
    TransmitEnable,
}

impl ListUse {
    /// Whether, once the list names a PGN, every PGN not on it is
    /// refused (network management apart).
    pub fn exclusive(self) -> bool {
        matches!(self, ListUse::StoredExclusive | ListUse::TransmitEnable)
    }
}

/// What a gateway does with the receive and the transmit PGN lists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct PgnListUse {
    pub rx: ListUse,
    pub tx: ListUse,
}

/// The CAN bus bit rates a gateway can be set to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Bitrates {
    /// None: the gateway decides, and canboat cannot set it.
    Fixed,
    /// Whatever the interface's driver takes.
    Any,
    /// These.
    Only(&'static [u32]),
}

impl Bitrates {
    /// Whether the gateway can be set to `bitrate`.
    pub fn allows(self, bitrate: u32) -> bool {
        match self {
            Bitrates::Fixed => false,
            Bitrates::Any => bitrate > 0,
            Bitrates::Only(rates) => rates.contains(&bitrate),
        }
    }
}

/// What a gateway can do. [`Kind::capabilities`] is the best case;
/// [`Device::capabilities`](super::open::Device::capabilities) also takes
/// its address into account (a `udp://port` only receives).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct Capabilities {
    /// It passes single CAN frames, so
    /// [`Device::open_raw`](super::open::Device::open_raw) works.
    pub raw: bool,
    /// It transmits single CAN frames as they are given, through
    /// `open_raw`.
    pub raw_sends: bool,
    /// What [`Device::open`](super::open::Device::open) is given to send
    /// reaches the bus.
    pub sends: bool,
    /// It puts its own source address on what it transmits.
    pub rewrites_source: bool,
    /// It carries 11-bit frames as well as 29-bit ones.
    pub standard_frames: bool,
    /// It transmits only whole NMEA 2000 messages, and joins and splits
    /// the fast-packets itself.
    pub n2k_tx_only: bool,
    /// J1939 messages over 8 bytes are sent, by ISO TP: only where canboat
    /// is the node. Elsewhere they are received only.
    pub iso_tp_sends: bool,
    /// canboat runs its own node on it, which claims an address.
    pub node: bool,
    /// It echoes what it transmits. canboat skips the echoes in `open`.
    pub echoes: bool,
    /// The bit rates it can be set to.
    pub bitrates: Bitrates,
    /// The protocols [`Device::open`](super::open::Device::open) carries
    /// on it.
    pub protocols: &'static [BusProtocol],
}

const N2K: &[BusProtocol] = &[BusProtocol::Nmea2000];
const EXTENDED: &[BusProtocol] = &[BusProtocol::Nmea2000, BusProtocol::J1939];
const EVERY: &[BusProtocol] = &[
    BusProtocol::Nmea2000,
    BusProtocol::J1939,
    BusProtocol::Quick,
];

/// A gateway that does NMEA 2000 framing itself.
const N2K_GATEWAY: Capabilities = Capabilities {
    raw: false,
    raw_sends: false,
    sends: true,
    rewrites_source: false,
    standard_frames: false,
    n2k_tx_only: true,
    iso_tp_sends: false,
    node: false,
    echoes: false,
    bitrates: Bitrates::Fixed,
    protocols: N2K,
};

/// A gateway that passes 29-bit CAN frames, and sends them from its own
/// address.
const FRAME_GATEWAY: Capabilities = Capabilities {
    raw: true,
    raw_sends: false,
    sends: true,
    rewrites_source: true,
    standard_frames: false,
    n2k_tx_only: false,
    iso_tp_sends: false,
    node: false,
    echoes: false,
    bitrates: Bitrates::Fixed,
    protocols: EXTENDED,
};

/// A CAN controller, with canboat's node on it.
const CONTROLLER: Capabilities = Capabilities {
    raw: true,
    raw_sends: true,
    sends: true,
    rewrites_source: false,
    standard_frames: true,
    n2k_tx_only: false,
    iso_tp_sends: true,
    node: true,
    echoes: false,
    bitrates: Bitrates::Any,
    protocols: EVERY,
};

impl Kind {
    /// The NGT-1: an Actisense gateway in NGT Actisense.
    pub const NGT1: Kind = Kind::Actisense(ActisenseFormat::NgtActisense);

    /// Every kind, in the order the help lists them.
    pub const ALL: &'static [Kind] = &[
        Kind::NGT1,
        Kind::Actisense(ActisenseFormat::N2kAscii),
        Kind::Actisense(ActisenseFormat::N2kActisense),
        Kind::Actisense(ActisenseFormat::RawAscii),
        Kind::Actisense(ActisenseFormat::RawActisense),
        Kind::Actisense(ActisenseFormat::Mxpgn),
        Kind::YachtDevices,
        Kind::Miniplex,
        Kind::SeaSmart,
        Kind::Ikonvert,
        Kind::Maretron,
        Kind::Aos,
        Kind::SocketCan,
        Kind::Canalyst,
        Kind::UsbCanA,
    ];

    /// canboat 8.3.0's names for three kinds, which still parse.
    pub const OLD_NAMES: &'static [(&'static str, Kind)] = &[
        ("ngt1", Kind::NGT1),
        ("w2k-ascii", Kind::Actisense(ActisenseFormat::N2kAscii)),
        ("ydwg", Kind::YachtDevices),
    ];

    /// The name `--kind` takes. The Actisense data formats share
    /// `actisense`; [`Kind::data_format`] tells them apart.
    pub fn name(self) -> &'static str {
        match self {
            Kind::Actisense(_) => "actisense",
            Kind::Aos => "aos",
            Kind::Ikonvert => "ikonvert",
            Kind::Maretron => "maretron",
            Kind::YachtDevices => "yachtd",
            Kind::Miniplex => "miniplex",
            Kind::SeaSmart => "seasmart",
            Kind::SocketCan => "socketcan",
            Kind::Canalyst => "canalyst",
            Kind::UsbCanA => "usb-can-a",
        }
    }

    /// An Actisense gateway's data format.
    pub fn data_format(self) -> Option<ActisenseFormat> {
        match self {
            Kind::Actisense(format) => Some(format),
            _ => None,
        }
    }

    /// The kind `name` and the data format `format` name together, as
    /// `--kind` and `--data-format` do: `format` only for `actisense`
    /// (where `None` is the NGT-1) and `yachtd` (which is `raw-ascii`).
    pub fn with_format(name: &str, format: Option<ActisenseFormat>) -> Result<Kind, UnknownKind> {
        let kind = Kind::by_name(name).ok_or_else(|| UnknownKind(name.to_string()))?;
        Ok(match (kind, format) {
            (_, None) => kind,
            (Kind::Actisense(_), Some(format)) => Kind::Actisense(format),
            (Kind::YachtDevices, Some(ActisenseFormat::RawAscii)) => kind,
            (_, Some(format)) => {
                return Err(UnknownKind(format!("{name}:{}", format.name())));
            }
        })
    }

    /// The kind `--kind` names, or an old name, without a data format.
    fn by_name(name: &str) -> Option<Kind> {
        Kind::ALL
            .iter()
            .find(|k| {
                k.name() == name
                    && k.data_format()
                        .is_none_or(|f| f == ActisenseFormat::NgtActisense)
            })
            .copied()
            .or_else(|| {
                Kind::OLD_NAMES
                    .iter()
                    .find(|(old, _)| *old == name)
                    .map(|&(_, kind)| kind)
            })
    }

    /// The gateways it is, for a list of them.
    pub fn describe(self) -> &'static str {
        match self {
            Kind::NGT1 => "Actisense NGT-1",
            Kind::Actisense(ActisenseFormat::N2kAscii)
            | Kind::Actisense(ActisenseFormat::N2kActisense)
            | Kind::Actisense(ActisenseFormat::RawAscii) => "Actisense W2K-1, PRO-NDC-1E2K",
            Kind::Actisense(ActisenseFormat::RawActisense) => "Actisense W2K-1, PRO-NDC-1E2K, NGX",
            Kind::Actisense(ActisenseFormat::Mxpgn) => "Actisense W2K-2 (CANDIRECT, NMEA 0183)",
            Kind::YachtDevices => "Yacht Devices YDWG-02, YDEN-02, YDNR-02, YDNU-02",
            Kind::Miniplex => "ShipModul MiniPlex-3 with NMEA 2000",
            Kind::SeaSmart => {
                "Chetco SeaSmart, Quark-elec QK-A032/A034/A036, Yacht Devices in NMEA 0183 mode"
            }
            Kind::Ikonvert => "Digital Yacht iKonvert, NavLink2",
            Kind::Maretron => "Maretron IPG100/200",
            Kind::Aos => "AOS NMEA 2000/0183 Simulator",
            Kind::SocketCan => "Linux SocketCAN interface",
            Kind::Canalyst => "CANalyst-II, Waveshare USB-CAN-B",
            Kind::UsbCanA => "Waveshare USB-CAN-A",
        }
    }

    /// How it is reached, the usual way first.
    pub fn transports(self) -> &'static [Transport] {
        use Transport::*;
        match self {
            Kind::NGT1 | Kind::Aos => &[Serial { baud: 115_200 }, Usb, Tcp { port: 6001 }],
            Kind::Actisense(ActisenseFormat::N2kAscii)
            | Kind::Actisense(ActisenseFormat::RawAscii) => {
                &[Tcp { port: 60002 }, Udp { port: None }]
            }
            Kind::Actisense(ActisenseFormat::N2kActisense) => &[Tcp { port: 60002 }],
            Kind::Actisense(ActisenseFormat::RawActisense) => {
                &[Tcp { port: 60002 }, Serial { baud: 115_200 }]
            }
            Kind::Actisense(ActisenseFormat::Mxpgn) => &[Tcp { port: 20000 }, Udp { port: None }],
            Kind::YachtDevices => &[
                Tcp { port: 1457 },
                Udp { port: None },
                Serial { baud: 38_400 },
            ],
            Kind::Miniplex => &[
                Tcp { port: 10110 },
                Udp { port: None },
                Serial { baud: 460_800 },
            ],
            Kind::SeaSmart => &[
                Tcp { port: 10001 },
                Udp { port: None },
                Serial { baud: 38_400 },
            ],
            Kind::Ikonvert => &[
                Serial { baud: 230_400 },
                Tcp { port: 2000 },
                Udp { port: Some(2000) },
            ],
            Kind::Maretron => &[Tcp { port: 6543 }],
            Kind::SocketCan => &[Interface],
            Kind::Canalyst => &[Usb],
            Kind::UsbCanA => &[Serial {
                baud: crate::engine::codec::usb_can_a::BAUD,
            }],
        }
    }

    /// The serial speed when none is given, for a kind that has a serial
    /// port.
    pub fn default_baud(self) -> Option<u32> {
        self.transports().iter().find_map(|t| match *t {
            Transport::Serial { baud } => Some(baud),
            _ => None,
        })
    }

    /// The settings it has, beyond those every kind has.
    pub fn settings(self) -> &'static [Setting] {
        use Setting::*;
        match self {
            Kind::NGT1 => &[Baud, Mode],
            Kind::Aos => &[Baud, Node, NoClaim],
            Kind::Ikonvert => &[Baud, NoTxRateLimit, RxFilter],
            Kind::Maretron => &[Password],
            Kind::YachtDevices | Kind::Actisense(ActisenseFormat::RawActisense) => &[Baud],
            Kind::Miniplex | Kind::SeaSmart => &[Baud, IgnoreChecksum],
            Kind::Actisense(_) => &[],
            Kind::SocketCan => &[Bitrate, ConfigureLink, Node, NoClaim],
            Kind::Canalyst => &[Bitrate, Channel, Node, NoClaim],
            Kind::UsbCanA => &[Baud, Bitrate, Node, NoClaim],
        }
    }

    /// Whether it has `setting`.
    pub fn takes(self, setting: Setting) -> bool {
        self.settings().contains(&setting)
    }

    /// What it does with the PGN lists.
    pub fn pgn_lists(self) -> PgnListUse {
        let (rx, tx) = match self {
            Kind::Ikonvert => (ListUse::Stored, ListUse::StoredExclusive),
            Kind::NGT1 => (ListUse::Ignored, ListUse::TransmitEnable),
            Kind::Aos | Kind::SocketCan | Kind::Canalyst | Kind::UsbCanA => {
                (ListUse::Advertised, ListUse::Advertised)
            }
            _ => (ListUse::Ignored, ListUse::Ignored),
        };
        PgnListUse { rx, tx }
    }

    /// What it can do at best: over a transport that sends, and in the
    /// data format that carries the most.
    pub fn capabilities(self) -> Capabilities {
        match self {
            // The NGT-1 transmits from the address it claims itself.
            Kind::NGT1 => Capabilities {
                rewrites_source: true,
                ..N2K_GATEWAY
            },
            // A message to send carries no source.
            Kind::Ikonvert | Kind::Maretron => Capabilities {
                rewrites_source: true,
                ..N2K_GATEWAY
            },
            Kind::SeaSmart => Capabilities {
                rewrites_source: true,
                ..N2K_GATEWAY
            },
            Kind::Aos => Capabilities {
                node: true,
                ..N2K_GATEWAY
            },
            Kind::Actisense(ActisenseFormat::N2kAscii)
            | Kind::Actisense(ActisenseFormat::N2kActisense) => N2K_GATEWAY,
            Kind::YachtDevices => Capabilities {
                echoes: true,
                ..FRAME_GATEWAY
            },
            Kind::Miniplex | Kind::Actisense(ActisenseFormat::Mxpgn) => FRAME_GATEWAY,
            // Not known to send a frame as it is given; canboat sends what
            // `open` is given as Yacht Devices RAW lines, as for a YDWG.
            Kind::Actisense(ActisenseFormat::RawAscii) => Capabilities {
                rewrites_source: false,
                ..FRAME_GATEWAY
            },
            Kind::Actisense(ActisenseFormat::RawActisense) => Capabilities {
                raw_sends: true,
                rewrites_source: false,
                echoes: true,
                ..FRAME_GATEWAY
            },
            Kind::SocketCan => CONTROLLER,
            Kind::Canalyst => Capabilities {
                bitrates: Bitrates::Only(&crate::engine::codec::canalyst::BITRATES),
                ..CONTROLLER
            },
            Kind::UsbCanA => Capabilities {
                bitrates: Bitrates::Only(&crate::engine::codec::usb_can_a::RATES),
                ..CONTROLLER
            },
        }
    }
}

impl fmt::Display for Kind {
    /// `actisense (raw-ascii)` for an Actisense gateway, otherwise its
    /// [name](Kind::name).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.data_format() {
            Some(format) => write!(f, "{} ({})", self.name(), format.name()),
            None => f.write_str(self.name()),
        }
    }
}

impl FromStr for Kind {
    type Err = UnknownKind;

    /// A `--kind` name, an old name, or `actisense:FORMAT`.
    fn from_str(s: &str) -> Result<Kind, UnknownKind> {
        let Some((name, format)) = s.split_once(':') else {
            return Kind::with_format(s, None);
        };
        let format = ActisenseFormat::ALL
            .iter()
            .find(|f| f.name() == format)
            .copied()
            .ok_or_else(|| UnknownKind(s.to_string()))?;
        Kind::with_format(name, Some(format)).map_err(|_| UnknownKind(s.to_string()))
    }
}

/// A name that is not a [`Kind`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownKind(String);

impl fmt::Display for UnknownKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "no such kind of gateway: {}", self.0)
    }
}

impl std::error::Error for UnknownKind {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_kind_parses_back_from_its_name() {
        for &kind in Kind::ALL {
            let spelled = match kind.data_format() {
                Some(format) => format!("{}:{}", kind.name(), format.name()),
                None => kind.name().to_string(),
            };
            assert_eq!(spelled.parse::<Kind>(), Ok(kind), "{spelled}");
        }
        assert_eq!("actisense".parse::<Kind>(), Ok(Kind::NGT1));
        for &(old, kind) in Kind::OLD_NAMES {
            assert_eq!(old.parse::<Kind>(), Ok(kind), "{old}");
        }
        assert!("yachtd:raw-ascii".parse::<Kind>().is_ok());
        assert!("yachtd:n2k-ascii".parse::<Kind>().is_err());
        assert!("miniplex:mxpgn".parse::<Kind>().is_err());
        assert!("ngt2".parse::<Kind>().is_err());
    }

    #[test]
    fn a_kind_with_a_serial_port_takes_a_baud_rate() {
        for &kind in Kind::ALL {
            assert_eq!(
                kind.takes(Setting::Baud),
                kind.default_baud().is_some(),
                "{kind}"
            );
        }
    }

    #[test]
    fn the_capabilities_hang_together() {
        for &kind in Kind::ALL {
            let caps = kind.capabilities();
            assert!(caps.protocols.contains(&BusProtocol::Nmea2000), "{kind}");
            // Another protocol needs the frames themselves.
            assert_eq!(caps.raw, caps.protocols.len() > 1, "{kind}");
            assert_eq!(
                caps.standard_frames,
                caps.protocols.contains(&BusProtocol::Quick),
                "{kind}"
            );
            assert!(!caps.raw_sends || caps.raw, "{kind}");
            assert!(!(caps.raw_sends && caps.rewrites_source), "{kind}");
            assert!(!(caps.n2k_tx_only && caps.raw), "{kind}");
            // canboat's node is what sends ISO TP, and claims an address.
            assert_eq!(caps.iso_tp_sends, caps.node && caps.raw, "{kind}");
            assert_eq!(caps.node, kind.takes(Setting::Node), "{kind}");
            assert_eq!(
                kind.takes(Setting::Bitrate),
                caps.bitrates != Bitrates::Fixed,
                "{kind}"
            );
            assert_eq!(
                kind.pgn_lists().rx == ListUse::Advertised,
                caps.node,
                "{kind}"
            );
        }
    }

    #[test]
    fn the_bit_rates_are_the_codecs() {
        let rates = |kind: Kind| match kind.capabilities().bitrates {
            Bitrates::Only(rates) => rates.to_vec(),
            other => panic!("{kind}: {other:?}"),
        };
        assert_eq!(
            rates(Kind::UsbCanA),
            crate::engine::codec::usb_can_a::bitrates().collect::<Vec<_>>()
        );
        assert!(rates(Kind::Canalyst).contains(&250_000));
        assert!(Kind::SocketCan.capabilities().bitrates.allows(500_000));
        assert!(!Kind::YachtDevices.capabilities().bitrates.allows(250_000));
    }
}
