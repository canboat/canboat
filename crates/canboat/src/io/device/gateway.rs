// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The network gateways — and their serial cousins — that `canboat
//! interface --kind` and `canboat server` open the same way: Yacht
//! Devices RAW, a ShipModul MiniPlex in `$MXPGN`, a SeaSmart gateway in
//! `$PCDIN`, and an Actisense W2K-1 or
//! PRO-NDC-1E2K in RAW ASCII, N2K ASCII, N2K Actisense (BST-D0) or RAW
//! Actisense (BST-95) mode.

use std::io;

use crate::engine::BusProtocol;
use crate::io::net::{open_line_endpoint, open_line_endpoint_sends, open_tcp};
use crate::io::open_serial_rw;

use super::{DeviceHandle, line_gateway};

/// Which gateway.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum GatewayKind {
    /// Yacht Devices, in RAW ASCII: a YDWG-02 / YDEN-02 / YDNR-02 /
    /// NavLink2 RAW port over `tcp://host[:port]` (port 1457 if not given)
    /// or `udp://…`, or a YDNU-02 on its serial port (38 400 baud).
    YachtDevices,
    /// A ShipModul MiniPlex-3 with NMEA 2000, in `$MXPGN`: over
    /// `tcp://host[:port]` (port 10110 if not given) or `udp://…`, or on its
    /// USB serial port (460 800 baud). canboat sets it to send every PGN.
    Miniplex,
    /// A gateway in Chetco's SeaSmart.Net `$PCDIN`, one whole message per
    /// sentence: a Chetco SeaSmart, a Quark-elec QK-A032/A034/A036, or a
    /// Yacht Devices gateway in NMEA 0183 mode. Over `tcp://host[:port]`
    /// (port 10001, Chetco's, if not given) or `udp://…`, or on a serial
    /// port (38 400 baud). Not every one takes `$PCDIN` to send: a Yacht
    /// Devices gateway takes only a few PGNs.
    SeaSmart,
    /// An Actisense W2K-1, PRO-NDC-1E2K or NGX, set to `format`. (An NGT-1,
    /// in NGT Actisense, is opened with its own driver: see
    /// [`crate::bus::open_ngt1`].)
    Actisense(ActisenseFormat),
}

/// The data formats of an Actisense W2K-1, PRO-NDC-1E2K or NGX, by
/// Actisense's names, apart from NGT Actisense.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ActisenseFormat {
    /// N2K ASCII, reassembled NMEA 2000 messages as text, over
    /// `tcp://host[:port]` (port 60002) or `udp://…`.
    N2kAscii,
    /// N2K Actisense (BST-D0), reassembled NMEA 2000 messages, binary,
    /// over `tcp://host[:port]` (port 60002).
    N2kActisense,
    /// RAW ASCII, CAN frames as the Yacht Devices RAW lines, over
    /// `tcp://host[:port]` (port 60002) or `udp://…`.
    RawAscii,
    /// RAW Actisense (BST-95), CAN frames, binary: a PRO-NDC-1E2K or W2K-1
    /// over `tcp://host[:port]` (port 60002), or an NGX in CAN Packet mode
    /// on its serial port (115 200 baud).
    RawActisense,
}

impl GatewayKind {
    /// The serial speed when none is given, for a gateway on a serial port.
    pub fn default_baud(self) -> u32 {
        match self {
            GatewayKind::YachtDevices => 38_400,
            GatewayKind::Miniplex => 460_800,
            GatewayKind::SeaSmart => 38_400,
            GatewayKind::Actisense(ActisenseFormat::RawActisense) => 115_200,
            GatewayKind::Actisense(ActisenseFormat::N2kAscii)
            | GatewayKind::Actisense(ActisenseFormat::N2kActisense)
            | GatewayKind::Actisense(ActisenseFormat::RawAscii) => 0,
        }
    }

    /// Whether it passes raw CAN frames, and so any [`BusProtocol`]; the
    /// others do NMEA 2000 framing themselves.
    pub fn passes_raw_frames(self) -> bool {
        matches!(
            self,
            GatewayKind::YachtDevices
                | GatewayKind::Miniplex
                | GatewayKind::Actisense(ActisenseFormat::RawActisense)
                | GatewayKind::Actisense(ActisenseFormat::RawAscii)
        )
    }

    /// Whether it sends lines of text, and so also works over UDP.
    fn is_line_gateway(self) -> bool {
        matches!(
            self,
            GatewayKind::YachtDevices
                | GatewayKind::Miniplex
                | GatewayKind::SeaSmart
                | GatewayKind::Actisense(ActisenseFormat::N2kAscii)
                | GatewayKind::Actisense(ActisenseFormat::RawAscii)
        )
    }

    /// A short name for log lines.
    pub fn name(self) -> &'static str {
        match self {
            GatewayKind::YachtDevices => "yachtd",
            GatewayKind::Miniplex => "miniplex",
            GatewayKind::SeaSmart => "seasmart",
            GatewayKind::Actisense(ActisenseFormat::N2kAscii) => "actisense-n2k-ascii",
            GatewayKind::Actisense(ActisenseFormat::RawAscii) => "actisense-raw-ascii",
            GatewayKind::Actisense(ActisenseFormat::N2kActisense) => "actisense-n2k-actisense",
            GatewayKind::Actisense(ActisenseFormat::RawActisense) => "actisense-raw-actisense",
        }
    }
}

/// A gateway, and where it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Gateway {
    pub kind: GatewayKind,
    /// `tcp://host[:port]`, `udp://…` (the line gateways), or a serial
    /// port (a YDNU-02, or an NGX in RAW Actisense).
    pub device: String,
}

/// Open `gateway` and start its codec. `baud`, for a serial port, defaults
/// to [`GatewayKind::default_baud`]. `ignore_checksum` accepts `$PCDIN` and
/// `$MXPGN` sentences whose checksum is wrong.
pub(crate) fn open(
    gateway: &Gateway,
    baud: Option<u32>,
    protocol: BusProtocol,
    ignore_checksum: bool,
) -> io::Result<DeviceHandle> {
    let device = gateway.device.as_str();
    let network = ["tcp://", "udp://"].iter().any(|p| device.starts_with(p));
    let serial = || {
        let baud = baud.unwrap_or_else(|| gateway.kind.default_baud());
        open_serial_rw(device, baud)
            .map_err(|e| io::Error::new(e.kind(), format!("opening serial port {device}: {e}")))
    };
    if device.starts_with("udp://") && !gateway.kind.is_line_gateway() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "{} takes tcp://host[:port], not udp://",
                gateway.kind.name()
            ),
        ));
    }
    Ok(match gateway.kind {
        GatewayKind::YachtDevices => {
            // A network endpoint is a YDWG-02 / YDEN; anything else is the
            // serial port of a YDNU-02, which is first put in RAW mode.
            let (reader, writer, init) = if network {
                let (r, w) = open_line_endpoint(device, 1457)?;
                (r, w, &[][..])
            } else {
                let (r, w) = serial()?;
                (r, w, line_gateway::YDNU_RAW_MODE)
            };
            line_gateway::run(
                reader,
                writer,
                line_gateway::LineFormat::YdwgRaw,
                protocol,
                init,
                ignore_checksum,
            )
        }
        GatewayKind::Miniplex => {
            let (reader, writer) = if network {
                let (pair, sends) = open_line_endpoint_sends(device, 10110)?;
                if !sends {
                    log::warn!(
                        "canboat can't set a MiniPlex it can't send to: set it to send \
                         every PGN as $MXPGN yourself ($PSMDNO options 30 and 31, \
                         mode 2), or give its address, udp://miniplex:port"
                    );
                }
                pair
            } else {
                serial()?
            };
            line_gateway::run(
                reader,
                writer,
                line_gateway::LineFormat::Mxpgn,
                protocol,
                line_gateway::MINIPLEX_MXPGN_ALL,
                ignore_checksum,
            )
        }
        GatewayKind::SeaSmart => {
            let (reader, writer) = if network {
                open_line_endpoint(device, 10001)?
            } else {
                serial()?
            };
            line_gateway::run(
                reader,
                writer,
                line_gateway::LineFormat::SeaSmart,
                protocol,
                &[],
                ignore_checksum,
            )
        }
        GatewayKind::Actisense(ActisenseFormat::RawAscii) => {
            let (reader, writer) = open_line_endpoint(device, 60002)?;
            line_gateway::run(
                reader,
                writer,
                line_gateway::LineFormat::YdwgRaw,
                protocol,
                &[],
                ignore_checksum,
            )
        }
        GatewayKind::Actisense(ActisenseFormat::N2kAscii) => {
            let (reader, writer) = open_line_endpoint(device, 60002)?;
            line_gateway::run(
                reader,
                writer,
                line_gateway::LineFormat::N2kAscii,
                protocol,
                &[],
                ignore_checksum,
            )
        }
        GatewayKind::Actisense(ActisenseFormat::N2kActisense) => {
            let (reader, writer) = open_tcp(device, 60002)?;
            super::bst_d0::run(reader, writer)
        }
        GatewayKind::Actisense(ActisenseFormat::RawActisense) => {
            // A network endpoint is a PRO-NDC-1E2K / W2K-1; anything else is
            // the serial port of an NGX.
            let (reader, writer) = if device.starts_with("tcp://") {
                open_tcp(device, 60002)?
            } else {
                serial()?
            };
            super::bst95::run(reader, writer, protocol)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_binary_gateway_is_not_opened_over_udp() {
        for kind in [
            GatewayKind::Actisense(ActisenseFormat::N2kActisense),
            GatewayKind::Actisense(ActisenseFormat::RawActisense),
        ] {
            let gateway = Gateway {
                kind,
                device: "udp://192.0.2.1:60002".into(),
            };
            let Err(e) = open(&gateway, None, BusProtocol::Nmea2000, false) else {
                panic!("{kind:?} opened over UDP");
            };
            assert_eq!(e.kind(), io::ErrorKind::InvalidInput, "{e}");
        }
    }
}
