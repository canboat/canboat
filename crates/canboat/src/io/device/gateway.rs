// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The network gateways — and their serial cousins — that `canboat
//! interface --kind` and `canboat server` open the same way: Yacht
//! Devices RAW, Actisense N2K ASCII, BST-D0 and BST-95.

use std::io;

use crate::engine::BusProtocol;
use crate::io::net::{open_line_endpoint, open_tcp};
use crate::io::open_serial_rw;

use super::{DeviceHandle, line_gateway};

/// Which gateway.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum GatewayKind {
    /// Yacht Devices RAW: a YDWG-02 / YDEN / NavLink2 RAW port over
    /// `tcp://host[:port]` (port 1457 if not given) or `udp://…`, or a
    /// YDNU-02 on its serial port (38 400 baud).
    Ydwg,
    /// Actisense W2K-1 in N2K ASCII mode, over `tcp://host[:port]` (port
    /// 60002) or `udp://…`.
    W2kAscii,
    /// Actisense BST-D0 (reassembled NMEA 2000): a W2K-1 or PRO-NDC-1E2K in
    /// Actisense mode, over `tcp://host[:port]` (port 60002).
    BstD0,
    /// Actisense BST-95 (raw CAN frames): a PRO-NDC-1E2K or W2K-1 in "CAN
    /// Actisense" mode over `tcp://host[:port]` (port 60002), or an NGX in
    /// CAN Packet mode on its serial port (115 200 baud).
    Bst95,
}

impl GatewayKind {
    /// The serial speed when none is given, for a gateway on a serial port.
    pub fn default_baud(self) -> u32 {
        match self {
            GatewayKind::Ydwg => 38_400,
            GatewayKind::Bst95 => 115_200,
            GatewayKind::W2kAscii | GatewayKind::BstD0 => 0,
        }
    }

    /// Whether it passes raw CAN frames, and so any [`BusProtocol`]; the
    /// others do NMEA 2000 framing themselves.
    pub fn passes_raw_frames(self) -> bool {
        matches!(self, GatewayKind::Ydwg | GatewayKind::Bst95)
    }

    /// A short name for log lines.
    pub fn name(self) -> &'static str {
        match self {
            GatewayKind::Ydwg => "ydwg",
            GatewayKind::W2kAscii => "w2k-ascii",
            GatewayKind::BstD0 => "bst-d0",
            GatewayKind::Bst95 => "bst-95",
        }
    }
}

/// A gateway, and where it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Gateway {
    pub kind: GatewayKind,
    /// `tcp://host[:port]`, `udp://…` (the line gateways), or a serial
    /// port (ydwg, bst-95).
    pub device: String,
}

/// Open `gateway` and start its codec. `baud`, for a serial port, defaults
/// to [`GatewayKind::default_baud`].
pub(crate) fn open(
    gateway: &Gateway,
    baud: Option<u32>,
    protocol: BusProtocol,
) -> io::Result<DeviceHandle> {
    let device = gateway.device.as_str();
    let network = ["tcp://", "udp://"].iter().any(|p| device.starts_with(p));
    let serial = || {
        let baud = baud.unwrap_or_else(|| gateway.kind.default_baud());
        open_serial_rw(device, baud)
            .map_err(|e| io::Error::new(e.kind(), format!("opening serial port {device}: {e}")))
    };
    Ok(match gateway.kind {
        GatewayKind::Ydwg => {
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
            )
        }
        GatewayKind::W2kAscii => {
            let (reader, writer) = open_line_endpoint(device, 60002)?;
            line_gateway::run(
                reader,
                writer,
                line_gateway::LineFormat::N2kAscii,
                protocol,
                &[],
            )
        }
        GatewayKind::BstD0 => {
            let (reader, writer) = open_tcp(device, 60002)?;
            super::bst_d0::run(reader, writer)
        }
        GatewayKind::Bst95 => {
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
