// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Digital Yacht iKonvert runner: the sans-I/O
//! [`crate::engine::codec::ikonvert`] codec on a reader and a writer
//! thread.

use std::io::{self, Read, Write};

pub use crate::engine::codec::ikonvert::{Config, Ikonvert, pgn_list_status};

use super::DeviceHandle;
use crate::io::net::{open_tcp, open_udp};
use crate::io::open_serial_rw;

/// The port a NavLink2 serves the iKonvert protocol on, over TCP and over
/// UDP (Digital Yacht, "NavLink2 Developer's Guide" V1.04).
pub const NAVLINK2_PORT: u16 = 2000;

/// An iKonvert link: what it reads, what it writes, and whether it only
/// receives (`udp://`), in which case the gateway must not be initialised.
pub(crate) type Link = (Box<dyn Read + Send>, Box<dyn Write + Send>, bool);

/// Open `device`: a serial port at `baud`, `tcp://host[:port]` (a NavLink2
/// on port 2000, or the iKonvert protocol through ser2net), or
/// `udp://[bind:][port]` (port 2000), a NavLink2's UDP output, which only
/// receives. A `udp://` that names the gateway, to send to it, is refused:
/// a NavLink2 takes nothing over UDP.
pub(crate) fn open(device: &str, baud: u32) -> io::Result<Link> {
    if device.starts_with("tcp://") {
        let (r, w) = open_tcp(device, NAVLINK2_PORT)?;
        return Ok((r, w, false));
    }
    if let Some(rest) = device.strip_prefix("udp://") {
        let rest = if rest.is_empty() {
            NAVLINK2_PORT.to_string()
        } else {
            rest.to_string()
        };
        let ((r, w), sends) = open_udp(&rest)?;
        if sends {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "ikonvert: udp://{rest} names the gateway, to send to it, but a \
                     NavLink2 only sends over UDP: give a port (udp://{NAVLINK2_PORT}) \
                     to receive, or tcp://host to send"
                ),
            ));
        }
        log::warn!(
            "ikonvert over udp:// only receives: canboat expects the gateway to be on \
             the bus already, put there by another client, and never takes it off; \
             tcp:// is the better mode"
        );
        return Ok((r, w, true));
    }
    let (r, w) = open_serial_rw(device, baud)
        .map_err(|e| io::Error::new(e.kind(), format!("opening serial port {device}: {e}")))?;
    Ok((r, w, false))
}

/// Start the iKonvert reader/writer threads.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    super::run_codec(Ikonvert::new(config), None, reader, writer)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::UdpSocket;

    /// A NavLink2's UDP output only receives: a local address is bound,
    /// and what is sent to it comes out of the reader.
    #[test]
    fn udp_receives_on_a_local_address() {
        // A free port, which another test may take before it is bound
        // again: then try another.
        let (port, (mut reader, _writer, receive_only)) = (0..10)
            .find_map(|_| {
                let port = UdpSocket::bind("127.0.0.1:0")
                    .ok()?
                    .local_addr()
                    .ok()?
                    .port();
                Some((port, open(&format!("udp://127.0.0.1:{port}"), 0).ok()?))
            })
            .expect("a free UDP port");
        assert!(receive_only);
        let line = b"!PDGY,127250,2,3,255,481.734,2C20AAAAAAE=\r\n";
        UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .send_to(line, ("127.0.0.1", port))
            .unwrap();
        let mut buf = [0u8; 256];
        let n = reader.read(&mut buf).unwrap();
        assert_eq!(&buf[..n], line);
    }

    /// A `udp://` naming the gateway, to send to it, is refused: a
    /// NavLink2 takes nothing over UDP.
    #[test]
    fn udp_to_the_gateway_is_refused() {
        let Err(e) = open("udp://192.0.2.1:2000", 0) else {
            panic!("opened udp:// to a gateway");
        };
        assert_eq!(e.kind(), io::ErrorKind::InvalidInput, "{e}");
        assert!(e.to_string().contains("tcp://"), "{e}");
    }
}
