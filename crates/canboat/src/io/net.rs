// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Network gateway endpoints, as a `(reader, writer)` pair for the
//! device runners: `tcp://host[:port]`, and `udp://…` for the line
//! gateways.

use std::io::{self, Read, Write};
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::time::Duration;

/// A reader and a writer over one connection.
pub(crate) type Pair = (Box<dyn Read + Send>, Box<dyn Write + Send>);

/// As a serial port's: a quiet connection still ticks the codec, so its
/// deadlines (frames held after Set Operating Mode) advance.
const READ_TIMEOUT: Duration = Duration::from_millis(250);

const CONNECT_TIMEOUT: Duration = Duration::from_secs(10);

/// `host:port` from `device` without its `scheme`, `default_port` when it
/// names none.
fn endpoint(device: &str, scheme: &str, default_port: u16) -> String {
    let endpoint = device.strip_prefix(scheme).unwrap_or(device);
    if endpoint.contains(':') {
        endpoint.to_string()
    } else {
        format!("{endpoint}:{default_port}")
    }
}

/// Connect to `tcp://host[:port]` (or `host[:port]`), trying each address
/// `host` resolves to.
pub(crate) fn open_tcp(device: &str, default_port: u16) -> io::Result<Pair> {
    let endpoint = endpoint(device, "tcp://", default_port);
    let context = |e: io::Error| io::Error::new(e.kind(), format!("connecting to {endpoint}: {e}"));
    let mut last = io::Error::new(io::ErrorKind::NotFound, "no addresses");
    let mut connected = None;
    for addr in endpoint.to_socket_addrs().map_err(context)? {
        match TcpStream::connect_timeout(&addr, CONNECT_TIMEOUT) {
            Ok(stream) => {
                connected = Some(stream);
                break;
            }
            Err(e) => last = e,
        }
    }
    let stream = connected.ok_or_else(|| context(last))?;
    stream.set_read_timeout(Some(READ_TIMEOUT))?;
    super::set_tcp_keepalive(&stream)?;
    let reader: Box<dyn Read + Send> = Box::new(stream.try_clone()?);
    Ok((reader, Box::new(stream)))
}

/// Open a line gateway's endpoint: `tcp://host[:port]`, or `udp://…`.
///
/// `udp://port`, or `udp://bind:port` with an address of this host,
/// only receives: what is sent is dropped, with a warning. With the
/// gateway's address, `udp://gateway:port` also sends, a datagram per
/// line, to that port (a YDEN-02 with its UDP server set to
/// Bidirectional, canboat#1039), and takes only the gateway's datagrams.
///
/// Also says whether what is written reaches the gateway: not for a
/// receive-only `udp://`.
pub(crate) fn open_line_endpoint_sends(
    device: &str,
    default_tcp_port: u16,
) -> io::Result<(Pair, bool)> {
    let Some(rest) = device.strip_prefix("udp://") else {
        return Ok((open_tcp(device, default_tcp_port)?, true));
    };
    let (pair, sends) = open_udp(rest)?;
    if sends {
        log::info!("UDP: sending to the gateway at udp://{rest}, and receiving from it");
    } else {
        log::warn!(
            "UDP gateway mode is receive-only; transmitted frames are dropped \
             (give the gateway's address, udp://gateway:port, to send)"
        );
    }
    Ok((pair, sends))
}

/// A `udp://` endpoint, `rest` being what follows `udp://`: a port, or
/// `host:port`. Receives only for a port or an address of this host, and
/// otherwise also sends to `host`, which it then only receives from;
/// whether it sends is returned alongside.
pub(crate) fn open_udp(rest: &str) -> io::Result<(Pair, bool)> {
    let addr = if rest.contains(':') {
        rest.to_socket_addrs()?.next().ok_or_else(|| {
            io::Error::new(io::ErrorKind::NotFound, format!("no addresses for {rest}"))
        })?
    } else {
        let port = rest.parse().map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("udp://{rest}: expected a port, or host:port"),
            )
        })?;
        SocketAddr::new(Ipv4Addr::UNSPECIFIED.into(), port)
    };
    let gateway = if addr.ip().is_unspecified() {
        None
    } else {
        match UdpSocket::bind(addr) {
            Ok(sock) => return Ok((receive_only(sock), false)),
            // Not an address of this host: the gateway's.
            Err(e) if e.kind() == io::ErrorKind::AddrNotAvailable => Some(addr),
            Err(e) => return Err(e),
        }
    };
    let any: IpAddr = match addr {
        SocketAddr::V4(_) => Ipv4Addr::UNSPECIFIED.into(),
        SocketAddr::V6(_) => Ipv6Addr::UNSPECIFIED.into(),
    };
    let sock = UdpSocket::bind((any, addr.port()))?;
    let Some(gateway) = gateway else {
        return Ok((receive_only(sock), false));
    };
    sock.set_read_timeout(Some(READ_TIMEOUT))?;
    let reader: Box<dyn Read + Send> = Box::new(UdpRead {
        sock: sock.try_clone()?,
        from: Some(gateway.ip()),
    });
    let writer: Box<dyn Write + Send> = Box::new(UdpLines {
        sock,
        to: gateway,
        pending: Vec::new(),
    });
    Ok(((reader, writer), true))
}

fn receive_only(sock: UdpSocket) -> Pair {
    let _ = sock.set_read_timeout(Some(READ_TIMEOUT));
    (Box::new(UdpRead { sock, from: None }), Box::new(io::sink()))
}

/// [`Read`] over a bound UDP socket: each `read` yields one datagram,
/// from `from` when that is set.
struct UdpRead {
    sock: UdpSocket,
    from: Option<IpAddr>,
}

impl Read for UdpRead {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        loop {
            let (n, peer) = self.sock.recv_from(buf)?;
            if self.from.is_none_or(|ip| ip == peer.ip()) {
                return Ok(n);
            }
        }
    }
}

/// [`Write`] that sends each complete line as a datagram to `to`.
struct UdpLines {
    sock: UdpSocket,
    to: SocketAddr,
    pending: Vec<u8>,
}

impl Write for UdpLines {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.pending.extend_from_slice(buf);
        while let Some(end) = self.pending.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.pending.drain(..=end).collect();
            self.sock.send_to(&line, self.to)?;
        }
        Ok(buf.len())
    }

    /// A partial line waits for its end.
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn udp_lines_go_out_a_datagram_each() {
        let gateway = UdpSocket::bind("127.0.0.1:0").unwrap();
        gateway
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut w = UdpLines {
            sock: UdpSocket::bind("127.0.0.1:0").unwrap(),
            to: gateway.local_addr().unwrap(),
            pending: Vec::new(),
        };
        w.write_all(b"09F80100 01 02\r\n09F80100 03").unwrap();
        w.write_all(b" 04\r\n").unwrap();
        let mut buf = [0u8; 64];
        let n = gateway.recv(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"09F80100 01 02\r\n");
        let n = gateway.recv(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"09F80100 03 04\r\n");
    }

    #[test]
    fn udp_read_takes_only_the_gateways_datagrams() {
        let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        let to = sock.local_addr().unwrap();
        let gateway = UdpSocket::bind("127.0.0.1:0").unwrap();
        let mut r = UdpRead {
            sock,
            from: Some(Ipv4Addr::LOCALHOST.into()),
        };
        gateway.send_to(b"a line\r\n", to).unwrap();
        let mut buf = [0u8; 64];
        let n = r.read(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"a line\r\n");

        r.from = Some(Ipv4Addr::new(192, 0, 2, 1).into());
        r.sock
            .set_read_timeout(Some(Duration::from_millis(100)))
            .unwrap();
        gateway.send_to(b"another\r\n", to).unwrap();
        let e = r.read(&mut buf).unwrap_err();
        assert!(
            matches!(
                e.kind(),
                io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
            ),
            "{e:?}"
        );
    }

    /// An address of this host is one to bind to: receive only.
    #[test]
    fn a_local_address_only_receives() {
        let ((_r, mut w), sends) = open_line_endpoint_sends("udp://127.0.0.1:0", 1457).unwrap();
        assert!(!sends);
        // io::sink takes anything.
        w.write_all(b"09F80100 01\r\n").unwrap();
    }
}
