// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! FTDI serial gateways opened directly over USB, without an OS serial
//! driver.
//!
//! The Actisense NGT-1 (and its siblings) is an FTDI FT232R behind
//! Actisense's own product id `0403:d9aa`. Linux's `ftdi_sio` knows that
//! id, but macOS's built-in FTDI driver does not, so no
//! `/dev/cu.usbserial-*` ever appears there. The UART protocol on top of
//! the chip is small: a few vendor control requests to set the line up,
//! then bulk transfers where every IN packet starts with two modem-status
//! bytes. This module speaks it through `nusb`, so a device spelled
//! `usb:[serial]` works wherever the interface is not claimed by a driver.
//!
//! Only the single-port chips with a 3 MHz baud clock (FT232BM/FT232R/FT-X)
//! are handled; every Actisense gateway is one of those.

use std::fmt;
use std::io::{self, Read, Write};
use std::time::Duration;

use nusb::transfer::{Buffer, Bulk, ControlOut, ControlType, In, Out, Recipient, TransferError};
use nusb::{Endpoint, Interface, MaybeFuture};

/// FTDI's vendor id.
pub const FTDI_VID: u16 = 0x0403;

/// Actisense product ids on FTDI's vendor id — the whole `ACTISENSE_*_PID`
/// block in Linux's `ftdi_sio_ids.h`: NDC, USG, NGT, NGW, UID, USA, NGX and
/// one reserved id. A plain `usb` also takes an FTDI device whose USB
/// manufacturer string names Actisense, so a later product id works too.
pub const ACTISENSE_PIDS: std::ops::RangeInclusive<u16> = 0xD9A8..=0xD9AF;

/// How long a read waits for data before reporting `TimedOut`; matches
/// the serial-port timeout in [`super::open_serial`] so device runners
/// tick at the same rate on either transport.
const READ_TIMEOUT: Duration = Duration::from_millis(250);
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// Bulk IN transfers kept queued, and the packets each one asks for.
const READ_TRANSFERS: usize = 4;
const PACKETS_PER_TRANSFER: usize = 8;

/// Bytes of modem/line status that open every IN packet.
const STATUS_LEN: usize = 2;

// FTDI vendor requests (libftdi / ftdi_sio names).
const SIO_RESET: u8 = 0;
const SIO_SET_MODEM_CTRL: u8 = 1;
const SIO_SET_FLOW_CTRL: u8 = 2;
const SIO_SET_BAUDRATE: u8 = 3;
const SIO_SET_DATA: u8 = 4;
const SIO_SET_LATENCY_TIMER: u8 = 9;

const SIO_RESET_SIO: u16 = 0;
const SIO_RESET_PURGE_RX: u16 = 1;
const SIO_RESET_PURGE_TX: u16 = 2;
/// DTR and RTS high, each with its "change this line" mask bit — what a
/// tty open does on the other platforms.
const SIO_SET_DTR_RTS_HIGH: u16 = 0x0303;
/// 8 data bits, no parity, 1 stop bit.
const SIO_DATA_8N1: u16 = 8;
/// Milliseconds the chip holds a partial IN packet; 16 (the default)
/// delays every short NGT-1 frame by that much.
const LATENCY_MS: u16 = 2;

/// Which USB device a `usb:` spelling names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Selector {
    /// `None`: any Actisense device on [`FTDI_VID`] (see [`ACTISENSE_PIDS`]).
    pub ids: Option<(u16, u16)>,
    /// `None`: any serial number.
    pub serial: Option<String>,
}

impl Selector {
    /// Parse `usb`, `usb:SERIAL`, `usb:VVVV:PPPP` or `usb:VVVV:PPPP:SERIAL`
    /// (ids in hex). Returns `None` when `device` is not a `usb` spelling.
    pub fn parse(device: &str) -> Option<io::Result<Self>> {
        let rest = match device.strip_prefix("usb") {
            Some("") => "",
            Some(r) => r.strip_prefix(':')?,
            None => return None,
        };
        let parts: Vec<&str> = if rest.is_empty() {
            Vec::new()
        } else {
            rest.split(':').collect()
        };
        let hex = |s: &str| {
            u16::from_str_radix(s, 16).map_err(|_| {
                io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("'{s}' is not a hex USB id"),
                )
            })
        };
        let serial = |s: &str| (!s.is_empty()).then(|| s.to_string());
        Some(match parts.as_slice() {
            [] => Ok(Self {
                ids: None,
                serial: None,
            }),
            [s] => Ok(Self {
                ids: None,
                serial: serial(s),
            }),
            [v, p] | [v, p, _] => (|| {
                Ok(Self {
                    ids: Some((hex(v)?, hex(p)?)),
                    serial: parts.get(2).and_then(|s| serial(s)),
                })
            })(),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "expected usb[:SERIAL] or usb:VVVV:PPPP[:SERIAL]".to_string(),
            )),
        })
    }

    fn matches(&self, c: &Candidate) -> bool {
        let ids_ok = match self.ids {
            Some((vid, pid)) => c.vid == vid && c.pid == pid,
            None => c.is_actisense(),
        };
        ids_ok
            && self
                .serial
                .as_deref()
                .is_none_or(|want| c.serial.as_deref() == Some(want))
    }
}

/// What matching and the error listings need from an attached device.
#[derive(Debug, Clone, Default)]
struct Candidate {
    vid: u16,
    pid: u16,
    manufacturer: Option<String>,
    product: Option<String>,
    serial: Option<String>,
}

impl Candidate {
    fn new(info: &nusb::DeviceInfo) -> Self {
        Self {
            vid: info.vendor_id(),
            pid: info.product_id(),
            manufacturer: info.manufacturer_string().map(str::to_string),
            product: info.product_string().map(str::to_string),
            serial: info.serial_number().map(str::to_string),
        }
    }

    fn is_actisense(&self) -> bool {
        self.vid == FTDI_VID
            && (ACTISENSE_PIDS.contains(&self.pid)
                || self
                    .manufacturer
                    .as_deref()
                    .is_some_and(|m| m.to_ascii_lowercase().contains("actisense")))
    }

    /// The shortest device spelling that picks this device out of
    /// everything `attached`: `usb:SERIAL` for an Actisense device whose
    /// serial number no other Actisense device shares (that spelling only
    /// considers Actisense devices), else with the ids in front. `None`
    /// when nothing tells it apart.
    fn spelling(&self, attached: &[Candidate]) -> Option<String> {
        let same_serial = |c: &&Candidate| c.serial == self.serial;
        let same_ids = |c: &&Candidate| c.vid == self.vid && c.pid == self.pid;
        let ids = format!("usb:{:04x}:{:04x}", self.vid, self.pid);
        match &self.serial {
            Some(serial)
                if self.is_actisense()
                    && attached
                        .iter()
                        .filter(|c| c.is_actisense())
                        .filter(same_serial)
                        .count()
                        == 1 =>
            {
                Some(format!("usb:{serial}"))
            }
            Some(serial) if attached.iter().filter(same_ids).filter(same_serial).count() == 1 => {
                Some(format!("{ids}:{serial}"))
            }
            None if attached.iter().filter(same_ids).count() == 1 => Some(ids),
            _ => None,
        }
    }

    fn describe(&self) -> String {
        let name = self.product.as_deref().unwrap_or("unnamed device");
        match &self.serial {
            Some(serial) => format!("{name}, serial {serial}"),
            None => format!("{name}, no serial number"),
        }
    }
}

/// One line per device in `shown`, each with the spelling that selects
/// it from everything `attached`.
fn listing(shown: &[Candidate], attached: &[Candidate]) -> String {
    shown
        .iter()
        .map(|c| match c.spelling(attached) {
            Some(spelling) => format!("\n  {spelling:<20} {}", c.describe()),
            None => format!(
                "\n  {:<20} {} (cannot be told apart from another)",
                "?",
                c.describe()
            ),
        })
        .collect()
}

/// Choose the one device `selector` names from what is attached, or say
/// why not — listing the Actisense devices present and how to name each.
fn choose(selector: &Selector, attached: &[Candidate]) -> io::Result<usize> {
    let matched: Vec<usize> = (0..attached.len())
        .filter(|&i| selector.matches(&attached[i]))
        .collect();
    let actisense: Vec<Candidate> = attached
        .iter()
        .filter(|c| c.is_actisense())
        .cloned()
        .collect();
    match matched.as_slice() {
        [one] => Ok(*one),
        [] => {
            let msg = if actisense.is_empty() {
                "no matching USB device, and no Actisense device is attached".to_string()
            } else {
                format!(
                    "no matching USB device; Actisense devices attached:{}",
                    listing(&actisense, attached)
                )
            };
            Err(io::Error::new(io::ErrorKind::NotFound, msg))
        }
        many => {
            let matched: Vec<Candidate> = many.iter().map(|&i| attached[i].clone()).collect();
            Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "{} USB devices match; name the one to use:{}",
                    matched.len(),
                    listing(&matched, attached)
                ),
            ))
        }
    }
}

impl fmt::Display for Selector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("usb")?;
        if let Some((vid, pid)) = self.ids {
            write!(f, ":{vid:04x}:{pid:04x}")?;
        }
        if let Some(serial) = &self.serial {
            write!(f, ":{serial}")?;
        }
        Ok(())
    }
}

/// The FT232BM/R/X divisor for `baud` off the chip's 3 MHz baud clock, as
/// the `(wValue, wIndex)` of `SIO_SET_BAUDRATE`, plus the baud rate that
/// divisor really gives. Integer part in bits 0..14, eighths in a
/// scrambled 3-bit code in bits 14..17 (libftdi `ftdi_to_clkbits`).
fn baud_divisor(baud: u32) -> io::Result<(u16, u16, u32)> {
    const CLOCK: u32 = 3_000_000;
    const FRAC_CODE: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
    if baud == 0 || baud > CLOCK {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("baud rate {baud} is out of range for an FTDI chip"),
        ));
    }
    let (encoded, actual) = if baud >= CLOCK {
        (0, CLOCK)
    } else if baud >= CLOCK * 2 / 3 {
        // Divisor 1.5 has a dedicated code.
        (1, CLOCK * 2 / 3)
    } else if baud >= CLOCK / 2 {
        // Fractions below 2 are not allowed (AN120), so this band is /2.
        (2, CLOCK / 2)
    } else {
        // Divisor in sixteenths, rounded to eighths.
        let sixteenths = CLOCK * 16 / baud;
        let eighths = (sixteenths.div_ceil(2)).min(0x1_FFFF);
        let encoded = (eighths >> 3) | (FRAC_CODE[(eighths & 7) as usize] << 14);
        (encoded, CLOCK * 8 / eighths)
    };
    Ok(((encoded & 0xFFFF) as u16, (encoded >> 16) as u16, actual))
}

/// Remove the two status bytes that head every `packet_size` packet of an
/// IN transfer, appending the payload to `out`.
fn strip_status(data: &[u8], packet_size: usize, out: &mut Vec<u8>) {
    for packet in data.chunks(packet_size) {
        if packet.len() > STATUS_LEN {
            out.extend_from_slice(&packet[STATUS_LEN..]);
        }
    }
}

/// Open the FTDI gateway `selector` names at `baud` 8N1 and return an
/// independent `(reader, writer)` pair, like [`super::open_serial_rw`].
pub fn open_rw(
    selector: &Selector,
    baud: u32,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    let mut devices: Vec<nusb::DeviceInfo> = nusb::list_devices()
        .wait()
        .map_err(io::Error::other)?
        .collect();
    let attached: Vec<Candidate> = devices.iter().map(Candidate::new).collect();
    let info = devices.swap_remove(choose(selector, &attached)?);
    // bcdDevice names the chip: 0x0400 FT232BM, 0x0600 FT232R, 0x1000
    // FT-X — single-port, 3 MHz baud clock. The rest need another divisor
    // encoding (AM, and the 12 MHz Hi-Speed parts) or a port index in
    // every request (FT2232C/D and the multi-port Hi-Speed parts).
    let chip = info.device_version();
    if ![0x0400, 0x0600, 0x1000].contains(&chip) {
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!("FTDI chip revision {chip:#06x} is not supported"),
        ));
    }
    let device = info.open().wait().map_err(io::Error::other)?;
    let interface = device.claim_interface(0).wait().map_err(|e| {
        io::Error::other(format!(
            "claiming the USB interface: {e} (is another program, or a serial driver, using it?)"
        ))
    })?;

    let (value, index, actual) = baud_divisor(baud)?;
    if actual.abs_diff(baud) * 100 > baud * 3 {
        log::warn!("{selector}: {baud} baud is {actual} on the wire");
    }
    let control = |request, value, index| {
        interface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data: &[],
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| io::Error::other(format!("FTDI request {request}: {e}")))
    };
    control(SIO_RESET, SIO_RESET_SIO, 0)?;
    control(SIO_SET_BAUDRATE, value, index)?;
    control(SIO_SET_DATA, SIO_DATA_8N1, 0)?;
    control(SIO_SET_FLOW_CTRL, 0, 0)?;
    control(SIO_SET_MODEM_CTRL, SIO_SET_DTR_RTS_HIGH, 0)?;
    control(SIO_SET_LATENCY_TIMER, LATENCY_MS, 0)?;
    control(SIO_RESET, SIO_RESET_PURGE_RX, 0)?;
    control(SIO_RESET, SIO_RESET_PURGE_TX, 0)?;

    let (in_addr, out_addr) = bulk_endpoints(&interface)?;
    let reader = UsbReader::new(&interface, in_addr)?;
    let writer = UsbWriter {
        endpoint: interface
            .endpoint::<Bulk, Out>(out_addr)
            .map_err(io::Error::other)?,
        _interface: interface,
    };
    log::info!(
        "{selector}: opened {} {} at {baud} baud",
        info.product_string().unwrap_or("FTDI device"),
        info.serial_number().unwrap_or("")
    );
    Ok((Box::new(reader), Box::new(writer)))
}

/// The bulk IN and OUT endpoint addresses of interface 0.
fn bulk_endpoints(interface: &Interface) -> io::Result<(u8, u8)> {
    let desc = interface
        .descriptor()
        .ok_or_else(|| io::Error::other("USB interface has no descriptor"))?;
    let bulk: Vec<u8> = desc
        .endpoints()
        .filter(|e| e.transfer_type() == nusb::descriptors::TransferType::Bulk)
        .map(|e| e.address())
        .collect();
    let in_addr = bulk.iter().copied().find(|a| a & 0x80 != 0);
    let out_addr = bulk.iter().copied().find(|a| a & 0x80 == 0);
    in_addr
        .zip(out_addr)
        .ok_or_else(|| io::Error::other("USB interface has no bulk IN/OUT endpoint pair"))
}

/// Maps a transfer failure onto the `io` error the device supervisor
/// treats as "gone": an unplugged gateway must end the session so it can
/// be reopened.
fn transfer_error(e: TransferError) -> io::Error {
    match e {
        TransferError::Disconnected => io::Error::new(io::ErrorKind::NotConnected, e),
        _ => io::Error::other(e),
    }
}

struct UsbReader {
    endpoint: Endpoint<Bulk, In>,
    packet_size: usize,
    /// Payload received but not yet handed out, from `pos`.
    pending: Vec<u8>,
    pos: usize,
    _interface: Interface,
}

impl UsbReader {
    fn new(interface: &Interface, address: u8) -> io::Result<Self> {
        let mut endpoint = interface
            .endpoint::<Bulk, In>(address)
            .map_err(io::Error::other)?;
        let packet_size = endpoint.max_packet_size();
        for _ in 0..READ_TRANSFERS {
            let buf = endpoint.allocate(packet_size * PACKETS_PER_TRANSFER);
            endpoint.submit(buf);
        }
        Ok(Self {
            endpoint,
            packet_size,
            pending: Vec::with_capacity(packet_size * PACKETS_PER_TRANSFER),
            pos: 0,
            _interface: interface.clone(),
        })
    }
}

impl Read for UsbReader {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        // The chip sends a status-only packet every latency period even
        // when idle, so loop until there is payload or the timeout passes.
        let deadline = std::time::Instant::now() + READ_TIMEOUT;
        while self.pos == self.pending.len() {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            let Some(done) = self.endpoint.wait_next_complete(left) else {
                return Err(io::ErrorKind::TimedOut.into());
            };
            let mut buf: Buffer = done.buffer;
            let status = done.status;
            self.pending.clear();
            self.pos = 0;
            if status.is_ok() {
                strip_status(&buf, self.packet_size, &mut self.pending);
            }
            buf.clear();
            buf.set_requested_len(self.packet_size * PACKETS_PER_TRANSFER);
            self.endpoint.submit(buf);
            status.map_err(transfer_error)?;
        }
        let n = out.len().min(self.pending.len() - self.pos);
        out[..n].copy_from_slice(&self.pending[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

struct UsbWriter {
    endpoint: Endpoint<Bulk, Out>,
    _interface: Interface,
}

impl Write for UsbWriter {
    fn write(&mut self, data: &[u8]) -> io::Result<usize> {
        if data.is_empty() {
            return Ok(0);
        }
        let done = self
            .endpoint
            .transfer_blocking(Buffer::from(data), WRITE_TIMEOUT);
        done.status.map_err(transfer_error)?;
        Ok(done.actual_len)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisors_match_libftdi() {
        assert_eq!(baud_divisor(230_400).unwrap(), (13, 0, 230_769));
        assert_eq!(baud_divisor(115_200).unwrap(), (26, 0, 115_384));
        // 312.5: half an eighth-step, frac code 1 in bit 14.
        assert_eq!(baud_divisor(9600).unwrap(), (0x4138, 0, 9600));
        assert_eq!(baud_divisor(3_000_000).unwrap(), (0, 0, 3_000_000));
        assert_eq!(baud_divisor(2_000_000).unwrap(), (1, 0, 2_000_000));
        // Between 2 MHz and 1.5 MHz only /2 is legal (libftdi does the same).
        assert_eq!(baud_divisor(1_900_000).unwrap(), (2, 0, 1_500_000));
        assert_eq!(baud_divisor(1_500_000).unwrap(), (2, 0, 1_500_000));
        assert_eq!(baud_divisor(921_600).unwrap(), (3 | (2 << 14), 0, 923_076));
        assert!(baud_divisor(0).is_err());
    }

    #[test]
    fn divisor_uses_bit_16_for_the_top_fraction_codes() {
        // 3_000_000 / 38400 = 78.125 = 78 + 1/8 → frac code 3 → bits 14, 15.
        assert_eq!(baud_divisor(38_400).unwrap().0, 78 | (3 << 14));
        // x + 3/8 → frac code 4 → bit 16, carried in wIndex.
        let (value, index, _) = baud_divisor(3_000_000 * 8 / (100 * 8 + 3)).unwrap();
        assert_eq!((value, index), (100, 1));
    }

    #[test]
    fn status_bytes_are_stripped_per_packet() {
        let mut data = vec![0x01, 0x60];
        data.extend(0..62u8);
        data.extend([0x01, 0x60, 0xAA, 0xBB]);
        let mut out = Vec::new();
        strip_status(&data, 64, &mut out);
        let mut want: Vec<u8> = (0..62u8).collect();
        want.extend([0xAA, 0xBB]);
        assert_eq!(out, want);

        out.clear();
        strip_status(&[0x01, 0x60], 64, &mut out);
        assert!(out.is_empty());
    }

    fn dev(pid: u16, product: &str, serial: Option<&str>) -> Candidate {
        Candidate {
            vid: FTDI_VID,
            pid,
            manufacturer: Some("Actisense".into()),
            product: Some(product.into()),
            serial: serial.map(str::to_string),
        }
    }

    fn any() -> Selector {
        Selector::parse("usb").unwrap().unwrap()
    }

    #[test]
    fn plain_usb_takes_every_actisense_product() {
        let ngx = dev(0xD9AE, "NGX-1", Some("A1"));
        assert!(any().matches(&ngx));
        // A product id outside the known block, named by its manufacturer.
        let future = dev(0xE000, "NGX-2", Some("A2"));
        assert!(any().matches(&future));
        // A plain FTDI cable is not a gateway.
        let cable = Candidate {
            manufacturer: Some("FTDI".into()),
            ..dev(0x6001, "FT232R USB UART", Some("B1"))
        };
        assert!(!any().matches(&cable));
        // Neither is another vendor's chip that says Actisense.
        let other_vid = Candidate {
            vid: 0x10C4,
            ..dev(0xD9AA, "NGT-1", Some("C1"))
        };
        assert!(!any().matches(&other_vid));
    }

    #[test]
    fn one_actisense_device_among_others_is_chosen() {
        let attached = [
            Candidate {
                manufacturer: Some("FTDI".into()),
                ..dev(0x6001, "FT232R USB UART", Some("B1"))
            },
            dev(0xD9AA, "NGT-1-A", Some("19FAC")),
        ];
        assert_eq!(choose(&any(), &attached).unwrap(), 1);
    }

    #[test]
    fn several_matches_are_refused_with_how_to_name_each() {
        let attached = [
            dev(0xD9AA, "NGT-1-A", Some("19FAC")),
            dev(0xD9AE, "NGX-1", Some("2B3C4")),
        ];
        let err = choose(&any(), &attached).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidInput);
        let msg = err.to_string();
        assert!(
            msg.starts_with("2 USB devices match; name the one to use:"),
            "{msg}"
        );
        assert!(
            msg.contains("\n  usb:19FAC            NGT-1-A, serial 19FAC"),
            "{msg}"
        );
        assert!(
            msg.contains("\n  usb:2B3C4            NGX-1, serial 2B3C4"),
            "{msg}"
        );
        // Following the advice works.
        let pick = Selector::parse("usb:2B3C4").unwrap().unwrap();
        assert_eq!(choose(&pick, &attached).unwrap(), 1);
    }

    #[test]
    fn spelling_falls_back_to_ids_when_the_serial_does_not_decide() {
        let attached = [
            dev(0xD9AA, "NGT-1", Some("X")),
            dev(0xD9AE, "NGX-1", Some("X")),
            dev(0xD9A9, "USG-1", None),
            dev(0xD9AB, "NGW-1", None),
            dev(0xD9AB, "NGW-1", None),
        ];
        let spell: Vec<Option<String>> = attached.iter().map(|c| c.spelling(&attached)).collect();
        assert_eq!(
            spell,
            [
                Some("usb:0403:d9aa:X".into()),
                Some("usb:0403:d9ae:X".into()),
                Some("usb:0403:d9a9".into()),
                None,
                None,
            ]
        );
        assert!(listing(&attached, &attached).contains("cannot be told apart"));
    }

    #[test]
    fn non_actisense_matches_are_named_with_their_ids() {
        // `usb:SERIAL` only looks at Actisense devices, so it must not be
        // offered for a plain FTDI cable picked by explicit ids.
        let cable = |serial| Candidate {
            manufacturer: Some("FTDI".into()),
            ..dev(0x6001, "FT232R USB UART", Some(serial))
        };
        let attached = [cable("B1"), cable("B2"), dev(0xD9AA, "NGT-1-A", Some("B1"))];
        let ids = Selector::parse("usb:0403:6001").unwrap().unwrap();
        let msg = choose(&ids, &attached).unwrap_err().to_string();
        assert!(msg.contains("\n  usb:0403:6001:B1 "), "{msg}");
        assert!(msg.contains("\n  usb:0403:6001:B2 "), "{msg}");
        // The NGT-1 keeps the short form: no other Actisense device is B1.
        assert_eq!(attached[2].spelling(&attached).as_deref(), Some("usb:B1"));
        // And each suggestion selects exactly its device.
        for (i, c) in attached.iter().enumerate() {
            let pick = Selector::parse(&c.spelling(&attached).unwrap())
                .unwrap()
                .unwrap();
            assert_eq!(choose(&pick, &attached).unwrap(), i);
        }
    }

    #[test]
    fn a_missing_device_lists_what_is_attached() {
        let attached = [dev(0xD9AA, "NGT-1-A", Some("19FAC"))];
        let wrong = Selector::parse("usb:NOPE").unwrap().unwrap();
        let err = choose(&wrong, &attached).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::NotFound);
        assert!(err.to_string().contains("usb:19FAC"), "{err}");
        let err = choose(&any(), &[]).unwrap_err();
        assert!(err.to_string().contains("no Actisense device"), "{err}");
    }

    #[test]
    fn selector_spellings() {
        let p = |s: &str| Selector::parse(s).map(|r| r.unwrap());
        assert_eq!(p("/dev/ttyUSB0"), None);
        assert_eq!(p("usbserial"), None);
        let any = Selector {
            ids: None,
            serial: None,
        };
        assert_eq!(p("usb"), Some(any.clone()));
        assert_eq!(p("usb:"), Some(any));
        assert_eq!(
            p("usb:19FAC"),
            Some(Selector {
                ids: None,
                serial: Some("19FAC".into())
            })
        );
        assert_eq!(
            p("usb:0403:d9aa:19FAC"),
            Some(Selector {
                ids: Some((0x0403, 0xD9AA)),
                serial: Some("19FAC".into())
            })
        );
        assert_eq!(p("usb:0403:d9aa").unwrap().to_string(), "usb:0403:d9aa");
        assert!(Selector::parse("usb:zz:1").unwrap().is_err());
        assert!(Selector::parse("usb:1:2:3:4").unwrap().is_err());
    }
}
