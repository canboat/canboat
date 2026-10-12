// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! CANalyst-II adapters opened over USB: the Waveshare USB-CAN-B and the
//! other "Chuangxin Tech USBCAN/CANalyst-II" boxes (`04d8:0053`).
//!
//! These are not serial devices. Each of the two CAN channels has a
//! command endpoint pair, which sets the bit rate and starts the channel,
//! and a message endpoint pair, which carries CAN frames in 64-byte
//! packets (see [`crate::engine::codec::canalyst`]). This module does the
//! commands at open and returns the message endpoints as a byte stream.
//! The protocol is the one
//! [python-canalystii](https://github.com/projectgus/python-canalystii)
//! documents.

use std::io::{self, Read, Write};
use std::time::Duration;

use nusb::transfer::{Buffer, Bulk, In, Out};
use nusb::{Endpoint, Interface, MaybeFuture};

use super::usb::{Selector, transfer_error};
use crate::engine::codec::canalyst::PACKET_LEN;

/// The ids every CANalyst-II seen so far has: Microchip's vendor id (the
/// adapter is a PIC32) and product `0x0053`.
pub const CANALYST_IDS: (u16, u16) = (0x04D8, 0x0053);

const READ_TIMEOUT: Duration = Duration::from_millis(250);
const COMMAND_TIMEOUT: Duration = Duration::from_millis(500);
const WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// Bulk IN transfers kept queued on the message endpoint, one packet
/// each: every packet is a full 64 bytes, so a larger transfer would wait
/// for more packets before completing.
const READ_TRANSFERS: usize = 16;

/// Command opcodes, in the first word of a 64-byte command packet.
const COMMAND_INIT: u32 = 1;
const COMMAND_START: u32 = 2;
const COMMAND_STOP: u32 = 3;
const COMMAND_CLEAR_RX_BUFFER: u32 = 5;

/// SJA1000-style bus timing (BTR0, BTR1) for the bit rates the adapter's
/// own software offers.
const TIMINGS: [(u32, u8, u8); 13] = [
    (10_000, 0x31, 0x1C),
    (20_000, 0x18, 0x1C),
    (40_000, 0x87, 0xFF),
    (50_000, 0x09, 0x1C),
    (80_000, 0x83, 0xFF),
    (100_000, 0x04, 0x1C),
    (125_000, 0x03, 0x1C),
    (200_000, 0x81, 0xFA),
    (250_000, 0x01, 0x1C),
    (400_000, 0x80, 0xFA),
    (500_000, 0x00, 0x1C),
    (800_000, 0x00, 0x16),
    (1_000_000, 0x00, 0x14),
];

/// A command packet: `words` little-endian, zero padded to 64 bytes.
fn command(words: &[u32]) -> Vec<u8> {
    let mut packet = vec![0u8; 64];
    for (slot, w) in packet.as_chunks_mut::<4>().0.iter_mut().zip(words) {
        slot.copy_from_slice(&w.to_le_bytes());
    }
    packet
}

/// The INIT command for `bitrate`: acceptance code 1, mask all ones,
/// single filter, normal mode — what the vendor software sends.
fn init_command(bitrate: u32) -> io::Result<Vec<u8>> {
    let &(_, btr0, btr1) = TIMINGS
        .iter()
        .find(|(rate, _, _)| *rate == bitrate)
        .ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "a CANalyst-II does not do {bitrate} bit/s; choose one of {}",
                    TIMINGS
                        .iter()
                        .map(|(rate, _, _)| rate.to_string())
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
            )
        })?;
    Ok(command(&[
        COMMAND_INIT,
        1,
        0xFFFF_FFFF,
        0,
        1,
        0,
        u32::from(btr0),
        u32::from(btr1),
        0,
        1,
    ]))
}

/// Open CAN `channel` (0 or 1) of the CANalyst-II `selector` names at
/// `bitrate` and return an independent `(reader, writer)` pair carrying
/// its 64-byte message packets. A plain `usb` takes the one adapter with
/// [`CANALYST_IDS`].
pub fn open_rw(
    selector: &Selector,
    channel: u8,
    bitrate: u32,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    if channel > 1 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("a CANalyst-II has channels 0 and 1, not {channel}"),
        ));
    }
    let init = init_command(bitrate)?;
    let (vid, pid) = selector.ids.unwrap_or(CANALYST_IDS);
    let mut matched: Vec<nusb::DeviceInfo> = nusb::list_devices()
        .wait()
        .map_err(io::Error::other)?
        .filter(|d| d.vendor_id() == vid && d.product_id() == pid)
        .filter(|d| {
            selector
                .serial
                .as_deref()
                .is_none_or(|want| d.serial_number() == Some(want))
        })
        .collect();
    let info = match matched.len() {
        1 => matched.swap_remove(0),
        0 => {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("no CANalyst-II ({vid:04x}:{pid:04x}) is attached"),
            ));
        }
        n => {
            // The adapters carry no serial number, so nothing tells two apart.
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("{n} CANalyst-II adapters are attached; only one is supported"),
            ));
        }
    };
    let device = info.open().wait().map_err(io::Error::other)?;
    if device
        .active_configuration()
        .map(|c| c.configuration_value())
        != Ok(1)
    {
        device
            .set_configuration(1)
            .wait()
            .map_err(|e| io::Error::other(format!("selecting USB configuration 1: {e}")))?;
    }
    let interface = device.claim_interface(0).wait().map_err(|e| {
        io::Error::other(format!(
            "claiming the USB interface: {e} (is another program using it?)"
        ))
    })?;

    // Channel 0 talks on endpoints 1 (messages) and 2 (commands), channel
    // 1 on 3 and 4.
    let message_ep = 1 + 2 * channel;
    let command_ep = message_ep + 1;
    let mut commands = interface
        .endpoint::<Bulk, Out>(command_ep)
        .map_err(io::Error::other)?;
    let mut send = |packet: Vec<u8>| -> io::Result<()> {
        commands
            .transfer_blocking(Buffer::from(packet), COMMAND_TIMEOUT)
            .status
            .map_err(transfer_error)
    };
    send(init)?;
    send(command(&[COMMAND_CLEAR_RX_BUFFER]))?;
    send(command(&[COMMAND_START]))?;

    let reader = PacketReader::new(&interface, 0x80 | message_ep)?;
    let writer = PacketWriter {
        endpoint: interface
            .endpoint::<Bulk, Out>(message_ep)
            .map_err(io::Error::other)?,
        commands,
        _interface: interface,
    };
    log::info!(
        "{selector}: opened {} channel {channel} at {bitrate} bit/s",
        info.product_string().unwrap_or("CANalyst-II")
    );
    Ok((Box::new(reader), Box::new(writer)))
}

struct PacketReader {
    endpoint: Endpoint<Bulk, In>,
    /// Bytes received but not yet handed out, from `pos`.
    pending: Vec<u8>,
    pos: usize,
    _interface: Interface,
}

impl PacketReader {
    fn new(interface: &Interface, address: u8) -> io::Result<Self> {
        let mut endpoint = interface
            .endpoint::<Bulk, In>(address)
            .map_err(io::Error::other)?;
        for _ in 0..READ_TRANSFERS {
            let buf = endpoint.allocate(PACKET_LEN);
            endpoint.submit(buf);
        }
        Ok(Self {
            endpoint,
            pending: Vec::with_capacity(PACKET_LEN),
            pos: 0,
            _interface: interface.clone(),
        })
    }
}

impl Read for PacketReader {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        while self.pos == self.pending.len() {
            let Some(done) = self.endpoint.wait_next_complete(READ_TIMEOUT) else {
                return Err(io::ErrorKind::TimedOut.into());
            };
            let mut buf: Buffer = done.buffer;
            let status = done.status;
            self.pending.clear();
            self.pos = 0;
            if status.is_ok() {
                self.pending.extend_from_slice(&buf);
            }
            buf.clear();
            buf.set_requested_len(PACKET_LEN);
            self.endpoint.submit(buf);
            status.map_err(transfer_error)?;
        }
        let n = out.len().min(self.pending.len() - self.pos);
        out[..n].copy_from_slice(&self.pending[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

struct PacketWriter {
    endpoint: Endpoint<Bulk, Out>,
    /// The channel's command endpoint, to stop it when the writer goes.
    commands: Endpoint<Bulk, Out>,
    _interface: Interface,
}

impl Write for PacketWriter {
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

impl Drop for PacketWriter {
    /// Take the channel off the bus. Best effort: the adapter may be gone.
    fn drop(&mut self) {
        let _ = self
            .commands
            .transfer_blocking(Buffer::from(command(&[COMMAND_STOP])), COMMAND_TIMEOUT);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_for_nmea_2000_matches_python_canalystii() {
        let packet = init_command(250_000).unwrap();
        assert_eq!(packet.len(), 64);
        let words: Vec<u32> = packet
            .as_chunks::<4>()
            .0
            .iter()
            .map(|w| u32::from_le_bytes(*w))
            .collect();
        assert_eq!(
            &words[..10],
            &[1, 1, 0xFFFF_FFFF, 0, 1, 0, 0x01, 0x1C, 0, 1]
        );
        assert!(words[10..].iter().all(|&w| w == 0));
    }

    #[test]
    fn an_unknown_bitrate_lists_the_choices() {
        let err = init_command(300_000).unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::InvalidInput);
        assert!(err.to_string().contains("250000"), "{err}");
    }
}
