// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Single CAN frames to and from the gateways that pass them, with nothing
//! of NMEA 2000 in between: every frame received comes out on its own, with
//! its whole identifier, and every frame sent goes out as it is. Nothing is
//! joined, split, filtered or rewritten, and nothing is sent but what the
//! caller sends. For a bus that is CAN but not NMEA 2000 or J1939 (Mastervolt
//! MasterBus, say), or an application that does its own framing.
//!
//! Each gateway is one [`RawCodec`]: bytes in, [`CanFrame`]s out, and back,
//! with no port, thread or clock of its own, as for the
//! [`Codec`](super::Codec)s. The gateways, and what they can carry:
//!
//! | Codec | Gateway | Receives | Sends |
//! |---|---|---|---|
//! | [`Bst95`] | Actisense BST-95 "CAN Actisense": W2K-1, PRO-NDC-1E2K, NGX | 29-bit | 29-bit |
//! | [`Canalyst`] | CANalyst-II, Waveshare USB-CAN-B | both | both |
//! | [`UsbCanA`] | Waveshare USB-CAN-A | both | both |
//! | [`Ydwg`] | Yacht Devices RAW: YDWG-02, YDEN-02, YDNR-02, YDNU-02 | 29-bit | no |
//! | [`Mxpgn`] | ShipModul MiniPlex `$MXPGN`, and Actisense's | 29-bit | no |
//!
//! The Yacht Devices gateways and the MiniPlex put their own source address
//! in the low byte of the identifier of every frame they transmit, so they
//! cannot send a frame as it is: their codecs only receive, and refuse to
//! send with [`Refused::ReceiveOnly`]. The MiniPlex, moreover, passes on
//! frames that look like an NMEA 2000 Address Claim (PGN 60928) or Product
//! Information (126996) only as its own `$PSMD` sentences, which carry no
//! CAN frame, so those identifiers never reach its codec.
//!
//! A BST-95 gateway echoes every frame it transmits, marked as going to the
//! bus; [`Bst95`] skips the echoes. The other gateways do not echo.

use std::time::Duration;

use smallvec::SmallVec;

use crate::engine::format::bst::{BdtpDecoder, bst95_can_message, encode_bst95_can};
use crate::engine::format::iso11783_compose;
use crate::engine::format::mxpgn::{self, DataOrder, MINIPLEX_MXPGN_ALL};
use crate::engine::format::ydwg02::YDNU_RAW_MODE;

use super::Refused;
use super::canalyst::{Message, PACKET_LEN, Packets, encode_packets, messages};
use super::usb_can_a::{self, Frames, bitrates, config_packet, encode_frame};

pub use super::raw_can::CanFrame;

/// What a [`RawCodec`] hands back as bytes arrive.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// A frame received from the bus.
    Frame(CanFrame),
    /// A framing or protocol error. The codec carries on.
    Error(String),
}

/// What a gateway can carry through its [`RawCodec`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct Capabilities {
    /// Whether it transmits frames as they are given. When not,
    /// [`RawCodec::send`] refuses every frame with [`Refused::ReceiveOnly`].
    pub sends: bool,
    /// Whether it carries 11-bit standard frames as well as 29-bit ones.
    pub standard_frames: bool,
}

/// A gateway's raw CAN framing, driven by its caller. See the [module
/// docs](self).
pub trait RawCodec {
    /// What the gateway can carry.
    fn capabilities(&self) -> Capabilities;

    /// Bytes to write as soon as the link is up: a USB-CAN-A's
    /// configuration, a YDNU-02's switch to RAW mode. Empty when the gateway
    /// needs none.
    fn open(&mut self) -> Vec<u8> {
        Vec::new()
    }

    /// Feed bytes read from the gateway; the frames and errors they complete
    /// are pushed onto `events`.
    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>);

    /// Encode `frame` for the gateway.
    fn send(&mut self, frame: &CanFrame) -> Result<Vec<u8>, Refused>;

    /// The least time from one frame written to the next, for a gateway
    /// that loses frames written faster than it sends them. Zero for most.
    fn write_gap(&self) -> Duration {
        Duration::ZERO
    }

    /// Bytes to write when the application closes the link on purpose.
    /// Empty when the gateway needs nothing.
    fn close(&mut self) -> Vec<u8> {
        Vec::new()
    }
}

/// A boxed codec, for a gateway chosen at run time.
impl<C: RawCodec + ?Sized> RawCodec for Box<C> {
    fn capabilities(&self) -> Capabilities {
        (**self).capabilities()
    }

    fn open(&mut self) -> Vec<u8> {
        (**self).open()
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        (**self).receive(bytes, events)
    }

    fn send(&mut self, frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        (**self).send(frame)
    }

    fn write_gap(&self) -> Duration {
        (**self).write_gap()
    }

    fn close(&mut self) -> Vec<u8> {
        (**self).close()
    }
}

/// Refuse what is no CAN frame, and (unless `standard`) an 11-bit frame.
fn check(frame: &CanFrame, standard: bool) -> Result<(), Refused> {
    if frame.data.len() > 8 {
        return Err(Refused::TooLarge);
    }
    if !frame.is_valid() || (!frame.extended && !standard) {
        return Err(Refused::Unsupported);
    }
    Ok(())
}

/// Actisense BST-95 raw CAN frames: a PRO-NDC-1E2K or W2K-1 in "CAN
/// Actisense" mode (TCP), an NGX in CAN Packet mode (serial). 29-bit frames
/// only. The gateway echoes the frames it transmits; those are skipped.
#[derive(Debug, Default)]
pub struct Bst95 {
    bdtp: BdtpDecoder,
}

impl Bst95 {
    /// A codec with nothing received yet.
    pub fn new() -> Self {
        Self::default()
    }
}

impl RawCodec for Bst95 {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            sends: true,
            standard_frames: false,
        }
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        let dropped = self.bdtp.dropped;
        for &b in bytes {
            let Some(m) = self.bdtp.push_byte(b) else {
                continue;
            };
            // Another BST message (a BEM reply, say) is not a CAN frame, and
            // a frame on its way to the bus is the echo of one sent.
            if let Some((id, data, false)) = bst95_can_message(&m) {
                events.push(Event::Frame(CanFrame {
                    id,
                    extended: true,
                    data: SmallVec::from_slice(data),
                }));
            }
        }
        let dropped = self.bdtp.dropped - dropped;
        if dropped > 0 {
            events.push(Event::Error(format!(
                "dropped {dropped} damaged BDTP frame(s)"
            )));
        }
    }

    fn send(&mut self, frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        check(frame, false)?;
        let mut out = Vec::new();
        encode_bst95_can(frame.id, &frame.data, true, &mut out);
        Ok(out)
    }
}

/// CANalyst-II raw CAN frames in 64-byte USB packets: the Waveshare
/// USB-CAN-B (`04d8:0053`). The caller opens the channel, which sets the
/// bit rate. Remote frames are skipped.
#[derive(Debug, Default)]
pub struct Canalyst {
    packets: Packets,
}

impl Canalyst {
    /// A codec with nothing received yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// Bytes in one USB packet.
    pub const PACKET_LEN: usize = PACKET_LEN;
}

impl RawCodec for Canalyst {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            sends: true,
            standard_frames: true,
        }
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        self.packets.push(bytes, |packet| match messages(packet) {
            Ok(ms) => events.extend(ms.iter().map(|m| {
                Event::Frame(CanFrame {
                    id: m.id,
                    extended: m.extended,
                    data: SmallVec::from_slice(m.data),
                })
            })),
            Err(count) => events.push(Event::Error(format!(
                "CANalyst packet claims {count} messages"
            ))),
        });
    }

    fn send(&mut self, frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        check(frame, true)?;
        let mut out = Vec::new();
        encode_packets(
            &[Message {
                id: frame.id,
                extended: frame.extended,
                data: &frame.data,
            }],
            &mut out,
        );
        Ok(out)
    }
}

/// Waveshare USB-CAN-A raw CAN frames in 20-byte packets, over the
/// adapter's serial port (`1a86:7523`, at
/// [`BAUD`](crate::engine::codec::usb_can_a::BAUD)). [`RawCodec::open`]
/// gives the configuration packet, which sets the bit rate; frames written
/// faster than the adapter sends them are lost, so
/// [`RawCodec::write_gap`] spaces them. Remote frames are skipped.
#[derive(Debug)]
pub struct UsbCanA {
    config: [u8; usb_can_a::FRAME_LEN],
    bitrate: u32,
    frames: Frames,
}

impl UsbCanA {
    /// A codec for a bus at `bitrate`, or `None` for a bit rate the adapter
    /// does not have (see [`UsbCanA::bitrates`]).
    pub fn new(bitrate: u32) -> Option<Self> {
        Some(Self {
            // The frame type it is set to only says what it sends by
            // default: it passes on and sends both kinds.
            config: config_packet(bitrate, crate::engine::BusProtocol::Nmea2000)?,
            bitrate,
            frames: Frames::default(),
        })
    }

    /// The CAN bit rates the adapter runs at.
    pub fn bitrates() -> impl Iterator<Item = u32> {
        bitrates()
    }
}

impl RawCodec for UsbCanA {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            sends: true,
            standard_frames: true,
        }
    }

    /// The configuration packet.
    fn open(&mut self) -> Vec<u8> {
        self.config.to_vec()
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        let skipped = self.frames.skipped;
        self.frames.push(bytes, |f| events.push(Event::Frame(f)));
        if self.frames.skipped != skipped {
            events.push(Event::Error(format!(
                "skipped {} bytes to find a USB-CAN-A frame",
                self.frames.skipped - skipped
            )));
        }
    }

    fn send(&mut self, frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        check(frame, true)?;
        let mut out = Vec::new();
        encode_frame(frame, &mut out);
        Ok(out)
    }

    fn write_gap(&self) -> Duration {
        usb_can_a::write_gap(self.bitrate)
    }
}

/// The longest line kept: far more than any frame's line, so a longer one
/// is a stream that is no gateway's, or lost its line ends.
const MAX_LINE: usize = 4096;

/// Text lines out of a byte stream, `\n`-terminated, trimmed.
#[derive(Debug, Default)]
struct Lines {
    acc: Vec<u8>,
    /// Skipping a line that grew past [`MAX_LINE`], up to its end.
    dropping: bool,
}

impl Lines {
    /// Call `each` for every non-empty line `bytes` completes. Returns how
    /// many lines grew past [`MAX_LINE`] and were dropped, so that a stream
    /// without line ends cannot fill the memory.
    fn push(&mut self, mut bytes: &[u8], mut each: impl FnMut(&str)) -> usize {
        let mut dropped = 0;
        loop {
            let end = bytes.iter().position(|&b| b == b'\n');
            let part = &bytes[..end.unwrap_or(bytes.len())];
            if !self.dropping {
                if self.acc.len() + part.len() > MAX_LINE {
                    self.acc.clear();
                    self.dropping = true;
                    dropped += 1;
                } else {
                    self.acc.extend_from_slice(part);
                }
            }
            let Some(end) = end else {
                return dropped;
            };
            if !self.dropping {
                let line = String::from_utf8_lossy(&self.acc);
                let line = line.trim();
                if !line.is_empty() {
                    each(line);
                }
            }
            self.acc.clear();
            self.dropping = false;
            bytes = &bytes[end + 1..];
        }
    }

    /// The error for `dropped` lines that were too long, if any.
    fn overflow(dropped: usize) -> Option<Event> {
        (dropped > 0).then(|| {
            Event::Error(format!(
                "dropped {dropped} line(s) longer than {MAX_LINE} bytes"
            ))
        })
    }
}

/// Yacht Devices RAW lines: a YDWG-02 or YDEN-02 over TCP or UDP, a YDNU-02
/// over its serial port ([`Ydwg::ydnu`]). Receive only: the gateway
/// transmits a frame with its own source address in the identifier's low
/// byte. A frame another client had it transmit (`T`) is bus traffic like
/// any other, and is passed on.
#[derive(Debug, Default)]
pub struct Ydwg {
    lines: Lines,
    init: &'static [u8],
}

impl Ydwg {
    /// A codec for a YDWG-02 or YDEN-02, which need nothing to start.
    pub fn new() -> Self {
        Self::default()
    }

    /// A codec for a YDNU-02, which [`open`](RawCodec::open) puts in RAW
    /// mode.
    pub fn ydnu() -> Self {
        Self {
            lines: Lines::default(),
            init: YDNU_RAW_MODE,
        }
    }
}

/// `<time> <R|T> <CANID> <data bytes…>` → the frame.
fn ydwg_line(line: &str) -> Result<CanFrame, String> {
    let mut toks = line.split_whitespace().skip(1);
    if !matches!(toks.next(), Some("R" | "T")) {
        return Err("no direction".into());
    }
    let id = toks
        .next()
        .and_then(|t| u32::from_str_radix(t, 16).ok())
        .ok_or("no CAN identifier")?;
    let data = toks
        .map(|t| u8::from_str_radix(t, 16).map_err(|_| format!("bad data byte {t:?}")))
        .collect::<Result<SmallVec<[u8; 8]>, _>>()?;
    let frame = CanFrame {
        id,
        extended: true,
        data,
    };
    // More than 8 bytes means two lines glued together (a lost datagram).
    if frame.is_valid() {
        Ok(frame)
    } else {
        Err("not a CAN frame".into())
    }
}

impl RawCodec for Ydwg {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            sends: false,
            standard_frames: false,
        }
    }

    fn open(&mut self) -> Vec<u8> {
        self.init.to_vec()
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        let dropped = self.lines.push(bytes, |line| {
            events.push(match ydwg_line(line) {
                Ok(f) => Event::Frame(f),
                Err(e) => Event::Error(format!("{line}: {e}")),
            })
        });
        events.extend(Lines::overflow(dropped));
    }

    fn send(&mut self, _frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        Err(Refused::ReceiveOnly)
    }
}

/// SocketCAN `candump` text over a socket: a SensESP N2K gateway, a
/// canboatjs N2K IP gateway in candump mode. Receives any candump shape
/// (log, pretty, Angstrom), 29-bit and 11-bit frames alike; sends the log
/// shape, `(<epoch.micros>) can0 <canid>#<hexbytes>`, frames as they are
/// given.
#[derive(Debug)]
pub struct Candump {
    lines: Lines,
    /// The time to give what is sent, in Unix milliseconds.
    clock: fn() -> u64,
}

impl Default for Candump {
    fn default() -> Self {
        Self::new()
    }
}

impl Candump {
    /// A codec that sends every line with time 0: a codec has no clock of
    /// its own ([`Candump::with_clock`] gives it one).
    pub fn new() -> Self {
        Self {
            lines: Lines::default(),
            clock: || 0,
        }
    }

    /// A codec that gives what it sends the time `clock` says, in Unix
    /// milliseconds.
    pub fn with_clock(clock: fn() -> u64) -> Self {
        Self {
            lines: Lines::default(),
            clock,
        }
    }
}

impl RawCodec for Candump {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            sends: true,
            standard_frames: true,
        }
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        let dropped = self.lines.push(
            bytes,
            |line| match crate::engine::format::candump::parse_can(line) {
                Ok(Some(frame)) => events.push(Event::Frame(frame)),
                Ok(None) => {}
                Err(e) => events.push(Event::Error(format!("{line}: {e}"))),
            },
        );
        events.extend(Lines::overflow(dropped));
    }

    fn send(&mut self, frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        check(frame, true)?;
        let mut out = String::with_capacity(48);
        crate::engine::format::candump::write_log(
            &mut out,
            frame.id,
            frame.extended,
            &frame.data,
            (self.clock)(),
        );
        Ok(out.into_bytes())
    }
}

/// `$MXPGN` sentences: a ShipModul MiniPlex ([`Mxpgn::miniplex`]), or an
/// Actisense gateway sending them ([`Mxpgn::actisense`]), which orders the
/// data the other way round. Receive only: the gateway transmits a frame
/// with its own source address. The NMEA 0183 sentences mixed in are
/// skipped.
#[derive(Debug)]
pub struct Mxpgn {
    lines: Lines,
    order: DataOrder,
    init: &'static [u8],
}

impl Mxpgn {
    /// A codec for a MiniPlex, which [`open`](RawCodec::open) sets to send
    /// every frame it receives as `$MXPGN`.
    pub fn miniplex() -> Self {
        Self {
            lines: Lines::default(),
            order: DataOrder::Reversed,
            init: MINIPLEX_MXPGN_ALL,
        }
    }

    /// A codec for an Actisense gateway in `$MXPGN` mode.
    pub fn actisense() -> Self {
        Self {
            lines: Lines::default(),
            order: DataOrder::Bus,
            init: b"",
        }
    }
}

impl RawCodec for Mxpgn {
    fn capabilities(&self) -> Capabilities {
        Capabilities {
            sends: false,
            standard_frames: false,
        }
    }

    fn open(&mut self) -> Vec<u8> {
        self.init.to_vec()
    }

    fn receive(&mut self, bytes: &[u8], events: &mut Vec<Event>) {
        let order = self.order;
        let dropped = self.lines.push(bytes, |line| {
            match mxpgn::parse_ordered(line, true, order) {
                // The sentence names priority, PGN (with the destination of a
                // PDU1 one) and source: the whole 29-bit identifier.
                Ok(Some(f)) => events.push(Event::Frame(CanFrame {
                    id: iso11783_compose(f.prio, f.pgn, f.src, f.dst),
                    extended: true,
                    data: f.data,
                })),
                Ok(None) => {}
                Err(e) => events.push(Event::Error(format!("{line}: {e}"))),
            }
        });
        events.extend(Lines::overflow(dropped));
    }

    fn send(&mut self, _frame: &CanFrame) -> Result<Vec<u8>, Refused> {
        Err(Refused::ReceiveOnly)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::format::bst::encode_bst95_can as bst95;

    /// A MasterBus identifier: CAN class in bits 24–28, a 24-bit device
    /// address below. Read as NMEA 2000 it is priority 7, both data pages
    /// set and PDU1.
    const MASTERBUS_ID: u32 = 0x1F_1F_02_A7;

    fn frames(events: Vec<Event>) -> Vec<CanFrame> {
        events
            .into_iter()
            .map(|e| match e {
                Event::Frame(f) => f,
                Event::Error(e) => panic!("{e}"),
            })
            .collect()
    }

    fn receive(codec: &mut impl RawCodec, bytes: &[u8]) -> Vec<CanFrame> {
        let mut events = Vec::new();
        codec.receive(bytes, &mut events);
        frames(events)
    }

    fn ext(id: u32, data: &[u8]) -> CanFrame {
        CanFrame::extended(id, data).unwrap()
    }

    #[test]
    fn frames_are_checked_as_they_are_made() {
        assert!(CanFrame::extended(0x1fff_ffff, &[0; 8]).is_some());
        assert!(CanFrame::extended(0x2000_0000, &[]).is_none());
        assert!(CanFrame::extended(1, &[0; 9]).is_none());
        assert!(CanFrame::standard(0x7ff, &[]).is_some());
        assert!(CanFrame::standard(0x800, &[]).is_none());
    }

    #[test]
    fn bst95_passes_frames_unchanged_and_skips_its_echoes() {
        let mut codec = Bst95::new();
        let sent = ext(MASTERBUS_ID, &[1, 2, 3]);
        let wire = codec.send(&sent).unwrap();
        assert!(receive(&mut codec, &wire).is_empty(), "the echo");

        // The same frame from the bus: one frame, never joined.
        let mut from_bus = Vec::new();
        bst95(MASTERBUS_ID, &[1, 2, 3], false, &mut from_bus);
        bst95(MASTERBUS_ID, &[4], false, &mut from_bus);
        assert_eq!(
            receive(&mut codec, &from_bus),
            [sent, ext(MASTERBUS_ID, &[4])]
        );
    }

    #[test]
    fn candump_frames_go_both_ways() {
        let mut codec = Candump::with_clock(|| 1_502_979_132_106);
        let mut events = Vec::new();
        codec.receive(
            b"(1502979132.106111) can0 09F50374#000A00FF\n  can0  123   [2]  01 02\n",
            &mut events,
        );
        assert_eq!(
            events,
            [
                Event::Frame(ext(0x09F5_0374, &[0x00, 0x0A, 0x00, 0xFF])),
                Event::Frame(CanFrame::standard(0x123, &[1, 2]).unwrap()),
            ]
        );
        assert_eq!(
            codec.send(&ext(0x09F5_0374, &[0xAB])).unwrap(),
            b"(1502979132.106000) can0 09F50374#AB\n"
        );
        assert_eq!(
            codec
                .send(&CanFrame::standard(0x7FF, &[]).unwrap())
                .unwrap(),
            b"(1502979132.106000) can0 7FF#\n"
        );
    }

    #[test]
    fn what_a_codec_refuses_matches_its_capabilities() {
        let standard = CanFrame::standard(0x123, &[1]).unwrap();
        let extended = ext(MASTERBUS_ID, &[1]);
        let codecs: [Box<dyn RawCodec>; 7] = [
            Box::new(Candump::new()),
            Box::new(Bst95::new()),
            Box::new(Canalyst::new()),
            Box::new(UsbCanA::new(250_000).unwrap()),
            Box::new(Ydwg::new()),
            Box::new(Mxpgn::miniplex()),
            Box::new(Mxpgn::actisense()),
        ];
        for mut codec in codecs {
            let caps = codec.capabilities();
            assert_eq!(codec.send(&extended).is_ok(), caps.sends, "{caps:?}");
            assert_eq!(
                codec.send(&standard).is_ok(),
                caps.sends && caps.standard_frames,
                "{caps:?}"
            );
        }
    }

    #[test]
    fn bst95_refuses_standard_frames() {
        let frame = CanFrame::standard(0x123, &[1]).unwrap();
        assert_eq!(Bst95::new().send(&frame), Err(Refused::Unsupported));
    }

    #[test]
    fn canalyst_round_trips_both_kinds() {
        let mut codec = Canalyst::new();
        let ours = [
            ext(MASTERBUS_ID, &[0; 8]),
            CanFrame::standard(0x7e6, &[9]).unwrap(),
        ];
        for f in &ours {
            let wire = codec.send(f).unwrap();
            assert_eq!(wire.len(), Canalyst::PACKET_LEN);
            assert_eq!(receive(&mut codec, &wire), std::slice::from_ref(f));
        }
    }

    #[test]
    fn usb_can_a_round_trips_and_spaces_its_writes() {
        assert!(UsbCanA::new(123_456).is_none());
        let mut codec = UsbCanA::new(250_000).unwrap();
        assert_eq!(codec.open().len(), usb_can_a::FRAME_LEN);
        assert_eq!(codec.write_gap(), Duration::from_micros(700));
        let frame = ext(MASTERBUS_ID, &[1, 2, 3, 4, 5, 6, 7, 8]);
        let wire = codec.send(&frame).unwrap();
        assert_eq!(receive(&mut codec, &wire), [frame]);
    }

    #[test]
    fn ydwg_reads_both_directions_and_sends_nothing() {
        let mut codec = Ydwg::new();
        assert!(codec.open().is_empty());
        let got = receive(
            &mut codec,
            b"00:17:55.475 R 1F1F02A7 01 02\r\n00:17:55.476 T 0DF50B23 FF\r\n\r\n00:17",
        );
        assert_eq!(got, [ext(MASTERBUS_ID, &[1, 2]), ext(0x0DF50B23, &[0xff])]);
        assert_eq!(
            receive(&mut codec, b":55.477 R 1F1F02A7\n"),
            [ext(MASTERBUS_ID, &[])]
        );
        assert_eq!(
            codec.send(&ext(MASTERBUS_ID, &[])),
            Err(Refused::ReceiveOnly)
        );
        assert_eq!(Ydwg::ydnu().open(), b"0\n");
    }

    #[test]
    fn ydwg_reports_a_line_that_is_no_frame() {
        let mut events = Vec::new();
        Ydwg::new().receive(
            b"00:17:55.475 R 1F1F02A7 01 02 03 04 05 06 07 08 09\n",
            &mut events,
        );
        assert!(matches!(&events[..], [Event::Error(_)]), "{events:?}");
    }

    /// A stream without line ends does not grow without bound: the long
    /// line is dropped up to its end, and the next one read.
    #[test]
    fn a_line_too_long_is_dropped_up_to_its_end() {
        let mut codec = Ydwg::new();
        let mut events = Vec::new();
        for _ in 0..10 {
            codec.receive(&[b'A'; 1000], &mut events);
        }
        assert!(codec.lines.acc.len() <= MAX_LINE);
        codec.receive(b"AAA\n00:17:55.475 R 1F1F02A7 01\n", &mut events);
        let [Event::Error(e), Event::Frame(f)] = &events[..] else {
            panic!("{events:?}");
        };
        assert!(e.contains("longer than 4096"), "{e}");
        assert_eq!(f, &ext(MASTERBUS_ID, &[1]));

        // A line of exactly the most is kept.
        let mut lines = Lines::default();
        let mut got = 0;
        let long = [b'x'; MAX_LINE];
        assert_eq!(lines.push(&long, |_| got += 1), 0);
        assert_eq!(lines.push(b"\n", |l| got += l.len()), 0);
        assert_eq!(got, MAX_LINE);
    }

    #[test]
    fn mxpgn_gives_back_the_whole_identifier() {
        let mut codec = Mxpgn::miniplex();
        assert_eq!(codec.open(), MINIPLEX_MXPGN_ALL);
        // The identifier's top bits are priority 7 and both data pages set,
        // so PGN 0x31F02 holds the PDU1 destination 0x02; the attribute
        // word priority 7, 3 bytes, source 0xA7; the data last byte first.
        let mut line = String::new();
        let raw = crate::engine::RawFrame::new(None, 7, 0x31F00, 0xA7, 0x02, [1, 2, 3]);
        mxpgn::write_ordered(&mut line, &raw, false, DataOrder::Reversed).unwrap();
        assert!(line.starts_with("$MXPGN,031F02,73A7,030201*"), "{line}");
        line.push_str("\r\n$GPGLL,5300.97,N,00517.86,E*7F\r\n");
        assert_eq!(
            receive(&mut codec, line.as_bytes()),
            [ext(MASTERBUS_ID, &[1, 2, 3])]
        );
        assert_eq!(
            codec.send(&ext(MASTERBUS_ID, &[])),
            Err(Refused::ReceiveOnly)
        );
        assert!(Mxpgn::actisense().open().is_empty());
    }
}
