// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Line-based network gateway codecs: Yacht Devices RAW (YDWG-02 /
//! YDEN, also the NavLink2's RAW port), Actisense N2K ASCII (W2K-1),
//! ShipModul `$MXPGN` (MiniPlex) and SeaSmart `$PCDIN`.
//!
//! They speak newline-delimited text over a TCP or UDP socket, or a
//! serial port. The decoder half accumulates a partial line
//! across reads, parses each complete line with the matching
//! [`crate::engine::format`] parser and — for RAW, whose lines are
//! individual CAN frames — reassembles fast-packets before emitting.
//! The encoder half writes the device's transmit shape: complete
//! `A…` lines for N2K ASCII, bare `<CANID> <bytes…>` lines for RAW
//! with fast-packet payloads fragmented per ISO 11783-3.

use std::collections::{HashMap, VecDeque};
use std::fmt::Write as _;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::engine::format::{actisense_ascii, iso11783_compose, mxpgn, seasmart, ydwg02};
use crate::engine::frame::FASTPACKET_MAX_SIZE;
use crate::engine::reassembly::{Reassembled, Reassembler};
use crate::engine::startup::format_iso_ms;
use crate::engine::{BusProtocol, FramePacketType, RawFrame};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};
use crate::engine::fastpacket;

/// Which line format the socket speaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineFormat {
    /// Yacht Devices RAW: `HH:MM:SS.mmm R <CANID> <bytes…>` per CAN
    /// frame received; transmit lines are `<CANID> <bytes…>`.
    YdwgRaw,
    /// Actisense N2K ASCII: `A…` lines carrying complete (coalesced)
    /// messages in both directions.
    N2kAscii,
    /// ShipModul MiniPlex `$MXPGN` sentences, one CAN frame each, among
    /// other NMEA 0183 sentences, which carry no frame. Sent in the send
    /// form: the MiniPlex sends from its own address.
    Mxpgn,
    /// SeaSmart `$PCDIN` sentences, one whole message each, among other
    /// NMEA 0183 sentences, which carry no message. Sent the same way,
    /// with no priority or destination: the sentence has neither.
    SeaSmart,
}

/// The CAN frames written to a RAW gateway, to know their `T` echoes
/// from another client's transmits: the gateway echoes every transmit on
/// every connection, with its own source address in place of ours.
#[derive(Clone, Default)]
pub struct Sent(Arc<Mutex<VecDeque<SentFrame>>>);

/// A CAN id without its source address, the data, and when it was sent.
type SentFrame = (u32, Vec<u8>, Instant);

/// How long an echo of a frame written is waited for.
const ECHO_WAIT: Duration = Duration::from_secs(2);

impl Sent {
    /// The CAN id without its source address.
    fn key(canid: u32) -> u32 {
        canid & !0xff
    }

    fn note(&self, canid: u32, data: &[u8]) {
        let mut sent = self.0.lock().expect("sent mutex");
        let now = Instant::now();
        while sent.front().is_some_and(|(_, _, at)| now - *at > ECHO_WAIT) {
            sent.pop_front();
        }
        sent.push_back((Self::key(canid), data.to_vec(), now));
    }

    /// Whether `canid` and `data` echo a frame written, which is then
    /// forgotten.
    fn is_echo(&self, canid: u32, data: &[u8]) -> bool {
        let mut sent = self.0.lock().expect("sent mutex");
        let now = Instant::now();
        let key = Self::key(canid);
        match sent
            .iter()
            .position(|(k, d, at)| *k == key && d == data && now - *at <= ECHO_WAIT)
        {
            Some(i) => {
                sent.remove(i);
                true
            }
            None => false,
        }
    }
}

pub struct Decoder {
    format: LineFormat,
    /// Decides which RAW frames are fast-packet fragments.
    protocol: BusProtocol,
    acc: String,
    reassembler: Reassembler,
    sent: Sent,
    /// Whether a `$PCDIN` or `$MXPGN` sentence's checksum is checked.
    checksum: bool,
}

impl Decoder {
    #[cfg(test)]
    pub fn new(format: LineFormat) -> Self {
        Self::with_protocol(format, BusProtocol::Nmea2000)
    }

    pub fn with_protocol(format: LineFormat, protocol: BusProtocol) -> Self {
        Self {
            format,
            protocol,
            acc: String::with_capacity(256),
            reassembler: Reassembler::new(),
            sent: Sent::default(),
            checksum: true,
        }
    }

    /// Accept `$PCDIN` and `$MXPGN` sentences whose checksum is wrong.
    pub fn ignore_checksum(mut self, ignore: bool) -> Self {
        self.checksum = !ignore;
        self
    }

    /// Know the echoes of what `sent` records.
    pub fn with_sent(mut self, sent: Sent) -> Self {
        self.sent = sent;
        self
    }

    /// Pass on a CAN frame received at `now`, joining fast-packets.
    fn push_frame(&mut self, mut frame: RawFrame, now: &str, events: &mut Vec<DeviceEvent>) {
        frame.timestamp = Some(now.to_string());
        let pt = self.protocol.packet_type(frame.pgn);
        match self.reassembler.push(frame, pt) {
            Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
                events.push(DeviceEvent::Frame(f))
            }
            Reassembled::Partial => {}
            Reassembled::Error(e) => events.push(DeviceEvent::Error(e.to_string())),
        }
    }
}

impl DeviceDecoder for Decoder {
    /// A quiet line still gets its minute's summary of incomplete
    /// fast-packets.
    fn tick(&mut self, _events: &mut Vec<DeviceEvent>) {
        self.reassembler.tick(super::now_ms());
    }

    /// Frames get the host's time, as from every other gateway: the
    /// gateways' own is a time of day without a date, and a YDEN's runs
    /// behind the host's, and now and then backwards (canboat#1039).
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        let now_ms = super::now_ms();
        self.reassembler.tick(now_ms);
        let now = format_iso_ms(now_ms);
        self.acc.push_str(&String::from_utf8_lossy(bytes));
        while let Some(pos) = self.acc.find('\n') {
            let line: String = self.acc[..pos].trim().to_string();
            self.acc.drain(..=pos);
            if line.is_empty() {
                continue;
            }
            match self.format {
                LineFormat::YdwgRaw => {
                    // A transmit, marked `T`, is echoed on every
                    // connection: ours is skipped, as no gateway reports
                    // what canboat sent, but another client's is bus
                    // traffic like any other.
                    let transmit = line.split_whitespace().nth(1) == Some("T");
                    match ydwg02::parse_line(&line) {
                        Ok(frame)
                            if transmit
                                && self.sent.is_echo(
                                    iso11783_compose(frame.prio, frame.pgn, frame.src, frame.dst),
                                    &frame.data,
                                ) => {}
                        Ok(frame) => self.push_frame(frame, &now, events),
                        Err(e) => events.push(DeviceEvent::Error(format!("{line}: {e}"))),
                    }
                }
                LineFormat::Mxpgn => match mxpgn::parse_sentence(&line, self.checksum) {
                    Ok(Some(frame)) => self.push_frame(frame, &now, events),
                    // The NMEA 0183 a MiniPlex mixes in.
                    Ok(None) => {}
                    Err(e) => events.push(DeviceEvent::Error(format!("{line}: {e}"))),
                },
                LineFormat::SeaSmart => match seasmart::parse_sentence(&line, self.checksum) {
                    Ok(Some(mut frame)) => {
                        frame.timestamp = Some(now.clone());
                        events.push(DeviceEvent::Frame(frame))
                    }
                    // The NMEA 0183 a gateway mixes in.
                    Ok(None) => {}
                    Err(e) => events.push(DeviceEvent::Error(format!("{line}: {e}"))),
                },
                LineFormat::N2kAscii => match actisense_ascii::parse_line(&line) {
                    Ok(mut frame) => {
                        frame.timestamp = Some(now.clone());
                        events.push(DeviceEvent::Frame(frame))
                    }
                    Err(e) => events.push(DeviceEvent::Error(format!("{line}: {e}"))),
                },
            }
        }
    }
}

pub struct Encoder {
    format: LineFormat,
    /// Decides which RAW transmissions are fast-packet framed.
    protocol: BusProtocol,
    /// Per-(pgn, src) fast-packet TX sequence counters, mod 8.
    seq: Mutex<HashMap<(u32, u8), u8>>,
    /// Bytes to write as soon as the link opens.
    init: Vec<u8>,
    sent: Sent,
}

impl Encoder {
    #[cfg(test)]
    pub fn new(format: LineFormat) -> Self {
        Self::with_protocol(format, BusProtocol::Nmea2000)
    }

    pub fn with_protocol(format: LineFormat, protocol: BusProtocol) -> Self {
        Self {
            format,
            protocol,
            seq: Mutex::new(HashMap::new()),
            init: Vec::new(),
            sent: Sent::default(),
        }
    }

    /// Record in `sent` the RAW frames written.
    pub fn with_sent(mut self, sent: Sent) -> Self {
        self.sent = sent;
        self
    }

    /// Write `init` as soon as the link opens.
    pub fn with_init(mut self, init: &[u8]) -> Self {
        self.init = init.to_vec();
        self
    }

    fn next_seq(&self, pgn: u32, src: u8) -> u8 {
        let mut m = self.seq.lock().expect("seq mutex");
        let e = m.entry((pgn, src)).or_insert(0);
        let s = *e;
        *e = (*e + 1) & 0x07;
        s
    }
}

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        self.init.clone()
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        // Synthetic canboat-internal PGNs never go on the wire.
        if frame.pgn >= fastpacket::CANBOAT_PGN_START {
            return None;
        }
        let mut out = String::with_capacity(64);
        match self.format {
            LineFormat::N2kAscii => {
                actisense_ascii::write_line(&mut out, frame).ok()?;
                out.push_str("\r\n");
            }
            LineFormat::SeaSmart => {
                if frame.data.len() > FASTPACKET_MAX_SIZE {
                    log::warn!(
                        "not sending PGN {}: {} bytes exceed a fast-packet",
                        frame.pgn,
                        frame.data.len()
                    );
                    return None;
                }
                if frame.dst != 255 && (frame.pgn >> 8) & 0xff < 240 {
                    log::warn!(
                        "PGN {} for {} goes out in $PCDIN, which has no destination: \
                         the gateway decides where it goes",
                        frame.pgn,
                        frame.dst
                    );
                }
                seasmart::write_line(&mut out, frame).ok()?;
                out.push_str("\r\n");
            }
            LineFormat::YdwgRaw | LineFormat::Mxpgn => {
                let canid = iso11783_compose(frame.prio, frame.pgn, frame.src, frame.dst);
                let write_one = |out: &mut String, data: &[u8]| {
                    if self.format == LineFormat::Mxpgn {
                        let one = RawFrame::new(
                            None,
                            frame.prio,
                            frame.pgn,
                            frame.src,
                            frame.dst,
                            data.iter().copied(),
                        );
                        let _ = mxpgn::write_line(out, &one, true);
                    } else {
                        self.sent.note(canid, data);
                        let _ = write!(out, "{canid:08X}");
                        for b in data {
                            let _ = write!(out, " {b:02X}");
                        }
                    }
                    out.push_str("\r\n");
                };
                if self.protocol.packet_type(frame.pgn) == FramePacketType::Fast {
                    let seq = self.next_seq(frame.pgn, frame.src);
                    let Some(chunks) = fastpacket::fragment(seq, &frame.data) else {
                        log::warn!(
                            "not sending PGN {}: {} bytes exceed one fast-packet",
                            frame.pgn,
                            frame.data.len()
                        );
                        return None;
                    };
                    for chunk in chunks {
                        write_one(&mut out, &chunk);
                    }
                } else if frame.data.len() > 8 {
                    // One RAW line is one CAN frame. On J1939 this payload
                    // needs ISO TP, which we cannot send yet.
                    log::warn!(
                        "not sending PGN {}: {} bytes do not fit one CAN frame",
                        frame.pgn,
                        frame.data.len()
                    );
                    return None;
                } else {
                    write_one(&mut out, &frame.data);
                }
            }
        }
        Some(out.into_bytes())
    }
}

/// What a ShipModul MiniPlex is sent when the link opens: options 30 and
/// 31 set to mode 2, every PGN received sent on as `$MXPGN` (ShipModul,
/// "MiniPlex-3 NMEA and Proprietary Sentences", `$PSMDNO`). Address
/// Claims and Product Information still come as `$PSMDNC` and `$PSMDNP`.
pub const MINIPLEX_MXPGN_ALL: &[u8] = b"$PSMDNO,C,30,0*57\r\n$PSMDNO,C,31,1*57\r\n";

/// What a Yacht Devices USB gateway (YDNU-02) is sent when its serial
/// port opens, to put it in RAW mode, as canboatjs does: `0` and a line
/// feed.
pub const YDNU_RAW_MODE: &[u8] = b"0\n";

/// Start the gateway reader/writer threads over any byte transport,
/// writing `init` as soon as the link opens. `ignore_checksum` accepts
/// `$PCDIN` and `$MXPGN` sentences whose checksum is wrong.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    format: LineFormat,
    protocol: BusProtocol,
    init: &[u8],
    ignore_checksum: bool,
) -> DeviceHandle {
    let sent = Sent::default();
    super::run(
        Decoder::with_protocol(format, protocol)
            .with_sent(sent.clone())
            .ignore_checksum(ignore_checksum),
        Encoder::with_protocol(format, protocol)
            .with_init(init)
            .with_sent(sent),
        reader,
        writer,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(pgn: u32, data: &[u8]) -> RawFrame {
        RawFrame::new(None, 3, pgn, 7, 255, data.iter().copied())
    }

    fn decode_all(dec: &mut Decoder, text: &str) -> Vec<RawFrame> {
        let mut events = Vec::new();
        dec.decode(text.as_bytes(), &mut events);
        events
            .into_iter()
            .filter_map(|e| match e {
                DeviceEvent::Frame(f) => Some(f),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn raw_single_frame_round_trips() {
        let f = frame(130306, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
        let enc = Encoder::new(LineFormat::YdwgRaw);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        // TX shape: bare `<CANID> <bytes…>`; the gateway's RX lines add
        // a time-of-day and direction, which the decoder expects.
        let rx: String = tx
            .lines()
            .map(|l| format!("00:00:01.000 R {l}\r\n"))
            .collect();
        let mut dec = Decoder::new(LineFormat::YdwgRaw);
        let out = decode_all(&mut dec, &rx);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 130306);
        assert_eq!(out[0].data.as_slice(), f.data.as_slice());
    }

    #[test]
    fn raw_rejects_a_line_with_more_than_eight_bytes() {
        // The head of one line glued to the tail of the next (#1001): an
        // error and no frame, and the gateway goes on with the next line.
        let mut dec = Decoder::new(LineFormat::YdwgRaw);
        let mut events = Vec::new();
        dec.decode(
            b"17:31:22.999 R 0DF80514 60 2F FC 25 C0 8A 5C 84 41 6A\r\n",
            &mut events,
        );
        assert!(matches!(events.as_slice(), [DeviceEvent::Error(_)]));
        let out = decode_all(
            &mut dec,
            "17:31:23.000 R 09F80114 2B 49 79 21 D0 34 AD 01\r\n",
        );
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 129025);
    }

    #[test]
    fn raw_fast_packet_fragments_and_reassembles() {
        // 129029 GNSS Position Data: 43-byte fast-packet payload.
        let payload: Vec<u8> = (0u8..43).collect();
        let f = frame(129029, &payload);
        let enc = Encoder::new(LineFormat::YdwgRaw);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        assert_eq!(tx.lines().count(), 7, "43 bytes -> 1x6 + 6x7 chunk frames");
        let rx: String = tx
            .lines()
            .map(|l| format!("00:00:01.000 R {l}\r\n"))
            .collect();
        let mut dec = Decoder::new(LineFormat::YdwgRaw);
        let out = decode_all(&mut dec, &rx);
        assert_eq!(out.len(), 1, "fragments must reassemble to one frame");
        assert_eq!(out[0].pgn, 129029);
        assert_eq!(out[0].data.as_slice(), payload.as_slice());
    }

    /// canboat#1039: the echo of a frame canboat wrote is skipped, with
    /// the gateway's source address in place of ours; another client's
    /// transmit is passed on.
    #[test]
    fn raw_skips_only_the_echoes_of_our_own_transmits() {
        let sent = Sent::default();
        let enc = Encoder::new(LineFormat::YdwgRaw).with_sent(sent.clone());
        let mut dec = Decoder::new(LineFormat::YdwgRaw).with_sent(sent);
        let ours = RawFrame::new(None, 6, 59904, 0, 0x0a, [0x14, 0xf0, 0x01]);
        let tx = String::from_utf8(enc.encode_frame(&ours).unwrap()).unwrap();
        assert_eq!(tx, "18EA0A00 14 F0 01\r\n");
        // Echoed from the gateway's address, 0xE2, twice: the second is
        // another client sending the same.
        let echo = "00:00:01.000 T 18EA0AE2 14 F0 01\r\n";
        assert!(decode_all(&mut dec, echo).is_empty());
        let other = decode_all(&mut dec, echo);
        assert_eq!(other.len(), 1);
        assert_eq!((other[0].pgn, other[0].src), (59904, 0xe2));
    }

    /// Not the gateway's time of day, which can run backwards.
    #[test]
    fn frames_get_the_host_time() {
        let before = format_iso_ms(super::super::now_ms());
        let mut dec = Decoder::new(LineFormat::YdwgRaw);
        let out = decode_all(
            &mut dec,
            "00:00:01.010 R 09F11223 01 02 03 04 05 06 07 08\r\n",
        );
        let ts = out[0].timestamp.clone().unwrap();
        assert!(ts >= before && !ts.contains("T00:00:01.010"), "{ts}");
    }

    #[test]
    fn raw_buffers_partial_lines() {
        let mut dec = Decoder::new(LineFormat::YdwgRaw);
        // A split line is buffered across reads and completes on the
        // second push.
        let out = decode_all(&mut dec, "00:00:01.010 R 09F1");
        assert!(out.is_empty());
        let out = decode_all(&mut dec, "1223 01 02 03 04 05 06 07 08\r\n");
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 127250);
    }

    /// A MiniPlex stream: `$MXPGN` frames among other NMEA 0183, a
    /// fast-packet in its frames; what canboat sends goes in the send form.
    #[test]
    fn mxpgn_frames_are_joined_and_sent_in_the_send_form() {
        let mut dec = Decoder::new(LineFormat::Mxpgn);
        let rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let mut text = String::from(rmc);
        let sent = frame(129029, &(0u8..20).collect::<Vec<_>>());
        let mut wire = String::new();
        for chunk in crate::engine::fastpacket::fragment(0, &sent.data).unwrap() {
            let one = RawFrame::new(None, 3, 129029, 7, 255, chunk);
            crate::engine::format::mxpgn::write_line(&mut wire, &one, false).unwrap();
            wire.push_str("\r\n");
        }
        text.push_str(&wire);
        text.push_str(rmc);
        let out = decode_all(&mut dec, &text);
        assert_eq!(out.len(), 1);
        assert_eq!((out[0].pgn, out[0].src), (129029, 7));
        assert_eq!(out[0].data.as_slice(), sent.data.as_slice());

        let enc = Encoder::new(LineFormat::Mxpgn);
        let request = RawFrame::new(None, 6, 59904, 0, 9, [0x16, 0xf0, 0x01]);
        let bytes = enc.encode_frame(&request).unwrap();
        assert_eq!(bytes, b"$MXPGN,00EA00,E309,01F016*6B\r\n");
        let lines = enc.encode_frame(&sent).unwrap();
        let lines = String::from_utf8(lines).unwrap();
        assert_eq!(lines.lines().count(), 3);
        assert!(lines.lines().all(|l| l.starts_with("$MXPGN,01F805,B800,")));
    }

    /// A SeaSmart stream: whole messages among other NMEA 0183, with the
    /// host's time; what canboat sends is one sentence per message.
    #[test]
    fn seasmart_messages_pass_whole_both_ways() {
        let payload: Vec<u8> = (0u8..43).collect();
        let f = frame(129029, &payload);
        let enc = Encoder::new(LineFormat::SeaSmart);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        assert_eq!(tx.lines().count(), 1);
        assert!(tx.starts_with("$PCDIN,01F805,00000000,07,"), "{tx}");

        let rmc = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let mut dec = Decoder::new(LineFormat::SeaSmart);
        let mut events = Vec::new();
        dec.decode(format!("{rmc}{tx}{rmc}").as_bytes(), &mut events);
        let [DeviceEvent::Frame(out)] = events.as_slice() else {
            panic!("{events:?}");
        };
        assert_eq!((out.pgn, out.src), (129029, 7));
        assert_eq!(out.data.as_slice(), payload.as_slice());
        assert!(!out.timestamp.as_deref().unwrap().starts_with("1970"));

        assert!(enc.encode_frame(&frame(129029, &[0; 224])).is_none());
    }

    /// A wrong checksum is an error, unless it is to be ignored.
    #[test]
    fn a_wrong_checksum_is_ignored_only_when_asked() {
        let wrong = "$PCDIN,01F119,00000000,0F,2AAF00D1067414FF*21\r\n\
                     $MXPGN,01F801,2801,C1308AC40C5DE343*18\r\n";
        for (format, pgn) in [(LineFormat::SeaSmart, 127257), (LineFormat::Mxpgn, 129025)] {
            let mut events = Vec::new();
            Decoder::new(format).decode(wrong.as_bytes(), &mut events);
            assert!(
                events.iter().all(|e| matches!(e, DeviceEvent::Error(_))),
                "{events:?}"
            );
            let mut dec = Decoder::new(format).ignore_checksum(true);
            let out = decode_all(&mut dec, wrong);
            assert_eq!(out.len(), 1, "{format:?}");
            assert_eq!(out[0].pgn, pgn);
        }
    }

    #[test]
    fn ascii_lines_pass_complete_frames() {
        // Round-trip a coalesced frame through the N2K ASCII writer and
        // parser: W2K lines carry complete messages, no reassembly.
        let payload: Vec<u8> = (0u8..43).collect();
        let f = frame(129029, &payload);
        let enc = Encoder::new(LineFormat::N2kAscii);
        let tx = String::from_utf8(enc.encode_frame(&f).unwrap()).unwrap();
        assert_eq!(tx.lines().count(), 1);
        let mut dec = Decoder::new(LineFormat::N2kAscii);
        let out = decode_all(&mut dec, &tx);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pgn, 129029);
        assert_eq!(out[0].data.as_slice(), payload.as_slice());
    }

    /// Collects what the writer thread writes.
    #[derive(Clone, Default)]
    struct Sink(std::sync::Arc<Mutex<Vec<u8>>>);
    impl Write for Sink {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    /// What `run` writes as the link opens, before any frame.
    fn written_on_open(init: &[u8]) -> Vec<u8> {
        let sink = Sink::default();
        run(
            Box::new(std::io::empty()),
            Box::new(sink.clone()),
            LineFormat::YdwgRaw,
            BusProtocol::Nmea2000,
            init,
            false,
        )
        .join();
        sink.0.lock().unwrap().clone()
    }

    #[test]
    fn ydnu_is_put_in_raw_mode_on_open() {
        // A network gateway is sent nothing; a YDNU-02 gets `0` + LF.
        assert!(written_on_open(&[]).is_empty());
        assert_eq!(written_on_open(YDNU_RAW_MODE), b"0\n");
    }

    #[test]
    fn synthetic_pgns_are_dropped() {
        let enc = Encoder::new(LineFormat::YdwgRaw);
        assert!(enc.encode_frame(&frame(0x40200, &[1, 2, 3])).is_none());
    }
}
