// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Waveshare USB-CAN-A device: a full NMEA 2000 or J1939 node on the
//! adapter ([`super::can_link`]), as on a CANalyst-II, its frames in the
//! 20-byte packets of [`crate::engine::codec::usb_can_a`] over the
//! adapter's serial port (see [`open_rw`]).

use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::atomic::AtomicU8;
use std::thread;
use std::time::Duration;

use crate::engine::codec::raw_can::CanFrame;
use crate::engine::codec::usb_can_a::{FRAME_LEN, Frames, bitrates, config_packet, encode_frame};

use super::DeviceHandle;
use super::can_link::{self, Link};
use super::iso11783_node::Config;

pub use crate::engine::codec::usb_can_a::BAUD;

/// How long the adapter is given to take a new configuration.
const SETTLE: Duration = Duration::from_millis(100);

struct UsbCanA;

impl Link for UsbCanA {
    const NAME: &'static str = "usb-can-a";
    // The adapter loses frames written faster than it sends them, so they
    // go one at a time; see `write_gap`.
    const FRAMES_PER_WRITE: usize = 1;
    const READ_LEN: usize = 64 * FRAME_LEN;
    type Decoder = Frames;

    fn write_gap(bitrate: u32) -> Duration {
        crate::engine::codec::usb_can_a::write_gap(bitrate)
    }

    fn decode(decoder: &mut Frames, bytes: &[u8], frames: &mut Vec<CanFrame>) {
        let skipped = decoder.skipped;
        decoder.push(bytes, |f| frames.push(f));
        if decoder.skipped != skipped {
            log::debug!(
                "usb-can-a: skipped {} bytes to find a frame",
                decoder.skipped - skipped
            );
        }
    }

    fn encode(frames: &[CanFrame], out: &mut Vec<u8>) {
        for f in frames {
            encode_frame(f, out);
        }
    }
}

/// Open the adapter's serial port, at [`BAUD`] unless `baud` names another
/// speed (one the adapter was switched to).
pub fn open_rw(
    path: &str,
    baud: Option<u32>,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    crate::io::open_serial_rw(path, baud.unwrap_or(BAUD))
}

/// Configure the adapter for `config.bitrate` and `config.protocol`, and run
/// the node on it. `claim_addr` follows the address the node holds, as for
/// SocketCAN.
pub fn run(
    reader: Box<dyn Read + Send>,
    mut writer: Box<dyn Write + Send>,
    config: Config,
    claim_addr: Arc<AtomicU8>,
) -> io::Result<DeviceHandle> {
    let packet = config_packet(config.bitrate, config.protocol).ok_or_else(|| {
        let rates: Vec<String> = bitrates().map(|r| r.to_string()).collect();
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!(
                "a USB-CAN-A runs at {}, not {} bit/s",
                rates.join(", "),
                config.bitrate
            ),
        )
    })?;
    writer.write_all(&packet)?;
    writer.flush()?;
    // It does not answer; until it has the configuration, what it passes on
    // may be in its other framing, which the reader skips.
    thread::sleep(SETTLE);
    can_link::run::<UsbCanA>(reader, writer, config, claim_addr)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;
    use std::sync::atomic::Ordering;

    use crate::engine::format::{iso11783_compose, iso11783_decompose};
    use crate::engine::frame::RawFrame;
    use crate::engine::{BusProtocol, iso_tp};
    use crate::io::device::socketcan::CLAIM_UNCLAIMED;
    use smallvec::SmallVec;

    /// A frame written: `(prio, pgn, src, dst, data)`.
    type Sent = (u8, u32, u8, u8, Vec<u8>);

    /// The adapter's side of the serial port: what the node writes.
    #[derive(Clone, Default)]
    struct Adapter {
        written: Arc<Mutex<Vec<u8>>>,
    }

    impl Write for Adapter {
        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            self.written.lock().unwrap().extend_from_slice(data);
            Ok(data.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Adapter {
        /// The configuration packet, then every frame written so far, as
        /// `(prio, pgn, src, dst, data)`.
        fn written(&self) -> (Vec<u8>, Vec<Sent>) {
            let written = self.written.lock().unwrap();
            let (config, rest) = written.split_at(FRAME_LEN);
            let mut frames = Vec::new();
            let mut decoder = Frames::default();
            decoder.push(rest, |f| {
                let (prio, pgn, src, dst) = iso11783_decompose(f.id);
                frames.push((prio, pgn, src, dst, f.data.to_vec()));
            });
            assert_eq!(decoder.skipped, 0);
            (config.to_vec(), frames)
        }
    }

    /// A serial port that hands out `chunks`, then times out.
    struct Feed(std::collections::VecDeque<Vec<u8>>);

    impl Read for Feed {
        fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
            match self.0.pop_front() {
                Some(p) => {
                    out[..p.len()].copy_from_slice(&p);
                    Ok(p.len())
                }
                None => {
                    thread::sleep(Duration::from_millis(5));
                    Err(io::ErrorKind::TimedOut.into())
                }
            }
        }
    }

    fn wire(id: u32, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        encode_frame(
            &CanFrame {
                id,
                extended: true,
                data: SmallVec::from_slice(data),
            },
            &mut out,
        );
        out
    }

    fn wait_for_claim(claim: &AtomicU8, address: u8) {
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while claim.load(Ordering::Relaxed) != address {
            assert!(std::time::Instant::now() < deadline, "never claimed");
            thread::sleep(Duration::from_millis(20));
        }
    }

    /// A frame split over two reads, behind some noise, reaches the
    /// application.
    #[test]
    fn received_frames_reach_the_application() {
        let frame = wire(iso11783_compose(2, 127250, 0x21, 255), &[1; 8]);
        let mut first = vec![0x55, 0x00];
        first.extend(&frame[..9]);
        let feed = Feed([first, frame[9..].to_vec()].into_iter().collect());
        let config = Config {
            no_claim: true,
            ..Config::default()
        };
        let handle = run(
            Box::new(feed),
            Box::new(Adapter::default()),
            config,
            Arc::new(AtomicU8::new(CLAIM_UNCLAIMED)),
        )
        .unwrap();
        let f = handle
            .frames_rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(
            (f.pgn, f.src, f.data.as_slice()),
            (127250, 0x21, &[1; 8][..])
        );
        handle.join();
    }

    /// The adapter is configured first; the node then claims its address
    /// and sends what the application sends from it, fast-packets split.
    #[test]
    fn the_node_configures_claims_and_then_sends() {
        let adapter = Adapter::default();
        let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
        let config = Config {
            address: 77,
            unique: 1234,
            heartbeat_ms: 0,
            ..Config::default()
        };
        let handle = run(
            Box::new(Feed(Default::default())),
            Box::new(adapter.clone()),
            config,
            claim.clone(),
        )
        .unwrap();
        wait_for_claim(&claim, 77);
        let data: Vec<u8> = (0..20).collect();
        handle
            .send_frame(RawFrame::new(None, 3, 129029, 0, 255, data.iter().copied()))
            .unwrap();
        assert_eq!(handle.close(), super::super::Closed::Confirmed);

        let (config, frames) = adapter.written();
        assert_eq!(
            config,
            config_packet(250_000, BusProtocol::Nmea2000).unwrap()
        );
        assert!(
            frames.iter().any(|f| f.1 == 60928 && f.2 == 77),
            "{frames:?}"
        );
        let sent: Vec<_> = frames.iter().filter(|f| f.1 == 129029).collect();
        assert_eq!(sent.len(), 3, "20 bytes: 3 fast-packet frames");
        assert!(sent.iter().all(|f| f.0 == 3 && f.2 == 77));
        assert_eq!(&sent[0].4[2..], &data[..6]);
    }

    /// A J1939-14 bus at 500 kbit/s: a long message goes out over ISO TP.
    #[test]
    fn a_long_j1939_message_goes_out_over_iso_tp() {
        let adapter = Adapter::default();
        let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
        let config = Config {
            address: 77,
            unique: 1234,
            heartbeat_ms: 0,
            protocol: BusProtocol::J1939,
            bitrate: 500_000,
            ..Config::default()
        };
        let handle = run(
            Box::new(Feed(Default::default())),
            Box::new(adapter.clone()),
            config,
            claim.clone(),
        )
        .unwrap();
        wait_for_claim(&claim, 77);
        let data: Vec<u8> = (0..20).collect();
        handle
            .send_frame(RawFrame::new(None, 6, 65226, 0, 255, data.iter().copied()))
            .unwrap();
        let packets = || {
            adapter
                .written()
                .1
                .into_iter()
                .filter(|f| f.1 == iso_tp::PGN_TP_DT)
                .count()
        };
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while packets() < 3 {
            assert!(std::time::Instant::now() < deadline, "no ISO TP");
            thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(handle.close(), super::super::Closed::Confirmed);
        let (config, frames) = adapter.written();
        assert_eq!(config[3], 0x03, "500 kbit/s");
        let bam = frames.iter().find(|f| f.1 == iso_tp::PGN_TP_CM).unwrap();
        assert_eq!(bam.4[..4], [32, 20, 0, 3]);
    }

    #[test]
    fn frames_go_out_a_frame_time_apart() {
        assert_eq!(UsbCanA::write_gap(250_000), Duration::from_micros(700));
        assert_eq!(UsbCanA::write_gap(500_000), Duration::from_micros(700));
        assert_eq!(UsbCanA::write_gap(125_000), Duration::from_micros(1400));
    }

    #[test]
    fn a_bit_rate_it_lacks_is_refused() {
        let config = Config {
            bitrate: 300_000,
            ..Config::default()
        };
        let Err(e) = run(
            Box::new(Feed(Default::default())),
            Box::new(Adapter::default()),
            config,
            Arc::new(AtomicU8::new(CLAIM_UNCLAIMED)),
        ) else {
            panic!("ran at 300 kbit/s");
        };
        assert_eq!(e.kind(), io::ErrorKind::InvalidInput);
        assert!(e.to_string().contains("250000"), "{e}");
    }
}
