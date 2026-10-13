// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! CANalyst-II (Waveshare USB-CAN-B) device: a full NMEA 2000 or J1939
//! node on the adapter's CAN channel ([`super::can_link`]), its frames in
//! the 64-byte packets of [`crate::engine::codec::canalyst`].

use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::atomic::AtomicU8;

use crate::engine::codec::canalyst::{Message, PACKET_LEN, Packets, encode_packets, messages};
use crate::engine::codec::raw_can::CanFrame;

use super::DeviceHandle;
use super::can_link::{self, Link};
use super::iso11783_node::Config;

/// Packets written in one USB transfer at most.
const PACKETS_PER_WRITE: usize = 16;

struct Canalyst;

impl Link for Canalyst {
    const NAME: &'static str = "canalyst";
    const FRAMES_PER_WRITE: usize = PACKETS_PER_WRITE * 3;
    const READ_LEN: usize = 16 * PACKET_LEN;
    type Decoder = Packets;

    fn decode(packets: &mut Packets, bytes: &[u8], frames: &mut Vec<CanFrame>) {
        packets.push(bytes, |packet| match messages(packet) {
            Ok(ms) => frames.extend(ms.iter().map(|m| CanFrame {
                id: m.id,
                extended: m.extended,
                data: m.data.iter().copied().collect(),
            })),
            Err(count) => log::warn!("canalyst: a packet claims {count} messages"),
        });
    }

    fn encode(frames: &[CanFrame], out: &mut Vec<u8>) {
        let messages: Vec<Message> = frames
            .iter()
            .map(|f| Message {
                id: f.id,
                extended: f.extended,
                data: &f.data,
            })
            .collect();
        encode_packets(&messages, out);
    }
}

/// Run the node over an open channel's packet streams (see
/// `io::canalyst::open_rw`). `claim_addr` follows the address the node
/// holds, [`CLAIM_UNCLAIMED`](super::socketcan::CLAIM_UNCLAIMED) while it
/// has none, as for SocketCAN.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
    claim_addr: Arc<AtomicU8>,
) -> io::Result<DeviceHandle> {
    can_link::run::<Canalyst>(reader, writer, config, claim_addr)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;
    use std::sync::atomic::Ordering;
    use std::thread;
    use std::time::Duration;

    use crate::engine::frame::RawFrame;
    use crate::io::device::socketcan::CLAIM_UNCLAIMED;

    use crate::engine::format::{iso11783_compose, iso11783_decompose};

    /// The adapter's side of the link: packets the test feeds the node,
    /// and the frames the node writes.
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
        /// Every frame written so far, as `(prio, pgn, src, dst, data)`.
        fn frames(&self) -> Vec<(u8, u32, u8, u8, Vec<u8>)> {
            let written = self.written.lock().unwrap();
            let (packets, rest) = written.as_chunks::<PACKET_LEN>();
            assert!(rest.is_empty());
            packets
                .iter()
                .flat_map(|p| {
                    messages(p)
                        .unwrap()
                        .into_iter()
                        .map(|m| {
                            let (prio, pgn, src, dst) = iso11783_decompose(m.id);
                            (prio, pgn, src, dst, m.data.to_vec())
                        })
                        .collect::<Vec<_>>()
                })
                .collect()
        }
    }

    /// A reader that hands out `packets`, then reports timeouts until
    /// the node stops reading.
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

    fn packet(id: u32, data: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        encode_packets(
            &[Message {
                id,
                extended: true,
                data,
            }],
            &mut out,
        );
        out
    }

    #[test]
    fn received_frames_reach_the_application() {
        let adapter = Adapter::default();
        let feed = Feed(
            [packet(iso11783_compose(2, 127250, 0x21, 255), &[1; 8])]
                .into_iter()
                .collect(),
        );
        let config = Config {
            no_claim: true,
            ..Config::default()
        };
        let handle = run(
            Box::new(feed),
            Box::new(adapter),
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

    /// The node claims its address on the adapter and, once it holds it,
    /// sends what the application sends from it, fast-packets split.
    #[test]
    fn the_node_claims_and_then_sends() {
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
        // Scan, then the claim's 250 ms to settle.
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while claim.load(Ordering::Relaxed) != 77 {
            assert!(std::time::Instant::now() < deadline, "never claimed");
            thread::sleep(Duration::from_millis(20));
        }
        let claims: Vec<_> = adapter
            .frames()
            .into_iter()
            .filter(|f| f.1 == 60928)
            .collect();
        assert!(claims.iter().any(|f| f.2 == 77), "{claims:?}");

        let data: Vec<u8> = (0..20).collect();
        handle
            .send_frame(RawFrame::new(None, 3, 129029, 0, 255, data.iter().copied()))
            .unwrap();
        assert_eq!(handle.close(), super::super::Closed::Confirmed);
        let sent: Vec<_> = adapter
            .frames()
            .into_iter()
            .filter(|f| f.1 == 129029)
            .collect();
        assert_eq!(sent.len(), 3, "20 bytes: 3 fast-packet frames");
        assert!(sent.iter().all(|f| f.0 == 3 && f.2 == 77));
        assert_eq!(&sent[0].4[2..], &data[..6]);
    }

    /// On a J1939 bus a message over 8 bytes goes out over ISO TP, as on
    /// SocketCAN: to global, a BAM announcement and then its packets.
    #[test]
    fn a_long_j1939_message_goes_out_over_iso_tp() {
        use crate::engine::BusProtocol;
        use crate::engine::iso_tp::{PGN_TP_CM, PGN_TP_DT};
        let adapter = Adapter::default();
        let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
        let config = Config {
            address: 77,
            unique: 1234,
            heartbeat_ms: 0,
            protocol: BusProtocol::J1939,
            ..Config::default()
        };
        let handle = run(
            Box::new(Feed(Default::default())),
            Box::new(adapter.clone()),
            config,
            claim.clone(),
        )
        .unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while claim.load(Ordering::Relaxed) != 77 {
            assert!(std::time::Instant::now() < deadline, "never claimed");
            thread::sleep(Duration::from_millis(20));
        }

        // DM1, 20 bytes: three 7-byte TP.DT packets.
        let data: Vec<u8> = (0..20).collect();
        handle
            .send_frame(RawFrame::new(None, 6, 65226, 0, 255, data.iter().copied()))
            .unwrap();
        let tp = || -> Vec<_> {
            adapter
                .frames()
                .into_iter()
                .filter(|f| f.1 == PGN_TP_CM || f.1 == PGN_TP_DT)
                .collect()
        };
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while tp().iter().filter(|f| f.1 == PGN_TP_DT).count() < 3 {
            assert!(std::time::Instant::now() < deadline, "{:?}", tp());
            thread::sleep(Duration::from_millis(20));
        }
        let tp = tp();
        // The BAM (control byte 32) names the length, packets and PGN.
        let bam = &tp[0];
        assert_eq!((bam.1, bam.2, bam.3), (PGN_TP_CM, 77, 255));
        assert_eq!(bam.4[..4], [32, 20, 0, 3]);
        assert_eq!(u32::from_le_bytes([bam.4[5], bam.4[6], bam.4[7], 0]), 65226);
        let packets: Vec<_> = tp.iter().filter(|f| f.1 == PGN_TP_DT).collect();
        assert_eq!(packets[0].4[..], [1, 0, 1, 2, 3, 4, 5, 6]);
        assert_eq!(packets[2].4[..], [3, 14, 15, 16, 17, 18, 19, 255]);
        assert_eq!(handle.close(), super::super::Closed::Confirmed);
    }
}
