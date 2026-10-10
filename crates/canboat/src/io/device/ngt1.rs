// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense NGT-1 runner: the sans-I/O [`crate::engine::codec::ngt1`]
//! codec on a reader and a writer thread.
//!
//! The AOS simulator speaks the NGT-1's framing but is no NMEA 2000 node:
//! what it is given in 0x94 messages it sends from an address it never
//! claims. Run with a node [`NodeConfig`] ([`run_with_node`]), a gateway
//! the codec finds to be the simulator gets canboat's own node, as a
//! CANalyst-II does ([`super::iso11783_node`]): it claims an address,
//! answers the network-management requests, and everything goes out from
//! that address, in 0x93 messages. An NGT-1 is a node itself, and stays
//! as it is.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, mpsc};

pub use crate::engine::codec::ngt1::{Config, Ngt1, pgn_list_status};
use crate::engine::codec::{Codec, Event, Refused, SYNTHETIC_PGN_START};
use crate::engine::format::ngt1::encode_n2k_received_frame;
use crate::engine::frame::RawFrame;

pub use super::iso11783_node::Config as NodeConfig;
use super::iso11783_node::{Bus, NmeaDevice, NoStats, TxBuffer, dispatch_cmd, handle_message};
use super::socketcan::CLAIM_UNCLAIMED;
use super::{DeviceHandle, WriterCmd};

/// The synthetic `NMEA 2000 gateway: network status`, which the node
/// emits in place of the codec once it runs.
const PGN_NETWORK_STATUS: u32 = 262400;

/// Start the NGT-1 reader/writer threads. See [`super::run`].
pub fn run(reader: Box<dyn Read + Send>, writer: Box<dyn Write + Send>) -> DeviceHandle {
    run_with_config(reader, writer, Config::default())
}

/// [`run`], with [`Config`].
pub fn run_with_config(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    super::run_codec(Ngt1::new(config), None, reader, writer)
}

/// [`run_with_config`], and a node on an AOS simulator, set up by `node`
/// (its `no_claim` leaves the simulator as it is). `claim_addr` follows the
/// address the node sends from, [`CLAIM_UNCLAIMED`] while it has none.
pub fn run_with_node(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
    node: NodeConfig,
    claim_addr: Arc<AtomicU8>,
) -> DeviceHandle {
    let codec = Ngt1Node {
        codec: Ngt1::new(config),
        config: (!node.no_claim).then_some(node),
        node: None,
        claim_addr,
    };
    super::run_codec(codec, None, reader, writer)
}

/// The NGT-1 codec, and a node once the gateway turns out to be an AOS
/// simulator.
struct Ngt1Node {
    codec: Ngt1,
    /// The node to start; `None` once it runs, or when there is none.
    config: Option<NodeConfig>,
    node: Option<Node>,
    claim_addr: Arc<AtomicU8>,
}

struct Node {
    device: NmeaDevice,
    tx_buf: TxBuffer,
    frames_tx: mpsc::Sender<RawFrame>,
    frames_rx: mpsc::Receiver<RawFrame>,
}

impl Node {
    fn new(config: &NodeConfig) -> Self {
        let (frames_tx, frames_rx) = mpsc::channel();
        Self {
            device: NmeaDevice::new(config, Box::new(NoStats)),
            tx_buf: TxBuffer::whole_messages(),
            frames_tx,
            frames_rx,
        }
    }

    fn bus(&mut self) -> (Bus<'_>, &mut NmeaDevice) {
        (
            Bus {
                tx_buf: &mut self.tx_buf,
                frames_tx: &self.frames_tx,
            },
            &mut self.device,
        )
    }

    /// The node's messages for the bus, in 0x93 messages that carry their
    /// source address.
    fn take_messages(&mut self) -> Vec<u8> {
        let mut out = Vec::new();
        for frame in self.tx_buf.messages.iter_mut().flat_map(|q| q.drain(..)) {
            match encode_n2k_received_frame(&frame, 0) {
                Some(bytes) => out.extend(bytes),
                None => log::warn!(
                    "ngt1: not sending PGN {}: {} bytes do not fit one message",
                    frame.pgn,
                    frame.data.len()
                ),
            }
        }
        out
    }
}

impl Ngt1Node {
    /// Run `codec_events` through the node, once there is one, and add
    /// what the node has for the application and the bus.
    fn after(&mut self, codec_events: Vec<Event>, now_ms: u64, events: &mut Vec<Event>) {
        if self.node.is_none()
            && self.codec.aos().is_some()
            && let Some(config) = self.config.take()
        {
            log::info!(
                "ngt1: the AOS simulator claims no address; joining the bus as a node from address {}",
                config.address
            );
            let mut node = Node::new(&config);
            let (mut bus, device) = node.bus();
            device.start(&mut bus);
            self.node = Some(node);
        }
        let Some(node) = &mut self.node else {
            events.extend(codec_events);
            return;
        };
        for event in codec_events {
            match event {
                Event::Frame(f) if f.pgn == PGN_NETWORK_STATUS => {}
                Event::Frame(f) if f.pgn < SYNTHETIC_PGN_START => {
                    let (mut bus, device) = node.bus();
                    handle_message(&mut bus, device, f);
                }
                other => events.push(other),
            }
        }
        let (mut bus, device) = node.bus();
        device.tick(&mut bus, now_ms);
        events.extend(node.frames_rx.try_iter().map(Event::Frame));
        let bytes = node.take_messages();
        if !bytes.is_empty() {
            events.push(Event::Send(bytes));
        }
        let addr = node.device.claim.send_address().unwrap_or(CLAIM_UNCLAIMED);
        self.claim_addr.store(addr, Ordering::Relaxed);
    }
}

impl Codec for Ngt1Node {
    fn open(&mut self) -> Vec<u8> {
        self.codec.open()
    }

    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        let mut codec_events = Vec::new();
        self.codec.receive(bytes, now_ms, &mut codec_events);
        self.after(codec_events, now_ms, events);
    }

    fn tick(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        let mut codec_events = Vec::new();
        self.codec.tick(now_ms, &mut codec_events);
        self.after(codec_events, now_ms, events);
    }

    /// Through the node once it runs: a frame without a source of its own
    /// (0 or 255) goes out from the node's address, and is dropped while
    /// it has none.
    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        let Some(node) = &mut self.node else {
            return self.codec.send(frame);
        };
        if frame.pgn >= SYNTHETIC_PGN_START {
            return Err(Refused::Synthetic);
        }
        let (mut bus, device) = node.bus();
        dispatch_cmd(&mut bus, device, WriterCmd::Frame(frame.clone()));
        Ok(node.take_messages())
    }

    fn close(&mut self) -> Vec<u8> {
        self.codec.close()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::codec::ngt1::PROBE_AFTER_MS;
    use crate::engine::format::aos::GET_INFO;
    use crate::engine::format::ngt1::{
        BEM_PRODUCT_INFO, N2K_MSG_RECEIVED, N2K_MSG_SEND, NGT_MSG_RECEIVED, Ngt1Decoder, NgtEvent,
        encode_ngt_message,
    };

    const ANSWER: &[u8] = b"{\r\n  \"cmdr\": \"GetInfo\",\r\n  \"Serial\": {\r\n    \"Type\": 24,\r\n    \"Sno\": 2154,\r\n    \"HwVersion\": \"2.0\",\r\n    \"FwVersion\": \"1.6.3.843\"\r\n  }\r\n}\r\n";

    fn now() -> u64 {
        super::super::iso11783_node::now_ms()
    }

    fn gateway() -> Ngt1Node {
        Ngt1Node {
            codec: Ngt1::default(),
            config: Some(NodeConfig {
                address: 35,
                unique: 1234,
                ..NodeConfig::default()
            }),
            node: None,
            claim_addr: Arc::new(AtomicU8::new(CLAIM_UNCLAIMED)),
        }
    }

    /// What `bytes` ask the gateway to do: (command, prio, pgn, src).
    fn sent(bytes: &[u8]) -> Vec<(u8, u32, Option<u8>)> {
        Ngt1Decoder::new()
            .push_bytes(bytes)
            .into_iter()
            .filter_map(|e| match e {
                NgtEvent::Message(m) => {
                    let pgn = u32::from_le_bytes([m.payload[1], m.payload[2], m.payload[3], 0]);
                    let src = (m.command == N2K_MSG_RECEIVED).then(|| m.payload[5]);
                    Some((m.command, pgn, src))
                }
                NgtEvent::Error(_) => None,
            })
            .collect()
    }

    fn sends(events: &[Event]) -> Vec<u8> {
        events
            .iter()
            .filter_map(|e| match e {
                Event::Send(b) if b != GET_INFO => Some(b.clone()),
                _ => None,
            })
            .flatten()
            .collect()
    }

    /// The simulator, once recognised, gets a node: it claims its
    /// address in 0x93 messages, and the application's frames go out from
    /// that address.
    #[test]
    fn the_simulator_gets_a_node_that_claims_and_sends_from_its_address() {
        let mut d = gateway();
        let t = now();
        let mut events = Vec::new();
        d.tick(t, &mut events);
        d.tick(t + PROBE_AFTER_MS, &mut events);
        assert!(
            events
                .iter()
                .any(|e| matches!(e, Event::Send(b) if b == GET_INFO))
        );

        // The node starts on the answer, with a Request for Address Claim
        // from the null address.
        events.clear();
        d.receive(ANSWER, t + PROBE_AFTER_MS + 1, &mut events);
        assert!(d.node.is_some());
        let mut on_the_bus = sent(&sends(&events));
        assert_eq!(on_the_bus[0], (N2K_MSG_RECEIVED, 59904, Some(254)));

        events.clear();
        d.tick(t + 10_000, &mut events);
        on_the_bus.extend(sent(&sends(&events)));
        assert!(
            on_the_bus.contains(&(N2K_MSG_RECEIVED, 60928, Some(35))),
            "{on_the_bus:?}"
        );
        assert_eq!(d.claim_addr.load(Ordering::Relaxed), 35);

        let bytes = d
            .send(&RawFrame::new(None, 6, 127508, 0, 255, [1; 8]))
            .unwrap();
        assert_eq!(sent(&bytes), vec![(N2K_MSG_RECEIVED, 127508, Some(35))]);
    }

    /// An NGT-1 answers BEM commands, is a node itself, and stays as it is.
    #[test]
    fn an_ngt1_gets_no_node() {
        let mut d = gateway();
        let t = now();
        let mut payload = vec![BEM_PRODUCT_INFO, 0, 0x0e, 0x00];
        payload.extend_from_slice(&[0; 8]);
        let mut wire = Vec::new();
        encode_ngt_message(NGT_MSG_RECEIVED, &payload, &mut wire);
        let mut events = Vec::new();
        d.receive(&wire, t, &mut events);
        d.tick(t + 10 * PROBE_AFTER_MS, &mut events);
        assert!(d.node.is_none());
        let bytes = d
            .send(&RawFrame::new(None, 6, 127508, 0, 255, [1; 8]))
            .unwrap();
        assert_eq!(sent(&bytes), vec![(N2K_MSG_SEND, 127508, None)]);
    }
}
