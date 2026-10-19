// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! canboat's own node ([`super::can_link`]) on a gateway that speaks
//! SocketCAN `candump` text, a SensESP N2K gateway: the frames are read in
//! any candump shape and written in the log shape, as by
//! [`crate::engine::codec::raw::Candump`], and the node claims an address,
//! answers ISO Requests and frames what the application sends, as on a
//! CANalyst-II.

use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::atomic::AtomicU8;

use crate::engine::codec::raw::{Candump as Codec, Event, RawCodec};
use crate::engine::codec::raw_can::CanFrame;
use crate::engine::format::candump::write_log;

use super::DeviceHandle;
use super::can_link::{self, Link};
use super::socketcan::Config;

/// The candump link, for [`can_link`].
pub(crate) struct Candump;

impl Link for Candump {
    const NAME: &'static str = "candump";
    const FRAMES_PER_WRITE: usize = 32;
    const READ_LEN: usize = 4096;
    type Decoder = Codec;

    fn decode(codec: &mut Codec, bytes: &[u8], frames: &mut Vec<CanFrame>) {
        let mut events = Vec::new();
        codec.receive(bytes, &mut events);
        for event in events {
            match event {
                Event::Frame(frame) => frames.push(frame),
                Event::Error(e) => log::warn!("candump: {e}"),
            }
        }
    }

    fn encode(frames: &[CanFrame], out: &mut Vec<u8>) {
        let mut text = String::with_capacity(48 * frames.len());
        let now = super::now_ms();
        for f in frames {
            write_log(&mut text, f.id, f.extended, &f.data, now);
        }
        out.extend_from_slice(text.as_bytes());
    }
}

/// Run canboat's node over a candump gateway's byte streams; `claim_addr`
/// follows the address the node holds.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
    claim_addr: Arc<AtomicU8>,
) -> io::Result<DeviceHandle> {
    can_link::run::<Candump>(reader, writer, config, claim_addr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_go_through_the_link_both_ways() {
        let mut decoder = Codec::default();
        let mut frames = Vec::new();
        Candump::decode(
            &mut decoder,
            b"(1.000000) can0 18EEFF00#8EF2DDE800966440\n  can0  123   [1]  05\n",
            &mut frames,
        );
        assert_eq!(frames.len(), 2);
        assert!(frames[0].extended && !frames[1].extended);
        let mut out = Vec::new();
        Candump::encode(&frames, &mut out);
        let text = String::from_utf8(out).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 2);
        assert!(
            lines[0].ends_with(" can0 18EEFF00#8EF2DDE800966440"),
            "{text}"
        );
        assert!(lines[1].ends_with(" can0 123#05"), "{text}");
    }
}
