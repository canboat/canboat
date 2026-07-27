// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `IkonvertStream` — async adapter for the Digital Yacht iKonvert
//! line-based protocol. iKonvert sends control sentences (`$PDGY,...`)
//! and frame lines (`!PDGY,...,<base64>`) over a serial port.
//!
//! Same shape as [`crate::Ngt1Stream`]: backed by a tokio task that
//! reads lines, parses iKonvert sentences, runs the same reassembly
//! and decode pipeline as the standalone binaries, and pushes
//! `DecodedPgn` events on an mpsc channel.

use std::pin::Pin;
use std::task::{Context, Poll};

use canboat_core::{
    DecodedPgn, FramePacketType, PacketType, PgnDatabase, Reassembled, Reassembler,
    format::ikonvert::{self, IkonvertLine},
};
use futures::Stream;
use tokio::io::{AsyncBufReadExt, AsyncRead, BufReader};
use tokio::sync::mpsc;

pub struct IkonvertStream {
    rx: mpsc::Receiver<DecodedPgn>,
}

impl IkonvertStream {
    pub const DEFAULT_CAPACITY: usize = 256;

    pub fn new<R>(reader: R, db: &'static PgnDatabase) -> Self
    where
        R: AsyncRead + Unpin + Send + 'static,
    {
        Self::with_capacity(reader, db, Self::DEFAULT_CAPACITY)
    }

    pub fn with_capacity<R>(reader: R, db: &'static PgnDatabase, capacity: usize) -> Self
    where
        R: AsyncRead + Unpin + Send + 'static,
    {
        let (tx, rx) = mpsc::channel(capacity);
        tokio::spawn(reader_task(reader, db, tx));
        Self { rx }
    }
}

impl Stream for IkonvertStream {
    type Item = DecodedPgn;
    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<DecodedPgn>> {
        self.rx.poll_recv(cx)
    }
}

async fn reader_task<R>(reader: R, db: &'static PgnDatabase, tx: mpsc::Sender<DecodedPgn>)
where
    R: AsyncRead + Unpin + Send + 'static,
{
    let mut br = BufReader::new(reader);
    let mut reasm = Reassembler::new();
    let mut line = String::with_capacity(512);
    loop {
        line.clear();
        let n = match br.read_line(&mut line).await {
            Ok(0) => return,
            Ok(n) => n,
            Err(e) => {
                log::error!("iKonvert read error: {e}");
                return;
            }
        };
        let _ = n;
        let trimmed = line.trim_end_matches(['\r', '\n']);
        let parsed = match ikonvert::parse_line(trimmed) {
            Ok(p) => p,
            Err(e) => {
                log::warn!("iKonvert parse error on line {trimmed:?}: {e}");
                continue;
            }
        };
        let frame = match parsed {
            IkonvertLine::Frame(f) => f,
            IkonvertLine::Control(c) => {
                log::debug!("iKonvert control: {c}");
                continue;
            }
            IkonvertLine::Other => continue,
        };
        let pt = db
            .first_pgn(frame.pgn)
            .map(|p| match p.packet_type {
                PacketType::Fast => FramePacketType::Fast,
                PacketType::Single => FramePacketType::Single,
                _ => FramePacketType::Other,
            })
            .unwrap_or(FramePacketType::Other);
        let assembled = match reasm.push(frame, pt) {
            Reassembled::PassThrough(f) | Reassembled::Complete(f) => f,
            Reassembled::Partial => continue,
            Reassembled::Error(e) => {
                log::warn!("reassembly error: {e}");
                continue;
            }
        };
        match db.decode(&assembled) {
            Ok(d) => {
                if tx.send(d).await.is_err() {
                    return;
                }
            }
            Err(e) => log::warn!("decode error: {e}"),
        }
    }
}
