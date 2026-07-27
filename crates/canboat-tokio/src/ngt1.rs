// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `Ngt1Stream` — async adapter feeding NGT-1 binary bytes into the
//! sans-I/O `Ngt1Decoder`, then into the same reassembly + decode
//! pipeline the standalone binaries use.
//!
//! The stream is backed by a tokio task that owns the underlying
//! reader; decoded events arrive on an mpsc channel. A `Drop` on the
//! `Stream` closes the receiver, which causes the task to exit on its
//! next send attempt.

use std::pin::Pin;
use std::task::{Context, Poll};

use canboat_core::{
    DecodedPgn, FramePacketType, PacketType, PgnDatabase, Reassembled, Reassembler,
    format::ngt1::{Ngt1Decoder, NgtEvent},
};
use futures::Stream;
use tokio::io::{AsyncRead, AsyncReadExt};
use tokio::sync::mpsc;

/// Streams [`DecodedPgn`] events off an `AsyncRead` byte source
/// carrying the Actisense NGT-1 binary protocol.
///
/// ```no_run
/// # async fn _example() -> Result<(), Box<dyn std::error::Error>> {
/// use canboat_core::PgnDatabase;
/// use canboat_tokio::Ngt1Stream;
/// use futures::StreamExt;
/// use tokio_serial::SerialPortBuilderExt;
///
/// let db = PgnDatabase::embedded(canboat_core::Units::Metric);
/// let port = tokio_serial::new("/dev/ttyUSB0", 115_200).open_native_async()?;
/// let mut stream = Ngt1Stream::new(port, db);
/// while let Some(decoded) = stream.next().await {
///     println!("{}", decoded.description);
/// }
/// # Ok(()) }
/// ```
pub struct Ngt1Stream {
    rx: mpsc::Receiver<DecodedPgn>,
}

impl Ngt1Stream {
    /// Default channel capacity. Sized for plenty of headroom on a
    /// 250 kbit/s N2K bus: at worst a few hundred frames/sec, mpsc
    /// pushes don't block while the consumer keeps up.
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

impl Stream for Ngt1Stream {
    type Item = DecodedPgn;
    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<DecodedPgn>> {
        self.rx.poll_recv(cx)
    }
}

async fn reader_task<R>(mut reader: R, db: &'static PgnDatabase, tx: mpsc::Sender<DecodedPgn>)
where
    R: AsyncRead + Unpin + Send + 'static,
{
    let mut decoder = Ngt1Decoder::new();
    let mut reasm = Reassembler::new();
    let mut buf = vec![0u8; 4096];
    loop {
        let n = match reader.read(&mut buf).await {
            Ok(0) => return,
            Ok(n) => n,
            Err(e) => {
                log::error!("NGT-1 read error: {e}");
                return;
            }
        };
        for ev in decoder.push_bytes(&buf[..n]) {
            match ev {
                NgtEvent::Message(m) => {
                    let Some(frame) = m.to_raw_frame() else {
                        continue;
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
                                // Consumer dropped — done.
                                return;
                            }
                        }
                        Err(e) => log::warn!("decode error: {e}"),
                    }
                }
                NgtEvent::Error(e) => log::warn!("NGT framing error: {e}"),
                // EBL header records only appear when the decoder is
                // built with `with_ebl()`; this stream sticks to the
                // live NGT-1 path so this arm is dead.
                NgtEvent::Header(_) => {}
            }
        }
    }
}
