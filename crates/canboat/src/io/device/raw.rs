// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Raw links: a [`RawCodec`] on the reader and writer threads of
//! [`super::run`], handing single [`CanFrame`]s in and out, and SocketCAN
//! the same way. No node, no fast-packets: nothing is sent but what the
//! application sends.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::engine::codec::Refused;
use crate::engine::codec::raw::{CanFrame, Event, RawCodec};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle, lock};

/// Run `codec` over `reader` and `writer`.
pub fn run<C: RawCodec + Send + 'static>(
    codec: C,
    reader: Box<dyn std::io::Read + Send>,
    writer: Box<dyn std::io::Write + Send>,
) -> DeviceHandle<CanFrame> {
    let codec = Arc::new(Mutex::new(codec));
    let decoder = Decoder {
        codec: codec.clone(),
        events: Vec::with_capacity(8),
    };
    super::run(decoder, Encoder { codec }, reader, writer)
}

/// The reader thread's view of a shared [`RawCodec`].
struct Decoder<C> {
    codec: Arc<Mutex<C>>,
    events: Vec<Event>,
}

impl<C: RawCodec + Send + 'static> DeviceDecoder<CanFrame> for Decoder<C> {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent<CanFrame>>) {
        lock(&self.codec).receive(bytes, &mut self.events);
        events.extend(self.events.drain(..).map(|ev| match ev {
            Event::Frame(f) => DeviceEvent::Frame(f),
            Event::Error(e) => DeviceEvent::Error(e),
        }));
    }
}

/// The writer thread's view of a shared [`RawCodec`].
struct Encoder<C> {
    codec: Arc<Mutex<C>>,
}

impl<C: RawCodec + Send + 'static> DeviceEncoder<CanFrame> for Encoder<C> {
    fn init_bytes(&self) -> Vec<u8> {
        lock(&self.codec).open()
    }

    fn encode_frame(&self, frame: &CanFrame) -> Option<Vec<u8>> {
        lock(&self.codec)
            .send(frame)
            .map_err(|e| log::warn!("not sending CAN frame {:08X}: {e}", frame.id))
            .ok()
    }

    fn shutdown_bytes(&self) -> Vec<u8> {
        lock(&self.codec).close()
    }

    fn write_gap(&self) -> Duration {
        lock(&self.codec).write_gap()
    }
}

/// Why a frame is not sent on SocketCAN.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn socketcan_check(frame: &CanFrame) -> Result<(), Refused> {
    if frame.data.len() > 8 {
        Err(Refused::TooLarge)
    } else if !frame.is_valid() {
        Err(Refused::Unsupported)
    } else {
        Ok(())
    }
}

/// Open SocketCAN interface `iface` as it is: its link is not configured
/// (bring it up with its bit rate first, `ip link set can0 up type can
/// bitrate 250000`). Frames this socket sends do not come back to it.
#[cfg(target_os = "linux")]
pub fn open_socketcan(iface: &str) -> std::io::Result<DeviceHandle<CanFrame>> {
    linux::open(iface)
}

/// SocketCAN is Linux only.
#[cfg(not(target_os = "linux"))]
pub fn open_socketcan(_iface: &str) -> std::io::Result<DeviceHandle<CanFrame>> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "SocketCAN is only available on Linux",
    ))
}

#[cfg(target_os = "linux")]
mod linux {
    use std::io;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::{Arc, mpsc};
    use std::thread;
    use std::time::{Duration, Instant};

    use smallvec::SmallVec;
    use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

    use super::{CanFrame, socketcan_check};
    use crate::io::device::{DeviceFailure, DeviceHandle, WriterCmd, from_parts};

    /// As the other links: a quiet bus still lets the reader see that
    /// nobody listens any more.
    const READ_TIMEOUT: Duration = Duration::from_millis(250);

    /// How long a write waits for room in the interface's transmit queue.
    const WRITE_WAIT: Duration = Duration::from_secs(1);

    pub(super) fn open(iface: &str) -> io::Result<DeviceHandle<CanFrame>> {
        let sock = CanSocket::open(iface)
            .map_err(|e| io::Error::new(e.kind(), format!("opening {iface}: {e}")))?;
        sock.set_read_timeout(READ_TIMEOUT)?;
        // One socket for both threads: the kernel echoes what a socket sends
        // to every other socket on the interface, so a second one would read
        // our own frames back.
        let sock = Arc::new(sock);
        let (frames_tx, frames_rx) = mpsc::channel::<CanFrame>();
        let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd<CanFrame>>();
        let progress = Arc::new(AtomicU64::new(0));
        let lost = DeviceFailure::default();

        let (rsock, rlost, name) = (sock.clone(), lost.clone(), iface.to_string());
        let reader = thread::Builder::new()
            .name("socketcan-raw-reader".into())
            .spawn(move || read(&rsock, &name, &frames_tx, &rlost))?;
        let (wprogress, name) = (progress.clone(), iface.to_string());
        let writer = thread::Builder::new()
            .name("socketcan-raw-writer".into())
            .spawn(move || write(&sock, &name, &cmd_rx, &wprogress))?;
        Ok(from_parts(
            frames_rx,
            cmd_tx,
            vec![reader, writer],
            progress,
            lost,
        ))
    }

    fn read(
        sock: &CanSocket,
        iface: &str,
        frames_tx: &mpsc::Sender<CanFrame>,
        lost: &DeviceFailure,
    ) {
        loop {
            let frame = match sock.read_frame() {
                Ok(socketcan::CanFrame::Data(d)) => {
                    let (id, extended) = match d.id() {
                        Id::Standard(id) => (u32::from(id.as_raw()), false),
                        Id::Extended(id) => (id.as_raw(), true),
                    };
                    CanFrame {
                        id,
                        extended,
                        data: SmallVec::from_slice(d.data()),
                    }
                }
                // Remote and error frames carry no data.
                Ok(_) => continue,
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    continue;
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    let why = format!("CAN {iface}: read error: {e}");
                    log::error!("{why}");
                    lost.set(&why);
                    return;
                }
            };
            if frames_tx.send(frame).is_err() {
                return;
            }
        }
    }

    fn write(
        sock: &CanSocket,
        iface: &str,
        cmd_rx: &mpsc::Receiver<WriterCmd<CanFrame>>,
        progress: &AtomicU64,
    ) {
        while let Ok(cmd) = cmd_rx.recv() {
            let frame = match cmd {
                WriterCmd::Frame(f) => f,
                // A raw link has no handshake bytes, and no goodbye.
                WriterCmd::Bytes(_) => continue,
                WriterCmd::Shutdown(done) => {
                    if let Some(done) = done {
                        let _ = done.send(true);
                    }
                    return;
                }
            };
            let Some(can) = socketcan_check(&frame)
                .map_err(|e| log::warn!("not sending CAN frame {:08X}: {e}", frame.id))
                .ok()
                .and_then(|()| to_socketcan(&frame))
            else {
                continue;
            };
            // A full transmit queue (ENOBUFS) empties as the bus takes the
            // frames: wait for room rather than drop the frame.
            let deadline = Instant::now() + WRITE_WAIT;
            loop {
                match sock.write_frame(&can) {
                    Ok(()) => {
                        progress.fetch_add(1, Ordering::Relaxed);
                        break;
                    }
                    Err(e)
                        if e.raw_os_error() == Some(libc_enobufs())
                            && Instant::now() < deadline =>
                    {
                        thread::sleep(Duration::from_millis(1));
                    }
                    Err(e) => {
                        log::warn!("CAN {iface}: write failed: {e}");
                        break;
                    }
                }
            }
        }
    }

    /// ENOBUFS, without the `libc` crate the `raw` feature does not need.
    const fn libc_enobufs() -> i32 {
        105
    }

    fn to_socketcan(f: &CanFrame) -> Option<socketcan::CanFrame> {
        let id = if f.extended {
            Id::Extended(ExtendedId::new(f.id)?)
        } else {
            Id::Standard(StandardId::new(u16::try_from(f.id).ok()?)?)
        };
        socketcan::CanFrame::new(id, &f.data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::codec::raw::{Bst95, UsbCanA, Ydwg};
    use crate::engine::format::bst::encode_bst95_can;
    use std::io::{self, Read, Write};
    use std::time::Instant;

    /// Bytes handed out once, then end of stream.
    struct Once(Option<Vec<u8>>);
    impl Read for Once {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match self.0.take() {
                Some(b) => {
                    buf[..b.len()].copy_from_slice(&b);
                    Ok(b.len())
                }
                None => {
                    // Keep the reader alive until the test is done.
                    std::thread::sleep(Duration::from_millis(50));
                    Err(io::ErrorKind::WouldBlock.into())
                }
            }
        }
    }

    /// One write, and when.
    type Written = (Instant, Vec<u8>);

    /// What was written, and when.
    #[derive(Clone, Default)]
    struct Recorder(Arc<Mutex<Vec<Written>>>);
    impl Write for Recorder {
        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().push((Instant::now(), data.to_vec()));
            Ok(data.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn frames_go_both_ways_unchanged() {
        let id = 0x1F1F_02A7;
        let mut wire = Vec::new();
        encode_bst95_can(id, &[1, 2], false, &mut wire);
        let out = Recorder::default();
        let handle = run(
            Bst95::new(),
            Box::new(Once(Some(wire))),
            Box::new(out.clone()),
        );
        let got = handle
            .frames_rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(got, CanFrame::extended(id, &[1, 2]).unwrap());

        handle
            .send_frame(CanFrame::extended(id, &[3]).unwrap())
            .unwrap();
        assert_eq!(handle.close(), crate::io::device::Closed::Confirmed);
        let mut want = Vec::new();
        encode_bst95_can(id, &[3], true, &mut want);
        let written: Vec<Vec<u8>> = out
            .0
            .lock()
            .unwrap()
            .iter()
            .map(|(_, b)| b.clone())
            .collect();
        assert_eq!(written, [want]);
    }

    #[test]
    fn a_receive_only_link_writes_nothing() {
        let out = Recorder::default();
        let handle = run(Ydwg::new(), Box::new(Once(None)), Box::new(out.clone()));
        handle
            .send_frame(CanFrame::extended(1, &[]).unwrap())
            .unwrap();
        assert_eq!(handle.close(), crate::io::device::Closed::Confirmed);
        assert!(out.0.lock().unwrap().is_empty());
    }

    #[test]
    fn writes_are_spaced_by_the_codecs_gap() {
        let out = Recorder::default();
        let handle = run(
            UsbCanA::new(250_000).unwrap(),
            Box::new(Once(None)),
            Box::new(out.clone()),
        );
        for i in 0..5 {
            handle
                .send_frame(CanFrame::extended(i, &[i as u8]).unwrap())
                .unwrap();
        }
        assert_eq!(handle.close(), crate::io::device::Closed::Confirmed);
        let written = out.0.lock().unwrap();
        // The configuration packet, then five frames 0.7 ms apart at least.
        assert_eq!(written.len(), 6);
        for w in written[1..].windows(2) {
            assert!(w[1].0 - w[0].0 >= Duration::from_micros(700));
        }
    }

    #[test]
    fn frames_socketcan_cannot_carry_are_refused() {
        assert_eq!(
            socketcan_check(&CanFrame {
                id: 0x800,
                extended: false,
                data: Default::default()
            }),
            Err(Refused::Unsupported)
        );
    }
}
