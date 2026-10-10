// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Linux SocketCAN device adapter — a full NMEA 2000 bus participant.
//!
//! Wraps a raw `CAN_RAW` socket on a Linux SocketCAN interface and runs
//! the ISO 11783-5 address-claim handshake, heartbeat scheduler, and
//! ISO Request / Group Function responders, all on a single internal
//! worker thread. Outbound writes go through a buffered ring drained
//! one frame per `POLLOUT` wakeup so a shallow netdev qdisc never
//! stalls RX or the claim timers.
//!
//! Mirrors the bus-protocol layer of `canboat/socketcan-serial/socketcan-serial.c`.
//! Both the standalone `socketcan-serial` binary and the integrated
//! `canboat-pipeline` use this module via [`run`], which returns a
//! standard [`super::DeviceHandle`].
//!
//! `src` convention on outbound frames sent via `DeviceHandle::send_frame`:
//! `frame.src == 0` is treated as "use my claim address" and rewritten to
//! the currently-claimed source; any other value is sent on the wire as
//! given. While no address is claimed such a frame is dropped: ISO
//! 11783-5 allows only address claims (and requests for them) from the
//! null address (254), so an explicit `src == 254` is dropped otherwise. This matches canboat C's stdin pump and lets quirk
//! synthesisers (e.g. the SCX-20 PGN 126996 fabrication) impersonate
//! other nodes by passing the impersonated `src` explicitly.
//!
//! Linux-only; on every other platform `run` returns an
//! `io::ErrorKind::Unsupported` error and `Config` is a placeholder.

#[cfg(target_os = "linux")]
pub use imp::run;

pub use super::iso11783_node::Config;

use crate::engine::pgn_list::{self, PgnListStatus, PgnListSupport, PgnLists};

/// Sentinel value stored in the claim-address atom when the gateway
/// hasn't successfully claimed an address yet. Callers (e.g.
/// `canboat-pipeline`'s CSV-port injector) treat this as "no rewrite
/// available, leave the caller's `src` alone".
pub const CLAIM_UNCLAIMED: u8 = 254;

/// PGNs the gateway itself originates, always first in its PGN 126464
/// Transmit list: ISO Acknowledgement, ISO Request, ISO Address Claim, Group
/// Function, PGN List, Heartbeat and Product Information.
pub(super) const BUILTIN_TX_PGNS: [u32; 7] = [59392, 59904, 60928, 126208, 126464, 126993, 126996];

/// PGNs the gateway itself consumes, always first in its Receive list: ISO
/// Request, ISO Address Claim and Group Function.
pub(super) const BUILTIN_RX_PGNS: [u32; 3] = [59904, 60928, 126208];

/// How the gateway advertises `lists`: it answers PGN 126464 itself, so both
/// lists are [`Answered`](PgnListSupport::Answered); `dropped` names the PGNs
/// that do not fit.
pub fn pgn_list_status(lists: &PgnLists) -> PgnListStatus {
    let (_, mut dropped) = pgn_list::merge(&BUILTIN_TX_PGNS, &lists.tx);
    let (_, rx_dropped) = pgn_list::merge(&BUILTIN_RX_PGNS, &lists.rx);
    for pgn in rx_dropped {
        if !dropped.contains(&pgn) {
            dropped.push(pgn);
        }
    }
    PgnListStatus {
        tx: PgnListSupport::Answered,
        rx: PgnListSupport::Answered,
        dropped,
    }
}

#[cfg(not(target_os = "linux"))]
pub fn run(
    _iface: &str,
    _config: Config,
    _claim_addr: std::sync::Arc<std::sync::atomic::AtomicU8>,
) -> std::io::Result<super::DeviceHandle> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "SocketCAN is only available on Linux",
    ))
}

#[cfg(target_os = "linux")]
mod imp {
    use std::os::fd::AsRawFd;
    use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};
    use std::sync::{Arc, mpsc};
    use std::thread;

    use crate::engine::frame::RawFrame;
    use crate::engine::{ADDR_GLOBAL, Reassembler};
    use socketcan::{CanInterface, CanSocket, EmbeddedFrame, ExtendedId, Socket, StandardId};

    use super::Config;
    use crate::io::address_claim::ClaimState;
    use crate::io::device::iso11783_node::{
        Bus, CAN_EFF_MASK, CAN_SFF_MASK, LinkStats, LoadSample, MAX_POLL_MS, NmeaDevice, TxBuffer,
        TxFrame, dispatch_cmd, format_iso, handle_frame, now_ms,
    };
    use crate::io::device::{DeviceHandle, WriterCmd, from_parts};

    const CAN_ERR_FLAG: u32 = 0x2000_0000;
    const CAN_EFF_FLAG: u32 = 0x8000_0000;
    const CAN_RTR_FLAG: u32 = 0x4000_0000;

    /// The kernel's view of a SocketCAN link, from
    /// `/sys/class/net/<iface>/`.
    pub(super) struct SysfsStats {
        pub(super) iface: String,
    }

    impl LinkStats for SysfsStats {
        fn bitrate(&self) -> Option<u32> {
            read_bitrate_bps(&self.iface)
        }
        fn errors(&self) -> Option<u32> {
            read_sysfs_counter(&self.iface, "rx_errors")
        }
        fn rejected_tx(&self) -> Option<u32> {
            read_sysfs_counter(&self.iface, "tx_dropped")
        }
        fn load_sample(&self, now_ms: u64) -> Option<LoadSample> {
            read_load_sample(&self.iface, now_ms)
        }
    }

    /// Read a kernel-maintained counter from
    /// `/sys/class/net/<iface>/statistics/<name>`. Returns `None` on
    /// any error (interface gone, file missing, parse fail) so the
    /// network-status emitter degrades to the "no data" sentinel
    /// rather than failing loudly. Cheap: sysfs is a virtual fs and
    /// each read is a handful of bytes; called once per emission
    /// interval (default 5 s).
    fn read_sysfs_counter(iface: &str, name: &str) -> Option<u32> {
        read_sysfs_counter_u64(iface, name).map(|v| v as u32)
    }

    /// The same counter at its native width. The kernel's `netdev`
    /// statistics are monotonic `u64` and sysfs prints them in full, so
    /// the load sampler reads them here rather than through the `u32`
    /// wrapper above: truncating first would manufacture a wrap every
    /// 4 GiB that no later subtraction can undo.
    fn read_sysfs_counter_u64(iface: &str, name: &str) -> Option<u64> {
        let path = format!("/sys/class/net/{iface}/statistics/{name}");
        let raw = std::fs::read_to_string(path).ok()?;
        raw.trim().parse::<u64>().ok()
    }

    /// Read the CAN bus bitrate (bits/s) from
    /// `/sys/class/net/<iface>/can_bittiming/bitrate`. The file
    /// exists for every SocketCAN device and contains a decimal
    /// integer (e.g. `250000`). `None` when missing, unreadable, or
    /// 0; the caller then falls back to `Config::bitrate`, which is
    /// 250 kbit/s unless a J1939-14 bus was configured at 500.
    fn read_bitrate_bps(iface: &str) -> Option<u32> {
        let path = format!("/sys/class/net/{iface}/can_bittiming/bitrate");
        let raw = std::fs::read_to_string(&path).ok()?;
        raw.trim().parse::<u32>().ok().filter(|&v| v != 0)
    }

    /// Take one read of `(rx_bytes, tx_bytes, rx_packets, tx_packets)`
    /// from sysfs, timestamped with `now_ms`. Returns `None` if any
    /// of the counters are unreadable; the emitter then reports
    /// `load_pct = None` and the canboat sentinel survives.
    fn read_load_sample(iface: &str, now_ms: u64) -> Option<LoadSample> {
        Some(LoadSample {
            rx_bytes: read_sysfs_counter_u64(iface, "rx_bytes")?,
            tx_bytes: read_sysfs_counter_u64(iface, "tx_bytes")?,
            rx_packets: read_sysfs_counter_u64(iface, "rx_packets")?,
            tx_packets: read_sysfs_counter_u64(iface, "tx_packets")?,
            at_ms: now_ms,
        })
    }

    /// Try to write the oldest queued frame. Returns true if a frame was
    /// actually delivered; false on empty queue or kernel backpressure
    /// (the frame stays queued, retried on the next wakeup).
    /// Write out everything in the TX ring, for a close. Gives up after
    /// [`super::super::CLOSE_TIMEOUT`] without a frame going out; returns
    /// whether the ring emptied.
    ///
    /// Each frame written bumps `progress`, which the closer watches: a
    /// backlog longer than the timeout keeps the close waiting.
    fn flush_tx(sock: &CanSocket, tx_buf: &mut TxBuffer, progress: &AtomicU64) -> bool {
        let mut last_progress = std::time::Instant::now();
        while !tx_buf.queue.is_empty() {
            if tx_drain_one(sock, tx_buf) {
                progress.fetch_add(1, Ordering::Relaxed);
                last_progress = std::time::Instant::now();
            } else if last_progress.elapsed() >= crate::io::device::CLOSE_TIMEOUT {
                log::warn!(
                    "socketcan: closing with {} frames unsent",
                    tx_buf.queue.len()
                );
                return false;
            } else {
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
        }
        true
    }

    fn tx_drain_one(sock: &CanSocket, tx_buf: &mut TxBuffer) -> bool {
        let Some(frame) = tx_buf.queue.front() else {
            return false;
        };
        let Some(frame) = can_frame(frame) else {
            log::error!("could not build CAN frame for id {:#x}", frame.id);
            tx_buf.pop_sent();
            return false;
        };
        match sock.write_frame(&frame) {
            Ok(()) => {
                tx_buf.pop_sent();
                true
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.raw_os_error() == Some(libc::ENOBUFS) =>
            {
                false
            }
            Err(e) => {
                log::error!("write to CAN: {e} (dropping frame)");
                tx_buf.pop_sent();
                false
            }
        }
    }

    /// The socket's frame for a queued one.
    fn can_frame(f: &TxFrame) -> Option<socketcan::CanFrame> {
        let id = if f.standard {
            socketcan::Id::Standard(StandardId::new(u16::try_from(f.id).ok()?)?)
        } else {
            socketcan::Id::Extended(ExtendedId::new(f.id)?)
        };
        socketcan::CanFrame::new(id, &f.data)
    }

    /// One inbound CAN frame plus its kernel SO_TIMESTAMP: extended
    /// id, payload length, payload bytes, and the timestamp in ms
    /// since the epoch (0 if the cmsg wasn't present).
    struct RecvFrame {
        id: u32,
        dlc: usize,
        data: [u8; 8],
        when_ms: u64,
    }

    /// How many CAN frames one `recvmmsg` syscall reads at most. A busy
    /// NMEA 2000 bus delivers fast-packet PGNs in back-to-back bursts;
    /// draining a whole burst per syscall (instead of one `recvmsg` per
    /// frame) is where the RX-path syscall savings come from.
    const RX_BATCH: usize = 64;

    /// Reusable `recvmmsg` scratch: `RX_BATCH` CAN-frame + control-buffer
    /// slots, plus the `iovec`/`mmsghdr` arrays that point into them.
    /// Boxed and never moved after construction so those interior raw
    /// pointers stay valid across calls; [`RxBatch::prepare`] re-wires
    /// them (and resets the kernel-written lengths) before every call.
    struct RxBatch {
        frames: [[u8; 16]; RX_BATCH], // struct can_frame per slot
        ctrls: [[u8; 64]; RX_BATCH],  // SO_TIMESTAMP cmsg per slot
        iovecs: [libc::iovec; RX_BATCH],
        msgs: [libc::mmsghdr; RX_BATCH],
    }

    impl RxBatch {
        fn new() -> Box<RxBatch> {
            // SAFETY: `iovec`/`mmsghdr` are C POD; an all-zero state is
            // valid (null pointers), and `prepare` wires every pointer
            // and length before the buffer is ever handed to the kernel.
            unsafe { Box::new(std::mem::zeroed()) }
        }

        /// Point each `mmsghdr` at its slot's frame + control buffers and
        /// reset the fields the kernel overwrites (`msg_controllen`,
        /// `msg_flags`, `msg_len`). Must run before each `recvmmsg`.
        fn prepare(&mut self) {
            for i in 0..RX_BATCH {
                self.iovecs[i] = libc::iovec {
                    iov_base: self.frames[i].as_mut_ptr() as *mut libc::c_void,
                    iov_len: self.frames[i].len(),
                };
                let hdr = &mut self.msgs[i].msg_hdr;
                hdr.msg_name = std::ptr::null_mut();
                hdr.msg_namelen = 0;
                hdr.msg_iov = &mut self.iovecs[i];
                hdr.msg_iovlen = 1;
                hdr.msg_control = self.ctrls[i].as_mut_ptr() as *mut libc::c_void;
                hdr.msg_controllen = self.ctrls[i].len() as _;
                hdr.msg_flags = 0;
                self.msgs[i].msg_len = 0;
            }
        }

        /// Decode slot `i` (valid only for `i < ` the count a preceding
        /// `recv_batch` returned) into a [`RecvFrame`]. `None` on a short
        /// datagram — anything below a full `struct can_frame`.
        fn parse(&self, i: usize) -> Option<RecvFrame> {
            if (self.msgs[i].msg_len as usize) < 16 {
                return None;
            }
            let raw = &self.frames[i];
            let mut when = 0u64;
            // SAFETY: `msg_hdr` describes the control buffer the kernel
            // just filled for this slot; walk it for the RX timestamp.
            unsafe {
                let msg: *const libc::msghdr = &self.msgs[i].msg_hdr;
                let mut cmsg = libc::CMSG_FIRSTHDR(msg);
                while !cmsg.is_null() {
                    let c = &*cmsg;
                    if c.cmsg_level == libc::SOL_SOCKET && c.cmsg_type == libc::SCM_TIMESTAMP {
                        let mut tv: libc::timeval = std::mem::zeroed();
                        std::ptr::copy_nonoverlapping(
                            libc::CMSG_DATA(cmsg),
                            &mut tv as *mut _ as *mut u8,
                            std::mem::size_of::<libc::timeval>(),
                        );
                        when = tv.tv_sec as u64 * 1000 + (tv.tv_usec as u64) / 1000;
                    }
                    cmsg = libc::CMSG_NXTHDR(msg, cmsg);
                }
            }
            let id = u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]);
            let dlc = raw[4] as usize;
            let mut data = [0u8; 8];
            data.copy_from_slice(&raw[8..16]);
            Some(RecvFrame {
                id,
                dlc: dlc.min(8),
                data,
                when_ms: when,
            })
        }
    }

    /// One `recvmmsg` call, filling `batch` with up to `RX_BATCH` frames
    /// (each with its kernel SO_TIMESTAMP). Returns the number of frames
    /// read; `Ok(0)` means the socket is drained right now (EAGAIN).
    /// Non-blocking via `MSG_DONTWAIT` so it never stalls the claim
    /// timers even though `poll` already reported readability.
    fn recv_batch(fd: i32, batch: &mut RxBatch) -> std::io::Result<usize> {
        batch.prepare();
        // SAFETY: `msgs` is a live array of `RX_BATCH` wired-up mmsghdrs;
        // the kernel writes only within the buffers `prepare` pointed at.
        let n = unsafe {
            libc::recvmmsg(
                fd,
                batch.msgs.as_mut_ptr(),
                RX_BATCH as libc::c_uint,
                // The flags parameter is c_int on glibc but c_uint on musl;
                // `as _` lets the same call compile for both libc targets
                // (the static-musl release build needs this).
                libc::MSG_DONTWAIT as _,
                std::ptr::null_mut(),
            )
        };
        if n < 0 {
            let e = std::io::Error::last_os_error();
            return match e.raw_os_error() {
                Some(code) if code == libc::EAGAIN || code == libc::EWOULDBLOCK => Ok(0),
                Some(code) if code == libc::EINTR => Ok(0),
                _ => Err(e),
            };
        }
        Ok(n as usize)
    }

    /// Open the SocketCAN interface and spawn the bus-participant
    /// worker thread. Returns a standard [`DeviceHandle`] for the
    /// caller to drain RX from and push TX into.
    ///
    /// `claim_addr` is shared with the worker: it starts at
    /// [`super::CLAIM_UNCLAIMED`] (254) and is updated to our
    /// currently-claimed source address every time the worker decides
    /// it. Callers that need to know the gateway's live address (e.g.
    /// `Bridge::claimed_address`, and the quirks that emit as our own
    /// node) read it from this atom. The supervisor reuses the same
    /// atom across reconnects.
    /// Bus-off auto-recovery delay for the managed bring-up (matches the
    /// historical `ip link … restart-ms 100`).
    const CAN_RESTART_MS: u32 = 100;

    /// Configure and bring up a SocketCAN link via netlink (RTM_NEWLINK),
    /// replacing an external `ip link set … up type can bitrate 250000 …`
    /// unit. A CAN controller must be *down* to set its bit timing, so we
    /// always cycle down → set bitrate + restart-ms → up. Retried a few times
    /// because some controllers (notably the MCP2515) intermittently fail to
    /// enter config mode on the first bring-up after boot, and a fresh down→up
    /// often clears it. Best-effort: on give-up it logs and returns rather
    /// than aborting the device session, so a later supervisor reconnect can
    /// try again. PRIVILEGED — requires the process to run as root.
    fn configure_can_link(iface: &str, bitrate: u32) {
        let ci = match CanInterface::open(iface) {
            Ok(ci) => ci,
            Err(e) => {
                log::error!("CAN {iface}: cannot open netlink handle to configure link: {e}");
                return;
            }
        };
        const ATTEMPTS: u32 = 5;
        for attempt in 1..=ATTEMPTS {
            // Down first: bit timing can only be set while down, and it also
            // resets a controller left half-configured by a prior failed
            // bring-up. An already-down interface makes this a no-op.
            let _ = ci.bring_down();
            let result = ci
                .set_bitrate(bitrate, None::<u32>)
                .and_then(|()| ci.set_restart_ms(CAN_RESTART_MS))
                .and_then(|()| ci.bring_up());
            match result {
                Ok(()) => {
                    log::info!(
                        "CAN {iface}: configured {bitrate} bit/s, \
                         restart-ms {CAN_RESTART_MS}, link up (attempt {attempt})"
                    );
                    return;
                }
                Err(e) => {
                    log::warn!("CAN {iface}: bring-up attempt {attempt}/{ATTEMPTS} failed: {e}");
                    thread::sleep(std::time::Duration::from_millis(500));
                }
            }
        }
        log::error!(
            "CAN {iface}: could not bring link up after {ATTEMPTS} attempts; opening socket \
             anyway (TX will fail with ENETDOWN until the controller recovers)"
        );
    }

    pub fn run(
        iface: &str,
        config: Config,
        claim_addr: Arc<AtomicU8>,
    ) -> std::io::Result<DeviceHandle> {
        // Own the link's lifecycle here instead of relying on an external
        // bring-up (systemd/ip) that may not be ordered before us — the
        // historical failure mode where the N2K interface was left DOWN
        // because its bring-up unit no longer ran first. Best-effort: `run()`
        // proceeds even if it can't come up, so the supervisor keeps retrying.
        if config.configure_link {
            configure_can_link(iface, config.bitrate);
        }
        let sock = CanSocket::open(iface).map_err(std::io::Error::other)?;
        sock.set_nonblocking(true)?;
        let fd = sock.as_raw_fd();

        // Kernel RX timestamps so emitted frames carry the wire time
        // rather than when we got round to draining the kernel queue.
        let on: libc::c_int = 1;
        // SAFETY: setsockopt with a valid fd and an int-sized option.
        let rc = unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_TIMESTAMP,
                &on as *const _ as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as _,
            )
        };
        if rc < 0 {
            log::warn!(
                "SO_TIMESTAMP: {} (continuing without kernel timestamps)",
                std::io::Error::last_os_error()
            );
        }

        let (frames_tx, frames_rx) = mpsc::channel::<RawFrame>();
        let (cmd_tx, cmd_rx) = mpsc::channel::<WriterCmd>();

        let iface_owned = iface.to_string();
        let progress = Arc::new(AtomicU64::new(0));
        let worker_progress = progress.clone();
        let join = thread::Builder::new()
            .name("socketcan-worker".into())
            .spawn(move || {
                let shared = Shared {
                    claim_addr,
                    progress: worker_progress,
                };
                worker(sock, fd, iface_owned, config, frames_tx, cmd_rx, shared)
            })?;

        Ok(from_parts(frames_rx, cmd_tx, vec![join], progress))
    }

    /// What the worker shares with the rest of the process: the claimed
    /// address, and its completed writes (for [`super::super::Closed`]).
    struct Shared {
        claim_addr: Arc<AtomicU8>,
        progress: Arc<AtomicU64>,
    }

    /// The single worker thread that owns the socket and runs the poll
    /// loop. Polls the CAN fd, drains `cmd_rx` via `try_recv`, runs the
    /// claim/heartbeat timers, and drains the TX buffer one frame per
    /// writability wakeup.
    fn worker(
        sock: CanSocket,
        fd: i32,
        iface: String,
        config: Config,
        frames_tx: mpsc::Sender<RawFrame>,
        cmd_rx: mpsc::Receiver<WriterCmd>,
        shared: Shared,
    ) {
        let Shared {
            claim_addr,
            progress,
        } = shared;
        let mut claimer = NmeaDevice::new(
            &config,
            Box::new(SysfsStats {
                iface: iface.clone(),
            }),
        );
        // Reset the claim atom to "unclaimed" so a reconnect resumes
        // with no stale value visible to consumers.
        claim_addr.store(super::CLAIM_UNCLAIMED, Ordering::Relaxed);
        let mut tx_buf = TxBuffer::with_protocol(config.protocol);
        let mut last_published_addr: u8 = super::CLAIM_UNCLAIMED;
        // Fast-packet reassembler driven by the build-time
        // `fastpacket` table. The library hands fully coalesced
        // `RawFrame`s to `frames_tx`, matching the NGT-1 / iKonvert
        // adapter contract; both the standalone binary and
        // canboat-pipeline can then run with `pre_coalesced = true`
        // and skip a second reassembly pass.
        let mut reasm = Reassembler::new();
        // Reused across the whole thread lifetime: one `recvmmsg` scratch
        // buffer so the RX drain allocates nothing per wakeup.
        let mut rx_batch = RxBatch::new();
        let mut last_frame = now_ms();
        let timeout_ms = config.timeout_secs.saturating_mul(1000);

        // Kick off the claim handshake if enabled.
        if claimer.claim.state() != ClaimState::Disabled {
            let mut bus = Bus {
                tx_buf: &mut tx_buf,
                frames_tx: &frames_tx,
            };
            claimer.start(&mut bus);
        }

        loop {
            let now = now_ms();

            // 1. Drain any user-side sends into the TX ring.
            loop {
                match cmd_rx.try_recv() {
                    Ok(WriterCmd::Shutdown(done)) => {
                        // Send what is still queued first, as the serial
                        // writers do, then close the socket — which is how
                        // the gateway leaves the bus — and confirm only if
                        // nothing was left behind.
                        // An ISO TP transfer still running is not waited for:
                        // a BAM paces its packets 50 ms apart (a 1785-byte
                        // one takes ~13 s), and an RTS waits on its
                        // receiver. Abort the RTS ones so their receivers
                        // know, and count the message as left behind.
                        let tp_unfinished = !tx_buf.tp.is_idle();
                        if tp_unfinished {
                            log::warn!("socketcan: leaving with an ISO TP transfer unfinished");
                            for f in tx_buf.tp.abort_all() {
                                tx_buf.push_frame(&f);
                            }
                        }
                        let flushed = flush_tx(&sock, &mut tx_buf, &progress);
                        drop(sock);
                        if let Some(done) = done {
                            let _ = done.send(flushed && !tp_unfinished);
                        }
                        return;
                    }
                    Ok(cmd) => {
                        let mut bus = Bus {
                            tx_buf: &mut tx_buf,
                            frames_tx: &frames_tx,
                        };
                        dispatch_cmd(&mut bus, &mut claimer, cmd);
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                    Err(mpsc::TryRecvError::Disconnected) => return,
                }
            }

            // 2. Pick a poll timeout: soonest of claim deadline,
            //    heartbeat, MAX_POLL_MS, and (when TX is backed up) 5 ms
            //    as a safety net in case POLLOUT lags qdisc availability.
            let mut wait: u64 = if claimer.claim.is_timing() && claimer.claim.deadline() > now {
                claimer.claim.deadline() - now
            } else if claimer.claim.is_claimed() && claimer.heartbeat_interval > 0 {
                claimer.next_heartbeat.saturating_sub(now)
            } else {
                MAX_POLL_MS
            };
            // ISO TP: the next BAM packet, or a CTS / EOMA timeout.
            tx_buf.poll_tp(now);
            if let Some(at) = tx_buf.tp.next_deadline() {
                wait = wait.min(at.saturating_sub(now));
            }
            if !tx_buf.is_empty() && wait > 5 {
                wait = 5;
            }
            if wait > MAX_POLL_MS {
                wait = MAX_POLL_MS;
            }
            let timeout_arg = wait.min(i32::MAX as u64) as i32;

            // 3. poll() — POLLIN always; POLLOUT only while TX is pending.
            let events = if tx_buf.is_empty() {
                libc::POLLIN
            } else {
                libc::POLLIN | libc::POLLOUT
            };
            let mut pfd = libc::pollfd {
                fd,
                events,
                revents: 0,
            };
            // SAFETY: poll over a single live pollfd.
            let r = unsafe { libc::poll(&mut pfd, 1, timeout_arg) };
            if r < 0 {
                let e = std::io::Error::last_os_error();
                if e.raw_os_error() == Some(libc::EINTR) {
                    continue;
                }
                log::error!("socketcan poll: {e}");
                return;
            }

            // 4. Idle-timeout check.
            if timeout_ms > 0 && now_ms().saturating_sub(last_frame) >= timeout_ms {
                log::error!("Timeout {} seconds; no data received", config.timeout_secs);
                return;
            }

            // 5. Drain one TX frame per writability wakeup (or per
            //    safety-net timeout).
            if !tx_buf.is_empty() {
                tx_drain_one(&sock, &mut tx_buf);
            }

            // 6. Drain RX in `recvmmsg` batches up to a per-wakeup budget;
            //    yield then so the claim timers keep advancing on a busy
            //    bus. Reading a whole batch per syscall also lets the
            //    downstream relay coalesce its wakeups: the batch's frames
            //    land in `frames_tx` back-to-back, so a greedy-draining
            //    consumer parks once per batch rather than once per frame.
            if pfd.revents & libc::POLLIN != 0 {
                last_frame = now_ms();
                let mut budget: i64 = 4096;
                loop {
                    let n = match recv_batch(fd, &mut rx_batch) {
                        Ok(0) => break,
                        Ok(n) => n,
                        Err(e) => {
                            log::error!("socketcan recvmmsg: {e}");
                            return;
                        }
                    };
                    for i in 0..n {
                        let Some(rx) = rx_batch.parse(i) else {
                            continue;
                        };
                        if rx.id & CAN_ERR_FLAG != 0 {
                            continue;
                        }
                        // NMEA 2000 and J1939 are always 29-bit extended,
                        // Quick always 11-bit standard: skip the other kind
                        // rather than reinterpret its identifier.
                        let standard = rx.id & CAN_EFF_FLAG == 0;
                        if standard != config.protocol.standard_frames() {
                            continue;
                        }
                        if standard {
                            // A remote frame carries no data to relay.
                            if rx.id & CAN_RTR_FLAG == 0 {
                                let when = if rx.when_ms != 0 {
                                    rx.when_ms
                                } else {
                                    now_ms()
                                };
                                let _ = frames_tx.send(RawFrame::new(
                                    Some(format_iso(when)),
                                    0,
                                    rx.id & CAN_SFF_MASK,
                                    0,
                                    ADDR_GLOBAL,
                                    rx.data[..rx.dlc].iter().copied(),
                                ));
                            }
                            continue;
                        }
                        let mut bus = Bus {
                            tx_buf: &mut tx_buf,
                            frames_tx: &frames_tx,
                        };
                        handle_frame(
                            &mut bus,
                            &mut claimer,
                            &mut reasm,
                            rx.id & CAN_EFF_MASK,
                            &rx.data[..rx.dlc],
                            if rx.when_ms != 0 {
                                rx.when_ms
                            } else {
                                now_ms()
                            },
                        );
                    }
                    budget -= n as i64;
                    // Socket drained (short batch) or budget spent: let the
                    // loop re-poll so claim/heartbeat timers advance.
                    if n < RX_BATCH || budget <= 0 {
                        break;
                    }
                }
            }

            // 7. Claim / heartbeat timers.
            let mut bus = Bus {
                tx_buf: &mut tx_buf,
                frames_tx: &frames_tx,
            };
            claimer.tick(&mut bus, now_ms());

            // 8. Publish the live claim to the shared atom so external
            //    consumers (e.g. the canboat-pipeline CSV injector,
            //    which needs to rewrite an incoming src=0/255 to our
            //    address before mirroring the frame onto the in-process
            //    pipeline) always see what's on the wire. Only update
            //    when actually claimed; while scanning/pending/failed
            //    leave it at CLAIM_UNCLAIMED so the caller knows not
            //    to rewrite yet.
            let live = claimer
                .claim
                .send_address()
                .unwrap_or(super::CLAIM_UNCLAIMED);
            if live != last_published_addr {
                claim_addr.store(live, Ordering::Relaxed);
                last_published_addr = live;
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use crate::engine::format::{iso11783_compose, iso11783_decompose};
        use crate::engine::iso_tp::PGN_TP_CM;
        use crate::engine::{BusProtocol, FramePacketType, Reassembled};
        use crate::io::device::socketcan::CLAIM_UNCLAIMED;

        /// The virtual CAN interface to drive the live-socket tests
        /// against, if one exists. CI creates `vcan0`; a developer box
        /// usually has none, so those tests no-op there rather than
        /// failing. Override with `CANBOAT_VCAN`.
        ///
        /// Never point this at a real bus: the gateway claims an
        /// address and transmits.
        fn vcan_iface() -> Option<String> {
            let iface = std::env::var("CANBOAT_VCAN").unwrap_or_else(|_| "vcan0".into());
            std::path::Path::new(&format!("/sys/class/net/{iface}"))
                .exists()
                .then_some(iface)
        }

        /// Gateway config for the vcan tests: claim an address so the
        /// handshake runs, but never touch the link (that needs root,
        /// and CI has already brought the interface up).
        ///
        /// Cargo runs these tests in parallel on one shared interface,
        /// so each gateway needs its own `unique` and preferred
        /// address: two identical NAMEs would contend for an address
        /// and make the claim flaky. Each test also uses a PGN of its
        /// own, so one test's peer socket never sees another's frames.
        fn vcan_config(unique: u32, address: u8) -> Config {
            Config {
                unique,
                address,
                heartbeat_ms: 0,
                configure_link: false,
                ..Default::default()
            }
        }

        /// A raw socket on the same bus, standing in for another node.
        /// vcan echoes every frame to the *other* sockets on the
        /// interface, which is what makes this a real round trip.
        fn bus_peer(iface: &str) -> CanSocket {
            let sock = CanSocket::open(iface).expect("open vcan peer");
            sock.set_read_timeout(std::time::Duration::from_secs(2))
                .expect("set read timeout");
            sock
        }

        /// A frame another node puts on the bus reaches the gateway's
        /// upstream channel, decomposed into its N2K header. Drives the
        /// whole worker path: poll, `recv_batch`, `handle_frame`,
        /// reassembly, `frames_tx`.
        #[test]
        fn a_frame_on_the_bus_reaches_the_upstream_channel() {
            let Some(iface) = vcan_iface() else {
                eprintln!("no vcan interface; skipping");
                return;
            };
            let peer = bus_peer(&iface);
            let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
            let handle = run(&iface, vcan_config(0x1111, 10), claim).expect("gateway starts");

            // PGN 127245 (Rudder), prio 3, src 0x17 — a single-frame
            // PDU2 message, so it needs no reassembly.
            let canid = iso11783_compose(3, 127245, 0x17, ADDR_GLOBAL);
            let frame = socketcan::CanFrame::new(
                ExtendedId::new(canid & CAN_EFF_MASK).expect("29-bit id"),
                &[0xff, 0xf8, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0xff],
            )
            .expect("build frame");

            // The gateway may still be mid-claim, so re-send until it
            // shows up rather than racing a single write.
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            let got = loop {
                peer.write_frame(&frame).expect("peer writes");
                match handle
                    .frames_rx
                    .recv_timeout(std::time::Duration::from_millis(250))
                {
                    Ok(f) if f.pgn == 127245 => break Some(f),
                    Ok(_) => continue, // network-status or claim traffic
                    Err(_) if std::time::Instant::now() < deadline => continue,
                    Err(_) => break None,
                }
            };

            let got = got.expect("the gateway never reported the frame");
            assert_eq!(got.prio, 3);
            assert_eq!(got.src, 0x17);
            assert_eq!(got.dst, ADDR_GLOBAL);
            assert_eq!(
                &got.data[..],
                &[0xff, 0xf8, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0xff]
            );
        }

        /// A frame handed to `send_frame` reaches the wire, and `src 0`
        /// is rewritten to the address the gateway claimed. Drives
        /// `dispatch_cmd`, the TX ring and `tx_drain_one`.
        #[test]
        fn a_sent_frame_reaches_the_bus_with_the_claimed_source() {
            let Some(iface) = vcan_iface() else {
                eprintln!("no vcan interface; skipping");
                return;
            };
            let peer = bus_peer(&iface);
            let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
            let handle =
                run(&iface, vcan_config(0x2222, 20), Arc::clone(&claim)).expect("gateway starts");

            // Wait out the ISO 11783-5 claim window; nothing contests
            // us on a virtual bus, so this settles quickly.
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            while claim.load(Ordering::Relaxed) == CLAIM_UNCLAIMED
                && std::time::Instant::now() < deadline
            {
                std::thread::yield_now();
            }
            let claimed = claim.load(Ordering::Relaxed);
            assert_ne!(claimed, CLAIM_UNCLAIMED, "gateway never claimed an address");

            // src 0 means "use my claim address". PGN 127251 (Rate of
            // Turn) is this test's alone; the receive test uses 127245.
            handle
                .send_frame(RawFrame::new(
                    None,
                    3,
                    127251,
                    0,
                    ADDR_GLOBAL,
                    [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
                ))
                .expect("writer accepts the frame");

            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            loop {
                assert!(
                    std::time::Instant::now() < deadline,
                    "the frame never reached the bus"
                );
                let Ok(f) = peer.read_frame() else { continue };
                let socketcan::CanFrame::Data(d) = f else {
                    continue;
                };
                let raw_id = match d.id() {
                    socketcan::Id::Extended(e) => e.as_raw(),
                    socketcan::Id::Standard(_) => continue,
                };
                let (prio, pgn, src, dst) = iso11783_decompose(raw_id);
                if pgn != 127251 {
                    continue; // claim traffic, or the other test's frames
                }
                assert_eq!(prio, 3);
                assert_eq!(src, claimed, "src 0 must become the claimed address");
                assert_eq!(dst, ADDR_GLOBAL);
                assert_eq!(d.data(), &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
                break;
            }
        }

        /// Start a J1939 gateway on `iface` and wait for its address.
        fn j1939_gateway(iface: &str, unique: u32, address: u8) -> (DeviceHandle, u8) {
            let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
            let config = Config {
                protocol: BusProtocol::J1939,
                ..vcan_config(unique, address)
            };
            let handle = run(iface, config, Arc::clone(&claim)).expect("gateway starts");
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            while claim.load(Ordering::Relaxed) == CLAIM_UNCLAIMED {
                assert!(std::time::Instant::now() < deadline, "never claimed");
                std::thread::yield_now();
            }
            (handle, claim.load(Ordering::Relaxed))
        }

        /// The next TP frame (PGN 60416 / 60160) `from` puts on the bus,
        /// as (pgn, dst, data, when).
        fn next_tp_frame(peer: &CanSocket, from: u8) -> (u32, u8, Vec<u8>, std::time::Instant) {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
            loop {
                assert!(std::time::Instant::now() < deadline, "no TP frame");
                let Ok(socketcan::CanFrame::Data(d)) = peer.read_frame() else {
                    continue;
                };
                let socketcan::Id::Extended(e) = d.id() else {
                    continue;
                };
                let (_, pgn, src, dst) = iso11783_decompose(e.as_raw());
                if src == from && (pgn == PGN_TP_CM || pgn == crate::engine::iso_tp::PGN_TP_DT) {
                    return (pgn, dst, d.data().to_vec(), std::time::Instant::now());
                }
            }
        }

        /// A 20-byte J1939 message to global leaves as a BAM whose
        /// packets are at least the J1939-21 gap apart and reassemble
        /// into the original.
        #[test]
        fn a_j1939_bam_reaches_the_bus() {
            let Some(iface) = vcan_iface() else {
                eprintln!("no vcan interface; skipping");
                return;
            };
            let peer = bus_peer(&iface);
            let (handle, addr) = j1939_gateway(&iface, 0x4444, 40);
            let data: Vec<u8> = (1..=20).collect();
            // PGN 0x1FF47 is this test's alone.
            handle
                .send_frame(RawFrame::new(
                    None,
                    6,
                    0x1FF47,
                    0,
                    ADDR_GLOBAL,
                    data.iter().copied(),
                ))
                .expect("writer accepts the frame");

            let mut reasm = Reassembler::new();
            let mut last: Option<std::time::Instant> = None;
            let got = loop {
                let (pgn, dst, bytes, when) = next_tp_frame(&peer, addr);
                assert_eq!(dst, ADDR_GLOBAL);
                if pgn == crate::engine::iso_tp::PGN_TP_DT {
                    if let Some(prev) = last {
                        assert!(
                            when - prev >= std::time::Duration::from_millis(40),
                            "BAM packets {:?} apart",
                            when - prev
                        );
                    }
                    last = Some(when);
                }
                let f = RawFrame::new(None, 7, pgn, addr, dst, bytes);
                if let Reassembled::Complete(m) = reasm.push(f, FramePacketType::Single) {
                    break m;
                }
            };
            assert_eq!(got.pgn, 0x1FF47);
            assert_eq!(got.data.as_slice(), data.as_slice());
        }

        /// On a Quick bus the gateway relays 11-bit frames both ways, with
        /// the identifier as the frame's `pgn`, and leaves 29-bit traffic
        /// alone.
        #[test]
        fn a_quick_bus_relays_11_bit_frames_both_ways() {
            let Some(iface) = vcan_iface() else {
                eprintln!("no vcan interface; skipping");
                return;
            };
            let peer = bus_peer(&iface);
            let config = Config {
                protocol: BusProtocol::Quick,
                ..vcan_config(0x6666, 60)
            };
            let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
            let handle = run(&iface, config, claim).expect("gateway starts");

            // In: identifiers 0x7E5 / 0x7E6 are this test's alone.
            let inbound = socketcan::CanFrame::new(
                StandardId::new(0x7E5).expect("11-bit id"),
                &[0xc1, 0x18, 0x6a, 0, 0, 0, 2, 0],
            )
            .expect("build frame");
            let got = loop {
                peer.write_frame(&inbound).expect("peer writes");
                match handle
                    .frames_rx
                    .recv_timeout(std::time::Duration::from_millis(250))
                {
                    Ok(f) if f.pgn == 0x7E5 => break f,
                    Ok(f) => panic!("only 11-bit frames reach a Quick gateway, got {f:?}"),
                    Err(_) => continue,
                }
            };
            assert_eq!((got.prio, got.src, got.dst), (0, 0, ADDR_GLOBAL));
            assert_eq!(got.data.as_slice(), &[0xc1, 0x18, 0x6a, 0, 0, 0, 2, 0]);

            // Out: the frame's pgn goes on the bus as an 11-bit identifier.
            handle
                .send_frame(RawFrame::new(None, 0, 0x7E6, 0, ADDR_GLOBAL, [1u8, 2, 3]))
                .expect("writer accepts the frame");
            let sent = loop {
                let f = peer.read_frame().expect("the gateway sends the frame");
                // vcan0 is shared with the other tests' gateways, whose
                // 29-bit claim traffic shows up here too: skip it.
                match f.id() {
                    socketcan::Id::Standard(id) if id.as_raw() == 0x7E6 => break f,
                    _ => continue,
                }
            };
            assert_eq!(sent.data(), &[1, 2, 3]);
        }

        /// A 20-byte J1939 message to one address runs the RTS/CTS
        /// handshake with that node: the peer here plays the receiver.
        #[test]
        fn a_j1939_rts_cts_transfer_completes() {
            let Some(iface) = vcan_iface() else {
                eprintln!("no vcan interface; skipping");
                return;
            };
            const PEER: u8 = 0x55;
            const PGN: u32 = 0xDA00; // PDU1, this test's alone
            let peer = bus_peer(&iface);
            let (handle, addr) = j1939_gateway(&iface, 0x5555, 50);
            let data: Vec<u8> = (101..=120).collect();
            handle
                .send_frame(RawFrame::new(None, 6, PGN, 0, PEER, data.iter().copied()))
                .expect("writer accepts the frame");

            let (pgn, dst, rts, _) = next_tp_frame(&peer, addr);
            assert_eq!((pgn, dst), (PGN_TP_CM, PEER));
            assert_eq!(rts, [16, 20, 0, 3, 0xFF, 0x00, 0xDA, 0x00], "RTS");

            let reply = |control: u8, b1: u8, b2: u8, b3: u8| {
                let id = iso11783_compose(7, PGN_TP_CM, PEER, addr);
                let f = socketcan::CanFrame::new(
                    ExtendedId::new(id & CAN_EFF_MASK).expect("29-bit id"),
                    &[control, b1, b2, b3, 0xFF, 0x00, 0xDA, 0x00],
                )
                .expect("build frame");
                peer.write_frame(&f).expect("peer writes");
            };
            reply(17, 3, 1, 0xFF); // CTS: all three packets, from 1
            let mut received = Vec::new();
            for seq in 1..=3u8 {
                let (pgn, dst, packet, _) = next_tp_frame(&peer, addr);
                assert_eq!((pgn, dst), (crate::engine::iso_tp::PGN_TP_DT, PEER));
                assert_eq!(packet[0], seq);
                received.extend_from_slice(&packet[1..]);
            }
            received.truncate(20);
            assert_eq!(received, data);
            reply(19, 20, 0, 3); // EOMA: 20 bytes in 3 packets
        }

        /// Opening a nonexistent interface is an error, not a panic or a
        /// hang — the supervisor relies on this to retry.
        #[test]
        fn opening_a_missing_interface_fails_cleanly() {
            let claim = Arc::new(AtomicU8::new(CLAIM_UNCLAIMED));
            assert!(run("definitely-not-an-iface", vcan_config(0x3333, 30), claim).is_err());
        }

        /// An interface with no `can_bittiming/bitrate` has no bit rate
        /// to report, so the node falls back to the configured one.
        #[test]
        fn a_missing_bitrate_file_has_no_rate() {
            assert_eq!(read_bitrate_bps("definitely-not-an-iface"), None);
        }

        /// Unreadable counters mean no sample, so the emitter keeps
        /// canboat's "unknown" sentinel.
        #[test]
        fn a_missing_interface_yields_no_load_sample() {
            assert!(read_load_sample("definitely-not-an-iface", 1000).is_none());
        }
    }
}
