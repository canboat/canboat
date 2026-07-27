// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Read-only TCP listeners shared by the canboat daemons.
//!
//! These are the endpoints whose behaviour is identical whether the
//! records arrive from a live device (the `server` pipeline) or as
//! analyzer JSON on stdin (`n2kd`):
//!
//! * **Snapshot server** (RO, one-shot) — on connect, dump every live
//!   `(pgn, src, secondary)` cache entry as analyzer JSON, then close.
//!   Mirrors canboat C `n2kd`'s base port (2597 by default).
//! * **AIS snapshot server** (RO, one-shot) — same, filtered to the AIS
//!   PGNs plus 129026 / 129029 (canboat C `port+4`).
//! * **Stream server** (RO) — every broadcast line from a [`Hub`] to
//!   every subscribed client, with optional connect-time header.
//!
//! All three read only from the shared [`canboat_core::snapshot`] store
//! and the [`Hub`], so both daemons drive them unchanged. The write /
//! inject and binary-postcard ports stay with the device pipeline —
//! they pull in the device writer and the wire protocol, which the
//! JSON-in daemon has no use for.
//!
//! Nagle is deliberately left ENABLED on every client socket (no
//! `set_nodelay`): these are telemetry streams, so kernel coalescing of
//! consecutive small writes is pure win, and the write loops batch at
//! the application level anyway.

use std::io::Write;
use std::net::{Ipv4Addr, Shutdown, SocketAddr, SocketAddrV4, TcpListener, TcpStream};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{Context, Result};

use canboat_core::snapshot::SnapshotStore;

use super::Hub;

/// How often a shutdown-aware accept loop wakes to re-check its stop flag.
/// A new connection is accepted within this interval of arriving; a handful
/// of idle listeners polling this slowly costs nothing.
pub(crate) const ACCEPT_POLL: Duration = Duration::from_millis(200);

/// Run `listener`'s accept loop, invoking `on_accept` for each blocking
/// client socket, until told to stop.
///
/// * `stop = Some(flag)` — the listener is switched to non-blocking and the
///   loop polls every [`ACCEPT_POLL`], returning (and dropping the listener,
///   so the port can be re-bound) once the flag is set. This is the flag
///   [`crate::server::Bridge::shutdown`] trips to release its served ports.
/// * `stop = None` — the loop blocks on `accept()` forever, the historical
///   behaviour the `n2kd` daemon relies on (it runs until process exit).
pub(crate) fn accept_until(
    name: &str,
    listener: TcpListener,
    stop: Option<Arc<AtomicBool>>,
    mut on_accept: impl FnMut(TcpStream, SocketAddr),
) {
    if stop.is_some()
        && let Err(e) = listener.set_nonblocking(true)
    {
        log::error!("{name}: set_nonblocking failed ({e}); port will not be shutdown-aware");
    }
    loop {
        if let Some(s) = &stop
            && s.load(Ordering::Relaxed)
        {
            log::info!("{name} server shutting down (port released)");
            return;
        }
        match listener.accept() {
            Ok((stream, peer)) => {
                // A socket accepted from a non-blocking listener can itself
                // be non-blocking on some platforms; the client handlers all
                // expect blocking I/O, so force it back.
                if stop.is_some() {
                    let _ = stream.set_nonblocking(false);
                }
                on_accept(stream, peer);
            }
            Err(ref e) if stop.is_some() && e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(ACCEPT_POLL);
            }
            Err(e) => {
                log::error!("{name} accept failed: {e}");
                return;
            }
        }
    }
}

/// One-shot header sent to every CSV (R/W) client on connect. Matches
/// the line the canboat C reader binaries (`actisense-serial`,
/// `ikonvert-serial`, `maretron-ipg`) emit on stdout to declare "frames
/// are already coalesced FAST". A downstream canboat `analyzer` or
/// canboatjs's `Liner + parseActisense` uses this to skip per-CAN-frame
/// reassembly.
pub const CANBOAT_FORMAT_FAST_HEADER: &[u8] = b"# format=FAST\n";

/// Bind the snapshot server. Each accepted client gets one dump of
/// every live `(pgn, src, secondary)` cache entry then the connection
/// closes — same shape as canboat C `n2kd`'s base port.
pub fn spawn_snapshot(
    bind: Ipv4Addr,
    port: u16,
    store: Arc<SnapshotStore>,
    header: Option<&'static [u8]>,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding snapshot TCP port {}:{}", bind, port))?;
    log::info!("snapshot server listening on {}:{}", bind, port);
    Ok(thread::Builder::new()
        .name("snapshot-accept".into())
        .spawn(move || snapshot_accept(listener, store, header, stop))
        .expect("spawn snapshot accept"))
}

fn snapshot_accept(
    listener: TcpListener,
    store: Arc<SnapshotStore>,
    header: Option<&'static [u8]>,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until("snapshot", listener, stop, |stream, peer| {
        log::info!("snapshot client connected: {peer}");
        let s = store.clone();
        thread::Builder::new()
            .name("snapshot-client".into())
            .spawn(move || run_snapshot_client(stream, s, header))
            .ok();
    });
}

fn run_snapshot_client(
    mut stream: TcpStream,
    store: Arc<SnapshotStore>,
    header: Option<&'static [u8]>,
) {
    // Snapshot is a strictly read-only one-shot: FIN the read
    // direction immediately so any client writes during the brief
    // window before the dump completes get ECONNRESET / EPIPE
    // instead of piling up in the kernel's receive buffer.
    if let Err(e) = stream.shutdown(Shutdown::Read) {
        log::debug!("snapshot: shutdown(read) failed: {e}");
    }
    // Optional version/units banner as the first line, so a consumer can
    // detect the unit system before parsing the JSON document.
    if let Some(bytes) = header
        && stream.write_all(bytes).is_err()
    {
        return;
    }
    // Snapshot is a one-shot dump — build the JSON document under
    // the cache lock, then drop the lock before sending so a slow
    // client can't stall the pipeline's `store()` calls.
    let doc = store.snapshot();
    let _ = stream.write_all(doc.as_bytes());
    let _ = stream.flush();
    // Explicit shutdown to signal end-of-stream to the client — TCP
    // close-after-flush is the canonical "one-shot snapshot done"
    // signal. The implicit drop would do the same, but being
    // explicit makes the intent obvious.
    let _ = stream.shutdown(Shutdown::Both);
}

/// Bind the AIS-snapshot server. Same connect-and-dump behaviour as
/// [`spawn_snapshot`], but the dump is filtered to AIS-described PGNs
/// plus PGN 129026 / 129029 — matches canboat C `n2kd`'s `port+4` AIS
/// port.
pub fn spawn_ais_snapshot(
    bind: Ipv4Addr,
    port: u16,
    store: Arc<SnapshotStore>,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding ais TCP port {}:{}", bind, port))?;
    log::info!("ais server listening on {}:{}", bind, port);
    Ok(thread::Builder::new()
        .name("ais-accept".into())
        .spawn(move || ais_snapshot_accept(listener, store, stop))
        .expect("spawn ais accept"))
}

fn ais_snapshot_accept(
    listener: TcpListener,
    store: Arc<SnapshotStore>,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until("ais", listener, stop, |stream, peer| {
        log::info!("ais client connected: {peer}");
        let s = store.clone();
        thread::Builder::new()
            .name("ais-client".into())
            .spawn(move || run_ais_snapshot_client(stream, s))
            .ok();
    });
}

fn run_ais_snapshot_client(mut stream: TcpStream, store: Arc<SnapshotStore>) {
    if let Err(e) = stream.shutdown(Shutdown::Read) {
        log::debug!("ais: shutdown(read) failed: {e}");
    }
    let doc = store.ais_snapshot();
    let _ = stream.write_all(doc.as_bytes());
    let _ = stream.flush();
    let _ = stream.shutdown(Shutdown::Both);
}

/// Bind a read-only TCP server that streams `hub` broadcast lines out
/// to every connected client. The read direction is FIN'd on accept —
/// all injection goes through the dedicated write-only input port, so
/// no broadcast stream is bidirectional.
///
/// `header` is optional bytes written to the client immediately on
/// connect, before the first broadcast line. Used by the raw output
/// port to emit canboat's `# format=FAST\n` so downstream PLAIN/FAST
/// parsers know the stream is pre-coalesced.
pub fn spawn_stream_server(
    name: &'static str,
    bind: Ipv4Addr,
    port: u16,
    hub: Arc<Hub>,
    header: Option<&'static [u8]>,
    stop: Option<Arc<AtomicBool>>,
) -> Result<JoinHandle<()>> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding {name} TCP port {}:{}", bind, port))?;
    log::info!("{name} server listening on {}:{} (RO)", bind, port);
    Ok(thread::Builder::new()
        .name(format!("{name}-accept"))
        .spawn(move || stream_accept(name, listener, hub, header, stop))
        .expect("spawn stream accept"))
}

fn stream_accept(
    name: &'static str,
    listener: TcpListener,
    hub: Arc<Hub>,
    header: Option<&'static [u8]>,
    stop: Option<Arc<AtomicBool>>,
) {
    accept_until(name, listener, stop, |stream, peer| {
        log::info!("{name} client connected: {peer}");
        let h = hub.clone();
        thread::Builder::new()
            .name(format!("{name}-client"))
            .spawn(move || run_stream_client(name, stream, h, header))
            .ok();
    });
}

fn run_stream_client(
    name: &'static str,
    stream: TcpStream,
    hub: Arc<Hub>,
    header: Option<&'static [u8]>,
) {
    // FIN the read direction so any client write attempts get
    // ECONNRESET / EPIPE instead of silently piling up in the kernel's
    // receive buffer — these broadcast ports are strictly read-only.
    if let Err(e) = stream.shutdown(Shutdown::Read) {
        log::debug!("{name}: shutdown(read) failed: {e}");
    }

    // Main: drain the subscription and write to the socket.
    let rx = hub.subscribe();
    let mut stream = stream;
    if let Some(bytes) = header
        && stream.write_all(bytes).is_err()
    {
        return;
    }
    // Batch: block for the first line, then greedily drain whatever
    // is already queued and push it all out with a single write.
    // A burst (one fast-packet flurry decoding into many lines, or a
    // slow client catching up) collapses into one syscall; a quiet
    // stream still sends each line immediately — `try_recv` comes
    // back `Empty` and the single-line batch goes straight out. The
    // cap only bounds memory; a burst larger than it simply takes
    // more than one write.
    const MAX_BATCH: usize = 64 * 1024;
    let mut batch: Vec<u8> = Vec::with_capacity(4096);
    while let Ok(line) = rx.recv() {
        batch.clear();
        batch.extend_from_slice(line.as_bytes());
        while batch.len() < MAX_BATCH {
            match rx.try_recv() {
                Ok(line) => batch.extend_from_slice(line.as_bytes()),
                Err(_) => break,
            }
        }
        if stream.write_all(&batch).is_err() {
            break;
        }
    }
    // Closing the write side drops the fd and ends the client.
    drop(stream);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpStream;
    use std::sync::atomic::AtomicBool;

    /// A shutdown-aware listener drops its socket when the stop flag trips,
    /// so the port is immediately re-bindable — the core of Bridge shutdown
    /// (no more leaked accept threads holding ports open).
    #[test]
    fn shutdown_aware_listener_releases_its_port() {
        // Grab an OS-assigned free port, then let go of it.
        let port = TcpListener::bind((Ipv4Addr::LOCALHOST, 0))
            .unwrap()
            .local_addr()
            .unwrap()
            .port();

        let stop = Arc::new(AtomicBool::new(false));
        let hub = Arc::new(Hub::new());
        let join = spawn_stream_server(
            "test",
            Ipv4Addr::LOCALHOST,
            port,
            hub,
            None,
            Some(stop.clone()),
        )
        .expect("bind test stream server");

        // The port accepts connections while serving.
        let mut connected = false;
        for _ in 0..100 {
            if TcpStream::connect((Ipv4Addr::LOCALHOST, port)).is_ok() {
                connected = true;
                break;
            }
            thread::sleep(Duration::from_millis(20));
        }
        assert!(connected, "server should accept connections while running");

        // Trip the flag; the accept loop must exit and drop the listener.
        stop.store(true, Ordering::Relaxed);
        join.join().expect("accept thread joins after shutdown");

        // Port is free again — a fresh bind succeeds.
        let rebind = TcpListener::bind(SocketAddrV4::new(Ipv4Addr::LOCALHOST, port));
        assert!(
            rebind.is_ok(),
            "port must be re-bindable after shutdown: {:?}",
            rebind.err()
        );
    }
}
