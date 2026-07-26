// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Lazy broadcast hub for the read-only TCP outputs.
//!
//! A daemon exposes several read streams a TCP client can subscribe
//! to: PLAIN/FAST CSV, NMEA 0183 (incl. AIVDM), and analyzer JSON. Each
//! stream is one `Hub`: clients subscribe, the producer calls
//! [`Hub::broadcast`] per line, dead subscribers are pruned on the next
//! broadcast.
//!
//! The point of the abstraction is the [`Hub::has_subscribers`] gate:
//! the producer checks it *before* running the formatter. When no one
//! is connected, the steady-state cost per frame is a single relaxed
//! atomic load.
//!
//! Shared by the `server` pipeline and the `n2kd` daemon — the serving
//! layer both feed (see [`super::tcp`]).

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::{Arc, Mutex};

pub struct Hub {
    subscribers: Mutex<Vec<Sender<Arc<str>>>>,
    count: AtomicUsize,
}

impl Hub {
    pub fn new() -> Self {
        Self {
            subscribers: Mutex::new(Vec::new()),
            count: AtomicUsize::new(0),
        }
    }

    #[inline]
    pub fn has_subscribers(&self) -> bool {
        self.count.load(Ordering::Relaxed) > 0
    }

    pub fn subscribe(&self) -> mpsc::Receiver<Arc<str>> {
        let (tx, rx) = mpsc::channel::<Arc<str>>();
        let mut subs = self.subscribers.lock().expect("hub poisoned");
        subs.push(tx);
        self.count.store(subs.len(), Ordering::Relaxed);
        rx
    }

    /// Broadcast `line` (caller's responsibility to include trailing
    /// `\n` if expected). The line is allocated once and shared by
    /// every subscriber — each send is a refcount bump, not a copy.
    /// Dead subscribers are pruned in place.
    pub fn broadcast(&self, line: &str) {
        let shared: Arc<str> = Arc::from(line);
        let mut subs = self.subscribers.lock().expect("hub poisoned");
        subs.retain(|tx| tx.send(shared.clone()).is_ok());
        self.count.store(subs.len(), Ordering::Relaxed);
    }
}

impl Default for Hub {
    fn default() -> Self {
        Self::new()
    }
}

/// Binary sibling of [`Hub`] for the length-prefixed postcard stream
/// (the `--analyzer-binary-port` server). Identical lazy-broadcast
/// design — `has_subscribers` gates the (comparatively expensive)
/// `WirePgn` encode in the pipeline — but the payload is an opaque
/// `Arc<[u8]>` chunk of whole frames rather than a text line.
pub struct BinHub {
    subscribers: Mutex<Vec<Sender<Arc<[u8]>>>>,
    count: AtomicUsize,
}

impl BinHub {
    pub fn new() -> Self {
        Self {
            subscribers: Mutex::new(Vec::new()),
            count: AtomicUsize::new(0),
        }
    }

    #[inline]
    pub fn has_subscribers(&self) -> bool {
        self.count.load(Ordering::Relaxed) > 0
    }

    pub fn subscribe(&self) -> mpsc::Receiver<Arc<[u8]>> {
        let (tx, rx) = mpsc::channel::<Arc<[u8]>>();
        let mut subs = self.subscribers.lock().expect("bin hub poisoned");
        subs.push(tx);
        self.count.store(subs.len(), Ordering::Relaxed);
        rx
    }

    /// Broadcast a chunk of one or more complete frames. The chunk is
    /// allocated once and shared by every subscriber (a refcount bump,
    /// not a copy). Dead subscribers are pruned in place.
    pub fn broadcast(&self, bytes: &[u8]) {
        let shared: Arc<[u8]> = Arc::from(bytes);
        let mut subs = self.subscribers.lock().expect("bin hub poisoned");
        subs.retain(|tx| tx.send(shared.clone()).is_ok());
        self.count.store(subs.len(), Ordering::Relaxed);
    }
}

impl Default for BinHub {
    fn default() -> Self {
        Self::new()
    }
}
