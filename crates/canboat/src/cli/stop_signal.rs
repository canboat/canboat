// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! SIGINT / SIGTERM as a request to stop cleanly, so a gateway can be told
//! goodbye (an iKonvert taken off the bus) before the process ends. A
//! second signal while the first is being handled exits at once.
//!
//! Unix only; elsewhere the signals keep their default, immediate effect.

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

static STOP: AtomicBool = AtomicBool::new(false);

#[cfg(unix)]
extern "C" fn on_signal(_signal: libc::c_int) {
    if STOP.swap(true, Ordering::SeqCst) {
        // SAFETY: `_exit` is async-signal-safe.
        unsafe { libc::_exit(130) };
    }
}

/// Catch SIGINT and SIGTERM from now on.
pub fn install() {
    #[cfg(unix)]
    // SAFETY: the handler only touches an atomic and calls `_exit`, both
    // async-signal-safe.
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
}

/// Whether a stop has been requested.
pub fn received() -> bool {
    STOP.load(Ordering::SeqCst)
}

/// Catch the signals, and on the first one run `stop` on a watcher thread
/// and then exit — for a command whose main thread is blocked and cannot
/// poll [`received`].
pub fn on_stop(stop: impl FnOnce() + Send + 'static) {
    install();
    thread::Builder::new()
        .name("stop-signal".into())
        .spawn(move || {
            while !received() {
                thread::sleep(Duration::from_millis(100));
            }
            log::info!("stopping on signal");
            stop();
            std::process::exit(0);
        })
        .expect("spawn stop-signal watcher");
}
