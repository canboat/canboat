// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Maretron IPG100/200 runner: the sans-I/O
//! [`crate::engine::codec::maretron`] codec on a reader and a writer
//! thread.

use std::io::{Read, Write};

use crate::engine::codec::maretron::{self, Maretron};

use super::DeviceHandle;

/// Maretron session configuration.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// Login password. IPG units without security accept the empty
    /// string.
    pub password: String,
    /// Optional fixed timestamp to stamp on every emitted frame —
    /// matches canboat C's `-fixtime` (deterministic-output tests).
    /// When `None`, the host clock is used.
    pub fixtime: Option<String>,
}

/// Start the Maretron reader/writer threads.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    let codec = Maretron::new(maretron::Config {
        password: config.password,
    });
    super::run_codec(codec, config.fixtime, reader, writer)
}
