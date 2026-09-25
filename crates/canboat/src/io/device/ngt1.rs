// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense NGT-1 runner: the sans-I/O [`crate::engine::codec::ngt1`]
//! codec on a reader and a writer thread.

use std::io::{Read, Write};

pub use crate::engine::codec::ngt1::{Config, Ngt1, pgn_list_status};

use super::DeviceHandle;

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
