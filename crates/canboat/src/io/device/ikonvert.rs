// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Digital Yacht iKonvert runner: the sans-I/O
//! [`crate::engine::codec::ikonvert`] codec on a reader and a writer
//! thread.

use std::io::{Read, Write};

pub use crate::engine::codec::ikonvert::{Config, Ikonvert, pgn_list_status};

use super::DeviceHandle;

/// Start the iKonvert reader/writer threads.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    super::run_codec(Ikonvert::new(config), None, reader, writer)
}
