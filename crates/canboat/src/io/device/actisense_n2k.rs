// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense W2K-1 in its Actisense data mode (BST `0xD0` messages over
//! TCP) runner: the sans-I/O [`crate::engine::codec::actisense_n2k`] codec
//! on a reader and a writer thread.

use std::io::{Read, Write};

use crate::engine::codec::actisense_n2k::ActisenseN2k;

use super::DeviceHandle;

/// Start the reader/writer threads. See [`super::run`].
pub fn run(reader: Box<dyn Read + Send>, writer: Box<dyn Write + Send>) -> DeviceHandle {
    super::run_codec(ActisenseN2k::new(), None, reader, writer)
}
