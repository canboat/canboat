// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense BST-D0 (a W2K-1 or PRO-NDC-1E2K in Actisense mode, over
//! TCP) runner: the sans-I/O [`crate::engine::codec::bst_d0`] codec
//! on a reader and a writer thread.

use std::io::{Read, Write};

use crate::engine::codec::bst_d0::BstD0;

use super::DeviceHandle;

/// Start the reader/writer threads. See [`super::run`].
pub fn run(reader: Box<dyn Read + Send>, writer: Box<dyn Write + Send>) -> DeviceHandle {
    super::run_codec(BstD0::new(), None, reader, writer)
}
