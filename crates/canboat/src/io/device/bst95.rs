// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense BST-95 (raw CAN frames: a PRO-NDC-1E2K or W2K-1 in its "CAN
//! Actisense" mode over TCP, or an NGX in CAN Packet mode over serial)
//! runner: the sans-I/O [`crate::engine::codec::bst95`] codec on a reader
//! and a writer thread.

use std::io::{Read, Write};

use crate::engine::BusProtocol;
use crate::engine::codec::bst95::Bst95;

use super::DeviceHandle;

/// Start the reader/writer threads for a bus carrying `protocol`. See
/// [`super::run`].
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    protocol: BusProtocol,
) -> DeviceHandle {
    super::run_codec(Bst95::new(protocol), None, reader, writer)
}
