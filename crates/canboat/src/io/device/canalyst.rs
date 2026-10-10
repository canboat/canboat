// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! CANalyst-II (Waveshare USB-CAN-B) runner: the sans-I/O
//! [`crate::engine::codec::canalyst`] codec on a reader and a writer
//! thread, over the USB message endpoints `io::canalyst` opens.

use std::io::{Read, Write};

use crate::engine::BusProtocol;
use crate::engine::codec::canalyst::Canalyst;

use super::DeviceHandle;

/// Start the reader/writer threads for a bus carrying `protocol`. See
/// [`super::run`].
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    protocol: BusProtocol,
) -> DeviceHandle {
    super::run_codec(Canalyst::new(protocol), None, reader, writer)
}
