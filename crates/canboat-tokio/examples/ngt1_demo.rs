// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Read N2K traffic off an Actisense NGT-1 over a serial port and
//! print one line per decoded PGN as canboat-style text. The
//! interesting part is what isn't here: zero protocol code lives in
//! this file — Ngt1Stream wraps the same sans-I/O state machine the
//! sync actisense-serial binary uses.
//!
//! Usage:
//!
//! ```sh
//!   cargo run -p canboat-tokio --example ngt1_demo -- \
//!       --serial /dev/ttyUSB0 [--baud 115200]
//! ```

use std::time::Duration;

use anyhow::{Context, Result};
use canboat_core::{
    PgnDatabase,
    output::{TextOptions, write_text},
};
use canboat_tokio::Ngt1Stream;
use clap::Parser;
use futures::StreamExt;
use tokio_serial::SerialPortBuilderExt;

#[derive(Parser, Debug)]
#[command(name = "ngt1_demo")]
struct Cli {
    /// Serial device path.
    #[arg(long)]
    serial: String,
    /// Baud rate (NGT-1's default is 115200).
    #[arg(long, default_value_t = 115_200)]
    baud: u32,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    env_logger::init();
    let cli = Cli::parse();

    let db = PgnDatabase::embedded(canboat_core::Units::Metric);

    let port = tokio_serial::new(&cli.serial, cli.baud)
        .timeout(Duration::from_millis(250))
        .open_native_async()
        .with_context(|| format!("opening {}", cli.serial))?;

    let mut stream = Ngt1Stream::new(port, db);
    let opts = TextOptions::default();
    let mut buf = String::with_capacity(256);
    while let Some(decoded) = stream.next().await {
        buf.clear();
        write_text(&mut buf, &decoded, &opts).expect("write to String");
        println!("{buf}");
    }
    Ok(())
}
