// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Read N2K traffic off a Digital Yacht iKonvert over a serial port
//! and consume `DecodedPgn` events directly (no JSON middleman). This
//! mirrors how merrimac-rs is expected to embed the canboat stack.
//!
//! Usage:
//!
//! ```sh
//!   cargo run -p canboat-tokio --example merrimac_demo -- \
//!       --serial /dev/ttyUSB0 [--baud 230400]
//! ```

use std::time::Duration;

use anyhow::{Context, Result};
use canboat_core::PgnDatabase;
use canboat_tokio::IkonvertStream;
use clap::Parser;
use futures::StreamExt;
use tokio_serial::SerialPortBuilderExt;

#[derive(Parser, Debug)]
#[command(name = "merrimac_demo")]
struct Cli {
    #[arg(long)]
    serial: String,
    /// Baud rate (iKonvert's default is 230400).
    #[arg(long, default_value_t = 230_400)]
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

    let mut stream = IkonvertStream::new(port, db);
    while let Some(decoded) = stream.next().await {
        println!(
            "[pgn {:>6} src {:>3}] {} ({} fields)",
            decoded.pgn,
            decoded.src,
            decoded.description,
            decoded.fields.len(),
        );
    }
    Ok(())
}
