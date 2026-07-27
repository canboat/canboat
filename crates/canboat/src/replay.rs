// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat replay`: pace a captured canboat PLAIN/FAST stream so each
//! line is emitted at its original wall-clock rhythm.
//!
//! Mirrors `canboat/replay/replay.c`. Each input line begins with an
//! ISO-8601 timestamp `YYYY-MM-DDTHH:MM:SS,mmm` (or `.mmm`). We
//! compute the delta between this line's timestamp and the previous
//! one, and sleep that long before forwarding the line. Deltas
//! outside the `(0, 10 000) ms` range are ignored — the line is
//! forwarded immediately. That matches canboat: forward jumps and
//! day-rollovers in the capture don't stall the replay.
//!
//! Typical use:
//!
//! ```text
//!   cat capture.raw | canboat replay | canboat convert --to json
//! ```

use std::io::{self, BufRead, BufWriter, Write};
use std::thread;
use std::time::Duration;

use anyhow::Result;
use canboat_core::format::days_since_epoch;

#[derive(Debug, clap::Args)]
#[command(after_help = canboat_cli::help_footer())]
pub struct Args {
    /// Verbose / debug logging — alias of `-d`. Matches canboat's C tool.
    #[arg(short = 'v', long)]
    verbose: bool,

    /// Debug logging.
    #[arg(short = 'd', long)]
    debug: bool,

    /// Quiet — only show errors.
    #[arg(short = 'q', long)]
    quiet: bool,

    /// Maximum gap to honour, in milliseconds. Gaps larger than this
    /// are treated as forward jumps and the next line is emitted
    /// immediately. Matches the hard-coded 10 000 ms cap in
    /// `replay.c`; exposed here so tests can shrink it.
    #[arg(long, value_name = "MS", default_value_t = 10_000u64)]
    max_gap: u64,
}

pub fn run(args: Args) -> Result<()> {
    let level = if args.quiet {
        "error"
    } else if args.debug || args.verbose {
        "debug"
    } else {
        "info"
    };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level)).init();
    canboat_cli::log_startup(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));

    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    let mut prev_ms: Option<i64> = None;
    let mut line = String::with_capacity(512);
    let mut lock = stdin.lock();

    loop {
        line.clear();
        match lock.read_line(&mut line) {
            Ok(0) => break,
            Ok(_) => {}
            Err(e) => return Err(e.into()),
        }
        let now_ms = parse_timestamp_ms(&line);
        if let (Some(now), Some(prev)) = (now_ms, prev_ms) {
            let delta = now - prev;
            if delta > 0 && (delta as u64) < args.max_gap {
                log::debug!("sleeping {} ms", delta);
                thread::sleep(Duration::from_millis(delta as u64));
            }
        }
        prev_ms = now_ms.or(prev_ms);
        out.write_all(line.as_bytes())?;
        out.flush()?;
    }
    Ok(())
}

/// Parse the leading `YYYY-MM-DDTHH:MM:SS[.,]mmm` prefix of a canboat
/// PLAIN line into Unix-epoch milliseconds (UTC). Returns `None`
/// when the line doesn't carry a recognised timestamp — the caller
/// treats that as "emit immediately, leave the previous timestamp
/// as-is".
fn parse_timestamp_ms(line: &str) -> Option<i64> {
    let bytes = line.as_bytes();
    if bytes.len() < 19 {
        return None;
    }
    let take = |range: std::ops::Range<usize>| std::str::from_utf8(&bytes[range]).ok();
    let y: i64 = take(0..4)?.parse().ok()?;
    if bytes[4] != b'-' {
        return None;
    }
    let mo: u32 = take(5..7)?.parse().ok()?;
    if bytes[7] != b'-' {
        return None;
    }
    let d: u32 = take(8..10)?.parse().ok()?;
    if bytes[10] != b'T' && bytes[10] != b'-' && bytes[10] != b' ' {
        return None;
    }
    let h: u32 = take(11..13)?.parse().ok()?;
    if bytes[13] != b':' {
        return None;
    }
    let m: u32 = take(14..16)?.parse().ok()?;
    if bytes[16] != b':' {
        return None;
    }
    let s: u32 = take(17..19)?.parse().ok()?;
    let mut ms: i64 = 0;
    if bytes.len() >= 23
        && (bytes[19] == b'.' || bytes[19] == b',')
        && let Some(v) = take(20..23).and_then(|s| s.parse::<i64>().ok())
    {
        ms = v;
    }
    let days = days_since_epoch(y, mo as i64, d as i64)?;
    let secs = days * 86_400 + (h as i64) * 3600 + (m as i64) * 60 + s as i64;
    Some(secs * 1000 + ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_iso_timestamps() {
        // 1970-01-01T00:00:00.000 = 0 ms.
        assert_eq!(parse_timestamp_ms("1970-01-01T00:00:00.000 …"), Some(0));
        // 1970-01-01T00:00:01,500 → 1500 ms.
        assert_eq!(
            parse_timestamp_ms("1970-01-01T00:00:01,500 anything"),
            Some(1_500)
        );
        // 2022-09-10T12:10:33.618 UTC → 1_662_811_833_618 ms.
        // (Cross-check with Python:
        //  `int(datetime.datetime.fromisoformat('2022-09-10T12:10:33.618+00:00').timestamp()*1000)`.)
        assert_eq!(
            parse_timestamp_ms("2022-09-10T12:10:33.618Z"),
            Some(1_662_811_833_618)
        );
    }

    #[test]
    fn ignores_unrecognised_prefixes() {
        assert_eq!(parse_timestamp_ms("# comment\n"), None);
        assert_eq!(parse_timestamp_ms("garbage\n"), None);
        assert_eq!(parse_timestamp_ms("12:34:56.789\n"), None);
    }
}
