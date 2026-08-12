// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Wire/line-format parsers (sans-I/O).
//!
//! Each submodule handles one canboat-recognised input format. The
//! [`detect`] / [`parse_any`] helpers auto-recognise the format from a
//! line's prefix; binaries pick a single parser via [`InputFormat`]
//! when the user forces it with `--format`.

pub mod actisense_ascii;
pub mod airmar;
pub mod candump;
pub mod chetco;
pub mod common;
pub mod ebl;
pub mod garmin_csv;
pub mod ikonvert;
pub mod maretron_ipg;
pub mod ngt1;
pub mod plain;
pub mod timestamp;
pub mod ydwg02;

pub use common::{iso11783_compose, iso11783_decompose};
pub use ngt1::{
    EblHeader, N2K_MSG_RECEIVED, N2K_MSG_SEND, NGT_MSG_RECEIVED, NGT_MSG_SEND, NGT_STARTUP_SEQ,
    Ngt1Decoder, NgtError, NgtEvent, NgtMessage, encode_n2k_send_frame, encode_n2k_send_payload,
    encode_ngt_message, encode_startup_ping,
};
pub use plain::{ParseError as PlainError, parse_line as parse_plain, write_line as write_plain};
pub use timestamp::{days_since_epoch, days_to_ymd, normalize_timestamp};

use crate::frame::RawFrame;

/// One of the canboat-supported ASCII line formats.
///
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputFormat {
    /// Canboat PLAIN / FAST line (`<ts>,<prio>,<pgn>,...,<hex>,...`).
    /// Once any line is wider than 8 payload bytes the parser locks
    /// into coalesced mode; matches canboat's RAWFORMAT_PLAIN_OR_FAST.
    Plain,
    /// Like [`Plain`] but with no global lock-in — every frame is
    /// dispatched on its own width. Matches canboat's
    /// RAWFORMAT_PLAIN_MIX_FAST, where a single capture interleaves
    /// pre-coalesced (>8 byte) FAST records with raw 8-byte
    /// continuation frames that still need reassembly.
    PlainMixFast,
    /// Actisense N2K ASCII (`A<HHMMSS.mmm> <SDP> <PGN> <data...>`).
    ActisenseAscii,
    /// YDWG-02 / YDEN (`<HH:MM:SS.mmm> R <CANID> <data...>`).
    Ydwg02,
    /// Digital Yacht iKonvert (`!PDGY,...` or `$PDGY,...`).
    Ikonvert,
    /// Airmar (`<ts> - <pgn> <canid> <data>`).
    Airmar,
    /// Chetco `$PCDIN,...` checksummed sentence.
    Chetco,
    /// Garmin CSV1 (relative-ms timestamps).
    GarminCsv,
    /// Garmin CSV2 (absolute `M_D_Y_H_M_S_ms` timestamps + an extra
    /// `Processed PGN` column).
    GarminCsv2,
    /// Linux SocketCAN `candump` text — the pretty per-frame shape
    /// (`  can0  18EEFF00   [8]  8E F2 …`) or the `-l`/`-L` log shape
    /// (`(1436509053.762905) can0 18EEFF00#8EF2…`). One raw CAN frame
    /// per line; canboat C reads these via `candump2analyzer`.
    Candump,
}

/// Parse a `# format=<NAME>` header comment line. Returns the
/// declared format when the line matches, else `None`.
///
/// Mirrors `canboat/analyzer/analyzer.c` and the
/// `CANBOAT_FORMAT_FAST_HEADER` emitted by the canboat C reader
/// binaries (`actisense-serial`, `ikonvert-serial`, `maretron-ipg`).
/// Trailing whitespace / newline are tolerated; matching is
/// case-insensitive on the format name.
pub fn parse_format_header(line: &str) -> Option<InputFormat> {
    let rest = line.strip_prefix("# format=")?;
    // canboat names → InputFormat. The names mirror the C
    // `RAW_FORMAT_STR` table; not every C enum has a Rust analogue
    // (e.g. `RAWFORMAT_NAVLINK2`), so unknown names return `None`.
    let name = rest.trim().to_ascii_uppercase();
    match name.as_str() {
        "PLAIN" | "PLAIN_OR_FAST" => Some(InputFormat::Plain),
        "PLAIN_MIX_FAST" => Some(InputFormat::PlainMixFast),
        // The C tools all emit `format=FAST` on stdout to signal
        // "frames are already coalesced". We don't have a dedicated
        // `Fast` variant — callers consult [`is_coalesced`] on
        // either the returned `InputFormat` or, more usefully, on
        // the raw header text via [`header_implies_coalesced`].
        "FAST" => Some(InputFormat::Plain),
        "ACTISENSE" | "ACTISENSE_ASCII" => Some(InputFormat::ActisenseAscii),
        "YDWG02" => Some(InputFormat::Ydwg02),
        "IKONVERT" => Some(InputFormat::Ikonvert),
        "AIRMAR" => Some(InputFormat::Airmar),
        "CHETCO" => Some(InputFormat::Chetco),
        "GARMIN_CSV1" | "GARMIN" => Some(InputFormat::GarminCsv),
        "GARMIN_CSV2" => Some(InputFormat::GarminCsv2),
        "CANDUMP" => Some(InputFormat::Candump),
        _ => None,
    }
}

/// True when a `# format=<NAME>` header declares a format whose
/// frames are already coalesced PGN payloads (so the reassembler
/// should be bypassed). Mirrors canboat C `analyzer.c:391` —
/// PLAIN/PLAIN_OR_FAST/PLAIN_MIX_FAST/YDWG02 are *not* coalesced;
/// everything else (FAST, ACTISENSE, IKONVERT, AIRMAR, CHETCO,
/// GARMIN_CSV*) is.
pub fn header_implies_coalesced(line: &str) -> bool {
    let Some(rest) = line.strip_prefix("# format=") else {
        return false;
    };
    let name = rest.trim().to_ascii_uppercase();
    !matches!(
        name.as_str(),
        "PLAIN" | "PLAIN_OR_FAST" | "PLAIN_MIX_FAST" | "YDWG02" | "CANDUMP"
    )
}

/// Auto-detect the line format from a single representative line.
/// Returns `None` if nothing matches; callers should fall back to
/// [`InputFormat::Plain`] in that case (canboat's behavior).
pub fn detect(line: &str) -> Option<InputFormat> {
    let t = line.trim_start();
    if t.is_empty() || t.starts_with('#') {
        return None;
    }
    if t.starts_with("!PDGY,") || t.starts_with("$PDGY,") {
        return Some(InputFormat::Ikonvert);
    }
    if t.starts_with("$PCDIN") {
        return Some(InputFormat::Chetco);
    }
    // Garmin CSV header lines stand out — both flavours expose a
    // distinct prefix. Matches `detectFormat` in canboat/analyzer.c.
    if t.starts_with("Sequence #,Timestamp,PGN,") {
        return Some(InputFormat::GarminCsv);
    }
    if t.starts_with("Sequence #,Month_Day_Year_Hours_Minutes_Seconds_msTicks,") {
        return Some(InputFormat::GarminCsv2);
    }
    if t.starts_with('A') && t.as_bytes().get(1).is_some_and(u8::is_ascii_digit) {
        return Some(InputFormat::ActisenseAscii);
    }
    // YDWG02: starts with `HH:MM:SS` followed by `.mmm R/T <hex CAN id>`.
    if looks_like_ydwg02(t) {
        return Some(InputFormat::Ydwg02);
    }
    // Airmar: any timestamp followed by ` - ` (a single dash with
    // surrounding spaces). Canboat's detect checks `p[1] == '-' ||
    // p[2] == '-'` after the first space — both forms get caught
    // here.
    if looks_like_airmar(t) {
        return Some(InputFormat::Airmar);
    }
    // SocketCAN candump text, either shape. Comma-free, so this can
    // never shadow PLAIN; checked after YDWG-02/Airmar, whose lines a
    // candump capture cannot resemble (no `[len]`, no `#`).
    if candump::looks_like_pretty(t) || candump::looks_like_log(t) {
        return Some(InputFormat::Candump);
    }
    // PLAIN/FAST: ISO-like timestamp + `,prio,pgn,…`.
    if t.contains(',') {
        return Some(InputFormat::Plain);
    }
    None
}

fn looks_like_airmar(line: &str) -> bool {
    // Mirror canboat: the first space marks the end of the timestamp
    // and the next byte (or the one after) starts a literal `-`.
    let bytes = line.as_bytes();
    let Some(space) = bytes.iter().position(|&c| c == b' ') else {
        return false;
    };
    matches!(bytes.get(space + 1), Some(&b'-')) || matches!(bytes.get(space + 2), Some(&b'-'))
}

fn looks_like_ydwg02(line: &str) -> bool {
    // `HH:MM:SS.mmm R|T <hex CAN id> …`. The direction token is what makes
    // this YDWG-02 rather than any other time-stamped line, and canboat
    // insists on it:
    //     sscanf(msg, "%d:%d:%d.%d %c %02X ", …) == 6 && (e == 'R' || e == 'T')
    // Matching only the timestamp claimed canboat PLAIN captures whose
    // timestamp happens to be a time of day -- samples/device_functions.csv
    // decoded as nothing at all instead of its three ISO Address Claims.
    let bytes = line.as_bytes();
    if bytes.len() < 13 {
        return false;
    }
    let stamp = bytes[0].is_ascii_digit()
        && bytes[1].is_ascii_digit()
        && bytes[2] == b':'
        && bytes[3].is_ascii_digit()
        && bytes[4].is_ascii_digit()
        && bytes[5] == b':'
        && bytes[6].is_ascii_digit()
        && bytes[7].is_ascii_digit()
        && bytes[8] == b'.';
    if !stamp {
        return false;
    }
    // Skip the fractional digits, then require ` R ` / ` T ` and a hex byte.
    let mut i = 9;
    while bytes.get(i).is_some_and(u8::is_ascii_digit) {
        i += 1;
    }
    if bytes.get(i) != Some(&b' ') {
        return false;
    }
    let dir = bytes.get(i + 1);
    if dir != Some(&b'R') && dir != Some(&b'T') {
        return false;
    }
    if bytes.get(i + 2) != Some(&b' ') {
        return false;
    }
    bytes.get(i + 3).is_some_and(|c| c.is_ascii_hexdigit())
        && bytes.get(i + 4).is_some_and(u8::is_ascii_hexdigit)
}

/// Parse a single line in `format`. iKonvert control sentences return
/// `Ok(None)`; everything else returns either a [`RawFrame`] or a
/// [`PlainError`].
pub fn parse_with(format: InputFormat, line: &str) -> Result<Option<RawFrame>, plain::ParseError> {
    match format {
        InputFormat::Plain | InputFormat::PlainMixFast => plain::parse_line(line).map(Some),
        InputFormat::ActisenseAscii => actisense_ascii::parse_line(line).map(Some),
        InputFormat::Ydwg02 => ydwg02::parse_line(line).map(Some),
        InputFormat::Candump => candump::parse_line(line).map(Some),
        InputFormat::Ikonvert => match ikonvert::parse_line(line)? {
            ikonvert::IkonvertLine::Frame(f) => Ok(Some(f)),
            // Control sentences and stray noise are not frames.
            _ => Ok(None),
        },
        InputFormat::Airmar => airmar::parse_line(line).map(Some),
        InputFormat::Chetco => chetco::parse_line(line).map(Some),
        InputFormat::GarminCsv | InputFormat::GarminCsv2 => {
            let variant = if format == InputFormat::GarminCsv2 {
                garmin_csv::Variant::Absolute
            } else {
                garmin_csv::Variant::Relative
            };
            match garmin_csv::parse_line(line, variant)? {
                garmin_csv::GarminCsvLine::Frame(f) => Ok(Some(f)),
                // Header lines are dropped before reaching the
                // decoder — same as canboat's `continue` on the
                // first line.
                garmin_csv::GarminCsvLine::Header => Ok(None),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_plain() {
        assert_eq!(
            detect("2022-09-10T12:10:16.614Z,6,60928,5,255,8,fb,9b,70,22,00,9b,50,c0"),
            Some(InputFormat::Plain)
        );
    }

    #[test]
    fn detect_actisense_ascii() {
        assert_eq!(
            detect("A173321.107 23FF7 1F119 01 02"),
            Some(InputFormat::ActisenseAscii)
        );
    }

    #[test]
    fn detect_ydwg02() {
        assert_eq!(
            detect("00:17:55.475 R 0DF50B23 FF FF"),
            Some(InputFormat::Ydwg02)
        );
    }

    #[test]
    fn detect_ikonvert() {
        assert_eq!(
            detect("!PDGY,127257,3,35,255,12.345,AQID"),
            Some(InputFormat::Ikonvert)
        );
    }

    #[test]
    fn detect_chetco() {
        assert_eq!(detect("$PCDIN,01F801,..."), Some(InputFormat::Chetco));
    }

    #[test]
    fn detect_ydwg02_needs_its_direction_token() {
        // Real YDWG-02: timestamp, R/T, CAN id.
        assert_eq!(
            detect("21:55:35.425 R 15FD0723 FF C0 D9 6F FF 7F FF FF"),
            Some(InputFormat::Ydwg02)
        );
        assert_eq!(
            detect("00:54:47.929 T 18EAFF44 14 F0 01"),
            Some(InputFormat::Ydwg02)
        );
        // canboat PLAIN whose timestamp is a time of day is NOT YDWG-02,
        // however much the prefix looks alike (samples/device_functions.csv).
        assert_eq!(
            detect("08:15:07.204,6,60928,64,255,8,4F,28,A9,59,00,89,32,C0"),
            Some(InputFormat::Plain)
        );
    }

    #[test]
    fn detect_blank_and_comment_ignored() {
        assert_eq!(detect(""), None);
        assert_eq!(detect("# a comment"), None);
    }
}
