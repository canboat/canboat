// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The Across Ocean Systems NMEA 2000 / NMEA 0183 Simulator
//! (SIM-N2K-N0183), as a gateway.
//!
//! The simulator is a Teensy running Timo Lappalainen's NMEA2000 library,
//! and passes the bus on in the NGT-1's binary framing
//! ([`super::ngt1`]): `N2K_MSG_RECEIVED` (0x93) out, and in both 0x93 —
//! sent from the source address it carries — and 0x94, sent from a fixed
//! address (75) the simulator never claims. It answers no BEM command.
//!
//! On the same serial port it takes JSON commands, `{"cmd":"<name>"}`, and
//! answers outside any frame with `{"cmdr":"<name>", …}`. `GetInfo` is
//! read-only; its answer names the product (`"Type": 24`), the unit's
//! serial number and the firmware. An NGT-1 ignores the command
//! (firmware 2.690, 2026-10): it is text outside any frame.

/// The `GetInfo` command.
pub const GET_INFO: &[u8] = br#"{"cmd":"GetInfo"}"#;

/// The product type the simulator reports in `GetInfo`, the `024` of its
/// firmware's `#Simulator_024_…#` tag.
pub const SIMULATOR_TYPE: u32 = 24;

/// What a `GetInfo` answer says about the unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AosInfo {
    /// `Serial.Type`: [`SIMULATOR_TYPE`] for the simulator.
    pub product_type: u32,
    /// `Serial.Sno`, the unit's serial number.
    pub serial: u32,
    /// `Serial.HwVersion`.
    pub hardware_version: String,
    /// `Serial.FwVersion`.
    pub firmware_version: String,
}

/// Picks `{"cmdr": …}` answers out of the text a gateway sends between
/// frames. Feed it that text; it hands out each complete object.
#[derive(Debug, Default)]
pub struct Answers {
    buf: Vec<u8>,
}

/// More than any answer the simulator gives: text beyond it is noise.
const MAX_ANSWER: usize = 4096;

impl Answers {
    /// Add `text`; return every answer it completes.
    pub fn push(&mut self, text: &[u8]) -> Vec<String> {
        let mut out = Vec::new();
        for &b in text {
            if self.buf.is_empty() && b != b'{' {
                continue;
            }
            self.buf.push(b);
            if b == b'}' && balanced(&self.buf) {
                let object = String::from_utf8_lossy(&self.buf).into_owned();
                self.buf.clear();
                if object.contains("\"cmdr\"") {
                    out.push(object);
                }
            } else if self.buf.len() > MAX_ANSWER {
                self.buf.clear();
            }
        }
        out
    }
}

/// Whether the braces in `text` close, outside strings.
fn balanced(text: &[u8]) -> bool {
    let (mut depth, mut in_string, mut escaped) = (0i32, false, false);
    for &b in text {
        match b {
            _ if escaped => escaped = false,
            b'\\' if in_string => escaped = true,
            b'"' => in_string = !in_string,
            b'{' if !in_string => depth += 1,
            b'}' if !in_string => depth -= 1,
            _ => {}
        }
    }
    depth == 0
}

/// The unit described by a `GetInfo` answer, or `None` for another
/// answer, or one without a `Serial.Type`.
pub fn parse_get_info(answer: &str) -> Option<AosInfo> {
    if string_value(answer, "cmdr")? != "GetInfo" {
        return None;
    }
    Some(AosInfo {
        product_type: number_value(answer, "Type")?,
        serial: number_value(answer, "Sno").unwrap_or(0),
        hardware_version: string_value(answer, "HwVersion").unwrap_or_default(),
        firmware_version: string_value(answer, "FwVersion").unwrap_or_default(),
    })
}

/// The text after `"key":` and any spaces. The simulator's keys are
/// unique within an answer, so the first match is the one.
fn value_of<'a>(answer: &'a str, key: &str) -> Option<&'a str> {
    let at = answer.find(&format!("\"{key}\""))? + key.len() + 2;
    answer[at..]
        .trim_start()
        .strip_prefix(':')
        .map(str::trim_start)
}

fn string_value(answer: &str, key: &str) -> Option<String> {
    let rest = value_of(answer, key)?.strip_prefix('"')?;
    Some(rest[..rest.find('"')?].to_string())
}

fn number_value(answer: &str, key: &str) -> Option<u32> {
    let rest = value_of(answer, key)?;
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A unit's answer, as it came off the serial port (2026-10).
    const GET_INFO_ANSWER: &str = "{\r\n  \"cmdr\": \"GetInfo\",\r\n  \"Serial\": {\r\n    \"Type\": 24,\r\n    \"Sno\": 2154,\r\n    \"Batch\": 23120,\r\n    \"InitializeTime\": \"2024-03-21T22:37:06+00:00\",\r\n    \"TeensySno\": 1470402,\r\n    \"Mac\": \"04:e9:e5:16:6f:c2\",\r\n    \"ChipSno\": \"1632979185-589068759\",\r\n    \"HwVersion\": \"2.0\",\r\n    \"FwVersion\": \"1.6.3.843\",\r\n    \"FwTime\": \"2024-04-23 17:40:33\"\r\n  },\r\n  \"Memory\": {\r\n    \"RAMFree\": 466944,\r\n    \"HeapUsed\": 42608\r\n  },\r\n  \"MessageStatistics\": {\r\n    \"SentN2kMessages\": 13,\r\n    \"ReceivedN2kMessages\": 54105,\r\n    \"SentFailedN2kMessages\": 0,\r\n    \"SentN2kFrames\": 37,\r\n    \"ReceivedN2kFrames\": 0,\r\n    \"LostN2kFrames\": 0,\r\n    \"OrphanN2kFrames\": 0,\r\n    \"MissedN2kMessages\": 0,\r\n    \"SentNMEA0183Messages\": 0,\r\n    \"ReceivedNMEA0183Messages\": 0,\r\n    \"SentFailedNMEA0183Messages\": 0\r\n  },\r\n  \"SystemTime\": \"1970-01-01T00:30:03+00:00\"\r\n}";

    #[test]
    fn get_info_names_the_simulator() {
        assert_eq!(
            parse_get_info(GET_INFO_ANSWER),
            Some(AosInfo {
                product_type: SIMULATOR_TYPE,
                serial: 2154,
                hardware_version: "2.0".into(),
                firmware_version: "1.6.3.843".into(),
            })
        );
    }

    #[test]
    fn another_answer_is_not_get_info() {
        assert_eq!(parse_get_info(r#"{"cmdr":"RestartOK"}"#), None);
    }

    /// The answer arrives in pieces, after stray text, and with a brace
    /// inside a string.
    #[test]
    fn answers_are_picked_out_of_split_text() {
        let mut answers = Answers::default();
        let text = format!("noise}}{GET_INFO_ANSWER}\r\n{{\"cmdr\":\"x\",\"s\":\"}}{{\"}}");
        let (a, b) = text.as_bytes().split_at(200);
        let mut got = answers.push(a);
        got.extend(answers.push(b));
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], GET_INFO_ANSWER);
        assert_eq!(got[1], r#"{"cmdr":"x","s":"}{"}"#);
    }

    #[test]
    fn an_object_without_cmdr_is_skipped() {
        assert!(Answers::default().push(br#"{"a":1}"#).is_empty());
    }
}
