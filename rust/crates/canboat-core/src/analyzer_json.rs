// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Minimal JSON-value extraction by substring scan over analyzer-shape
//! JSON. Matches the pragmatic style canboat's C `getJSONValue` uses
//! — we never need a real parser because the analyzer's output is
//! line-delimited and has a known shape.
//!
//! Used by [`crate::snapshot::classify_json_line`] to compute snapshot
//! keys directly from an analyzer JSON line, and by the n2kd binary's
//! NMEA-0183 / AIS converters.

/// If `line` is a `-camel` analyzer record — `{"<pgnId>":{…}}`, where
/// the record is wrapped under the PGN's camelCase id and every field
/// key is a camelCase `id` — return that wrapper id. Bare `-json` opens
/// with one of the record's own top-level keys (`timestamp`/`prio`/…),
/// none of which is a PGN id, so it returns `None`.
///
/// Callers use this to decide whether to read field keys by `id`
/// (camel) or human `name` (bare). The substring helpers below read
/// `pgn`/`src`/etc. identically in either shape, since they scan the
/// whole line.
pub(crate) fn camel_wrapper_id(line: &str) -> Option<&str> {
    let s = line.trim_start().strip_prefix('{')?;
    let s = s.trim_start().strip_prefix('"')?;
    let key = &s[..s.find('"')?];
    match key {
        "timestamp" | "prio" | "src" | "dst" | "pgn" | "description" | "fields" => None,
        _ => Some(key),
    }
}

/// Pull `"<field>":<value>` out of `msg`. Returns the value as a
/// borrowed slice with any surrounding quotes stripped. Skips
/// whitespace between `:` and the value. Returns `None` if the
/// field isn't present.
pub fn value<'a>(msg: &'a str, field: &str) -> Option<&'a str> {
    let needle = format!("\"{field}\":");
    let mut start = msg.find(&needle)? + needle.len();
    let bytes = msg.as_bytes();
    while start < bytes.len() && (bytes[start] == b' ' || bytes[start] == b'\t') {
        start += 1;
    }
    if start >= bytes.len() {
        return None;
    }
    // Strings are delimited by `"`; numbers / true / false / null
    // end at the first `,` or `}`. Nested objects are bounded by
    // matching braces — `Lookup` shows up as `{"value":N,"name":"…"}`,
    // so we balance.
    let quoted = bytes[start] == b'"';
    if quoted {
        let body_start = start + 1;
        let mut idx = body_start;
        while idx < bytes.len() && bytes[idx] != b'"' {
            // Skip JSON escapes — but escape sequences are rare in
            // analyzer output (no embedded quotes inside strings we
            // care about). Keep it simple.
            if bytes[idx] == b'\\' && idx + 1 < bytes.len() {
                idx += 2;
                continue;
            }
            idx += 1;
        }
        return Some(&msg[body_start..idx]);
    }
    // Object? Walk the brace depth.
    if bytes[start] == b'{' {
        let mut depth = 0;
        let mut idx = start;
        while idx < bytes.len() {
            match bytes[idx] {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return Some(&msg[start..=idx]);
                    }
                }
                _ => {}
            }
            idx += 1;
        }
        return None;
    }
    // Array? Walk the bracket depth.
    if bytes[start] == b'[' {
        let mut depth = 0;
        let mut idx = start;
        while idx < bytes.len() {
            match bytes[idx] {
                b'[' => depth += 1,
                b']' => {
                    depth -= 1;
                    if depth == 0 {
                        return Some(&msg[start..=idx]);
                    }
                }
                _ => {}
            }
            idx += 1;
        }
        return None;
    }
    // Bare value: number / bool / null. End on `,` or `}` or whitespace.
    let mut idx = start;
    while idx < bytes.len() && !matches!(bytes[idx], b',' | b'}' | b' ' | b'\t' | b'\n' | b'\r') {
        idx += 1;
    }
    Some(&msg[start..idx])
}

/// Like [`value`], but parse the result as an `f64`. `null` / missing
/// → `None`.
pub fn number(msg: &str, field: &str) -> Option<f64> {
    let v = value(msg, field)?;
    if v == "null" || v.is_empty() {
        return None;
    }
    // canboat -nv puts lookup values inside `{"value":N,"name":"…"}`.
    if v.starts_with('{') {
        return number(v, "value");
    }
    v.trim().parse().ok()
}

/// Parse the field as a signed integer.
pub fn int(msg: &str, field: &str) -> Option<i64> {
    let v = value(msg, field)?;
    if v.starts_with('{') {
        return int(v, "value");
    }
    v.trim().parse().ok()
}

/// Resolve a lookup-style field — accepts either the bare string
/// (canboat default JSON) or the `{value,name}` shape from `-nv`.
/// Returns the integer code.
pub fn lookup_int(msg: &str, field: &str) -> Option<i64> {
    // Try the `{value, name}` form first.
    if let Some(obj) = value(msg, field) {
        if obj.starts_with('{') {
            return int(obj, "value");
        }
        // Bare integer?
        if let Ok(n) = obj.trim().parse() {
            return Some(n);
        }
    }
    None
}

/// Pull the rendered text of a field. For `-nv` lookup objects this
/// is the `name`; for plain JSON it's the bare value. Returns the
/// substring unchanged when the analyzer has already rendered it as
/// a string (e.g. dates / times in `-nv` mode are `{"value":N,"name":"…"}`
/// where the canonical text is in `name`).
pub fn value_or_name<'a>(msg: &'a str, field: &str) -> Option<&'a str> {
    let v = value(msg, field)?;
    if v.starts_with('{') {
        return value(v, "name");
    }
    Some(v)
}

/// Resolve `field`'s text for the schema-driven snapshot key.
///
/// For `-nv` lookup objects `{"value":N,"name":"X"}` we emit the
/// numeric `value` rather than the display `name`: composite keys
/// stay short and remain stable when canboat renames an enum label
/// (e.g. tweaking `"True"` → `"True (ground referenced to North)"`
/// would otherwise reshuffle cache entries between schema bumps).
///
/// Bare-string fields (non-`-nv` input, plain integers) pass through
/// verbatim — no numeric is available in that path. When a `-nv`
/// object is missing `value` for any reason, fall back to `name`.
pub fn lookup_text<'a>(msg: &'a str, field: &str) -> Option<&'a str> {
    let v = value(msg, field)?;
    if v.starts_with('{') {
        value(v, "value").or_else(|| value(v, "name"))
    } else {
        Some(v)
    }
}

/// Locate `"<field>":[...]` in `msg` and return the half-open byte
/// span covering the array (`[` through `]` inclusive). Used by the
/// per-iteration snapshot path to splice one element out of a
/// repeating-set `"list":[…]` while leaving the rest of the JSON
/// line byte-identical.
pub fn array_span(msg: &str, field: &str) -> Option<(usize, usize)> {
    let needle = format!("\"{field}\":");
    let mut start = msg.find(&needle)? + needle.len();
    let bytes = msg.as_bytes();
    while start < bytes.len() && (bytes[start] == b' ' || bytes[start] == b'\t') {
        start += 1;
    }
    if start >= bytes.len() || bytes[start] != b'[' {
        return None;
    }
    let mut depth = 0;
    let mut idx = start;
    while idx < bytes.len() {
        match bytes[idx] {
            b'[' => depth += 1,
            b']' => {
                depth -= 1;
                if depth == 0 {
                    return Some((start, idx + 1));
                }
            }
            b'"' => {
                // Skip string body so brackets inside strings don't
                // throw off the depth count. Honour `\"` escapes.
                idx += 1;
                while idx < bytes.len() && bytes[idx] != b'"' {
                    if bytes[idx] == b'\\' && idx + 1 < bytes.len() {
                        idx += 2;
                        continue;
                    }
                    idx += 1;
                }
            }
            _ => {}
        }
        idx += 1;
    }
    None
}

/// Split an array body like `[{e1},{e2},{e3}]` into its `{…}`
/// element substrings (without the outer brackets). Skips
/// non-object elements — every analyzer-emitted repeating set is
/// an array of objects.
pub fn split_object_array(arr: &str) -> Vec<&str> {
    let bytes = arr.as_bytes();
    if bytes.first() != Some(&b'[') {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut idx = 1;
    while idx < bytes.len() {
        match bytes[idx] {
            b' ' | b'\t' | b',' => idx += 1,
            b']' => break,
            b'{' => {
                let start = idx;
                let mut depth = 0;
                while idx < bytes.len() {
                    match bytes[idx] {
                        b'{' => depth += 1,
                        b'}' => {
                            depth -= 1;
                            if depth == 0 {
                                idx += 1;
                                break;
                            }
                        }
                        b'"' => {
                            idx += 1;
                            while idx < bytes.len() && bytes[idx] != b'"' {
                                if bytes[idx] == b'\\' && idx + 1 < bytes.len() {
                                    idx += 2;
                                    continue;
                                }
                                idx += 1;
                            }
                        }
                        _ => {}
                    }
                    idx += 1;
                }
                out.push(&arr[start..idx]);
            }
            _ => idx += 1,
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extracts_number() {
        let m = r#"{"pgn":127251,"src":7,"fields":{"Rate":-0.357703}}"#;
        assert_eq!(int(m, "pgn"), Some(127251));
        assert_eq!(int(m, "src"), Some(7));
        assert_eq!(number(m, "Rate"), Some(-0.357703));
    }

    #[test]
    fn extracts_string() {
        let m = r#"{"timestamp":"2026-01-01","fields":{"Reference":"Magnetic"}}"#;
        assert_eq!(value(m, "timestamp"), Some("2026-01-01"));
        assert_eq!(value(m, "Reference"), Some("Magnetic"));
    }

    #[test]
    fn handles_nv_lookup_object() {
        let m = r#"{"fields":{"Reference":{"value":1,"name":"Magnetic"}}}"#;
        assert_eq!(int(m, "Reference"), Some(1));
        assert_eq!(lookup_int(m, "Reference"), Some(1));
    }

    #[test]
    fn missing_returns_none() {
        let m = r#"{"foo":1}"#;
        assert_eq!(value(m, "bar"), None);
        assert_eq!(number(m, "bar"), None);
    }

    #[test]
    fn lookup_text_prefers_numeric_value() {
        // Composite snapshot keys use the raw numeric `value` rather
        // than the display `name` — short and stable across schema
        // label edits.
        let m = r#"{"Reference":{"value":1,"name":"Magnetic"}}"#;
        assert_eq!(lookup_text(m, "Reference"), Some("1"));
    }

    #[test]
    fn lookup_text_handles_key_true_objects() {
        // PGN 127501 Instance is `{"value":N,"key":true}` — same path.
        let m = r#"{"Instance":{"value":0,"key":true}}"#;
        assert_eq!(lookup_text(m, "Instance"), Some("0"));
    }

    #[test]
    fn lookup_text_falls_back_to_name_when_value_missing() {
        // Defensive: if a -nv object has no `value` for any reason,
        // fall back to the display name rather than dropping the key.
        let m = r#"{"Reference":{"name":"Magnetic"}}"#;
        assert_eq!(lookup_text(m, "Reference"), Some("Magnetic"));
    }

    #[test]
    fn lookup_text_returns_bare_value_when_no_object() {
        let m = r#"{"User ID":"244180106"}"#;
        assert_eq!(lookup_text(m, "User ID"), Some("244180106"));
    }

    #[test]
    fn array_span_locates_top_level_list() {
        let line = r#"{"pgn":130824,"fields":{"list":[{"a":1},{"a":2}]}}"#;
        let (s, e) = array_span(line, "list").expect("list found");
        assert_eq!(&line[s..e], "[{\"a\":1},{\"a\":2}]");
    }

    #[test]
    fn array_span_skips_brackets_inside_strings() {
        // Brackets inside JSON string literals must not throw off the
        // depth counter — the analyzer occasionally emits strings
        // containing `]` (e.g. a Free-Text Message with brackets).
        let line = r#"{"fields":{"Note":"a]b","list":[{"a":1}]}}"#;
        let (s, e) = array_span(line, "list").expect("list span");
        assert_eq!(&line[s..e], "[{\"a\":1}]");
    }

    #[test]
    fn split_object_array_breaks_top_level_objects_only() {
        let arr = r#"[{"a":1,"b":[2,3]},{"a":4},{"a":{"nested":5}}]"#;
        let elems = split_object_array(arr);
        assert_eq!(
            elems,
            vec![
                r#"{"a":1,"b":[2,3]}"#,
                r#"{"a":4}"#,
                r#"{"a":{"nested":5}}"#,
            ]
        );
    }

    #[test]
    fn split_object_array_handles_empty_and_whitespace() {
        assert!(split_object_array("[]").is_empty());
        assert_eq!(
            split_object_array("[ {\"a\":1} , {\"a\":2} ]"),
            vec!["{\"a\":1}", "{\"a\":2}"]
        );
    }
}
