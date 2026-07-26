# Quirk catalog

Every oddity of the old C generation pipeline that keel reproduced in order to
keep `canboat.xml` byte-identical through the migration (DESIGN.md step 1).
Each entry says where the quirk came from, where keel reproduced it, and what
was decided.

Status legend:

- **keep** — harmless or load-bearing; changing it costs more than it gains.
- **done** — resolved; the entry is kept as a record of why the output looks
  the way it does.
- **open** — still to decide.

Reviewed with Kees on 2026-07-26, right after the v8 switchover merged
(PR #752). The cleanup entries landed together as one change, verified
contract-neutral by `tools/contract.py diff` (`No contract changes.`) — every
edit below is formatting or dead configuration, not semantics.

Note that the historical "fix-in-C" tier is gone: it meant "fix before
switchover so the C emitter and the golden test prove it". There is no C
source to fix any more and no independent oracle — `canboat.xml` *is* keel's
output. Those entries became ordinary database edits reviewed through
`tools/contract.py`.

## Output-format quirks (printf artifacts in analyzer-explain.c)

| # | Quirk | Origin | In keel | Status |
|---|---|---|---|---|
| Q1 | `<Priority>` was indented 4 spaces where all sibling elements use 6 | `explainPGNXML`: `printf("    <Priority>...")` | `emit_xml::Emitter::pgn()` | **done** — now emitted through the normal `xml_u(6, ...)` helper |
| Q2 | `EnumFieldType`'s `Signed` attribute was followed by a newline *inside* the tag, so the next attribute started a line with one leading space | `explainFieldtypeXMLv2`: `printf(" Signed='%s'\n", ...)` | `emit_xml::Emitter::enum_fieldtype()` | **done** — each `EnumFieldType` is one line |
| Q3 | `printXML()` escaped `& < > "` but **not** the apostrophe — yet the lookup sections put names in single-quoted attributes (`Name='...'`), so a lookup name containing `'` would have produced malformed XML | `printXML()` | `cformat::xml_escape()` | **done** via Q5 — with every attribute double-quoted, `"` (already escaped) is the only quote that can break one. The interim "validator rule forbidding `'` in names" this entry proposed was never written and is no longer needed |
| Q4 | The `FieldTypes` and `PhysicalQuantities` sections were emitted with raw `printf` — no XML escaping at all. Safe only because no current text contains `& < >` | `explainFieldTypesXML`, `explainPhysicalQuantityXML` | `emit_xml::Emitter::fieldtypes()` / `physical_quantities()` | **done** — both sections now route every name, element text and unit through `xml_escape()`. Byte-identical today (0 instances); the hazard is retired rather than deferred |
| Q5 | Attribute quoting was inconsistent: lookup sections used single quotes, `FieldType`/`PhysicalQuantity`/`MissingAttribute` double quotes | different printf authors | `emit_xml` lookup sections + `enum_fieldtype()` | **done** — every attribute keel emits is double-quoted. This also retires Q3: `"` was already escaped, and `'` needs no escaping inside a double-quoted attribute |
| Q6 | Floats print as C `%g` (6 significant digits) / `%.15g`. Notably lossy for resolutions: `1/16384` printed as `6.10352e-05`, `2^-38` as `3.63798e-12` | all float printfs | `cformat::c_g()` / `c_15g()`; **`c_g_roundtrip()` for Resolution** | **Resolution done** (2026-07-26); rest **keep**. `<Resolution>` and the `EnumFieldType Resolution=` attribute now print at the fewest significant digits that round-trip, keeping `%g`'s exponent style. This was a real defect, not cosmetics: the old XML printed Resolution with `%g` but `RangeMax` with `%.15g`, so the bootstrap converter stored a **truncated resolution** (28 of them, rel. err up to 2.4e-6) plus a compensating explicit `rangeMax`. The exact values were recovered from the pre-switchover `pgn.h` (`1 / 16384.`, `POW2NEG(n)`, `2 * Pi / 65536`). Contract: minor (22 fields' decode moves in the 7th significant digit). A wholesale switch to Rust float formatting remains a net loss — audited 2026-07: 266 lines of float noise like `6553.200000000001` |
| Q7 | `RangeMax` of an unsigned 64-bit resolution-1 field prints as the integer `18446744073709551615` instead of `%.15g` (which would print `1.84467440737096e+19`) | `explainPGNXML` special case | `emit_xml.Emitter.field()` | keep (it is the *more* correct output) |
| Q8 | The `<Copyright>` element and the leading XML comment embed the license with a trailing blank line before the closing tag | `printf("  <Copyright>" COPYRIGHT "\n</Copyright>\n")` | `emit_xml.header()` | keep |
| Q9 | The Actisense/iKonvert BEM documents carried the `canboat.xsl` stylesheet PI although no stylesheet ships for them and the sections it styles are absent | `explainXML()` shares one header path | `emit_xml::Emitter::header(styled)` | **done** — the PI is emitted for the main and J1939 documents only. The rest of the header (SchemaVersion/Version/Copyright) is kept: it identifies the document |

## Data-model quirks (fieldtype.c / pgn.h semantics)

| # | Quirk | Origin | In keel | Status |
|---|---|---|---|---|
| Q10 | The tri-state `Bool` enum is `{Null=0, False=1, True=2}`, so C lowercase `false` aliased to **Null** and `true` to **False**. ISO_NAME's `.hasSign = false` was a live instance: almost certainly meant `False` (unsigned), but produced *no* `Signed` attribute at all | `fieldtype.h` enum + ISO_NAME initializer | `database/fieldtypes.yaml` | **done** — ISO_NAME now carries `signed: false` and emits `<Signed>false</Signed>` (and `.hasSign = False` in `fieldtype-data.h`). Contract-neutral: `Signed` is not part of the FieldType contract signature. Beware the second-order effect this surfaced — see Q11 |
| Q11 | Fieldtype-level explicit `.rangeMin`/`.rangeMax` initializers (MMSI, FIELD_INDEX) never reach the XML: `fillFieldType`'s `rangeMax == 0.0` guard routes any explicit value to the NaN branch | `fieldtype.c:315` | `database/fieldtypes.yaml` + `emit_xml::Emitter::fieldtypes()` | **done** — initializers deleted. See the correction below: they were *suppressors*, not dead config |
| Q12 | `min()`/`max()` macros are `x <= y ? x : y`: comparing NaN yields the *other* operand, so `fixupUnit()`'s rad clamp turns a NaN range into a concrete ±π bound. Whether any field currently exercises this is unverified — the port is faithful either way | `analyzer.h` macros + `fixupUnit()` | `derive.c_min()` / `c_max()` | keep (semantics), audit later |
| Q13 | A match field's `<Description>` is derived by scanning the lookup for the match value; when the lookup names no such value the element would be emitted *empty*. Also relies on calling a fieldtype-lookup enumerator through the pair-enumerator union member (works by ABI accident in C) | `explainPGNXML` + `filterPair()` | `emit_xml.match_description()` (implemented safely) | keep behavior; the C union pun dies with analyzer-explain.c at switchover |
| Q14 | `BitLengthField` (pointing at the length field of a variable BINARY field) is hardcoded to `order - 1` and only for fields whose *specific* type name is `BINARY` — not driven by any declared relationship | `explainPGNXML` | `emit_xml::Emitter::field()` | **guarded** — rule **R15** now asserts the preceding field really is an unscaled integer bit count (all 8 instances pass; verified it trips when broken). It cannot catch a pointer at the *wrong* integer — an explicit authored reference, like `dynamicFieldLength`, is still the real fix |
| Q15 | An unknown transmission interval (C `interval == 0`) *silently adds* `Interval` to `<Missing>`; the authored YAML therefore never stores that entry (`model.missing_effective()`) | `fieldtype.c:486` | derived, by design | keep — this is derivation, not a defect |
| Q16 | Lookup fields whose type has `hasSign == Null` (e.g. BITLOOKUP) get NaN ranges, then a *fallback* emission branch prints `RangeMin 0` / `RangeMax 2^bits-1` anyway | `explainPGNXML` range else-branches | `emit_xml.Emitter.field()` | keep |
| Q17 | `fixupUnit()` SI branch clamped `rad` field ranges to ±π (signed) / 2π (unsigned) with the magic literal `3.1415926` (not M_PI) | `fieldtype.c fixupUnit()` | `derive::fixup_unit()` | **done** — now `std::f64::consts::PI`. Moved 101 emitted bounds in the 8th significant digit (±3.14159265358979, 6.28318530717959); contract-neutral |
| Q18 | Sentinel (`UnknownValue`/...) emission is suppressed for 64-bit fields, match fields, and non-TopOfRange roots; `reservedCount` is a gap computation between raw bit-max and rangeMax, capped by width — subtle but semantically intended | `fieldtype.c:448` + `explainPGNXML` | `derive.fill_field()` + emitter | keep — this is the sentinel model, documented in fieldtype.h |
| Q19 | The `unit` string doubled as the match encoding in C (`unit = "=275"`), 982 instances. No XML impact (`<Match>` is its own element), but the pun shaped the C tables | pgn.h macros | `struct Field` + `emit_c` | **done** — `struct Field` carries `hasMatchValue` / `matchValue`; the four sniff sites (fieldtype.c, pgn.c ×2, print.c) read the member. Fixed a latent display bug: print.c appends `unit` in plain text, so match fields printed `Report Type = 15 =15`. JSON was never affected. NB `struct Field` is duplicated in `pgn-j1939.h` — both copies need any new member |
| Q20 | `camelName`/`camelDescription` **presence** was used as a proxy for the `-camel` mode in three runtime output decisions (plain-text repeating suffix `_N` vs ` N`, JSON `{"camelId": ...}` wrapper, a dead fieldName fallback). Broke down for pinned ids: PGN 130846 emitted *wrapped* JSON even in plain mode, baked into four test fixtures | analyzer.c:1304/1494/1657 | **FIXED in C** (2026-07): all three now key on `showCamel`; the generated tables set camelName/camelDescription on every entry; the 130846 fixtures were corrected. Behavior change surface: only pinned ids in plain modes (1 field, 1 PGN — scanned) | done (fix-in-C) |

### Correction to Q11: those initializers were suppressors, not dead config

Q11 called the fieldtype-level `rangeMin`/`rangeMax` initializers "dead
configuration". They were not: **their presence suppressed range emission**.
`fillFieldType` routes an explicit value to the NaN branch, and NaN ranges are
not emitted — so deleting them *un-suppressed* a derived range and **added**
three range pairs to `canboat.xml` rather than removing config:

| FieldType | authored (never emitted) | derived once deleted |
|---|---|---|
| MMSI | 2000000 .. 999999999 | 0 .. 4294967292 — the raw 32-bit span, actively misleading for a 9-digit identifier |
| FIELD_INDEX | 0 .. 252 | 0 .. 252 — genuinely redundant |
| ISO_NAME | *(none)* | 0 .. 1.84467440737096e+19 — surfaced by the Q10 fix, since `hasSign: Null` → NaN range but `False` → a computed unsigned one |

Two further notes: Q11's stated MMSI range was wrong (it says `0..999999999`;
the initializer was `2000000..999999999`), and the new elements **failed XSD
validation** — `canboat.xsd`'s `FieldType` complexType has no `RangeMin` or
`RangeMax` at all.

The schema is the contract, so the resolution is upstream of all three rows:
**the `FieldTypes` section no longer emits ranges.** A range only means
something once bits, resolution and sign are concrete — i.e. per *field*,
where it has always been emitted. The C emitter had the same dead branch; it
stayed invisible because the only two fieldtypes that could reach it carried
suppressors. With the branch gone the initializers are *actually* dead, and
deleting them (plus the Q10 fix) leaves the XML byte-identical apart from
ISO_NAME's intended `<Signed>false</Signed>`.

## Authored-override noise (converter reconciliation fallbacks)

Not quirks of the emitter but places where the YAML carries explicit values
because derivation cannot know them; worth reviewing whether each is *truly*
authored data or a C-side inconsistency:

- **Authored `rangeMin`/`rangeMax` — audited 2026-07-26, re-measured after the
  Q6 fix.** 184 fields carry one.
  - **43 are redundant** — derivation reproduces them exactly. 25 already were;
    the Q6 resolution fix added **18** more (the nine `129541 gpsAlmanacData`
    almanac parameters, the eight `powerFactor` fields, and `65289 maretron010V`).
    All 43 have been deleted: 102 lines from 36 files, with `canboat.xml`
    byte-identical afterwards.
  - **141 are load-bearing genuine semantics** — a real-world valid range that
    is narrower than the raw width, which derivation cannot know: the `pgn`
    field of 59392/59904/60416 (24 bits carrying an 18-bit PGN number,
    `0..262143`), `TIME` fields (`0..86401`), `latitude`/`longitude`
    (`±90`/`±180` vs a derived `±214.7483647`), `temperatureOffset`
    (`±9.999` vs a derived `±32.767`), and the `FLOAT` fields of 126720, whose
    authored `±3.40282346638529e+38` is FLT_MAX where derivation produces an
    int32 span.

  **Correction:** an earlier pass through this section claimed 134 of these were
  "fallout from Q6" and would become derivable once Resolution carried full
  precision. That was wrong — it generalised from the one `powerFactor` example.
  Measured properly (strip all ranges, regenerate, diff, before and after the
  resolution fix) the true number is **18**. The great majority of the ranges
  that merely *differ* from the derived value are genuine bounds, not precision
  artifacts.
- **32 fields carry `specialValues`** (the `SPECIAL_VALUES()` overrides) —
  audited 2026-07-26. All 32 emit `.reservedOverride` into `pgn-data.h`, but
  only 9 do anything observable:
  - **8 encode a count that differs from `reservedCountForSize()`**, all saying
    "no special values, every raw value is valid": `Device Instance Lower`
    (3 bits, auto 1 → 0), `Device Instance Upper` (5 bits, auto 2 → 0),
    `Sequence Number` (2 bits, auto 1 → 0). Removing these would change how
    many top-of-range values the runtime treats as special.
  - **1 more, `129541 rootOfSemiMajorAxis`**, restates the auto count (3) yet
    is still load-bearing: dropping it loses the field's whole sentinel triple
    from the XML. Worth understanding before touching — the sentinel gate in
    `emit_xml` gets there through `reserved_count` *and* a non-NaN range, so
    presence appears to matter independently of value.
  - The remaining 23 restate the auto count and change neither the XML nor the
    C behaviour. Cosmetically removable; no reason to hurry.
- ~~`database/lookups.order.yaml` preserves lookup.h definition order purely
  for the golden byte-diff; **dies at switchover** in favor of sorted order~~
  **done (2026-07-26)** — retired. `Database::ordered_lookups()` now sorts by
  name, and the manifest plus its six code sites (`model.rs`, `yamlio.rs`, and
  four in `edit.rs`, including the editor's undo snapshot of it) are gone.
  Emission order is derivable again, so adding an enumeration no longer means
  keeping a second file in step. One reordering diff in `lookup.h` and
  `canboat.xml`; contract-neutral (order is not part of the signature).
- ~~`variantOrder` fallback placement is asymmetric: the 0xE800 fallback sits
  first within 59392, but the 0x1EF00 fallback sits last within 126720~~
  **stale — no asymmetry left (checked 2026-07-26).** `variantOrder` is still
  genuinely semantic (runtime match precedence), but every one of the eight
  fallback-bearing groups now emits its fallback **first**: 59392, 61184,
  61440, 65280, 126208, 126720, 126976, 130816. PR #744 ("restore shadowed
  variants and dedupe Maretron/BEP PGNs") fixed 126720. Nothing to review.
