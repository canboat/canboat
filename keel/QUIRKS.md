# Why the generated output looks like this

Reference for anyone writing or changing an emitter. Every entry is a rule the
C pipeline established that keel still honours, or once honoured — the
non-obvious reasons `canboat.xml`, the C tables and the Rust schema come out
the way they do.

**This is no longer a migration document.** It began as the catalogue of
oddities keel reproduced to keep `canboat.xml` byte-identical through the v8
switchover; that gate retired with the switchover, and all twenty entries are
closed. What is left is the half that still describes live behaviour, and a
record of the half that does not.

It has already paid for itself twice over: porting `canboat-core/build.rs` to
derive its schema from the YAML (PR #774) needed **Q16** and **Q7** to get the
range rules right, and both are cited from that code. Sixteen places in the
Rust sources point here by entry number, so **the Q-numbers are stable
identifiers — never renumber them.**

## Live — rules that still shape today's output

| # | Rule | Origin | In keel | Status |
|---|---|---|---|---|
| Q6 | Floats print as C `%g` (6 significant digits) / `%.15g`. Notably lossy for resolutions: `1/16384` printed as `6.10352e-05`, `2^-38` as `3.63798e-12` | all float printfs | `cformat::c_g_roundtrip()` for Resolution, `c_15g()` for ranges | **Resolution done** (2026-07-26); rest **keep**. `<Resolution>` and the `EnumFieldType Resolution=` attribute now print at the fewest significant digits that round-trip, keeping `%g`'s exponent style. This was a real defect, not cosmetics: the old XML printed Resolution with `%g` but `RangeMax` with `%.15g`, so the bootstrap converter stored a **truncated resolution** (28 of them, rel. err up to 2.4e-6) plus a compensating explicit `rangeMax`. The exact values were recovered from the pre-switchover `pgn.h` (`1 / 16384.`, `POW2NEG(n)`, `2 * Pi / 65536`). Contract: minor (22 fields' decode moves in the 7th significant digit). The `--float-style c|rust` audit flag that existed to measure a wholesale switch has been **removed** now that Q6 is settled — it only ever emitted deliberately-worse artifacts (see "the other half" below) |
| Q7 | `RangeMax` of an unsigned 64-bit resolution-1 field prints as the integer `18446744073709551615` instead of `%.15g` (which would print `1.84467440737096e+19`) | `explainPGNXML` special case | `emit_xml.Emitter.field()` | keep (it is the *more* correct output) |

### Q6, the other half: RangeMin/RangeMax stay on `%.15g`

Measured 2026-07-26 by pointing `c_g_roundtrip` at the four range sites too.
882 lines move, 50 distinct transitions, and **both** halves are a regression:

- **21 transitions are cosmetic damage.** Round-trip picks the shortest
  representation, so clean integers become exponent notation: `RangeMin -90` →
  `-9e+01`, `RangeMax 180` → `1.8e+02`, `16380` → `1.638e+04`. (This same trap
  bit the Resolution change first — `10` came out as `1e+01` — which is why
  `c_g_roundtrip` now starts its search at `%g`'s own 6 digits and only ever
  *adds* digits. Two tests in `cformat` pin that.)
- **29 transitions expose float dirt that `%.15g` was usefully hiding.**
  `RangeMax 6553.2` → `6553.200000000001`, `RangeMin -3276.7` →
  `-3276.7000000000003`, `-838.8607` → `-838.8607000000001`. These bounds are
  computed as `raw * resolution`; the product is not the clean decimal a reader
  expects, and 15 digits rounds it back to the intended value.

So the 2026-07 verdict stands, now with a specific reason rather than a general
one: `%.15g` is the right formatter for ranges. It is lossy only in the last
couple of bits, and that lossiness is *load-bearing* — it absorbs the noise of
a floating-point multiply. Resolution was different because it is an authored
constant, not a computed product: there is a single exact intended value, and
6 digits could not express it.

| # | Rule | Origin | In keel | Status |
|---|---|---|---|---|
| Q12 | `min()`/`max()` macros are `x <= y ? x : y`: comparing NaN yields the *other* operand, so `fixupUnit()`'s rad clamp would turn a NaN range into a concrete ±π bound | `analyzer.h` macros + `fixupUnit()` | `derive::c_min()` / `c_max()` | **keep — audited 2026-07-26, the NaN branch is unexercised.** Reaching it needs a rad field whose fieldtype has `hasSign == Null`; all **76** rad fields carry an explicit `Signed` (4 FLOAT signed, 51 NUMBER unsigned, 21 NUMBER signed), so every emitted ±π/2π bound is a genuine clamp of a wider computed range — the FLOAT ones clamp down from FLT_MAX. The port stays faithful either way; re-check if a rad field is ever added on a sign-less type |
| Q13 | A match field's `<Description>` is derived by scanning the lookup for the match value; when the lookup names no such value the element would be emitted *empty*. The C also called a fieldtype-lookup enumerator through the pair-enumerator union member, which worked by ABI accident | `explainPGNXML` + `filterPair()` | `emit_xml::Emitter::match_description()` | **keep** (the empty-description behaviour) — **the union pun is gone**: verified 2026-07-26 that `analyzer/analyzer-explain.c` no longer exists and no `filterPair` remains anywhere in the C. keel's port never punned; it resolves the enumerator by kind |
| Q15 | An unknown transmission interval (C `interval == 0`) *silently adds* `Interval` to `<Missing>`; the authored YAML therefore never stores that entry (`model.missing_effective()`) | `fieldtype.c:486` | derived, by design | keep — this is derivation, not a defect |
| Q16 | Lookup fields whose type has `hasSign == Null` (e.g. BITLOOKUP) get NaN ranges, then a *fallback* emission branch prints `RangeMin 0` / `RangeMax 2^bits-1` anyway | `explainPGNXML` range else-branches | `emit_xml.Emitter.field()` | keep |
| Q18 | Sentinel (`UnknownValue`/...) emission is suppressed for 64-bit fields, match fields, and non-TopOfRange roots; `reservedCount` is a gap computation between raw bit-max and rangeMax, capped by width — subtle but semantically intended | `fieldtype.c:448` + `explainPGNXML` | `derive.fill_field()` + emitter | keep — this is the sentinel model, documented in fieldtype.h |

## Fixed — for the record

Closed during the 2026-07-26/27 review. Kept because "why did it ever look
like that?" is a fair question, and because a few carry a warning worth
heeding.

| # | Was | Resolution |
|---|---|---|
| Q1 | `<Priority>` indented 4 spaces, siblings 6 | emitted through the normal `xml_u(6, ...)` helper |
| Q2 | a newline *inside* the `EnumFieldType` tag after `Signed` | each `EnumFieldType` is one line |
| Q3 | `'` unescaped inside single-quoted attributes | retired by Q5 — every attribute is double-quoted, and `"` was already escaped |
| Q4 | `FieldTypes`/`PhysicalQuantities` emitted with no escaping at all | both route names, text and units through `xml_escape()` |
| Q5 | attribute quoting inconsistent between sections | every attribute keel emits is double-quoted |
| Q9 | BEM documents carried a `canboat.xsl` PI that ships for neither | the PI is emitted for the main and J1939 documents only |
| Q10 | C's lowercase `false` aliased to the tri-state `Null`, so ISO_NAME emitted no `Signed` at all | ISO_NAME carries `signed: false`; see the Q11 note below for the second-order effect |
| Q11 | fieldtype-level `rangeMin`/`rangeMax` initializers looked like dead config | they were **suppressors**, not dead — see below |
| Q14 | `BitLengthField` hardcoded to `order - 1`, keyed on the type name | authored as `bitLengthField: <id>`, validated by rule **R15** |
| Q17 | the rad clamp used the literal `3.1415926` | one full-precision `Pi` in `common.h`, used by both the C clamp and keel |
| Q19 | a match value was encoded into the `unit` string as `"=275"` | `struct Field` carries `hasMatchValue` / `matchValue` |
| Q20 | `camelName` presence used as a proxy for `-camel` mode | all three decisions key on `showCamel` |

Three of those left a warning behind:

- **Q11 — the initializers were suppressors.** An explicit fieldtype range hit
  `fillFieldType`'s `rangeMax == 0.0` guard and landed in the NaN branch, and
  NaN ranges are not emitted. So *deleting* them **added** three range pairs to
  `canboat.xml` (MMSI would have advertised `0..4294967292`, the raw 32-bit
  span, for a 9-digit identifier) and failed XSD validation — `canboat.xsd`'s
  `FieldType` has no `RangeMin`/`RangeMax` at all. The fix was upstream of all
  of it: **the `FieldTypes` section does not emit ranges.** A range only means
  something once bits, resolution and sign are concrete, i.e. per *field*.
- **Q17 — do not reintroduce a local π.** The project had three
  (`3.1415926`, `common.h`'s `3.141592654`, and `2 * Pi / 65536` baked into two
  126720 resolutions). `common.h` defines `Pi (3.141592653589793)`, spelled out
  because `M_PI` is not standard C and needs `_USE_MATH_DEFINES` on MSVC.
- **Q19 — `struct Field` is duplicated** in `analyzer/pgn.h` *and*
  `analyzer/pgn-j1939.h`. Both copies need any new member or the `-DJ1939`
  build breaks. Q19 also fixed a latent display bug: `print.c` appended the
  fake unit, so match fields printed `Report Type = 15 =15`.

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
- **`specialValues` — audited and pruned 2026-07-26.** Was 32 fields; now **8**.
  All 32 emitted `.reservedOverride` into `pgn-generated-data.h`, but only the 8 whose
  count differs from `reservedCountForSize()` do anything: `Device Instance
  Lower` (3 bits, auto 1 -> 0), `Device Instance Upper` (5 bits, auto 2 -> 0)
  and `Sequence Number` (2 bits, auto 1 -> 0), each appearing in several PGNs —
  all saying "no special values, every raw value is valid". The other 24 merely
  restated the auto count and have been deleted; `canboat.xml` and
  `canboat.json` are byte-identical across the removal, and the only change to
  `pgn-generated-data.h` is the 24 dropped clauses.

  Worth noting how this moved: an earlier pass found `129541
  rootOfSemiMajorAxis` load-bearing *despite* matching the auto count — dropping
  it lost the field's whole sentinel triple. The Q6 resolution fix (exact
  `POW2NEG(11)`, and the redundant authored range deleted with it) changed that
  derivation path, so it now derives its sentinels without help and could be
  pruned with the rest. Sentinels verified still present: 16777215 / 16777214 /
  16777213.

- ~~`database/lookups.order.yaml` preserved the generated lookup header's hand-authored definition order purely
  for the golden byte-diff; **dies at switchover** in favor of sorted order~~
  **done (2026-07-26)** — retired. `Database::ordered_lookups()` now sorts by
  name, and the manifest plus its six code sites (`model.rs`, `yamlio.rs`, and
  four in `edit.rs`, including the editor's undo snapshot of it) are gone.
  Emission order is derivable again, so adding an enumeration no longer means
  keeping a second file in step. One reordering diff in the generated lookup header and
  `canboat.xml`; contract-neutral (order is not part of the signature).
- ~~`variantOrder` fallback placement is asymmetric: the 0xE800 fallback sits
  first within 59392, but the 0x1EF00 fallback sits last within 126720~~
  **stale — no asymmetry left (checked 2026-07-26).** `variantOrder` is still
  genuinely semantic (runtime match precedence), but every one of the eight
  fallback-bearing groups now emits its fallback **first**: 59392, 61184,
  61440, 65280, 126208, 126720, 126976, 130816. PR #744 ("restore shadowed
  variants and dedupe Maretron/BEP PGNs") fixed 126720. Nothing to review.
