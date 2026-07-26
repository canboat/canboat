# Quirk catalog

Every oddity of the current C generation pipeline that keel deliberately
reproduces to keep `canboat.xml` byte-identical (DESIGN.md migration step 1).
Collected here for review: each entry says where the quirk comes from, where
keel reproduces it, and a recommendation.

Recommendation legend:

- **keep** — harmless or load-bearing; changing it costs more than it gains.
- **cleanup** — fix *at or after switchover* in one deliberate "quirk
  cleanup" release, reviewed through `tools/contract.py` (mostly cosmetic
  XML churn downstream parsers won't notice, but bundle them so there is
  one diff to review, not ten).
- **fix-in-C** — a genuine defect in today's C source worth fixing *before*
  switchover, so the fix is proven by the C emitter and the golden test
  keeps guarding it.

## Output-format quirks (printf artifacts in analyzer-explain.c)

| # | Quirk | Origin | In keel | Recommendation |
|---|---|---|---|---|
| Q1 | `<Priority>` is indented 4 spaces where all sibling elements use 6 | `explainPGNXML`: `printf("    <Priority>...")` | `emit_xml.Emitter.pgn()` | cleanup |
| Q2 | `EnumFieldType`'s `Signed='...'` attribute is followed by a newline *inside* the tag, so the next attribute starts a line with one leading space | `explainFieldtypeXMLv2`: `printf(" Signed='%s'\n", ...)` | `emit_xml.Emitter.enum_fieldtype()` | cleanup |
| Q3 | `printXML()` escapes `& < > "` but **not** the apostrophe — yet the lookup sections put names in single-quoted attributes (`Name='...'`). A lookup name containing `'` would produce malformed XML | `printXML()` | `cformat.xml_escape()` | cleanup (until then: validator rule forbidding `'` in names) |
| Q4 | The `FieldTypes` and `PhysicalQuantities` sections are emitted with raw `printf` — no XML escaping at all. Safe only because no current text contains `& < >` | `explainFieldTypesXML`, `explainPhysicalQuantityXML` | `emit_xml.fieldtypes()` / `physical_quantities()` | cleanup (same guard rule until then) |
| Q5 | Attribute quoting is inconsistent: lookup sections use single quotes, `FieldType`/`PhysicalQuantity`/`MissingAttribute` use double quotes | different printf authors | both styles reproduced | cleanup |
| Q6 | Floats print as C `%g` (6 significant digits) / `%.15g`. Notably lossy for resolutions: 1/11 prints as `0.0909091`, so the XML/JSON never contains the exact wire resolution | all float printfs | `cformat::c_g()` / `c_15g()` via libc snprintf; `--float-style rust` emits Rust shortest-round-trip for comparison | **keep** — audited 2026-07: a wholesale switch to Rust formatting changes 266 lines and is a net *loss* (float noise like `6553.200000000001`, misleading full-decimal expansions like `18446744073709600000`). The one real improvement it exposes: resolutions could carry full precision (`0.09090909090909091` = exact 1/11) — worth doing *selectively for Resolution only* at a schema bump, not as a formatting switch |
| Q7 | `RangeMax` of an unsigned 64-bit resolution-1 field prints as the integer `18446744073709551615` instead of `%.15g` (which would print `1.84467440737096e+19`) | `explainPGNXML` special case | `emit_xml.Emitter.field()` | keep (it is the *more* correct output) |
| Q8 | The `<Copyright>` element and the leading XML comment embed the license with a trailing blank line before the closing tag | `printf("  <Copyright>" COPYRIGHT "\n</Copyright>\n")` | `emit_xml.header()` | keep |
| Q9 | The Actisense/iKonvert BEM documents include the `canboat.xsl` stylesheet PI and full header, although no stylesheet ships for them and the sections it styles are absent | `explainXML()` shares one header path | `emit_xml.emit()` | cleanup |

## Data-model quirks (fieldtype.c / pgn.h semantics)

| # | Quirk | Origin | In keel | Recommendation |
|---|---|---|---|---|
| Q10 | The tri-state `Bool` enum is `{Null=0, False=1, True=2}`, so C lowercase `false` aliases to **Null** and `true` to **False**. ISO_NAME's `.hasSign = false` is a live instance: almost certainly meant `False` (unsigned), but produces *no* `Signed` attribute at all | `fieldtype.h` enum + ISO_NAME initializer | `cheader._parse_value()` maps the aliasing; YAML stores the effective `Null` | **fix-in-C** — change to `False`; contract diff: ISO_NAME entries gain `Signed=false` (minor) |
| Q11 | Fieldtype-level explicit `.rangeMin`/`.rangeMax` initializers (MMSI: 0..999999999, FIELD_INDEX) are **inert**: `fillFieldType`'s `rangeMax == 0.0` guard routes any explicit value to the NaN branch. Dead configuration that looks load-bearing | `fieldtype.c:315` | `derive.fill_fieldtypes()` reproduces; values parked in `fieldtypes.yaml` as `rangeMin/rangeMax` | **fix-in-C** or delete the initializers — decide whether MMSI *should* advertise 0..999999999 (that would be a real XML change) or the config should go |
| Q12 | `min()`/`max()` macros are `x <= y ? x : y`: comparing NaN yields the *other* operand, so `fixupUnit()`'s rad clamp turns a NaN range into a concrete ±π bound. Whether any field currently exercises this is unverified — the port is faithful either way | `analyzer.h` macros + `fixupUnit()` | `derive.c_min()` / `c_max()` | keep (semantics), audit later |
| Q13 | A match field's `<Description>` is derived by scanning the lookup for the match value; when the lookup names no such value the element would be emitted *empty*. Also relies on calling a fieldtype-lookup enumerator through the pair-enumerator union member (works by ABI accident in C) | `explainPGNXML` + `filterPair()` | `emit_xml.match_description()` (implemented safely) | keep behavior; the C union pun dies with analyzer-explain.c at switchover |
| Q14 | `BitLengthField` (pointing at the length field of a variable BINARY field) is hardcoded to `order - 1` and only for fields whose *specific* type name is `BINARY` — not driven by any declared relationship | `explainPGNXML` | `emit_xml.Emitter.field()` | cleanup — should become an explicit authored reference (like `dynamicFieldLength`) |
| Q15 | An unknown transmission interval (C `interval == 0`) *silently adds* `Interval` to `<Missing>`; the authored YAML therefore never stores that entry (`model.missing_effective()`) | `fieldtype.c:486` | derived, by design | keep — this is derivation, not a defect |
| Q16 | Lookup fields whose type has `hasSign == Null` (e.g. BITLOOKUP) get NaN ranges, then a *fallback* emission branch prints `RangeMin 0` / `RangeMax 2^bits-1` anyway | `explainPGNXML` range else-branches | `emit_xml.Emitter.field()` | keep |
| Q17 | `fixupUnit()` SI branch clamps `rad` field ranges to ±π (signed) / 2π (unsigned) with the magic literal `3.1415926` (not M_PI) | `fieldtype.c fixupUnit()` | `derive.fixup_unit()` | keep the clamp; consider M_PI at cleanup (changes `%.15g` output!) |
| Q18 | Sentinel (`UnknownValue`/...) emission is suppressed for 64-bit fields, match fields, and non-TopOfRange roots; `reservedCount` is a gap computation between raw bit-max and rangeMax, capped by width — subtle but semantically intended | `fieldtype.c:448` + `explainPGNXML` | `derive.fill_field()` + emitter | keep — this is the sentinel model, documented in fieldtype.h |
| Q19 | The `unit` string doubles as the match encoding in C (`unit = "=275"`). No XML impact (`<Match>` element), but the pun shapes the C tables | pgn.h macros | dead in YAML (`match:` is first-class); generated pgn.h may keep it internally until the runtime is cleaned | cleanup (in generated-C form) |
| Q20 | `camelName`/`camelDescription` **presence** was used as a proxy for the `-camel` mode in three runtime output decisions (plain-text repeating suffix `_N` vs ` N`, JSON `{"camelId": ...}` wrapper, a dead fieldName fallback). Broke down for pinned ids: PGN 130846 emitted *wrapped* JSON even in plain mode, baked into four test fixtures | analyzer.c:1304/1494/1657 | **FIXED in C** (2026-07): all three now key on `showCamel`; the generated tables set camelName/camelDescription on every entry; the 130846 fixtures were corrected. Behavior change surface: only pinned ids in plain modes (1 field, 1 PGN — scanned) | done (fix-in-C) |

## Authored-override noise (converter reconciliation fallbacks)

Not quirks of the emitter but places where the YAML carries explicit values
because derivation cannot know them; worth reviewing whether each is *truly*
authored data or a C-side inconsistency:

- ~103 `pgns/*.yaml` files carry authored `rangeMin`/`rangeMax` — explicit
  initializers in pgn.h macros (e.g. `PGN_FIELD` 0..262143, the
  "all values valid" idiom) plus fields whose macro resolution is itself
  `%g`-lossy in the XML.
- 32 fields carry `specialValues` (the `SPECIAL_VALUES()` overrides).
- `database/lookups.order.yaml` preserves lookup.h definition order purely
  for the golden byte-diff; **dies at switchover** in favor of sorted order
  (one deliberate reordering diff, reviewed via contract.py).
- `variantOrder` on multi-entry PGNs is genuinely semantic (runtime match
  precedence) *including* fallback placement: the 0xE800 fallback sits first
  within 59392, but the 0x1EF00 fallback sits last within 126720. Review
  whether that asymmetry is intended.
