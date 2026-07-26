# Merging canboat-rs into canboat (the keel line)

Status: **proposal / not started.** Companion to `DESIGN.md`. This describes how the
Rust runtime that currently lives in the separate [`canboat-rs`] repository folds into
this repository (the `refactor_pgn_database` / v8 line), now that `keel` has made the
PGN database Rust-authored and the C build already ships without a Rust toolchain.

Audience: maintainers deciding whether and how to co-locate the two codebases. No build
wiring is changed by this document.

[`canboat-rs`]: https://github.com/canboat/canboat-rs

---

## 1. Context

Two Rust codebases now exist against the same NMEA 2000 database:

- **`keel`** (this repo, `keel/`, ~6.1k LoC, deps: `yaml-rust2` only). *Authoring/codegen.*
  Owns the YAML database under `database/`. Generates `analyzer/pgn-data.h`,
  `lookup.h`, `fieldtype-data.h`, `physicalquantity-data.h` and `canboat.xml`, all
  **committed**, so the C analyzer builds with no cargo. `canboat.xml` → `canboat.json`
  via the unchanged XSLT chain.

- **`canboat-rs`** (separate repo, ~29k LoC hand-written across 7 crates). *Runtime.*
  Live decode/encode, snapshot, reassembly, an `n2kd` replacement, a server, and a TUI.
  Its schema (`canboat-core`) is built at compile time from a **vendored copy** of
  `canboat.json` plus a hand-maintained `synthetic-pgns.json`.

The v8 refactor already solved the hard half of any merge: **the rust-free C build is
not a thing to design, it is a thing to inherit** (keel commits its generated C; cargo
is only needed to *regenerate*). What remains is to relate the two Rust models cleanly.

## 2. Goals and non-negotiables

1. **A plain `make` of the C tools never invokes cargo.** (Already true for keel; must
   stay true after the runtime crates arrive.)
2. **One source of truth for the database, with no vendoring lag.** Today the runtime is
   several derivation hops downstream of the YAML (see §3); that gap is the entire reason
   the sync machinery exists.
3. **A path to retire the aging C `n2kd`** in favour of the Rust one, gated on measured
   parity — never a flag-day swap.
4. **keel stays dependency-thin.** It is cargo-shimmed *inside* `analyzer/Makefile`;
   dragging the runtime dependency tree (tokio, socketcan, ratatui) into that shim would
   wreck first-build time for C-only contributors.

## 3. Current state: where drift lives

The runtime's schema is derived from the YAML the long way around:

```
database/*.yaml
  └─ keel ────────────────► analyzer/*-data.h        (committed; C build)
  └─ keel ────────────────► canboat.xml
                              └─ xsltproc ──────────► docs/canboat.json
                                                        └─ (vendored copy in
                                                            canboat-core/data/)
                                                            └─ canboat-core/build.rs
                                                                └─ src/schema_data.rs
                                                                   (include!d)
```

Everything on the `canboat-rs` side of that chain exists *only* to bridge the gap:

- `crates/canboat-core/data/canboat.json` — vendored schema copy.
- `crates/canboat-core/data/synthetic-pgns.json` — hand-maintained mirror of the
  `CANBOAT_BEM` (0x40000+) PGNs that `analyzer/pgn.h` defines directly and that therefore
  never appear in `canboat.json`. Its own header comment says *"Keep in sync with
  canboat/analyzer/pgn.h's BEM definitions."*
- A `CANBOAT_REF` pin, the `canboat-sync.yml` weekly cron, and a CI equality gate.
- Most of `crates/canboat-core/build.rs` (~1.36k lines): parse `canboat.json` +
  `synthetic-pgns.json`, emit static `FieldInfo`/`PgnInfo`/lookup tables and the id-keyed
  constant modules (`pub mod pgn`, `pub mod field`) into `schema_data.rs`.

`keel/src/model.rs` and `canboat-schema/src/lib.rs` are, at this point, **two independent
Rust models of the same database** kept in step by that pipeline.

## 4. Target architecture

Co-locate as one workspace with **strict dependency layering** and **one generator**:

```
canboat-primitives   (thin: field-type & packet-type enums, physical-quantity/unit
     │                 tables, bit extraction primitives — NO heavy deps)
     ├── keel                    (yaml + emit_*  — authoring/codegen)
     └── canboat-schema          (static-table types: FieldInfo, PgnInfo, FieldRef,
          │                        Lookup*Table — the shape keel emits & core consumes)
          └── canboat-core       (decode/encode/snapshot/reassembly — runtime, light deps)
               └── canboat-io / canboat-tokio / canboat (server+tui) / n2kd
                       ▲
                       └── ALL heavy deps (tokio, socketcan, ratatui) live ONLY here
```

Invariant: **nothing keel depends on may reach the heavy layer.** keel depends on
`canboat-primitives` (and optionally `canboat-schema` for the emit target types) and
nothing below it.

The generation principle mirrors what keel already does for C: **keel is the single
generator; every consumer builds from committed output without needing the generator.**

| Consumer            | Reads (committed)                   | Needs keel toolchain? |
|---------------------|-------------------------------------|-----------------------|
| C analyzer & tools  | `analyzer/*-data.h`                 | No (regen only)       |
| docs (`canboat.json`)| `canboat.xml` → XSLT               | No (regen only)       |
| Rust runtime        | generated Rust schema tables        | No (regen only)       |
| keel itself         | `database/*.yaml`                   | — it is the tool      |

## 5. The core move: `keel` gains `emit_rust.rs`

keel already has `emit_c.rs`, `emit_xml.rs`, `emit_text.rs`, wired through
`generate.rs::emit_artifacts()` and the `generate` / `emit` subcommands. Add
`emit_rust.rs` beside them.

It emits, directly from the in-memory `keel::model::Database`, the exact artifact that
`canboat-core/build.rs` produces today from `canboat.json`:

- the static `&[FieldInfo]` / `&PgnInfo` tables (canboat-schema types),
- the `by_value` / `by_bit` / `by_pair` lookup tables,
- the id-keyed constant modules `pub mod pgn { pub static WIND_DATA: &PgnInfo … }` and
  `pub mod field { pub mod wind_data { pub const WIND_ANGLE: FieldRef … } }`.

This is a **port, not a rewrite**: the table-writing back half of
`canboat-core/build.rs` (the `write!(out, "static {ident}:&[FieldInfo]=&[…")` machinery
and `emit_id_constants`) already knows how to serialize these structures. It moves into
keel; only its *front end* changes — from "parse JSON into `RawPgn`/`RawField`" to "walk
`keel::model`". keel's model is a strictly richer source than `canboat.json`, so no
information is lost.

Output lands as a committed `schema_data.rs` (path TBD, e.g.
`crates/canboat-core/src/schema_data.rs`), `include!`d exactly as today. `canboat-core`
then has **no build script** and **no vendored data**.

Add it to `generate.rs::emit_artifacts()` alongside the C/XML outputs so a single
`keel generate` (or `make generated`) refreshes C, XML→JSON, *and* Rust together, and the
same git-diff CI gate covers all three.

## 6. What gets deleted

On the `canboat-rs` side, once §5 lands:

- `crates/canboat-core/data/canboat.json` — vendored copy.
- `crates/canboat-core/data/synthetic-pgns.json` — see §7.
- `crates/canboat-core/build.rs` — replaced by keel's `emit_rust.rs`.
- The `CANBOAT_REF` pin, `.github/workflows/canboat-sync.yml` weekly cron, and the
  schema-equality CI gate — nothing to sync *to* once the source is in-repo.
- The `serde` / `serde_json` build-dependencies of `canboat-core` (only used to parse the
  vendored JSON).

`canboat-schema` stays — it is the shared *type* layer (`FieldInfo`, `PgnInfo`,
`FieldRef`, `Lookup*Table`) that keel emits into and `canboat-core` consumes. It may move
up next to / into `canboat-primitives`.

## 7. Synthetic PGNs fold into keel

`synthetic-pgns.json` is a manual mirror of the `CANBOAT_BEM` PGNs that `analyzer/pgn.h`
defines directly. In the merged world **keel owns `pgn.h` from the YAML**, so those PGNs
are already in keel's model (or belong in the YAML). `emit_rust.rs` emits them into the
Rust tables the same way `emit_c.rs` emits them into `pgn-data.h` — one source, one
generator, the manual mirror gone. This removes an entire class of "I edited pgn.h but
forgot the JSON mirror" drift.

## 8. Sharing code: converge the *generator*, not the *models*

The tempting-but-wrong version of "share code with canboat-core" is to merge
`keel::model` with `canboat-core`'s decoder. Resist it:

- **keel's model is authoring-shaped.** It carries `res_*` derived fields, tri-state
  `Option<bool>`, and byte-for-byte-with-`analyzer-explain.c` emission quirks. It decodes
  only to *validate sample data*.
- **canboat-core's model is runtime-shaped.** `smallvec`/`itoa`/`indexmap`, snapshotting,
  the two-schema SI/Metric split, and a hot decode path (observed live at ~189 frames/s on
  the reference workload). Authoring baggage would tax it; runtime tuning would bloat keel
  and break its minimal-dep rule.

So share at two well-defined seams only:

1. **The generated tables** (keel → canboat-core), per §5. High value, low risk.
2. **A thin `canboat-primitives` crate** — the field-type & packet-type enums, the
   physical-quantity / unit definitions, and possibly the bit-extraction primitives
   (`keel/src/bits.rs` ↔ `canboat-core` decode). Both keel and canboat-core depend on it;
   neither inherits the other's deps.

Do (2) incrementally: share the **enum/const definitions first** (cheap, unambiguous —
today `PacketType`/`FieldType` are duplicated between `keel::model` and `canboat-schema`).
Only merge the bit-extraction *loop* if a benchmark says the shared version keeps
canboat-core's hot path intact. keel validates samples; canboat-core decodes live frames —
same math, different performance envelope.

## 9. Dependency hygiene (the rule that makes or breaks it)

`keel` is built by a cargo shim *inside* `analyzer/Makefile`. Its dependency closure must
stay tiny. Concretely, in the merged workspace:

- keel → `canboat-primitives` (+ maybe `canboat-schema`). **No path to** `canboat-core`,
  `canboat-io`, `canboat-tokio`, `canboat`, or `n2kd`.
- A CI check (e.g. `cargo tree -p keel` asserted against an allowlist, or a
  `cargo-deny`/`cargo-hakari` rule) that fails if keel's closure ever grows tokio/
  socketcan/ratatui. This invariant is load-bearing for goal #1 and should be enforced,
  not trusted.

## 10. Build wiring

- **Top-level `Makefile`:** `SUBDIRS` and the default `all`/`compile` targets stay
  C-only. Add an **opt-in** `rust:` target (`cd <rust-root> && cargo build --release`)
  that is *not* a dependency of `all`. C-only contributors and packagers never touch it.
- **`analyzer/Makefile`:** the existing `keel-generate` phony target gains the Rust output
  (via keel's extended `emit_artifacts`), so `make generated` refreshes C + XML/JSON +
  Rust and the single git-diff gate covers all three.
- **CI:** keep `canboat-rs`'s existing Rust jobs (Build & Test on the three OSes, Clippy,
  MSRV, Rustfmt). Delete `canboat-sync.yml` and the schema-equality gate (§6). Add the
  keel-closure guard (§9). Release: the C release path (`rel/`) stays Rust-free; Rust
  artifacts build in their own job.

## 11. Retiring C `n2kd`

Unchanged from prior discussion, and deliberately *not* part of the initial merge:

1. Land the Rust `n2kd` alongside C `n2kd`; keep C `n2kd` in `SUBDIRS`.
2. Prove parity with `canboat-rs`'s harness (`crates/n2kd/parity/run-parity.sh`) — a live
   C-vs-Rust diff across all five ports — green **on the Pi**, on real traffic.
3. Only then drop the C `n2kd` subdir. Retirement is a separate PR with its own decision.

## 12. Migration steps (each independently shippable)

1. **Move the crates in.** Land `canboat-rs`'s workspace under a Rust root in this repo,
   still vendoring `canboat.json` (no behaviour change). Add the opt-in `make rust` target.
   Verify: C build untouched; `cargo test` green; keel dep-closure guard passes.
2. **`emit_rust.rs`.** Port the table/id-constant back end of `canboat-core/build.rs` into
   keel. Wire into `emit_artifacts`. **Golden test:** the keel-emitted `schema_data.rs`
   must be byte-identical to today's build.rs output for the current database (same
   discipline keel used for the XML switchover). Ship only when byte-identical.
3. **Cut the cord.** Delete `canboat-core/build.rs`, `data/canboat.json`,
   `synthetic-pgns.json`, `CANBOAT_REF`, `canboat-sync.yml`, the equality gate, and
   canboat-core's serde build-deps. canboat-core now reads only the committed generated
   Rust. Verify: full runtime test suite + n2kd parity harness still green.
4. **Extract `canboat-primitives`.** Deduplicate the field-type/packet-type enums and
   unit tables shared by keel and canboat-schema/core. Deps-only refactor; no schema change.
5. **(Later, separate)** Rust `n2kd` retirement per §11.

Steps 1–3 deliver goal #2 (single source, zero drift) and preserve goals #1 and #4.
Step 4 is cleanup. Step 5 is a distinct project.

## 13. Risks and open questions

- **Byte-parity of `emit_rust.rs`.** The build.rs output encodes camelization, SI/Metric
  handling, `FieldRef` layout and dedup rules (`emit_id_constants` skips empty/duplicate
  modules). The golden test in step 2 is the gate; budget time for reconciling subtle
  differences between "parsed from JSON" and "walked from keel model".
- **Synthetic PGNs' authoritative home.** Confirm every `CANBOAT_BEM` PGN in
  `synthetic-pgns.json` is representable in the YAML / keel model, or decide they live in a
  keel-internal table. Either way keel must emit them to *both* C and Rust.
- **Rust root layout.** `rust/` subtree vs. crates at repo root. Prefer a subtree so the C
  tree is visually undisturbed and `.github`/`Makefile` ownership is obvious.
- **MSRV / edition skew.** keel is edition 2024; canboat-rs is edition 2024, MSRV 1.88.
  A shared `canboat-primitives` must satisfy the stricter of the two.
- **Version & release cadence.** canboat-rs uses release-please (`0.5.0`) independent of
  canboat's `8.0.0-alpha`. Decide whether the Rust crates version with the repo or keep an
  independent line.
