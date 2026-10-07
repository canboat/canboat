# Downstream survey: who uses CANboat (October 2026)

A survey of public GitHub projects for NMEA 2000, made on 7 October 2026 to find the
projects that build on CANboat: which part they use, which CANboat version, whether
they credit it, and how actively they are maintained.

> **This is a snapshot.** Versions, star counts and dates are as of 7 October 2026 and
> will drift. The current CANboat release at the time was **v8.3.0** (schema 2.7.0).
> `analyzer/pgns.xml` and `analyzer/pgns.json` were removed in May 2025, before v6.0.0,
> so anything still built from those files is older than v6.

The README's table of [other projects using the CANboat PGN definitions](../README.md#other-projects-using-the-canboat-pgn-definitions)
is the maintained, short version of this survey.

## Summary

- About **450 repositories** screened.
- About **20 projects** outside the README list generate their code from canboat.json
  or canboat.xml, or vendor it. About 8 more use canboatjs or ts-pgns outside the
  Signal K core, and about 15 run or package the CANboat tools.
- **Credit** is missing, or only in a file header, in ioBroker.nmea, sergei/sailvue,
  antipole2/JavaScript_pi, jpilet/anemomind, phobicdotno/actuisense, si6n/UCANLAB and
  MENIER/RaymarineAutoPilot (and the fgorina copies of it).
- **Old data** (pre-v6, or not updated for years): jpilet/anemomind (regenerated in June
  2026 from a pre-v6 `pgns.xml`), sergei/sailvue (fork last synced 2023), MO-RISE/marulc
  (2021), technocreatives/n2k-gen (2022), jxltom/nmea2000 (frozen at v6.0.0-alpha) and
  Sterwen-Technology/navigation_server (2012 XML base). mbj4668/n2k, in the README list,
  vendors **v4.12.0**. Several active Signal K plugins pin canboatjs 1.x or 2.x.
- **Suspected malware:** one repository re-uploads open-ships/n2k with a README that
  pushes a "Download" badge for a zip file kept in the repository. It is not named or
  linked here; it was not downloaded.

## 1. Projects that use CANboat

"Credit" is whether the README or docs credit CANboat: **yes**, **header only** (only in
a file header, a data folder or a NOTICE file), or **no**. "Last push" is the
repository's, as of 7 October 2026.

### 1a. Generated from, or vendoring, canboat.json / canboat.xml

| Repository | Language | ★ | Last push | How | CANboat version | Credit | Evidence |
|---|---|---|---|---|---|---|---|
| [boatkit-io/n2k](https://github.com/boatkit-io/n2k) | Go | 9 | 2026-10-05 | `cmd/pgngen` downloads canboat.json at a pinned commit and generates Go decoders and encoders | **v8.1.0**; has a `-check-canboat-update` flag | yes | `main.go`: `const canboatRelease = "v8.1.0"`; README "based on CANboat data" |
| [open-ships/n2k](https://github.com/open-ships/n2k) | Go | 3 | 2026-09-30 | Vendored canboat.json (`cmd/pgngen/schema.json`), about 600 typed PGNs, and a full bus node | **v7.1.0**, schema 2.5.0 (updated 2026-09-05) | yes | README "generated from the community-maintained canboat schema", plus an acknowledgements section. The vendored JSON has "canboat" unicode-escaped, so code search misses it. Inspired by boatkit. |
| [herostrat/nmea2000-rs](https://github.com/herostrat/nmea2000-rs) | Rust | 0 | 2026-02-28 | Build-time code generation from `data/canboat.json`; uses CANboat's test vectors | **v6.1.3** | yes | README "Built on the canboat project…" |
| [Sherin-SEF-AI/CanLab](https://github.com/Sherin-SEF-AI/CanLab) | Python | 84 | 2026-09-23 | `n2k_pgns.json` built from canboat.json by `tools/build_n2k_table.py --tag` | **v8.2.1** | yes | README licence section and `CANBOAT-NOTICE.txt` |
| [phobicdotno/pgntui](https://github.com/phobicdotno/pgntui) | Python | 2 | 2026-06-11 | Vendored canboat.json as `decode/pgns.json` | **v6.2.0** | yes | README "TUI for NMEA 2000 with canboat decoding" |
| [phobicdotno/actuisense](https://github.com/phobicdotno/actuisense) | Python | 1 | 2026-10-06 | `data/pgns.json` (PGN, name, fast-packet flag) generated from canboat master | **v6.2.0** | header only | `"_source": "canboat docs/canboat.json (Apache-2.0)"`; README silent |
| [knifter/canboat_explorer](https://github.com/knifter/canboat_explorer) | Python | 0 | 2026-06-26 | Vendored canboat.json, GUI explorer | **v6.1.8** | yes | README credits "CANboat Project PGN database" |
| [wdantuma/signalk-server-go](https://github.com/wdantuma/signalk-server-go) | Go | 3 | 2026-09-29 | Vendored `canboat/canboat.xml`, Go types generated from `canboat.xsd` | **v6.2.2** (updated 2026-06-27) | yes, but the README's "canboat" link points to the Signal K specification | `canboat/canboatgenerated.go` |
| [si6n/UCANLAB](https://github.com/si6n/UCANLAB) | Python | 0 | 2026-10-07 | Vendored canboat.json and canboat.xml, and a CANboat-derived DBC in `data/dbc/marine/canboat_refs/` | **v8.1.0** | header only (the data folder's README; the repository has no licence) | data README "canboat PGN Reference Definitions (v8.1.0)" |
| [ajsb85/nmea2000-pgn-reference](https://github.com/ajsb85/nmea2000-pgn-reference) | JavaScript | 1 | 2026-06-22 | Web PGN reference built from canboat.json | **v6.2.0** | yes | README states schema 2.3.0 / v6.2.0 |
| [antipole2/JavaScript_pi](https://github.com/antipole2/JavaScript_pi) | C++ (OpenCPN plugin) | 4 | 2026-09-08 | Ships `data/scripts/canboat.json`; `canboatAnalyzer.js` pipes frames through `analyzer -json` | **v6.1.3** | not in the README (the user guide was not checked) | `canboatAnalyzer.js` |
| [fkie-cad/maritime-dissector](https://github.com/fkie-cad/maritime-dissector) | Lua | 22 | 2026-03-24 | Wireshark dissector generated from canboat.json, downloaded at generation time | not pinned | yes | README "based on … CANBoat Documentation" |
| [Maps-Messaging/canbus_interface](https://github.com/Maps-Messaging/canbus_interface) (and Maps-Messaging/n2k) | Java | 0 / 1 | 2026-09-28 / 2026-04 | Parses `NMEA_database_1_300.xml`, CANboat's `sources/` file contributed by @elmue (in CANboat from 2025-05 to 2026-06) | the NMEA v1.300 subset, not canboat.json | in the docs ("derived from public CANboat XML metadata"), not the README | the XML file |
| [jpilet/anemomind](https://github.com/jpilet/anemomind) | C++ | 16 | 2026-07-14 | `PgnClasses.*` generated from a local CANboat checkout's `analyzer/pgns.xml` | **pre-v6** (regenerated 2026-06-17) | header only | generated header "index.js ../canboat/analyzer/pgns.xml" |
| [sergei/sailvue](https://github.com/sergei/sailvue) | C++ | 14 | 2026-09-24 | Submodule `n2k/canboat` from fork sergei/canboat (last push 2023-12); `InitCanBoat.c` | **≤ v5** (the submodule commit is not on the fork's default branch, so uncertain) | no | `.gitmodules` |
| [Sterwen-Technology/navigation_server](https://github.com/Sterwen-Technology/navigation_server) | Python | 3 | 2026-10-02 | `PGNDefns.N2kDfn.xml` "derived from Keversoft NMEA2000 Analyzer" (the 2012 XML, via OpenSkipper), heavily modified | 2012 base | in `doc/NMEA2000.md` only | doc: "XML … initially published by Keversoft … moved into canboat" |
| [jxltom/nmea2000](https://github.com/jxltom/nmea2000) | Python | 0 | 2025-12-08 | A non-fork copy of tomer-w/nmea2000 | **v6.0.0-alpha** (2025-05); its copied sync workflow does not run | yes | `nmea2000/canboat.json` |
| [MO-RISE/marulc](https://github.com/MO-RISE/marulc) | Python | 11 | 2023-09-14 | Vendored CANboat JSON | **2.0.0** (2021) | yes | README "identical to … CANBOAT" |
| [technocreatives/n2k-gen](https://github.com/technocreatives/n2k-gen) | Rust | 1 | 2024-08-08 | Code generation from CANboat's `pgns.xml` | 2022 (pre-v6) | yes | README |
| [MENIER/RaymarineAutoPilot](https://github.com/MENIER/RaymarineAutoPilot) (and fgorina/Autopilot2000, MeteoGNSS, Test_NMEA_2000) | C / C++ | 4 / 0 | 2023-04 / 2026-07 | CANboat PGN id strings in `pgnsToString.h` | about 2021–2023 | header only (`pgns_def.h`: "base of works: CANboat"); the fgorina copies keep only MENIER's header | `case 126208L: return PSTR("nmeaRequestGroupFunction")` |

Already in the README list before this survey:

| Repository | ★ | Last push | How | CANboat version | Notes |
|---|---|---|---|---|---|
| [aldas/go-nmea-client](https://github.com/aldas/go-nmea-client) | 16 | 2023-07-18 | canboat.json loaded at runtime | whichever file is supplied | inactive since 2023; newer schema fields may not be handled (uncertain) |
| [fard-draf/korri-n2k](https://github.com/fard-draf/korri-n2k) | 5 | 2026-09-23 | code generated from canboat.json | **v7.1.0** | credited |
| [mbj4668/n2k](https://github.com/mbj4668/n2k) | 7 | 2026-04-30 | vendored CANboat definitions | **v4.12.0** (last synced 2023-06; a 2026-04 commit only added local Maretron PGNs) | credited |
| [negrusti/NMEA2000-Analyzer](https://github.com/negrusti/NMEA2000-Analyzer) | 13 | 2026-07-10 | downloads `master/docs/canboat.json` at runtime | latest | credited |
| [tomer-w/nmea2000](https://github.com/tomer-w/nmea2000) | 21 | 2026-10-06 | vendored canboat.json, weekly sync workflow | **v8.1.0** | credited; also credits Smart Boat Innovations |
| [tomer-w/ha-nmea2000](https://github.com/tomer-w/ha-nmea2000) | 17 | 2026-10-05 | through nmea2000 | v8.1.0 | credited |
| [SmartBoatInnovations](https://github.com/SmartBoatInnovations) ha-smart2000usb / ha-smart2000esp | 10 / 5 | 2026-04-06 / 2025-11-15 | decoders generated from canboat.json | **v5.0.3** | credited only in `pgn_type.json`; asked in [ha-smart2000usb#10](https://github.com/SmartBoatInnovations/ha-smart2000usb/issues/10) |

### 1b. Using canboatjs or ts-pgns outside the Signal K core

| Repository | ★ | Last push | Dependency | Credit |
|---|---|---|---|---|
| [ioBroker/ioBroker.nmea](https://github.com/ioBroker/ioBroker.nmea) | 3 | 2026-10-06 | canboatjs ^3.20.0 | **no** |
| [NearlCrews/signalk-nmea2000-emitter-cannon](https://github.com/NearlCrews/signalk-nmea2000-emitter-cannon) | 1 | 2026-10-05 | canboatjs ^3.20, ts-pgns | yes |
| [jonaswitt/yacht-data-streams](https://github.com/jonaswitt/yacht-data-streams) | 1 | 2026-08-29 | canboatjs ^2.4.2, @canboat/pgns ^3 | yes |
| [MarcusWun/n2k-race-logger](https://github.com/MarcusWun/n2k-race-logger) | 0 | 2026-08-23 | canboatjs | yes |
| [BigaOSTeam/BigaOS](https://github.com/BigaOSTeam/BigaOS) (plugin) | 0 | 2026-07-11 | canboatjs ^2.0.0 | not found |
| [nparcher24/OpenHelm](https://github.com/nparcher24/OpenHelm) | 0 | 2026-05-15 | canboatjs ^3.14 | no |
| [twigmarine/nori-can](https://github.com/twigmarine/nori-can) | 2 | 2023-06 | @canboat/pgns ^2 | yes |
| [dirkwa/sensesp-n2k-gateway](https://github.com/dirkwa/sensesp-n2k-gateway) | 3 | 2026-08 | interoperates only: speaks canboatjs's `candump3` to `N2kIpGateway` | yes |

Signal K plugins that use canboatjs or ts-pgns, listed for completeness: sbender9/\*,
htool/\*, mairas/signalk-dst800-calibration-plugin,
johansolve/signalk-navico-autopilot-bridge, afds/\*, openwatersio/aiscast,
meri-imperiumi/signalk-autostate and others.

### 1c. Running the CANboat tools, or packaging them

- [wellenvogel/avnav](https://github.com/wellenvogel/avnav) (113★, active): a plugin
  reads the `n2kd` JSON stream on port 2598; documentation page "Canboat and SignalK".
  Credited.
- [openplotter/openplotter-can](https://github.com/openplotter/openplotter-can): clones
  and builds CANboat.
- [robotic-esp/canboat_vendor](https://github.com/robotic-esp/canboat_vendor): ROS 2
  wrapper, pinned to a CANboat commit of 2026-08-10 (just before v8.0.0-beta3); also
  published as ros2-gbp/canboat_vendor-release in the ROS 2 Jazzy distribution.
- [RISE-Maritime/porla-nmea](https://github.com/RISE-Maritime/porla-nmea) and
  [MO-RISE/porla-pontos](https://github.com/MO-RISE/porla-pontos): `analyzer --json` in
  pipelines.
- [RISE-Maritime/keelson](https://github.com/RISE-Maritime/keelson): re-implements the
  `actisense-serial` framing and the BASIC_STRING format; credited in code comments only.
- [mjcumming/cobalt-r6-surf](https://github.com/mjcumming/cobalt-r6-surf),
  [johba37/canboat-nmea2000-bridge](https://github.com/johba37/canboat-nmea2000-bridge),
  [lanceberc/polarize](https://github.com/lanceberc/polarize) (20★): use the analyzer.
- Packaging: MarkWalters-dev/aur and taotieren/aur-repo (AUR),
  opencpn-fedora-packaging/canboat (RPM spec, 2017), shantanoo-desai/meta-canboat
  (Yocto, CANboat 1.2.1, 2019).
- Older: iotfablab/n2kparser, violapaul/RegattaAnalysis, mglonnro/canboat-\*,
  sbender9/boat-scripts (archived), lkilpatrick/viamboat, chrismetcalf/canboat-ansible,
  Soups71/NEMO.

### 1d. Old or abandoned repositories that vendor CANboat data

- philmay57/N2KLib: pgns.xml v1.2.0, 2018.
- sbender9/N2KEncoder: 2016.
- sbender9/pgns: v6.0.1, deprecated in favour of ts-pgns.
- cbatson/nmea2k: Ruby, 2021.
- alexphredorg/nmea2000: its update script downloads `analyzer/pgns.json`, which no
  longer exists.
- timmathews/argo: 12★, 2022.
- yhmtmt/aws and yhmtmt/n2k_ngt1: vendored the C analyzer, about 2019.
- nowpor/canboat-clone: a 2019 copy of the CANboat repository.
- htool/RaymarineAPtoFakeNavicoAutoPilot: 28★, vendors canboatjs with @canboat/pgns ^1.0.4.
- cape-ookb/canboat-web: 2018.
- pkg40/greenboat-homeassistant: contains a copy of Smart Boat Innovations'
  smart2000esp.

### 1e. Hand-written decoders that cite CANboat as their reference

AlexAsplund/Vanchor (120★), csakos1/sailing-assistant (strong README credit),
Ranman86/NauticPinnace, sankeysoft/nmea_dashboard, frye/N2k-simulator,
constantineau/Agent_C4, erh/viam-chartplotter, sdoque/systems,
gjk4all/NMEA2000-Relay-Controller, taerugh/can-cpp, skrewby/nmea,
auvents-brave/BoatTools, lnx13/Actisense-Emulator.

## 2. Notable NMEA 2000 projects that do not use CANboat's definitions

| Repository | ★ | Notes |
|---|---|---|
| ttlappalainen/NMEA2000 (and its _esp32, _mcp, _Teensyx, … drivers) | 682 | Own hand-written PGN code; one comment links a CANboat PR |
| OpenCPN/OpenCPN | 1493 | Own NMEA 2000 handling, but copies CANboat's fast-packet PGN list (`comm_can_util.cpp`) and the NGT-1 startup command (`comm_drv_n2k_serial.cpp`: "Copied from canboat Project"); credited in the source |
| jvde-github/AIS-catcher | 788 | Own code; cites CANboat as a source |
| Wireshark `packet-nmea2000.c` | — | Hand-written dissector (2025); cites the CANboat docs and fkie-cad |
| ntpsec/gpsd `driver_nmea2000.c` | — | Hand-written; cites CANboat |
| wellenvogel/esp32-nmea2000 | 98 | Built on ttlappalainen |
| AK-Homberger/\* | 92 | Built on ttlappalainen |
| hatlabs/SH-ESP32-nmea2000-gateway | 33 | Built on ttlappalainen |
| TwoCanPlugIn | 10 | Own decoders, "inspired by Canboat"; supports the CANboat log format |
| OpenSkipper | 71 | Does use the 2012 Keversoft/CANboat XML, credited; inactive since 2023 |
| digitalyacht/iKonvert | — | Own SDK |
| sankeysoft/nmea_dashboard | — | Own Dart parsers |

## 3. Method and limits

- **Repository search:** nmea2000, nmea-2000, n2k, "NMEA 2000", canboat, actisense,
  ngt-1, nmea2k, "pgn decoder", "n2k gateway", "home assistant nmea", "n2k rust",
  "canboat python", and the topics nmea2000, nmea-2000, n2k, canboat and nmea2k. Each
  of the roughly 450 repositories found was checked for a README mention of canboat or
  Verruijt and for CANboat-like paths in its git tree.
- **Code search**, paced for GitHub's rate limit:
  - files and manifests: `filename:canboat.json`, and `canboat` in Cargo.toml, go.mod,
    pyproject.toml, package.json, READMEs, CMakeLists and Dockerfiles;
  - schema strings: `"CreatorCode" "SchemaVersion"`, `"Canboat NMEA2000 Analyzer"`, the
    `Lookup*Enumerations` names, `"docs/canboat.json"`, `"actisense-serial"`;
  - names and packages: `"Kees Verruijt"`, `@canboat/canboatjs`, `@canboat/ts-pgns`;
  - `canboat` and `"canboat.json"` per language;
  - distinctive PGN ids such as `nmeaRequestGroupFunction`, `seatalk1PilotMode` and
    `airmarAttitudeOffset`.
- **Verification:** versions come from the headers of vendored files, and update dates
  from those files' commit history.
- **Limits:**
  - GitHub code search does not index files over about 384 KB, so most vendored copies
    of canboat.json are invisible to it. Projects were found mainly through repository
    search and tree scans, so a renamed file in an uncredited repository can be missed.
    The unicode-escaped "canboat" in open-ships/n2k is another blind spot.
  - Code search covers default branches only, and the per-language "canboat" queries
    were noisy.
  - GitLab, Codeberg, and projects published only on PyPI or crates.io are not covered.
    In the crates.io index only canboat-rs and korri-n2k turned up.
  - Still uncertain: the JavaScript_pi user guide, the sailvue submodule commit, whether
    the Maps-Messaging server pulls in its NMEA 2000 module, and how well
    aldas/go-nmea-client handles the current schema.
