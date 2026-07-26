# Changelog

## [0.6.0](https://github.com/canboat/canboat-rs/compare/v0.5.0...v0.6.0) (2026-07-10)


### Features

* **analyzer:** version banner with commit + units, and -si flag ([57c6872](https://github.com/canboat/canboat-rs/commit/57c6872f5e75b12d40b8346e746cb0f0adbe73dc))
* **canboat:** add `canboat` binary with the `convert` subcommand ([f3a1f78](https://github.com/canboat/canboat-rs/commit/f3a1f78c224d48f06af689666916d7d47fb5800c))
* **canboat:** add `interface` subcommand over the unified device layer ([85eacb0](https://github.com/canboat/canboat-rs/commit/85eacb04d06b8ee1282f1a0eae0a5d9fa887dd74))
* **canboat:** add `replay` subcommand + shim, retire the replay crate ([1b23267](https://github.com/canboat/canboat-rs/commit/1b232677cb90fece7c7a4688b9f4be9f98f74ff1))
* **canboat:** argv[0] shims + install-shims for the retired tool names ([95eb414](https://github.com/canboat/canboat-rs/commit/95eb414c2b106e5b8f6b96dbb236c7e561803557))
* **canboat:** close convert/interface flag gaps for legacy shims ([d9d81f3](https://github.com/canboat/canboat-rs/commit/d9d81f375d836b3fc16ffe58151c60fdad516733))
* **canboat:** drop the nif2analyzer shim ([ef3a0e9](https://github.com/canboat/canboat-rs/commit/ef3a0e9d2e492f507323d864f74cca258b2e284c))
* **canboat:** fold n2kd into `canboat`, retire the standalone bin ([0fabfa5](https://github.com/canboat/canboat-rs/commit/0fabfa562145fa5c1b3a57dc9f8b25875922445b))
* **canboat:** move canboat-pipeline in as the `server` subcommand ([f2b2d99](https://github.com/canboat/canboat-rs/commit/f2b2d99e0ef2f9b1382b92d7ace8e1f66e3deefa))
* **canboat:** move canboat-tui in as the `tui` subcommand ([c81b912](https://github.com/canboat/canboat-rs/commit/c81b912480b04cef556e39e9466325ec62e15f4c))
* **convert:** add YDWG02, Actisense ASCII, and Actisense EBL output ([2cc390d](https://github.com/canboat/canboat-rs/commit/2cc390d78d9d9fab836aa65ac1e2d5d9589d45c8))
* **convert:** default --to to json instead of plain ([5d616a9](https://github.com/canboat/canboat-rs/commit/5d616a94bbd0f389a534c4b83c67c5d52c3be41f))
* **convert:** surface supported input/output formats in --help ([6feb9ba](https://github.com/canboat/canboat-rs/commit/6feb9ba097f76ae943677d2dea0287ead5f148eb))
* **core:** accept PGN/field constants in the encoder ([429a040](https://github.com/canboat/canboat-rs/commit/429a0409fb168929a5b28fce4ae2a027794eb814))
* **core:** consume camelCase analyzer JSON; dispatch NMEA 0183 by variant id ([df4d901](https://github.com/canboat/canboat-rs/commit/df4d901e53632ef61b440dbaeb8449bbe30e255c))
* **core:** DecodedPgn::iso_name — the ISO NAME from decoded fields ([52eca80](https://github.com/canboat/canboat-rs/commit/52eca80bdbb267ba0a1231cead2ad1bc0aa32e8d))
* **core:** encode all self-contained field datatypes ([87dd92b](https://github.com/canboat/canboat-rs/commit/87dd92b2c969076e1b0286459eeaec700d40ec11))
* **core:** generate compile-time PGN/field id constants ([6b35420](https://github.com/canboat/canboat-rs/commit/6b354209a972b038d44f6b6e19838e7e69041bd5))
* **core:** normalize emitted timestamps to canonical ISO-8601 UTC ([30c9e46](https://github.com/canboat/canboat-rs/commit/30c9e46cd81e9fc21b904f344e369d8476c91029))
* **core:** SI/Metric unit schemas, message encoder, and format-message ([4f89109](https://github.com/canboat/canboat-rs/commit/4f89109a2e05930011b383140d2cb76378fd3297))
* **io:** read Actisense .ebl captures as input ([08f08eb](https://github.com/canboat/canboat-rs/commit/08f08eb3dacdd21cf5f06a373c53eace4461ab15))
* **n2kd:** apply the per-NAME NMEA 0183 filter in the daemon ([919e6d9](https://github.com/canboat/canboat-rs/commit/919e6d9ecc8b3db0fe252489b63ae4316974955a))
* **n2kd:** consume the analyzer units banner; convert per requested unit ([8b2588f](https://github.com/canboat/canboat-rs/commit/8b2588fc238e4f3e17e3220077318941dbf2adca))
* **n2kd:** filter control channel — apply Set frames, emit Reports ([8f26e29](https://github.com/canboat/canboat-rs/commit/8f26e29f89f09c3955986a812b842d828e751d34))
* **server:** add --quirk wmm and encode all self-contained field datatypes ([4822779](https://github.com/canboat/canboat-rs/commit/4822779553d438e142c9dd3f8d3fe38d60912cf8))
* **server:** add --quirk wmm, process quirks from decoded PGNs ([d3bdbab](https://github.com/canboat/canboat-rs/commit/d3bdbab18e3028dd315e885524d6ffdbe4f06dd3))
* **server:** add --si to decode JSON ports in SI base units ([80eac72](https://github.com/canboat/canboat-rs/commit/80eac72b4f0f007dca6d47e28f2a4b47b04efce1))
* **server:** dedicated bidirectional NMEA 0183 filter control port ([60d89cd](https://github.com/canboat/canboat-rs/commit/60d89cdc1b5b9e19c405c21286b60257fc368795))
* **server:** emit version/units banner on snapshot + analyzer ports ([59b34c5](https://github.com/canboat/canboat-rs/commit/59b34c50c5fca7e2f8a10dbba6736d5690c1e187))
* **server:** key camelCase snapshot by pgn id, unwrap records ([131a6b6](https://github.com/canboat/canboat-rs/commit/131a6b609bbc4d0721995b5c5c51f017713e3610))
* **server:** own PGN-rate overrides; add TUI Overrides view ([326a90c](https://github.com/canboat/canboat-rs/commit/326a90ce204e34f0b7dd87fcfe51f1a8602d8590))
* **server:** own PGN-rate overrides; add TUI Overrides view ([54de770](https://github.com/canboat/canboat-rs/commit/54de7709b40ce7ba7d0daac9801ddcbfbb5be8cf))
* **tui:** consume server --camel by canonicalizing records at ingestion ([b2781e5](https://github.com/canboat/canboat-rs/commit/b2781e54766ecc1a160c1eb9dd89eea68cefd261))
* **tui:** forget persisted overrides the bus NAKs ([7e7ad16](https://github.com/canboat/canboat-rs/commit/7e7ad16a0d3b480bd37263b063da6944d9f08af3))
* **wire:** declare + convert units across the handshake ([fd5f16d](https://github.com/canboat/canboat-rs/commit/fd5f16d383b1966497244be3ba4d82eadb0813d6))


### Bug Fixes

* **core:** key JSON message wrapper on -camel mode, drop timestamp bridge ([333987a](https://github.com/canboat/canboat-rs/commit/333987a5dc3619edfbb3d5478ad3ecb893c08a2b))
* **io:** only warn on FAST-mislabel when no PGN variant fits in 8 bytes ([838fd40](https://github.com/canboat/canboat-rs/commit/838fd40f669c93bcbe0f3a3ba25d5917de88fa28))
* **server:** emit coalesced messages on the raw port, not fragments ([69cd3f8](https://github.com/canboat/canboat-rs/commit/69cd3f85db0944a460f4ff104a975ea918d65cd7))

## [0.5.0](https://github.com/canboat/canboat-rs/compare/v0.4.0...v0.5.0) (2026-07-06)


### Features

* **io:** read Navico .nif and SocketCAN .pcap captures ([5e5ccfc](https://github.com/canboat/canboat-rs/commit/5e5ccfcb6db4565880d591c97193d3cc5fed8403))
* **io:** read Navico .nif and SocketCAN .pcap captures ([9821de7](https://github.com/canboat/canboat-rs/commit/9821de7714d1ff7188b3ffba14337b860ce59074))
* **tui:** Turbo Pascal UI — menus, file browser, async save, faster timeline ([787ea93](https://github.com/canboat/canboat-rs/commit/787ea938f618447ff02a6cfbda207e82e3d23549))
* **tui:** update UI to Turbo Pascal like — menus, file browser, async save, perf & fixes ([4b7cc18](https://github.com/canboat/canboat-rs/commit/4b7cc1891f77766c60534485eac413b5ced09158))


### Bug Fixes

* **core:** decode STRING_LAU with 0xff encoding byte as an empty field ([987fe13](https://github.com/canboat/canboat-rs/commit/987fe135bb8231f827f3c96fed772b2967b8d43a))
* **core:** decode STRING_LAU with 0xff encoding byte as empty ([6b9b4d3](https://github.com/canboat/canboat-rs/commit/6b9b4d3060e1bd0f46bef9d8f300136d70ac541f))
* **core:** parse canboat's dash-separated timestamp in parse_iso_ms ([1fc507d](https://github.com/canboat/canboat-rs/commit/1fc507d41f4d01f4151cc68e720826ba0a5b81be))
* **tui:** keep the PGN-load rate bar visible on the selected row ([17164e2](https://github.com/canboat/canboat-rs/commit/17164e2d53c441abd58802d8f2ede18c007f3c44))
* **tui:** label on-request PGNs "on request" instead of a fake cadence ([82484ad](https://github.com/canboat/canboat-rs/commit/82484adf4bc543e41e1e6044f6b8a6847100d3d3))
* **tui:** PGN-load rate is occurrences over capture duration ([dd74c1a](https://github.com/canboat/canboat-rs/commit/dd74c1aac6e459bf1827465c0e439415cf9fa135))
* **tui:** show a robust median cadence for the device-detail "every" ([b62190e](https://github.com/canboat/canboat-rs/commit/b62190ef5886867043d1c91eb477c02fb5c7373a))

## [0.4.0](https://github.com/canboat/canboat-rs/compare/v0.3.0...v0.4.0) (2026-07-05)


### Features

* **canboat-core:** decode ISO_NAME dynamic field values ([43cc23b](https://github.com/canboat/canboat-rs/commit/43cc23bf034a7b7c07d0302bec790b828b744ad4))
* **canboat-core:** vendor schema inside the crate, pin + gate upstream sync ([76865fb](https://github.com/canboat/canboat-rs/commit/76865fbdb9f08afa5fe6ad209cb63ac67b202570))
* **canboat-pipeline:** binary WirePgn transport, canonical pipeline ports, performance ([465a138](https://github.com/canboat/canboat-rs/commit/465a138e6a75a9dad5243775dd97a3f2d56f6093))
* **canboat-pipeline:** live control channel for the NMEA0183 filter ([f13dbd5](https://github.com/canboat/canboat-rs/commit/f13dbd56d92716cb368c3bedb10468c86151be44))
* **canboat-pipeline:** per-device NMEA0183 filter keyed by NAME ([c2b1a73](https://github.com/canboat/canboat-rs/commit/c2b1a733671438523246c0dd7d1f95bc5d22bf11))
* **canboat-pipeline:** per-device NMEA0183 filter with live control channel and TUI ([37500ab](https://github.com/canboat/canboat-rs/commit/37500ab1e1a35ab1c75e192295433bf7b595aac1))
* **canboat-pipeline:** rate-limit NMEA0183 output to 1 Hz ([cf5a171](https://github.com/canboat/canboat-rs/commit/cf5a17132105142673426d76a38955dda52f6398))
* **canboat-pipeline:** rate-limit NMEA0183 output to 1 Hz ([5259eb6](https://github.com/canboat/canboat-rs/commit/5259eb69dfe57a616731baab9f447a2270661542))
* **canboat-tui:** NMEA0183 filter mode ([d124ea4](https://github.com/canboat/canboat-rs/commit/d124ea4d787c19ffa472e34eefdd4792b270e541))
* **canboat-wire:** binary WirePgn transport for pipeline consumers ([dd2dbe9](https://github.com/canboat/canboat-rs/commit/dd2dbe956f2abfd02f622c0466e6664857a4ff48))
* startup banner with binary version and canboat.json version ([40cf05a](https://github.com/canboat/canboat-rs/commit/40cf05abf51f9deef09adabcb48b158483b6661b))


### Bug Fixes

* **canboat-core:** don't let frame 0 complete a fast packet off stale bits ([3544487](https://github.com/canboat/canboat-rs/commit/35444879a3c361f0546920eee6e04e316c01922a))
* **canboat-core:** don't let frame 0 complete a fast packet off stale bits ([1d04646](https://github.com/canboat/canboat-rs/commit/1d046467eedb27a7466e9b42f2ea7f866730a793))


### Performance Improvements

* **canboat-pipeline:** batch TCP output, serialize JSON once per record ([8f37772](https://github.com/canboat/canboat-rs/commit/8f377727fd3c49737097d5528fd6d72922ea80b3))
* **canboat-pipeline:** batch TCP output, serialize JSON once per record ([4956f9b](https://github.com/canboat/canboat-rs/commit/4956f9b262b9d4287d49a2b2c9f7e5a880bbb160))

## [0.3.0](https://github.com/canboat/canboat-rs/compare/v0.2.0...v0.3.0) (2026-07-04)


### Features

* **analyser:** compile canboat.json into the canboat-core crate ([4036686](https://github.com/canboat/canboat-rs/commit/40366863cb81ac70dcf181adfd277aa0bdb6c602))
* **canboat-core:** ISO Transport Protocol reassembly ([0202997](https://github.com/canboat/canboat-rs/commit/0202997cafe004629244d4e92c00ef85186cc43f))
* **canboat-tui:** connecting modal + fatal-error modal ([26b6a68](https://github.com/canboat/canboat-rs/commit/26b6a6885fa5fba9d257d8331de33ceecf597b6d))
* **canboat-tui:** correct interval in log mode + hide age ([a2ef747](https://github.com/canboat/canboat-rs/commit/a2ef74780a1c81ce0eecba606f066e34f43ae121))
* **canboat-tui:** interactive TUI for n2kd / canboat-pipeline ([9aae7ab](https://github.com/canboat/canboat-rs/commit/9aae7abe85190504b06eb3cfea70f1f1f9c90663))
* **canboat-tui:** log-file replay mode + analyzer library extraction ([b57828c](https://github.com/canboat/canboat-rs/commit/b57828ca8da76c5bdda2e3f9d2d6b4b216a193bd))
* **canboat-tui:** measured transmission interval per PGN row ([f0a1bbe](https://github.com/canboat/canboat-rs/commit/f0a1bbe05e3b718c1d206449ea16f183c04b3172))
* **canboat-tui:** persistent NAME → info cache + return-to on EntryDetail ([c4774d6](https://github.com/canboat/canboat-rs/commit/c4774d625f1f6202b53b4154dc7d1bd9eb1a1901))
* **canboat-tui:** require --host or --log; suppress connect modal in log mode ([cc77dc5](https://github.com/canboat/canboat-rs/commit/cc77dc5cfcbf310e2dd1e81c04aa4ce4a4d9cd1d))
* **canboat-tui:** scrollable entry-detail screen ([53b4523](https://github.com/canboat/canboat-rs/commit/53b45238ec26839adf8b36722235ef25d426dc37))
* **canboat-tui:** surface non-OK PGN 126208 ACKs as status-bar alerts ([0c2ccef](https://github.com/canboat/canboat-rs/commit/0c2ccef868c2d4d281ec2a83e418a6c29c090bef))
* **canboat-tui:** TimeView + per-entry history + ←/→ instance nav ([b4eaed7](https://github.com/canboat/canboat-rs/commit/b4eaed7984a6c9b93ad9904ed05b846270eaf132))
* **canboat-tui:** TimeView search + PGN filter + source multi-select ([842c1ba](https://github.com/canboat/canboat-rs/commit/842c1ba8743fdab658cc1a1d55f93cf3d973f1bc))
* **canboat-tui:** writable-gated overrides; show silenced PGNs ([438a225](https://github.com/canboat/canboat-rs/commit/438a225c2dae7b877759ec12b94d8222610b92f7))
* embed copyright string in help texts, logs, and source files ([5953248](https://github.com/canboat/canboat-rs/commit/59532486f11ebbcc832b04440abf36bb7b763c44))


### Bug Fixes

* accept up to 1785-byte payloads + route TUI logs to a file ([e38e110](https://github.com/canboat/canboat-rs/commit/e38e11014b0dcceee46931c5f182a65d90027bf9))
* **canboat-core:** prefer specific no-Match variant over Fallback range catch-all ([cf2750a](https://github.com/canboat/canboat-rs/commit/cf2750ab1a7982a76e333a81c3d86f0acabfd59e))
* **canboat-core:** treat PGN 126464 Function Code as primary-key ([e74ffac](https://github.com/canboat/canboat-rs/commit/e74ffac5f22f70b6d06ea094fe8405fcd42a6e05))
* **canboat-tui:** advertise `t timeline` in Devices + DeviceDetail hints ([85f91c6](https://github.com/canboat/canboat-rs/commit/85f91c615daf4c2778ae71303741b5ec98647527))
* **canboat-tui:** bump connect timeout to 10s ([5958582](https://github.com/canboat/canboat-rs/commit/59585820f9805c23c7447bebc2ef5772051c9736))
* **canboat-tui:** emit PGN 126208 Request (fn 0), not Command (fn 1) ([b1e629b](https://github.com/canboat/canboat-rs/commit/b1e629b35e5200e30019048ff4eca9601a8f50f9))
* **canboat-tui:** non-blocking startup with bounded connect timeouts ([ebdd321](https://github.com/canboat/canboat-rs/commit/ebdd321945f68d4c4fc81e1a0c3b54e99b17e2f4))
* **canboat-tui:** per-screen keybinding hint bars ([586243b](https://github.com/canboat/canboat-rs/commit/586243b07f447d846794bb1f7ebf72b533640ec8))
* **canboat-tui:** preserve field order in entry-detail JSON view ([1adefa9](https://github.com/canboat/canboat-rs/commit/1adefa98188bd5893273f9a856c47361cafee115))
* **canboat-tui:** round measured interval to a stable step ([7be0f0e](https://github.com/canboat/canboat-rs/commit/7be0f0ed55898e25f5da4948c77a420f49384ed8))
* **canboat-tui:** stop column-shift on first ↓ in device list ([9827575](https://github.com/canboat/canboat-rs/commit/982757532ff036d42ce8f43d1f105ec68c613998))
* repair stale oversized-len test, emit C's match pseudo-unit in text mode ([cd843da](https://github.com/canboat/canboat-rs/commit/cd843da0651f886acb4301c16d3cc8b8a3769911))
