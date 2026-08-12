# Navico UDB database and source selection

Navico-family devices share configuration and settings across the network with a
group of proprietary key/value PGNs. The central one, **PGN 130822**, is a
general database / table-sharing transport (the "UDB" database). *Source
selection* — which device provides each data type — is a related but **separate**
mechanism, carried by **PGN 130840**.

This note covers both, because they are easy to confuse: 130822 carries a
*table* of source selections among many other tables, but it is a periodic dump
that **lags** live changes; the actual change-driven selection is 130840.

As always, the manufacturer code in the first two data bytes disambiguates these
from unrelated uses of the same PGN numbers (130822 is also used by BEP Marine
(295) and Mercury (144)).

## PGN 130822 — the UDB database (Navico: UDB Database)

A general table-sharing transport. Every frame shares an outer container:

| Field | Meaning |
|-------|---------|
| Marker | always `0xFF` |
| Command | message type (the dispatch opcode) |
| Address / Section / Item | address of the object being reported |
| (payload) | depends on Command |

Two commands appear in normal traffic:

- **Command 1 — Source Report** (the everyday form): a fixed per-object record
  carrying an `Object Value` (`0xFC00` = no value), and a `Sub` + `Token` pair
  that is a stable identity for the object definition. It is best read as a
  periodic *per-object value* broadcast; it does **not** name a device.
- **Command 6 — Object Dump** (occasional): an object's full table, as a list of
  length-prefixed records `[length][class][data type id : u16][value]`. The
  `Data Type` is decoded with `NAVICO_DATA_TYPE`, and `Value` is a tagged blob:
  a device-NAME wrapper (`0x0A 0x00` + 64-bit NAME + `0xFF`), a single `0x00`
  (none/auto), or a small typed value.

The Command 6 dump *includes* a source-selection table (data type → selected
NAME), but it is one of several replicated tables and is broadcast periodically.
Because it is a database snapshot rather than an event, it can show a stale
selection for some seconds after a change, and it keys data types by the full
`NAVICO_DATA_TYPE` space (e.g. "Attitude Yaw", "Heading", pilot limits, ...),
which is broader than the live-selection id space below.

## PGN 130840 — Simnet: Data Source Selection (the live selection)

This is the **change-driven** PGN that fires when a source is chosen on a display
or controller. The 17-byte payload, confirmed against a live capture of a
compass-source change, is:

| Byte | Field | Meaning |
|------|-------|---------|
| 0–1 | Manufacturer + Industry | Simrad (1857), marine |
| 2 | Reserved | `0xFF` |
| 3 | Sequence | observed constant `0x00` |
| 4 | B | observed constant `0x01` |
| 5 | **Data Type** | which quantity (`SIMNET_DATA_SOURCE` lookup, e.g. 27 = Heading) |
| 6 | Reserved | `0xFF` |
| 7 | C | a **change counter** (seen incrementing by one per source change) |
| 8–15 | **Source** | the selected device's full 64-bit NMEA 2000 NAME |
| 16 | Reserved | `0xFF` |

For example, switching the compass source on an autopilot controller emits a
single 130840 with `Data Type = Heading` and `Source` = the newly selected
device's NAME (and `C` ticks up by one). It appears immediately on the change,
not periodically. (Values `0xFA–0xFE` in the Data Type byte are not data types —
they are the auto-select handshake codes.)

Note the two id spaces differ: 130840's `SIMNET_DATA_SOURCE` is a small, compact
data-type id (Heading = 27), **distinct** from the larger `NAVICO_DATA_TYPE` id
(Heading = 36) used by the 130822 Command 6 table.

## Which to watch

- To observe a source **change** as it happens: watch **130840** — it is emitted
  once, immediately, with the new device's NAME.
- To read the current **database** of selections (and other replicated tables):
  read the **130822** Command 6 dumps, keeping in mind they are periodic and can
  lag.
- To resolve a NAME to a device, match it to that device's ISO Address Claim
  (PGN 60928) and Product Information (PGN 126996) on the same bus.

## Related key/value PGNs

- **PGN 130823** — a sibling UDB container with the same Command 6 record layout;
  carries additional per-object tables.
- **PGN 130824 (B&G)** — an H3000-style key/value broadcast: a packed stream of
  `{12-bit key, 4-bit length, value}` records, where the per-key length is the
  fixed unit width of the value (not a free-form size).
- **PGN 130845 (Simnet: Key Value)** — single key/value parameter messages keyed
  by `SIMNET_KEY_VALUE`; this is where per-parameter limits (e.g. alarm
  thresholds) are published.
