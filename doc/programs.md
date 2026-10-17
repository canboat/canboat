# The `canboat` binary

`canboat` is one executable with subcommands: it does what CANboat's C programs
do, plus `canboat server`, `canboat tui` and the gateways the C drivers never
had. Both are **tier 1**: supported, tested and released together, and where
both do a job they are interchangeable. The C programs are in
[c/programs.md](./c/programs.md).

The PGN database is compiled into it: there is nothing to load at runtime, and
every subcommand decodes the same way.

| Job | `canboat` | The C programs |
| --- | --- | --- |
| Decode a capture | [`canboat convert`](#canboat-convert) | [`analyzer`](./c/programs.md#analyzer) |
| Read and write a gateway | [`canboat interface`](#canboat-interface), for every gateway in [gateways.md](./gateways.md) | [`actisense-serial`](./c/programs.md#actisense-serial), [`ikonvert-serial`](./c/programs.md#ikonvert-serial), [`socketcan-serial`](./c/programs.md#socketcan-serial), [`maretron-ipg`](./c/programs.md#maretron-ipg) |
| Gateway, decode and serve in one process | [`canboat server`](./server.md) | |
| Serve decoded JSON to TCP clients | [`canboat n2kd`](#canboat-n2kd) | |
| Browse a live bus or a capture | [`canboat tui`](#canboat-tui) | |
| Build a message | [`canboat format-message`](#canboat-format-message) | [`format-message`](./c/programs.md#format-message) (PHP), [`request-group-function`, `command-group-function`](./c/programs.md#request-group-function-and-command-group-function) |
| Play a capture at its own pace | [`canboat replay`](#canboat-replay) | [`replay`](./c/programs.md#replay) |
| Convert other capture formats | `canboat convert` | [`candump2analyzer`](./c/programs.md#candump2analyzer), [`pcap2candump`](./c/programs.md#pcap2candump) |
| Write to a SocketCAN interface | `canboat interface --kind socketcan -w` | [`socketcan-writer`](./c/programs.md#socketcan-writer) |
| List the devices on the bus | `canboat tui` | [`list-product-information`](./c/programs.md#list-product-information) (PHP) |
| NMEA 0183 plumbing | | [`nmea0183-serial`](./c/programs.md#nmea0183-serial), [`iptee`](./c/programs.md#iptee) |

Every subcommand takes `--help`. [using.md](./using.md) puts them together
into pipelines.

> [!WARNING]
> The PGN database is reverse engineered. Most definitions have been confirmed
> against several devices, but some fields are educated guesses. Do not build
> something safety-critical on a decoded value without checking it against what
> the instrument itself shows.

## `canboat convert`

Decodes a capture, or converts it to another capture format. The input format is
detected from the first line, or set with `--from`; `.pcap`, `.pcap.gz` and
`.nif` containers are unwrapped. [file-formats.md](./file-formats.md) lists the
formats.

```sh
canboat convert capture.log                 # JSON
canboat convert --to text capture.log       # human-readable
canboat convert --to plain capture.candump  # canboat PLAIN, for analyzer
canboat convert --pgn 129025 --src 3 capture.log
```

```text
$ canboat convert small.log
{"version":"8.3.0","commit":"7ac57fdb","units":"si","protocol":"nmea2000","showLookupValues":false}
{"timestamp":"2011-11-24T22:42:04.388Z","prio":2,"src":36,"dst":255,"pgn":127251,"description":"Rate of Turn","fields":{"sid":125,"rate":0.0050963438}}
{"timestamp":"2011-11-24T22:42:04.390Z","prio":2,"src":36,"dst":255,"pgn":127250,"description":"Vessel Heading","fields":{"sid":0,"heading":3.1834,"deviation":0.0000,"variation":0.0000,"reference":"Magnetic"}}
```

The first line is a header, not a message: the version, the units, the
protocol, and whether lookups are written as value and name (`--no-banner`
leaves it out). Its JSON is not quite the C `analyzer`'s: field keys are lowerCamelCase and values
are in SI units (radians, kelvin, pascal). `--id spaces --units metric` gives
`analyzer`'s keys and units. `--nv`, `--empty` and `--debug` match `analyzer`'s
`-nv`, `-empty` and `-debug`; `--protocol j1939` or `quick` decodes those
buses. `--quirk gps-rollover` corrects dates, as in
[server.md](./server.md#device-quirks).

## `canboat interface`

Bridges a live gateway to stdin and stdout, in canboat's FAST format: what it
receives goes to stdout, what it reads on stdin goes onto the bus.

```sh
canboat interface --kind actisense /dev/ttyUSB0
canboat interface --kind yachtd tcp://ydwg
canboat interface --kind socketcan can0
```

`-r` only reads, `-w` only writes, and `-b` sets the baud rate. The gateways,
their options and their PGN lists are in [gateways.md](./gateways.md).

## `canboat server`

The whole gateway → decode → serve chain in one process; see
[server.md](./server.md).

## `canboat n2kd`

Reads decoded JSON on stdin, from `analyzer -json` or `canboat convert`, and
serves it to TCP clients on the same ports as `canboat server`, from base port
2597 (`-p`): the snapshot, the JSON stream, NMEA 0183, the raw input port and
the AIS snapshot. It listens on 127.0.0.1 unless given `--public`.

```sh
actisense-serial -r /dev/ttyUSB0 | analyzer -json -nv | canboat n2kd
```

It replaces the C `n2kd`, which was removed in v8. For a new setup,
`canboat server` is usually the better choice: being one process, it can also
answer ISO Requests and Group Functions, track the devices on the bus, and
request what has gone stale. Use `canboat n2kd` when you want the pipeline, for
instance to decode elsewhere or to tee a stage with
[`iptee`](./c/programs.md#iptee).

## `canboat tui`

An interactive terminal browser: the PGNs on the bus, the devices sending them,
and the decoded fields of the one you select.

```sh
canboat tui --host pi.local       # a canboat server or n2kd
canboat tui --log capture.log     # a capture
```

It reads the snapshot (2597) and JSON stream (2598) ports. There is no default
host: with neither option it starts idle, with a dialog to load a capture or
connect.

## `canboat format-message`

Builds one message from field values, for any PGN, in canboat PLAIN or another
format (`--to`). A number is in the field's unit; `raw:N` writes the wire bits.

```sh
canboat format-message --list               # every PGN
canboat format-message 126996 --help        # a PGN's fields
canboat format-message isoRequest --dst 0 PGN=126996
```

## `canboat replay`

Writes a PLAIN or FAST capture from stdin to stdout at the pace it was
recorded, as the C [`replay`](./c/programs.md#replay) does. `canboat server
--replay` serves a capture as a live bus instead; see [server.md](./server.md).

## `canboat install-shims`

Makes the subcommands answer to the old names (`analyzer`, `n2kd`,
`actisense-serial`, …), with symlinks in the binary's own directory or in
`--dir`. It never takes a name that another program already holds, so it is
safe in a directory with the C programs installed, and it says which names it
left alone:

```text
/usr/local/bin/analyzer: left alone, a file is already there
/usr/local/bin/n2kd -> /usr/local/bin/canboat
```

