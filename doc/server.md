# `canboat server`

`canboat server` is the whole device → analyzer → n2kd pipeline in one
process: it reads a gateway (see [gateways.md](./gateways.md)), decodes every
message, applies the device quirks you switch on, and serves the result over
TCP. It replaces `actisense-serial | analyzer | n2kd` and its siblings.

```sh
canboat server --kind actisense --device /dev/ttyUSB0
```

## Ports

Every port is a TCP port, listening on all interfaces unless `--bind
127.0.0.1` keeps it to this host. The first five are the C `n2kd`'s
(`-p` and the ports after it), so its clients work unchanged. `0` turns a
port off.

| Option | Default | What it serves |
| --- | --- | --- |
| `--snapshot-port` | 2597 | On connect, the latest JSON record of every PGN (one per PGN, source and instance), then close. AIS PGNs are on `--ais-port` instead, apart from 129026 and 129029, which are on both |
| `--analyzer-port` | 2598 | Decoded JSON, one record per line; read-only |
| `--nmea0183-port` | 2599 | NMEA 0183 sentences, AIVDM included; read-only |
| `--input-port` | 2600 | Write-only: PLAIN/FAST lines written here are sent onto the bus |
| `--ais-port` | 2601 | On connect, the latest record of every AIS PGN, plus 129026 and 129029, then close |
| `--raw-port` | 2603 | Raw frames as `# format=FAST` PLAIN lines, fast-packets already joined; read-only |
| `--nmea0183-filter-port` | 2605 | NMEA 0183 filter control (PGN 262657), for `canboat tui`; read and write |
| `--overrides-port` | 2606 | PGN rate override control (PGN 262658), for `canboat tui`; read and write |

The state the filter and override ports set is kept in `--config-dir`:
`/etc/default/canboat` if it is writable (the systemd/root case), otherwise
`~/.local/canboat`. `canboat server --help` has the rest of the options.

## Device quirks

Some devices misbehave in ways CANboat can cover for. Each workaround is a
named *quirk*, and every one of them is **off by default** — they all rewrite
or invent data, so you switch on the ones you need:

| Quirk | What it does | Where |
| --- | --- | --- |
| `gps-rollover` | Corrects GNSS dates from a receiver that never learned about the GPS 1024-week rollover and reports one or two epochs in the past (PGN 129029, 129033, and 126992 when its source is GPS) | `analyzer -quirk gps-rollover`, `canboat convert --quirk gps-rollover`, `canboat server --quirk gps-rollover` |
| `gps-rollover=<device>,…` | The same, plus every date the listed devices stamp from that clock — a DSC radio's Date of Receipt on PGN 129808, a converter's System Time, and so on. A device is a source address (`4`), its manufacturer code and unique number (`1851:491603`), or its ISO NAME in hex (`0x…`); `all` means every device on the bus | same three commands |
| `gps-relay` | Re-sends the data PGNs the devices listed in `gps-rollover=…` broadcast (not their network management, and not anything addressed), from canboat's own address with the dates corrected, so other devices on the bus can select canboat as their GPS and time source instead of the rolled-over one. The original keeps transmitting; you pick the source in each display. Needs a real device list — `gps-rollover=all` is refused here | `canboat server --quirk gps-rollover=<device> --quirk gps-relay` (needs a writable backend) |
| `scx20` | Answers a PGN 59904 request for a Furuno SCX-20's Product Information on its behalf, so the Furuno Setting Tool can find it | `canboat server --quirk scx20` (needs `--kind socketcan`, `canalyst` or `usb-can-a`) |
| `wmm` | Computes magnetic variation locally with WMM 2025 and emits its own PGN 127258, asking older-WMM sources to stop | `canboat server --quirk wmm` |
| `motion` | Impersonates a B&G H5000 Motion Sensor so a Navico Hercules accepts an SCX-20 | `canboat server --quirk motion` (needs `--kind socketcan`, `canalyst` or `usb-can-a`) |

`gps-rollover` is the odd one out: it corrects a decoded value rather than
putting a frame on the bus, which is why it is available when decoding a
capture file too. Do not use it on a capture made before April 2019 — it
cannot tell a rolled-over date from a genuinely old one, and will move the
whole log forward twenty years.

Without a device list it only touches the dates a GNSS receiver produced,
because those are the only ones whose origin the PGN itself tells us. Any
other device that takes its clock from the bus repeats the wrong date, and
nothing in its PGNs says so; that is what the device list is for. Naming a
device by manufacturer and unique number, or by NAME, needs its PGN 60928
Address Claim to have gone by — `canboat server` asks for those at startup,
a capture has to contain one. The AIS reports (129793, 129794) relay another
station's clock and 127258 carries the variation model's date, so those are
never corrected.

None of that helps a display that reads the broken GPS directly. `gps-relay`
is the nearest thing to a fix short of replacing the receiver: `canboat
server` re-sends every broadcast PGN from the listed devices — position,
COG/SOG, satellites, not just the dated ones, or no display would accept it
as a GPS — from its own address, byte for byte except for the corrected
dates. The other devices then see a second GPS source, canboat, and you
select it in each of them. That needs devices with real source selection,
which not every brand gets right, and it leaves the old receiver on the bus,
since it is where the data comes from.

## A capture as a live bus (`--replay`)

`canboat server --replay <capture>` plays a capture as if it were a gateway,
so you can test a consumer such as Signal K without a boat. It reads any
format canboat reads, keeps the capture's pacing but stamps every frame with
the current time, and starts over at the end. The TCP ports are the same as
for a real gateway, the input port included: frames written to it go to
`--replay-sent <file>` as PLAIN/FAST lines, or are dropped, and are not
echoed back.

```sh
canboat server --replay samples/dirona-actisense-serial.raw --replay-sent sent.raw
```

## Performance

Same decode work — `-json -nv` over 1.26 M PGN frames (canboat's
`dirona-actisense-serial.raw` × 50) on an M4 Pro, release build:

| Implementation         | Wall time | vs canboat  |
|------------------------|----------:|--------------:|
| canboatjs (Node 25)    |  27.8 s   |   **8.1 ×**   |
| canboat C              |   9.1 s   |   **2.6 ×**   |
| canboat `convert`      |   3.4 s   |       1.0 ×   |

`canboat server` goes one step further and collapses the
`canboat convert | canboat n2kd` pipeline into a single process with no
JSON text serialisation between stages:

| Pipeline                                          | Wall time | Throughput          |
|---------------------------------------------------|----------:|---------------------|
| `canboat convert` alone (PGN decode only)         |   3.3 s   | 380 k frames / s    |
| `canboat convert \| canboat n2kd` (piped, 2 proc) |   6.5 s   | 194 k sentences / s |
| `canboat server` (single proc)                    |   3.5 s   | 360 k sentences / s |

That's 46 % less wall time than the piped setup while doing strictly more
work (it's a long-running service with TCP fan-out); on CPU time the ratio
is closer to 3.4 ×.
