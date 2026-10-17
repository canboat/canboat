# The C programs

The C programs are CANboat's originals, and stay **tier 1** next to the
`canboat` binary ([programs.md](../programs.md)): supported, tested and released
together. They are small and need no more than `make` and a C compiler, which
suits a small machine. [using.md](./using.md) puts them together into pipelines.

`make` builds them into `rel/<platform>/`; see [BUILDING.md](../../BUILDING.md).
The gateway drivers write canboat's FAST format to stdout and send what they
read on stdin, in PLAIN or FAST, to the bus.

## `analyzer`

Reads NMEA 2000 messages as text on stdin, and writes them as human-readable
text or JSON. The format is detected from the first line, or set with
`-format`; [file-formats.md](../file-formats.md) lists the formats, and the
helper programs for the ones it does not read itself.

```text
$ analyzer -q < small.log
2011-11-24T22:42:04.388Z 2  36 255 127251 Rate of Turn:  SID = 125; Rate = 0.29199899 deg/s
2011-11-24T22:42:04.390Z 2  36 255 127250 Vessel Heading:  SID = 0; Heading = 182.4 deg; Deviation = 0.0 deg; Variation = 0.0 deg; Reference = Magnetic

$ analyzer -q -json < small.log
{"version":"8.3.0","units":"std","protocol":"nmea2000","showLookupValues":false}
{"timestamp":"2011-11-24T22:42:04.388Z","prio":2,"src":36,"dst":255,"pgn":127251,"description":"Rate of Turn","fields":{"SID":125,"Rate":0.29199899}}
{"timestamp":"2011-11-24T22:42:04.390Z","prio":2,"src":36,"dst":255,"pgn":127250,"description":"Vessel Heading","fields":{"SID":0,"Heading":182.4,"Deviation":0.0,"Variation":0.0,"Reference":"Magnetic"}}
```

The first JSON line is a header, not a message: the database version, the
units, the protocol, and whether lookups are written as value and name. A
consumer should read and skip it. Numbers are JSON numbers, and a field without
a value is left out.

| Option | |
| --- | --- |
| `-json` | JSON output; empty values are left out |
| `-empty` | JSON, with empty values as `null` |
| `-nv` | JSON, with a lookup as `{"value":N,"name":"…"}` |
| `-camel` | field names in lowerCamelCase |
| `-si` | strict SI units: kelvin, radians per second, Hz for rpm, m³ for litres, ratios for percentages |
| `-geo dd`, `dm`, `dms` | how to write a latitude and longitude |
| `-format <fmt>` | the input format, instead of detecting it |
| `-ignore-checksum` | accept `$PCDIN` and `$MXPGN` sentences whose checksum is wrong |
| `-quirk gps-rollover[=<device>,…]` | correct dates from a GPS that missed the week rollover; see [server.md](../server.md#device-quirks) |
| `-src <src>`, `-dst <dst>`, `<pgn>` | only messages from this source, to this destination, or of this PGN |
| `-clocksrc <src>` | set the system clock from this source's time messages |
| `-q`, `-d` | log only errors; log debug messages too |
| `-version` | print the version |

To debug a decode:

| Option | |
| --- | --- |
| `-raw` | print each message as PLAIN, to feed to `analyzer` again |
| `-data` | print each message three times: hex, ASCII and decoded |
| `-debug` | print the raw value of every field |
| `-debugdata` | print the raw value of every PGN |
| `-fixtime <str>` | write `<str>` as the time stamp in the log, for reproducible test output |

`analyzer-j1939` is the same program for SAE J1939 buses.

## `actisense-serial`

The driver for an Actisense NGT-1, over USB or serial, or at
`tcp://<host>[:<port>]`. It also reads an Actisense capture: a raw log file, an
`.ebl` log or a W2K-1 JSON capture.

```text
Usage: actisense-serial [-w] [-p] [-r] [-v] [-d] [-s <n>] [-t <n>] [--mode rx-all|normal] device

  -w      writeonly mode, no data is read from device
  -r      readonly mode, no data is sent to device
  -p      passthru mode, data on stdin is also sent to stdout (in addition to the device)
  -v      verbose
  -d      debug
  -s <n>  set baudrate to 38400, 57600, 115200, 230400
  -t <n>  timeout, if no message is received after <n> seconds the program quits
  --mode rx-all|normal  the NGT-1's operating mode: rx-all (default) forwards every
          PGN received, normal only those on its Receive PGN Enable list
```

```sh
actisense-serial -r /dev/ttyUSB0 | analyzer
actisense-serial -r capture.ebl | analyzer
```

- **Baud rate.** The default is 115200. An NGT-1 with firmware 2.500 or later,
  or one set to it, needs `-s 230400`. With the wrong rate there is no output,
  or only errors such as `DLE followed by unexpected char 21, ignore message`.
- **What it sends.** An NGT-1 transmits only the PGNs on its Transmit PGN
  Enable list, and `actisense-serial` does not change that list. `canboat
  interface --kind actisense --tx-pgn …` does; see
  [gateways.md](../gateways.md#transmit-and-receive-pgn-lists---tx-pgn---rx-pgn).
- **Two buses.** One gateway's output can be written to another, while both are
  decoded:

  ```sh
  actisense-serial -r /dev/ttyS1 | actisense-serial /dev/ttyS2 | analyzer
  ```

The Linux driver for an NGT-1 is in [gateways.md](../gateways.md#the-actisense-ngt-1-on-linux-and-macos).

## `ikonvert-serial`

The driver for a Digital Yacht iKonvert, over USB or at
`tcp://<host>[:<port>]`.

```text
Usage: ikonvert-serial [-w] [-p] [-r] [-v] [-d] [-s <n>] [-t <n>] device

  -w                    writeonly mode, data from device is not sent to stdout
  -r                    readonly mode, data from stdin is not sent to device
  -p                    passthru mode, data from stdin is also sent to stdout
  -v                    verbose
  -d                    debug
  -rx <list>            Set PGN receive list
  -tx <list>            Set PGN transmit list
  -l | --rate-limit-off Disable TX rate limits (use at own risk)
  -s <n>                set baudrate to 38400, 57600, 115200, 230400 (default 230400)
  -t <n>                timeout, if no message is received after <n> seconds the program quits
  -x                    hex instead of base64 mode
```

```sh
ikonvert-serial -r /dev/ttyUSB0 | analyzer
```

The iKonvert drops what is sent too fast; `--rate-limit-off` turns that off.
It uses a standard FTDI USB ID, so the standard driver works on Linux and
macOS; on Windows you may need to install FTDI's.

## `socketcan-serial`

Bridges a Linux SocketCAN interface. It claims an address, announces an ISO
NAME and sends a Heartbeat, unless `-n` makes it a passive bridge. Linux only.

```text
Usage: socketcan-serial [-w] [-r] [-p] [-v] [-d] [-n] [-t <n>] [-a <addr>] [-u <n>] [-m <n>] [-si <n>] [-hb <ms>] <can-device>

  -w        writeonly mode, received frames are not sent to stdout
  -r        readonly mode, data from stdin is not sent to the device
  -p        passthru mode, data from stdin is also echoed to stdout
  -v        verbose
  -d        debug
  -n        do not claim an address (passive bridge only)
  -t <n>    timeout, quit if no frame is received for <n> seconds
  -a <addr> preferred source address to claim (default 0)
  -u <n>    unique number for the ISO NAME (default derived from pid)
  -m <n>    manufacturer code for the ISO NAME
  -si <n>   ISO NAME System Instance, 0..15 (default 15 = yield to other devices)
  -hb <ms>  heartbeat (PGN 126993) interval in ms, 0 disables
```

```sh
socketcan-serial can0 | analyzer
```

## `maretron-ipg`

The driver for a Maretron IPG100, over TCP.

```text
Usage: maretron-ipg [-w] [-p] [-r] [-v] [-d] [-q] [-t <n>] [--password=<str>] tcp://<host>[:<port>]

  -w                    writeonly mode, data from device is not sent to stdout
  -r                    readonly mode, data from stdin is not sent to device
  -p                    passthru mode, data from stdin is also sent to stdout
  -v                    verbose
  -d                    debug
  -q                    quiet, suppress INFO logs
  -t <n>                timeout, if no message is received after <n> seconds the program quits
  --password=<str>      IPG login password (default empty)
```

The default port, 6543, is bus 0; `:6553` is bus 1.

```sh
maretron-ipg tcp://ipg100.local | analyzer
```

## `socketcan-writer`

Writes the FAST messages it reads on stdin to a SocketCAN interface,
and does nothing else: no address claim, no reading. Linux only.

```sh
replay < capture.log | socketcan-writer vcan0
```

## `candump2analyzer`

Turns a raw CAN capture into PLAIN, for `analyzer`: the outputs of Linux
`candump` in its several flavours, a Wireshark text export, a Navico dump and a
PCAN-View trace (examples in [file-formats.md](../file-formats.md)). It reads the
file it is given, or stdin.

```sh
candump can0 | candump2analyzer | analyzer
```

## `pcap2candump`

A Python script that turns a raw `.pcap` CAN capture into a candump log, for
`candump2analyzer`: `pcap2candump/pcap2candump.py capture.pcap`.

## `replay`

Writes a PLAIN or FAST capture from stdin to stdout at the pace it was
recorded, from each line's time stamp. A gap of more than 10 seconds, or a time
stamp that goes back, is not waited for. `-d` debugs.

```sh
replay < capture.log | analyzer
```

## `request-group-function` and `command-group-function`

Build a Group Function (PGN 126208) in PLAIN, for a gateway driver to send.

- `request-group-function <dest> <prio> <pgn> <interval> <field>=<value> ...`
  asks a device to report a PGN at an interval in milliseconds, optionally only
  when fields have the given values.
- `command-group-function <dest> <prio> <pgn> <field>=<value> ...` tells a
  device to set fields.

Note that the *request* takes the interval, not the command. A `<field>` is the
decimal field number in the PGN; a `<value>` is hexadecimal, and its length is
the number of bytes written. At most 20 fields. What a device accepts is up to
the device.

```sh
request-group-function 35 6 126996 2000 | socketcan-writer can0
```

## `nmea0183-serial`

Reads an NMEA 0183 serial port, at 38400 baud, and writes it to stdout, along
with what it reads on stdin unless given `-r`; `-d` debugs. With
[`iptee`](#iptee) it feeds, for instance, AIS to online services:

```sh
nmea0183-serial -r /dev/ttyUSB0 | iptee -u 195.251.168.18 6047 -u 87.121.76.11 2112
```

and it can merge two ports:

```sh
nmea0183-serial -r /dev/serial-0 | nmea0183-serial /dev/serial-1 > combined.log
```

## `iptee`

Copies stdin to up to 256 TCP and UDP listeners, reconnecting to those that go
away, and to stdout unless given `-w`. It suits UDP listeners best: a TCP
connect blocks the stream while it waits.

```text
Usage: iptee [-w] [-d] [-q] [-s|-t|-u] host port [host port ...] | -version

-w - writeonly - only write to network clients/servers, not stdout
-d - debug     - log debug information
-q - quiet     - do not log status information
-s - server    - host and port are a TCP server
-u - udp       - host and port are a UDP address that data is sent to
-t - tcp       - host and port are a TCP server that data is sent to
```

## `format-message`

A PHP script in `util/` that builds one of a few messages, as PLAIN (or YDWG RAW with
`--format YDWG`), for a gateway driver to send: `format-message [--format
YDWG|CANBOAT] <dst> <command> <args>`. Without arguments it lists the
commands, among them `request_pgn`, `request_interval`, `command_address`
and `setvolume`. A request for every device's Product Information:

```sh
format-message 255 request_pgn 126996
```

`canboat format-message` builds any PGN.

## `list-product-information`

A PHP script in `util/` that lists the devices on the bus and their software versions. It
needs a `canboat server`, or a pipeline ending in `canboat n2kd`: it sends ISO
Requests for Product Information (126996) and Configuration Information
(126998) to the input port, 2600, and reads the answers from the snapshot port,
2597. `/etc/default/canboat` can set other ports or hosts (`N2K_WRITE_HOST`,
`N2K_WRITE_PORT`, …).

```text
$ list-product-information
Product Information:
60: MasterBus NMEA Interface         1.061.00
36: PB200 WeatherStation             1.600,1.611,1.001,1.611,00044-802-2-01,HW6,OEM0
 0: NMEA 2000 PC Interface (NGT-1)   1.090, 2.190NGT-1-USB  hv1.03
35: DST200                           1.002,1.02244-127-3-01,HW9,OEM0
```

## Removed in v8

- **The C `n2kd`.** `canboat n2kd` replaces it, with the same ports; `canboat
  install-shims` keeps the name working.
- **`n2kd_monitor`**, the perl script that restarted `n2kd` and its gateway
  driver, configured in `/etc/default/n2kd`. `canboat server` replaces the
  whole arrangement, run by your init system; see
  [server.md](../server.md#running-it-as-a-service).
- **`analyzer -explain` and `-explain-xml`.** The PGN documentation is
  generated from the database; see [definitions.md](../definitions.md).
