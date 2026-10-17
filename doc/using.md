# Using CANboat

CANboat is not turnkey: it is meant for engineers and hobbyists who want to see
what is on their NMEA 2000 network, for reverse engineering, analysis, replay,
testing and debugging other devices. If you just want to use your instruments,
a project built on CANboat is friendlier: [Signal K](https://signalk.org/), or
[OpenPlotter](https://openplotter.org/).

This page works up from the simplest case, with the `canboat` binary
([programs.md](./programs.md)). The same with the C programs is in
[c/using.md](./c/using.md).

## Decode a capture you already have

```sh
canboat convert --to text capture.log    # for you to read
canboat convert capture.log              # JSON, for another program
```

The format is detected from the first line; [file-formats.md](./file-formats.md)
lists what canboat reads.

## Watch a live bus

`canboat interface` reads a gateway and writes its frames to stdout; pipe them
into `canboat convert`. For an Actisense NGT-1 on `/dev/ttyUSB0`:

```sh
canboat interface --kind actisense -r /dev/ttyUSB0 | canboat convert --to text
```

[gateways.md](./gateways.md) has the other gateways and how to name them.

## Watch one device, or one PGN

This is what you want when reverse engineering. Only what source address 35
sends:

```sh
canboat interface --kind actisense -r /dev/ttyUSB0 | canboat convert --to text --src 35
```

Once you know which PGN interests you, add `--pgn`, and `--debug` to see each
field's bytes and bits next to its value, which is what you need when a field
looks wrong:

```sh
canboat interface --kind actisense -r /dev/ttyUSB0 | canboat convert --to text --pgn 130311 --debug
```

## Browse interactively

```sh
canboat server --kind actisense --device /dev/ttyUSB0 &
canboat tui --host localhost
```

`canboat tui` shows the PGNs on the bus, the devices sending them, and the
decoded fields of the one you select. It also opens a capture: `canboat tui
--log capture.log`.

## Serve the data to other software

`canboat server` reads the gateway, decodes, and serves JSON, NMEA 0183 and the
raw frames on TCP ports, all in one process:

```sh
canboat server --kind actisense --device /dev/ttyUSB0
```

It keeps running; in another terminal:

```sh
nc localhost 2598                       # every decoded message, as JSON
nc localhost 2599                       # NMEA 0183, for a chart plotter program
```

[server.md](./server.md) has the ports, and how to run it as a service.

If the decoding happens elsewhere, `canboat n2kd` serves a JSON stream it reads
on stdin on the same ports:

```sh
canboat interface --kind actisense -r /dev/ttyUSB0 | canboat convert | canboat n2kd
```

## Replay a capture

At the pace it was recorded:

```sh
canboat replay < capture.log | canboat convert --to text
```

Or as a live bus, ports and all, to test a consumer without a boat:

```sh
canboat server --replay capture.log
```

## Write to the bus

`canboat interface` sends what it reads on stdin, as PLAIN or FAST lines, to the
bus. `canboat format-message` builds the line, so you need not write the hex
yourself. An ISO Request to every device for its Product Information:

```sh
canboat format-message isoRequest --dst 255 PGN=126996 | canboat interface --kind actisense -w /dev/ttyUSB0
```

While `canboat server` has the gateway, write to its input port instead:

```sh
canboat format-message isoRequest --dst 255 PGN=126996 | nc localhost 2600
```

Whether a gateway passes a message on is up to the gateway: an NGT-1 or an
iKonvert sends only the PGNs on its transmit list. See
[gateways.md](./gateways.md#transmit-and-receive-pgn-lists---tx-pgn---rx-pgn).

## With Signal K

Signal K uses CANboat's sibling, [canboatjs](https://github.com/canboat/canboatjs),
which is built for it.
