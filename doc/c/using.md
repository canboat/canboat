# Using the C programs

The C programs ([programs.md](./programs.md)) are made to be put together with
UNIX pipes: one reads the bus, one decodes, one serves or stores. This page works
up from the simplest case; the same with the `canboat` binary is in
[../using.md](../using.md), and the two mix: a C driver can feed `canboat
convert`, and `canboat interface` can feed `analyzer`.

## Decode a capture you already have

```sh
analyzer < capture.log           # for you to read
analyzer -json < capture.log     # for another program
```

The format is detected from the first line. A capture `analyzer` does not read
itself goes through a helper first, such as `actisense-serial -r capture.ebl |
analyzer`; see [../file-formats.md](../file-formats.md).

## Watch a live bus

Read the gateway and pipe it into `analyzer`. For an Actisense NGT-1 on
`/dev/ttyUSB0`:

```sh
actisense-serial -r /dev/ttyUSB0 | analyzer
```

The other drivers are `ikonvert-serial`, `socketcan-serial` and `maretron-ipg`.

## Watch one device, or one PGN

Only what source address 35 sends:

```sh
actisense-serial -r /dev/ttyUSB0 | analyzer -src 35
```

Once you know which PGN interests you, give it, and `-data` to see the bytes
next to the decode, which is what you need when a field looks wrong:

```sh
actisense-serial -r /dev/ttyUSB0 | analyzer -data 130311
```

`-debug` goes further and prints the raw value of every field.

## Serve the data to other software

End the pipeline with `canboat n2kd`, which serves the JSON to TCP clients (the C
`n2kd` was removed in v8):

```sh
actisense-serial -r /dev/ttyUSB0 | analyzer -json -nv | canboat n2kd
```

`canboat server` does all three in one process, and is the better choice for a
permanent installation; see [../server.md](../server.md).

## Replay a capture at its own pace

```sh
replay < capture.log | analyzer
```

## Write to the bus

A driver sends what it reads on stdin to the bus. If nothing else has the
gateway open:

```sh
echo 2012-06-17-15:02:11.000,6,59904,0,255,3,14,f0,01 | actisense-serial /dev/ttyUSB0
```

That is an ISO Request to every device for its Product Information. An NGT-1
transmits only the PGNs on its Transmit PGN Enable list, which `actisense-serial`
does not change; see [programs.md](./programs.md#actisense-serial).

When a reader already has the gateway open, give the pipeline a way in: a fifo,

```sh
mkfifo /tmp/canbus-write
actisense-serial /dev/ttyUSB0 < /tmp/canbus-write | analyzer
# then, from anywhere:
echo 2012-06-17-15:02:11.000,6,59904,0,255,3,14,f0,01 > /tmp/canbus-write
```

or a socket, with `nc`:

```sh
nc -lk localhost 5000 | actisense-serial /dev/ttyUSB0 | analyzer
echo 2012-06-17-15:02:11.000,6,59904,0,255,3,14,f0,01 | nc localhost 5000
```

To build the line rather than write the hex, use `canboat format-message`, or
`format-message` and the group-function programs. On SocketCAN there is also
`socketcan-writer`.

## Two buses

A driver's output can be written to another gateway, while both are decoded:

```sh
actisense-serial -r /dev/ttyS1 | actisense-serial /dev/ttyS2 | analyzer
ikonvert-serial -r /dev/ttyS1 | actisense-serial /dev/ttyS2 | analyzer
```

## Tee a stage

`iptee` copies a stage of a pipeline to the network, without disturbing the
pipe. With `-s` it is the TCP server, here on port 2610, so other programs can
connect to the raw frames while `analyzer` decodes them:

```sh
actisense-serial -r /dev/ttyUSB0 | iptee -s 0.0.0.0 2610 | analyzer -json
```

With `-u` or `-t` it sends to a UDP address or a TCP server instead.
