# Gateways

To read or write a live NMEA 2000 bus, canboat needs a CAN interface: a
marketed-as-such NMEA 2000 gateway, or a plain CAN adapter. This page lists
the ones canboat drives, how to name them, and what each can and cannot do.

## Gateways and their options

`canboat interface` and `canboat server` take the same gateways and options:
`canboat interface --kind yachtd tcp://host` and `canboat server --kind yachtd
--device tcp://host`. An Actisense gateway also takes the data format it is
set to, by Actisense's names (`--data-format`, default `ngt-actisense`):

| `--kind` | `--data-format` | Gateways | Device | Read/write | J1939 | 11-bit |
| --- | --- | --- | --- | --- | --- | --- |
| `actisense` | `ngt-actisense` | NGT-1 | serial, `usb`, `tcp://` (6001) | rw | – | – |
| | `n2k-ascii` | W2K-1, PRO-NDC-1E2K | `tcp://` (60002), `udp://` | rw | – | – |
| | `n2k-actisense` (BST-D0) | W2K-1, PRO-NDC-1E2K | `tcp://` (60002) | rw | – | – |
| | `raw-ascii` | W2K-1, PRO-NDC-1E2K | `tcp://` (60002), `udp://` | rw | yes | – |
| | `raw-actisense` (BST-95) | W2K-1, PRO-NDC-1E2K, NGX | `tcp://` (60002), NGX serial | rw | yes | – |
| | `mxpgn` | W2K-2 (CANDIRECT, NMEA 0183) | `tcp://` (20000), `udp://` | rw | yes | – |
| `yachtd` | `raw-ascii` | YDWG-02, YDEN-02, YDNR-02, YDNU-02 | `tcp://` (1457), `udp://`, YDNU-02 serial | rw | yes | – |
| `miniplex` | | ShipModul MiniPlex-3 with NMEA 2000, in `$MXPGN` | `tcp://` (10110), `udp://`, serial | rw | yes | – |
| `seasmart` | | Chetco SeaSmart, Quark-elec QK-A032/A034/A036, Yacht Devices in NMEA 0183 mode, in `$PCDIN` | `tcp://` (10001), `udp://`, serial | rw | – | – |
| `ikonvert` | | iKonvert, NavLink2 | serial, `tcp://` (2000), `udp://` (2000) | rw | – | – |
| `maretron` | | IPG100/200 | `host[:6543]` | rw | – | – |
| `aos` | | AOS simulator | serial, `usb`, `tcp://` (6001) | rw | – | – |
| `socketcan` | | Linux CAN interface | `can0` | rw | yes | yes |
| `canalyst` | | CANalyst-II, Waveshare USB-CAN-B | `usb` | rw | yes | yes |
| `usb-can-a` | | Waveshare USB-CAN-A | serial | rw | yes | yes |

- **Read/write:** `udp://[bind:]port` only receives. A W2K-1 sends
  `ngt-actisense` but does not take it; canboat does not handle that yet.
- **J1939** (`--protocol j1939`): ISO TP messages are received wherever it says
  yes, but sent only on SocketCAN, the CANalyst-II and the USB-CAN-A, where
  canboat is the node. **11-bit** frames are Quick PCS (`--protocol
  quick`). The rest carry NMEA 2000 only.
- **ikonvert: use TCP for a NavLink2.** A NavLink2 serves the iKonvert
  protocol on TCP *or* UDP, whichever its web page is set to, on port 2000
  (canboat's default for both). Over `tcp://` canboat puts the gateway on the
  bus, sets its PGN lists, and reads and writes; when it exits, it takes the
  gateway off the bus again (`$PDGY,N2NET_OFFLINE`). Over `udp://[bind:][port]`
  canboat only receives:
  - **The gateway must already be active.** A NavLink2 passes no PGNs until a
    client puts it on the bus (`$PDGY,N2NET_INIT`), and canboat can't send
    that over UDP. canboat expects some other client to have done so since the
    gateway started (it stays on the bus after that client leaves, until it
    is power cycled or taken off), and never takes it off the bus itself. It
    warns when the gateway reports that it is off the bus. A canboat session
    over `tcp://` can't be that client: it takes the gateway off the bus when
    it exits.
  - **Nothing reaches the gateway:** `--tx-pgn`, `--rx-pgn` and
    `--no-tx-rate-limit` are ignored with a warning. `--rx-filter` still works,
    because canboat applies it. To send, use `tcp://`: Digital Yacht says TCP
    is needed to drive an autopilot.

  Use `udp://2000` rather than a local address, or the broadcast may be
  missed. A ser2net bridge needs its own port given.
- **miniplex:** when it connects, canboat sets the MiniPlex to send every
  PGN it receives as `$MXPGN` (`$PSMDNO` options 30 and 31, mode 2), and asks
  it for every device's Address Claim and Product Information (`$PSMDNR`).
  Over a receive-only `udp://port` it can't, so set that on the MiniPlex
  first, or give its address, `udp://miniplex:port`. The MiniPlex never sends
  Address Claims (60928) and Product Information (126996) as `$MXPGN`, but as
  its own `$PSMDNC` and `$PSMDNP`, and the latter only for a request it made:
  canboat reads both as those PGNs, also in a capture (`canboat convert`,
  `analyzer`), and sends an ISO Request for either as `$PSMDNR`.
- **W2K-2:** each data server (60001–60003) takes one Tx Format, set on its
  *Serial* page: `ngt-actisense`, `n2k-actisense`, `n2k-ascii`, or NMEA
  0183, which holds only the sentences the W2K-2 converts, never a PGN.
  Each needs a route from N2K on the *Routing* page, which a change of Tx
  Format can clear; to send, a route to an enabled virtual device (N2KV1,
  "Data1", by default; Data2 and Data3 start disabled, and what is routed
  to them is dropped). It also has a hidden data server, CANDIRECT, on port
  20000, which its web pages do not show: in BST-95 it is `raw-actisense`
  (`tcp://w2k:20000`), in NMEA 0183 it sends every CAN frame as `$MXPGN`,
  for `--data-format mxpgn`. canboat's NGT-1 code works with it: the W2K-2
  refuses the NGT-1's Rx All mode, and canboat goes on in its mode. Seen on
  firmware 3.025 and 3.095; what a W2K-1 does is not known.
- **actisense mxpgn:** an Actisense gateway writes `$MXPGN`'s data in bus
  order, first byte first, where a MiniPlex writes it as one number, last
  byte first; both cite the same rule. Actisense's own example
  (`$MXPGN,01F214,6842,00B004FF7FFFFFFF`, a battery at 12.00 V) and a
  W2K-2 have it so, and a W2K-2 sends only what is written so. Nothing in a
  stream says which, so a capture is read as ShipModul's unless told:
  `canboat convert --from actisense-mxpgn`, `analyzer -format
  ACTISENSE_MXPGN`.
- **seasmart:** `$PCDIN` carries one whole message, with its source but no
  priority or destination, so an addressed PGN goes where the gateway decides.
  Not every gateway takes it to send: Yacht Devices gateways take only 59904,
  127488, 127489, 127493, 127502, 127505 and 127508, and only on a server
  port set to bidirectional; Quark-elec does not document taking it at all.
  Chetco's port is 10001; give the port of the others (Quark-elec 2000, Yacht
  Devices 1456 by default). Received messages get the host's time: the
  sentence's own time stamp is no clock time.
- **seasmart, miniplex:** a sentence whose checksum is wrong is dropped, with
  an error. Some sources write wrong ones (the examples in Quark-elec's
  manuals, Chetco's own switch page): `--ignore-checksum` takes them anyway,
  as do `canboat convert --ignore-checksum` and `analyzer -ignore-checksum`
  for a capture.
- An option only some gateways have (`--bitrate`, `--channel`, `--password`,
  `--mode`, the ISO NAME options) says which in `--help`, and is refused for
  the others.
- The names of canboat 8.3.0 still work: `--kind ngt1`, `w2k-ascii` and
  `ydwg`, and `canboat server --actisense …`, `--ikonvert …`, `--maretron …`
  and `--socketcan …`.

## Plain CAN adapters

A plain USB CAN adapter works on every platform: a Linux SocketCAN
interface, a CANalyst-II such as the Waveshare USB-CAN-B (`04d8:0053`),
which `canboat` opens directly over USB — `canboat interface --kind canalyst usb`
or `canboat server --kind canalyst --device usb` — or a Waveshare USB-CAN-A
(`1a86:7523`) on its serial port, `canboat interface --kind usb-can-a
/dev/ttyUSB0`. On each, canboat is a full NMEA 2000 node: it claims an
address and answers ISO Requests.

### The CANalyst-II on Windows

canboat opens a CANalyst-II (Waveshare USB-CAN-B) directly over USB, and on
Windows that only works when the adapter is bound to Microsoft's **WinUSB**
driver. It does not get WinUSB by itself, and the vendor's software installs
its own driver. So:

1. Run [Zadig](https://zadig.akeo.ie/), select the USB-CAN-B (`04D8 0053`;
   use *Options → List All Devices* if it is not shown), choose **WinUSB** and
   click *Replace Driver*.
2. `canboat interface --kind canalyst usb` and `canboat server --kind canalyst
   --device usb` then work as on Linux and macOS.

With WinUSB bound, the vendor's own tools (USB-CAN Tool, ControlCAN.dll) no
longer see the adapter until its original driver is restored (Device Manager →
*Roll back driver* or *Update driver*). canboat checks the driver, and says so
when it is not WinUSB. This has not yet been tried on real Windows hardware.

## The AOS simulator

The Across Ocean Systems NMEA 2000 / NMEA 0183 Simulator also works as a
gateway: `--kind aos`. It speaks the NGT-1's framing but claims no address
of its own, so canboat is the node, as on a CANalyst-II: `--address` and the
other node options apply, and `--no-claim` leaves it sending from its fixed,
unclaimed address 75. Each kind refuses the other: `--kind aos` gives up on a
gateway that turns out to be a real NGT-1 (it answers Actisense BEM
commands), and `--kind actisense` on one that says it is the simulator.

## Transmit and receive PGN lists (`--tx-pgn`, `--rx-pgn`)

Other devices ask a node which PGNs it sends and reads (PGN 126464).
`--tx-pgn <pgn>[,…] --rx-pgn <pgn>[,…]` (on `canboat server` and `canboat
interface` alike, or
`BridgeConfig::pgn_lists` in the library) names the PGNs your application
sends and reads, so the gateway can answer truthfully. What happens with
them depends on the gateway:

| Gateway | Transmit list | Receive list |
|---|---|---|
| SocketCAN, CANalyst-II, USB-CAN-A | Advertised by canboat itself. Nothing is filtered. | Advertised |
| iKonvert | Written into the gateway, which only transmits what is on it | Written into the gateway, which answers PGN 126464 from it; a filter only with `--rx-filter` (`NORMAL` mode) |
| NGT-1 | Missing PGNs enabled in the gateway's Transmit PGN Enable list (EEPROM, only when something is missing); it only transmits what is on it | Not supported |
| AOS simulator (`aos`) | Advertised by canboat itself, as for a CANalyst-II. Nothing is filtered. | Advertised |
| Others | Ignored, with a warning | Ignored, with a warning |

> [!WARNING]
> ## ON AN iKONVERT OR NGT-1, THE TRANSMIT LIST IS FINAL
>
> **Once you name any transmit PGN (`--tx-pgn`, or
> `pgn_lists.tx`), canboat REFUSES TO SEND EVERY PGN THAT IS NOT ON THE
> LIST.** The gateway only transmits the PGNs in its list, and canboat
> sets that list once, when it starts, so canboat drops the rest up front,
> logging each refused PGN once, instead of handing the gateway frames it
> will reject.
>
> **Decide the complete list before you start.** Only the ISO/NMEA
> network-management PGNs, and the PGNs of the quirks you enabled (such as
> `wmm`'s 127258), are allowed without being named. With no transmit PGN
> named, nothing is refused and the gateway's own list decides.

The PGN lists only advertise; they never filter what you receive. To receive
only some PGNs, use `--rx-filter <pgn>[,…]` (or `BridgeConfig::rx_filter`):
an iKonvert filters in the gateway, and for every other gateway canboat drops
the rest as they arrive. The network-management PGNs (address claims,
requests, product information, …) and canboat's own gateway status always
come through.
