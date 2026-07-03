# Navico / Simrad alarms and commands over NMEA 2000

This note explains how alarms and the broader event/command channel work
on a Navico-family network — Simrad, B&G and Lowrance displays, autopilot
controllers and instruments. It is based on observing real bus traffic while
raising/clearing alarms, running race and trip timers, driving an autopilot, and
triggering a man-overboard, and is meant to help interpret the decoded PGNs in
canboat.

It is **not** part of the NMEA 2000 standard alert mechanism (PGNs 126983/126984/
126985). Navico devices use their own proprietary PGNs, disambiguated by the
manufacturer code in the first two data bytes (Simrad 1857, Navico 275).

> **A note on "alarm" vs "alert".** The NMEA 2000 standard and Navico's newer
> Ethernet/MQTT generation (Neon — the SRX/SR displays) call these *alerts*. The
> older generation that emits the proprietary PGNs documented here (Triton, the
> AP controllers, the NOS displays) calls them *alarms*. canboat follows the
> emitting devices and uses **"alarm"** for these proprietary Simnet/Navico PGNs
> and their fields, reserving "alert" for the standard NMEA alert PGNs.

## PGN 130850 — the Simnet command channel

PGN 130850 is a single Simrad-proprietary message that carries **all** of these
event/command families — alarms, race/trip timers, autopilot commands, MOB and
more. Despite canboat historically modelling it as several differently-shaped
variants, every 130850 frame on the wire uses **one fixed 12-byte layout**:

| Byte | Field | Meaning |
|------|-------|---------|
| 0–1 | Manufacturer + Industry | Simrad 1857, marine |
| 2 | Address | target address; `0xff` = broadcast to all |
| 3 | (reserved) | `0xff` |
| 4 | **Network Group** | display/alarm group: 1 = Default, 2..9 = Group x-1, `0xff` = all |
| 5 | **Command Type** | the discriminator that selects the family (see below) |
| 6 | **Command** | the action within that family |
| 7 | (spare) | `0x00` |
| 8–9 | Target id | meaning depends on family (alarm id, timer index, …) |
| 10–11 | Parameters | family-specific parameters |

There is **no "proprietary id"** in 130850, and the address does not move between
byte positions — earlier canboat variants that placed `Address` at byte 3, or
matched byte 4 as a "proprietary id", were mis-framed and produced wrong decodes
for anything outside the default group.

### Byte 5 — the command type (`SIMNET_EVENT_TYPE`)

Byte 5 is the single discriminator. The values seen on the wire:

| Value | Family | Byte 6 lookup |
|-------|--------|---------------|
| 2 | Follow Up | — |
| 10 | AP Command | autopilot command set |
| 23 | Timer | `SIMNET_TIMER_EVENT` (race / trip) |
| 31 | Siren | — |
| 36 | AIS | — |
| 255 | Alarm | `SIMNET_ALARM_COMMAND` |

The autopilot command type and the alarm/event type are **the same field** —
they are mutually-exclusive values of this one byte, not separate fields. Byte 4
is always the network/display group, independent of which family byte 5 selects.

### Byte 4 — the network group

Byte 4 (`SIMNET_DISPLAY_GROUP`) scopes the command to a display/alarm group:
`1` = Default, `2` = Group 1, etcetera (and `0xff` = all groups, used for ship-wide events
like MOB). A device assigned to "Group 2" emits and reacts to byte 4 = 3; a
listener that assumes byte 4 = 1 will silently miss those frames.

## The alarm families

A single alarm shows up on the wire through up to three cooperating PGNs, each
with a distinct job, all tied together by the same small **alarm id**:

| PGN | canboat name | Manufacturer | Role |
|-----|--------------|--------------|------|
| 130845 | Simnet: Key Value | Simrad (1857) | the **threshold** for each alarm type |
| 130850 | Simnet: Alarm | Simrad (1857) | the alarm **lifecycle command** |
| 130825 | Navico: Alarm | Navico (275) | the alarm **state** (id, state, severity) |
| 130856 | Simnet: Alarm Message | Simrad (1857) | the human-readable **text** for the id |

### Thresholds — PGN 130845 (Simnet: Key Value)

Alarm limits are published as key/value pairs keyed by `SIMNET_KEY_VALUE`. Each
display group broadcasts the current limit for every configured alarm. For
example the "True wind low" limit (key 516) is a `SPEED_UFIX16_CM` value:

```
Simnet: Key Value  Key=516 "True wind low"  Value=12.86   (= 25.0 kn)
```

Changing the limit on one device re-broadcasts the key with the new value, and
the other devices (including autopilot controllers) pick it up — which is why a
threshold edited on one display immediately appears on another. Related limit
keys live next to each other in the lookup, e.g. key 517 "Low boat speed".

### Display unit preferences — PGN 130845 (Simnet: Key Value)

The same Key Value channel also carries the **unit-of-measure preference** for
each quantity a display can show (Distance, Speed, Wind speed, Depth,
Temperature, Volume, Pressure, Barometric pressure, Heading reference). Each
quantity is its own key, but all of them share **command group 20** (the low
byte of the composite key), e.g.:

```
Simnet: Key Value  Display Group=Group 1  Key=5127 "Distance unit"  Value=Kilometers
```

Confirmed live against a Triton head (source 48) by starting a capture
filtered to `130845` from that source and cycling every option in its Units
menu, one at a time, correlating each Set frame with the button just pressed.
Findings:

- Like the alarm thresholds above, a unit change is broadcast per **Display
  Group** — switching the head to a different group (e.g. "Group 2") and
  changing a unit there produces a Set frame tagged with that group, using the
  *same* key and the *same* value encoding, so unit preferences are
  independent per display group rather than a single global setting.
- Distance (key 5127) always broadcasts together with a second key (5174,
  "Distance unit (companion)") that was only ever observed at value 0; its
  purpose is unconfirmed.
- Distance has a second, short-range unit (key 36871, "Distance unit
  (small)") for readouts like depth/anchor/waypoint proximity, distinct from
  the main Distance unit.
- Pressure (key 5163) and Barometric pressure (key 5164) are separate keys —
  the Triton menu has one setting for general/engine pressure and another for
  barometric pressure — and only some of their enum values were observed
  during the capture (see `SIMNET_PRESSURE_UNIT` / `SIMNET_BARO_PRESSURE_UNIT`
  in `lookup.h`), so gaps remain (e.g. mbar/hPa/mmHg/atm variants not yet
  captured for the general Pressure key). See #729.

### The lifecycle command — PGN 130850 (Simnet: Alarm)

When byte 5 = 255 (Alarm), byte 6 carries the lifecycle command
(`SIMNET_ALARM_COMMAND`), and bytes 8–9 carry the alarm id:

| Command | meaning |
|---------|---------|
| 57 Activate | the limit was crossed — raise the alarm |
| 58 Acknowledge | a user acknowledged it |
| 68 Silence | a user muted the audible part |
| 56 Deactivate | the condition cleared — clear the alarm |
| 104 Alarm History | history request |
| 107 MOB Activated | man overboard raised (see MOB below) |
| 108 MOB Cancelled | man overboard cleared |

When an alarm fires, the participating devices broadcast `Activate` for that id
(several devices within a second or two). User actions then broadcast
`Acknowledge` / `Silence`, and the clear broadcasts `Deactivate`. So a single
alarm id is seen cycling Activate → … → Deactivate. **Because these are broadcast
commands, acknowledging an alarm on one display clears it on the others** — that
is the network mechanism behind cross-device acknowledge.

### The alarm record — PGN 130825 (Navico: Alarm)

The alarm *status* is a small fixed record:

| Field | Meaning |
|-------|---------|
| Alarm Id | which alarm (matches the id in 130845 / 130850 / 130856) |
| Alarm State | lifecycle: Normal, Active, Silenced, Acknowledged, Awaiting Acknowledge |
| Alarm Severity | Emergency Alarm, Alarm, Warning, Caution |
| Value | associated value (often 0) |

In practice 130825 mainly re-announces **`Active`** while the condition holds — it
is the *state* view of the alarm. The actual lifecycle (acknowledge, silence,
clear) is driven by the 130850 **commands** above, not by 130825 state changes, so
a listener watching only 130825 sees it stay `Active` even after an acknowledge.
Watch 130850 for the transitions and 130825 for the steady-state severity. The
companion PGN 130856 carries the same id with the human-readable text.

## Timers — PGN 130850, byte 5 = 23

Race and trip timers ride the same 130850 with byte 5 = 23 (Timer); byte 6 is the
timer event (`SIMNET_TIMER_EVENT`):

| Command | meaning |
|---------|---------|
| 61 Race Timer Start | start the race countdown |
| 62 Race Timer Stop | stop it |
| 63 Race Timer Sync | round the countdown to the nearest minute (rolling start) |
| 64 Race Timer Reset | reset it |
| 65 Trip Timer Reset All | reset all trip logs |
| 100 Trip Timer Enable | start a trip log (parameter 1 = trip index) |
| 101 Trip Timer Disable | stop a trip log |

The trip enable/disable commands carry the trip index in the parameter bytes, and
a trip started on one display is picked up network-wide (the distance/time/average
fields then appear on other displays). A *partial* reset of a single field on a
display does **not** go on the wire — only the enable/disable and full-reset
commands are broadcast.

## Man overboard — PGN 130850 + 127233

A man-overboard is an alarm-class event (byte 5 = 255) with byte 6 = **107 MOB
Activated** / **108 MOB Cancelled**, broadcast to all groups (byte 4 = `0xff`,
no alarm id). Unlike depth/wind alarms — which burst from several devices — the
MOB is emitted only by the **originating display** and is not re-broadcast by the
other devices.

The MOB **position** does not ride 130850; it rides the standard NMEA 2000 PGN
**127233 (Man Overboard Notification)**, emitted by the same source at the same
instant. So a complete MOB is a *pair*: 130850 carries the lifecycle
(Activated/Cancelled) and 127233 carries the latitude/longitude. To follow a MOB
in a capture, watch for the 130850 `MOB Activated` and read the position from the
paired 127233.

## Autopilot commands — PGN 130850, byte 5 = 10

Autopilot mode changes (Auto, Wind, Nav, No Drift), heading/course adjustments and
the like ride 130850 with byte 5 = 10 (AP Command). The autopilot computer echoes
accepted commands back on PGN 130851 (AP command reply). These share the same
12-byte frame and the same byte-4 network group as the alarm and timer families.

## Propagation — why an alarm sometimes stays local

An alarm is only network-wide if it is actually broadcast. The same alarm type
can behave differently depending on where it is enabled:

- Raised through a display/instrument that broadcasts: every configured device
  reacts (audible/visual alarm) and the PGNs above appear on the bus.
- Enabled locally on a single controller without broadcasting: it alarms only on
  that device, and other displays stay silent — there is no traffic on the bus.

So when troubleshooting "device A beeps but device B does not", first check
whether the alarm PGNs appear on the bus at all.

## Correlating a capture

To follow an alarm in a capture:

1. Watch **130845** for the configured thresholds (and any live edits).
2. Follow the **130850** commands (Activate / Acknowledge / Silence / Deactivate) —
   note the alarm id (bytes 8–9), the command (byte 6) and the network group (byte 4).
3. Match that id in **130825** for the state and severity (and in **130856** for the
   alarm text).

The same alarm id ties the records together, and the manufacturer code in bytes
0–1 keeps the Simrad and Navico variants of these PGN numbers distinct from the
unrelated uses by other manufacturers.
