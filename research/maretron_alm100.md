# Maretron ALM100 annunciator: which PGN actually drives it

This note records what a live Maretron ALM100 reports about itself, and how it is
actually driven. Two questions the database left open are now settled:

- PGN 130824 (`Maretron: Annunciator`) is **not** a command channel. It is a status
  report, and writing it at the device does nothing.
- The annunciator is commanded by a **126208 NMEA Command group function addressed to the
  device, targeting PGN 130824 and writing its fields**. This was confirmed by capturing
  the vessel's own configuration software doing it, then reproducing both the sounding and
  the silent state from canboat against real hardware.

## The device

An ISO Request (59904) for 126996 identifies it:

```
Product Code = 8165; Model ID = ALM100; Software Version Code = 1.0.6;
Model Version = 1.0; Model Serial Code = 1462536; Certification Level = Level A;
Load Equivalency = 2
```

Its ISO Address Claim carries a two-field fingerprint that identifies an annunciator
without needing the product code at all:

```
Manufacturer Code = Maretron; Device Function = Alarm Enunciator;
Device Class = Safety systems
```

`MARETRON_PRODUCT_CODE` already contained `8165: ALM100`, so 126996 named the device
correctly before any of this work.

## 130817 Annunciator Capabilities — confirmed

The existing definition was never backed by a capture: every 130817 frame in the sample
corpus belonged to another manufacturer. Soliciting it directly from the ALM100 produces
the first real Maretron evidence, and the definition decodes it exactly as written:

```
Annunciator Instance = 0; Number of Tones = 5;
Tone 1 = 0; Tone 2 = 1; Tone 3 = 2; Tone 4 = 3; Tone 5 = 4
```

| Byte | Field | Value |
|------|-------|-------|
| 0–1 | Manufacturer + Industry | Maretron (137), Marine (4) |
| 2 | Annunciator Instance | 0 |
| 3 | Number of Tones | 5 |
| 4.. | Tone, UINT16 × count | 0, 1, 2, 3, 4 |

So an ALM100 supports five tones, numbered 0 to 4. The tones are reported as bare
integers; the device does not name them on the bus, so there is no tone enumeration to
add — a lookup here would be invented, not observed.

Capture: `samples/maretron-alm100-capabilities.raw`.

## 130824 Annunciator — a status report, not a command

Idle, the ALM100 emits 130824 about every ten seconds with a constant payload:

```
89 98 00 00 ff ff ff ff ff
```

This is byte-for-byte what M/V Dirona's annunciator emitted in 2016, and it does not vary
over a three minute idle capture — which is why this PGN looked like a contentless beacon
for a long time.

**It is not contentless. It reports the sounding state, and it had simply never been
captured while the annunciator was actually sounding.** With N2KView driving the device
for a real alarm, the payload becomes:

```
89 98 00 64 04 00 17 0d 50        at 1 Hz, instead of one frame per 10 s
```

The trailing `0d 50` is **alert id 20493, little-endian** — precisely the alert that was
active in 130819 and 126983 at that moment. So the last field identifies *which* alert the
annunciator is sounding for. When that alert was acknowledged the device fell silent and
reverted to the idle payload and the ten second rate; in 126983 the same transition shows
as `Acknowledge Status` going No→Yes and `Alert State` going Active→Acknowledged. See
"Stopping it" below for what that does and does not establish.

| Byte | Field | Idle | Sounding |
|------|-------|------|----------|
| 0–1 | Manufacturer + Industry | Maretron, Marine | same |
| 2 | Annunciator Instance | 0 | 0 |
| 3 | **Annunciator State** | 0 | **100** |
| 4–5 | **Pattern** | unavailable | 4 |
| 6 | *unidentified* | unavailable | 23 |
| 7–8 | **Alert ID** | unavailable | **20493** |

The field *count* is not guesswork. Asked for this PGN, the device replies with a 126208
Acknowledge group function reporting exactly **five parameters**, matching the five fields
carved after the proprietary preamble. The names above were confirmed by commanding each
field over 126208 and watching the device echo the value back in its own status — see
"The command channel" below. Byte 6 was 23 in every frame ever observed and is
unidentified.

Capture: `samples/maretron-alm100-sounding.raw`.

### It is not written directly

The ALM100 reports 130824 only on its *transmit* list, its receive list does not contain
it, and writing it at the device — broadcast or addressed — does nothing. It is commanded
indirectly, through 126208, as described below.

Two independent lines of evidence show it is not itself a writable channel.

**The device says so.** Asked for its PGN list (126464), the ALM100 answers:

- *Transmit* list: 59392, 59904, 60160, 60416, 60928, 61184, 65282, 126208, 126464,
  126720, 126996, 126998, **130818, 130825, 130817, 130824**
- *Receive* list: 59392, 59904, 60160, 60416, 60928, 61184, 65240, 126208, **130819**

130824 appears only on the transmit side. The receive list ends at **130819
(`Maretron: Alert Transmission`)** and does not contain 130824 at all.

**Writing it does nothing.** Sending 130824 with tone values 1 through 4, both broadcast
and addressed to the device, produced no sound and no change in the beacon. The frames
demonstrably reached the bus — they were observed coming back with the gateway's source
address and the injected payload — so this is a rejection by the device, not a failed
transmit. Note that the IPG100 rewrites the destination to broadcast, so an "addressed"
130824 arrives as `dst=255`.

Captures: `samples/maretron-alm100-pgnlist.raw`,
`samples/maretron-alm100-capabilities.raw`.

## Confirmation from the firmware

The ALM100 1.0.6 firmware image contains a PGN descriptor table that names these PGNs in
Maretron's own words — the strings `Annunciator Capabilities`, `Alert Transmission` and
`Annunciator` sit consecutively, and the three descriptor records point at them in that
order. Each record carries an interval field:

| PGN | Firmware name | Interval field |
|-----|---------------|----------------|
| 130817 | Annunciator Capabilities | 0 |
| 130819 | Alert Transmission | 0 |
| 130824 | Annunciator | **1000** |

Only 130824 has a non-zero interval. The firmware classifies it as a timed periodic
broadcast, which is independent confirmation of the beacon behaviour observed on the wire.

## 130819 is accepted, but an alert alone does not sound it

The ALM100 lists 130819 on its receive side, so that is the only one of the three
annunciator PGNs it will take from the bus. Replaying a captured, well-formed 130819
alert at it — unique alert id, occurrence number 1, on an otherwise completely silent bus
— produced **no sound and no response of any kind**. Repeating the frame at 1 Hz for
several seconds changed nothing; the device carried on emitting only its 10-second
beacon.

This result is worth stating carefully, because an earlier round of testing appeared to
show the opposite. During that round the vessel's N2KView installation was running and
continuously broadcasting eight genuine `Emergency Alarm` / "Tripped" alerts (ids 20493,
20494, 20496, 20497, 20499, 20500, 20501 and 20502 — the range is not contiguous) plus two
state alerts, at roughly 1 Hz. The annunciator was responding to
*those*, not to the injected frames. Stopping N2KView removed all 130819 traffic from the
bus and the annunciator fell silent, at which point the injected frames demonstrably did
nothing. Any tone-selection experiment run against that noisy background is likewise
uninformative, so none of it is recorded here.

The lesson generalises: when reverse-engineering an alarm device, first confirm the bus is
silent, then confirm your stimulus is the only thing on it.

## The command channel: 126208 targeting 130824

Replaying alert traffic never worked — not the 126988/126983/130819 triplet, not a
byte-for-byte replay of frames captured while the device *was* sounding, not an
acknowledged-to-active transition, not the full five-PGN set including 126986 Alert
Configuration. All were verified onto the bus; none produced any reaction. That was the
clue: **the trigger is not in the alert frames at all.**

Capturing the vessel's configuration software from a cold start, with the bus otherwise
silent, showed what it actually sends. Immediately on connecting it addresses the
annunciator directly with a series of **126208 NMEA Command group functions targeting PGN
130824**, one per alert it wants annunciated:

```
,3,126208,128,164,18,01,08,ff,01,f8,05,04,00,05,00,06,ff,ff,07,17,08,16,50
                        |  \______/    |  \___/ \______/ \___/ \______/
                        |   PGN 130824 |  state  pattern  fld7   alert id
                        |              field 4 = instance
                        Function Code 1 = Command
```

The parameter pairs are `<field number> <value>`, matching 130824's fields:

| Pair | Field | Arming value | Sounding value |
|------|-------|--------------|----------------|
| `04 00` | Annunciator Instance | 0 | 0 |
| `05 nn` | Annunciator State | `00` | `64` (100) |
| `06 nn nn` | Pattern | `ff ff` | `04 00` |
| `07 17` | *unidentified* | 23 | 23 |
| `08 nn nn` | Alert ID | e.g. `16 50` = 20502 | `0d 50` = 20493 |

Nine such commands went out at startup, one for each alert id the operator had bound to
the annunciator — the eight "Tripped" ids listed above, plus 8204. Those carry
`state = 0`; they register the binding.
The command that actually sounds the device is identical but carries `state = 100` and a
pattern.

### Reproduced from canboat

Both directions were then driven from this repository against the real device, with the
vessel's software stopped and the bus silent:

```sh
IPG=tcp://ipg100.example.lan:6543

# sound it
echo ",3,126208,0,164,18,01,08,ff,01,f8,05,04,00,05,64,06,04,00,07,17,08,0d,50" \
  | maretron-ipg -w -q $IPG

# stop it
echo ",3,126208,0,164,18,01,08,ff,01,f8,05,04,00,05,00,06,ff,ff,07,17,08,0d,50" \
  | maretron-ipg -w -q $IPG
```

The first makes the ALM100 sound and its 130824 status switches to the 1 Hz sounding form
echoing the commanded values; the second silences it and returns it to the 10 second idle
form. Note the destination is preserved for 126208 (`dst=164`), unlike 130824.

### Pattern, not pitch

Sweeping the Pattern field through 0, 1, 2, 3 and 4 — the five entries 130817 advertises —
produces audibly different results, but they differ in **beep cadence rather than pitch**:
single beeps versus more frequent repeated beeps. Each commanded value is echoed back
distinctly in the device's own 130824 status, so the mapping is certain even though naming
the five patterns would be guesswork. For that reason no lookup is added for them.

Capture: `samples/maretron-alm100-command.raw`.

## Stopping it

There are two ways to silence the annunciator, and both were observed.

**Directly.** A 126208 command carrying Annunciator State 0 and Pattern unavailable
silences it immediately. This was driven from canboat against the real device — see
"Reproduced from canboat" above — so it is proven rather than inferred, and it is the
mechanism a consumer that already commands the annunciator would use.

**By acknowledging the alert.** When the vessel's own software was driving the device, the
operator acknowledged alert 20493: 126983 for that id flipped `Acknowledge Status` No→Yes
and `Alert State` Active→Acknowledged, and in the same second 130824 dropped from its 1 Hz
sounding payload back to the idle one. 126983 advertises `Acknowledge Support = Yes` for
these alerts.

Note what that second observation does and does not establish. The annunciator was being
driven by software that was also watching the alert state, so the correlation is real but
the causal step in between — whether the acknowledgement reached the annunciator directly,
or whether the driving software noticed it and sent a state 0 command — was not isolated.
No 126984 Alert Response frame was captured in that window, and 126984 does not appear on
the ALM100's receive list, so acknowledgement most likely acts through the controlling
software rather than on the annunciator itself.

## What remains open

The binding turned out to live in 126208, not in 126720 — the alert-to-annunciator
association is established by the same Command group function that sounds the device,
carrying `state = 0`. So no proprietary configuration exchange is needed to drive an
ALM100, and 126720 opcodes 64/65 (Write/Read Alert Config) were never involved in any of
the observed traffic. Bare 126720 opcode requests (65, 2 and 0) addressed to the device
were ignored; whatever they are for, it is not this.

Two smaller things are still unidentified:

- **Field 7** of 130824, which was 23 in every frame ever observed, from the vessel's own
  software and in the commands replayed here alike. Nothing varied it, so nothing
  constrains it.
- **Names for the five patterns.** Values 0 to 4 are confirmed distinct and audibly
  different in cadence, but the device does not name them on the bus, so a lookup would be
  invented rather than observed.

### The Maretron extension

The 128-bit extension is not fully carved, but its tail is now clear. On a live bus the
last bytes are consistently a canboat-style counted ASCII string — a length byte holding
the text length **plus two**, then `01`, then the text:

| Bytes | Text | Length byte |
|-------|------|-------------|
| `32 00 09 01 54 72 69 70 70 65 64` | `Tripped` (7) | 9 |
| `32 00 05 01 4f 66 66` | `Off` (3) | 5 |
| `32 00 04 01 4f 6e` | `On` (2) | 4 |

That is exactly canboat's `STRING_LAU` encoding. What the leading two bytes select is
unknown. The three rows above come from the vessel's own alert traffic, where they were
always `32 00`; the frame stored in `samples/maretron-alm100-alert.raw` is one that was
replayed with `01 00` in that position instead, so the prefix is at least not fixed by the
protocol. Nothing observed constrains what it means.

Capture: `samples/maretron-alm100-alert.raw` (prefix `01 00`); the `32 00` rows are from
the live alert traffic described above.

## Method

Everything here is reproducible with the tools in this repository:

```sh
IPG=tcp://ipg100.example.lan:6543

# passive capture
maretron-ipg -r -q $IPG > capture.raw

# solicit a PGN from the device (canboat encodes, the IPG segments and addresses)
canboat format-message isoRequest --dst 164 pgn=130817 | maretron-ipg -p -q $IPG
canboat format-message isoRequest --dst 164 pgn=126464 | maretron-ipg -p -q $IPG
```

The 126464 PGN-list request is worth reaching for early on any unfamiliar device: it
answers "can I command this thing, and with what?" directly, and it saved carving a
heartbeat here.
