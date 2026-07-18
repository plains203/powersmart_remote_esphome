# Power Smart roller shutter control for ESPHome (ESP32 + CC1101)

An ESPHome external component that emulates a **Power Smart** roller shutter
remote. With an ESP32 and a cheap CC1101 433 MHz module, it can drive Power
Smart shutter motors (Up / Stop / Down, across 6 channels per remote ID)
directly from Home Assistant.

It reimplements the Power Smart Sub-GHz protocol — packet layout, checksum,
and Manchester line coding — as a small, self-contained encoder. All the
radio work (SPI, registers, PA table, frequency synthesis, TX/idle
transitions) is handled by ESPHome's official [`cc1101:`](https://esphome.io/components/cc1101/)
component, so this project stays focused on just the protocol.

> **Status / scope.** This is a community reverse-engineering project, not an
> official product. It only *transmits* (it emulates a remote); it does not
> receive or decode. It has been tested against real Power Smart hardware, but
> your motors, frequency, and local RF regulations may differ — see
> [Regulatory note](#regulatory-note).

---

## Contents

- [Hardware needed](#hardware-needed)
- [How it works](#how-it-works)
- [Wiring](#wiring)
- [Installation](#installation)
- [Configuration](#configuration)
- [Finding your remote IDs](#finding-your-remote-ids)
- [Creating new (virtual) remote IDs](#creating-new-virtual-remote-ids)
- [Helper scripts](#helper-scripts)
- [Tuning and behaviour notes](#tuning-and-behaviour-notes)
- [Troubleshooting](#troubleshooting)
- [Protocol reference](#protocol-reference)
- [Regulatory note](#regulatory-note)
- [Credits and licence](#credits-and-licence)

---

## Hardware needed

- An **ESP32** board (classic ESP32, ESP32-S2, ESP32-S3, etc.). Any board
  ESPHome supports with enough free GPIOs for SPI + one data pin.
- A **CC1101 433 MHz module**. Common 8-pin eBay/AliExpress breakouts work, as
  do SMA-connector modules like the Ebyte E07-M1101D. Make sure the module's
  frequency band matches your shutters (usually 433 MHz).
- Basic soldering tools and jumper wires.
- A way to read your existing remotes' IDs — a **Flipper Zero** or an RF
  capture tool (e.g. a Broadlink RM4) is the easiest. See
  [Finding your remote IDs](#finding-your-remote-ids).

You do **not** need to keep any special tool connected at runtime — the ESP32
does everything once configured.

## How it works

The component builds a 64-bit Power Smart packet from a `remote_id`,
`channel`, and `command`, encodes it as Manchester at the correct timing, and
streams the resulting on/off pattern out through ESPHome's
`remote_transmitter` (the ESP32 RMT peripheral). That data line drives the
CC1101's `GDO0` pin while the radio is in **async serial OOK** mode, keying the
433 MHz carrier on and off — exactly how a Flipper Zero replays these signals.

A `power_smart.send_command` action is exposed, so you wire up whatever
entities you like (buttons, covers, scripts) and call the action from their
automations. The included `example.yaml` sets up simple template buttons.

## Wiring

The CC1101 talks to the ESP32 over **SPI** (`SCK`, `MOSI`, `MISO`, `CSN`), plus
one extra pin, **`GDO0`**, which carries the transmitted bitstream. `GDO2` is
not used.

> ⚠️ **Power the CC1101 from 3.3 V, never 5 V.** It is a 3.3 V part.

You can use almost any free GPIOs — just make the YAML match your wiring. A
known-good example on a **Wemos/LOLIN S2 Mini** with an **Ebyte E07-M1101D**
module (this is what `example.yaml` uses):

| CC1101 / E07 pin | Signal | S2 Mini GPIO | ESPHome key |
|---|---|---|---|
| VCC | 3.3 V | `3V3` | — |
| GND | GND | `GND` | — |
| GDO0 | RF data out | `GPIO4` | `remote_transmitter: pin:` |
| CSN | SPI chip select | `GPIO12` | `cc1101: cs_pin:` |
| SCK | SPI clock | `GPIO7` | `spi: clk_pin:` |
| MOSI | SPI MOSI | `GPIO11` | `spi: mosi_pin:` |
| MISO | SPI MISO | `GPIO9` | `spi: miso_pin:` |
| GDO2 | (unused) | — | — |

On a classic ESP32 dev board, typical pins would be `SCK=GPIO18`,
`MOSI=GPIO23`, `MISO=GPIO19`, `CSN=GPIO5`, `GDO0=GPIO4`. Pick whatever is
convenient and update the YAML.

> **Module pinout warning.** Some CC1101 breakouts (the Ebyte E07 in
> particular) have shipped with different pin orders across revisions. Check
> the silkscreen on *your* board against the labels above. If the `cc1101:`
> component logs a chip-ID mismatch at boot, a swapped `MISO`/`MOSI` or `CSN`
> is the usual cause.

## Installation

This is an ESPHome **external component**, loaded straight from GitHub — no
manual file copying needed. Add this to your device YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/plains203/powersmart_remote_esphome
      ref: main
    components: [power_smart]
```

Then add the `cc1101:`, `remote_transmitter:`, and `power_smart:` blocks (see
[Configuration](#configuration)). The easiest path is to copy
[`example.yaml`](example.yaml) and adapt it.

> **Pin to a version for reliability.** `ref: main` tracks the latest commit.
> For a setup you rely on, point `ref:` at a specific tag or commit so an
> upstream change can't alter your firmware unexpectedly.

## Configuration

Minimal working configuration (see [`example.yaml`](example.yaml) for a
complete, commented device config with buttons):

```yaml
spi:
  clk_pin: GPIO7
  mosi_pin: GPIO11
  miso_pin: GPIO9

cc1101:
  id: cc1101_radio
  cs_pin: GPIO12
  frequency: 433.43MHz      # set to YOUR shutters' frequency
  modulation_type: "ASK/OOK"
  output_power: 10          # dBm — see the regulatory note
  manchester: false         # Manchester is done in software; leave this off

remote_transmitter:
  id: cc1101_tx
  pin: GPIO4                # -> CC1101 GDO0
  carrier_duty_percent: 100%
  non_blocking: true        # required — see "Tuning and behaviour notes"

power_smart:
  id: shutter_radio
  cc1101_id: cc1101_radio
  remote_transmitter_id: cc1101_tx
  repeat: 3                 # how many times each command is repeated
  max_queue_depth: 8        # commands received while busy are queued
```

### `power_smart:` options

| Option | Default | Description |
|---|---|---|
| `cc1101_id` | *(required)* | ID of the `cc1101:` radio to transmit through. |
| `remote_transmitter_id` | *(required)* | ID of the `remote_transmitter:` driving `GDO0`. |
| `repeat` | `10` | How many times each command burst is sent. Higher = more reliable but slower. See [tuning](#tuning-and-behaviour-notes). |
| `max_queue_depth` | `8` | Commands that arrive while the radio is busy are queued (up to this many) and sent in order. Oldest is dropped if the queue overflows. |

### The `send_command` action

Call it from any automation. `channel` is 1–6; `command` is `UP`, `DOWN`, or
`STOP`:

```yaml
button:
  - platform: template
    name: "Living Room Up"
    on_press:
      - power_smart.send_command:
          remote_id: 0x9BAC
          channel: 1
          command: UP
```

`remote_id`, `channel`, and `command` are all templatable, so they can be
lambdas if you want dynamic behaviour.

### Optional: expose "is transmitting" to Home Assistant

```yaml
binary_sensor:
  - platform: template
    name: "Shutter Radio Transmitting"
    lambda: 'return id(shutter_radio).is_busy();'
```

## Finding your remote IDs

Each physical Power Smart remote has a 16-bit ID, and each of its 6 channels
controls one shutter. To emulate a remote, you need its ID.

1. Capture a button press from the physical remote using a Flipper Zero
   (Sub-GHz → Read) or another 433 MHz capture tool, and note the raw key —
   e.g. `FD C1 36 AC AA 3E C9 52`.
2. Decode it with the included helper:

   ```
   python3 decode_key.py "FD C1 36 AC AA 3E C9 52"
   ```

   It prints the `remote_id`, `channel`, and `command`, and verifies the
   checksum (so it tells you if the hex was mistyped).
3. Put the `remote_id` into your YAML (the `substitutions:` block in
   `example.yaml` is set up for this).

Repeat for each physical remote you want to emulate.

## Creating new (virtual) remote IDs

You may want *extra* remotes that don't correspond to a physical unit — e.g.
to control more shutters, or to keep Home Assistant's control separate from
your handheld remotes. To do that you pair a **new** ID into the shutter's
receiver using its normal learning procedure, then drive that ID from
ESPHome.

**Important:** on the hardware this was developed against, the receiver does
**not** accept arbitrary IDs. Testing revealed that a valid ID's **low byte**
must match a specific pattern:

> The **low byte** must be one of:
> `0xA2 0xA4 0xA6 0xA8 0xAA 0xAC 0xAE 0xB2 0xB4 0xB6 0xB8 0xBA 0xBC 0xBE`
> (in bits: `101xxxx0` — even, in the `0xA0–0xBE` range, excluding `0xA0`/`0xB0`).
> The **high byte** can be anything (`0x00–0xFF`).

That yields ~3,584 usable IDs. The `generate_ids.py` helper produces valid
IDs for you:

```
python3 generate_ids.py -n 6 --yaml      # 6 valid IDs as YAML substitution lines
python3 generate_ids.py --check 0xE0AE   # test whether a specific ID is valid
```

To actually use a new ID you must **pair it** with the shutter's receiver.
The exact procedure varies by motor; a common one is: power-cycle the motor in
a set sequence, then hold `STOP` to enter pairing mode, then send `UP` to
confirm. Consult your motor's manual. Once paired, the ID behaves like any
other.

> ⚠️ The low-byte rule above was derived empirically from one set of hardware.
> It may differ for other Power Smart variants. If a valid-by-the-rule ID
> won't pair on your motors, that's useful data — please open an issue with
> which IDs did and didn't work.

## Helper scripts

Both are plain Python 3 scripts (no dependencies beyond the standard library):

- **`decode_key.py`** — decode a captured raw key into `remote_id` / `channel`
  / `command`, with checksum validation.
- **`generate_ids.py`** — generate valid remote IDs, check a specific ID, or
  emit ready-to-paste YAML. Run with `-h` for all options. Edit the
  `EXISTING_IDS` set near the top to record IDs you've already assigned so it
  never suggests a duplicate.

## Tuning and behaviour notes

- **`repeat` and speed.** Each command is sent as a burst of repeats with a
  short gap between them (mirroring how a held-down real remote behaves). One
  repeat is ~0.73 s, so `repeat: 3` ≈ 2.2 s and the default `repeat: 10` ≈
  7.3 s. Fewer repeats = snappier response; more = more robust reception. Start
  around `3` and increase only if commands are missed.
- **Non-blocking is required.** A full burst takes seconds, so the component
  transmits in the background via the RMT peripheral. Keep
  `non_blocking: true` on the `remote_transmitter` (it's the ESPHome default).
  Blocking that long would trip the ESP32 task watchdog and reboot the device.
- **Command queueing.** If commands arrive faster than they can transmit,
  they're queued (up to `max_queue_depth`) and sent in order, so rapid button
  presses aren't lost. The `is_busy()` state (see above) lets your UI reflect
  this.
- **One remote ID = 6 channels.** You don't need a separate ID per shutter;
  one ID gives you channels 1–6. Use extra IDs when you need more than 6, or to
  separate groups logically.

## Troubleshooting

- **Boot log shows a CC1101 chip-ID mismatch** → wiring problem. Recheck SPI
  pins, especially `MISO`/`MOSI`/`CSN`, and confirm the module has 3.3 V power.
- **Device reboots when sending** → make sure `non_blocking: true` is set on
  the `remote_transmitter`.
- **Shutters don't respond at all** → confirm the `cc1101: frequency:` matches
  your remotes. Some units use 433.42 MHz, 433.92 MHz, etc. Check the
  `Frequency:` line of a Flipper `.sub` capture from the real remote.
- **A new invented ID won't pair** → verify it passes `generate_ids.py
  --check`, and re-check your motor's pairing steps. If a rule-valid ID still
  won't pair, see the note under
  [Creating new remote IDs](#creating-new-virtual-remote-ids).
- **Commands feel unreliable at range** → increase `repeat`, check antenna
  connection, and consider a module with an SMA antenna.

## Protocol reference

For anyone wanting to understand or extend the encoding. Each command is a
64-bit packet, MSB first:

```
byte0 = 0xFD                            fixed sync
byte1 = K1(1) | CHANNEL(6) | ID_b15(1)
byte2 = ID_b14..8(7) | K2(1)
byte3 = ID_b7..0(8)
byte4 = 0xAA                            fixed sync
byte5:byte6 (16-bit) = ~(byte1:byte2)   check (bitwise inverse)
byte7 = 0xFE - byte3                    check
```

- `K1,K2` form a 2-bit command: `1` = Down, `2` = Up, `3` = Stop.
- `CHANNEL` is a 6-bit **one-hot** value (exactly one bit set) selecting
  channel 1–6.
- `ID` is the 16-bit remote identifier.

Line coding is standard **Manchester** with ~225 µs (short) / ~450 µs (long)
symbols. Each 64-bit packet is sent 8× back-to-back in one continuous stream,
followed by a ~500 ms gap; that whole unit is repeated `repeat` times. RF is
OOK/ASK with the CC1101 in async-serial mode.

For a full, standalone specification — enough to re-implement the protocol on
another platform (different MCU, SDR, RF bridge, etc.) — see
[**PROTOCOL.md**](PROTOCOL.md). It documents the exact bit layout, checksum
maths, Manchester timing, repetition structure, and a fully worked example.

## Regulatory note

The 433 MHz band is licence-exempt in many regions but with **conditions**
(power/EIRP limits, duty cycle, etc.) that vary by country. `output_power`
defaults to 10 dBm here as a reasonable starting point, but **you are
responsible for ensuring your transmissions comply with local regulations**
(e.g. the ACMA LIPD class licence in Australia, ETSI EN 300 220 in the EU, FCC
Part 15 in the US). Reduce power and duty cycle if in doubt.

## Credits and licence

The Power Smart protocol implementation is derived from the open-source
[Flipper Zero firmware](https://github.com/flipperdevices/flipperzero-firmware)
(`lib/subghz/protocols/power_smart.c`) and validated against real captures.
Radio handling uses ESPHome's official `cc1101:` component.

See [LICENSE](LICENSE) for licence terms. Contributions, bug reports, and
especially data points about ID pairing on other Power Smart hardware are
welcome — please open an issue or PR.
