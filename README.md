# Power Smart roller shutter control for ESP32 + CC1101 (ESPHome)

This is a from-scratch ESP32 reimplementation of the "Power Smart" Sub-GHz
protocol from your Flipper Zero firmware, packaged as an ESPHome external
component. It transmits Up/Down/Stop commands to Power Smart roller shutter
receivers via a CC1101 module.

Radio handling (SPI, registers, PA table, frequency synthesis, TX/IDLE
state transitions) is delegated to ESPHome's official `cc1101:` component.
This custom component only contains the part that doesn't exist anywhere
else: the Power Smart packet encoder (layout + checksum + Manchester
timing) and a `power_smart.send_command` action. It drives the encoded
bitstream through `remote_transmitter` (the ESP32 RMT peripheral) into the
CC1101's GDO0 pin in async serial OOK mode - the same technique the Flipper
Zero itself uses.

An earlier version of this component talked to the CC1101 directly with its
own hand-written register tables. That was replaced once ESPHome shipped an
official `cc1101:` driver: the radio-specific code is now maintained
upstream (and verifies the chip ID on boot), leaving this component roughly
150 lines smaller and focused solely on the protocol.

## How this was built / validated

I extracted `lib/subghz/protocols/power_smart.c` from your firmware zip and
worked out the packet layout and checksum from the decode logic. Rather than
just trust that reading, I validated it three independent ways:

1. Reconstructed the 4 worked examples in the source file's own comments
   (different button/channel combinations) byte-for-byte from first
   principles, using only `remote_id`/`channel`/`command`.
2. Wrote a Python port of the actual Manchester **decoder** state machine
   (`lib/toolbox/manchester_decoder.c`) and ran it against the raw capture in
   `applications/debug/.../subghz/power_smart_raw.sub` - it successfully
   pulled out 6 identical, checksum-valid packets, confirming the timing
   constants (225us/450us) and checksum formula against a real RF capture,
   not just the source code.
3. Wrote a Python port of the **encoder** (`lib/toolbox/manchester_encoder.c`)
   and confirmed its pulse-train output for that same packet is byte-for-byte
   a superset match of the real captured pulse train.
4. Re-implemented the encoder a third time in standalone C++ (identical logic
   to what's in `power_smart.cpp`) and confirmed it produces the exact same
   754-pulse output as the Python version.
5. After the refactor onto the official `cc1101:` driver, re-ran the C++
   encoder check to confirm the protocol logic was untouched - still the
   same known-good packet (0xFD0337ACAAFCC852) and same 754-pulse train.

So the packet structure, checksum, and timing in this component aren't a
guess - they reproduce a real captured transmission exactly. The whole
config (custom component + official cc1101 block + 72 buttons) was also run
through ESPHome's real config validation and C++ codegen to confirm it
wires up against the actual cc1101 API.

### Packet layout (64 bits)

```
byte0 = 0xFD                           fixed sync
byte1 = K1(1) | CHANNEL(6) | ID_b15(1)
byte2 = ID_b14-8(7) | K2(1)
byte3 = ID_b7-0(8)
byte4 = 0xAA                           fixed sync
byte5:byte6 (16-bit)  = ~(byte1:byte2)
byte7 = 0xFE - byte3
```

- `K1,K2` form a 2-bit command: `1` = Down, `2` = Up, `3` = Stop
- `CHANNEL` is a 6-bit **one-hot** value (exactly one bit set) - this is your
  remote's channel selector, 1-6
- `ID` is a 16-bit value fixed per physical remote

Line coding is standard Manchester at 225us (short) / 450us (long) symbols.
Each 64-bit packet is sent 8x back-to-back in one continuous Manchester
stream, followed by a ~500ms silent gap - this whole unit repeats 10 times by
default, matching what the Flipper Zero itself sends on replay.

RF-wise this is 433.43MHz, OOK/ASK, with the CC1101 in "async serial" mode -
the ESP32 bit-bangs the data pin (via the ESP32's RMT peripheral, through
ESPHome's `remote_transmitter`) and the CC1101 just keys the carrier on/off
accordingly. This is the same technique the Flipper Zero itself uses.

## What you get

A `power_smart:` component that references a `cc1101:` radio and a
`remote_transmitter:`, plus a `power_smart.send_command` action you can call
from any ESPHome automation (e.g. a `button`'s `on_press:`), taking
`remote_id`, `channel` (1-6), and `command` (`UP`/`DOWN`/`STOP`).

`example.yaml` wires this up as 72 template buttons (4 remotes x 6 channels x
3 commands) - since you wanted simple buttons rather than full `cover`
entities with position tracking.

## Wiring

Common 8-pin CC1101 breakout board, 26MHz crystal:

| CC1101 pin | ESP32 pin (example) |
|---|---|
| VCC | 3.3V (**not 5V**) |
| GND | GND |
| SCK | GPIO18 |
| MOSI | GPIO23 |
| MISO | GPIO19 |
| CSN | GPIO5 |
| GDO0 | GPIO4 |
| GDO2 | not used |

GDO0 is the important one - it's the pin the ESP32 actually drives with the
bitstream. Any free GPIO works, just update `remote_transmitter: pin:` and
match it mentally to where you solder GDO0.

## Getting your 4 remote IDs

Use your Flipper Zero to capture a button press from each physical remote
(Sub-GHz -> Read, or pull the "Key:" from an existing saved `.sub` capture),
then run:

```
python3 decode_key.py "FD C1 36 AC AA 3E C9 52"
```

It prints the `remote_id` and `channel` and tells you if the checksum didn't
validate (i.e. you mistyped the hex). Put the `remote_id` into the
`substitutions:` block at the top of `example.yaml`.

Since `channel` in the protocol is one-hot (bit position = channel number),
if a capture ever prints a "non-standard mask", it just means that
particular button press wasn't a plain single-channel command - re-capture
and try again.

## One subtle issue worth knowing about

`send_command()` puts the CC1101 back to idle right after the transmitter's
`perform()` call returns, which only works if that call *blocks* until the
whole repeated transmission is actually done. ESPHome changed
`remote_transmitter`'s default to non-blocking in 2025.11.0, which would
otherwise make `perform()` return almost immediately and cut the RF off
within microseconds of starting. The component forces `non_blocking: false`
on its transmitter via codegen (and `example.yaml` sets it explicitly too,
so it's not a surprise if you inspect the config).

(An earlier self-contained version of this component also had a PATABLE
slot bug and lacked a chip-presence check at boot. Both are now moot - the
official `cc1101:` driver handles PA table setup correctly and verifies the
chip ID on setup.)

## Notes

- **TX power / regulations**: `output_power` in the `cc1101:` block defaults
  to 10dBm here. Australia's 433.05-434.79MHz band is available under an
  ACMA class licence for low-interference devices, but there are conditions
  (EIRP limits, duty cycle) - worth a quick check of the current ACMA LIPD
  class licence rather than assuming, since I haven't verified current
  numbers here.
- **Repeat count**: default 10 (matches Flipper's default replay). Each send
  takes several seconds because of this - if 10 feels excessive once you've
  confirmed reliable reception, `repeat: 5` in the `power_smart:` config will
  speed things up.
- `frequency:` is set to 433.43MHz in the `cc1101:` block to match your
  remotes. If you ever add a remote that doesn't respond, double check its
  `.sub` capture's `Frequency:` line in case it's a different variant.
- This only implements *sending*. Your Flipper Zero remains the best tool for
  capturing/identifying new remotes.
