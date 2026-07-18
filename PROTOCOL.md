# Power Smart roller shutter RF protocol

A complete, implementation-oriented description of the **Power Smart** 433 MHz
roller-shutter remote protocol. This document is intended to be sufficient to
re-implement the protocol on any platform (a different MCU, an SDR, an RF
bridge, etc.) without reference to the accompanying ESPHome code.

The description is derived from the open-source Flipper Zero firmware decoder
(`lib/subghz/protocols/power_smart.c`) and verified against real over-the-air
captures. Every constant below has been checked against a working transmitter.

> **Scope.** This covers the *transmitted command* format. It is a fixed-code
> (non-rolling) protocol: a given remote ID + channel + command always produces
> exactly the same bits. There is no rolling counter, cryptography, or
> challenge/response.

---

## 1. Physical layer

| Parameter | Value |
|---|---|
| Carrier frequency | 433 MHz band. Common values: **433.43 MHz**, 433.92 MHz. Confirm per device. |
| Modulation | **OOK / ASK** (on-off keying — carrier is simply switched on and off) |
| Line coding | Manchester (see §4) |
| Short element (`te_short`) | **225 µs** |
| Long element (`te_long`) | **450 µs** (= 2 × short) |
| Bit rate | ~2.2 kbit/s effective (one Manchester bit ≈ 450 µs) |

A receiver/transmitter only needs to key the carrier on and off; there is no
sub-carrier or frequency modulation. On a CC1101 this corresponds to the
"async serial OOK" mode with the MCU driving the data pin directly.

## 2. Frame structure

Each command is a **64-bit packet** (8 bytes), transmitted **MSB first**. The
byte layout is:

```
Byte  Bits (MSB..LSB)          Meaning
----  ----------------------   ------------------------------------------
 0    1 1 1 1 1 1 0 1          0xFD  — fixed sync/preamble byte
 1    K1 C5 C4 C3 C2 C1 C0 I15  command hi-bit, 6-bit channel, ID bit 15
 2    D14 D13 D12 D11 D10 D9 D8 K2   ID bits 14..8, command lo-bit
 3    D7 D6 D5 D4 D3 D2 D1 D0   ID bits 7..0
 4    1 0 1 0 1 0 1 0          0xAA  — fixed sync byte
 5    (~byte1)                  checksum: high byte of ~(byte1:byte2)
 6    (~byte2)                  checksum: low  byte of ~(byte1:byte2)
 7    0xFE - byte3              checksum over the ID low byte
```

Where:

- **`0xFD`** (byte 0) and **`0xAA`** (byte 4) are constant framing bytes. A
  decoder can use `0xFD........AA......` as the frame signature.
- **Channel** (`C5..C0`, byte 1 bits 6..1) is a **6-bit one-hot** value —
  exactly one bit set — selecting channels 1 through 6. Channel *n* is encoded
  as `1 << (n-1)`. So channel 1 = `000001`, channel 2 = `000010`, …,
  channel 6 = `100000`.
- **Command** is a 2-bit value formed from `K1` (byte 1 bit 7) and `K2`
  (byte 2 bit 0), as `command = (K1 << 1) | K2`:

  | Value | K1 | K2 | Meaning |
  |---|---|---|---|
  | 0 | 0 | 0 | Unknown / unused |
  | 1 | 0 | 1 | **Down** |
  | 2 | 1 | 0 | **Up** |
  | 3 | 1 | 1 | **Stop** |

- **Remote ID** is 16 bits, split across the frame:
  - bit 15 → byte 1 bit 0 (`I15`)
  - bits 14..8 → byte 2 bits 7..1 (`D14..D8`)
  - bits 7..0 → byte 3 (`D7..D0`)

## 3. Checksum / integrity fields

Three bytes are fully determined by the others and serve as integrity checks.
A receiver should validate all three; a transmitter must compute them.

```
Let  W = (byte1 << 8) | byte2        # the 16-bit command+channel+id-high word

byte5 = high 8 bits of (~W & 0xFFFF)   # bitwise NOT of W, high byte
byte6 = low  8 bits of (~W & 0xFFFF)   # bitwise NOT of W, low  byte
byte7 = (0xFE - byte3) & 0xFF          # 0xFE minus the ID low byte
```

Equivalently, `byte5:byte6 == ~(byte1:byte2)` (16-bit one's complement), and
`byte7 == 0xFE - byte3`.

A frame is valid if and only if byte0 == 0xFD, byte4 == 0xAA, the two
complement bytes match, and the byte7 relation holds.

## 4. Manchester line coding

The 64 data bits are Manchester-encoded before transmission. This
implementation uses the classic IEEE-802.3-style scheme with a mid-bit
transition, where **runs of identical bits merge into long elements** rather
than always emitting two short half-bits.

Concretely, the bitstream is expressed as a sequence of on/off *elements*, each
either short (225 µs) or long (450 µs):

- A `0`→`1` or `1`→`0` transition within a bit cell encodes the bit.
- Two consecutive equal bits produce one **long** element at the held level,
  followed by a transition; alternating bits produce **short** elements.

The four element types are:

| Element | Level | Duration |
|---|---|---|
| short-low | off (carrier off) | 225 µs |
| long-low | off | 450 µs |
| short-high | on (carrier on) | 225 µs |
| long-high | on | 450 µs |

> **Implementation note — bit inversion.** The reference encoder Manchester-
> encodes the **logical inverse** of each data bit (it feeds `NOT bit` into the
> Manchester encoder). If you build the element stream directly from a
> Manchester-encoder library, feed it the inverted bits to match on-air
> behaviour. If you instead precompute the element stream from a known-good
> capture (recommended for verification), you can ignore this and just
> reproduce the captured timing.

The simplest robust way to verify your Manchester implementation is to encode a
known packet and compare the resulting element durations against a real
capture of the same remote/channel/command.

## 5. Repetition and framing in time

A single logical command is not sent once. The on-air structure is:

1. The 64-bit packet is Manchester-encoded and sent **8 times back-to-back**
   as one continuous element stream (no gap between these 8 inner copies).
2. That whole burst is followed by a silent **inter-repeat gap** of
   `te_long × 1111 = 499 950 µs ≈ 500 ms`.
3. Steps 1–2 form one "repeat unit" (~730 ms total: ~230 ms of data + ~500 ms
   gap). The repeat unit is transmitted **N times** (typically 3–10). A real
   remote held down simply keeps emitting repeat units.

More repeats improve reliability at the cost of time. Receivers act on the
first cleanly-decoded, checksum-valid packet, so a handful of repeats is
usually enough. Holding a button (e.g. for pairing) is emulated by sending many
repeat units.

Approximate durations:

| Repeats | Total on-air time |
|---|---|
| 1 | ~0.73 s |
| 3 | ~2.2 s |
| 8 | ~5.8 s |
| 10 | ~7.3 s |

## 6. Worked example

Command: **remote ID `0x9BAC`, channel `1`, command `Up` (2)**.

Encoding steps:

```
command = Up = 2  ->  K1 = 1, K2 = 0
channel = 1       ->  one-hot mask = 000001
remote_id = 0x9BAC = 1001 1011 1010 1100
  ID bit15 (I15)      = 1
  ID bits14..8        = 001 1011  (0x1B)
  ID bits7..0 (byte3) = 1010 1100 (0xAC)

byte0 = 0xFD
byte1 = K1<<7 | mask<<1 | I15 = 1 0000010 1 = 1000 0011 = 0x83
byte2 = (IDhi<<1) | K2        = 0011011 0   = 0011 0110 = 0x36
byte3 = 0xAC
byte4 = 0xAA
W     = 0x8336 ; ~W = 0x7CC9
byte5 = 0x7C
byte6 = 0xC9
byte7 = 0xFE - 0xAC = 0x52
```

Resulting 64-bit packet:

```
FD 83 36 AC AA 7C C9 52     (0xFD8336ACAA7CC952)
```

This is the exact byte sequence a real Power Smart remote emits for
ID 0x9BAC, channel 1, Up — verified against hardware.

## 7. Decoding

To decode a received transmission:

1. Demodulate OOK to a sequence of on/off element durations.
2. Classify each element as short (~225 µs) or long (~450 µs), with a tolerance
   window of roughly ±100–150 µs.
3. Run a Manchester decoder over the element stream to recover bits (remember
   the inversion note in §4 if matching the reference exactly).
4. Search the bitstream for the `0xFD … 0xAA` frame signature and extract 64
   bits.
5. Validate the three checksum relations (§3). Discard frames that fail.
6. Extract command, channel, and ID per §2.

Because the packet repeats many times per transmission, a decoder can require
several identical valid decodes before acting, which rejects noise cheaply.

## 8. Remote ID validity (receiver pairing)

The protocol itself permits any 16-bit ID. However, **the shutter receivers
tested for this project only pair with IDs whose low byte matches a specific
pattern**. On that hardware, a valid ID's low byte (`byte3` / ID bits 7..0)
must be one of:

```
0xA2 0xA4 0xA6 0xA8 0xAA 0xAC 0xAE 0xB2 0xB4 0xB6 0xB8 0xBA 0xBC 0xBE
```

i.e. binary `101x xxx0`: even, in the `0xA0–0xBE` range, with at least one of
bits 1–3 set (this excludes `0xA0` and `0xB0`). The high byte may be anything.

This is an **empirically-derived property of the receivers tested**, not part
of the wire protocol, and it may differ on other Power Smart variants. It only
matters if you want to *invent* new IDs to pair; IDs read from existing physical
remotes always work. If re-implementing on different hardware, treat this rule
as a starting hypothesis to re-test, not a guarantee.

## 9. Summary for implementers

- 64-bit fixed-code packet, MSB first, framed by `0xFD` … `0xAA`.
- Fields: 2-bit command (Down/Up/Stop), 6-bit one-hot channel, 16-bit ID.
- Three derived checksum bytes (two one's-complement, one subtractive).
- Manchester line coding at 225 µs / 450 µs, OOK on 433 MHz.
- Packet sent 8× per burst, burst repeated N× with a ~500 ms gap between bursts.
- No rolling code — replay and synthesis both work, subject to receiver ID
  pairing.
