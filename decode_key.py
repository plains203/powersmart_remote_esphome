#!/usr/bin/env python3
"""
Decode a Flipper Zero "Power Smart" capture (.sub file "Key:" hex value, or
the raw 64-bit int) into the remote_id / channel / command fields needed for
the power_smart ESPHome component.

Usage:
    python3 decode_key.py FDC136ACAA3EC952
    python3 decode_key.py "FD C1 36 AC AA 3E C9 52"

You can get the "Key:" value directly from a .sub file that Flipper Zero
saved after capturing a button press with your remote (Sub-GHz -> Read, or
from Saved captures). It looks like:
    Key: FD C1 36 AC AA 3E C9 52

This also double-checks the packet's checksum, so it will tell you if the
hex you pasted got mangled.
"""
import sys

COMMAND_NAMES = {0: "Unknown", 1: "Down", 2: "Up", 3: "Stop"}


def decode(data: int):
    btn = ((data >> 54) & 0x02) | ((data >> 40) & 0x1)
    serial = ((data >> 33) & 0x3FFF00) | ((data >> 32) & 0xFF)

    remote_id = serial & 0xFFFF
    channel_mask = (serial >> 16) & 0x3F

    # checksum check (same formula the ESPHome component's encoder uses)
    mask64 = (1 << 64) - 1
    data_1 = (data >> 40) & 0xFFFF
    notdata = (~data) & mask64
    data_2 = (notdata >> 8) & 0xFFFF
    data_3 = (data >> 32) & 0xFF
    data_4 = ((notdata & 0xFF) - 1) & 0xFF
    valid = (data_1 == data_2) and (data_3 == data_4)
    header_ok = ((data >> 56) & 0xFF) == 0xFD and ((data >> 24) & 0xFF) == 0xAA

    channel = None
    if channel_mask and (channel_mask & (channel_mask - 1)) == 0:
        channel = channel_mask.bit_length()  # which single bit is set, 1-6

    return {
        "btn": btn,
        "remote_id": remote_id,
        "channel_mask": channel_mask,
        "channel": channel,
        "valid": valid and header_ok,
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    raw = "".join(sys.argv[1:]).replace(" ", "").replace(":", "")
    if raw.lower().startswith("0x"):
        raw = raw[2:]
    try:
        data = int(raw, 16)
    except ValueError:
        print(f"Could not parse '{raw}' as hex")
        sys.exit(1)

    result = decode(data)

    print(f"Packet:      0x{data:016X}")
    print(f"Valid:       {result['valid']}")
    if not result["valid"]:
        print("  (checksum or sync bytes didn't match - double check the hex you pasted)")
    print(f"Command:     {result['btn']} ({COMMAND_NAMES.get(result['btn'], '?')})")
    print(f"Remote ID:   0x{result['remote_id']:04X}")
    if result["channel"] is not None:
        print(f"Channel:     {result['channel']}")
    else:
        print(f"Channel:     non-standard mask 0b{result['channel_mask']:06b} (expected exactly one bit set)")
    print()
    print("ESPHome substitution line:")
    print(f'  remote_N_id: "0x{result["remote_id"]:04X}"')


if __name__ == "__main__":
    main()
