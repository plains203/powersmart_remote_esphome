#!/usr/bin/env python3
"""
Generate valid Power Smart remote IDs.

VALIDATED ID RULE (confirmed against 12 pair / no-pair tests on real hardware):
    The LOW BYTE of a remote ID must be one of these 14 values:
        0xA2 0xA4 0xA6 0xA8 0xAA 0xAC 0xAE 0xB2 0xB4 0xB6 0xB8 0xBA 0xBC 0xBE
    In bit terms the low byte must match 101xxxx0, i.e. it must be:
        - even (bit 0 == 0)
        - have bits 7 and 5 set and bit 6 clear (so 0xA0-0xBF range)
        - have at least one of bits 1..3 set (rules out 0xA0 and 0xB0)
    The HIGH BYTE can be anything (0x00-0xFF), giving 14 * 256 = 3584 valid IDs.

An ID whose low byte is outside that set will not pair with the receiver.

Usage:
    python3 generate_ids.py                 # print 10 random valid IDs
    python3 generate_ids.py -n 25           # print 25 random valid IDs
    python3 generate_ids.py --all           # print every valid ID (3584 of them)
    python3 generate_ids.py --prefix 0xE0   # only IDs with high byte 0xE0 (16 of them)
    python3 generate_ids.py -n 6 --yaml     # emit ready-to-paste ESPHome substitutions
    python3 generate_ids.py --check 0xE0AE  # test whether a specific ID is valid

By default, IDs already known to be in use (see EXISTING_IDS below) are never
returned, so generated IDs won't collide with your existing remotes.
"""
import argparse
import random
import sys
from typing import Optional

# IDs already in use - generated IDs are kept clear of these to avoid collisions.
# Add any others you pair so this script never hands you a duplicate.
EXISTING_IDS = {
    0x9BAC,  # factory remote 1
    0x7CA2,  # factory remote 2
    0x1ABA,  # factory remote 3
    0xE0AC,  # factory remote 4 (from Broadlink capture)
    0xE0AE,  # virtual 1
    0xE0A2,  # virtual 2
    0xE0A4,  # virtual 3
    0xC4B6,  # virtual - confirmed working (new high-byte, validated the rule)
}

# The 14 valid low bytes, derived directly from the rule.
VALID_LOW_BYTES = [
    b for b in range(256)
    if (b >> 7) & 1 == 1        # bit 7 set
    and (b >> 6) & 1 == 0       # bit 6 clear
    and (b >> 5) & 1 == 1       # bit 5 set
    and b & 1 == 0             # even
    and (b >> 1) & 0b111 != 0   # at least one of bits 1..3 set
]


def is_valid(remote_id: int) -> bool:
    """True if remote_id satisfies the pairing rule."""
    if not (0 <= remote_id <= 0xFFFF):
        return False
    return (remote_id & 0xFF) in VALID_LOW_BYTES


def all_valid_ids(prefix: Optional[int] = None):
    """Yield every valid ID, optionally restricted to a given high byte."""
    highs = [prefix] if prefix is not None else range(256)
    for hi in highs:
        for lo in VALID_LOW_BYTES:
            yield (hi << 8) | lo


def available_ids(prefix: Optional[int] = None):
    """Valid IDs excluding ones already in use."""
    return [v for v in all_valid_ids(prefix) if v not in EXISTING_IDS]


def main():
    p = argparse.ArgumentParser(
        description="Generate valid Power Smart remote IDs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("-n", "--count", type=int, default=10,
                   help="how many IDs to generate (default 10)")
    p.add_argument("--all", action="store_true",
                   help="print every valid ID instead of a random sample")
    p.add_argument("--prefix", type=lambda s: int(s, 16),
                   help="restrict to a high byte, e.g. 0xE0")
    p.add_argument("--yaml", action="store_true",
                   help="emit ESPHome substitution lines")
    p.add_argument("--check", type=lambda s: int(s, 16),
                   help="check whether a specific ID (hex) is valid, then exit")
    args = p.parse_args()

    if args.check is not None:
        v = args.check
        ok = is_valid(v)
        in_use = v in EXISTING_IDS
        print(f"0x{v:04X}: {'VALID' if ok else 'INVALID'}"
              f" (low byte 0x{v & 0xFF:02X})"
              + ("  [already in use]" if in_use else ""))
        if ok and not in_use:
            print("  -> safe to use")
        elif ok and in_use:
            print("  -> valid, but already assigned to a remote")
        else:
            print(f"  -> low byte must be one of: "
                  + " ".join(f"0x{b:02X}" for b in VALID_LOW_BYTES))
        sys.exit(0 if ok else 1)

    pool = available_ids(args.prefix)
    if not pool:
        print("No available IDs for that prefix (all in use?).")
        sys.exit(1)

    if args.all:
        ids = pool
    else:
        n = min(args.count, len(pool))
        ids = sorted(random.sample(pool, n))

    if args.yaml:
        for i, v in enumerate(ids, 1):
            print(f'  virtual_{i}_id: "0x{v:04X}"')
    else:
        for v in ids:
            print(f"0x{v:04X}  (low byte 0x{v & 0xFF:02X})")
        print(f"\n{len(ids)} ID(s)"
              + (f" with prefix 0x{args.prefix:02X}" if args.prefix is not None else "")
              + f"; {len(pool)} available in total"
              + (f" (of {14 * (1 if args.prefix is not None else 256)} valid)"))


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        # Happens when output is piped into a command that closes early
        # (e.g. `| head`). Exit quietly rather than dumping a traceback.
        sys.exit(0)
