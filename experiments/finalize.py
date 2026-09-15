#!/usr/bin/env python3
"""Verify the two servos are now independently addressable, then normalize
their IDs to the canonical 1 and 2.

Run after split_ids.py has separated the two same-ID servos. Because they
now sit at distinct IDs, ordinary addressed reads/writes work -- no more
collisions -- so this is plain, reliable servo configuration.
"""

from __future__ import annotations

import sys
import time

import scservo_sdk as scs

from _common import JOINT_NAMES, find_port

ADDR_ID = 5
ADDR_TORQUE = 40
ADDR_LOCK = 55
ADDR_POS = 56
ADDR_VOLT = 62
ADDR_TEMP = 63


def clean_read_count(pk, ph, sid, n=12):
    """How many of n position reads come back clean. One servo -> ~all clean;
    two servos at this id -> collisions -> few/none clean."""
    ok = 0
    for _ in range(n):
        _, res, err = pk.read2ByteTxRx(ph, sid, ADDR_POS)
        if res == 0 and err == 0:
            ok += 1
        time.sleep(0.004)
    return ok


def identify(pk, ph, sid):
    pos, _, _ = pk.read2ByteTxRx(ph, sid, ADDR_POS)
    volt, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    temp, _, _ = pk.read1ByteTxRx(ph, sid, ADDR_TEMP)
    return pos, volt, temp


def set_id(pk, ph, old, new):
    pk.write1ByteTxRx(ph, old, ADDR_TORQUE, 0)
    time.sleep(0.05)
    pk.write1ByteTxRx(ph, old, ADDR_LOCK, 0)   # unlock EEPROM
    time.sleep(0.05)
    pk.write1ByteTxRx(ph, old, ADDR_ID, new)
    time.sleep(0.05)
    pk.write1ByteTxRx(ph, new, ADDR_LOCK, 1)   # re-lock at new id
    time.sleep(0.05)
    _, res, _ = pk.ping(ph, new)
    return res == 0


def scan(pk, ph, ids=range(1, 7)):
    present = []
    for sid in ids:
        n = clean_read_count(pk, ph, sid, n=10)
        if n == 0:
            continue
        # distinguish single (mostly clean) from double (mostly collide)
        kind = "SINGLE" if n >= 6 else "DOUBLE?"
        present.append((sid, kind, n))
    return present


def main() -> None:
    port = find_port()
    if port is None:
        print("ERROR: no adapter")
        sys.exit(1)
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    if not ph.openPort():
        print("ERROR: cannot open port")
        sys.exit(1)
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}\n")

    print("== Verify split (clean position reads per ID) ==")
    present = scan(pk, ph)
    for sid, kind, n in present:
        pos, volt, temp = identify(pk, ph, sid)
        print(f"  ID {sid}: {kind} ({n}/10 clean)  pos={pos} volt={volt/10:.1f}V temp={temp}C")

    singles = [sid for sid, kind, _ in present if kind == "SINGLE"]
    doubles = [sid for sid, kind, _ in present if kind != "SINGLE"]
    if doubles or len(singles) < 2:
        print("\nNot cleanly split yet (still a doubled ID). Re-run split_ids.py.")
        ph.closePort()
        sys.exit(1)

    print(f"\n  -> Two independently addressable servos at IDs {sorted(singles)}. "
          f"Split confirmed.\n")

    # Normalize to 1 and 2 (lowest current id -> 1, next -> 2), ordered so
    # we never create a transient collision.
    targets = [1, 2]
    plan = list(zip(sorted(singles), targets))
    print("== Normalize to canonical IDs ==")
    for old, new in plan:
        if old == new:
            print(f"  ID {old} already correct")
            continue
        ok = set_id(pk, ph, old, new)
        print(f"  ID {old} -> {new}: {'OK' if ok else 'FAILED'}")
        time.sleep(0.1)

    print("\n== Final state ==")
    final = scan(pk, ph)
    ok_final = True
    for sid, kind, n in final:
        pos, volt, temp = identify(pk, ph, sid)
        joint = JOINT_NAMES.get(sid, "?")
        print(f"  ID {sid} ({joint}): {kind} ({n}/10 clean)  "
              f"pos={pos} volt={volt/10:.1f}V temp={temp}C")
        if kind != "SINGLE":
            ok_final = False

    final_ids = sorted(sid for sid, kind, _ in final if kind == "SINGLE")
    ph.closePort()

    print()
    if ok_final and final_ids == [1, 2]:
        print("RESULT: SUCCESS -- two servos at IDs 1 and 2, each cleanly addressable.")
        sys.exit(0)
    print(f"RESULT: servos at {final_ids} (each addressable).")
    sys.exit(0 if len(final_ids) >= 2 else 1)


if __name__ == "__main__":
    main()
