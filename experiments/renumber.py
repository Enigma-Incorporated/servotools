#!/usr/bin/env python3
"""Renumber servos to a target mapping, safely (via temp IDs, no collisions).

Mapping below is the chain order found by superres_order.py, applied so the
servo nearest the adapter becomes ID 1:
    current ID 3 (nearest)  -> ID 1
    current ID 2 (middle)   -> ID 2
    current ID 1 (farthest) -> ID 3
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

# current_id -> new_id
MAPPING = {3: 1, 2: 2, 1: 3}
TEMP_BASE = 100  # temp space well clear of real IDs


def set_id(pk, ph, old, new):
    if old == new:
        return True
    pk.write1ByteTxRx(ph, old, ADDR_TORQUE, 0)
    time.sleep(0.04)
    pk.write1ByteTxRx(ph, old, ADDR_LOCK, 0)
    time.sleep(0.04)
    pk.write1ByteTxRx(ph, old, ADDR_ID, new)
    time.sleep(0.06)
    pk.write1ByteTxRx(ph, new, ADDR_LOCK, 1)
    time.sleep(0.04)
    _, res, _ = pk.ping(ph, new)
    return res == 0


def clean(pk, ph, sid, n=8):
    ok = 0
    for _ in range(n):
        _, res, err = pk.read2ByteTxRx(ph, sid, ADDR_POS)
        if res == 0 and err == 0:
            ok += 1
        time.sleep(0.004)
    return ok


def main() -> None:
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}")
    print(f"Mapping (current -> new): {MAPPING}\n")

    # Step 1: everyone to temp space (no final collides with a current id)
    print("Step 1: move to temp IDs")
    for cur, new in MAPPING.items():
        tmp = TEMP_BASE + new
        ok = set_id(pk, ph, cur, tmp)
        print(f"  ID {cur} -> {tmp}: {'OK' if ok else 'FAIL'}")

    # Step 2: temp -> final
    print("Step 2: temp -> final")
    for cur, new in MAPPING.items():
        tmp = TEMP_BASE + new
        ok = set_id(pk, ph, tmp, new)
        print(f"  ID {tmp} -> {new}: {'OK' if ok else 'FAIL'}")

    print("\nFinal verification:")
    all_ok = True
    for sid in (1, 2, 3):
        c = clean(pk, ph, sid)
        joint = JOINT_NAMES.get(sid, "?")
        print(f"  ID {sid} ({joint}): {c}/8 clean reads")
        if c < 6:
            all_ok = False
    ph.closePort()

    print("\nRESULT:", "renumbered, nearest=ID1 -> ID1,ID2,ID3 along the chain."
          if all_ok else "something off -- re-scan.")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
