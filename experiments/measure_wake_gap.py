#!/usr/bin/env python3
"""Measure the gap between two servos waking, using the collision as the instrument.

With both servos on one ID, a position read after a reboot goes through three phases:

    silence   -> nobody is awake yet
    CLEAN     -> exactly one servo is awake (its position identifies which)
    collision -> both are awake, replies overlap, checksum fails

The width of the CLEAN phase is the wake gap, observed directly instead of being
differenced out of two separately-estimated wake times. This is the window the ID
sweep in servotools/split.py aims at.

DESTRUCTIVE: collapses both servos onto one ID. Run `servo autonumber` afterwards.

    python experiments/measure_wake_gap.py --trials 60 --out gap.json
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import serial as pyserial  # noqa: E402

from servotools import registers as reg  # noqa: E402
from servotools.bus import INST_READ, INST_WRITE, Bus, find_frame, packet  # noqa: E402
from servotools.config import BAUD, BROADCAST_ID  # noqa: E402
from servotools.ports import NOT_FOUND, find_port  # noqa: E402
from servotools.split import SKIP_S  # noqa: E402

POLL_TIMEOUT = 0.0004
DEADLINE_S = 1.3
SETTLE_S = 0.30


def flatten_to(ser, target: int) -> None:
    for addr, val, settle in [(reg.TORQUE_ENABLE, 0, 0.05), (reg.LOCK, 0, 0.05),
                              (reg.ID, target, 0.20), (reg.LOCK, 1, 0.05)]:
        ser.reset_input_buffer()
        ser.write(packet(BROADCAST_ID, INST_WRITE, bytes([addr, val])))
        time.sleep(settle)


def trial(ser, sid: int) -> dict:
    """One reboot. Record the silence -> clean -> collision transitions."""
    poll = packet(sid, INST_READ, bytes([reg.PRESENT_POSITION, 2]))
    samples = []
    ser.reset_input_buffer()
    ser.write(packet(BROADCAST_ID, 0x08))
    t0 = time.perf_counter()
    time.sleep(SKIP_S)

    first_clean = first_collision = None
    clean_positions = []
    while time.perf_counter() - t0 < DEADLINE_S:
        ser.reset_input_buffer()
        ser.write(poll)
        resp = ser.read(16)
        t = (time.perf_counter() - t0) * 1000.0
        if not resp:
            kind = "silence"
        elif (params := find_frame(resp, sid, 2)) is not None:
            kind = "clean"
            pos = params[0] | (params[1] << 8)
            clean_positions.append(pos)
            if first_clean is None:
                first_clean = (t, pos)
        else:
            kind = "collision"
            if first_clean is not None and first_collision is None:
                first_collision = t
                break
            if first_collision is None:
                first_collision = t
                break
        samples.append((round(t, 3), kind))

    time.sleep(SETTLE_S)
    gap = (first_collision - first_clean[0]) if (first_clean and first_collision) else None
    return {
        "first_clean_ms": first_clean[0] if first_clean else None,
        "first_pos": first_clean[1] if first_clean else None,
        "first_collision_ms": first_collision,
        "gap_ms": gap,
        "clean_polls": len(clean_positions),
        "positions_seen": sorted(set(clean_positions)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        print(NOT_FOUND)
        return 1

    ser = pyserial.Serial(port, BAUD, timeout=0.006)
    bus = Bus(ser)
    time.sleep(0.05)
    before = {sid: bus.read(sid, reg.PRESENT_POSITION, 2) for sid in bus.scan(range(1, 20))}
    print(f"Port: {port}\n  units by position before flatten: {before}")
    flatten_to(ser, args.id)
    print(f"  flattened onto ID {args.id}: {bus.scan([args.id])}")
    ser.timeout = POLL_TIMEOUT

    rows = []
    for i in range(args.trials):
        r = trial(ser, args.id)
        r["trial"] = i
        rows.append(r)
        g = f"{r['gap_ms']:6.3f}" if r["gap_ms"] is not None else "  none"
        fc = f"{r['first_clean_ms']:8.3f}" if r["first_clean_ms"] else "      --"
        print(f"  [{i + 1:>3}/{args.trials}] first_clean={fc}ms  pos={str(r['first_pos']):>5}  "
              f"gap={g}ms  clean_polls={r['clean_polls']}", flush=True)
    ser.close()

    gaps = [r["gap_ms"] for r in rows if r["gap_ms"] is not None]
    firsts = Counter(r["first_pos"] for r in rows if r["first_pos"] is not None)
    print(f"\n{'=' * 78}\nWake gap between the two servos, {len(rows)} reboots\n{'=' * 78}")
    print(f"  trials with an observable one-servo window: {len(gaps)}/{len(rows)}")
    if gaps:
        print(f"  gap  mean={statistics.mean(gaps):6.3f}ms  median={statistics.median(gaps):6.3f}ms  "
              f"sd={statistics.pstdev(gaps):6.3f}  min={min(gaps):6.3f}  max={max(gaps):6.3f}")
    print("\n  which unit wakes first (by its position fingerprint):")
    for pos, n in firsts.most_common():
        print(f"    position {pos:>5}: {n:>4} / {sum(firsts.values())}  "
              f"{'#' * round(40 * n / sum(firsts.values()))}")

    if args.out:
        Path(args.out).write_text(json.dumps(
            {"id": args.id, "units_before": {str(k): v for k, v in before.items()},
             "rows": rows}, indent=1), encoding="utf-8")
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
