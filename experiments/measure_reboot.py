#!/usr/bin/env python3
"""Measure when each servo wakes after a broadcast reboot (instruction 0x08).

Polls both servos round-robin after the reboot and records, per trial, the first
instant each one answers. The polling order is alternated between trials so the
round-robin's own bias can be measured and removed rather than assumed away.

    python experiments/measure_reboot.py --trials 60 --out wake.json
    python experiments/measure_reboot.py --swap        # swap the two IDs first

Read-only apart from --swap. Reboots make servos briefly limp.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import serial as pyserial  # noqa: E402

from servotools import registers as reg  # noqa: E402
from servotools.bus import INST_READ, Bus, find_frame, packet  # noqa: E402
from servotools.config import BAUD  # noqa: E402
from servotools.ports import NOT_FOUND, find_port  # noqa: E402
from servotools.renumber import set_id  # noqa: E402
from servotools.split import SKIP_S  # noqa: E402

POLL_TIMEOUT = 0.0004  # a reply is 8 bytes = 80us at 1 Mbps; this bounds the wait
RETURN_DELAY = reg.RETURN_DELAY
DEADLINE_S = 1.4
SETTLE_S = 0.25


def trial(ser, ids, order) -> dict:
    """One reboot; return the ms after reboot at which each ID first answered."""
    polls = {sid: packet(sid, INST_READ, bytes([reg.PRESENT_POSITION, 2])) for sid in ids}
    woke: dict[int, float] = {}
    n_polls = 0

    ser.reset_input_buffer()
    ser.write(packet(0xFE, 0x08))
    t0 = time.perf_counter()
    time.sleep(SKIP_S)

    while len(woke) < len(ids) and time.perf_counter() - t0 < DEADLINE_S:
        for sid in order:
            if sid in woke:
                continue
            ser.reset_input_buffer()
            ser.write(polls[sid])
            resp = ser.read(16)
            n_polls += 1
            if resp and find_frame(resp, sid, 2) is not None:
                woke[sid] = (time.perf_counter() - t0) * 1000.0

    elapsed = (time.perf_counter() - t0) * 1000.0
    time.sleep(SETTLE_S)
    # Temperature, to test whether the slow upward drift in wake time is thermal.
    old, ser.timeout = ser.timeout, 0.006
    temps = {}
    for sid in ids:
        ser.reset_input_buffer()
        ser.write(packet(sid, INST_READ, bytes([reg.TEMPERATURE, 1])))
        time.sleep(0.001)
        f = find_frame(ser.read(16), sid, 1)
        if f is not None:
            temps[str(sid)] = f[0]
    ser.timeout = old
    return {"order": list(order), "woke": {str(k): v for k, v in woke.items()},
            "polls": n_polls, "elapsed_ms": elapsed, "temps": temps}


def summarise(rows, ids) -> None:
    a, b = ids
    print(f"\n{'=' * 78}\nWake time after reboot (ms), {len(rows)} reboots\n{'=' * 78}")
    for sid in ids:
        vs = [r["woke"][str(sid)] for r in rows if str(sid) in r["woke"]]
        if not vs:
            print(f"  ID {sid}: never answered")
            continue
        print(f"  ID {sid}:  n={len(vs):<4} mean={statistics.mean(vs):8.3f}  "
              f"median={statistics.median(vs):8.3f}  sd={statistics.pstdev(vs):6.3f}  "
              f"min={min(vs):8.3f}  max={max(vs):8.3f}")

    paired = [r for r in rows if str(a) in r["woke"] and str(b) in r["woke"]]
    print(f"\n{'-' * 78}\nPaired difference  ID{b} - ID{a}  (ms), by polling order\n{'-' * 78}")
    by_order = {}
    for r in paired:
        d = r["woke"][str(b)] - r["woke"][str(a)]
        by_order.setdefault(tuple(r["order"]), []).append(d)
    for order, ds in sorted(by_order.items()):
        print(f"  polled {order[0]} first:  n={len(ds):<4} mean={statistics.mean(ds):+8.3f}  "
              f"sd={statistics.pstdev(ds):6.3f}")

    if len(by_order) == 2:
        (o1, d1), (o2, d2) = sorted(by_order.items())
        m1, m2 = statistics.mean(d1), statistics.mean(d2)
        print(f"\n  round-robin bias  = (mean1 - mean2)/2 = {(m1 - m2) / 2:+.3f} ms")
        print(f"  bias-free offset  = (mean1 + mean2)/2 = {(m1 + m2) / 2:+.3f} ms")

    # Only the time after SKIP_S is spent polling; the rest is the silent boot wait.
    per_poll = [(r["elapsed_ms"] - SKIP_S * 1000) / r["polls"] for r in rows if r["polls"]]
    if per_poll:
        p_us = statistics.median(per_poll) * 1000
        print(f"\n  poll period ~{p_us:.0f} us  -> each servo revisited every ~{p_us * 2:.0f} us")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", type=int, nargs=2, default=[1, 2])
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--out", default=None)
    ap.add_argument("--swap", action="store_true", help="swap the two IDs before measuring")
    ap.add_argument("--poll-timeout", type=float, default=POLL_TIMEOUT,
                    help="serial read timeout per poll; sets the sampling resolution")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        print(NOT_FOUND)
        return 1

    ser = pyserial.Serial(port, BAUD, timeout=args.poll_timeout)
    bus = Bus(ser)
    time.sleep(0.05)
    a, b = args.ids

    if args.swap:
        print(f"Swapping ID {a} <-> ID {b} (same physical servos, exchanged addresses)")
        set_id(bus, a, 99)
        set_id(bus, b, a)
        set_id(bus, 99, b)
        print(f"  now: {bus.scan(range(1, 20))}")

    # Position identifies which physical unit holds which ID.
    ser.timeout = 0.006
    units = {sid: bus.read(sid, reg.PRESENT_POSITION, 2) for sid in (a, b)}
    ser.timeout = args.poll_timeout
    print(f"Port: {port}   ID->position fingerprint: {units}")
    print(f"Running {args.trials} reboots, alternating polling order...")

    rows = []
    for i in range(args.trials):
        order = (a, b) if i % 2 == 0 else (b, a)
        r = trial(ser, (a, b), order)
        r["trial"] = i
        rows.append(r)
        got = " ".join(f"ID{k}={v:.2f}" for k, v in sorted(r["woke"].items()))
        print(f"  [{i + 1:>3}/{args.trials}] first={order[0]}  {got}", flush=True)

    ser.close()
    summarise(rows, (a, b))

    if args.out:
        Path(args.out).write_text(json.dumps(
            {"ids": [a, b], "units": {str(k): v for k, v in units.items()},
             "swapped": args.swap, "rows": rows}, indent=1), encoding="utf-8")
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
