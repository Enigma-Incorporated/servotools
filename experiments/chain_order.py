#!/usr/bin/env python3
"""Determine daisy-chain ORDER electrically -- no eyes, no wiggle.

Method: load each servo individually (hard oscillation = current draw) and
measure the supply-voltage sag at EVERY servo. Current to the loaded servo
flows adapter -> ... -> loaded servo, so:
  - servos UPSTREAM of the loaded one (toward adapter) sag a little,
  - the loaded servo + everything DOWNSTREAM sag the most (same node).
Each servo's sag in test X is proportional to the resistance from the
adapter to whichever of {that servo, X} is nearer. Summing each servo's
sag across all load tests therefore ranks them near -> far, and the sum
accumulates the per-link differences so they clear the 0.1V ADC floor.

Assumes power is injected at the adapter end -> least total sag = nearest.
"""

from __future__ import annotations

import statistics
import sys
import time

import scservo_sdk as scs

from _common import find_port

ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_POS = 56
ADDR_VOLT = 62
ADDR_TEMP = 63

SPAN = 400
FLIP_S = 0.18
LOAD_S = 1.6


def read_volt(pk, ph, sid):
    v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    return v if res == 0 else None


def low_pct(vals, pct=10):
    """Low percentile -> the loaded (sagged) voltage, ignoring coast spikes."""
    if not vals:
        return None
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(len(s) * pct / 100)))
    return s[k]


def temps(pk, ph, ids):
    out = {}
    for sid in ids:
        t, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_TEMP)
        out[sid] = t if res == 0 else None
    return out


def main() -> None:
    ids = [int(x) for x in sys.argv[1:]] or [1, 2, 3]
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}  IDs {ids}  (power assumed injected at adapter end)\n")

    t0 = temps(pk, ph, ids)
    print("Start temps: " + "  ".join(f"ID{s}={t0[s]}C" for s in ids))
    if any(t and t >= 55 for t in t0.values()):
        print("Too hot to load safely. Let them cool.")
        ph.closePort()
        sys.exit(1)

    # home positions
    home = {}
    for sid in ids:
        p, res, _ = pk.read2ByteTxRx(ph, sid, ADDR_POS)
        home[sid] = p if res == 0 else 2048

    # idle baseline voltages (at rest)
    idle = {}
    for sid in ids:
        vs = [read_volt(pk, ph, sid) for _ in range(10)]
        idle[sid] = statistics.median([v for v in vs if v is not None])

    # sag[X][Y] = how much servo Y sagged while X was the load
    sag = {x: {} for x in ids}
    for x in ids:
        loaded = {y: [] for y in ids}
        pk.write1ByteTxRx(ph, x, ADDR_TORQUE, 1)
        time.sleep(0.03)
        lo, hi = max(50, home[x] - SPAN), min(4045, home[x] + SPAN)
        phase = 0
        last_flip = 0.0
        t_end = time.perf_counter() + LOAD_S
        while time.perf_counter() < t_end:
            now = time.perf_counter()
            if now - last_flip > FLIP_S:
                phase ^= 1
                pk.write2ByteTxRx(ph, x, ADDR_GOAL, hi if phase else lo)
                last_flip = now
            for y in ids:
                v = read_volt(pk, ph, y)
                if v is not None:
                    loaded[y].append(v)
        # stop this servo
        pk.write2ByteTxRx(ph, x, ADDR_GOAL, home[x])
        time.sleep(0.3)
        pk.write1ByteTxRx(ph, x, ADDR_TORQUE, 0)
        for y in ids:
            sag[x][y] = idle[y] - low_pct(loaded[y])
        time.sleep(0.3)

    # matrix
    print("\nVoltage sag matrix (0.1V units): rows = loaded servo, cols = measured")
    header = "load\\meas " + "  ".join(f"ID{y:>2}" for y in ids)
    print("  " + header)
    for x in ids:
        row = "  ".join(f"{sag[x][y]:>4}" for y in ids)
        print(f"  ID{x:>2}      {row}")

    total = {y: sum(sag[x][y] for x in ids) for y in ids}
    print("\nTotal sag per servo (across all load tests):")
    for y in ids:
        print(f"  ID {y}: {total[y]} (0.1V units)")

    order = sorted(ids, key=lambda y: total[y])  # least sag = nearest
    print("\nInferred chain order  near -> far:")
    print("  " + " -> ".join(f"ID{y}" for y in order))

    spread = max(total.values()) - min(total.values())
    t1 = temps(pk, ph, ids)
    print("\nEnd temps:   " + "  ".join(f"ID{s}={t1[s]}C" for s in ids))
    if spread < 2:
        print("\n  ** Total-sag spread too small to trust. Inconclusive. **")
    else:
        print(f"\n  Spread = {spread} (0.1V units) -> order is distinguishable.")
        # convention: nearest = ID 1
        print("\n  Desired remap (nearest adapter = ID 1):")
        for new, y in enumerate(order, start=1):
            print(f"    physical pos {new} (ID {y} now) -> ID {new}")

    ph.closePort()


if __name__ == "__main__":
    main()
