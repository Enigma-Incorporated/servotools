#!/usr/bin/env python3
"""Research: which software signal best recovers daisy-chain LOCATION?

Validated against eyes-confirmed ground truth (6 servos, current IDs 1..6):
    ID3, ID4, ID1, ID2, ID6, ID5   (location 1=nearest adapter .. 6=farthest)

Method under test: PER-SERVO CURRENT INJECTION. Load one servo at a time and
read the voltage sag at the PASSIVE others. A passive servo draws ~idle
current, so its sag is pure position-dependent IR drop (no self-current
confound that wrecks the all-load method). drop_Y(X) ~ I_X * R(adapter ->
nearer(X,Y)). Within one loader, comparing two passive servos' sags tells
which is nearer the adapter (smaller sag = nearer), and that comparison is
invariant to how much current the loader happens to draw.

Prints the sag matrix and several deductions, each scored vs ground truth.
"""

from __future__ import annotations

import statistics
import time
from itertools import combinations, permutations

import scservo_sdk as scs

from _common import find_port

IDS = [1, 2, 3, 4, 5, 6]
GROUND_TRUTH = {1: 3, 2: 4, 3: 1, 4: 2, 5: 6, 6: 5}   # id -> location

ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_POS = 56
ADDR_VOLT = 62

SPAN = 500
FLIP_S = 0.16
LOAD_S = 6.0          # heavy averaging: ~20x the original dwell
IDLE_PASSES = 600


def rv(pk, ph, sid):
    v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    return v if res == 0 else None


def kendall(order):
    """Fraction of pairs ordered consistently with ground-truth location,
    taking the better of the two chain directions."""
    pairs = list(combinations(order, 2))
    fwd = sum(1 for a, b in pairs if GROUND_TRUTH[a] < GROUND_TRUTH[b])
    return max(fwd, len(pairs) - fwd) / len(pairs)


def main() -> None:
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}")
    print("Ground truth (loc 1=near adapter):",
          " ".join(f"ID{i}@{GROUND_TRUTH[i]}" for i in IDS), "\n")

    home = {s: (pk.read2ByteTxRx(ph, s, ADDR_POS)[0] or 2048) for s in IDS}

    # idle baseline (all torque off)
    idle = {s: [] for s in IDS}
    for _ in range(IDLE_PASSES):
        for s in IDS:
            v = rv(pk, ph, s)
            if v is not None:
                idle[s].append(v)
    idle_mean = {s: statistics.mean(idle[s]) for s in IDS}

    # per-servo injection -> sag matrix (drop[X][Y] = sag at Y while X loaded)
    drop = {x: {} for x in IDS}
    for x in IDS:
        pk.write1ByteTxRx(ph, x, ADDR_TORQUE, 1)
        time.sleep(0.04)
        lo, hi = max(50, home[x] - SPAN), min(4045, home[x] + SPAN)
        loaded = {s: [] for s in IDS}
        phase = 0
        last = 0.0
        t_end = time.perf_counter() + LOAD_S
        while time.perf_counter() < t_end:
            now = time.perf_counter()
            if now - last > FLIP_S:
                phase ^= 1
                pk.write2ByteTxRx(ph, x, ADDR_GOAL, hi if phase else lo)
                last = now
            for s in IDS:
                v = rv(pk, ph, s)
                if v is not None:
                    loaded[s].append(v)
        pk.write2ByteTxRx(ph, x, ADDR_GOAL, home[x])
        time.sleep(0.25)
        pk.write1ByteTxRx(ph, x, ADDR_TORQUE, 0)
        for y in IDS:
            drop[x][y] = (idle_mean[y] - statistics.mean(loaded[y])) / 10.0

    # ---- print matrix (mV) ----
    print("Sag matrix (mV): row = loaded servo, col = measured servo")
    print("       " + "  ".join(f"ID{y:>2}" for y in IDS))
    for x in IDS:
        print(f"  ID{x:>2} " + "  ".join(f"{drop[x][y]*1000:>4.0f}" for y in IDS))
    print()

    # ---- deduction A: total passive coupling (col sum over loaders != Y) ----
    coupling = {y: sum(drop[x][y] for x in IDS if x != y) for y in IDS}
    orderA = sorted(IDS, key=lambda y: coupling[y])
    print("A) total passive coupling (asc = nearest):")
    print("   " + "  ".join(f"ID{y}:{coupling[y]*1000:.0f}mV" for y in orderA))
    print(f"   order: {orderA}  score={kendall(orderA):.2f}\n")

    # ---- deduction B: pairwise voting (within-loader, current-invariant) ----
    nearer_wins = {y: 0 for y in IDS}
    EPS = 0.0005  # with heavy averaging, trust finer (0.5mV) differences
    for y, z in combinations(IDS, 2):
        vy = vz = 0
        for x in IDS:
            if x in (y, z):
                continue
            d = drop[x][y] - drop[x][z]
            if d < -EPS:
                vy += 1        # y sagged less -> y nearer
            elif d > EPS:
                vz += 1
        if vy > vz:
            nearer_wins[y] += 1
        elif vz > vy:
            nearer_wins[z] += 1
    orderB = sorted(IDS, key=lambda y: -nearer_wins[y])
    print("B) pairwise voting (most 'nearer' wins = nearest):")
    print("   " + "  ".join(f"ID{y}:{nearer_wins[y]}" for y in orderB))
    print(f"   order: {orderB}  score={kendall(orderB):.2f}\n")

    # ---- deduction C: load-the-farthest single profile ----
    # farthest loader = the one whose injection sags everyone most (its current
    # crosses the most links). Then rank others by their sag (asc = nearest).
    far = max(IDS, key=lambda x: sum(drop[x][y] for y in IDS if y != x))
    prof = sorted((y for y in IDS if y != far), key=lambda y: drop[far][y])
    orderC = prof + [far]
    print(f"C) load-farthest (={far}) then rank by its column:")
    print(f"   order: {orderC}  score={kendall(orderC):.2f}\n")

    truth_order = sorted(IDS, key=lambda i: GROUND_TRUTH[i])
    print(f"Ground-truth order (near->far): {truth_order}")


if __name__ == "__main__":
    main()
