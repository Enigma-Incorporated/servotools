#!/usr/bin/env python3
"""Probe MOVEMENT-FREE signals for chain location (reboot-race methodology:
modulate a perturbation, measure a positional response). Scored vs eyes
ground truth. No servo motion is commanded.

Signals probed:
  1. Comm latency + read-error rate per servo (signal integrity vs distance).
  2. TX-driver sag: spam reads at X (its line driver draws current through its
     upstream links) and measure each other servo's voltage drop -> who is
     upstream of X. Pure comm current, no movement.
"""

from __future__ import annotations

import statistics
import time
from itertools import combinations

import scservo_sdk as scs

from _common import find_port

IDS = [1, 2, 3, 4, 5, 6]
GT = {1: 3, 2: 4, 3: 1, 4: 2, 5: 6, 6: 5}   # id -> location (1=near adapter)

ADDR_POS = 56
ADDR_VOLT = 62


def kendall(order):
    pairs = list(combinations(order, 2))
    fwd = sum(1 for a, b in pairs if GT[a] < GT[b])
    return max(fwd, len(pairs) - fwd) / len(pairs)


def main() -> None:
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}")
    print("GT (loc):", " ".join(f"ID{i}@{GT[i]}" for i in IDS), "\n")

    # ---- 1. comm latency + error rate ----
    print("== comm latency / error rate (movement-free, zero current) ==")
    lat = {s: [] for s in IDS}
    err = {s: 0 for s in IDS}
    N = 400
    for _ in range(N):
        for s in IDS:
            t0 = time.perf_counter()
            _, res, _ = pk.read4ByteTxRx(ph, s, ADDR_POS)  # 4 bytes = more bits to corrupt
            dt = (time.perf_counter() - t0) * 1e6  # us
            if res == 0:
                lat[s].append(dt)
            else:
                err[s] += 1
    print(f"{'ID':>3} {'loc':>4} {'lat_us':>8} {'jit_us':>8} {'err%':>6}")
    for s in IDS:
        m = statistics.mean(lat[s]) if lat[s] else 0
        j = statistics.pstdev(lat[s]) if len(lat[s]) > 1 else 0
        print(f"{s:>3} {GT[s]:>4} {m:>8.1f} {j:>8.1f} {100*err[s]/N:>5.1f}%")
    lat_order = sorted(IDS, key=lambda s: statistics.mean(lat[s]) if lat[s] else 9e9)
    err_order = sorted(IDS, key=lambda s: err[s])
    print(f"  latency order: {lat_order}  score={kendall(lat_order):.2f}")
    print(f"  error   order: {err_order}  score={kendall(err_order):.2f}\n")

    # ---- 2. TX-driver sag matrix ----
    print("== TX-driver sag: spam reads at X, measure others' voltage ==")
    rv = lambda s: (pk.read1ByteTxRx(ph, s, ADDR_VOLT)[0])

    base = {s: [] for s in IDS}
    for _ in range(400):
        for s in IDS:
            v = rv(s)
            if v is not None:
                base[s].append(v)
    base_mean = {s: statistics.mean(base[s]) for s in IDS}

    drop = {x: {} for x in IDS}
    for x in IDS:
        loaded = {s: [] for s in IDS}
        t_end = time.perf_counter() + 1.5
        while time.perf_counter() < t_end:
            for _ in range(6):                  # hammer X -> its TX driver active
                pk.read4ByteTxRx(ph, x, ADDR_POS)
            for y in IDS:                        # sample everyone
                v = rv(y)
                if v is not None:
                    loaded[y].append(v)
        for y in IDS:
            drop[x][y] = (base_mean[y] - statistics.mean(loaded[y])) / 10.0 * 1000  # mV

    print("Sag matrix (mV): row = spammed X, col = measured Y")
    print("       " + "  ".join(f"ID{y:>2}" for y in IDS))
    for x in IDS:
        print(f"  ID{x:>2} " + "  ".join(f"{drop[x][y]:>4.0f}" for y in IDS))

    coupling = {y: sum(drop[x][y] for x in IDS if x != y) for y in IDS}
    orderA = sorted(IDS, key=lambda y: coupling[y])
    print(f"  coupling order: {orderA}  score={kendall(orderA):.2f}")
    print(f"\nGround truth order (near->far): {sorted(IDS, key=lambda i: GT[i])}")


if __name__ == "__main__":
    main()
