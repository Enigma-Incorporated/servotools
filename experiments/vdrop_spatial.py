#!/usr/bin/env python3
"""Offset-free spatial check: rank servos by voltage DROP, not absolute V.

The superres voltage method compares servos' absolute readings, which can be
polluted by per-unit ADC calibration offset. Subtracting each servo's OWN
idle baseline cancels that offset, leaving only the load-induced IR drop --
a pure spatial signal. Dithered means (thousands of samples) give sub-ADC
resolution. Most drop = farthest from the power-injection point.
"""

from __future__ import annotations

import statistics
import time

import scservo_sdk as scs

from _common import find_port

IDS = [1, 2, 3]
GROUND_TRUTH_NEAR_TO_FAR = [1, 3, 2]   # by adapter, from wiggle
ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_POS = 56
ADDR_VOLT = 62
SPAN = 450
FLIP_S = 0.16


def rv(pk, ph, sid):
    v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    return v if res == 0 else None


def main() -> None:
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}")
    print("Offset-free: drop_i = mean(idle_i) - mean(loaded_i)  [per-unit ADC offset cancels]\n")

    home = {}
    for s in IDS:
        p, res, _ = pk.read2ByteTxRx(ph, s, ADDR_POS)
        home[s] = p if res == 0 else 2048

    # idle means (torque off, at rest)
    idle = {s: [] for s in IDS}
    for _ in range(700):
        for s in IDS:
            v = rv(pk, ph, s)
            if v is not None:
                idle[s].append(v)
    idle_mean = {s: statistics.mean(idle[s]) for s in IDS}

    # load WHOLE chain, sample everyone (interleaved) under load
    for s in IDS:
        pk.write1ByteTxRx(ph, s, ADDR_TORQUE, 1)
    time.sleep(0.05)
    loaded = {s: [] for s in IDS}
    phase = 0
    last = 0.0
    t_end = time.perf_counter() + 6.0
    while time.perf_counter() < t_end:
        now = time.perf_counter()
        if now - last > FLIP_S:
            phase ^= 1
            for s in IDS:
                lo, hi = max(50, home[s] - SPAN), min(4045, home[s] + SPAN)
                pk.write2ByteTxRx(ph, s, ADDR_GOAL, hi if phase else lo)
            last = now
        for s in IDS:
            v = rv(pk, ph, s)
            if v is not None:
                loaded[s].append(v)
    for s in IDS:
        pk.write2ByteTxRx(ph, s, ADDR_GOAL, home[s])
    time.sleep(0.3)
    for s in IDS:
        pk.write1ByteTxRx(ph, s, ADDR_TORQUE, 0)
    ph.closePort()

    loaded_mean = {s: statistics.mean(loaded[s]) for s in IDS}
    drop = {s: (idle_mean[s] - loaded_mean[s]) / 10 for s in IDS}

    print(f"{'ID':>3} {'idle':>8} {'loaded':>8} {'drop(V)':>9}  n_load")
    for s in IDS:
        print(f"{s:>3} {idle_mean[s]/10:8.3f} {loaded_mean[s]/10:8.3f} "
              f"{drop[s]:9.3f}  {len(loaded[s])}")

    far_to_near = sorted(IDS, key=lambda s: -drop[s])  # most drop = farthest from power
    print(f"\nBy drop, farthest->nearest power: " +
          " -> ".join(f"ID{s}" for s in far_to_near))
    print(f"Adapter ground truth far->near:   " +
          " -> ".join(f"ID{s}" for s in GROUND_TRUTH_NEAR_TO_FAR[::-1]))
    # far_to_near is ordered farthest-from-POWER first. If that equals the
    # adapter's NEAR->far order, then nearest-adapter == farthest-from-power,
    # i.e. power is injected at the END OPPOSITE the adapter.
    if far_to_near == GROUND_TRUTH_NEAR_TO_FAR:
        print("=> SPATIAL signal confirmed. nearest-adapter drops most => "
              "power injected at the FAR-from-adapter end (opposite the USB).")
    elif far_to_near == GROUND_TRUTH_NEAR_TO_FAR[::-1]:
        print("=> SPATIAL signal confirmed. nearest-adapter drops least => "
              "power injected at the adapter end.")
    else:
        print("=> Offset-free order matches neither -> inconclusive / noisy.")


if __name__ == "__main__":
    main()
