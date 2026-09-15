#!/usr/bin/env python3
"""Try to read the daisy-chain ORDER from the supply-voltage gradient.

All current to downstream servos flows through the nearest link, so under
load each servo sees: V_servo = Vsup - (cumulative IR drop to it). That
makes voltages stair-step  V_near > V_mid > V_far  -- IF we can draw enough
current to clear the 0.1V ADC resolution. We force current by oscillating
all servos at once, sampling each one's voltage many times under load.

Assumes power is injected at the adapter end (Waveshare board) -> highest
voltage = nearest the adapter.
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
ADDR_LOAD = 58


def median_volt(pk, ph, ids, samples_each=1):
    out = {}
    for sid in ids:
        vs = []
        for _ in range(samples_each):
            v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
            if res == 0:
                vs.append(v)
        if vs:
            out[sid] = vs
    return out


def main() -> None:
    ids = [int(x) for x in sys.argv[1:]] or [1, 2, 3]
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}  IDs {ids}\n")

    # Baseline idle voltage
    p0 = {}
    for sid in ids:
        pos, res, _ = pk.read2ByteTxRx(ph, sid, ADDR_POS)
        p0[sid] = pos if res == 0 else 2048
    base = median_volt(pk, ph, ids, samples_each=8)
    print("Idle voltage (0.1V units):")
    for sid in ids:
        print(f"  ID {sid}: {statistics.median(base[sid])/10:.1f}V")

    # Torque on, oscillate all simultaneously, sample voltage under load
    for sid in ids:
        pk.write1ByteTxRx(ph, sid, ADDR_TORQUE, 1)
    time.sleep(0.05)

    span = 350
    targets = {sid: (max(50, p0[sid] - span), min(4045, p0[sid] + span)) for sid in ids}
    loaded = {sid: [] for sid in ids}

    t_end = time.perf_counter() + 5.0
    phase = 0
    last_flip = 0.0
    while time.perf_counter() < t_end:
        now = time.perf_counter()
        if now - last_flip > 0.22:
            phase ^= 1
            for sid in ids:
                pk.write2ByteTxRx(ph, sid, ADDR_GOAL, targets[sid][phase])
            last_flip = now
        # sample everyone's voltage while they drive
        for sid in ids:
            v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
            if res == 0:
                loaded[sid].append(v)

    # Stop: return home, torque off
    for sid in ids:
        pk.write2ByteTxRx(ph, sid, ADDR_GOAL, p0[sid])
    time.sleep(0.4)
    for sid in ids:
        pk.write1ByteTxRx(ph, sid, ADDR_TORQUE, 0)
    ph.closePort()

    print("\nUnder load (oscillating all together):")
    rows = []
    for sid in ids:
        vs = loaded[sid]
        med = statistics.median(vs) / 10
        lo = min(vs) / 10
        n = len(vs)
        rows.append((sid, med, lo, n))
        print(f"  ID {sid}: median {med:.1f}V  min {lo:.1f}V  ({n} samples)")

    ranked = sorted(rows, key=lambda r: (-r[1], -r[2]))
    print("\nRanked by loaded voltage (high=near adapter -> low=far):")
    order = " -> ".join(f"ID{sid}" for sid, *_ in ranked)
    print(f"  {order}")

    spread = max(r[1] for r in rows) - min(r[1] for r in rows)
    if spread < 0.05:
        print("\n  ** Gradient under the ADC resolution (flat). Cannot rank "
              "electrically -- use the wiggle test instead. **")
    else:
        print(f"\n  Gradient spread = {spread:.1f}V -> order is readable.")


if __name__ == "__main__":
    main()
