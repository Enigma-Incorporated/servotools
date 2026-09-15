#!/usr/bin/env python3
"""Confirm the full daisy-chain order via dithered-mean superresolution.

For each loader X, oscillate X (draw current) and compare the two PASSIVE
servos' mean voltage under identical conditions: the one reading higher is
nearer the adapter. Collect all three pairwise verdicts and check they form
a consistent total order (transitive). Nearest = wins the most comparisons.
"""

from __future__ import annotations

import statistics
import time
from itertools import combinations

import scservo_sdk as scs

from _common import find_port

ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_POS = 56
ADDR_VOLT = 62
ADDR_TEMP = 63

SPAN = 450
FLIP_S = 0.16
BURSTS = 3
BURST_S = 1.5


def rv(pk, ph, sid):
    v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    return v if res == 0 else None


def compare_under_load(pk, ph, loader, a, b, home):
    """Return how many bursts a>b vs b>a (mean voltage; higher=nearer)."""
    lo, hi = max(50, home - SPAN), min(4045, home + SPAN)
    votes = {a: 0, b: 0}
    margins = []
    for _ in range(BURSTS):
        sa, sb = [], []
        phase = 0
        last = 0.0
        t_end = time.perf_counter() + BURST_S
        while time.perf_counter() < t_end:
            now = time.perf_counter()
            if now - last > FLIP_S:
                phase ^= 1
                pk.write2ByteTxRx(ph, loader, ADDR_GOAL, hi if phase else lo)
                last = now
            va = rv(pk, ph, a)
            vb = rv(pk, ph, b)
            if va is not None:
                sa.append(va)
            if vb is not None:
                sb.append(vb)
        ma, mb = statistics.mean(sa), statistics.mean(sb)
        votes[a if ma > mb else b] += 1
        margins.append((ma - mb) / 10)
    return votes, margins


def main() -> None:
    ids = [1, 2, 3]
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}  IDs {ids}\n")

    home = {}
    for s in ids:
        p, res, _ = pk.read2ByteTxRx(ph, s, ADDR_POS)
        home[s] = p if res == 0 else 2048

    # pairwise "nearer" wins
    nearer_wins = {s: 0 for s in ids}
    for loader in ids:
        a, b = [s for s in ids if s != loader]
        pk.write1ByteTxRx(ph, loader, ADDR_TORQUE, 1)
        time.sleep(0.03)
        votes, margins = compare_under_load(pk, ph, loader, a, b, home[loader])
        pk.write2ByteTxRx(ph, loader, ADDR_GOAL, home[loader])
        time.sleep(0.2)
        pk.write1ByteTxRx(ph, loader, ADDR_TORQUE, 0)
        winner = max(votes, key=votes.get)
        nearer_wins[winner] += 1
        mstr = ", ".join(f"{m:+.3f}" for m in margins)
        print(f"  load ID{loader}: ID{a} vs ID{b} -> nearer ID{winner} "
              f"(votes {votes[a]}:{votes[b]}; margins ID{a}-ID{b}=[{mstr}]V)")
        time.sleep(0.2)

    ph.closePort()

    order = sorted(ids, key=lambda s: -nearer_wins[s])  # most wins = nearest
    print(f"\nPairwise 'nearer' wins: " +
          ", ".join(f"ID{s}={nearer_wins[s]}" for s in ids))
    # consistent total order if wins are 2,1,0
    if sorted(nearer_wins.values()) == [0, 1, 2]:
        print(f"Transitively consistent. Chain near->far: " +
              " -> ".join(f"ID{s}" for s in order))
        print("\nRemap (nearest adapter = ID 1):")
        for pos, s in enumerate(order, start=1):
            print(f"  ID{s} (chain pos {pos}) -> ID{pos}")
    else:
        print("Not transitively consistent -> inconclusive at this resolution.")


if __name__ == "__main__":
    main()
