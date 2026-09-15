#!/usr/bin/env python3
"""Break the ID2/ID3 tie via dithering superresolution.

ID1 is the farthest servo, so current drawn by ID1 flows through the links
of both ID2 and ID3. Whichever of ID2/ID3 is NEARER the adapter sags
slightly less. The difference is below one ADC code (0.1V), but under heavy
oscillating load the readings dither across codes -- so the MEAN over
thousands of interleaved samples can resolve which is consistently higher.

Higher mean voltage = nearer the adapter.
"""

from __future__ import annotations

import statistics
import time

import scservo_sdk as scs

from _common import find_port

ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_POS = 56
ADDR_VOLT = 62

LOADER = 1          # farthest servo; drives current through the 2<->3 link
PAIR = (2, 3)
SPAN = 450
FLIP_S = 0.16


def rv(pk, ph, sid):
    v, res, _ = pk.read1ByteTxRx(ph, sid, ADDR_VOLT)
    return v if res == 0 else None


def burst(pk, ph, home, dur):
    """Oscillate LOADER for `dur` s; interleave-sample the PAIR. Returns
    {sid: [samples]}."""
    samples = {PAIR[0]: [], PAIR[1]: []}
    lo, hi = max(50, home - SPAN), min(4045, home + SPAN)
    phase = 0
    last = 0.0
    t_end = time.perf_counter() + dur
    while time.perf_counter() < t_end:
        now = time.perf_counter()
        if now - last > FLIP_S:
            phase ^= 1
            pk.write2ByteTxRx(ph, LOADER, ADDR_GOAL, hi if phase else lo)
            last = now
        # interleave so both see the same average load conditions
        for sid in PAIR:
            v = rv(pk, ph, sid)
            if v is not None:
                samples[sid].append(v)
    return samples


def main() -> None:
    port = find_port()
    ph = scs.PortHandler(port)
    pk = scs.PacketHandler(0)
    ph.openPort()
    ph.setBaudRate(1_000_000)
    print(f"Port: {port}  loader=ID{LOADER}  comparing ID{PAIR[0]} vs ID{PAIR[1]}\n")

    home, _, _ = pk.read2ByteTxRx(ph, LOADER, ADDR_POS)

    # control: no load
    base = {s: [] for s in PAIR}
    for _ in range(400):
        for s in PAIR:
            v = rv(pk, ph, s)
            if v is not None:
                base[s].append(v)
    print("no-load means:  " +
          "  ".join(f"ID{s}={statistics.mean(base[s])/10:.3f}V" for s in PAIR))

    # loaded bursts, repeated for consistency
    pk.write1ByteTxRx(ph, LOADER, ADDR_TORQUE, 1)
    time.sleep(0.03)
    wins = {PAIR[0]: 0, PAIR[1]: 0}
    for r in range(5):
        s = burst(pk, ph, home, dur=2.0)
        m0 = statistics.mean(s[PAIR[0]])
        m1 = statistics.mean(s[PAIR[1]])
        nearer = PAIR[0] if m0 > m1 else PAIR[1]
        wins[nearer] += 1
        print(f"  burst {r}: ID{PAIR[0]} mean={m0/10:.3f}V ({len(s[PAIR[0]])} smp)  "
              f"ID{PAIR[1]} mean={m1/10:.3f}V ({len(s[PAIR[1]])} smp)  "
              f"-> nearer: ID{nearer}")
    pk.write2ByteTxRx(ph, LOADER, ADDR_GOAL, home)
    time.sleep(0.3)
    pk.write1ByteTxRx(ph, LOADER, ADDR_TORQUE, 0)
    ph.closePort()

    print(f"\nNearer-adapter votes: ID{PAIR[0]}={wins[PAIR[0]]}/5, "
          f"ID{PAIR[1]}={wins[PAIR[1]]}/5")
    if max(wins.values()) >= 4:
        near = max(wins, key=wins.get)
        far = PAIR[0] if near == PAIR[1] else PAIR[1]
        print(f"=> ID{near} is nearer the adapter than ID{far}.")
        print(f"=> Full chain near->far: ID{near} -> ID{far} -> ID{LOADER}")
    else:
        print("=> Split decision -- still under the noise floor. Inconclusive.")


if __name__ == "__main__":
    main()
