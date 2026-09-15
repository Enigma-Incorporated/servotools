#!/usr/bin/env python3
"""Does bus-collision arbitration encode distance from the adapter?

When several servos answer at once (e.g. a broadcast ping), their TX drivers
fight on the half-duplex line. If the servo nearest the adapter wins the
contention (lower-impedance path -> its bytes survive), then collision-win
frequency ranks servos by distance from the adapter -- movement-free,
current-free. Scored vs eyes ground truth.

Part A re-confirms the bus is fully shared (reboot the NEAREST servo; everyone
else must stay reachable -> signal passes through it).
Part B forces collisions and tallies who wins.
"""

from __future__ import annotations

import time
from collections import Counter
from itertools import combinations

import serial as pyserial

from _common import find_port

IDS = [1, 2, 3, 4, 5, 6]
GT = {1: 3, 2: 4, 3: 1, 4: 2, 5: 6, 6: 5}   # id -> location (1=near adapter)
NEAREST = min(IDS, key=lambda i: GT[i])      # id at location 1


def _cs(b):
    return (~sum(b)) & 0xFF


def _ping(sid):
    body = bytes([sid, 0x02, 0x01])
    return b"\xff\xff" + body + bytes([_cs(body)])


def _reboot(sid):
    body = bytes([sid, 0x02, 0x08])
    return b"\xff\xff" + body + bytes([_cs(body)])


BCAST_PING = _ping(0xFE)


def clean_ids(resp):
    """All ids whose checksum-valid status packet survived in resp."""
    out = []
    i = 0
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] not in (0xFF,):
            sid = resp[i + 2]
            ln = resp[i + 3]
            end = i + 4 + ln
            if 2 <= ln <= 4 and end <= len(resp):
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1] and sid in IDS:
                    out.append(sid)
                    i = end
                    continue
        i += 1
    return out


def kendall(order):
    pairs = list(combinations(order, 2))
    fwd = sum(1 for a, b in pairs if GT[a] < GT[b])
    return max(fwd, len(pairs) - fwd) / len(pairs)


def main() -> None:
    port = find_port()
    ser = pyserial.Serial(port, 1_000_000, timeout=0.006)
    time.sleep(0.05)
    print(f"Port: {port}")
    print("GT (loc):", " ".join(f"ID{i}@{GT[i]}" for i in IDS), "\n")

    # ---- Part A: reboot the NEAREST servo, check everyone else stays up ----
    print(f"== Part A: reboot nearest servo ID{NEAREST} (loc 1); who stays reachable? ==")
    ser.reset_input_buffer()
    ser.write(_reboot(NEAREST))
    t0 = time.perf_counter()
    seen = {s: 0 for s in IDS}
    trials = 0
    while time.perf_counter() - t0 < 0.7:        # during its offline window
        trials += 1
        for s in IDS:
            ser.reset_input_buffer()
            ser.write(_ping(s))
            time.sleep(0.0015)
            if clean_ids(ser.read(8)) == [s]:
                seen[s] += 1
    for s in IDS:
        print(f"   ID{s} (loc{GT[s]}): reachable {100*seen[s]/trials:.0f}% "
              f"while ID{NEAREST} rebooting")
    print("   -> if all non-target ~100%, the signal passes through it: fully shared bus.\n")
    time.sleep(0.5)

    # ---- Part B: force collisions, tally winners ----
    print("== Part B: broadcast-ping collisions; tally clean winners ==")
    wins = Counter()
    clean_per_trial = []
    N = 400
    for _ in range(N):
        ser.reset_input_buffer()
        ser.write(BCAST_PING)
        time.sleep(0.003)
        ids = clean_ids(ser.read(64))
        clean_per_trial.append(len(ids))
        wins.update(ids)

    avg_clean = sum(clean_per_trial) / len(clean_per_trial)
    print(f"   avg clean packets per broadcast: {avg_clean:.2f} "
          f"({'STAGGERED (no real collision)' if avg_clean > 4 else 'COLLIDING'})")
    print(f"   {'ID':>3} {'loc':>4} {'wins':>6} {'win%':>6}")
    for s in IDS:
        print(f"   {s:>3} {GT[s]:>4} {wins[s]:>6} {100*wins[s]/N:>5.0f}%")
    order = sorted(IDS, key=lambda s: -wins[s])   # most wins first
    print(f"   win-frequency order: {order}  score={kendall(order):.2f}")
    print(f"   ground-truth order : {sorted(IDS, key=lambda i: GT[i])}")

    ser.close()


if __name__ == "__main__":
    main()
