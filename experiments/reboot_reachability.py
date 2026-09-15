#!/usr/bin/env python3
"""Test: does rebooting one servo make the servos BEHIND it unreachable?

If the daisy chain is buffered/relayed through each servo, rebooting servo X
should knock out X and everything DOWNSTREAM of it (farther from the adapter)
while upstream servos keep answering. That would give chain order directly.
If the bus is passive straight-through, only X drops out.

Reboot a target ID, then rapidly ping every servo for ~1s and report who
went silent. Annotated with eyes-confirmed ground-truth locations.

Usage: python research/reboot_reachability.py 1
"""

from __future__ import annotations

import sys
import time

import serial as pyserial

from _common import find_port

IDS = [1, 2, 3, 4, 5, 6]
GT = {1: 3, 2: 4, 3: 1, 4: 2, 5: 6, 6: 5}   # id -> location (1=near adapter)


def _cs(b):
    return (~sum(b)) & 0xFF


def _ping_pkt(sid):
    body = bytes([sid, 0x02, 0x01])
    return b"\xff\xff" + body + bytes([_cs(body)])


def _reboot_pkt(sid):
    body = bytes([sid, 0x02, 0x08])
    return b"\xff\xff" + body + bytes([_cs(body)])


def ping(ser, sid):
    ser.reset_input_buffer()
    ser.write(_ping_pkt(sid))
    time.sleep(0.0015)
    resp = ser.read(8)
    i = 0
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            pkt = resp[i:i + 6]
            if len(pkt) >= 6 and ((~sum(pkt[2:-1])) & 0xFF) == pkt[5]:
                return True
        i += 1
    return False


def main() -> None:
    target = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    port = find_port()
    ser = pyserial.Serial(port, 1_000_000, timeout=0.0025)
    time.sleep(0.05)
    print(f"Port: {port}")
    print(f"Rebooting ID{target} (location {GT[target]}). "
          f"Watching who goes silent.\n")

    base = {s: ping(ser, s) for s in IDS}
    print("baseline reachable: " + " ".join(f"ID{s}={'Y' if base[s] else 'N'}" for s in IDS))

    ser.reset_input_buffer()
    ser.write(_reboot_pkt(target))
    t0 = time.perf_counter()

    log = {s: [] for s in IDS}
    while time.perf_counter() - t0 < 1.1:
        for s in IDS:
            r = ping(ser, s)
            log[s].append(((time.perf_counter() - t0) * 1000, r))

    print("\nDuring the reboot window:")
    print(f"{'ID':>4} {'loc':>4} {'reach%':>7} {'silent-span(ms)':>18}")
    rows = []
    for s in IDS:
        seq = log[s]
        reach = sum(1 for _, r in seq if r)
        frac = 100.0 * reach / len(seq)
        sil = [t for t, r in seq if not r]
        span = f"{min(sil):.0f}-{max(sil):.0f}" if sil else "-"
        rows.append((GT[s], s, frac, span))
        print(f"{s:>4} {GT[s]:>4} {frac:>6.0f}% {span:>18}")

    # interpret: who got knocked out (besides target)?
    knocked = sorted((loc, s) for loc, s, frac, _ in rows if frac < 50 and s != target)
    print(f"\n  target ID{target} is at location {GT[target]}.")
    print(f"  knocked out (besides target): " +
          (", ".join(f"ID{s}(loc{loc})" for loc, s in knocked) or "none"))
    downstream = sorted(s for s in IDS if GT[s] > GT[target])
    print(f"  true downstream (loc > {GT[target]}): " +
          (", ".join(f"ID{s}(loc{GT[s]})" for s in downstream) or "none"))


if __name__ == "__main__":
    main()
