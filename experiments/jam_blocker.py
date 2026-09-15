#!/usr/bin/env python3
"""'Wire blocker' test: can a servo's bus jam block the adapter's command to
ANOTHER servo, position-dependently?

Mechanism to create jam-during-command on a half-duplex bus:
  - jammer J: Return_Delay=0 (responds instantly); fire a long read at it so
    it drives the line for ~500us.
  - in the SAME write buffer, append a ping to target C, so the adapter
    transmits C's command WHILE J is still jamming -> they contend on the wire.
  - give C a long Return_Delay so C's reply lands AFTER J's jam clears, so we
    can read it cleanly. If C replied, it heard the ping despite the jam.

If 'jam-survival' drops with distance from the adapter, jamming is spatially
graded -> a location signal. If flat, the bus is equipotential (idea dead).
Scored vs eyes ground truth. No movement.
"""

from __future__ import annotations

import sys
import time

import serial as pyserial

from _common import find_port

IDS = [1, 2, 3, 4, 5, 6]
GT = {1: 3, 2: 4, 3: 1, 4: 2, 5: 6, 6: 5}   # id -> location (1=near adapter)
ADDR_RDT = 7
TRIALS = 25


def _cs(b):
    return (~sum(b)) & 0xFF


def ping_pkt(s):
    b = bytes([s, 0x02, 0x01])
    return b"\xff\xff" + b + bytes([_cs(b)])


def read_pkt(s, addr, n):
    b = bytes([s, 0x04, 0x02, addr, n])
    return b"\xff\xff" + b + bytes([_cs(b)])


def writeb_pkt(s, addr, val):
    b = bytes([s, 0x04, 0x03, addr, val])
    return b"\xff\xff" + b + bytes([_cs(b)])


def get_rdt(ser, s):
    ser.reset_input_buffer()
    ser.write(read_pkt(s, ADDR_RDT, 1))
    time.sleep(0.003)
    r = ser.read(8)
    i = 0
    while i < len(r) - 5:
        if r[i] == 0xFF and r[i + 1] == 0xFF and r[i + 2] == s:
            return r[i + 5]
        i += 1
    return None


def set_rdt(ser, s, val):
    ser.reset_input_buffer()
    ser.write(writeb_pkt(s, ADDR_RDT, val))
    time.sleep(0.02)
    ser.read(16)


def heard(resp, sid):
    i = 0
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            pkt = resp[i:i + 6]
            if len(pkt) >= 6 and ((~sum(pkt[2:-1])) & 0xFF) == pkt[5]:
                return True
        i += 1
    return False


def main() -> None:
    jam = int(sys.argv[1]) if len(sys.argv) > 1 else 5   # ID5 = loc6 (far end)
    port = find_port()
    ser = pyserial.Serial(port, 1_000_000, timeout=0.004)
    time.sleep(0.05)
    print(f"Jammer = ID{jam} (location {GT[jam]}). Trials per target: {TRIALS}\n")

    orig = {s: get_rdt(ser, s) for s in IDS}
    set_rdt(ser, jam, 0)                       # jammer fires instantly
    jam_trigger = read_pkt(jam, 0, 50)         # ~500us of line drive

    print(f"{'ID':>3} {'loc':>4} {'base':>6} {'underJam':>9}")
    res = {}
    for c in IDS:
        if c == jam:
            continue
        base = sum(heard((ser.reset_input_buffer(), ser.write(ping_pkt(c)),
                          time.sleep(0.003), ser.read(8))[-1], c)
                   for _ in range(TRIALS))
        set_rdt(ser, c, 254)                   # C replies late, after the jam
        jammed = 0
        for _ in range(TRIALS):
            ser.reset_input_buffer()
            ser.write(jam_trigger + ping_pkt(c))   # command C while J jams
            time.sleep(0.004)
            if heard(ser.read(120), c):
                jammed += 1
        set_rdt(ser, c, orig[c] if orig[c] is not None else 0)
        res[c] = (base, jammed)
        print(f"{c:>3} {GT[c]:>4} {base:>5}/{TRIALS} {jammed:>7}/{TRIALS}")

    set_rdt(ser, jam, orig[jam] if orig[jam] is not None else 0)
    ser.close()

    print("\nJam-survival by location (near -> far):")
    for c in sorted(res, key=lambda x: GT[x]):
        bar = "#" * res[c][1]
        print(f"  loc{GT[c]} ID{c}: {res[c][1]:>2}/{TRIALS}  {bar}")
    print("\n  gradient with location -> position-dependent jamming (signal!)")
    print("  flat -> equipotential bus (idea dead)")


if __name__ == "__main__":
    main()
