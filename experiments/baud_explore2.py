#!/usr/bin/env python3
"""Idea #4, thorough: does any UNDOCUMENTED baud-register value exceed 1 Mbps?

Baud reg = addr 6, writes switch immediately, index 0 = 1M is the fastest
documented index. Here we test undocumented values (8+) on ID1: write it,
find where ID1 landed (scan a comprehensive baud set incl custom 2M/3M),
record the actual baud, restore to 1M. Stop-on-loss with wide recovery.
"""

from __future__ import annotations

import time

import serial as pyserial

from _common import find_port

BAUD_ADDR = 6
LOCK = 55
POS = 56
SID = 1

STD = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]
CUSTOM = [2000000, 1500000, 3000000, 4000000, 1250000, 1843200]
WIDE = STD + [19200, 14400, 9600, 4800]
TEST_VALUES = [8, 9, 10, 11, 12, 16]


def cs(b):
    return (~sum(b)) & 0xFF


def pkt(sid, inst, params=b""):
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([cs(body)])


def read1(ser, sid, addr):
    ser.reset_input_buffer()
    ser.write(pkt(sid, 2, bytes([addr, 1])))
    time.sleep(0.004)
    r = ser.read(8)
    i = 0
    while i < len(r) - 6:
        if r[i] == 0xFF and r[i + 1] == 0xFF and r[i + 2] == sid:
            p = r[i:i + 7]
            if len(p) >= 7 and ((~sum(p[2:-1])) & 0xFF) == p[-1] and p[4] == 0:
                return p[5]
        i += 1
    return None


def write1(ser, sid, addr, val):
    ser.reset_input_buffer()
    ser.write(pkt(sid, 3, bytes([addr, val])))
    time.sleep(0.02)
    ser.read(16)


def opn(port, b):
    try:
        s = pyserial.Serial(port, b, timeout=0.006)
        time.sleep(0.04)
        return s
    except Exception:
        return None


def find(port, bauds):
    for b in bauds:
        s = opn(port, b)
        if s is None:
            continue
        if read1(s, SID, POS) is not None:
            return b, s
        s.close()
    return None, None


def main() -> None:
    port = find_port()

    # which custom (>1M) bauds can the adapter even open?
    openable = []
    for b in CUSTOM:
        s = opn(port, b)
        if s:
            openable.append(b)
            s.close()
    print(f"Adapter can OPEN these >1M bauds: {openable or 'none'}")
    probe = [1000000] + openable + STD + WIDE
    probe = list(dict.fromkeys(probe))  # dedup, keep order

    # ensure ID1 at 1M
    b, s = find(port, probe)
    if b is None:
        print("ID1 not found at any baud!")
        return
    if b != 1000000:
        print(f"ID1 at {b}; restoring to 1M")
        write1(s, SID, LOCK, 0)
        write1(s, SID, BAUD_ADDR, 0)
        s.close()
        b, s = find(port, [1000000] + probe)
    print(f"ID1 at {b}. Probing undocumented baud values...\n")

    results = {}
    lost = False
    for v in TEST_VALUES:
        write1(s, SID, LOCK, 0)
        write1(s, SID, BAUD_ADDR, v)   # immediate switch
        s.close()
        nb, ns = find(port, probe)
        if nb is None:
            print(f"  baud value {v}: ID1 LOST (not at any probed baud). Wide recovery...")
            nb, ns = find(port, probe + [b for b in range(2000000, 250000, -100000)])
            if nb is None:
                print(f"  !! could not recover ID1; left at unknown baud (value {v}).")
                lost = True
                break
        results[v] = nb
        print(f"  baud value {v:>3} -> actual baud {nb}")
        # restore to 1M
        write1(ns, SID, LOCK, 0)
        write1(ns, SID, BAUD_ADDR, 0)
        ns.close()
        _, s = find(port, [1000000] + probe)
        if s is None:
            print("  lost ID1 during restore!")
            lost = True
            break

    if not lost and s:
        write1(s, SID, LOCK, 1)
        ok = read1(s, SID, POS) is not None
        s.close()
        print(f"\nID1 restored, locked, reachable at 1M: {ok}")

    print("\nResults (register value -> actual baud):")
    for v, nb in results.items():
        flag = "  <-- ABOVE 1 Mbps!" if nb and nb > 1000000 else ""
        print(f"  {v:>3} -> {nb}{flag}")
    mx = max([nb for nb in results.values() if nb] + [1000000])
    if mx > 1000000:
        print(f"\n=> Found a baud ABOVE 1 Mbps ({mx})! Idea #4 is testable.")
    else:
        print("\n=> No value exceeds 1 Mbps. Idea #4 is HARDWARE-BLOCKED (1M is the cap).")


if __name__ == "__main__":
    main()
