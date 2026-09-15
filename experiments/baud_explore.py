#!/usr/bin/env python3
"""Idea #4 feasibility: can the bus run ABOVE 1 Mbps?

A higher baud would make the electrically-farther servo corrupt first =
signal-integrity location signal. But it needs servos that can actually run
faster than 1M. This safely probes the baud register on ID1 only:
  1. identify the baud register (the one reading 0 = 1Mbps; addr5 is ID).
  2. test if a baud write is DEFERRED to reboot (safe to map) or IMMEDIATE.
  3. if deferred, map which register values are accepted (no baud switch, zero
     risk), then report whether any exceeds 1M.
Full restore via try/finally; recovers ID1 if an immediate switch loses it.
"""

from __future__ import annotations

import time

import serial as pyserial

from _common import STANDARD_BAUDS, find_port

ADDR_LOCK = 55
ADDR_POS = 56
SID = 1


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


def main() -> None:
    port = find_port()
    ser = pyserial.Serial(port, 1_000_000, timeout=0.006)
    time.sleep(0.05)
    print(f"Port {port}\n")

    # 1. identify registers near the baud reg
    print("EEPROM dump ID1 addr 3-9:")
    table = {a: read1(ser, SID, a) for a in range(3, 10)}
    for a, v in table.items():
        tag = " <- ID" if a == 5 else ""
        print(f"  addr {a}: {v}{tag}")
    # baud reg = a register (not ID@5) reading 0 (=1Mbps); prefer addr 6 then 4
    cands = [a for a in (6, 4) if table.get(a) == 0]
    if not cands:
        print("\nCannot identify baud register (none of addr 4/6 read 0). Aborting safely.")
        ser.close()
        return
    baud_addr = cands[0]
    print(f"\n-> baud register = addr {baud_addr} (currently 0 = 1 Mbps, the fastest index)\n")

    orig = table[baud_addr]
    write1(ser, SID, ADDR_LOCK, 0)
    try:
        # 2. deferred vs immediate
        write1(ser, SID, baud_addr, 1)
        rb = read1(ser, SID, baud_addr)
        if rb == 1:
            print("baud write is DEFERRED to reboot (still reachable at 1M) -> safe to map\n")
            write1(ser, SID, baud_addr, orig)
            print("value-set -> readback (accepted if equal):")
            valid = []
            for v in range(0, 17):
                write1(ser, SID, baud_addr, v)
                got = read1(ser, SID, baud_addr)
                if got == v:
                    valid.append(v)
                print(f"  set {v:>2} -> {got}")
            write1(ser, SID, baud_addr, orig)
            print(f"\nValid baud indices: {valid}")
            print("Index 0 = 1 Mbps is the FASTEST; higher index = SLOWER.")
            extra = [v for v in valid if v > 7]
            if extra:
                print(f"Undocumented indices accepted: {extra} -- worth a reboot+probe for secret high baud.")
            else:
                print("Only indices 0-7 valid (1M..38400). No setting exceeds 1 Mbps.")
                print("=> Idea #4 is HARDWARE-BLOCKED: no higher-baud regime exists to exploit.")
        else:
            print(f"baud write is IMMEDIATE (ID1 left 1M; readback={rb}). Recovering ID1...")
            ser.close()
            found = None
            for b in [500000] + [x for x in STANDARD_BAUDS if x != 500000]:
                ser = pyserial.Serial(port, b, timeout=0.006)
                time.sleep(0.05)
                if read1(ser, SID, ADDR_POS) is not None:
                    found = b
                    break
                ser.close()
            if found:
                print(f"  recovered ID1 at {found}; restoring to 1 Mbps (index {orig})")
                write1(ser, SID, ADDR_LOCK, 0)
                write1(ser, SID, baud_addr, orig)
                write1(ser, SID, ADDR_LOCK, 1)
            else:
                print("  WARNING: could not find ID1 at standard bauds!")
            print("Immediate-switch mode; index 0 is the fastest, so 1 Mbps is still the cap.")
            ser.close()
            return
    finally:
        # always restore baud reg + lock at 1M
        write1(ser, SID, baud_addr, orig)
        write1(ser, SID, ADDR_LOCK, 1)
        rb = read1(ser, SID, baud_addr)
        print(f"\nrestored ID1 baud reg -> {rb} (lock on); ID1 reachable at 1M: "
              f"{read1(ser, SID, ADDR_POS) is not None}")
    ser.close()


if __name__ == "__main__":
    main()
