#!/usr/bin/env python3
"""Show why you cannot just set one servo's ID when two of them share it.

DESTRUCTIVE: collapses every servo onto one ID, then tries the naive fix and
shows what actually happens. Run `servo autonumber` afterwards.

    python experiments/show_setid_blocker.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import serial as pyserial  # noqa: E402

from servotools import registers as reg  # noqa: E402
from servotools.bus import (  # noqa: E402
    INST_PING,
    INST_READ,
    INST_WRITE,
    Bus,
    checksum,
    packet,
)
from servotools.config import BAUD, BROADCAST_ID  # noqa: E402
from servotools.ports import NOT_FOUND, find_port  # noqa: E402

FLAT = 1
TARGET = 2


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b) if b else "(silence)"


def valid(raw: bytes) -> bool:
    """True if the buffer is exactly one checksum-valid packet."""
    if len(raw) < 6 or raw[0] != 0xFF or raw[1] != 0xFF:
        return False
    end = 4 + raw[3]
    return end == len(raw) and checksum(raw[2:-1]) == raw[-1]


def shot(ser, pkt: bytes, label: str, wait: float = 0.006) -> bytes:
    ser.reset_input_buffer()
    ser.write(pkt)
    time.sleep(wait)
    raw = ser.read(32)
    verdict = "valid" if valid(raw) else ("no reply" if not raw else "CHECKSUM FAILS")
    print(f"  {label:<34} {hexs(pkt):<26} -> {hexs(raw):<26} {verdict}")
    return raw


def banner(n: int, title: str) -> None:
    print(f"\n{'=' * 96}\n{n}. {title}\n{'=' * 96}")
    print(f"  {'':<34} {'sent':<26}    {'received':<26} verdict")


def main() -> int:
    port = find_port()
    if port is None:
        print(NOT_FOUND)
        return 1
    ser = pyserial.Serial(port, BAUD, timeout=0.006)
    bus = Bus(ser)
    time.sleep(0.05)
    print(f"Port: {port}  baud {BAUD}")

    banner(1, f"Baseline: two servos on distinct IDs, then collapse both onto ID {FLAT}")
    found = bus.scan(range(1, 20))
    print(f"  before: {found}")
    for addr, val, name in [(reg.TORQUE_ENABLE, 0, "torque off"), (reg.LOCK, 0, "unlock"),
                            (reg.ID, FLAT, f"set ID={FLAT}"), (reg.LOCK, 1, "re-lock")]:
        ser.reset_input_buffer()
        ser.write(packet(BROADCAST_ID, INST_WRITE, bytes([addr, val])))
        time.sleep(0.2)
    print(f"  after broadcast flatten: {bus.scan(range(1, 20))}")

    banner(2, f"Both servos now answer to ID {FLAT}. A position read proves it")
    a = shot(ser, packet(FLAT, INST_READ, bytes([reg.PRESENT_POSITION, 2])), "READ Present_Position")
    for _ in range(3):
        shot(ser, packet(FLAT, INST_READ, bytes([reg.PRESENT_POSITION, 2])), "READ Present_Position")
    print(f"\n  The reply is the bitwise OR of both servos' packets, so the checksum fails.")
    print(f"  That is how we know there are two. It is also all we know: the protocol")
    print(f"  offers no way to say 'the other one at ID {FLAT}'. The ID *is* the address.")

    banner(3, f"THE BLOCKER: try to move just one of them to ID {TARGET}")
    shot(ser, packet(FLAT, INST_WRITE, bytes([reg.LOCK, 0])), "WRITE lock=0")
    shot(ser, packet(FLAT, INST_WRITE, bytes([reg.ID, TARGET])), f"WRITE ID={TARGET}")
    time.sleep(0.2)
    shot(ser, packet(TARGET, INST_WRITE, bytes([reg.LOCK, 1])), "WRITE lock=1")
    time.sleep(0.2)

    banner(4, "What actually happened")
    shot(ser, packet(FLAT, INST_PING), f"PING old ID {FLAT}")
    shot(ser, packet(TARGET, INST_PING), f"PING new ID {TARGET}")
    shot(ser, packet(TARGET, INST_READ, bytes([reg.PRESENT_POSITION, 2])),
         f"READ Present_Position ID {TARGET}")
    print(f"\n  final scan: {bus.scan(range(1, 20))}")

    print(f"""
{'=' * 96}
  Both servos heard the write, because both were listening on ID {FLAT}.
  Both are now ID {TARGET}. The collision did not get resolved -- it moved.

  Worse, the bus said everything was fine. Both servos ACK a write with the
  same status packet, so the two ACKs OR together into a *valid* packet and
  the write looks like it succeeded on one servo.

  There is no addressing trick left: no serial number, no index, no
  "second device at this ID". The only thing that ever differs between two
  identical servos on one wire is WHEN they are listening -- which is why the
  fix is a reboot and a race, not a write.
{'=' * 96}
  Run `servo autonumber` to clean up.""")
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
