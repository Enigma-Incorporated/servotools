#!/usr/bin/env python3
"""Dump the raw bytes a servo (or a colliding pair) puts on the bus.

Read-only. Sends a Present_Position read to one ID and prints exactly what came
back, byte for byte, with the checksum verdict. With one servo you get a clean
status packet every time; with two servos sharing the ID you get mangled frames,
and the occasional clean one reveals that there are two distinct positions.

    python experiments/show_collision.py 1 --samples 12
    python experiments/show_collision.py --ping 1 3
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import serial as pyserial  # noqa: E402

from servotools import registers as reg  # noqa: E402
from servotools.bus import INST_PING, INST_READ, checksum, packet  # noqa: E402
from servotools.config import BAUD  # noqa: E402
from servotools.ports import NOT_FOUND, find_port  # noqa: E402


def hexs(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def frames(data: bytes):
    """Every FF FF-framed packet in the buffer, with its checksum verdict."""
    out = []
    i = 0
    while i < len(data) - 3:
        if data[i] == 0xFF and data[i + 1] == 0xFF:
            ln = data[i + 3]
            end = i + 4 + ln
            if 2 <= ln <= 8 and end <= len(data):
                pkt = data[i:end]
                out.append((pkt, checksum(pkt[2:-1]) == pkt[-1]))
                i = end
                continue
        i += 1
    return out


def sample(ser, sid: int, n: int, addr: int, width: int):
    rows = []
    for _ in range(n):
        ser.reset_input_buffer()
        ser.write(packet(sid, INST_READ, bytes([addr, width])))
        time.sleep(0.004)
        rows.append(ser.read(32))
        time.sleep(0.003)
    return rows


def show_reads(ser, sid: int, n: int) -> None:
    addr, width = reg.PRESENT_POSITION, 2
    print(f"\nREAD Present_Position from ID {sid}   "
          f"-> {hexs(packet(sid, INST_READ, bytes([addr, width])))}")
    print(f"{'':4} {'raw bytes returned':<44} verdict")
    print("-" * 78)

    clean, mangled, silent = 0, 0, 0
    positions = Counter()
    for k, raw in enumerate(sample(ser, sid, n, addr, width), 1):
        if not raw:
            silent += 1
            print(f"{k:>3}. {'(silence)':<44} no reply")
            continue
        found = frames(raw)
        good = [p for p, ok in found if ok]
        if good and len(found) == 1:
            clean += 1
            pos = good[0][5] | (good[0][6] << 8)
            positions[pos] += 1
            verdict = f"OK    checksum valid, position={pos}"
        else:
            mangled += 1
            verdict = "BAD   checksum fails: two servos answered at once"
        print(f"{k:>3}. {hexs(raw):<44} {verdict}")

    print("-" * 78)
    print(f"     {clean} clean, {mangled} mangled, {silent} silent")
    if positions:
        listed = ", ".join(f"{p} (x{c})" for p, c in sorted(positions.items()))
        print(f"     distinct positions seen: {listed}")
    if len(positions) > 1:
        print(f"     -> {len(positions)} servos are answering to ID {sid}")
    elif mangled:
        print(f"     -> collisions prove >=2 servos answer to ID {sid}")
    else:
        print(f"     -> exactly one servo at ID {sid}")


def show_pings(ser, ids) -> None:
    print(f"\n{'':4} {'PING sent':<26} {'reply':<26} verdict")
    print("-" * 78)
    for sid in ids:
        out = packet(sid, INST_PING)
        ser.reset_input_buffer()
        ser.write(out)
        time.sleep(0.004)
        raw = ser.read(32)
        found = frames(raw)
        if found and found[0][1] and len(found) == 1:
            err = found[0][0][4]
            verdict = f"OK    one servo, error byte 0x{err:02x}"
        elif not raw:
            verdict = "--    no reply"
        else:
            verdict = "BAD   checksum fails: collision"
        print(f"ID{sid:<2} {hexs(out):<26} {hexs(raw) or '(silence)':<26} {verdict}")
    print("-" * 78)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("id", nargs="?", type=int, help="ID to read from repeatedly")
    ap.add_argument("--ping", nargs="+", type=int, metavar="ID", help="ping these IDs once each")
    ap.add_argument("--samples", type=int, default=12)
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        print(NOT_FOUND)
        return 1

    ser = pyserial.Serial(port, BAUD, timeout=0.006)
    time.sleep(0.05)
    print(f"Port: {port}  baud {BAUD}")
    if args.ping:
        show_pings(ser, args.ping)
    if args.id is not None:
        show_reads(ser, args.id, args.samples)
    ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
