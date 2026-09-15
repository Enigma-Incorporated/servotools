#!/usr/bin/env python3
"""Find the single-servo window at the reboot recovery edge, precisely.

Key idea: probe with a POSITION read, not a ping. The two servos sit at
different shaft angles, so a position read only returns a CLEAN packet when
EXACTLY ONE servo is alive. When both are up it always collides. So:

    silence            -> both still booting
    CLEAN pos=X        -> exactly one servo up  (THE WINDOW)
    collision          -> both up

Fine-grained polling (low timeout) to resolve the edge to a few ms.
Observe-only. Repeats several reboot cycles.
"""

from __future__ import annotations

import time

import serial as pyserial

from _common import find_port


def _cs(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _pkt(sid: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([_cs(body)])


READ_POS = _pkt(1, 0x02, bytes([56, 2]))
REBOOT = _pkt(0xFE, 0x08)


def _decode_pos(resp: bytes):
    """Return ('C', pos) if one clean position packet, ('x', None) collision,
    ('-', None) silence."""
    if not resp:
        return "-", None
    i = 0
    while i < len(resp) - 5:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] != 0xFF:
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == 4 and end <= len(resp):  # id len err lo hi cs
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1]:
                    return "C", pkt[5] | (pkt[6] << 8)
            i += 1
        else:
            i += 1
    return "x", None


def cycle(ser):
    ser.reset_input_buffer()
    ser.write(REBOOT)
    t0 = time.perf_counter()
    # Skip most of the ~780ms boot silence, then fine-poll the edge.
    time.sleep(0.62)
    samples = []
    while time.perf_counter() - t0 < 1.4:
        ser.reset_input_buffer()
        ser.write(READ_POS)
        time.sleep(0.001)
        r = ser.read(16)
        t = (time.perf_counter() - t0) * 1000.0
        cls, pos = _decode_pos(r)
        samples.append((t, cls, pos))
    return samples


def main() -> None:
    port = find_port()
    if port is None:
        raise SystemExit("No adapter found.")
    ser = pyserial.Serial(port, 1_000_000, timeout=0.004)
    time.sleep(0.05)
    print(f"Port: {port}  baud 1000000")
    print("Probe = POSITION read.  '-'=silence  'C(pos)'=ONE servo up  'x'=both up\n")

    for c in range(4):
        s = cycle(ser)
        # compress to runs, but keep position values for C runs
        runs = []
        for t, cls, pos in s:
            key = (cls, pos if cls == "C" else None)
            if runs and runs[-1][0] == key:
                runs[-1][2] = t
            else:
                runs.append([key, t, t])
        poll_ms = (s[-1][0] - s[0][0]) / max(1, len(s) - 1)
        print(f"--- cycle {c}: {len(s)} polls @ ~{poll_ms:.1f}ms after the 620ms skip ---")
        first_one_up = None
        for (cls, pos), t0, t1 in runs:
            if cls == "-":
                label = "silence"
            elif cls == "C":
                label = f"ONE-UP pos={pos}"
                if first_one_up is None:
                    first_one_up = (t0, t1, pos)
            else:
                label = "both-up(collision)"
            print(f"    {t0:7.1f}-{t1:7.1f}ms  ({t1-t0:6.1f}ms)  {label}")
        if first_one_up:
            t0, t1, pos = first_one_up
            print(f"    => FIRST single-servo window: {t1-t0:.1f}ms wide @ {t0:.1f}ms, pos={pos}")
        print()
        time.sleep(0.3)

    ser.close()


if __name__ == "__main__":
    main()
