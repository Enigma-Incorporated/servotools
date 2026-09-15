#!/usr/bin/env python3
"""Measure the reboot recovery edge at high resolution.

INST 0x08 (broadcast) reboots both servos. We poll ID 1 as fast as
possible and watch the recovery sequence:

    silence (both booting) -> CLEAN (one servo up, other still booting)
                           -> collision (both up again)

The CLEAN stretch is the boot-time difference between the two units.
If it is wide enough (> a few ms), we can write a new ID into just the
first servo during that window.

Observe-only (reboots + pings). Repeats a few cycles to check consistency.
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


PING = _pkt(1, 0x01)
REBOOT = _pkt(0xFE, 0x08)


def _classify(resp: bytes) -> str:
    if not resp:
        return "-"
    i = 0
    clean = False
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] != 0xFF:
            ln = resp[i + 3]
            end = i + 4 + ln
            if 2 <= ln <= 8 and end <= len(resp):
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1]:
                    clean = True
                    i = end
                    continue
        i += 1
    return "C" if clean else "x"  # C=clean single, x=collision/garbage


def one_cycle(ser, dur=2.0):
    ser.reset_input_buffer()
    ser.write(REBOOT)
    t0 = time.perf_counter()
    samples = []  # (t_ms, class)
    while time.perf_counter() - t0 < dur:
        ser.write(PING)
        time.sleep(0.0015)
        r = ser.read(16)
        t = (time.perf_counter() - t0) * 1000.0
        samples.append((t, _classify(r)))
        ser.reset_input_buffer()
    return samples


def runs(samples):
    out = []
    for t, c in samples:
        if out and out[-1][0] == c:
            out[-1][2] = t
        else:
            out.append([c, t, t])
    return out


def main() -> None:
    port = find_port()
    if port is None:
        raise SystemExit("No adapter found.")
    ser = pyserial.Serial(port, 1_000_000, timeout=0.05)
    time.sleep(0.05)
    print(f"Port: {port}  baud 1000000")
    print("Legend: '-' silence(both booting)  'C' clean(one up)  'x' collision(both up)\n")

    for cycle in range(4):
        s = one_cycle(ser)
        r = runs(s)
        poll_ms = (s[-1][0] - s[0][0]) / max(1, len(s) - 1)
        print(f"--- reboot cycle {cycle}  ({len(s)} polls, ~{poll_ms:.1f}ms/poll) ---")
        for c, t0, t1 in r:
            name = {"-": "silence ", "C": "ONE-UP  ", "x": "both-up "}[c]
            width = t1 - t0
            mark = "  <-- exploitable window" if c == "C" and width > 8 else ""
            print(f"    {t0:7.1f}-{t1:7.1f}ms  [{name}] {width:6.1f}ms{mark}")
        # find first ONE-UP window after the initial silence
        first_silence = next((i for i, x in enumerate(r) if x[0] == "-"), None)
        if first_silence is not None:
            after = r[first_silence:]
            win = next((x for x in after if x[0] == "C"), None)
            if win:
                print(f"    => first ONE-UP after reboot: {win[2]-win[1]:.1f}ms wide, "
                      f"starting at {win[1]:.1f}ms")
        print()
        time.sleep(0.3)

    ser.close()


if __name__ == "__main__":
    main()
