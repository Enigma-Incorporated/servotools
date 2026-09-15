#!/usr/bin/env python3
"""Probe whether the two servos reboot at different times.

If a reset/reboot makes both servos restart, tiny per-unit boot-time
variance could open a window where only ONE has come back online. During
that window a ping to ID 1 would return a SINGLE clean packet (one
responder) instead of a collision (two responders). That window is where
we could write a new ID to just one servo.

This script only observes the timeline. It tries a few candidate "reboot"
mechanisms and watches how the bus recovers.

Read-mostly: sends a reset/reboot instruction, then only pings.
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


def _classify(resp: bytes) -> str:
    """single = one clean packet, collide = framed but bad, empty/none."""
    if not resp:
        return "----"  # silence
    # look for a clean framed packet
    i = 0
    clean = False
    framed = False
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] != 0xFF:
            framed = True
            ln = resp[i + 3]
            end = i + 4 + ln
            if 2 <= ln <= 8 and end <= len(resp):
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1]:
                    clean = True
                    i = end
                    continue
        i += 1
    if clean:
        return "CLEAN"  # likely single responder (or perfectly aligned)
    if framed or resp:
        return "coll"  # collision / garbage
    return "----"


def poll_timeline(ser, trigger_pkt, label, dur=0.8):
    print(f"\n=== {label} ===")
    ping = _pkt(1, 0x01)

    # Baseline before trigger
    base = []
    for _ in range(15):
        ser.reset_input_buffer()
        ser.write(ping)
        time.sleep(0.004)
        base.append(_classify(ser.read(32)))
        time.sleep(0.004)
    print(f"  baseline (pre):  {' '.join(base)}")

    # Fire trigger
    ser.reset_input_buffer()
    ser.write(trigger_pkt)
    t0 = time.perf_counter()

    # Poll as fast as we can, timestamped
    timeline = []
    while time.perf_counter() - t0 < dur:
        ser.write(ping)
        time.sleep(0.003)
        r = ser.read(32)
        t = (time.perf_counter() - t0) * 1000.0
        timeline.append((t, _classify(r)))
        ser.reset_input_buffer()

    # Compress into runs
    runs = []
    for t, c in timeline:
        if runs and runs[-1][0] == c:
            runs[-1][2] = t
        else:
            runs.append([c, t, t])
    print(f"  post-trigger timeline ({len(timeline)} polls over {dur*1000:.0f}ms):")
    for c, t_start, t_end in runs:
        print(f"     {t_start:6.1f}-{t_end:6.1f}ms  {c}  ({'single responder!' if c=='CLEAN' else ''})")

    # Did we ever see a stretch of silence (reboot) followed by recovery?
    saw_silence = any(c == "----" for c, _, _ in runs)
    print(f"  -> reboot signature (silence then recovery): {saw_silence}")


def main() -> None:
    port = find_port()
    if port is None:
        raise SystemExit("No adapter found.")
    ser = pyserial.Serial(port, 1_000_000, timeout=0.05)
    time.sleep(0.05)
    print(f"Port: {port}  baud 1000000")

    # Candidate reboot/reset triggers (broadcast 0xFE so both react):
    candidates = [
        ("INST 0x06 (reset/factory) broadcast", _pkt(0xFE, 0x06)),
        ("INST 0x08 (reboot?) broadcast", _pkt(0xFE, 0x08)),
        ("INST 0x64 (reboot?) broadcast", _pkt(0xFE, 0x64)),
    ]
    for label, pkt in candidates:
        poll_timeline(ser, pkt, label)
        time.sleep(0.5)  # let bus settle between trials

    ser.close()


if __name__ == "__main__":
    main()
