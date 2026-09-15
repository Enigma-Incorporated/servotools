#!/usr/bin/env python3
"""Measure the true single-servo boot gap at sub-ms resolution."""
import time
import serial as pyserial
from _common import find_port


def _cs(b):
    return (~sum(b)) & 0xFF


def _pkt(sid, inst, params=b""):
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([_cs(body)])


REBOOT = _pkt(0xFE, 0x08)


def _decode(resp, want):
    if not resp:
        return "-", None
    i = 0
    while i < len(resp) - 5:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] != 0xFF:
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == 4 and end <= len(resp):
                p = resp[i:end]
                if p[2] == want and ((~sum(p[2:-1])) & 0xFF) == p[-1]:
                    return "C", p[5] | (p[6] << 8)
        i += 1
    return "x", None


def main():
    port = find_port()
    ser = pyserial.Serial(port, 1_000_000, timeout=0.0012)
    time.sleep(0.05)

    # detect current doubled id quickly
    doubled = None
    for sid in (1, 2, 3, 4, 5):
        p = _pkt(sid, 0x02, bytes([56, 2]))
        coll = 0
        for _ in range(6):
            ser.reset_input_buffer()
            ser.write(p)
            time.sleep(0.003)
            c, _ = _decode(ser.read(16), sid)
            if c == "x":
                coll += 1
        if coll >= 2:
            doubled = sid
            break
    print(f"doubled id = {doubled}")
    if doubled is None:
        print("no doubled id detected")
        ser.close()
        return
    rp = _pkt(doubled, 0x02, bytes([56, 2]))

    for cyc in range(6):
        ser.reset_input_buffer()
        ser.write(REBOOT)
        t0 = time.perf_counter()
        time.sleep(0.70)
        samples = []
        while time.perf_counter() - t0 < 0.90:
            ser.reset_input_buffer()
            ser.write(rp)
            c, pos = _decode(ser.read(16), doubled)
            samples.append(((time.perf_counter() - t0) * 1000, c, pos))
        first_clean = None
        first_coll_after = None
        for t, c, pos in samples:
            if c == "C" and first_clean is None:
                first_clean = t
            if c == "x" and first_clean is not None and first_coll_after is None and t > first_clean:
                first_coll_after = t
        n_clean = sum(1 for _, c, _ in samples if c == "C")
        avg_dt = (samples[-1][0] - samples[0][0]) / max(1, len(samples) - 1)
        gap = (first_coll_after - first_clean) if (first_clean and first_coll_after) else None
        print(f"cyc{cyc}: polls={len(samples)} dt~{avg_dt:.2f}ms  "
              f"first_clean={first_clean and round(first_clean, 1)}ms  "
              f"clean_polls={n_clean}  one-up gap~{gap and round(gap, 1)}ms")
        time.sleep(0.3)
    ser.close()


main()
