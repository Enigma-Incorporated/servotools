#!/usr/bin/env python3
"""Can we 'decode' two same-ID servos apart from their colliding responses?

Strategy: hammer ID 1 with the same request many times. On a half-duplex
bus, two servos answering at once collide -> checksum fails. But thanks to
tiny clock/timing skew, occasionally ONE servo 'wins' the line and we get a
clean, checksum-valid packet. Over many samples those clean packets cluster
into the DISTINCT register values of the two physical servos.

So: checksum = the filter that separates signal from collision.
Distinct clean values = the two servos, decoded statistically.

Read-only. No writes.
"""

from __future__ import annotations

import collections
import time

import serial as pyserial

from _common import find_port


def _cs(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _ping(sid: int) -> bytes:
    b = bytes([sid, 0x02, 0x01])
    return b"\xff\xff" + b + bytes([_cs(b)])


def _read(sid: int, addr: int, n: int) -> bytes:
    b = bytes([sid, 0x04, 0x02, addr, n])
    return b"\xff\xff" + b + bytes([_cs(b)])


def _clean_packets(data: bytes):
    """Yield (id, err, params) for every checksum-valid framed packet."""
    i = 0
    while i < len(data) - 3:
        if data[i] == 0xFF and data[i + 1] == 0xFF and data[i + 2] != 0xFF:
            sid = data[i + 2]
            ln = data[i + 3]
            end = i + 4 + ln
            if 2 <= ln <= 8 and end <= len(data):
                pkt = data[i:end]
                body = pkt[2:-1]
                if ((~sum(body)) & 0xFF) == pkt[-1]:
                    yield sid, pkt[4], bytes(pkt[5:-1])
                    i = end
                    continue
        i += 1


def sample(ser, pkt, n, decode):
    clean = collections.Counter()
    n_clean = n_collide = n_empty = 0
    for _ in range(n):
        ser.reset_input_buffer()
        ser.write(pkt)
        time.sleep(0.015)
        resp = ser.read(64)
        time.sleep(0.015)
        if not resp:
            n_empty += 1
            continue
        got = list(_clean_packets(resp))
        if got:
            n_clean += 1
            for sid, err, params in got:
                clean[decode(sid, err, params)] += 1
        else:
            n_collide += 1
    return clean, n_clean, n_collide, n_empty


def main() -> None:
    port = find_port()
    if port is None:
        raise SystemExit("No adapter found.")
    ser = pyserial.Serial(port, 1_000_000, timeout=0.1)
    time.sleep(0.05)
    print(f"Port: {port}  baud: 1000000\n")

    N = 120
    trials = [
        ("PING", _ping(1), lambda s, e, p: f"id={s} err={e}"),
        ("POSITION (addr56)", _read(1, 56, 2),
         lambda s, e, p: f"pos={p[0] | (p[1] << 8)}" if len(p) == 2 else f"raw={p.hex()}"),
        ("TEMP (addr63)", _read(1, 63, 1),
         lambda s, e, p: f"temp={p[0]}C" if len(p) == 1 else f"raw={p.hex()}"),
        ("VOLTAGE (addr62)", _read(1, 62, 1),
         lambda s, e, p: f"v={p[0] / 10:.1f}V" if len(p) == 1 else f"raw={p.hex()}"),
    ]

    for label, pkt, decode in trials:
        clean, nc, ncol, nem = sample(ser, pkt, N, decode)
        print(f"== {label}: {nc} clean / {ncol} collided / {nem} empty (of {N}) ==")
        for val, cnt in clean.most_common():
            bar = "#" * min(cnt, 40)
            print(f"     {val:<16} x{cnt:<3} {bar}")
        distinct = len(clean)
        print(f"     -> {distinct} distinct clean value(s)\n")

    ser.close()


if __name__ == "__main__":
    main()
