#!/usr/bin/env python3
"""Read-only diagnostic for the two-servos-share-an-ID situation.

Looks at the RAW bytes on the bus so we can tell the difference between
"one servo at ID 1" and "two servos both at ID 1 colliding". With two
servos at the same ID, a single ping triggers two overlapping status
packets -> bus contention -> corrupted/checksum-failing reply, or
occasionally two back-to-back packets.

No writes. Safe to run repeatedly.
"""

from __future__ import annotations

import time

import serial as pyserial

from _common import STANDARD_BAUDS, find_port


def _cs(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _ping(sid: int) -> bytes:
    body = bytes([sid, 0x02, 0x01])  # len=2, inst=PING
    return b"\xff\xff" + body + bytes([_cs(body)])


def _read(sid: int, addr: int, n: int) -> bytes:
    body = bytes([sid, 0x04, 0x02, addr, n])  # len=4, inst=READ
    return b"\xff\xff" + body + bytes([_cs(body)])


def _hex(b: bytes) -> str:
    return " ".join(f"{x:02x}" for x in b)


def _parse_status_packets(data: bytes) -> list[dict]:
    """Find all FF FF .. framed packets and check their checksums."""
    out = []
    i = 0
    while i < len(data) - 3:
        if data[i] == 0xFF and data[i + 1] == 0xFF:
            sid = data[i + 2]
            ln = data[i + 3]
            end = i + 4 + ln  # ff ff id len ... (len includes err+params+cs)
            if ln >= 2 and end <= len(data):
                pkt = data[i : end]
                body = pkt[2:-1]
                ok = ((~sum(body)) & 0xFF) == pkt[-1]
                out.append({"id": sid, "len": ln, "raw": pkt, "checksum_ok": ok})
                i = end
                continue
        i += 1
    return out


def probe(port: str, baud: int) -> None:
    ser = pyserial.Serial(port, baud, timeout=0.15)
    time.sleep(0.05)

    print(f"\n=== baud {baud} ===")

    for label, pkt in [
        ("PING id=1   ", _ping(1)),
        ("PING bcast  ", _ping(0xFE)),
        ("READ id=1 ID", _read(1, 0x05, 1)),
    ]:
        # Repeat a few times to expose intermittent collisions
        reps = []
        for _ in range(6):
            ser.reset_input_buffer()
            ser.write(pkt)
            time.sleep(0.02)
            resp = ser.read(64)
            reps.append(resp)
            time.sleep(0.02)

        nonempty = [r for r in reps if r]
        print(f"\n  {label} tx={_hex(pkt)}")
        if not nonempty:
            print("    no response (x6)")
            continue
        for j, r in enumerate(reps):
            pkts = _parse_status_packets(r)
            tag = ",".join(
                f"id{p['id']}/{'ok' if p['checksum_ok'] else 'BAD'}" for p in pkts
            )
            extra = "" if pkts else "(unframed)"
            print(f"    rep{j}: {len(r):>2}B  {_hex(r)}   [{tag}{extra}]")

    ser.close()


def main() -> None:
    port = find_port()
    if port is None:
        raise SystemExit("No Waveshare adapter found.")
    print(f"Port: {port}")
    # Try 1Mbps first (factory default), then the rest.
    for baud in [1_000_000] + [b for b in STANDARD_BAUDS if b != 1_000_000]:
        probe(port, baud)


if __name__ == "__main__":
    main()
