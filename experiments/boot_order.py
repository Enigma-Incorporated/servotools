#!/usr/bin/env python3
"""Does reboot wake-time correlate with daisy-chain position?

Now that the servos have distinct IDs we can measure each one's absolute
wake time after a broadcast reboot (instr 0x08). We reboot repeatedly and,
each reboot, finely poll ONE id until it first answers -- recording its wake
time. Interleaving ids across reboots cancels drift. Then compare mean wake
times against the KNOWN chain order (ID1 nearest -> ID3 farthest).

If wake time rises with chain distance -> boot order reveals position
(e.g. far servo browns out longer on shared inrush). If it's an unrelated
fixed permutation -> boot time is intrinsic per-unit variance.
"""

from __future__ import annotations

import statistics
import time

import serial as pyserial

from _common import find_port

IDS = [1, 2, 3]
CHAIN_NEAR_TO_FAR = [1, 3, 2]  # GROUND TRUTH from wiggle (user-observed)
REBOOTS_PER_ID = 12
SKIP_S = 0.76  # silence before the ~797ms wake edge


def _cs(b):
    return (~sum(b)) & 0xFF


def _ping(sid):
    body = bytes([sid, 0x02, 0x01])
    return b"\xff\xff" + body + bytes([_cs(body)])


REBOOT = b"\xff\xff\xfe\x02\x08" + bytes([_cs(b"\xfe\x02\x08")])
PINGS = {s: _ping(s) for s in IDS}


def _woke(resp, sid):
    """True if resp contains a valid ping reply from sid."""
    i = 0
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            pkt = resp[i:i + 6]
            if len(pkt) >= 6 and ((~sum(pkt[2:-1])) & 0xFF) == pkt[5]:
                return True
        i += 1
    return False


def wake_time(ser, sid):
    """Reboot all; return ms until `sid` first answers."""
    ser.reset_input_buffer()
    ser.write(REBOOT)
    t0 = time.perf_counter()
    time.sleep(SKIP_S)
    while time.perf_counter() - t0 < 1.2:
        ser.reset_input_buffer()
        ser.write(PINGS[sid])
        r = ser.read(12)
        if _woke(r, sid):
            return (time.perf_counter() - t0) * 1000.0
    return None


def main() -> None:
    port = find_port()
    ser = pyserial.Serial(port, 1_000_000, timeout=0.0015)
    time.sleep(0.05)
    print(f"Port: {port}")
    print(f"Ground-truth chain near->far: " +
          " -> ".join(f"ID{s}" for s in CHAIN_NEAR_TO_FAR))
    print(f"Measuring wake time per ID over {REBOOTS_PER_ID} reboots each "
          f"(interleaved)\n")

    samples = {s: [] for s in IDS}
    for rnd in range(REBOOTS_PER_ID):
        for sid in IDS:
            t = wake_time(ser, sid)
            if t is not None:
                samples[sid].append(t)
            time.sleep(0.25)
    ser.close()

    print(f"{'ID':>3}  {'mean':>7}  {'stdev':>6}  {'min':>6}  {'max':>6}  n")
    means = {}
    for sid in IDS:
        xs = samples[sid]
        if not xs:
            print(f"{sid:>3}  (no data)")
            continue
        means[sid] = statistics.mean(xs)
        sd = statistics.pstdev(xs) if len(xs) > 1 else 0.0
        print(f"{sid:>3}  {means[sid]:7.2f}  {sd:6.2f}  {min(xs):6.1f}  "
              f"{max(xs):6.1f}  {len(xs)}")

    if len(means) == len(IDS):
        boot_order = sorted(IDS, key=lambda s: means[s])
        spread = max(means.values()) - min(means.values())
        print(f"\nBoot order fastest->slowest: " +
              " -> ".join(f"ID{s}" for s in boot_order))
        print(f"Ground-truth near->far:      " +
              " -> ".join(f"ID{s}" for s in CHAIN_NEAR_TO_FAR))
        print(f"Spread of means: {spread:.2f} ms")
        if boot_order == CHAIN_NEAR_TO_FAR:
            print("=> MATCHES chain order (nearest boots fastest). "
                  "Boot time DOES map to position.")
        elif boot_order == CHAIN_NEAR_TO_FAR[::-1]:
            print("=> REVERSE of chain order (farthest boots fastest). "
                  "Still a clean positional mapping.")
        else:
            print("=> Does NOT map to chain order -> boot time is intrinsic "
                  "per-unit variance, not position.")


if __name__ == "__main__":
    main()
