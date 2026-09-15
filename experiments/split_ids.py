#!/usr/bin/env python3
"""Give two same-ID servos different IDs WITHOUT unplugging anything.

The trick, end to end:

1. Two servos share ID 1 on one half-duplex bus. Any write to ID 1 hits
   BOTH, so they can't be addressed apart -- normally you must unplug one.

2. INST 0x08 (broadcast) reboots both. They boot in ~800ms but NOT at
   exactly the same instant: there is a ~few-ms window where only the
   faster unit is awake. We detect that window with a POSITION read --
   it returns a clean packet only when exactly one servo answers (the two
   sit at different angles, so two-up always collides).

3. In that window we fire UNLOCK (EEPROM lock addr 55 -> 0) then SET-ID
   (addr 5 -> new). The unlock is the key: only the awake servo gets it,
   and only an unlocked servo accepts the ID write. So exactly one servo
   moves to the new ID; the other stays put. IDs are now different.

We loop reboot->race->verify until the bus shows two distinct IDs.

Safe to re-run. Reboots reset volatile state only; the EEPROM ID sticks.
"""

from __future__ import annotations

import sys
import time

import serial as pyserial

from _common import JOINT_NAMES, find_port

ADDR_ID = 5
ADDR_LOCK = 55
ADDR_POS = 56

CANDIDATE_IDS = list(range(1, 7))


def _cs(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _pkt(sid: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([_cs(body)])


def _read_pos_pkt(sid: int) -> bytes:
    return _pkt(sid, 0x02, bytes([ADDR_POS, 2]))


REBOOT = _pkt(0xFE, 0x08)


def _decode_pos(resp: bytes, want_id: int):
    """('C', pos) one clean packet from want_id; ('x', None) collision/garbage;
    ('-', None) silence."""
    if not resp:
        return "-", None
    i = 0
    while i < len(resp) - 5:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] != 0xFF:
            sid = resp[i + 2]
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == 4 and end <= len(resp):
                pkt = resp[i:end]
                if sid == want_id and ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1]:
                    return "C", pkt[5] | (pkt[6] << 8)
        i += 1
    return "x", None


def probe_id(ser, sid: int, n: int = 10) -> str:
    """Classify what lives at `sid`: SINGLE, DOUBLE, or EMPTY."""
    pkt = _read_pos_pkt(sid)
    clean = collide = silent = 0
    for _ in range(n):
        ser.reset_input_buffer()
        ser.write(pkt)
        time.sleep(0.004)
        cls, _ = _decode_pos(ser.read(16), sid)
        if cls == "C":
            clean += 1
        elif cls == "x":
            collide += 1
        else:
            silent += 1
        time.sleep(0.003)
    if collide > 0:
        return "DOUBLE"  # only two responders corrupt a position read
    if clean > 0:
        return "SINGLE"
    return "EMPTY"


def scan_state(ser) -> dict[int, str]:
    state = {}
    for sid in CANDIDATE_IDS:
        st = probe_id(ser, sid, n=8)
        if st != "EMPTY":
            state[sid] = st
    return state


def race_peel(ser, doubled_id: int, new_id: int, fire_at_ms: float) -> None:
    """Reboot, then fire a single tight unlock+set-id burst BLIND at a
    precisely calibrated instant -- when servo A has woken (~797ms) but
    servo B has not yet. The boot time is consistent to <0.5ms, and the
    one-up gap is only ~1.5-3ms, far too short to detect-then-fire. So we
    skip detection, busy-wait to fire_at_ms, and drop the burst in one
    write() (both packets land together, no straddle).

    unlock+set-id as ONE burst: if it lands in the window only servo A
    gets it -> IDs split. If it lands late (both up) both change together
    -> harmless ping-pong, caught by verify and retried at a new offset.
    """
    # One contiguous buffer: unlock (key) immediately followed by set-id.
    burst = (_pkt(doubled_id, 0x03, bytes([ADDR_LOCK, 0]))
             + _pkt(doubled_id, 0x03, bytes([ADDR_ID, new_id])))

    ser.reset_input_buffer()
    ser.write(REBOOT)
    t0 = time.perf_counter()

    target = fire_at_ms / 1000.0
    time.sleep(max(0.0, target - 0.010))      # coarse sleep to ~10ms before
    while time.perf_counter() - t0 < target:  # busy-wait for sub-ms precision
        pass
    ser.write(burst)                          # single tight burst, fired blind
    time.sleep(0.06)                          # let the EEPROM write commit


def main() -> None:
    port = find_port()
    if port is None:
        print("ERROR: No adapter found.")
        sys.exit(1)
    ser = pyserial.Serial(port, 1_000_000, timeout=0.006)
    time.sleep(0.05)
    print(f"Port: {port}  baud 1000000\n")

    print("Initial bus state:")
    state = scan_state(ser)
    for sid, st in sorted(state.items()):
        print(f"  ID {sid}: {st}")
    print()

    # Dither the blind fire time across this range (ms after reboot). The
    # fastest servo wakes ~797ms; later peels (slower units, or a pair left
    # on a shared ID) wake a few ms later, so the sweep runs ~795-802ms to
    # land the burst inside whichever unit's one-up window we're peeling.
    FIRE_SWEEP = [796.5, 797.5, 795.5, 798.5, 796.0, 799.5, 797.0,
                  800.5, 798.0, 801.5, 795.0, 799.0, 800.0, 802.0]

    MAX_CYCLES = 120
    for cycle in range(1, MAX_CYCLES + 1):
        doubles = [s for s, st in state.items() if st == "DOUBLE"]
        singles = [s for s, st in state.items() if st == "SINGLE"]

        if not doubles and len(singles) >= 2:
            print(f"SUCCESS after {cycle - 1} cycle(s): servos split across IDs "
                  f"{sorted(singles)}.")
            break

        if not doubles:
            print("No doubled ID found; rescanning...")
            state = scan_state(ser)
            continue

        doubled_id = doubles[0]
        occupied = set(state)
        new_id = next(i for i in range(2, 30) if i not in occupied)
        fire_at = FIRE_SWEEP[(cycle - 1) % len(FIRE_SWEEP)]

        print(f"[cycle {cycle}] two at ID {doubled_id} -> peel one onto ID {new_id}, "
              f"blind fire @ {fire_at}ms ...", end="", flush=True)
        race_peel(ser, doubled_id, new_id, fire_at)

        time.sleep(0.15)
        state = scan_state(ser)
        summary = ", ".join(f"ID{sid}:{st}" for sid, st in sorted(state.items()))
        print(f" -> {summary}")

        if not [s for s, st in state.items() if st == "DOUBLE"] and \
           len([s for s, st in state.items() if st == "SINGLE"]) >= 2:
            print(f"SUCCESS after {cycle} cycle(s): servos split across IDs "
                  f"{sorted(s for s, st in state.items() if st == 'SINGLE')}.")
            break
        time.sleep(0.2)
    else:
        print("Did not converge within cycle budget; rerun to continue.")

    print("\nFinal bus state:")
    final = scan_state(ser)
    for sid, st in sorted(final.items()):
        tag = f" ({JOINT_NAMES.get(sid)})" if sid in JOINT_NAMES else ""
        print(f"  ID {sid}: {st}{tag}")
    ser.close()

    singles = [s for s, st in final.items() if st == "SINGLE"]
    ok = len([s for s, st in final.items() if st == "DOUBLE"]) == 0 and len(singles) >= 2
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
