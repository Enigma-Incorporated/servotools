#!/usr/bin/env python3
"""Auto-number a Feetech daisy chain by PHYSICAL position, over the bus only.

Assumption (this version): every servo starts at ID 1 (factory default),
1 Mbps. Up to 7 servos. Nothing needs unplugging.

Pipeline, built from the primitives we worked out:

  1) SPLIT  -- reboot + ID sweep. Broadcast reboot (instr 0x08); the servos
     wake ~797ms later, a few hundred us apart. We keep the bus silent until
     just before the wake edge (measured per host), then stream ONE contiguous
     blob of unlock+set-id bursts, each assigning a DIFFERENT fresh ID. The
     UART paces the blob at hardware precision (~160us/burst), so each servo
     takes whichever burst it wakes into and stops listening on the old ID --
     several peel per reboot, and the host never needs sub-ms timing (USB
     send jitter on e.g. a Raspberry Pi defeats the old single-burst blind
     race, kept as split_all_race / --legacy-race).

  2) ORDER  -- voltage sag under load. Load all servos (oscillate) and read
     each one's supply voltage; the drop from its own idle baseline is pure
     IR drop to its position (per-unit ADC offset cancels). Dithered means
     over thousands of samples resolve sub-ADC differences. Least drop =
     nearest the power-injection point.

  3) NUMBER -- renumber via temp IDs (collision-free) so position -> ID.

Cross-check: boot wake-time also orders by distance from the power feed; we
measure it and confirm it agrees with the voltage order.

DIRECTION: the only chain end detectable in software is the POWER-injection
point (least sag). Default: nearest-power = ID `--base`. If your 12V enters
at the far end and you want the USB/adapter end to be ID 1, pass --reverse.

Usage:
    python chain_autonumber.py                 # nearest-power = ID 1
    python chain_autonumber.py --reverse       # farthest-power = ID 1
    python chain_autonumber.py --base 1 --no-boot-check
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

import serial as pyserial

from _common import enable_high_res_timers, find_port

BAUD = 1_000_000

# --- registers (STS3215) ---
ADDR_ID = 5
ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_LOCK = 55
ADDR_POS = 56
ADDR_VOLT = 62

# --- split tuning ---
CANDIDATE_IDS = list(range(1, 13))
SKIP_S = 0.70                      # boot silence before the ~797ms wake edge
PROBE_N = 14                       # reads per ID; high enough to catch a near-same-position pair
# Blind-fire time (ms after reboot), busy-waited to sub-ms precision. We home
# in on the one-up window by bisection (too-early -> raise, too-late -> lower),
# then hold near the boundary; the natural ~0.5ms wake jitter then drops the
# (sometimes sub-ms) window onto our fire instant. Blind timing is immune to
# the position degeneracy that defeats collision detection at equal angles.
FIRE_LO, FIRE_HI = 794.0, 799.5    # initial fire-time bracket
MAX_SPLIT_CYCLES = 300
STALL_LIMIT = 60                   # give up if no peel for this many cycles
TEMP_BASE = 100                    # temp ID space for collision-free renumber

# Optional split behaviors (off = historical behavior). A slow host (e.g. a
# Raspberry Pi) both shifts the effective fire time (USB stack latency) and
# preempts the busy-wait, so shots land later than the bisection believes.
REJECT_JITTER_MS = None            # e.g. 0.3: a shot whose lateness deviates >this from the run's
                                   # median doesn't update the bracket (a CONSTANT send latency is
                                   # harmless -- the bisection absorbs it -- but variance around it
                                   # attributes outcomes to fire times that never happened)
CALIBRATE_BRACKET = False          # measure this machine's wake edge; center the bracket on it
FAST_PROBE = False                 # after a race, re-probe only the two affected IDs
FULL_RESCAN_EVERY = 10             # with FAST_PROBE: periodic full-scan safety net

# --- order tuning ---
IDLE_SAMPLES_PER_ID = 150
LOAD_S = 2.0
LOAD_REPEATS = 3
OSC_SPAN = 450
OSC_FLIP_S = 0.16
AMBIGUOUS_MARGIN_V = 0.015         # warn if adjacent positions are this close

# --- boot cross-check tuning ---
BOOT_REBOOTS = 4


# ====================== raw half-duplex bus helpers ======================
def _cs(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def _pkt(sid: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([_cs(body)])


REBOOT = _pkt(0xFE, 0x08)


def read_reg(ser, sid: int, addr: int, length: int):
    """Return register value (little-endian) or None if no clean reply."""
    ser.reset_input_buffer()
    ser.write(_pkt(sid, 0x02, bytes([addr, length])))
    time.sleep(0.001)
    resp = ser.read(16)
    i = 0
    while i < len(resp) - 4:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == length + 2 and end <= len(resp):
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1] and pkt[4] == 0:
                    return sum(pkt[5 + k] << (8 * k) for k in range(length))
        i += 1
    return None


def write_reg(ser, sid: int, addr: int, data: bytes, settle: float = 0.01) -> None:
    """Fire a write. Status reply (if any) is flushed by the next read."""
    ser.reset_input_buffer()
    ser.write(_pkt(sid, 0x03, bytes([addr]) + data))
    if settle:
        time.sleep(settle)


def write_word(ser, sid: int, addr: int, val: int, settle: float = 0.0) -> None:
    write_reg(ser, sid, addr, bytes([val & 0xFF, (val >> 8) & 0xFF]), settle)


def ping(ser, sid: int) -> bool:
    ser.reset_input_buffer()
    ser.write(_pkt(sid, 0x01))
    time.sleep(0.002)
    resp = ser.read(12)
    i = 0
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            pkt = resp[i:i + 6]
            if len(pkt) >= 6 and ((~sum(pkt[2:-1])) & 0xFF) == pkt[5]:
                return True
        i += 1
    return False


def _pos_class(ser, sid: int) -> str:
    """'C' one clean position reply (exactly one servo), 'x' collision, '-' silence."""
    ser.reset_input_buffer()
    ser.write(_pkt(sid, 0x02, bytes([ADDR_POS, 2])))
    time.sleep(0.004)
    resp = ser.read(16)
    if not resp:
        return "-"
    i = 0
    while i < len(resp) - 5:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == 4 and end <= len(resp):
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1]:
                    return "C"
        i += 1
    return "x"


def _clean_pos_in(resp: bytes, sid: int) -> bool:
    """True if resp holds one checksum-valid position reply from sid."""
    i = 0
    while i < len(resp) - 5:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == 4 and end <= len(resp):
                pkt = resp[i:end]
                if ((~sum(pkt[2:-1])) & 0xFF) == pkt[-1]:
                    return True
        i += 1
    return False


def probe_id(ser, sid: int, n: int = PROBE_N) -> str:
    """Classify SINGLE / DOUBLE / EMPTY. A collision is definitive proof of
    >=2 servos; a same-position pair only collides intermittently, so we read
    n times before trusting a SINGLE verdict."""
    clean = collide = 0
    for _ in range(n):
        c = _pos_class(ser, sid)
        if c == "C":
            clean += 1
        elif c == "x":
            collide += 1
        time.sleep(0.003)
    if collide > 0:
        return "DOUBLE"
    if clean > 0:
        return "SINGLE"
    return "EMPTY"


def scan_state(ser, ids=None) -> dict[int, str]:
    return {sid: st for sid in (CANDIDATE_IDS if ids is None else ids)
            if (st := probe_id(ser, sid)) != "EMPTY"}


# ============================ phase 1: split ============================
def race_peel(ser, doubled_id: int, new_id: int, fire_at_ms: float) -> float:
    """Reboot, then drop ONE unlock+set-id burst blind at a precisely
    busy-waited instant. If the burst lands in the one-up window only the
    awake servo (unlocked by the burst's first packet) takes the new ID.
    Returns how many ms LATE the burst was handed to the driver (0 = on
    time); scheduler preemption between the busy-wait and the write shows
    up here, and such shots land at an instant the bisection didn't ask for."""
    burst = (_pkt(doubled_id, 0x03, bytes([ADDR_LOCK, 0]))
             + _pkt(doubled_id, 0x03, bytes([ADDR_ID, new_id])))
    ser.reset_input_buffer()
    ser.write(REBOOT)
    t0 = time.perf_counter()
    target = fire_at_ms / 1000.0
    time.sleep(max(0.0, target - 0.010))
    while time.perf_counter() - t0 < target:   # busy-wait: sub-ms precision
        pass
    ser.write(burst)
    sent_ms = (time.perf_counter() - t0) * 1000.0
    time.sleep(0.06)                           # let the EEPROM ID write commit
    return sent_ms - fire_at_ms


def calibrate_wake_edge(ser, sid: int, repeats: int = 3) -> float | None:
    """Measure when the FIRST servo on `sid` answers after a reboot, on THIS
    machine. The blind-fire window sits just after that edge, but the
    hard-coded FIRE_LO/FIRE_HI bracket bakes in the tuning host's USB-stack
    latency -- on a different host the true window can sit outside it, and
    the bisection can only creep ~0.1ms/cycle, which looks like a hang.
    The poll is a few ms coarse (late-biased), so take the min of a few
    reboots and let the bisection do the sub-ms work inside the bracket."""
    old_timeout = ser.timeout
    ser.timeout = 0.001
    edges = []
    try:
        for _ in range(repeats):
            ser.reset_input_buffer()
            ser.write(REBOOT)
            t0 = time.perf_counter()
            time.sleep(SKIP_S)
            deadline = t0 + 1.5
            while time.perf_counter() < deadline:
                ser.reset_input_buffer()
                ser.write(_pkt(sid, 0x02, bytes([ADDR_POS, 2])))
                resp = ser.read(16)
                if resp and _clean_pos_in(resp, sid):
                    edges.append((time.perf_counter() - t0) * 1000.0)
                    break
            time.sleep(0.25)
    finally:
        ser.timeout = old_timeout
    return min(edges) if edges else None


def split_all_race(ser) -> tuple[list[int], list[int]]:
    """LEGACY blind-fire bisection race. Needs sub-ms host->wire timing, which
    holds on macOS but NOT on Linux/xhci/cdc-acm (measured ~1ms send jitter on
    a Raspberry Pi 5 -- 0/3 successful splits). split_all() below sweeps the
    window with hardware-paced bursts instead and does not need host precision;
    this is kept for reference and for --legacy-race."""
    state = scan_state(ser)
    print("  initial bus: " + (", ".join(f"ID{s}:{state[s]}" for s in sorted(state)) or "(nothing)"))
    cyc = 0
    since_peel = 0
    fire_lo, fire_hi = FIRE_LO, FIRE_HI
    if CALIBRATE_BRACKET:
        doubles = [s for s, v in state.items() if v == "DOUBLE"]
        if doubles:
            edge = calibrate_wake_edge(ser, min(doubles))
            if edge is not None:
                # Poll granularity biases the edge late; open the bracket a
                # few ms below it and a little above.
                fire_lo, fire_hi = edge - 5.0, edge + 1.5
                print(f"  wake edge on this host: ~{edge:.1f}ms -> "
                      f"bracket [{fire_lo:.1f}, {fire_hi:.1f}]ms")
            else:
                print("  wake-edge calibration failed; using default bracket.")
    lo, hi = fire_lo, fire_hi
    lates: list[float] = []
    while cyc < MAX_SPLIT_CYCLES:
        doubles = [s for s, v in state.items() if v == "DOUBLE"]
        if not doubles:
            break
        cyc += 1
        did = min(doubles)
        occupied = set(state)
        new_id = next(i for i in range(2, 90) if i not in occupied)

        # Once the bracket is tight, re-widen and hold near center so the
        # natural wake jitter brings the one-up window onto our fire instant.
        if hi - lo < 0.20:
            c = (lo + hi) / 2.0
            lo, hi = c - 0.6, c + 0.6
        fire = round((lo + hi) / 2.0, 3)

        singles_before = sum(1 for v in state.values() if v == "SINGLE")
        late_ms = race_peel(ser, did, new_id, fire)
        time.sleep(0.12)
        if FAST_PROBE and cyc % FULL_RESCAN_EVERY:
            # The race only touches servos listening on `did`; everything else
            # keeps its ID, so re-probing the two affected IDs is enough.
            for s in (did, new_id):
                st = probe_id(ser, s)
                if st == "EMPTY":
                    state.pop(s, None)
                else:
                    state[s] = st
        else:
            state = scan_state(ser)
        singles_after = sum(1 for v in state.values() if v == "SINGLE")

        # A shot that left the host unusually late (vs this run's typical send
        # latency) didn't test the time the bisection asked for -- using its
        # outcome would poison the bracket.
        lates.append(late_ms)
        mistimed = (REJECT_JITTER_MS is not None
                    and abs(late_ms - statistics.median(lates)) > REJECT_JITTER_MS)

        if singles_after > singles_before:
            outcome = "PEELED"
            lo, hi = fire_lo, fire_hi          # re-home for the next group
            since_peel = 0
        elif mistimed:
            outcome = "jittr"                  # mistimed: outcome not attributable to `fire`
            since_peel += 1
        elif state.get(new_id) == "DOUBLE" and did not in state:
            outcome = "late "                  # whole group moved -> fired too late
            hi = fire
            since_peel += 1
        elif state.get(did) == "DOUBLE":
            outcome = "early"                  # nothing moved -> fired too early
            lo = fire
            since_peel += 1
        else:
            outcome = "shufl"                  # partial / ambiguous; keep bracket
            since_peel += 1

        print(f"  [split {cyc:>3}] ID{did}->ID{new_id} @ {fire:.2f}ms "
              f"(+{late_ms:.2f}) [{outcome}]  ->  "
              + ", ".join(f"ID{s}:{state[s]}" for s in sorted(state)), flush=True)
        if since_peel >= STALL_LIMIT:
            print(f"  no peel in {STALL_LIMIT} cycles -- stopping split.")
            break
    ids = sorted(s for s, v in state.items() if v == "SINGLE")
    leftover_doubles = [s for s, v in state.items() if v == "DOUBLE"]
    return ids, leftover_doubles


# ---- sweep split: hardware-paced, host-jitter-immune (default) ----
WIDE_IDS = list(range(1, 91))      # sweeps park servos anywhere in 2..89
SWEEP_SPAN_IDS = 84                # bursts per sweep; 84 x 16 B = 13.4 ms of window
SWEEP_LEAD_MS = 6.0                # start the sweep this far before the wake edge
MAX_SWEEP_ROUNDS = 12


def sweep_peel(ser, doubled_id: int, new_ids: list[int], start_ms: float) -> None:
    """Fire ONE contiguous blob of unlock+set-id bursts, each assigning the
    next fresh ID. The UART clocks the blob out back-to-back at 1 Mbps
    (~160us per burst), so the whole wake-jitter window is swept at hardware
    pacing: the first servo awake takes the first burst it hears and stops
    listening on the old ID; later wakers take later bursts' different IDs.
    The host only has to hit the blob START to within a few ms -- which even
    a jittery USB stack manages -- and several servos can peel per reboot.
    (The bus must stay SILENT until then: continuous traffic during the boot
    window aborts the bootloader wait and the servos wake early.)"""
    blob = b"".join(
        _pkt(doubled_id, 0x03, bytes([ADDR_LOCK, 0]))
        + _pkt(doubled_id, 0x03, bytes([ADDR_ID, nid]))
        for nid in new_ids
    )
    ser.reset_input_buffer()
    ser.write(REBOOT)
    t0 = time.perf_counter()
    target = start_ms / 1000.0
    time.sleep(max(0.0, target - 0.010))
    while time.perf_counter() - t0 < target:
        pass
    ser.write(blob)
    ser.flush()
    time.sleep(0.12)               # EEPROM commit
    ser.reset_input_buffer()       # drop any TX echo the adapter looped back


def split_all(ser) -> tuple[list[int], list[int]]:
    """Resolve every duplicate-ID collision. Motion-free (reboots make the
    servos briefly limp). Hardware-timed: works on hosts with sloppy USB
    write latency (Raspberry Pi) as well as tight ones (macOS)."""
    state = scan_state(ser, WIDE_IDS)
    print("  initial bus: " + (", ".join(f"ID{s}:{state[s]}" for s in sorted(state)) or "(nothing)"))
    doubles = [s for s, v in state.items() if v == "DOUBLE"]
    if doubles:
        edge = calibrate_wake_edge(ser, min(doubles))
        if edge is None:
            edge = (FIRE_LO + FIRE_HI) / 2.0
            print(f"  wake-edge calibration failed; assuming ~{edge:.1f}ms.")
        start = edge - SWEEP_LEAD_MS
        print(f"  wake edge ~{edge:.1f}ms on this host -> ID sweep from {start:.1f}ms")
        rounds = 0
        while rounds < MAX_SWEEP_ROUNDS:
            doubles = [s for s, v in state.items() if v == "DOUBLE"]
            if not doubles:
                break
            rounds += 1
            did = min(doubles)
            occupied = set(state)
            fresh = [i for i in range(2, 90) if i not in occupied][:SWEEP_SPAN_IDS]
            sweep_peel(ser, did, fresh, start)
            state = scan_state(ser, WIDE_IDS)
            # Sweep-assigned IDs are RAM-volatile (no re-lock, no quiet time to
            # commit mid-stream): the NEXT round's reboot would revert those
            # servos to their EEPROM ID. Re-commit every single now, with the
            # proper unlock -> write -> lock sequence, so progress is permanent.
            for s, v in sorted(state.items()):
                if v == "SINGLE":
                    set_id(ser, s, s)
            print(f"  [sweep {rounds}] ID{did} -> {len(fresh)}-ID sweep  ->  "
                  + ", ".join(f"ID{s}:{state[s]}" for s in sorted(state)), flush=True)
    ids = sorted(s for s, v in state.items() if v == "SINGLE")
    leftover = [s for s, v in state.items() if v == "DOUBLE"]
    return ids, leftover


# ===================== phase 2: order by voltage sag =====================
def measure_order(ser, ids: list[int]):
    idle = {s: [] for s in ids}
    for _ in range(IDLE_SAMPLES_PER_ID):
        for s in ids:
            v = read_reg(ser, s, ADDR_VOLT, 1)
            if v is not None:
                idle[s].append(v)
    idle_mean = {s: statistics.mean(idle[s]) for s in ids}
    home = {s: (read_reg(ser, s, ADDR_POS, 2) or 2048) for s in ids}

    for s in ids:
        write_reg(ser, s, ADDR_TORQUE, bytes([1]), settle=0.02)

    loaded_all = {s: [] for s in ids}
    per_repeat_order = []
    for _ in range(LOAD_REPEATS):
        loaded = {s: [] for s in ids}
        phase = 0
        last = 0.0
        t_end = time.perf_counter() + LOAD_S
        while time.perf_counter() < t_end:
            now = time.perf_counter()
            if now - last > OSC_FLIP_S:
                phase ^= 1
                for s in ids:
                    lo = max(50, home[s] - OSC_SPAN)
                    hi = min(4045, home[s] + OSC_SPAN)
                    write_word(ser, s, ADDR_GOAL, hi if phase else lo)
                last = now
            for s in ids:
                v = read_reg(ser, s, ADDR_VOLT, 1)
                if v is not None:
                    loaded[s].append(v)
                    loaded_all[s].append(v)
        dr = {s: idle_mean[s] - statistics.mean(loaded[s]) for s in ids}
        per_repeat_order.append(tuple(sorted(ids, key=lambda s: dr[s])))

    for s in ids:
        write_word(ser, s, ADDR_GOAL, home[s], settle=0.0)
    time.sleep(0.3)
    for s in ids:
        write_reg(ser, s, ADDR_TORQUE, bytes([0]), settle=0.01)

    drop = {s: (idle_mean[s] - statistics.mean(loaded_all[s])) / 10 for s in ids}
    order = sorted(ids, key=lambda s: drop[s])      # ascending sag = nearest power
    stable = all(o == per_repeat_order[0] for o in per_repeat_order)
    return order, drop, stable


def measure_boot(ser, ids: list[int]):
    samples = {s: [] for s in ids}
    for _ in range(BOOT_REBOOTS):
        for s in ids:
            ser.reset_input_buffer()
            ser.write(REBOOT)
            t0 = time.perf_counter()
            time.sleep(SKIP_S)
            while time.perf_counter() - t0 < 1.2:
                if ping(ser, s):
                    samples[s].append((time.perf_counter() - t0) * 1000)
                    break
            time.sleep(0.18)
    return {s: (statistics.mean(samples[s]) if samples[s] else None) for s in ids}


# ========================= phase 3: renumber =========================
def set_id(ser, old: int, new: int) -> bool:
    write_reg(ser, old, ADDR_TORQUE, bytes([0]), settle=0.03)
    write_reg(ser, old, ADDR_LOCK, bytes([0]), settle=0.03)
    write_reg(ser, old, ADDR_ID, bytes([new]), settle=0.05)
    write_reg(ser, new, ADDR_LOCK, bytes([1]), settle=0.03)
    return ping(ser, new)


def renumber(ser, mapping: dict[int, int]) -> bool:
    ok = True
    for cur, fin in mapping.items():            # everyone -> temp space first
        ok &= set_id(ser, cur, TEMP_BASE + fin)
    for cur, fin in mapping.items():            # temp -> final
        ok &= set_id(ser, TEMP_BASE + fin, fin)
    return ok


def wiggle_in_order(ser, ids: list[int]) -> None:
    """Nudge each servo ~22deg in ID order. If numbering matches the chain,
    you see a clean wave travel end-to-end; a servo moving out of sequence
    means the order is wrong there."""
    print("\nWIGGLE VERIFICATION — watch for a clean sweep along the chain:")
    for sid in ids:
        pos = read_reg(ser, sid, ADDR_POS, 2)
        if pos is None:
            print(f"  ID{sid}: no read, skipping")
            continue
        print(f"  >>> ID{sid} moving", flush=True)
        write_reg(ser, sid, ADDR_TORQUE, bytes([1]), settle=0.03)
        for tgt in (min(4045, pos + 250), max(50, pos - 250), pos):
            write_word(ser, sid, ADDR_GOAL, tgt)
            time.sleep(0.45)
        write_reg(ser, sid, ADDR_TORQUE, bytes([0]), settle=0.02)
        time.sleep(0.7)


# ================================ main ================================
def main() -> None:
    ap = argparse.ArgumentParser(description="Auto-number a daisy chain by physical position")
    ap.add_argument("--port", default=None)
    ap.add_argument("--base", type=int, default=1, help="ID of the first position (default 1)")
    ap.add_argument("--reverse", action="store_true",
                    help="make the FARTHEST-from-power end the base ID")
    ap.add_argument("--no-boot-check", action="store_true")
    ap.add_argument("--legacy-race", action="store_true",
                    help="split with the old blind-fire bisection race (needs sub-ms "
                         "host->wire timing; fails on hosts with USB send jitter, e.g. Raspberry Pi)")
    ap.add_argument("--wiggle", action="store_true",
                    help="after renumber, nudge each servo in ID order to verify visually")
    args = ap.parse_args()

    enable_high_res_timers()   # Windows: sub-ms sleeps for the reboot-race (no-op on POSIX)

    port = args.port or find_port()
    if port is None:
        print("ERROR: no Waveshare adapter found.")
        sys.exit(1)
    ser = pyserial.Serial(port, BAUD, timeout=0.006)
    time.sleep(0.05)
    print(f"Port: {port}  baud {BAUD}\n")

    # ---- phase 1 ----
    print("PHASE 1 — split shared IDs apart (reboot + ID sweep)")
    ids, leftover = (split_all_race if args.legacy_race else split_all)(ser)
    if leftover:
        print(f"\nERROR: could not fully split (still colliding at {leftover}). "
              f"Rerun; the race is probabilistic.")
        ser.close()
        sys.exit(1)
    if not ids:
        print("\nERROR: no servos found at 1 Mbps.")
        ser.close()
        sys.exit(1)
    n = len(ids)
    print(f"  -> {n} servo(s) on distinct IDs: {ids}\n")

    if n == 1:
        print("Single servo; assigning base ID.")
        renumber(ser, {ids[0]: args.base})
        print(f"Done: ID {args.base}.")
        ser.close()
        return

    # ---- phase 2 ----
    print("PHASE 2 — order by voltage sag under load (offset-free)")
    order, drop, stable = measure_order(ser, ids)
    for rank, s in enumerate(order):
        print(f"  pos {rank}: ID{s}  drop={drop[s]:.3f}V")
    print(f"  power-distance order (near->far): "
          + " -> ".join(f"ID{s}" for s in order))
    if not stable:
        print("  WARNING: ranking flipped between repeats -> some positions are close.")
    margins = [abs(drop[order[i]] - drop[order[i + 1]]) for i in range(n - 1)]
    if any(m < AMBIGUOUS_MARGIN_V for m in margins):
        tight = [f"ID{order[i]}/ID{order[i+1]}" for i, m in enumerate(margins)
                 if m < AMBIGUOUS_MARGIN_V]
        print(f"  WARNING: small voltage margin between {tight} "
              f"(< {AMBIGUOUS_MARGIN_V}V) -- order there is low-confidence.")

    # ---- cross-check ----
    if not args.no_boot_check:
        print("\nPHASE 2b — boot-time cross-check (independent spatial signal)")
        boots = measure_boot(ser, ids)
        boot_order = sorted(ids, key=lambda s: (boots[s] is None, boots[s] or 0.0))
        for s in order:
            b = boots[s]
            print(f"  ID{s}: wake {b:.1f}ms" if b is not None else f"  ID{s}: no wake")
        if boot_order == order:
            print("  AGREES with voltage order — high confidence.")
        else:
            print(f"  DISAGREES (boot: {'->'.join(f'ID{s}' for s in boot_order)}). "
                  f"Trusting voltage (direct IR); recheck close positions.")

    # ---- phase 3 ----
    final_order = order[::-1] if args.reverse else order
    mapping = {final_order[i]: args.base + i for i in range(n)}
    print("\nPHASE 3 — renumber to chain position")
    end_label = "farthest-from-power" if args.reverse else "nearest-power"
    for cur, fin in sorted(mapping.items(), key=lambda kv: kv[1]):
        print(f"  ID{cur} -> ID{fin}")
    ok = renumber(ser, mapping)

    # ---- verify ----
    final_ids = list(range(args.base, args.base + n))
    print("\nVerification:")
    good = True
    for fid in final_ids:
        st = probe_id(ser, fid, n=8)
        print(f"  ID{fid}: {st}")
        if st != "SINGLE":
            good = False
    stragglers = [s for s in scan_state(ser) if s not in final_ids]
    if stragglers:
        good = False
        print(f"  WARNING: unexpected IDs still present: {stragglers}")

    if args.wiggle and good:
        wiggle_in_order(ser, final_ids)
    ser.close()

    print()
    if ok and good:
        print(f"DONE. {n} servos numbered {args.base}..{args.base + n - 1} "
              f"along the chain; ID {args.base} = {end_label} end.")
    else:
        print("FINISHED WITH WARNINGS — re-scan; you may want to rerun.")
    sys.exit(0 if (ok and good) else 1)


if __name__ == "__main__":
    main()
