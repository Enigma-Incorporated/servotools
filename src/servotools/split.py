"""Duplicate-ID collision splitting. Timing-critical: see docs/autonumber.md."""

from __future__ import annotations

import statistics
import time

from . import registers as reg
from .bus import DOUBLE, INST_READ, INST_WRITE, SINGLE, Bus, find_frame, packet
from .config import PORT_TIMEOUT_WAKE_POLL
from .renumber import set_id


def _print(msg: str) -> None:
    print(msg, flush=True)


CANDIDATE_IDS = list(range(1, 13))  # default scan range
WIDE_IDS = list(range(1, 91))  # sweeps park servos anywhere in 2..89
FRESH_ID_RANGE = range(2, 90)  # pool the sweep draws replacement IDs from

SKIP_S = 0.70  # boot silence held before the wake edge
FIRE_LO, FIRE_HI = 794.0, 799.5  # legacy blind-fire bracket, ms after reboot
MAX_SPLIT_CYCLES = 300
STALL_LIMIT = 60

SWEEP_SPAN_IDS = 84  # bursts per sweep; 84 x 16 B = 13.4 ms of covered window
SWEEP_LEAD_MS = 6.0  # start the sweep this far before the wake edge
MAX_SWEEP_ROUNDS = 12

REJECT_JITTER_MS = None  # legacy race: ignore shots this far off the median lateness
CALIBRATE_BRACKET = False  # legacy race: centre the bracket on the measured wake edge
FAST_PROBE = False  # legacy race: re-probe only the two affected IDs
FULL_RESCAN_EVERY = 10


def _state_line(state: dict[int, str]) -> str:
    return ", ".join(f"ID{s}:{state[s]}" for s in sorted(state))


def _set_id_burst(doubled_id: int, new_id: int) -> bytes:
    """Unlock then set-ID, as one contiguous pair of packets."""
    return (packet(doubled_id, INST_WRITE, bytes([reg.LOCK, 0]))
            + packet(doubled_id, INST_WRITE, bytes([reg.ID, new_id])))


def calibrate_wake_edge(bus: Bus, sid: int, repeats: int = 3) -> float | None:
    """Measure when the first servo answers after a reboot, on this machine."""
    old_timeout = bus.timeout
    bus.timeout = PORT_TIMEOUT_WAKE_POLL
    poll = packet(sid, INST_READ, bytes([reg.PRESENT_POSITION, 2]))
    edges = []
    try:
        for _ in range(repeats):
            bus.reboot()
            t0 = time.perf_counter()
            time.sleep(SKIP_S)
            deadline = t0 + 1.5
            while time.perf_counter() < deadline:
                resp = bus.exchange(poll, 0.0, 16)
                if resp and find_frame(resp, sid, 2) is not None:
                    edges.append((time.perf_counter() - t0) * 1000.0)
                    break
            time.sleep(0.25)
    finally:
        bus.timeout = old_timeout
    return min(edges) if edges else None


def sweep_peel(bus: Bus, doubled_id: int, new_ids: list[int], start_ms: float) -> None:
    """Fire one contiguous blob of unlock+set-id bursts, each assigning a different ID."""
    blob = b"".join(_set_id_burst(doubled_id, nid) for nid in new_ids)
    bus.reboot()
    t0 = time.perf_counter()
    target = start_ms / 1000.0
    time.sleep(max(0.0, target - 0.010))
    while time.perf_counter() - t0 < target:  # busy-wait for sub-ms precision
        pass
    bus.send(blob)
    bus.flush()
    time.sleep(0.12)  # let the EEPROM commit
    bus.discard_input()  # drop any TX echo the adapter looped back


def split_all(bus: Bus, ids=None, log=_print) -> tuple[list[int], list[int]]:
    """Resolve every duplicate-ID collision. Motion-free; reboots make servos briefly limp."""
    ids = WIDE_IDS if ids is None else ids
    state = bus.scan(ids)
    log("  initial bus: " + (_state_line(state) or "(nothing)"))
    doubles = [s for s, v in state.items() if v == DOUBLE]
    if doubles:
        edge = calibrate_wake_edge(bus, min(doubles))
        if edge is None:
            edge = (FIRE_LO + FIRE_HI) / 2.0
            log(f"  wake-edge calibration failed; assuming ~{edge:.1f}ms.")
        start = edge - SWEEP_LEAD_MS
        log(f"  wake edge ~{edge:.1f}ms on this host -> ID sweep from {start:.1f}ms")
        rounds = 0
        while rounds < MAX_SWEEP_ROUNDS:
            doubles = [s for s, v in state.items() if v == DOUBLE]
            if not doubles:
                break
            rounds += 1
            did = min(doubles)
            occupied = set(state)
            fresh = [i for i in FRESH_ID_RANGE if i not in occupied][:SWEEP_SPAN_IDS]
            sweep_peel(bus, did, fresh, start)
            state = bus.scan(ids)
            # Sweep-assigned IDs are RAM-only; re-commit each single before the next reboot.
            for s, v in sorted(state.items()):
                if v == SINGLE:
                    set_id(bus, s, s)
            log(f"  [sweep {rounds}] ID{did} -> {len(fresh)}-ID sweep  ->  "
                + _state_line(state))
    singles = sorted(s for s, v in state.items() if v == SINGLE)
    leftover = [s for s, v in state.items() if v == DOUBLE]
    return singles, leftover


def race_peel(bus: Bus, doubled_id: int, new_id: int, fire_at_ms: float) -> float:
    """Legacy: drop one burst blind at a busy-waited instant. Returns how many ms late."""
    burst = _set_id_burst(doubled_id, new_id)
    bus.reboot()
    t0 = time.perf_counter()
    target = fire_at_ms / 1000.0
    time.sleep(max(0.0, target - 0.010))
    while time.perf_counter() - t0 < target:
        pass
    bus.send(burst)
    sent_ms = (time.perf_counter() - t0) * 1000.0
    time.sleep(0.06)
    return sent_ms - fire_at_ms


def split_all_race(bus: Bus, ids=None, log=_print) -> tuple[list[int], list[int]]:
    """Legacy blind-fire bisection race. Needs sub-ms host timing; fails on a Pi."""
    ids = CANDIDATE_IDS if ids is None else ids
    state = bus.scan(ids)
    log("  initial bus: " + (_state_line(state) or "(nothing)"))
    cyc = 0
    since_peel = 0
    fire_lo, fire_hi = FIRE_LO, FIRE_HI
    if CALIBRATE_BRACKET:
        doubles = [s for s, v in state.items() if v == DOUBLE]
        if doubles:
            edge = calibrate_wake_edge(bus, min(doubles))
            if edge is not None:
                fire_lo, fire_hi = edge - 5.0, edge + 1.5
                log(f"  wake edge on this host: ~{edge:.1f}ms -> "
                    f"bracket [{fire_lo:.1f}, {fire_hi:.1f}]ms")
            else:
                log("  wake-edge calibration failed; using default bracket.")
    lo, hi = fire_lo, fire_hi
    lates: list[float] = []
    while cyc < MAX_SPLIT_CYCLES:
        doubles = [s for s, v in state.items() if v == DOUBLE]
        if not doubles:
            break
        cyc += 1
        did = min(doubles)
        occupied = set(state)
        new_id = next(i for i in FRESH_ID_RANGE if i not in occupied)

        if hi - lo < 0.20:  # re-widen and hold near centre; wake jitter does the rest
            c = (lo + hi) / 2.0
            lo, hi = c - 0.6, c + 0.6
        fire = round((lo + hi) / 2.0, 3)

        singles_before = sum(1 for v in state.values() if v == SINGLE)
        late_ms = race_peel(bus, did, new_id, fire)
        time.sleep(0.12)
        if FAST_PROBE and cyc % FULL_RESCAN_EVERY:
            for s in (did, new_id):
                st = bus.probe(s)
                if st == "EMPTY":
                    state.pop(s, None)
                else:
                    state[s] = st
        else:
            state = bus.scan(ids)
        singles_after = sum(1 for v in state.values() if v == SINGLE)

        lates.append(late_ms)
        mistimed = (REJECT_JITTER_MS is not None
                    and abs(late_ms - statistics.median(lates)) > REJECT_JITTER_MS)

        if singles_after > singles_before:
            outcome = "PEELED"
            lo, hi = fire_lo, fire_hi
            since_peel = 0
        elif mistimed:
            outcome = "jittr"
            since_peel += 1
        elif state.get(new_id) == DOUBLE and did not in state:
            outcome = "late "
            hi = fire
            since_peel += 1
        elif state.get(did) == DOUBLE:
            outcome = "early"
            lo = fire
            since_peel += 1
        else:
            outcome = "shufl"
            since_peel += 1

        log(f"  [split {cyc:>3}] ID{did}->ID{new_id} @ {fire:.2f}ms "
            f"(+{late_ms:.2f}) [{outcome}]  ->  " + _state_line(state))
        if since_peel >= STALL_LIMIT:
            log(f"  no peel in {STALL_LIMIT} cycles -- stopping split.")
            break
    singles = sorted(s for s, v in state.items() if v == SINGLE)
    leftover = [s for s, v in state.items() if v == DOUBLE]
    return singles, leftover
