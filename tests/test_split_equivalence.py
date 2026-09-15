"""The split and renumber paths must behave exactly as chain_autonumber did.

This is the code that cannot be checked on hardware without an arm, so the bar here is
byte identity: same packets, same order, same resulting EEPROM.
"""

from __future__ import annotations

import io
from contextlib import redirect_stdout

import pytest
import scenarios
from fakebus import FakeBus, FakeSerial

from servotools import split as new_split
from servotools.bus import Bus
from servotools.renumber import TEMP_BASE, renumber, set_id


@pytest.fixture
def legacy():
    import chain_autonumber
    return chain_autonumber


def _pair(clock, scenario, seed=7):
    return (FakeBus(scenarios.ALL[scenario](), clock, seed=seed),
            FakeBus(scenarios.ALL[scenario](), clock, seed=seed))


def _serial(bus, timeout=0.006):
    FakeSerial.bus = bus
    return FakeSerial("/dev/fake", 1_000_000, timeout=timeout)


def _run(fn, bus):
    out = io.StringIO()
    with redirect_stdout(out):
        result = fn(bus)
    return result, out.getvalue()


def _compare(clock, scenario, old_call, new_call, seed=7):
    old_bus, new_bus = _pair(clock, scenario, seed)
    old_result, old_out = _run(old_call, _serial(old_bus))
    new_result, new_out = _run(new_call, Bus(_serial(new_bus)))
    FakeSerial.bus = None
    assert old_bus.oplog == new_bus.oplog, "wire traffic diverged"
    assert old_out == new_out, "printed output diverged"
    assert old_result == new_result, "result diverged"
    assert old_bus.state() == new_bus.state(), "servo state diverged"
    return old_result


@pytest.mark.parametrize("scenario", ["collided", "collided_degenerate", "scattered", "single"])
def test_split_all(clock, legacy, scenario):
    ids, leftover = _compare(
        clock, scenario,
        lambda s: legacy.split_all(s),
        lambda b: new_split.split_all(b),
    )
    if scenario.startswith("collided"):
        assert len(ids) == 6 and not leftover, "the sweep must fully resolve the collision"


@pytest.mark.parametrize("scenario", ["scattered", "healthy"])
def test_renumber(clock, legacy, scenario):
    mapping = ({88: 1, 61: 2, 45: 3, 23: 4, 7: 5, 1: 6} if scenario == "scattered"
               else {1: 6, 2: 5, 3: 4, 4: 3, 5: 2, 6: 1})
    _compare(clock, scenario,
             lambda s: legacy.renumber(s, dict(mapping)),
             lambda b: renumber(b, dict(mapping)))


@pytest.mark.parametrize("scenario", ["healthy", "scattered"])
def test_set_id(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.set_id(s, 1, 9),
             lambda b: set_id(b, 1, 9))


def test_calibrate_wake_edge(clock, legacy):
    old_bus, new_bus = _pair(clock, "collided")
    old = legacy.calibrate_wake_edge(_serial(old_bus), 1)
    new = new_split.calibrate_wake_edge(Bus(_serial(new_bus)), 1)
    FakeSerial.bus = None
    assert old_bus.oplog == new_bus.oplog
    assert old == new
    assert 780.0 < new < 820.0, "the wake edge should land near the documented ~797ms"


def test_sweep_blob_is_byte_identical(legacy):
    """The blob the UART paces out is the whole mechanism; it must not shift by a byte."""
    new_ids = list(range(2, 20))
    expected = b"".join(
        legacy._pkt(1, 0x03, bytes([legacy.ADDR_LOCK, 0]))
        + legacy._pkt(1, 0x03, bytes([legacy.ADDR_ID, nid]))
        for nid in new_ids
    )
    got = b"".join(new_split._set_id_burst(1, nid) for nid in new_ids)
    assert got == expected
    assert len(got) == len(new_ids) * 16, "16 bytes per burst is what paces the sweep"


def test_renumber_never_collides():
    """Temp IDs must keep every intermediate state free of duplicates, for any order."""
    from itertools import permutations
    for perm in permutations(range(1, 6)):
        mapping = {old: new for old, new in zip(range(1, 6), perm, strict=True)}
        live = set(mapping)
        for cur, fin in mapping.items():
            live.discard(cur)
            assert TEMP_BASE + fin not in live
            live.add(TEMP_BASE + fin)
        for fin in mapping.values():
            live.discard(TEMP_BASE + fin)
            assert fin not in live
            live.add(fin)
        assert live == set(perm)


def test_ids_survive_a_reboot(clock, bus_factory):
    """A set-ID that was not re-locked reverts on the next reboot. It must not."""
    import time
    bus_factory("scattered")
    b = Bus(FakeSerial("/dev/fake", 1_000_000, timeout=0.006))
    assert set_id(b, 7, 12)
    b.reboot()
    time.sleep(1.2)
    assert b.ping(12), "the new ID did not survive the reboot: EEPROM was not committed"
    assert not b.ping(7)
