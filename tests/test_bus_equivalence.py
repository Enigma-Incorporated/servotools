"""Bus methods must be indistinguishable from the chain_autonumber helpers they replace.

Each case drives the old function and the new method against identical fake buses and
compares both the returned value and the exact packets put on the wire.
"""

from __future__ import annotations

import pytest
import scenarios
from fakebus import FakeBus, FakeSerial

from servotools.bus import Bus

SCENARIOS = ["healthy", "scattered", "collided", "collided_degenerate", "single", "empty"]


def _pair(clock, scenario, seed=7):
    """Two independent buses seeded identically: one for the old code, one for the new."""
    old = FakeBus(scenarios.ALL[scenario](), clock, seed=seed)
    new = FakeBus(scenarios.ALL[scenario](), clock, seed=seed)
    return old, new


def _serial(bus, timeout=0.006):
    FakeSerial.bus = bus
    return FakeSerial("/dev/fake", 1_000_000, timeout=timeout)


@pytest.fixture
def legacy():
    import chain_autonumber
    return chain_autonumber


def _compare(clock, scenario, old_call, new_call, seed=7):
    old_bus, new_bus = _pair(clock, scenario, seed)
    old_result = old_call(_serial(old_bus))
    new_result = new_call(Bus(_serial(new_bus)))
    FakeSerial.bus = None
    assert old_bus.oplog == new_bus.oplog, "wire traffic diverged"
    assert old_result == new_result, "result diverged"
    assert old_bus.state() == new_bus.state(), "servo state diverged"
    return old_result


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_read_register(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.read_reg(s, 1, 56, 2),
             lambda b: b.read(1, 56, 2))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_read_one_byte(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.read_reg(s, 1, 62, 1),
             lambda b: b.read(1, 62, 1))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_ping(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.ping(s, 1),
             lambda b: b.ping(1))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_position_class(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy._pos_class(s, 1),
             lambda b: b.position_class(1))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_probe(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.probe_id(s, 1),
             lambda b: b.probe(1))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_probe_short(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.probe_id(s, 1, n=8),
             lambda b: b.probe(1, n=8))


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_scan(clock, legacy, scenario):
    ids = list(range(1, 13))
    _compare(clock, scenario,
             lambda s: legacy.scan_state(s, ids),
             lambda b: b.scan(ids))


@pytest.mark.parametrize("scenario", ["healthy", "scattered"])
def test_write_byte(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.write_reg(s, 1, 40, bytes([1]), settle=0.02),
             lambda b: b.write_byte(1, 40, 1, settle=0.02))


@pytest.mark.parametrize("scenario", ["healthy", "scattered"])
def test_write_word(clock, legacy, scenario):
    _compare(clock, scenario,
             lambda s: legacy.write_word(s, 1, 42, 2500),
             lambda b: b.write_word(1, 42, 2500))


def test_reboot_packet_is_byte_identical(legacy):
    from servotools.bus import REBOOT
    assert REBOOT == legacy.REBOOT
    assert REBOOT == b"\xff\xff\xfe\x02\x08\xf7"


def test_packet_builder_matches(legacy):
    from servotools.bus import packet
    for sid, inst, params in [(1, 0x02, b"\x38\x02"), (0xFE, 0x08, b""),
                              (5, 0x03, b"\x05\x07"), (253, 0x01, b"")]:
        assert packet(sid, inst, params) == legacy._pkt(sid, inst, params)


def test_checksum_matches(legacy):
    from servotools.bus import checksum
    for body in [b"", b"\x01\x02\x03", bytes(range(64)), b"\xff" * 8]:
        assert checksum(body) == legacy._cs(body)
