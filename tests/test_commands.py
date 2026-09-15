"""Pure-function and CLI-surface tests. No bus needed for most of these."""

from __future__ import annotations

import pytest

from servotools import registers as reg
from servotools.bus import Bus, checksum, find_frame, packet
from servotools.cli import build_parser
from servotools.commands.autonumber import circ_delta
from servotools.commands.watch import chain_status, poll_chain
from servotools.config import POSITION_MAX


class FakeReader:
    """Stands in for a Bus: ID 2 never answers."""

    def read(self, sid, addr, length=None):
        return None if sid == 2 else 100 + sid


def test_poll_chain_keeps_last_known_position():
    positions = {2: 202}
    states = poll_chain(FakeReader(), tuple(range(1, 7)), positions)
    assert states == {1: True, 2: False, 3: True, 4: True, 5: True, 6: True}
    assert positions[1] == 101
    assert positions[2] == 202


def test_chain_status_names_the_missing_joint():
    positions = {2: 202}
    states = {1: True, 2: False, 3: True, 4: True, 5: True, 6: True}
    assert "ID2 shoulder_lift" in chain_status(states, positions)


def test_chain_status_reports_the_downstream_suffix():
    """A one-poll miss on a low ID must not mask the real break further down."""
    got = chain_status({1: False, 2: True, 3: True, 4: False, 5: False, 6: False}, {})
    assert "ID4 wrist_flex" in got


def test_chain_status_all_and_none():
    assert chain_status({1: True, 2: True}, {}) == "ALL SERVOS CONNECTED"
    assert chain_status({1: False, 2: False}, {}) == "ALL SERVOS DISCONNECTED"


@pytest.mark.parametrize("a,b,expected", [
    (0, 0, 0), (100, 180, 80), (0, 4095, 1), (4095, 0, 1),
    (0, 2048, 2048), (0, 2049, 2047), (2048, 0, 2048),
])
def test_circ_delta_wraps(a, b, expected):
    assert circ_delta(a, b) == expected
    assert circ_delta(b, a) == expected


def test_circ_delta_never_exceeds_half_a_turn():
    for a in range(0, POSITION_MAX, 97):
        for b in range(0, POSITION_MAX, 101):
            assert 0 <= circ_delta(a, b) <= POSITION_MAX // 2


def test_checksum_and_packet_round_trip():
    pkt = packet(3, 0x02, bytes([reg.PRESENT_POSITION, 2]))
    assert pkt[:2] == b"\xff\xff"
    assert pkt[2] == 3
    assert pkt[3] == 4  # params + 2
    assert checksum(pkt[2:-1]) == pkt[-1]


def test_find_frame_skips_leading_garbage():
    good = b"\xff\xff\x01\x04\x00\x00\x08\xf2"
    assert find_frame(b"\x11\x22" + good, 1, 2) is not None
    assert find_frame(good, 1, 2) == b"\x00\x08"


def test_find_frame_rejects_a_bad_checksum():
    bad = bytearray(b"\xff\xff\x01\x04\x00\x00\x08\xf2")
    bad[-1] ^= 0xFF
    assert find_frame(bytes(bad), 1, 2) is None


def test_find_frame_rejects_the_wrong_id_or_length():
    good = b"\xff\xff\x01\x04\x00\x00\x08\xf2"
    assert find_frame(good, 2, 2) is None
    assert find_frame(good, 1, 1) is None


def test_find_frame_rejects_a_truncated_packet():
    good = b"\xff\xff\x01\x04\x00\x00\x08\xf2"
    for cut in range(1, len(good)):
        assert find_frame(good[:-cut], 1, 2) is None


def test_find_frame_error_byte_is_only_checked_when_asked():
    """position_class tolerates a flagged error; read() must not."""
    body = bytes([1, 4, 0x20, 0x00, 0x08])
    pkt = b"\xff\xff" + body + bytes([checksum(body)])
    assert find_frame(pkt, 1, 2) is not None
    assert find_frame(pkt, 1, 2, check_error=True) is None


def test_read_decodes_little_endian(bus_factory):
    from fakebus import FakeSerial
    bus_factory("healthy")
    bus = Bus(FakeSerial("/dev/fake", 1_000_000, timeout=0.006))
    assert bus.read(2, reg.PRESENT_POSITION, 2) == 1500  # scenario position
    assert bus.read(2, reg.TEMPERATURE, 1) == 32


# Every documented invocation must still parse and reach the right command.
INVOCATIONS = [
    (["health"], "health"),
    (["health", "--port", "/dev/x"], "health"),
    (["scan"], "scan"),
    (["scan", "--quick"], "scan"),
    (["scan", "--baud", "500000"], "scan"),
    (["watch"], "watch"),
    (["watch", "--joints", "4", "--interval", "0.1"], "watch"),
    (["torque"], "torque"),
    (["torque", "off"], "torque"),
    (["torque", "on"], "torque"),
    (["defaults"], "defaults"),
    (["set-id", "2"], "set-id"),
    (["set-id", "2", "--baud", "115200"], "set-id"),
    (["autonumber"], "autonumber"),
    (["autonumber", "--scan-only"], "autonumber"),
    (["autonumber", "--detect-only"], "autonumber"),
    (["autonumber", "--apply", "3,1,2", "--yes"], "autonumber"),
    (["autonumber", "--base", "1", "--max-id", "20"], "autonumber"),
    (["flatten"], "flatten"),
    (["flatten", "--scan-only"], "flatten"),
    (["flatten", "--yes", "--id", "1"], "flatten"),
    (["wiggle", "3"], "wiggle"),
    (["wiggle", "1", "2", "3"], "wiggle"),
]


@pytest.mark.parametrize("argv,command", INVOCATIONS)
def test_cli_invocations_parse(argv, command):
    args = build_parser().parse_args(argv)
    assert args.command == command
    assert callable(args.run)
    assert hasattr(args, "port")


def test_torque_defaults_to_off():
    assert build_parser().parse_args(["torque"]).state == "off"


def test_unknown_command_is_rejected():
    with pytest.raises(SystemExit):
        build_parser().parse_args(["nope"])


def test_no_command_prints_help(capsys):
    from servotools.cli import main
    assert main([]) == 2
    assert "usage: servo" in capsys.readouterr().out


def test_version(capsys):
    from servotools import __version__
    from servotools.cli import main
    assert main(["--version"]) == 0
    assert capsys.readouterr().out.strip() == __version__


def test_port_discovery(monkeypatch):
    from serial.tools import list_ports

    from servotools import ports

    class P:
        def __init__(self, dev, vid, pid):
            self.device, self.vid, self.pid = dev, vid, pid

    good = P("/dev/a", ports.ADAPTER_VID, ports.ADAPTER_PID)
    other = P("/dev/b", 0x1234, 0x5678)

    monkeypatch.setattr(list_ports, "comports", lambda: [])
    assert ports.find_port() is None
    assert ports.find_ports() == []

    monkeypatch.setattr(list_ports, "comports", lambda: [other, good])
    assert ports.find_port() == "/dev/a"

    second = P("/dev/c", ports.ADAPTER_VID, ports.ADAPTER_PID)
    monkeypatch.setattr(list_ports, "comports", lambda: [good, second])
    assert ports.find_ports() == ["/dev/a", "/dev/c"]
    assert ports.find_port() == "/dev/a"


def test_confirm_auto_accepts_with_yes(capsys):
    from servotools.cli import confirm
    assert confirm(True, "Press Enter:") is True
    assert "[auto]" in capsys.readouterr().out


def test_confirm_declines_on_n(monkeypatch):
    from servotools.cli import confirm
    monkeypatch.setattr("builtins.input", lambda _="": "n")
    assert confirm(False, "Press Enter:") is False
    monkeypatch.setattr("builtins.input", lambda _="": "")
    assert confirm(False, "Press Enter:") is True


def test_confirm_refuses_without_a_terminal(monkeypatch, capsys):
    """Piped stdin must not crash, and must not be read as consent."""
    from servotools.cli import confirm

    def boom(_=""):
        raise EOFError
    monkeypatch.setattr("builtins.input", boom)
    assert confirm(False, "Press Enter:") is False
    assert "--yes" in capsys.readouterr().out
