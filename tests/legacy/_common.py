"""Shared constants and helpers for STS3215 servo tools."""

from __future__ import annotations

import os

from serial.tools import list_ports

_SO101_VID = 0x1A86
_SO101_PID = 0x55D3

EXPECTED_ACCELERATION = 50
EXPECTED_P_COEFF = 16
EXPECTED_D_COEFF = 32

JOINT_NAMES: dict[int, str] = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}

STANDARD_BAUDS = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]


def find_port() -> str | None:
    """Auto-detect the Waveshare bus servo adapter by USB VID/PID."""
    for p in list_ports.comports():
        if p.vid == _SO101_VID and p.pid == _SO101_PID:
            return p.device
    return None


def find_all_ports() -> list[str]:
    """Return all connected Waveshare adapter ports."""
    return [
        p.device
        for p in list_ports.comports()
        if p.vid == _SO101_VID and p.pid == _SO101_PID
    ]


def enable_high_res_timers() -> None:
    """Windows: raise the system timer resolution to 1 ms so time.sleep() and
    the reboot-race blind-fire timing (~797 ms) are accurate. No-op on POSIX.

    Windows' default timer is ~15.6 ms, which would make the split race fire
    late every cycle. Python 3.11+ already uses high-resolution sleeps on
    Windows, but this also covers older interpreters and is harmless. Safe to
    call once at startup."""
    if os.name != "nt":
        return
    try:
        import atexit
        import ctypes

        winmm = ctypes.WinDLL("winmm")
        winmm.timeBeginPeriod(1)
        atexit.register(winmm.timeEndPeriod, 1)
    except Exception:
        pass
