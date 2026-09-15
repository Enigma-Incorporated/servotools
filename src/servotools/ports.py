"""Serial port discovery."""

from __future__ import annotations

from serial.tools import list_ports

from .config import ADAPTER_PID, ADAPTER_VID

NOT_FOUND = "ERROR: no servo bus adapter found. Is the USB cable connected?"


def find_ports() -> list[str]:
    """Every connected Waveshare bus servo adapter, by USB VID/PID."""
    return [p.device for p in list_ports.comports()
            if p.vid == ADAPTER_VID and p.pid == ADAPTER_PID]


def find_port() -> str | None:
    ports = find_ports()
    return ports[0] if ports else None
