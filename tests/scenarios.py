"""Named bus scenarios shared by the golden capture and the test suite."""

from __future__ import annotations

from fakebus import Servo

POSITIONS = [2048, 1500, 2600, 900, 3100, 2048]


def healthy():
    return [Servo(i, position=POSITIONS[i - 1], temp=30 + i) for i in range(1, 7)]


def missing():
    return [s for s in healthy() if s.sid != 4]


def hot():
    servos = healthy()
    servos[2].set("temp", 62)
    servos[4].set("temp", 47)
    return servos


def misconfigured():
    servos = healthy()
    servos[1].set("accel", 0)
    servos[1].set("p", 32)
    servos[1].set("d", 0)
    servos[1].set("mode", 1)
    servos[1].set("torque", 1)
    return servos


def collided():
    """All six answering to ID 1, at distinct positions."""
    return [Servo(1, position=p) for p in (2048, 1500, 2600, 900, 3100, 1200)]


def collided_degenerate():
    """Two of the six parked at the same angle: collisions are intermittent."""
    return [Servo(1, position=p) for p in (2048, 2048, 2600, 900, 3100, 1200)]


def scattered():
    """Servos parked at arbitrary IDs, as after an interrupted split."""
    return [Servo(i, position=p) for i, p in
            zip((1, 7, 23, 45, 61, 88), POSITIONS, strict=True)]


def single():
    return [Servo(1, position=2048)]


def empty():
    return []


def other_baud():
    """One servo left at 500 kbps, the rest at 1 Mbps."""
    servos = healthy()
    servos[3].baudrate = 500_000
    return servos


ALL = {
    "healthy": healthy,
    "missing": missing,
    "hot": hot,
    "misconfigured": misconfigured,
    "collided": collided,
    "collided_degenerate": collided_degenerate,
    "scattered": scattered,
    "single": single,
    "empty": empty,
    "other_baud": other_baud,
}
