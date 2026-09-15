"""Virtual clock. See docs/testing.md."""

from __future__ import annotations

import time as _time

TICK = 1e-4  # busy-waits and timeout spins advance by this per clock read


class VirtualClock:
    def __init__(self, start: float = 1_700_000_000.0) -> None:
        self.t = start
        self._real = {
            "time": _time.time,
            "perf_counter": _time.perf_counter,
            "monotonic": _time.monotonic,
            "sleep": _time.sleep,
        }

    def now(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        if seconds > 0:
            self.t += seconds

    def _read(self) -> float:
        self.t += TICK
        return self.t

    def _sleep(self, seconds: float) -> None:
        self.advance(seconds)

    def install(self) -> None:
        _time.time = self._read
        _time.perf_counter = self._read
        _time.monotonic = self._read
        _time.sleep = self._sleep

    def uninstall(self) -> None:
        for name, fn in self._real.items():
            setattr(_time, name, fn)

    def __enter__(self) -> VirtualClock:
        self.install()
        return self

    def __exit__(self, *exc) -> None:
        self.uninstall()
