"""Windows timer resolution. See docs/protocol.md."""

from __future__ import annotations

import os


def enable_high_res_timers() -> None:
    """Ask Windows for a 1ms timer so the split's sub-ms waits land. No-op on POSIX."""
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
