"""Shim so the frozen experiment scripts keep working against the current package."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from servotools.config import ADAPTER_PID as _SO101_PID  # noqa: E402,F401
from servotools.config import ADAPTER_VID as _SO101_VID  # noqa: E402,F401
from servotools.config import (  # noqa: E402,F401
    EXPECTED_ACCELERATION,
    EXPECTED_D_COEFF,
    EXPECTED_P_COEFF,
    JOINT_NAMES,
    STANDARD_BAUDS,
)
from servotools.ports import find_port, find_ports  # noqa: E402,F401
from servotools.timers import enable_high_res_timers  # noqa: E402,F401

find_all_ports = find_ports
