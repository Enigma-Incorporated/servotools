"""Bus, arm and timing constants. See docs/protocol.md for where the timings come from."""

from __future__ import annotations

BAUD = 1_000_000
STANDARD_BAUDS = [1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400]

BROADCAST_ID = 0xFE
MAX_ID = 253

ADAPTER_VID = 0x1A86
ADAPTER_PID = 0x55D3

JOINT_NAMES: dict[int, str] = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
ARM_IDS = tuple(range(1, 7))

EXPECTED_ACCELERATION = 50
EXPECTED_P_COEFF = 16
EXPECTED_D_COEFF = 32
EXPECTED_MODE = 0

TEMP_WARNING_C = 45
TEMP_CRITICAL_C = 60

POSITION_MAX = 4096

# Port timeouts, per call site. These are not interchangeable: each was tuned against
# hardware and collapsing them to one value changes behavior.
PORT_TIMEOUT = 0.006  # normal register traffic
PORT_TIMEOUT_FIRE_AND_FORGET = 0.1  # torque commands, which expect no reply
PORT_TIMEOUT_WAKE_POLL = 0.001  # polling for the reboot wake edge

# Per-operation settle times.
WAIT_READ = 0.001
WAIT_PING = 0.002
WAIT_POSITION = 0.004
WAIT_BETWEEN_PROBES = 0.003
WAIT_WRITE = 0.01
WAIT_EEPROM = 0.05
