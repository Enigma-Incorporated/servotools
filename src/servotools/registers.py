"""STS3215 register map. See docs/protocol.md."""

from __future__ import annotations

from typing import NamedTuple

MODEL = 3
ID = 5
BAUD_RATE = 6
RETURN_DELAY = 7
RESPONSE_LEVEL = 8
MIN_ANGLE = 9
MAX_ANGLE = 11
MAX_TORQUE = 16
P_COEFF = 21
D_COEFF = 22
I_COEFF = 23
MODE = 33
ACCELERATION = 37
TORQUE_ENABLE = 40
GOAL_POSITION = 42
TORQUE_LIMIT = 48
LOCK = 55
PRESENT_POSITION = 56
PRESENT_SPEED = 58
PRESENT_LOAD = 60
VOLTAGE = 62
TEMPERATURE = 63

WIDTH = {
    MODEL: 2, ID: 1, BAUD_RATE: 1, RETURN_DELAY: 1, RESPONSE_LEVEL: 1,
    MIN_ANGLE: 2, MAX_ANGLE: 2, MAX_TORQUE: 2, P_COEFF: 1, D_COEFF: 1,
    I_COEFF: 1, MODE: 1, ACCELERATION: 1, TORQUE_ENABLE: 1, GOAL_POSITION: 2,
    TORQUE_LIMIT: 2, LOCK: 1, PRESENT_POSITION: 2, PRESENT_SPEED: 2,
    PRESENT_LOAD: 2, VOLTAGE: 1, TEMPERATURE: 1,
}

EEPROM_END = 40  # writes below here need the lock register cleared first


class Reg(NamedTuple):
    addr: int
    size: int
    name: str


# Ordered dump used by `servo scan`.
DUMP = [
    Reg(PRESENT_POSITION, 2, "Present_Position"),
    Reg(TEMPERATURE, 1, "Temperature"),
    Reg(MODE, 1, "Mode"),
    Reg(TORQUE_ENABLE, 1, "Torque_Enable"),
    Reg(ACCELERATION, 1, "Acceleration"),
    Reg(P_COEFF, 1, "P_Coefficient"),
    Reg(D_COEFF, 1, "D_Coefficient"),
    Reg(I_COEFF, 1, "I_Coefficient"),
    Reg(VOLTAGE, 1, "Voltage"),
    Reg(PRESENT_LOAD, 2, "Present_Load"),
    Reg(PRESENT_SPEED, 2, "Present_Speed"),
    Reg(MAX_TORQUE, 2, "Max_Torque"),
    Reg(TORQUE_LIMIT, 2, "Torque_Limit"),
    Reg(ID, 1, "ID"),
    Reg(BAUD_RATE, 1, "Baud_Rate_Reg"),
    Reg(RESPONSE_LEVEL, 1, "Response_Level"),
]
