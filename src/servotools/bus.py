"""Feetech half-duplex bus protocol. See docs/protocol.md."""

from __future__ import annotations

import time

import serial as pyserial

from . import registers as reg
from .config import (
    BAUD,
    BROADCAST_ID,
    PORT_TIMEOUT,
    WAIT_BETWEEN_PROBES,
    WAIT_PING,
    WAIT_POSITION,
    WAIT_READ,
    WAIT_WRITE,
)

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REBOOT = 0x08  # undocumented; see docs/protocol.md

PROBE_N = 14  # position reads before a SINGLE verdict is trusted

SINGLE = "SINGLE"
DOUBLE = "DOUBLE"
EMPTY = "EMPTY"


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def packet(sid: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


REBOOT = packet(BROADCAST_ID, INST_REBOOT)


def find_frame(resp: bytes, sid: int, length: int, check_error: bool = False) -> bytes | None:
    """Return the params of the first checksum-valid status packet from `sid`, else None."""
    i = 0
    while i < len(resp) - 3:
        if resp[i] == 0xFF and resp[i + 1] == 0xFF and resp[i + 2] == sid:
            ln = resp[i + 3]
            end = i + 4 + ln
            if ln == length + 2 and end <= len(resp):
                pkt = resp[i:end]
                if checksum(pkt[2:-1]) == pkt[-1] and not (check_error and pkt[4] != 0):
                    return pkt[5:-1]
        i += 1
    return None


class Bus:
    """A serial port carrying Feetech servos."""

    def __init__(self, ser) -> None:
        self.ser = ser

    @classmethod
    def open(cls, port: str, baud: int = BAUD, timeout: float = PORT_TIMEOUT) -> Bus:
        ser = pyserial.Serial(port, baud, timeout=timeout)
        time.sleep(0.05)
        return cls(ser)

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> Bus:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def baudrate(self) -> int:
        return self.ser.baudrate

    @baudrate.setter
    def baudrate(self, value: int) -> None:
        self.ser.baudrate = value

    @property
    def timeout(self) -> float:
        return self.ser.timeout

    @timeout.setter
    def timeout(self, value: float) -> None:
        self.ser.timeout = value

    def send(self, data: bytes) -> None:
        """Write pre-built bytes as-is. The ID sweep relies on the UART pacing them."""
        self.ser.write(data)

    def flush(self) -> None:
        self.ser.flush()

    def discard_input(self) -> None:
        self.ser.reset_input_buffer()

    def exchange(self, pkt: bytes, wait: float, nbytes: int) -> bytes:
        """Send one packet and collect the reply window. The port timeout bounds the read."""
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        if wait:
            time.sleep(wait)
        return self.ser.read(nbytes)

    def read(self, sid: int, addr: int, length: int | None = None) -> int | None:
        """Read a register as a little-endian int, or None if no clean reply."""
        length = reg.WIDTH.get(addr, 1) if length is None else length
        resp = self.exchange(packet(sid, INST_READ, bytes([addr, length])), WAIT_READ, 16)
        params = find_frame(resp, sid, length, check_error=True)
        if params is None:
            return None
        return sum(params[k] << (8 * k) for k in range(length))

    def write(self, sid: int, addr: int, data: bytes, settle: float = WAIT_WRITE) -> None:
        """Fire a write. Any status reply is dropped by the next reset_input_buffer."""
        self.ser.reset_input_buffer()
        self.ser.write(packet(sid, INST_WRITE, bytes([addr]) + data))
        if settle:
            time.sleep(settle)

    def write_byte(self, sid: int, addr: int, val: int, settle: float = WAIT_WRITE) -> None:
        self.write(sid, addr, bytes([val & 0xFF]), settle)

    def write_word(self, sid: int, addr: int, val: int, settle: float = 0.0) -> None:
        self.write(sid, addr, bytes([val & 0xFF, (val >> 8) & 0xFF]), settle)

    def ping(self, sid: int) -> bool:
        resp = self.exchange(packet(sid, INST_PING), WAIT_PING, 12)
        return find_frame(resp, sid, 0) is not None

    def reboot(self) -> None:
        """Broadcast reboot. Servos go limp and wake ~797ms later, microseconds apart."""
        self.ser.reset_input_buffer()
        self.ser.write(REBOOT)

    def position_class(self, sid: int) -> str:
        """'C' one clean position reply, 'x' collision, '-' silence."""
        resp = self.exchange(
            packet(sid, INST_READ, bytes([reg.PRESENT_POSITION, 2])), WAIT_POSITION, 16
        )
        if not resp:
            return "-"
        return "C" if find_frame(resp, sid, 2) is not None else "x"

    def probe(self, sid: int, n: int = PROBE_N) -> str:
        """Classify an ID as SINGLE, DOUBLE or EMPTY. See docs/protocol.md."""
        clean = collide = 0
        for _ in range(n):
            c = self.position_class(sid)
            if c == "C":
                clean += 1
            elif c == "x":
                collide += 1
            time.sleep(WAIT_BETWEEN_PROBES)
        if collide > 0:
            return DOUBLE
        if clean > 0:
            return SINGLE
        return EMPTY

    def scan(self, ids, n: int = PROBE_N) -> dict[int, str]:
        """Map every occupied ID to SINGLE or DOUBLE."""
        return {sid: st for sid in ids if (st := self.probe(sid, n)) != EMPTY}
