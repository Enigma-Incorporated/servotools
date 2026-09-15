"""In-memory STS3215 bus simulator. See docs/testing.md for the model."""

from __future__ import annotations

import random

REG_SIZE = 80

ADDR_MODEL = 3
ADDR_ID = 5
ADDR_BAUD = 6
ADDR_RETURN_DELAY = 7
ADDR_RESPONSE_LEVEL = 8
ADDR_MAX_TORQUE = 16
ADDR_P = 21
ADDR_D = 22
ADDR_I = 23
ADDR_MODE = 33
ADDR_ACCEL = 37
ADDR_TORQUE = 40
ADDR_GOAL = 42
ADDR_TORQUE_LIMIT = 48
ADDR_LOCK = 55
ADDR_POS = 56
ADDR_LOAD = 58
ADDR_VOLT = 62
ADDR_TEMP = 63

EEPROM_END = 40  # addresses below this are lock-guarded and survive reboot
BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REBOOT = 0x08

WAKE_MEAN_S = 0.797
WAKE_JITTER_S = 0.0004
BOOT_SILENT_S = 0.05  # servo ignores the bus this long after a reboot command

DEFAULTS = {
    ADDR_MODEL: 0x09,
    ADDR_MODEL + 1: 0x03,
    ADDR_BAUD: 0,
    ADDR_RESPONSE_LEVEL: 1,
    ADDR_P: 16,
    ADDR_D: 32,
    ADDR_I: 0,
    ADDR_MODE: 0,
    ADDR_ACCEL: 50,
    ADDR_TORQUE: 0,
    ADDR_LOCK: 1,
    ADDR_VOLT: 121,
    ADDR_TEMP: 32,
}


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def packet(sid: int, inst: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, inst]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def status_packet(sid: int, error: int = 0, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, error]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


class Servo:
    """One virtual STS3215."""

    def __init__(self, sid: int, position: int = 2048, baudrate: int = 1_000_000,
                 **overrides: int) -> None:
        self.baudrate = baudrate
        self.regs = bytearray(REG_SIZE)
        for addr, val in DEFAULTS.items():
            self.regs[addr] = val
        self.set_word(ADDR_POS, position)
        self.set_word(ADDR_GOAL, position)
        self.set_word(ADDR_MAX_TORQUE, 1000)
        self.set_word(ADDR_TORQUE_LIMIT, 1000)
        self.regs[ADDR_ID] = sid
        for name, val in overrides.items():
            self.set(name, val)
        self.eeprom = bytearray(self.regs[:EEPROM_END])
        self.wake_at: float | None = None  # not None while rebooting

    def set(self, name: str, val: int) -> None:
        addr = globals()["ADDR_" + name.upper()]
        if name.lower() in ("pos", "goal", "load", "max_torque", "torque_limit"):
            self.set_word(addr, val)
        else:
            self.regs[addr] = val & 0xFF

    def set_word(self, addr: int, val: int) -> None:
        self.regs[addr] = val & 0xFF
        self.regs[addr + 1] = (val >> 8) & 0xFF

    def word(self, addr: int) -> int:
        return self.regs[addr] | (self.regs[addr + 1] << 8)

    @property
    def sid(self) -> int:
        return self.regs[ADDR_ID]

    @property
    def locked(self) -> bool:
        return self.regs[ADDR_LOCK] != 0

    def awake_at(self, t: float) -> bool:
        return self.wake_at is None or t >= self.wake_at

    def reboot(self, t: float, rng: random.Random) -> None:
        self.regs[:EEPROM_END] = self.eeprom  # uncommitted EEPROM edits revert
        self.wake_at = t + rng.gauss(WAKE_MEAN_S, WAKE_JITTER_S)

    def write(self, addr: int, data: bytes) -> None:
        for i, byte in enumerate(data):
            a = addr + i
            if a >= REG_SIZE:
                return
            if a < EEPROM_END and self.locked:
                continue  # EEPROM writes are ignored while locked
            self.regs[a] = byte
        if addr <= ADDR_LOCK < addr + len(data) and self.regs[ADDR_LOCK]:
            self.eeprom[:] = self.regs[:EEPROM_END]  # re-lock commits to EEPROM
        if addr <= ADDR_GOAL < addr + len(data) and self.regs[ADDR_TORQUE]:
            self.set_word(ADDR_POS, self.word(ADDR_GOAL))

    def state(self) -> dict[str, int]:
        return {
            "id": self.sid,
            "pos": self.word(ADDR_POS),
            "torque": self.regs[ADDR_TORQUE],
            "mode": self.regs[ADDR_MODE],
            "accel": self.regs[ADDR_ACCEL],
            "p": self.regs[ADDR_P],
            "d": self.regs[ADDR_D],
            "lock": self.regs[ADDR_LOCK],
            "temp": self.regs[ADDR_TEMP],
            "volt": self.regs[ADDR_VOLT],
            "eeprom_id": self.eeprom[ADDR_ID],
        }


class FakeBus:
    """A half-duplex bus carrying several Servos."""

    def __init__(self, servos, clock, seed: int = 0, p_arbitrate: float = 0.15,
                 baudrate: int = 1_000_000) -> None:
        self.servos = list(servos)
        self.clock = clock
        self.rng = random.Random(seed)
        self.p_arbitrate = p_arbitrate
        self.baudrate = baudrate
        self.rx = bytearray()  # bytes waiting for the host
        self._pending: list[bytes] = []  # replies raised by the write being processed
        self.oplog: list[tuple] = []
        self.movements: list[tuple[float, Servo, int]] = []  # (at, servo, delta)
        self.silences: list[tuple[float, int, bool]] = []  # (at, sid, silent)
        self.silenced: set[int] = set()
        self.max_writes: int | None = None  # KeyboardInterrupt past this, to bound watch loops
        self.writes = 0

    def schedule_move(self, at: float, servo: Servo, delta: int) -> None:
        self.movements.append((at, servo, delta))

    def schedule_silence(self, at: float, sid: int, silent: bool = True) -> None:
        self.silences.append((at, sid, silent))

    def _apply_silences(self, t: float) -> None:
        for entry in list(self.silences):
            when, sid, silent = entry
            if t >= when:
                self.silenced.add(sid) if silent else self.silenced.discard(sid)
                self.silences.remove(entry)

    def _apply_movements(self, t: float) -> None:
        for entry in list(self.movements):
            when, servo, delta = entry
            if t >= when:
                servo.set_word(ADDR_POS, (servo.word(ADDR_POS) + delta) % 4096)
                self.movements.remove(entry)

    def byte_time(self) -> float:
        return 10.0 / self.baudrate

    def write(self, data: bytes) -> int:
        """Clock `data` onto the wire; each servo sees only what arrives after it wakes."""
        self.writes += 1
        if self.max_writes is not None and self.writes > self.max_writes:
            raise KeyboardInterrupt
        t0 = self.clock.now()
        bt = self.byte_time()
        self._apply_movements(t0)
        self._apply_silences(t0)
        self._log(data)
        self._pending = []
        for servo in self.servos:
            if servo.baudrate != self.baudrate or servo.sid in self.silenced:
                continue
            if servo.awake_at(t0):
                first = 0
            elif servo.wake_at is not None and servo.wake_at < t0 + len(data) * bt:
                first = int((servo.wake_at - t0) / bt) + 1
            else:
                continue
            servo.wake_at = None
            self._feed(servo, data[first:], t0 + first * bt, bt)
        self._resolve()
        self.clock.advance(len(data) * bt)
        return len(data)

    def _resolve(self) -> None:
        """Collapse everything that answered into what the host actually hears."""
        replies = self._pending
        self._pending = []
        if not replies:
            return
        if len(replies) == 1 or all(r == replies[0] for r in replies):
            self.rx.extend(replies[0])  # identical replies overlay: the same-angle degeneracy
            return
        if self.rng.random() < self.p_arbitrate:
            self.rx.extend(self.rng.choice(replies))  # one servo won the line
            return
        a, b = self.rng.sample(replies, 2)
        self.rx.extend(self._garble(a, b))

    def _log(self, stream: bytes) -> None:
        """Record every well-formed packet the host emitted, for before/after comparison."""
        i = 0
        while i < len(stream) - 3:
            if stream[i] != 0xFF or stream[i + 1] != 0xFF:
                i += 1
                continue
            sid, length = stream[i + 2], stream[i + 3]
            end = i + 4 + length
            if length < 2 or end > len(stream):
                return
            pkt = stream[i:end]
            if checksum(pkt[2:-1]) != pkt[-1]:
                i += 1
                continue
            self.oplog.append((self.baudrate, sid, pkt[4], tuple(pkt[5:-1])))
            i = end

    def _feed(self, servo: Servo, stream: bytes, t_start: float, bt: float) -> None:
        i = 0
        while i < len(stream) - 3:
            if stream[i] != 0xFF or stream[i + 1] != 0xFF:
                i += 1
                continue
            sid, length = stream[i + 2], stream[i + 3]
            end = i + 4 + length
            if length < 2 or end > len(stream):
                return
            pkt = stream[i:end]
            if checksum(pkt[2:-1]) != pkt[-1]:
                i += 1
                continue
            inst, params = pkt[4], pkt[5:-1]
            self._deliver(servo, sid, inst, params, t_start + end * bt)
            i = end

    def _deliver(self, servo: Servo, sid: int, inst: int, params: bytes, t: float) -> None:
        broadcast = sid == BROADCAST_ID
        if not broadcast and sid != servo.sid:
            return
        if inst == INST_REBOOT:
            servo.reboot(t, self.rng)
            return
        if inst == INST_PING:
            if not broadcast:
                self._reply(servo, status_packet(servo.sid))
            return
        if inst == INST_READ:
            if broadcast or len(params) < 2:
                return
            addr, length = params[0], params[1]
            self._reply(servo, status_packet(servo.sid, 0, bytes(servo.regs[addr:addr + length])))
            return
        if inst == INST_WRITE and params:
            servo.write(params[0], params[1:])
            if not broadcast and servo.regs[ADDR_RESPONSE_LEVEL]:
                self._reply(servo, status_packet(servo.sid))

    def _reply(self, servo: Servo, pkt: bytes) -> None:
        self._pending.append(pkt)

    @staticmethod
    def _garble(a: bytes, b: bytes) -> bytes:
        n = max(len(a), len(b))
        a = a.ljust(n, b"\x00")
        b = b.ljust(n, b"\x00")
        return bytes(a[i] if i % 2 == 0 else b[i] for i in range(n))

    def read(self, n: int, timeout: float | None) -> bytes:
        self._apply_movements(self.clock.now())
        if not self.rx and timeout:
            self.clock.advance(timeout)
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def state(self) -> list[dict[str, int]]:
        """Creation order, not ID order: several servos may share an ID."""
        return [s.state() for s in self.servos]


class FakeSerial:
    """serial.Serial stand-in bound to the FakeBus in `FakeSerial.bus`."""

    bus: FakeBus | None = None

    def __init__(self, port=None, baudrate=1_000_000, timeout=None, **kwargs) -> None:
        if FakeSerial.bus is None:
            raise RuntimeError("FakeSerial.bus is not set")
        self._bus = FakeSerial.bus
        self._bus.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.is_open = True

    @property
    def baudrate(self) -> int:
        return self._bus.baudrate

    @baudrate.setter
    def baudrate(self, value: int) -> None:
        self._bus.baudrate = value

    @property
    def in_waiting(self) -> int:
        return len(self._bus.rx)

    def read(self, n: int = 1) -> bytes:
        return self._bus.read(n, self.timeout)

    def write(self, data) -> int:
        return self._bus.write(bytes(data))

    def reset_input_buffer(self) -> None:
        self._bus.reset_input_buffer()

    def reset_output_buffer(self) -> None:
        pass

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False
