# servotools

Command-line tools for Feetech STS3215 servos on a half-duplex bus — the motors in the
[SO-101](https://github.com/TheRobotStudio/SO-ARM100) and similar arms.

The headline feature: **it can renumber the servos of an assembled arm without unplugging
anything**, including when several of them share an ID and can't be addressed apart.

```bash
uvx servotools health
```

```
Port: /dev/tty.usbmodem58FD0168501

========================================================================
ID   Joint              Pos  Temp  Mode  Torq  Load  Status
------------------------------------------------------------------------
1    shoulder_pan      2048   31C     0     0    0%  OK
2    shoulder_lift     1500   32C     0     0    0%  OK
3    elbow_flex        2600   33C     0     0    0%  OK
4    wrist_flex         900   34C     0     0    0%  OK
5    wrist_roll        3100   35C     0     0    0%  OK
6    gripper           2048   36C     0     0    0%  OK
------------------------------------------------------------------------
Temperature range: 31C - 36C

RESULT: All 6 servos healthy.
```

## Install

```bash
pip install servotools     # or: uv tool install servotools
```

Python 3.10+, macOS / Linux / Windows. The only dependency is `pyserial`. Nothing needs to be
installed on the arm.

Running from a clone works too — `./servo health` on macOS and Linux, `servo health` on
Windows.

## Commands

| Command | What it does |
|---|---|
| `servo health` | Check every joint: comms, temperature, mode, torque, PID |
| `servo autonumber` | Number the joints by physical position |
| `servo watch` | Log dropouts while you pose the arm by hand |
| `servo scan` | Find servos at any baud rate; detect a servo poisoning the bus |
| `servo torque off` / `on` | Emergency broadcast torque control |
| `servo defaults` | Write project-standard acceleration and PID to all servos |
| `servo set-id 2` | Give a single fresh servo an ID |
| `servo wiggle 3` | Move one servo so you can see which it is |
| `servo flatten` | Deliberately collide every servo onto one ID (test fixture) |

`servo <command> --help` for options. Every command takes `--port`; without it the adapter is
found by USB VID/PID.

## Numbering an assembled arm

A new arm arrives with every servo set to ID 1. They all answer at once, so none can be
addressed individually, and the official fix is to unplug five of them and set IDs one at a
time — on a built arm, that means taking it apart.

```bash
servo autonumber
```

Instead this:

1. **Splits the collision.** Broadcasts the undocumented reboot instruction `0x08`. The servos
   come back ~797 ms later, a few hundred microseconds apart, and a pre-built sweep of set-ID
   bursts is clocked out across that window by the UART — so whichever burst is passing when a
   servo wakes is the one it takes. Several peel off per reboot, and the host never needs
   precise timing (this matters: the earlier version needed sub-millisecond accuracy and
   never worked on a Raspberry Pi).
2. **Asks which joint is which.** Torque off, and you wiggle each joint by hand in the order
   you want them numbered. The encoder reports angle even unpowered, so the arm tells us which
   one you touched. We tried to deduce this electrically from voltage sag; it was lovely and it
   wasn't reliable. See [docs/autonumber.md](docs/autonumber.md).
3. **Renumbers**, via temporary IDs so nothing collides on the way, then verifies.

The software never commands motion. Reboots do make the arm briefly limp, so support it.

```bash
servo autonumber --scan-only     # read-only: just report what's on the bus
servo autonumber --detect-only   # detect the order, print it, change nothing
servo autonumber --apply 3,1,2,4,6,5
```

## Finding a pinched cable

```bash
servo torque off
servo watch
```

Pose the arm slowly. `watch` logs the first servo to go silent and the angle it was at. A
joint that only drops out at certain positions is a cable being strained, not a dead servo.

## Documentation

- [docs/protocol.md](docs/protocol.md) — the wire format, the register map, instruction
  `0x08`, the EEPROM lock trap, and reading servo count out of collision checksums
- [docs/autonumber.md](docs/autonumber.md) — how the split and the ordering work, and why the
  clever version of the ordering was thrown away
- [docs/reboot-timing.md](docs/reboot-timing.md) — measured wake times, why the apparent
  jitter is the measurement, and why the split needs several rounds
- [docs/troubleshooting.md](docs/troubleshooting.md)
- [docs/testing.md](docs/testing.md) — the simulator, and how the refactor was verified
- [docs/hardware-acceptance.md](docs/hardware-acceptance.md) — what to run on a real arm
- [experiments/](experiments/) — the dead ends, kept on purpose

## Development

```bash
git clone https://github.com/Enigma-Incorporated/servotools
cd servotools
pip install -e ".[dev]"

pytest                                   # no hardware needed
python tests/capture.py --impl new --check
ruff check .
```

The test suite runs against an STS3215 simulator that models UART pacing, reboot wake jitter,
EEPROM lock semantics and bus-collision arbitration — enough that the collision split actually
converges in it. `tests/legacy/` holds the pre-refactor implementation, and the equivalence
tests assert the current code puts byte-identical traffic on the wire.

What the simulator can't tell you is whether a particular host still hits the wake window.
That needs [an arm](docs/hardware-acceptance.md).

## Hardware notes

Built against STS3215 servos and a Waveshare bus servo adapter (USB `1A86:55D3`) at 1 Mbps.
Other Feetech STS/SMS servos speak the same protocol and will probably work; other adapters
need `--port`.

## License

MIT
