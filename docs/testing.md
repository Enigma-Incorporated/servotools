# Testing without an arm

These tools talk to hardware, so the test strategy is built around a simulator rather
than mocks. The goal is narrow and specific: **prove that refactoring did not change what
the tools do to the bus or print to the user**, on a machine with no servos attached.

## The three pieces

### `tests/clock.py` — virtual clock

Patches `time.time`, `time.perf_counter`, `time.monotonic` and `time.sleep` process-wide.
`sleep()` advances the clock instantly; every clock *read* advances it by `TICK` (100 µs) so
that busy-wait loops such as the one in `race_peel` still terminate. A reboot cycle that takes
0.8 s of wall time on hardware costs microseconds here, and the results are reproducible.

`TICK` is a compromise. Too large and the sub-millisecond fire-time bisection loses resolution;
too small and the SDK's ~34 ms packet-timeout spin takes millions of iterations. 100 µs keeps
both usable.

### `tests/fakebus.py` — STS3215 simulator

A `serial.Serial`-compatible object (`FakeSerial`) in front of a `FakeBus` carrying several
`Servo` objects. It is deliberately faithful in the places the tools branch on:

- **Framing.** `FF FF id len inst params checksum`, `checksum = ~sum & 0xFF`, little-endian
  words. Instructions `0x01` PING, `0x02` READ, `0x03` WRITE, `0x08` REBOOT, broadcast `0xFE`.
- **UART pacing.** `write()` treats the payload as bytes clocked onto the wire at
  `10 / baudrate` seconds each. A servo that wakes mid-write only sees the bytes that arrive
  after it wakes. This is the entire mechanism the sweep split depends on, so it is modelled
  rather than faked: the sweep peels servos apart in the simulator for the same reason it does
  on hardware.
- **Reboot timing.** Each servo wakes at `797 ms ± 0.4 ms`, re-rolled per reboot from a seeded
  RNG, so wake order varies but runs are reproducible.
- **EEPROM lock.** Writes below address 40 are ignored while locked. Writing `1` to the lock
  register commits the EEPROM region; a reboot restores the last committed copy. An ID written
  without a re-lock therefore *reverts on the next reboot* — the real trap, encoded so a
  regression that drops the re-lock fails a test instead of shipping.
- **Collisions.** When several servos answer one transaction, the replies are resolved once:
  identical replies overlay cleanly, otherwise one servo wins the line with probability
  `p_arbitrate` and the rest are garbled into a checksum failure. Two servos parked at the
  *same angle* produce byte-identical replies and so read as a clean single — reproducing the
  same-angle degeneracy that `probe_id` guards against with 14 repeats.

Arbitration is resolved per transaction, not pairwise. Pairwise merging makes a clean packet
require every other servo to lose independently, which is ~10⁻⁵ for six servos; wake-edge
calibration then never sees the single-servo window and the split cannot converge.

What is *not* modelled: bus traffic aborting the bootloader wait, per-servo return delay,
temperature drift, and current draw. None of the shipped tools branch on them.

### `tests/capture.py` — golden records

Runs a tool end to end against a seeded scenario and writes everything observable to
`tests/golden/<case>.txt`:

1. exit code
2. full stdout
3. the **decoded op-log** — every well-formed packet the host emitted, as
   `baud id=N INST addr=A len=L`
4. the final register state of every virtual servo

```bash
python tests/capture.py            # rewrite the records
python tests/capture.py --check    # compare, non-zero exit on any diff
python tests/capture.py --only split.
```

The op-log is the load-bearing artifact. Raw byte streams are *not* comparable across the
`scservo_sdk` → raw-bus migration — the SDK sizes reads differently and its `ping` chases the
packet with a model-number read — but the sequence of semantic operations must survive, and
that is what the op-log captures.

`serial.Serial` is patched globally, which covers the raw tools and the SDK alike, because
`scservo_sdk.PortHandler` holds a plain `serial.Serial` in `self.ser`. That is what makes it
possible to capture the *pre-refactor* code's behavior and diff the refactor against it.

## Scenarios

`tests/scenarios.py`: `healthy`, `missing`, `hot`, `misconfigured`, `collided`,
`collided_degenerate`, `scattered`, `single`, `empty`, `other_baud`.

## What this does not cover

- **Real timing.** The simulator says the sweep split's *logic* is right, not that the host
  hits the wake window on a given machine. Only hardware can say that; see the acceptance
  checklist in `docs/hardware-acceptance.md`.
- **The interactive wiggle flow.** Detection is unit-tested against a scripted bus, but the
  full hand-wiggle loop is a manual test.
- **The legacy `--legacy-race` split**, which needs 300 reboot cycles to characterise. Covered
  by a byte-identity test on the packets it emits, not by an end-to-end run.
