# Hardware acceptance checklist

The test suite runs against a simulator. It proves the logic and the wire format, and it
cannot prove that a given host still hits the ~797 ms wake window. **Run this on a real arm
before trusting a release**, and in full after any change to `split.py`, `bus.py` or
`config.py`.

The pre-refactor implementation is tagged `pre-refactor`, so every step can be A/B'd against
the code as it was when it was known to work.

## Setup

```bash
git checkout pre-refactor && python health_check.py > /tmp/before-health.txt
git checkout -              && servo health          > /tmp/after-health.txt
```

## 1. Read-only tools

| Step | Expect |
|---|---|
| `servo scan --quick` | all six servos found at 1 Mbps |
| `servo scan` | same six; now also probes 76 800 baud, which the old SDK path skipped |
| `servo health` | six rows, sensible temperatures, torque off, mode 0 |
| `diff /tmp/before-health.txt /tmp/after-health.txt` | no differences beyond live values |

## 2. Dropout logging

```bash
servo watch
```

Pose the arm through its range by hand. A servo that goes silent at particular angles is a
pinched cable, not a dead servo. Confirm the log fires on disconnect and recovers.

## 3. Write path

```bash
servo defaults          # then re-run `servo health`: no ACCEL/P/D complaints
servo torque off        # arm goes limp
servo torque on         # arm holds
```

## 4. The full recovery loop — five times

```bash
servo flatten           # deliberately collide every servo onto ID 1
servo autonumber        # split, wiggle, renumber
servo health            # six joints, correctly numbered
```

**Support the arm during the split** — reboots make it briefly limp.

Five consecutive clean runs. Record any that need a retry.

## 5. IDs must survive a power cycle

This is the EEPROM re-lock check and it is the one most likely to regress silently.

```bash
servo autonumber
# physically power-cycle the arm
servo scan --quick      # must still show IDs 1..6
```

If IDs revert, a `set_id` lost its re-lock. See `docs/protocol.md`.

## 6. Split A/B — the real test of the timing path

```bash
git checkout pre-refactor
python auto_ids/test_split.py --mode widesweep --trials 10 --log /tmp/before.jsonl

git checkout -
python experiments/split_bench.py --trials 10 --log /tmp/after.jsonl
python experiments/split_bench.py --compare /tmp/before.jsonl /tmp/after.jsonl
```

Convergence rate and median rounds must be indistinguishable. A drop here is a real
regression in the timing path, and nothing in CI will catch it.

## 7. Raspberry Pi

Repeat steps 4 and 6 on a Pi. This is the platform the hardware-paced sweep exists for — the
blind-fire race scored 0/3 there. Also confirm the legacy path still fails as documented:

```bash
servo autonumber --legacy-race   # expected to struggle on a Pi; fine on macOS
```

## 8. Bench-only

```bash
servo wiggle 3          # ID 3 alone should swing
```

## Recording the result

Note the host, OS, Python version, adapter, servo count and firmware, and the outcome of each
step. Anything that needed a retry is worth writing down even if it eventually passed — the
split is probabilistic and the retry rate is the number that matters.
