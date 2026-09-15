# How autonumbering works

`servo autonumber` takes an assembled arm whose servos have unknown, possibly identical IDs
and gives each joint the right number — without unplugging anything and without the software
ever commanding motion.

It has two hard problems and they are solved separately.

---

## Problem 1: several servos share an ID

Six identical STS3215s, all answering to ID 1. Every write reaches all six, so none can be
addressed apart. The official fix is to unplug five of them and set IDs one at a time, which
on a built arm means taking it apart.

### The 800 ms crack

Broadcast the undocumented reboot instruction `0x08` and every servo restarts. They come back
~797 ms later, but a few hundred microseconds apart. For a moment, exactly one servo is
listening.

Fire `unlock` + `set-ID` into that gap and precisely one servo takes the new ID and stops
answering on the old one. Repeat until the collisions are gone.

### Why the obvious version broke

The first implementation busy-waited to a precisely calculated instant and fired one burst
(`race_peel`, still present as `--legacy-race`). It worked on a Mac. On a Raspberry Pi 5 it
never converged once in three attempts: the Pi's USB stack has about a millisecond of write
jitter, which is wider than the window. No OS knob fixed it.

### The fix: stop timing anything

Don't aim at the window — **sweep it, and let the hardware do the timing.**

Pre-build one contiguous blob of `unlock` + `set-ID` bursts, each burst assigning a
*different* fresh ID, and hand the whole thing to the UART in a single write. At 1 Mbps the
wire clocks it out back-to-back at ~160 µs per 16-byte burst. Whichever burst is passing when
a servo wakes is the one it takes.

```
     reboot                                      wake edge
        |                                            |
        |<--------------- ~797 ms ------------------>|
                                             [====== blob ======]
                                              ^  ^  ^
                                              |  |  servo C wakes -> takes ID 33
                                              |  servo B wakes -> takes ID 31
                                              servo A wakes -> takes ID 29
```

Consequences, all good:

- The host only has to hit the blob's **start** to within a few milliseconds. A jittery USB
  stack manages that.
- **Several servos peel per reboot** instead of one, so it converges in a handful of rounds
  rather than dozens.
- 84 bursts covers 13.4 ms of wake window, far wider than the jitter.

`calibrate_wake_edge()` measures the edge on the current machine first, because the number
includes host USB latency and a constant baked in from the development laptop would put the
sweep in the wrong place elsewhere.

### The trap

IDs assigned mid-sweep are **RAM-only**. There is no quiet moment inside the blob to write the
lock register back, and an unlocked ID reverts on the next reboot — which is exactly what the
next round starts with. So after each sweep round, every servo now on a distinct ID is
re-committed with a full `set_id(s, s)` (torque off, unlock, write, **re-lock**). Without
that, progress silently unwinds every round and the split never finishes.

---

## Problem 2: which servo is which joint?

Distinct IDs still don't tell you which one is the shoulder.

### The approach that lost

You should be able to work this out electrically. Servos are daisy-chained, so each one sits
further along a resistive path from the power injection point. Load a servo, and the voltage
sag at the others encodes distance.

We built it properly: cancel each unit's ADC offset against its own idle baseline so only the
IR drop remains, then average thousands of dithered samples to resolve differences below one
ADC count. We cross-checked it against boot wake-time ordering and against
reboot-reachability.

Scored against known ground truth, **it was not reliable enough to ship.** Adjacent joints
routinely came out within noise of each other. The implementation is preserved in
`tests/legacy/chain_autonumber.py` (`measure_order`, `measure_boot`) and the supporting
experiments are in `experiments/` — `vdrop_spatial.py`, `superres_order.py`,
`locate_research.py`, `boot_order.py`, `collision_arbitration.py`.

### What actually works: ask the human

Turn the torque off and have the operator wiggle each joint by hand, in the order they want
them numbered. The encoder reports position even with torque off, so the arm says which joint
was touched.

It is not clever, and it is correct every time.

Detection is deliberately conservative:

- a movement must exceed `MOVE_THRESHOLD` (80 counts, ~7°); a hand-wiggle is hundreds, noise
  is single digits
- the moved joint must beat the runner-up by `DOMINANCE` (1.8×), so shaking a neighbour
  doesn't register
- the candidate must hold across 3 of 4 confirming re-reads
- everything is re-baselined after each joint, since moving one joint shifts others
- wiggling an already-assigned joint prints a hint rather than being silently ignored

---

## The three steps

```
servo autonumber
```

1. **Scan and split.** Every ID from 1 to `--max-id` is probed. Collisions are resolved as
   above. Reboots make the arm briefly limp, so the tool warns and waits before doing it.
2. **Wiggle-order.** Torque off; you wiggle each joint in turn.
3. **Renumber.** IDs are assigned in wiggle order via a temp ID space (`TEMP_BASE = 100`) so
   no two servos ever collide mid-renumber, then verified.

Useful variants:

```bash
servo autonumber --scan-only      # read-only, no motion, no changes
servo autonumber --detect-only    # detect the order, print it, change nothing
servo autonumber --apply 3,1,2,4,6,5   # commit a known order without wiggling
servo flatten                     # make the mess again, to test the fix
```

## Changing any of this

The split's timing is physical. The simulator in `tests/fakebus.py` models UART pacing and
wake jitter well enough to prove the *logic*, and `tests/test_split_equivalence.py` holds the
emitted bytes to the pre-refactor implementation exactly — but neither can tell you whether a
given host still hits the window. Before and after any change to `split.py`, run the A/B in
`docs/hardware-acceptance.md`.
