# Troubleshooting

## "no servo bus adapter found"

The adapter is matched by USB VID/PID `1A86:55D3` (Waveshare bus servo adapter). If yours is a
different model it won't be auto-detected — pass `--port` explicitly:

```bash
servo health --port /dev/ttyACM0        # Linux
servo health --port /dev/tty.usbmodem1  # macOS
servo health --port COM4                # Windows
```

`python -m serial.tools.list_ports -v` lists what's actually attached.

On Linux you may need to be in the `dialout` group:
`sudo usermod -a -G dialout $USER`, then log out and back in.

## Nothing found, but the adapter is there

In order of likelihood:

1. **No 12 V.** The servos need their own supply; USB alone powers the adapter, not the bus.
   Nothing answers and nothing warns you.
2. **Jumper in the wrong position.** The Waveshare adapter has a USB/UART jumper.
3. **Wrong baud.** `servo scan` sweeps every standard rate. If servos turn up at, say,
   115 200, the auto-numbering path still needs 1 Mbps.
4. **Cable in the wrong port.** The chain has an in and an out.

## "N distinct + 1 collided ID(s)"

Two or more servos share an ID. That's what `servo autonumber` is for. It is also the expected
state of a freshly built arm, since every servo ships as ID 1.

## The split doesn't converge

`servo autonumber` reports `could not fully split (still colliding at [...])`.

- **Just rerun it.** The split is probabilistic; a second run usually finishes it.
- **Keep the bus quiet.** Nothing else may talk to the port during a run. Traffic during the
  boot window aborts the bootloader wait and the servos wake early.
- **Don't use `--legacy-race`** unless you're on a host with sub-millisecond USB write timing.
  It is kept for reference and is known not to converge on a Raspberry Pi.
- **Check the reported wake edge.** The line `wake edge ~797.0ms on this host` should be in
  the 780–820 ms range. Something far off means calibration failed and the sweep is aiming at
  the wrong moment.

## IDs revert after a power cycle

An ID write didn't get its EEPROM re-lock. Re-run `servo autonumber`; if it recurs, that's a
bug — see the lock section of `docs/protocol.md`.

## A joint drops out only at some angles

A pinched or fatigued cable, not a dead servo.

```bash
servo torque off
servo watch
```

Pose the arm slowly. `watch` logs the first ID to go silent along with the position it was at,
which usually points straight at the joint whose cable is being strained.

## "WARNING: Received N unsolicited bytes!"

A servo is transmitting unprompted and drowning out everyone else. Disconnect servos one at a
time until the noise stops.

## Servos are hot

`servo health` warns at 45 °C and calls it critical at 60 °C. Usual cause is torque left on
while the arm holds a pose against gravity:

```bash
servo torque off
```

## Windows timing

Python 3.11+ is recommended. The split needs sub-millisecond sleeps; the tools call
`timeBeginPeriod(1)` to raise the system timer resolution from its ~15.6 ms default, but older
interpreters are less reliable about honouring short sleeps.

## Everything looks wrong after an interrupted run

An interrupted split can leave servos parked at arbitrary IDs anywhere up to 89. That's
recoverable and expected:

```bash
servo autonumber --scan-only    # see where everything actually is
servo autonumber                # picks up from any starting state
```
