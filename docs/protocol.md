# The STS3215 bus, as we found it

Everything `servotools` knows about the wire. Some of this is in Feetech's datasheet; the
parts that aren't were measured, and the experiments that measured them are in
`experiments/`.

## Physical layer

One half-duplex serial line. Every servo sees every byte; the host and the servos take turns
on the same wire. Default 1 Mbps, 8N1. The whole reason this project exists is that a servo
is addressed only by an ID stored in its own EEPROM — so if two servos hold the same ID, both
answer at once, their replies overlap on the wire, and there is no way to talk to one without
the other.

## Packet format

```
FF FF  <id>  <len>  <inst>  <params...>  <checksum>
```

- `len` = number of parameter bytes + 2
- `checksum` = `~(id + len + inst + sum(params)) & 0xFF`
- Multi-byte register values are **little-endian**
- `id` `0xFE` is the broadcast address: every servo acts on it and **none replies**

A status reply has the same shape, with an error byte where the instruction byte was:

```
FF FF  <id>  <len>  <error>  <params...>  <checksum>
```

`servotools.bus.find_frame` scans a response window for the first checksum-valid packet from
a given ID. It resyncs a byte at a time rather than assuming the reply starts at offset 0,
because on a shared line it often doesn't.

## Instructions

| Code | Name | Notes |
|---|---|---|
| `0x01` | PING | Reply is a bare status packet |
| `0x02` | READ | Params: address, length |
| `0x03` | WRITE | Params: address, data… |
| `0x08` | REBOOT | **Undocumented.** See below |

### `0x08`, the reboot instruction

Not in the datasheet and not in any Feetech SDK, but it is real: it is the STS3215 firmware's
bootloader-entry opcode. Broadcast `FF FF FE 02 08 F7` and every servo on the bus restarts.

This is the single most useful thing we found, because of what happens next.

## The wake window

After a reboot the bus goes silent for about **797 ms**, then the servos come back — but not
at the same instant. They wake a few hundred microseconds apart, and for a brief moment
exactly one servo is listening.

That is the crack that lets two servos sharing an ID be told apart. See
`docs/autonumber.md` for what we do with it.

Two practical constraints, both learned the hard way:

- **The bus must stay quiet during the boot window.** Continuous traffic aborts the
  bootloader wait and the servos come back early. `SKIP_S = 0.70` is how long the tools sit
  on their hands before starting to poll for the edge.
- **The exact wake time varies by host**, because the measurement includes USB stack latency.
  `calibrate_wake_edge()` measures it per machine rather than trusting a constant.

## The EEPROM lock

Register 55 is a lock. While it is `1`, writes to the EEPROM region (addresses below 40,
which includes the ID at address 5) are **silently ignored** — no error, no reply, nothing.

The full sequence to change an ID is:

```
torque off  ->  write lock=0  ->  write ID  ->  write lock=1
```

**Writing `1` back to the lock is what commits the change to EEPROM.** Skip it and the servo
answers to its new ID immediately, everything looks fine, and then the next reboot or power
cycle silently reverts it. This cost us a confusing afternoon, and it is the reason the split
re-commits every peeled servo with a proper `set_id(s, s)` before the next round's reboot: the
IDs the sweep assigns are RAM-only, because there is no quiet moment mid-sweep to lock them.

`tests/fakebus.py` models this, so a regression that drops the re-lock fails a test.

## Collisions, and reading them

When two servos answer at once their bytes overlap and the checksum fails. That failure is
information:

- **a clean, checksum-valid reply** means exactly one servo answered
- **garbage** proves at least two did

So you can *count* servos on an ID without touching them. `Bus.position_class()` returns `C`,
`x` or `-` for clean / collision / silence, and `Bus.probe()` calls it 14 times.

Why 14: two servos parked at the **same angle** send byte-identical replies, which overlay
into a valid packet. Such a pair looks like a single servo on most reads and only betrays
itself occasionally. One clean read proves nothing; repeated reads are needed before a SINGLE
verdict is trustworthy. `experiments/decode_collision.py` is where this was worked out — hammer
ID 1 a few thousand times, keep the valid packets, and the positions cluster into two groups.

Bus arbitration also means one servo sometimes simply wins the line and gets a clean packet
through even in a collision, which is why "clean" is never proof of "single".

## Register map

`servotools/registers.py` is the source of truth. The ones that matter:

| Addr | Register | Width | Notes |
|---|---|---|---|
| 3 | Model number | 2 | |
| 5 | ID | 1 | EEPROM, lock-guarded |
| 6 | Baud rate | 1 | |
| 7 | Return delay | 1 | |
| 8 | Response level | 1 | |
| 21 / 22 / 23 | P / D / I | 1 | EEPROM |
| 33 | Mode | 1 | 0 = position |
| 37 | Acceleration | 1 | EEPROM |
| 40 | Torque enable | 1 | Must be 0 for EEPROM writes |
| 42 | Goal position | 2 | |
| 55 | Lock | 1 | See above |
| 56 | Present position | 2 | Reported even with torque off |
| 58 | Present speed | 2 | |
| 60 | Present load | 2 | |
| 62 | Voltage | 1 | Tenths of a volt |
| 63 | Temperature | 1 | °C |

> **Unverified on hardware.** `servo health` labels a column "Load" but reads address 58,
> which the standard map calls Present Speed. This is inherited from the pre-refactor code and
> was kept deliberately so the refactor changed no behavior. If 58 really is speed, both that
> column and `Present_Load` in `servo scan`'s dump are mislabelled. Worth confirming against a
> servo under known load before changing.

## Baud rates

`STANDARD_BAUDS` is 1 M, 500 k, 250 k, 128 k, 115.2 k, 76.8 k, 57.6 k, 38.4 k.

Note that `scservo_sdk` refuses 76 800 — its `getCFlagBaud()` whitelist omits it, so
`setBaudRate()` returns False and leaves the port where it was. The old `scan_bus` therefore
scanned 115 200 twice and never looked at 76 800 at all. Setting `pyserial.Serial.baudrate`
directly works, so `servo scan` now covers it.

## Timing constants

Every wait in `config.py` and `split.py` was tuned against hardware. They are not
interchangeable and collapsing them to one value is a real behavior change:

| Constant | Value | What it is |
|---|---|---|
| `PORT_TIMEOUT` | 6 ms | Normal register traffic |
| `PORT_TIMEOUT_FIRE_AND_FORGET` | 100 ms | Torque commands, no reply expected |
| `PORT_TIMEOUT_WAKE_POLL` | 1 ms | Polling for the wake edge |
| `WAIT_READ` / `WAIT_PING` / `WAIT_POSITION` | 1 / 2 / 4 ms | Settle before reading the reply |
| `SKIP_S` | 700 ms | Silence held before the wake edge |
| `SWEEP_LEAD_MS` | 6 ms | How early the sweep starts |
| `SWEEP_SPAN_IDS` | 84 | Bursts per sweep — 84 × 16 B = 13.4 ms of covered window |

On Windows, `timers.enable_high_res_timers()` asks for a 1 ms system timer so the sub-millisecond
waits mean anything. Without it the default granularity is ~15.6 ms.
