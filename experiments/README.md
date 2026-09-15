# experiments

Dead ends, one-shot probes and bench harnesses. **Not part of the shipped tool.**

These are kept deliberately. Working out how to renumber six identical servos on one
half-duplex bus took a lot of things that didn't work, and the record of what was measured —
and what the measurements actually said — is more useful to someone poking at a Feetech bus
than a clean repo would be. `docs/protocol.md` and the blog post both draw on what is here.

## Ground rules

- **Frozen.** These scripts are preserved as they were run. They are not refactored onto the
  shared bus layer, not linted, not tested, and not packaged. Rewriting them would destroy the
  thing that makes them worth keeping.
- Most take no arguments. IDs, ground truth and tuning constants are baked in as module
  constants; several encode a specific arm on a specific bench.
- Several assume a particular hardware state (a known collision, a known chain order) and will
  report nonsense without it. Two of them disagree with each other about ground truth.
- They command motion. Do not point them at an assembled arm you care about.

`_common.py` is the one exception: it is a shim onto `servotools` so the scripts keep working
without carrying a third copy of the port-discovery helpers.

## What's here

### Seeing the collision

| Script | Finding |
|---|---|
| `diag_double_id.py` | Read-only test for "one servo at ID 1" vs "two colliding". |
| `decode_collision.py` | Hammer ID 1, keep only checksum-valid replies; positions cluster into two groups. Counts servos without touching them. |

### The reboot window

| Script | Finding |
|---|---|
| `gap_measure.py` | Sub-millisecond measurement of the single-servo boot gap. |
| `reboot_window.py`, `reboot_edge.py`, `reboot_probe.py` | Where the silence → clean → collision edges sit after instruction `0x08`. |
| `split_ids.py` | The original two-servo blind-fire race. Direct ancestor of the shipped splitter. |
| `finalize.py` | Post-split cleanup: verify independent addressability, normalise to IDs 1 and 2. |

### Deducing chain order electrically — the idea that lost

All of these tried to work out which servo is physically where, in software. Scored against
ground truth, **none was reliable enough to ship.** The tool asks you to wiggle the joints
instead.

| Script | Approach |
|---|---|
| `vdrop_spatial.py` | Rank by voltage drop from each servo's own idle baseline (cancels per-unit ADC offset). |
| `vgrad.py`, `chain_order.py` | Supply-voltage gradient under load. |
| `superres_order.py`, `id2_vs_id3.py` | Dithered-mean superresolution to resolve sub-ADC differences. |
| `locate_research.py` | Per-servo current injection; sag matrix scored against ground truth. |
| `boot_order.py` | Does reboot wake time correlate with distance from the power feed? |
| `collision_arbitration.py` | Does bus arbitration encode distance from the adapter? |
| `reboot_reachability.py` | Does rebooting servo X knock out everything downstream? |
| `movefree_probe.py` | Motion-free signals: comm latency, error rate, TX-driver sag. |
| `jam_blocker.py` | Can one servo's bus jam block another, position-dependently? |

The shipped implementation of the voltage-sag ordering is not duplicated here: it is
preserved as `measure_order` / `measure_boot` in `tests/legacy/chain_autonumber.py`, which is
kept as the pre-refactor reference anyway.

### Bus limits and utilities

| Script | Purpose |
|---|---|
| `baud_explore.py`, `baud_explore2.py` | Can the bus run above 1 Mbps? Which register selects it? |
| `renumber.py` | Apply a hardcoded ID mapping via temp IDs. |
| `wiggle_test.py` | Wiggle each servo in turn so you can see which is which. |
| `split_bench.py` | Hardware A/B harness for split strategies. See below. |
| `show_collision.py` | Raw bytes from a read or ping, with the checksum verdict. |
| `show_setid_blocker.py` | Why setting one servo's ID fails when two share it. |
| `measure_reboot.py` | Wake-time statistics after instruction 0x08. |
| `measure_wake_gap.py` | Wake gap via the collision window. Superseded — see docs/reboot-timing.md. |
| `plot_reboot.py` | Renders the figures in docs/reboot-timing.md. |

## `split_bench.py`

The one script here that is maintained, because the hardware acceptance checklist depends on
it. Each trial flattens the arm onto ID 1 and splits it again with a chosen strategy, logging
JSON lines.

```bash
python experiments/split_bench.py --mode widesweep --trials 10 --log trials.jsonl
```

Use it to compare split success rate across a change. It deliberately destroys joint
numbering — re-run `servo autonumber` afterwards.
