#!/usr/bin/env python3
"""A/B harness for the collision split, on real hardware.

Each trial flattens every servo onto one ID and splits them apart again, recording
whether it converged, how many sweep rounds it took and how long it ran. Results go
to a JSON-lines file so two branches can be compared.

    python experiments/split_bench.py --trials 10 --log before.jsonl
    git checkout <other-branch>
    python experiments/split_bench.py --trials 10 --log after.jsonl
    python experiments/split_bench.py --compare before.jsonl after.jsonl

BENCH USE ONLY: this deliberately destroys joint numbering. Re-run `servo autonumber`
afterwards. The pre-refactor version of this harness, which also compared the
blind-fire race variants (stock/reject/calib/fast/stream/padstream), is preserved at
the `pre-refactor` git tag.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from servotools.bus import DOUBLE, Bus  # noqa: E402
from servotools.commands.flatten import flatten  # noqa: E402
from servotools.config import BAUD  # noqa: E402
from servotools.ports import NOT_FOUND, find_port  # noqa: E402
from servotools.split import WIDE_IDS, split_all  # noqa: E402

FLAT_ID = 1


def run_trial(bus: Bus, index: int) -> dict:
    flatten(bus, FLAT_ID)
    before = bus.scan(WIDE_IDS)
    started = time.perf_counter()
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        ids, leftover = split_all(bus, WIDE_IDS)
    elapsed = time.perf_counter() - started
    text = log.getvalue()
    return {
        "trial": index,
        "ok": not leftover and len(ids) >= 2,
        "servos": len(ids),
        "leftover": leftover,
        "rounds": text.count("[sweep "),
        "seconds": round(elapsed, 2),
        "collided_before": [s for s, v in before.items() if v == DOUBLE],
        "output": text,
    }


def summarise(rows: list[dict]) -> dict:
    ok = [r for r in rows if r["ok"]]
    rounds = [r["rounds"] for r in ok]
    return {
        "trials": len(rows),
        "converged": len(ok),
        "rate": round(len(ok) / len(rows), 3) if rows else 0.0,
        "median_rounds": statistics.median(rounds) if rounds else None,
        "median_seconds": statistics.median([r["seconds"] for r in ok]) if ok else None,
    }


def compare(before: Path, after: Path) -> int:
    def load(p):
        text = p.read_text(encoding="utf-8")
        return [json.loads(line) for line in text.splitlines() if line.strip()]

    a, b = summarise(load(before)), summarise(load(after))
    print(f"{'':<18}{before.name:>16}{after.name:>16}")
    for key in ("trials", "converged", "rate", "median_rounds", "median_seconds"):
        print(f"{key:<18}{str(a[key]):>16}{str(b[key]):>16}")
    if b["rate"] < a["rate"]:
        print("\nWARNING: the split converged less often after the change.")
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None)
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--log", default="split_trials.jsonl")
    ap.add_argument("--compare", nargs=2, metavar=("BEFORE", "AFTER"))
    args = ap.parse_args()

    if args.compare:
        return compare(Path(args.compare[0]), Path(args.compare[1]))

    port = args.port or find_port()
    if port is None:
        print(NOT_FOUND)
        return 1

    bus = Bus.open(port, BAUD)
    rows = []
    with open(args.log, "a", encoding="utf-8") as fh:
        for i in range(1, args.trials + 1):
            print(f"\n=== trial {i}/{args.trials} ===", flush=True)
            row = run_trial(bus, i)
            rows.append(row)
            fh.write(json.dumps(row) + "\n")
            fh.flush()
            print(f"  {'OK ' if row['ok'] else 'FAIL'} "
                  f"{row['servos']} servos, {row['rounds']} rounds, {row['seconds']}s")
    bus.close()

    print(f"\n{json.dumps(summarise(rows), indent=2)}")
    print(f"\nwrote {args.log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
