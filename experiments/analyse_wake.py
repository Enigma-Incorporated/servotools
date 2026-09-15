#!/usr/bin/env python3
"""Combine wake-time runs and test what the per-servo offset is attached to.

Each run is keyed by the physical unit (its resting position), not by ID, so runs
taken with different ID assignments and different chain positions are comparable.

Wake time drifts upward within a run, so the offset is reported two ways: raw, and
after removing a per-servo linear trend in trial index.

    python experiments/analyse_wake.py run.json:original other.json:swapped
"""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path


def fit(xs, ys):
    """Least-squares slope, mean, and standard error of the mean."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True)) / den if den else 0.0
    resid = [y - (my + slope * (x - mx)) for x, y in zip(xs, ys, strict=True)]
    return slope, my, statistics.pstdev(resid) / (n ** 0.5)


def load(path):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    units = d["units"]
    out = {}
    for sid in d["ids"]:
        unit = int(units[str(sid)])
        out[unit] = [(r["trial"], r["woke"][str(sid)]) for r in d["rows"] if str(sid) in r["woke"]]
    # Paired per-trial difference, split by polling order so the round-robin bias cancels.
    a, b = d["ids"]
    by_order: dict[tuple, list] = {}
    for r in d["rows"]:
        if str(a) in r["woke"] and str(b) in r["woke"]:
            val = r["woke"][str(a)] - r["woke"][str(b)]
            if int(units[str(a)]) != 318:  # always report unit@318 minus unit@2500
                val = -val
            by_order.setdefault(tuple(r["order"]), []).append(val)
    return out, by_order


def main() -> int:
    specs = [s.rsplit(":", 1) for s in sys.argv[1:]]
    if not specs:
        print(__doc__)
        return 2

    print(f"{'run':<32} {'raw 318-2500':>19} {'order-balanced':>19} {'bias':>8}")
    print("-" * 84)
    groups: dict[str, list] = {}
    for path, cond in specs:
        units, by_order = load(path)
        s318, m318, se318 = fit(*zip(*units[318], strict=True))
        s2500, m2500, se2500 = fit(*zip(*units[2500], strict=True))
        raw, raw_se = m318 - m2500, (se318 ** 2 + se2500 ** 2) ** 0.5

        means = [statistics.mean(v) for v in by_order.values()]
        sems = [statistics.pstdev(v) / (len(v) ** 0.5) for v in by_order.values()]
        bal = sum(means) / len(means)
        bal_se = (sum(s ** 2 for s in sems) ** 0.5) / len(means)
        bias = (means[0] - means[1]) / 2 if len(means) == 2 else float("nan")

        print(f"{Path(path).stem[:31]:<32} {raw:+9.3f} +/- {raw_se:.3f} "
              f"{bal:+9.3f} +/- {bal_se:.3f} {bias:+8.3f}")
        print(f"{'  [' + cond + ']':<32} {'within-run drift:':>19} "
              f"318 {s318 * len(units[318]) * 1000:+.0f}us   2500 {s2500 * len(units[2500]) * 1000:+.0f}us")
        groups.setdefault(cond, []).append((bal, bal_se))

    print("-" * 84)
    print("\nCombined per condition (order-balanced, inverse-variance weighted):")
    combined = {}
    for cond, rows in groups.items():
        w = sum(1 / se ** 2 for _, se in rows)
        m = sum(v / se ** 2 for v, se in rows) / w
        combined[cond] = (m, 1 / w ** 0.5)
        print(f"  unit@318 - unit@2500,  {cond:<22} {m:+.3f} +/- {1 / w ** 0.5:.3f} ms"
              f"   ({len(rows)} run(s))")

    if len(combined) == 2:
        (_, (m1, s1)), (_, (m2, s2)) = combined.items()
        d, ds = m1 - m2, (s1 ** 2 + s2 ** 2) ** 0.5
        print(f"\n  change on swapping the chain: {d:+.3f} +/- {ds:.3f} ms  ({abs(d / ds):.1f} sigma)")
        print("    intrinsic to the servo  -> expect 0")
        print("    set by chain position   -> expect twice the offset, i.e. a sign flip")
    return 0


if __name__ == "__main__":
    sys.exit(main())
