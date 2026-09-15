#!/usr/bin/env python3
"""Plot the reboot wake-time statistics collected by measure_reboot.py.

    python experiments/plot_reboot.py --run a.json:original --run b.json:swapped \
        --coarse wake_800us.json --out docs/images/reboot-wake
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

THEMES = {
    "light": {"surface": "#fcfcfb", "ink": "#0b0b0b", "ink2": "#52514e",
              "muted": "#b8b7b0", "s1": "#2a78d6"},
    "dark": {"surface": "#1a1a19", "ink": "#ffffff", "ink2": "#c3c2b7",
             "muted": "#55544e", "s1": "#3987e5"},
}
POLL_US = {"fine": 629, "coarse": 1087}  # measured poll period per sample
BURST_US = 160  # one 16-byte set-ID burst at 1 Mbps: the sweep's resolution


def load(path, cond=None):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    units = d["units"]
    rows = []
    for r in d["rows"]:
        if len(r["woke"]) < 2:
            continue
        rows.append({int(units[str(k)]): v for k, v in r["woke"].items()})
    return {"cond": cond, "rows": rows,
            "all": [v for w in rows for v in w.values()],
            "delta": [w[318] - w[2500] for w in rows]}


def style(ax, t):
    ax.set_facecolor(t["surface"])
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(t["muted"])
        ax.spines[s].set_linewidth(1)
    ax.tick_params(colors=t["ink2"], labelsize=9, length=3, width=1)
    ax.grid(True, color=t["muted"], alpha=0.35, linewidth=0.8)
    ax.set_axisbelow(True)


def panel_wake(ax, runs, t):
    """Every measurement from both servos, undifferentiated: they are one population."""
    xs, ys, swap_at, prev = [], [], None, None
    x = 0
    for run in runs:
        if prev is not None and run["cond"] != prev:
            swap_at = x - 0.5
        prev = run["cond"]
        for w in run["rows"]:
            for v in w.values():
                xs.append(x)
                ys.append(v)
            x += 1
    ax.plot(xs, ys, "o", ms=3, color=t["s1"], alpha=0.4, markeredgecolor="none")
    grand = statistics.mean(ys)
    ax.axhline(grand, color=t["s1"], lw=2, alpha=0.85)
    ax.text(x * 1.005, grand, f" mean {grand:.2f} ms", color=t["s1"], fontsize=10,
            va="center", fontweight="bold")
    if swap_at is not None:
        ax.axvline(swap_at, color=t["ink2"], lw=1.5, ls=(0, (5, 3)))
        ax.annotate("servos swapped in the chain", (swap_at, max(ys)),
                    xytext=(9, 0), textcoords="offset points", color=t["ink2"],
                    fontsize=9.5, va="top")
    ax.set_xlim(-x * 0.015, x * 1.16)
    ax.set_xlabel("reboot", color=t["ink2"], fontsize=10)
    ax.set_ylabel("wake time (ms)", color=t["ink2"], fontsize=10)
    ax.set_title(f"Wake time, {x} reboots, both servos",
                 color=t["ink"], fontsize=12.5, loc="left", pad=8)


def panel_quantisation(ax, runs, coarse, t):
    labels = [f"{POLL_US['fine']} us poll\n(revisit {POLL_US['fine'] * 2} us)"]
    measured = [statistics.pstdev(runs[0]["all"]) * 1000]
    predicted = [POLL_US["fine"] * 2 / (12 ** 0.5)]
    if coarse is not None:
        labels.append(f"{POLL_US['coarse']} us poll\n(revisit {POLL_US['coarse'] * 2} us)")
        measured.append(statistics.pstdev(coarse["all"]) * 1000)
        predicted.append(POLL_US["coarse"] * 2 / (12 ** 0.5))
    idx = range(len(labels))
    ax.bar([i - 0.19 for i in idx], measured, 0.36, color=t["s1"],
           edgecolor=t["surface"], linewidth=2, label="measured sd")
    ax.bar([i + 0.19 for i in idx], predicted, 0.36, color=t["muted"],
           edgecolor=t["surface"], linewidth=2, label="predicted by quantisation")
    for i, (m, p) in enumerate(zip(measured, predicted, strict=True)):
        ax.text(i - 0.19, m + 12, f"{m:.0f}", ha="center", color=t["ink"],
                fontsize=9.5, fontweight="bold")
        ax.text(i + 0.19, p + 12, f"{p:.0f}", ha="center", color=t["ink2"], fontsize=9.5)
    ax.set_xticks(list(idx))
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel("standard deviation (us)", color=t["ink2"], fontsize=10)
    ax.set_title("Spread scales with the poll interval",
                 color=t["ink"], fontsize=12.5, loc="left", pad=8)
    leg = ax.legend(frameon=False, fontsize=9.5, loc="upper left")
    for txt in leg.get_texts():
        txt.set_color(t["ink2"])


def panel_delta(ax, runs, t):
    """The measured gap sits at +/- one poll period, because the grid puts it there."""
    deltas = [d for run in runs for d in run["delta"]]
    lim = 1.6
    ax.hist([d for d in deltas if abs(d) <= lim], bins=33, range=(-lim, lim),
            color=t["s1"], edgecolor=t["surface"], linewidth=1.2, zorder=2)
    ax.axvline(0, color=t["muted"], lw=1.5, zorder=1)
    ax.set_xlim(-lim, lim)
    ax.set_xlabel("gap between the two servos, per reboot (ms)",
                  color=t["ink2"], fontsize=10)
    ax.set_ylabel("reboots", color=t["ink2"], fontsize=10)
    ax.set_title("Gap", color=t["ink"], fontsize=12.5, loc="left", pad=8)


def draw(runs, coarse, theme: str, out: Path) -> None:
    t = THEMES[theme]
    fig = plt.figure(figsize=(12.5, 6.9), facecolor=t["surface"])
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1], hspace=0.42, wspace=0.22,
                          left=0.07, right=0.975, top=0.935, bottom=0.09)
    for ax, fn in ((fig.add_subplot(gs[0, :]), lambda a: panel_wake(a, runs, t)),
                   (fig.add_subplot(gs[1, 0]), lambda a: panel_quantisation(a, runs, coarse, t)),
                   (fig.add_subplot(gs[1, 1]), lambda a: panel_delta(a, runs, t))):
        style(ax, t)
        fn(ax)
    out.parent.mkdir(parents=True, exist_ok=True)
    path = out.with_name(f"{out.name}-{theme}.png")
    fig.savefig(path, dpi=170, facecolor=t["surface"])
    plt.close(fig)
    print(f"wrote {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", action="append", default=[], metavar="PATH:CONDITION")
    ap.add_argument("--coarse", default=None)
    ap.add_argument("--out", default="docs/images/reboot-wake")
    args = ap.parse_args()
    runs = [load(*spec.rsplit(":", 1)) for spec in args.run]
    coarse = load(args.coarse) if args.coarse else None
    for theme in ("light", "dark"):
        draw(runs, coarse, theme, Path(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
