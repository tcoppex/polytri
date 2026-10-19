#!/usr/bin/env python3

# -----------------------------------------------------------------------------
"""
Plot the results of `polytri-compare --csv results.csv`.

    python3 plot.py results.csv [--out DIR] [--libs polytri,earcut,CDT,poly2tri]

Writes
  time.png     time per vertex vs size (flat = linear scaling, rising = superlinear)
  ratio.png    heatmap : time relative to the fastest library, per size
  memory.png   peak heap vs size

(depends only on matplotlib)
"""
# -----------------------------------------------------------------------------

import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm

# -----------------------------------------------------------------------------

STYLE = {
    "polytri":  ("#d55e00", "o"),
    "earcut":   ("#0072b2", "s"),
    "CDT":      ("#009e73", "^"),
    "poly2tri": ("#cc79a7", "D"),
}
FALLBACK = [("#555555", "v"), ("#e69f00", "P"), ("#56b4e9", "X")]

# -----------------------------------------------------------------------------

def load(path):
    # data[case][library] = [(vertices, ms, mib), ...]
    data = defaultdict(lambda: defaultdict(list))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if row["ok"] != "1":
                continue
            data[row["case"]][row["library"]].append(
                (int(row["vertices"]), float(row["ms"]), float(row["peak_mib"])))
    return data


def plot(data, libs, cases, column, ylabel, title, path, per_vertex=False):
    fig, axes = plt.subplots(1, len(cases), figsize=(4.6 * len(cases), 4.2), sharey=True)
    if len(cases) == 1:
        axes = [axes]
    extra = iter(FALLBACK)
    style = {}
    for lib in libs:
        style[lib] = STYLE.get(lib) or next(extra, ("#000000", "."))

    for ax, case in zip(axes, cases):
        for lib in libs:
            pts = sorted(data[case].get(lib, []))
            if not pts:
                continue
            color, marker = style[lib]
            ys = [(p[column] * 1e6 / p[0]) if per_vertex else p[column] for p in pts]
            ax.plot([p[0] for p in pts], ys, marker=marker,
                    color=color, label=lib, linewidth=1.8, markersize=5)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_title(case)
        ax.set_xlabel("vertices")
        ax.grid(True, which="major", alpha=0.35)
        ax.grid(True, which="minor", alpha=0.12)
    axes[0].set_ylabel(ylabel)
    axes[-1].legend(frameon=False)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("wrote", path)


def heatmap(data, libs, cases, path):
    """Time of each library divided by the fastest one, for each size."""
    fig, axes = plt.subplots(1, len(cases), figsize=(5.6 * len(cases), 0.7 * len(libs) + 2.2))
    if len(cases) == 1:
        axes = [axes]
    for ax, case in zip(axes, cases):
        sizes = sorted({p[0] for lib in libs for p in data[case].get(lib, [])})
        table = {lib: {p[0]: p[1] for p in data[case].get(lib, [])} for lib in libs}
        best = {n: min(table[lib][n] for lib in libs if n in table[lib]) for n in sizes}
        grid = [[table[lib][n] / best[n] if n in table[lib] else float("nan") for n in sizes] for lib in libs]
        top = max(v for row in grid for v in row if v == v)
        im = ax.imshow(grid, cmap="YlOrRd", norm=LogNorm(vmin=1.0, vmax=max(top, 2.0)), aspect="auto")
        for i, row in enumerate(grid):
            for j, v in enumerate(row):
                if v != v:
                    continue
                dark = LogNorm(vmin=1.0, vmax=max(top, 2.0))(v) > 0.6
                ax.text(j, i, "best" if v < 1.005 else "x%.1f" % v, ha="center", va="center",
                        color="white" if dark else "black", fontsize=10,
                        fontweight="bold" if v < 1.005 else "normal")
        ax.set_xticks(range(len(sizes)))
        ax.set_xticklabels(["%g" % n if n < 1000 else ("%gk" % (n / 1000) if n < 1e6 else "%gM" % (n / 1e6)) for n in sizes])
        ax.set_yticks(range(len(libs)))
        ax.set_yticklabels(libs)
        ax.set_xlabel("vertices")
        ax.set_title(case)
        for edge in ax.spines.values():
            edge.set_visible(False)
        ax.tick_params(length=0)
    fig.suptitle("Time relative to the fastest library (lower is better)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--out", default=".")
    ap.add_argument("--libs", default="polytri,earcut,CDT,poly2tri")
    ap.add_argument("--cases", default="star,star + many holes", help="comma separated case names")
    args = ap.parse_args()

    libs = [s for s in args.libs.split(",") if s]
    data = load(args.csv)
    cases = [c.strip() for c in args.cases.split(",") if c.strip() in data]
    os.makedirs(args.out, exist_ok=True)
    plot(data, libs, cases, 1, "ns per vertex (median call)", "Time per vertex (lower is better, flat = linear)",
         os.path.join(args.out, "time.png"), per_vertex=True)
    heatmap(data, libs, cases, os.path.join(args.out, "ratio.png"))
    plot(data, libs, cases, 2, "peak heap during one call (MiB)", "Peak memory (lower is better)",
         os.path.join(args.out, "memory.png"))


if __name__ == "__main__":
    main()

# -----------------------------------------------------------------------------
