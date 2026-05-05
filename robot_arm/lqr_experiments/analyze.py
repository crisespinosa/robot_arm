"""
Post-processing for LQR experiments (run_experiments.py).

Reads ./results/summary.json + ./results/*.csv and produces:
  ./results/figures/*.png  (one per experiment + comparison figures)
  ./results/figures/*.pdf  (same, vector)
  ./results/summary.xlsx   (metrics table, one row per config)

Usage:
  pip install matplotlib pandas openpyxl
  python analyze.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # no display needed

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


RESULTS_DIR = Path(__file__).parent / "results"
FIG_DIR = RESULTS_DIR / "figures"

JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]

# Plot style
plt.rcParams.update({
    "figure.dpi": 110,
    "savefig.dpi": 200,
    "font.size": 10,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "lines.linewidth": 1.3,
})


# ────────────────────────────────────────────────────────────────────
#                        HELPERS
# ────────────────────────────────────────────────────────────────────

def savefig(fig, stem: str) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(FIG_DIR / f"{stem}.png")
    fig.savefig(FIG_DIR / f"{stem}.pdf")
    plt.close(fig)
    print(f"  saved {stem}.png / .pdf")


def load_run(csv_name: str) -> pd.DataFrame:
    return pd.read_csv(RESULTS_DIR / csv_name)


# ────────────────────────────────────────────────────────────────────
#               FIGURE BUILDERS — PER EXPERIMENT
# ────────────────────────────────────────────────────────────────────

def fig_tracking_single(df: pd.DataFrame, title: str, stem: str) -> None:
    """Single plot: q(t) and q_ref(t) for all 6 joints."""
    fig, ax = plt.subplots(figsize=(11, 6))

    for i in range(6):
        ax.plot(
            df["t"], df[f"qref{i}"],
            linestyle="--",
            linewidth=1.2,
            label=f"{JOINT_NAMES[i]} ref"
        )
        ax.plot(
            df["t"], df[f"q{i}"],
            linewidth=1.5,
            label=f"{JOINT_NAMES[i]} plant"
        )

    ax.set_xlabel("t [s]")
    ax.set_ylabel("q [rad]")
    ax.set_title(title)
    ax.legend(fontsize=8, ncol=2)
    savefig(fig, stem)


def fig_error_single(df: pd.DataFrame, title: str, stem: str) -> None:
    """Single plot: tracking error q(t)-q_ref(t) for all 6 joints."""
    fig, ax = plt.subplots(figsize=(11, 5))

    for i in range(6):
        err = df[f"q{i}"] - df[f"qref{i}"]
        ax.plot(df["t"], err, linewidth=1.4, label=JOINT_NAMES[i])

    ax.axhline(0.0, color="black", linewidth=0.7)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("eq [rad]")
    ax.set_title(title)
    ax.legend(fontsize=8, ncol=3)
    savefig(fig, stem)


def fig_torque_profile(df: pd.DataFrame, title: str, stem: str) -> None:
    """τ(t) for the 6 joints on one plot."""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for i in range(6):
        ax.plot(df["t"], df[f"tau{i}"], label=JOINT_NAMES[i])
    ax.set_xlabel("t [s]")
    ax.set_ylabel("τ [N·m]")
    ax.legend(fontsize=8, ncol=3)
    ax.set_title(title)
    savefig(fig, stem)


# ────────────────────────────────────────────────────────────────────
#               FIGURE BUILDERS — SWEEPS
# ────────────────────────────────────────────────────────────────────

def fig_sweep_error_grid(runs: list, stem: str, title: str,
                         sort_key=None) -> None:
    """One subplot per joint, each overlays q-qref for every run in the sweep."""
    if sort_key:
        runs = sorted(runs, key=sort_key)

    fig, axes = plt.subplots(3, 2, figsize=(11, 8.5), sharex=True)
    for i in range(6):
        ax = axes[i // 2, i % 2]
        for summary in runs:
            df = load_run(summary["csv"])
            err = df[f"q{i}"] - df[f"qref{i}"]
            ax.plot(df["t"], err, label=summary["label"], linewidth=1.1)
        ax.set_title(JOINT_NAMES[i])
        ax.set_ylabel("eq [rad]")
        ax.axhline(0, color="black", linewidth=0.4)
        if i == 0:
            ax.legend(fontsize=7, loc="best")

    for j in range(2):
        axes[-1, j].set_xlabel("t [s]")
    fig.suptitle(title, fontsize=12)
    savefig(fig, stem)


def fig_sweep_tradeoff(runs: list, stem: str, title: str,
                       x_key: str, x_label: str) -> None:
    """Bar/scatter of RMS error vs control energy for a sweep."""
    # sort by x_key numerically when possible
    try:
        runs = sorted(runs, key=lambda r: r[x_key])
    except Exception:
        pass

    labels  = [r["label"] for r in runs]
    rms     = [r["rms_eq"] for r in runs]
    energy  = [r["u_energy"] for r in runs]
    itae    = [r["itae"] for r in runs]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.3))
    x = np.arange(len(labels))

    ax1.bar(x, rms, color="tab:blue")
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=30, ha="right", fontsize=9)
    ax1.set_ylabel("RMS eq [rad]")
    ax1.set_title("Tracking error")

    ax2.bar(x, energy, color="tab:orange")
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=30, ha="right", fontsize=9)
    ax2.set_ylabel(r"$\int \|\tau\|^2\, dt$  [N²·m²·s]")
    ax2.set_title("Control energy")

    fig.suptitle(title, fontsize=12)
    savefig(fig, stem)

    # Also a 2-D trade-off: rms vs energy (each run a point, labelled)
    fig2, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(energy, rms, s=60, color="tab:purple", zorder=3)
    for i, lbl in enumerate(labels):
        ax.annotate(lbl, (energy[i], rms[i]),
                    textcoords="offset points", xytext=(6, 4), fontsize=9)
    ax.set_xlabel(r"Control energy  $\int \|\tau\|^2\, dt$")
    ax.set_ylabel("RMS eq [rad]")
    ax.set_title(f"{title} — trade-off curve")
    savefig(fig2, stem + "_tradeoff")


def fig_exp4_accuracy_bars(runs: list, stem: str, title: str) -> None:
    """2x2 bar grid: rms_eq / max_abs_eq / eq_final / u_energy across targets.

    Unlike fig_sweep_tradeoff, the 3 runs do not share a numeric sweep
    variable, so we just plot them in insertion order and show all four
    accuracy/effort metrics side by side. This is the defence-facing
    figure that demonstrates the LQR behaves consistently across
    different goal poses.
    """
    labels     = [r["label"] for r in runs]
    rms        = [r["rms_eq"]     for r in runs]
    max_eq     = [r["max_abs_eq"] for r in runs]
    eq_fin     = [r["eq_final"]   for r in runs]
    energy     = [r["u_energy"]   for r in runs]
    x = np.arange(len(labels))

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5))

    axes[0, 0].bar(x, rms,    color="tab:blue")
    axes[0, 0].set_title("RMS eq [rad]")
    axes[0, 1].bar(x, max_eq, color="tab:red")
    axes[0, 1].set_title("max |eq| [rad]")
    axes[1, 0].bar(x, eq_fin, color="tab:green")
    axes[1, 0].set_title("eq final [rad]")
    axes[1, 1].bar(x, energy, color="tab:orange")
    axes[1, 1].set_title(r"$\int \|\tau\|^2\, dt$  [N²·m²·s]")

    for ax in axes.flat:
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha="right", fontsize=9)

    fig.suptitle(title, fontsize=12)
    savefig(fig, stem)


# ────────────────────────────────────────────────────────────────────
#                        MAIN
# ────────────────────────────────────────────────────────────────────

def main() -> int:
    summary_path = RESULTS_DIR / "summary.json"
    if not summary_path.exists():
        print(f"ERROR: {summary_path} not found. Run run_experiments.py first.",
              file=sys.stderr)
        return 1

    with summary_path.open() as f:
        summaries = json.load(f)

    if not summaries:
        print("ERROR: summary.json is empty.", file=sys.stderr)
        return 1

    FIG_DIR.mkdir(parents=True, exist_ok=True)

    # ── Exp 1: single run, full tracking + torque profile ──────────
    # ── Exp 1: single run, tracking + error + torque ───────────────
    exp1 = [s for s in summaries if s["experiment"] == "exp1"]
    if exp1:
        print("\n[Exp 1] Baseline tracking")
        s = exp1[0]
        df = load_run(s["csv"])

        fig_tracking_single(
            df,
            f"Exp 1 — Baseline LQR tracking vs reference ({s['label']})",
            "exp1_tracking_single"
        )

        fig_error_single(
            df,
            f"Exp 1 — Tracking error ({s['label']})",
            "exp1_error_single"
        )

        fig_torque_profile(
            df,
            f"Exp 1 — Control torques ({s['label']})",
            "exp1_torques"
        )

    # ── Exp 2: Q/R sweep ────────────────────────────────────────────
    exp2 = [s for s in summaries if s["experiment"] == "exp2"]
    if exp2:
        print("\n[Exp 2] Q/R sweep")
        fig_sweep_error_grid(exp2, "exp2_errors",
                             "Exp 2 — Joint tracking error for Q/R sweep",
                             sort_key=lambda r: r["w_wu"])
        fig_sweep_tradeoff(exp2, "exp2_bars",
                           "Exp 2 — Q/R trade-off",
                           x_key="w_wu", x_label="wu")

    # ── Exp 3: horizon sweep ────────────────────────────────────────
    exp3 = [s for s in summaries if s["experiment"] == "exp3"]
    if exp3:
        print("\n[Exp 3] Horizon sweep")
        fig_sweep_error_grid(exp3, "exp3_errors",
                             "Exp 3 — Joint tracking error vs horizon N",
                             sort_key=lambda r: r["horizonN"])
        fig_sweep_tradeoff(exp3, "exp3_bars",
                           "Exp 3 — Horizon N",
                           x_key="horizonN", x_label="N")

    # ── Exp 4: multi-target generalisation ─────────────────────────
    exp4 = [s for s in summaries if s["experiment"] == "exp4"]
    if exp4:
        print("\n[Exp 4] Multi-target generalisation")
        # One full tracking grid per target (q vs q_ref, 6 joints).
        for s in exp4:
            df = load_run(s["csv"])
            stem = f"exp4_tracking_{s['name'].replace('exp4_target_', '')}"
            fig_tracking_grid(df,
                              f"Exp 4 — {s['label']} (LQR baseline)",
                              stem)
        # Per-joint error overlay across the 3 targets.
        fig_sweep_error_grid(exp4, "exp4_errors",
                             "Exp 4 — Joint tracking error per target",
                             sort_key=lambda r: r["name"])
        # 2x2 bar chart of accuracy + effort metrics per target.
        fig_exp4_accuracy_bars(exp4, "exp4_bars",
                               "Exp 4 — LQR metrics across different q_target")

    # ── Combined metrics table ─────────────────────────────────────
    print("\nWriting summary table")
    df = pd.DataFrame(summaries)
    # keep useful columns in a sensible order
    cols = [c for c in [
        "experiment", "name", "label",
        "horizonN", "w_wq", "w_wdq", "w_wu", "w_wqN", "w_wdqN",
        "rms_eq", "rms_edq", "max_abs_eq", "eq_final", "u_energy", "itae",
        "n_samples", "T", "dt",
    ] if c in df.columns]
    df = df[cols]

    xlsx_path = RESULTS_DIR / "summary.xlsx"
    try:
        df.to_excel(xlsx_path, index=False, float_format="%.6f")
        print(f"  saved {xlsx_path.name}")
    except Exception as e:
        # openpyxl missing → fall back to CSV
        fallback = RESULTS_DIR / "summary.csv"
        df.to_csv(fallback, index=False, float_format="%.6f")
        print(f"  xlsx write failed ({e}), wrote {fallback.name} instead")

    # Also a nicer console print
    print("\nSummary table:")
    with pd.option_context("display.max_columns", None,
                            "display.width", 160,
                            "display.float_format", lambda v: f"{v:.4f}"):
        print(df.to_string(index=False))

    print(f"\nFigures → {FIG_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
