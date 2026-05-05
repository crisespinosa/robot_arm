"""
LQR experiments for the UR5e thesis defense (Stage 1).

Runs four experiments against the Drogon C++ backend on port 8848:
  Exp 1: basic tracking (baseline weights, baseline horizon)
  Exp 2: Q/R weight sweep (state-vs-control trade-off)
  Exp 3: prediction horizon sweep (N = 5, 10, 20, 40, 80)
  Exp 4: generalisation across multiple q_target configurations
         (same LQR, same weights, same N, only the goal pose varies)

Each run:
  1. POST /arm/set_reference  with q_start, q_target, T, dt
  2. Loop /arm/step for t in [0, T] stepping dt
  3. Log every step to a CSV in ./results/

No Unity. Pure HTTP against the C++ backend.

Usage:
  # 1) start the backend (in a separate terminal)
  cd robot_arm/cmake-build-debug && ./robot_arm_app

  # 2) run the experiments
  pip install requests numpy pandas
  python run_experiments.py

  # 3) generate plots + summary table
  python analyze.py
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import requests

# ────────────────────────────────────────────────────────────────────
#                        CONSTANTS / DEFAULTS
# ────────────────────────────────────────────────────────────────────

BACKEND_URL = "http://127.0.0.1:8848"
RESULTS_DIR = Path(__file__).parent / "results"

# Baseline motion: "reach" task. Not trivial (all 6 joints move),
# not extreme (no limits hit). Good for defense demos.
Q_START = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Q_TARGET = [0.8, -0.6, 0.7, -0.4, 0.5, -0.3]

T_TRAJ = 1.5    # trajectory duration (s)
DT = 0.02       # integration step (s) → 75 steps per episode

# Baseline LQR weights
W_BASELINE = dict(wq=30.0, wdq=2.0, wu=0.1, wqN=30.0, wdqN=2.0)

# Baseline horizon for experiments 1 and 2
N_BASELINE = 20

# ────────────────────────────────────────────────────────────────────
#                        EXPERIMENT CONFIGS
# ────────────────────────────────────────────────────────────────────

@dataclass
class RunConfig:
    """One LQR rollout: everything the backend needs to run a single episode."""
    name: str                       # unique identifier for CSV filename
    experiment: str                 # exp1 / exp2 / exp3 / exp4 (grouping for analyze.py)
    label: str                      # human-readable label for plots
    weights: Dict[str, float] = field(default_factory=lambda: dict(W_BASELINE))
    horizonN: int = N_BASELINE
    T: float = T_TRAJ
    dt: float = DT
    q_start: List[float] = field(default_factory=lambda: list(Q_START))
    q_target: List[float] = field(default_factory=lambda: list(Q_TARGET))
    u_max: float = 8.0
    mode: str = "lqr"
    use_kalman: bool = False


def build_configs() -> List[RunConfig]:
    configs: List[RunConfig] = []

    # ─── Exp 1: baseline tracking ──────────────────────────────────
    configs.append(RunConfig(
        name="exp1_baseline",
        experiment="exp1",
        label="Baseline (wq=30, wu=0.1, N=20)",
    ))

    # ─── Exp 2: Q/R weight sweep ───────────────────────────────────
    # Keep wq fixed, vary wu to sweep the state-vs-control trade-off.
    # Also keep wdq, wqN, wdqN consistent so the sweep axis is clean.
    #
    # Effective ratio is wq / wu:
    #     very soft:  wq=30,  wu=1.0   (ratio  30) → smooth, slow
    #     soft:       wq=30,  wu=0.3   (ratio 100)
    #     balanced:   wq=30,  wu=0.1   (ratio 300) — baseline
    #     aggressive: wq=30,  wu=0.03  (ratio 1000)
    #     very aggr:  wq=30,  wu=0.01  (ratio 3000) → fast, high torque
    for label_suffix, wu in [
        ("very_soft",  1.0),
        ("soft",       0.3),
        ("balanced",   0.1),     # same as baseline
        ("aggressive", 0.03),
        ("very_aggr",  0.01),
    ]:
        w = dict(W_BASELINE)
        w["wu"] = wu
        configs.append(RunConfig(
            name=f"exp2_wu_{label_suffix}",
            experiment="exp2",
            label=f"wu={wu:g}",
            weights=w,
        ))

    # ─── Exp 3: horizon sweep ──────────────────────────────────────
    for N in [5, 10, 20, 40, 80]:
        configs.append(RunConfig(
            name=f"exp3_N_{N}",
            experiment="exp3",
            label=f"N={N}",
            horizonN=N,
        ))

    # ─── Exp 4: generalisation across q_target ─────────────────────
    # Same LQR, same weights, same horizon. Only q_target changes.
    # Demonstrates that the regulator is not tuned to a single
    # trajectory but handles different goal poses of the manipulator.
    #   A (moderado):   half-amplitude baseline, smooth
    #   B (asimétrico): joints 2/4/6 frozen, joints 1/3/5 forced
    #   C (amplio):     1.5× baseline amplitude, stresses u_max
    exp4_targets = [
        ("A_moderado",   "Target A (moderate)",   [0.4, -0.3, 0.4, -0.2, 0.3, -0.15]),
        ("B_asimetrico", "Target B (asymmetric)", [1.0,  0.0, 0.8,  0.0, 0.4,  0.0 ]),
        ("C_amplio",     "Target C (wide)",       [1.2, -0.9, 1.0, -0.7, 0.8, -0.5 ]),
    ]
    for tag, lbl, q_tgt in exp4_targets:
        configs.append(RunConfig(
            name=f"exp4_target_{tag}",
            experiment="exp4",
            label=lbl,
            q_target=list(q_tgt),
        ))

    return configs


# ────────────────────────────────────────────────────────────────────
#                        BACKEND HTTP CLIENT
# ────────────────────────────────────────────────────────────────────

class BackendClient:
    def __init__(self, base_url: str = BACKEND_URL, timeout: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.sess = requests.Session()

    def ping(self) -> bool:
        """Returns True if the backend responds to anything on 8848."""
        try:
            r = self.sess.post(f"{self.base_url}/arm/plan_minjerk_q",
                               json={"q_target": [0] * 6, "T": 0.5, "dt": 0.1},
                               timeout=self.timeout)
            return r.status_code == 200
        except Exception:
            return False

    def set_reference(self, cfg: RunConfig) -> dict:
        body = {
            "q_start": cfg.q_start,
            "q_target": cfg.q_target,
            "T": cfg.T,
            "dt": cfg.dt,
        }
        r = self.sess.post(f"{self.base_url}/arm/set_reference",
                           json=body, timeout=self.timeout)
        r.raise_for_status()
        return r.json()

    def step(self, q: Sequence[float], dq: Sequence[float],
             t: float, cfg: RunConfig) -> dict:
        body = {
            "q": list(q),
            "dq": list(dq),
            "t": t,
            "dt": cfg.dt,
            "weights": cfg.weights,
            "u_max": cfg.u_max,
            "mode": cfg.mode,
            "N": cfg.horizonN,
            "use_kalman": cfg.use_kalman,
        }
        r = self.sess.post(f"{self.base_url}/arm/step",
                           json=body, timeout=self.timeout)
        r.raise_for_status()
        return r.json()


# ────────────────────────────────────────────────────────────────────
#                        RUN A SINGLE CONFIG
# ────────────────────────────────────────────────────────────────────

CSV_HEADER = [
    "t", "step",
    # measured / plant state
    "q0", "q1", "q2", "q3", "q4", "q5",
    "dq0", "dq1", "dq2", "dq3", "dq4", "dq5",
    # reference
    "qref0", "qref1", "qref2", "qref3", "qref4", "qref5",
    "dqref0", "dqref1", "dqref2", "dqref3", "dqref4", "dqref5",
    # control
    "tau0", "tau1", "tau2", "tau3", "tau4", "tau5",
    # backend-reported
    "eq_rms_step", "edq_rms_step",
]


def run_single(cfg: RunConfig, client: BackendClient,
               results_dir: Path) -> dict:
    """Run one config end-to-end, save CSV. Returns per-run summary metrics."""
    print(f"\n▶ {cfg.name}  [{cfg.label}]")
    client.set_reference(cfg)

    # Plant state (what we send as "measured q"). Closed-loop: we
    # use the q_cmd returned by the backend as our next measurement.
    # That models "the plant integrated under the computed torque".
    q = list(cfg.q_start)
    dq = [0.0] * 6

    n_steps = int(round(cfg.T / cfg.dt))
    csv_path = results_dir / f"{cfg.name}.csv"

    # Per-run summary accumulators
    sum_eq_sq  = 0.0        # for RMS
    sum_edq_sq = 0.0
    sum_u_sq   = 0.0        # for energy
    sum_itae   = 0.0        # ∫ t · Σ |eq_i| dt
    max_abs_eq = 0.0        # peak tracking error (any joint)
    n_samples  = 0
    eq_final   = 0.0

    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)

        for k in range(n_steps + 1):
            t = k * cfg.dt
            resp = client.step(q, dq, t, cfg)

            q_cmd = resp["q_cmd"]
            dq_cmd = resp["dq_cmd"]
            tau = resp["tau_cmd"]
            q_ref = resp["q_ref"]
            dq_ref = resp["dq_ref"]
            dbg = resp.get("debug", {})

            # per-step errors (against REFERENCE, not target)
            eq = [q[i] - q_ref[i] for i in range(6)]
            edq = [dq[i] - dq_ref[i] for i in range(6)]

            eq_sq_step  = sum(e * e for e in eq)
            edq_sq_step = sum(e * e for e in edq)
            u_sq_step   = sum(u * u for u in tau)
            abs_eq_step = max(abs(e) for e in eq)

            sum_eq_sq  += eq_sq_step
            sum_edq_sq += edq_sq_step
            sum_u_sq   += u_sq_step
            sum_itae   += t * sum(abs(e) for e in eq)
            if abs_eq_step > max_abs_eq:
                max_abs_eq = abs_eq_step
            n_samples += 1
            eq_final = (eq_sq_step / 6.0) ** 0.5

            w.writerow(
                [t, k]
                + list(q) + list(dq)
                + list(q_ref) + list(dq_ref)
                + list(tau)
                + [dbg.get("eq_rms", ""), dbg.get("edq_rms", "")]
            )

            # Plant update: take backend's q_cmd / dq_cmd as new measurement.
            q = list(q_cmd)
            dq = list(dq_cmd)

    # Aggregate metrics for the summary table
    N = float(n_samples)
    dt = cfg.dt
    rms_eq  = (sum_eq_sq / N / 6.0) ** 0.5     # RMS across joints & time
    rms_edq = (sum_edq_sq / N / 6.0) ** 0.5
    u_energy = sum_u_sq * dt                    # ∫‖τ‖² dt
    itae_total = sum_itae * dt                  # discrete ITAE, averaged over joints

    summary = {
        "name": cfg.name,
        "experiment": cfg.experiment,
        "label": cfg.label,
        "T": cfg.T,
        "dt": cfg.dt,
        "horizonN": cfg.horizonN,
        **{f"w_{k}": v for k, v in cfg.weights.items()},
        "n_samples": n_samples,
        "rms_eq": rms_eq,
        "rms_edq": rms_edq,
        "max_abs_eq": max_abs_eq,
        "eq_final": eq_final,
        "u_energy": u_energy,
        "itae": itae_total,
        "csv": csv_path.name,
    }

    print(f"    rms_eq={rms_eq:.4f} rad   u_energy={u_energy:.3f}   "
          f"itae={itae_total:.4f}   max|eq|={max_abs_eq:.4f}")
    return summary


# ────────────────────────────────────────────────────────────────────
#                        MAIN
# ────────────────────────────────────────────────────────────────────

def main() -> int:
    # Use a literal description rather than __doc__.splitlines()[0] so the
    # script still works under `python -OO` / `PYTHONOPTIMIZE=2`, which
    # strip module docstrings and would otherwise make __doc__ None.
    parser = argparse.ArgumentParser(
        description="LQR experiments for the UR5e thesis defense (Stage 1)."
    )
    parser.add_argument("--backend", default=BACKEND_URL,
                        help=f"Backend URL (default: {BACKEND_URL})")
    parser.add_argument("--only", choices=["exp1", "exp2", "exp3", "exp4"],
                        default=None,
                        help="Run only one experiment group")
    parser.add_argument("--results-dir", default=str(RESULTS_DIR),
                        help="Where to write CSV files")
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    client = BackendClient(args.backend)
    if not client.ping():
        print(f"ERROR: backend not reachable at {args.backend}", file=sys.stderr)
        print("  Start the C++ server first:", file=sys.stderr)
        print("  cd robot_arm/cmake-build-debug && ./robot_arm_app", file=sys.stderr)
        return 1

    configs = build_configs()
    if args.only:
        configs = [c for c in configs if c.experiment == args.only]

    print(f"Running {len(configs)} LQR rollouts → {results_dir}")
    t0 = time.time()
    summaries: List[dict] = []
    for cfg in configs:
        try:
            summaries.append(run_single(cfg, client, results_dir))
        except requests.HTTPError as e:
            print(f"  HTTP error: {e}", file=sys.stderr)
            print(f"  Response: {e.response.text if e.response else 'n/a'}",
                  file=sys.stderr)
        except Exception as e:
            print(f"  failed: {e}", file=sys.stderr)

    # Write master summary JSON (analyze.py reads this)
    summary_path = results_dir / "summary.json"
    with summary_path.open("w") as f:
        json.dump(summaries, f, indent=2)

    elapsed = time.time() - t0
    print(f"\nDone. {len(summaries)}/{len(configs)} runs in {elapsed:.1f}s")
    print(f"Summary written to {summary_path}")
    print("Next: python analyze.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
