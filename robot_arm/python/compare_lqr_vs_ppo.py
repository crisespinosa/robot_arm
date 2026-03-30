#!/usr/bin/env python3
"""
Comparative evaluation: Fixed LQR weights vs PPO-adaptive weights.

Generates publication-quality plots for thesis showing:
  1. Tracking error (eq_rms) over time
  2. Control effort (torque energy) over time
  3. Smoothness (jerk / du_energy) over time
  4. Box plots of final metrics across episodes
  5. Per-joint error comparison
  6. Statistical summary table

Requirements:
  pip install stable-baselines3 gymnasium numpy requests matplotlib

Usage:
  1. Start backend:  cd build && ./robot_arm
  2. Run comparison:  python compare_lqr_vs_ppo.py --model checkpoints/ur5e_ppo_final.zip
"""

import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving figures
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from stable_baselines3 import PPO
from env_backend_robot_arm import RobotArmBackendEnv


# ============================================================
# Data collection
# ============================================================

def run_episode(backend, q_start, q_target, T, dt, mode, N,
                model=None, fixed_action=None):
    """
    Run one episode and collect per-step metrics.

    If model is provided, use PPO policy for actions.
    If fixed_action is provided, send that action every step (fixed LQR).
    If neither, send no action (backend uses default weights).
    """
    obs, reset_info = backend.reset(q_start, q_target, T=T, dt=dt, mode=mode, N=N)

    trajectory = {
        "t": [],
        "eq_rms": [],
        "edq_rms": [],
        "u_energy": [],
        "du_energy": [],
        "reward": [],
        "tau_cmd": [],
        "q_cmd": [],
        "dq_cmd": [],
        "weights": [],
    }

    done = False
    total_reward = 0.0

    while not done:
        if model is not None:
            action, _ = model.predict(obs, deterministic=True)
        elif fixed_action is not None:
            action = fixed_action
        else:
            action = np.zeros(5, dtype=np.float32)

        obs, reward, done, info = backend.step(action)
        total_reward += reward

        trajectory["t"].append(info.get("t", 0))
        trajectory["eq_rms"].append(info.get("eq_rms", 0))
        trajectory["edq_rms"].append(info.get("edq_rms", 0))
        trajectory["u_energy"].append(info.get("u_energy", 0))
        trajectory["du_energy"].append(info.get("du_energy", 0))
        trajectory["reward"].append(reward)

        # Per-joint data
        tau = info.get("tau_cmd", {})
        if isinstance(tau, dict):
            trajectory["tau_cmd"].append([tau.get(f"q{i}", 0) for i in range(6)])
        elif isinstance(tau, list):
            trajectory["tau_cmd"].append(tau)

        q = info.get("q_cmd", {})
        if isinstance(q, dict):
            trajectory["q_cmd"].append([q.get(f"q{i}", 0) for i in range(6)])
        elif isinstance(q, list):
            trajectory["q_cmd"].append(q)

        dq = info.get("dq_cmd", {})
        if isinstance(dq, dict):
            trajectory["dq_cmd"].append([dq.get(f"q{i}", 0) for i in range(6)])
        elif isinstance(dq, list):
            trajectory["dq_cmd"].append(dq)

        w = info.get("weights_used", {})
        if isinstance(w, dict):
            trajectory["weights"].append([
                w.get("wq", 0), w.get("wdq", 0), w.get("wu", 0),
                w.get("wqN", 0), w.get("wdqN", 0)
            ])

    # Convert to arrays
    for key in trajectory:
        trajectory[key] = np.array(trajectory[key])

    trajectory["total_reward"] = total_reward
    trajectory["final_eq_rms"] = trajectory["eq_rms"][-1] if len(trajectory["eq_rms"]) > 0 else np.inf
    trajectory["success"] = info.get("success", False)
    trajectory["mean_u_energy"] = np.mean(trajectory["u_energy"]) if len(trajectory["u_energy"]) > 0 else 0
    trajectory["mean_du_energy"] = np.mean(trajectory["du_energy"]) if len(trajectory["du_energy"]) > 0 else 0

    return trajectory


def run_comparison(backend, model, n_episodes, T, dt, mode, N, seed=42):
    """Run episodes with both fixed LQR and PPO, using same start/target pairs."""
    rng = np.random.RandomState(seed)

    q_ranges = [
        (-1.5, 1.5), (-1.5, 0.5), (-2.0, 0.5),
        (-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5),
    ]

    fixed_results = []
    ppo_results = []

    # Fixed action = zeros means backend uses default LQR weights
    fixed_action = np.zeros(5, dtype=np.float32)

    for ep in range(n_episodes):
        # Generate random start/target pair
        q_start = np.zeros(6, dtype=np.float64)
        if rng.random() > 0.3:
            for i, (lo, hi) in enumerate(q_ranges):
                q_start[i] = rng.uniform(lo * 0.5, hi * 0.5)

        q_target = np.zeros(6, dtype=np.float64)
        for i, (lo, hi) in enumerate(q_ranges):
            q_target[i] = rng.uniform(lo, hi)

        while np.linalg.norm(q_target - q_start) < 0.1:
            for i, (lo, hi) in enumerate(q_ranges):
                q_target[i] = rng.uniform(lo, hi)

        print(f"  Episode {ep+1}/{n_episodes}...", end=" ", flush=True)

        # Run with fixed LQR weights
        traj_fixed = run_episode(backend, q_start, q_target, T, dt, mode, N,
                                 fixed_action=fixed_action)
        fixed_results.append(traj_fixed)

        # Run with PPO (same start/target)
        traj_ppo = run_episode(backend, q_start, q_target, T, dt, mode, N,
                               model=model)
        ppo_results.append(traj_ppo)

        print(f"Fixed: eq={traj_fixed['final_eq_rms']:.4f}, "
              f"PPO: eq={traj_ppo['final_eq_rms']:.4f}")

    return fixed_results, ppo_results


# ============================================================
# Plotting functions
# ============================================================

def setup_thesis_style():
    """Configure matplotlib for thesis-quality figures."""
    plt.rcParams.update({
        'font.family': 'serif',
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10,
        'figure.dpi': 150,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'axes.grid': True,
        'grid.alpha': 0.3,
        'lines.linewidth': 1.5,
    })


def plot_time_series_comparison(fixed_results, ppo_results, output_dir):
    """Plot 1: Tracking error, control effort, and smoothness over time."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    metrics = [
        ("eq_rms", "Tracking Error (RMS)", "rad"),
        ("u_energy", "Control Energy", "Nm²"),
        ("du_energy", "Torque Variation (Smoothness)", "Nm²/s²"),
    ]

    colors_fixed = '#d62728'  # red
    colors_ppo = '#1f77b4'    # blue

    for ax, (key, title, unit) in zip(axes, metrics):
        # Collect all trajectories and compute mean ± std
        max_len = max(
            max(len(r[key]) for r in fixed_results),
            max(len(r[key]) for r in ppo_results)
        )

        fixed_matrix = np.full((len(fixed_results), max_len), np.nan)
        ppo_matrix = np.full((len(ppo_results), max_len), np.nan)

        for i, r in enumerate(fixed_results):
            fixed_matrix[i, :len(r[key])] = r[key]
        for i, r in enumerate(ppo_results):
            ppo_matrix[i, :len(r[key])] = r[key]

        t = np.arange(max_len) * 0.02  # dt = 0.02

        fixed_mean = np.nanmean(fixed_matrix, axis=0)
        fixed_std = np.nanstd(fixed_matrix, axis=0)
        ppo_mean = np.nanmean(ppo_matrix, axis=0)
        ppo_std = np.nanstd(ppo_matrix, axis=0)

        ax.plot(t, fixed_mean, color=colors_fixed, label='Fixed LQR', linewidth=1.8)
        ax.fill_between(t, fixed_mean - fixed_std, fixed_mean + fixed_std,
                        color=colors_fixed, alpha=0.15)

        ax.plot(t, ppo_mean, color=colors_ppo, label='PPO Adaptive', linewidth=1.8)
        ax.fill_between(t, ppo_mean - ppo_std, ppo_mean + ppo_std,
                        color=colors_ppo, alpha=0.15)

        ax.set_ylabel(f"{title}\n[{unit}]")
        ax.legend(loc='upper right')

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Time Series Comparison: Fixed LQR vs PPO Adaptive", fontsize=14, y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "01_time_series_comparison.png"))
    fig.savefig(os.path.join(output_dir, "01_time_series_comparison.pdf"))
    plt.close(fig)
    print("  -> 01_time_series_comparison.png/pdf")


def plot_box_comparison(fixed_results, ppo_results, output_dir):
    """Plot 2: Box plots comparing final metrics."""
    fig, axes = plt.subplots(1, 4, figsize=(14, 4.5))

    metrics = [
        ("final_eq_rms", "Final Error\n(RMS rad)", 1.0),
        ("total_reward", "Total\nReward", 1.0),
        ("mean_u_energy", "Mean Control\nEnergy (Nm²)", 1.0),
        ("mean_du_energy", "Mean Torque\nVariation (Nm²/s²)", 1.0),
    ]

    for ax, (key, label, scale) in zip(axes, metrics):
        fixed_vals = [r[key] * scale for r in fixed_results]
        ppo_vals = [r[key] * scale for r in ppo_results]

        bp = ax.boxplot(
            [fixed_vals, ppo_vals],
            labels=["Fixed\nLQR", "PPO\nAdaptive"],
            patch_artist=True,
            widths=0.5,
            medianprops=dict(color='black', linewidth=1.5),
        )
        bp['boxes'][0].set_facecolor('#ffcccc')
        bp['boxes'][0].set_edgecolor('#d62728')
        bp['boxes'][1].set_facecolor('#cce5ff')
        bp['boxes'][1].set_edgecolor('#1f77b4')
        ax.set_ylabel(label)

    fig.suptitle("Metric Distribution: Fixed LQR vs PPO Adaptive",
                 fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "02_box_comparison.png"))
    fig.savefig(os.path.join(output_dir, "02_box_comparison.pdf"))
    plt.close(fig)
    print("  -> 02_box_comparison.png/pdf")


def plot_single_trajectory(fixed_results, ppo_results, output_dir, episode_idx=0):
    """Plot 3: Detailed single trajectory comparison (best PPO episode)."""
    # Pick the episode with highest PPO advantage
    advantages = []
    for i in range(len(fixed_results)):
        adv = fixed_results[i]["final_eq_rms"] - ppo_results[i]["final_eq_rms"]
        advantages.append(adv)
    episode_idx = int(np.argmax(advantages))

    rf = fixed_results[episode_idx]
    rp = ppo_results[episode_idx]

    n_steps_f = len(rf["t"])
    n_steps_p = len(rp["t"])
    t_f = np.arange(n_steps_f) * 0.02
    t_p = np.arange(n_steps_p) * 0.02

    fig = plt.figure(figsize=(12, 10))
    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.3)

    # Tracking error
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t_f, rf["eq_rms"], color='#d62728', label='Fixed LQR', linewidth=1.8)
    ax1.plot(t_p, rp["eq_rms"], color='#1f77b4', label='PPO', linewidth=1.8)
    ax1.set_ylabel("Position Error\n(RMS rad)")
    ax1.set_title("Tracking Error")
    ax1.legend()

    # Velocity error
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t_f, rf["edq_rms"], color='#d62728', label='Fixed LQR', linewidth=1.8)
    ax2.plot(t_p, rp["edq_rms"], color='#1f77b4', label='PPO', linewidth=1.8)
    ax2.set_ylabel("Velocity Error\n(RMS rad/s)")
    ax2.set_title("Velocity Error")
    ax2.legend()

    # Torque energy
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t_f, rf["u_energy"], color='#d62728', label='Fixed LQR', linewidth=1.8)
    ax3.plot(t_p, rp["u_energy"], color='#1f77b4', label='PPO', linewidth=1.8)
    ax3.set_ylabel("Control Energy\n(Nm²)")
    ax3.set_title("Control Effort")
    ax3.legend()

    # Smoothness
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t_f, rf["du_energy"], color='#d62728', label='Fixed LQR', linewidth=1.8)
    ax4.plot(t_p, rp["du_energy"], color='#1f77b4', label='PPO', linewidth=1.8)
    ax4.set_ylabel("Δτ Energy\n(Nm²/s²)")
    ax4.set_title("Motion Smoothness")
    ax4.legend()

    # Per-joint torques (PPO)
    if len(rp["tau_cmd"]) > 0 and rp["tau_cmd"].ndim == 2:
        ax5 = fig.add_subplot(gs[2, 0])
        joint_names = ["Base", "Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
        for j in range(min(6, rp["tau_cmd"].shape[1])):
            ax5.plot(t_p[:len(rp["tau_cmd"])], rp["tau_cmd"][:, j],
                     label=joint_names[j], linewidth=1.2)
        ax5.set_ylabel("Torque (Nm)")
        ax5.set_xlabel("Time [s]")
        ax5.set_title("Per-Joint Torques (PPO)")
        ax5.legend(fontsize=8, ncol=2)

    # PPO adaptive weights over time
    if len(rp["weights"]) > 0 and rp["weights"].ndim == 2:
        ax6 = fig.add_subplot(gs[2, 1])
        w_names = ["wq", "wdq", "wu", "wqN", "wdqN"]
        for j in range(min(5, rp["weights"].shape[1])):
            ax6.plot(t_p[:len(rp["weights"])], rp["weights"][:, j],
                     label=w_names[j], linewidth=1.2)
        ax6.set_ylabel("LQR Weight")
        ax6.set_xlabel("Time [s]")
        ax6.set_title("PPO Adaptive LQR Weights")
        ax6.legend(fontsize=8, ncol=2)

    fig.suptitle(f"Detailed Trajectory — Episode {episode_idx+1}\n"
                 f"(highest PPO advantage)", fontsize=14)
    fig.savefig(os.path.join(output_dir, "03_single_trajectory_detail.png"))
    fig.savefig(os.path.join(output_dir, "03_single_trajectory_detail.pdf"))
    plt.close(fig)
    print("  -> 03_single_trajectory_detail.png/pdf")


def plot_success_and_improvement(fixed_results, ppo_results, output_dir):
    """Plot 4: Per-episode improvement bar chart."""
    n = len(fixed_results)
    fixed_errors = [r["final_eq_rms"] for r in fixed_results]
    ppo_errors = [r["final_eq_rms"] for r in ppo_results]
    improvement_pct = [
        (fe - pe) / fe * 100 if fe > 1e-9 else 0
        for fe, pe in zip(fixed_errors, ppo_errors)
    ]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7))

    # Error final per episode
    x = np.arange(n)
    width = 0.35
    axes[0].bar(x - width/2, fixed_errors, width, color='#d62728',
                alpha=0.8, label='Fixed LQR')
    axes[0].bar(x + width/2, ppo_errors, width, color='#1f77b4',
                alpha=0.8, label='PPO Adaptive')
    axes[0].set_ylabel("Final Error (RMS rad)")
    axes[0].set_title("Final Error per Episode")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([f"Ep{i+1}" for i in range(n)], fontsize=8)
    axes[0].legend()
    axes[0].axhline(y=0.03, color='green', linestyle='--', linewidth=1,
                    label='Success Threshold (0.03 rad)')
    axes[0].legend()

    # Percentage improvement
    colors = ['#2ca02c' if v > 0 else '#d62728' for v in improvement_pct]
    axes[1].bar(x, improvement_pct, color=colors, alpha=0.8)
    axes[1].set_ylabel("PPO Improvement (%)")
    axes[1].set_xlabel("Episode")
    axes[1].set_title("Percentage Error Reduction with PPO")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([f"Ep{i+1}" for i in range(n)], fontsize=8)
    axes[1].axhline(y=0, color='black', linewidth=0.8)

    avg_improvement = np.mean(improvement_pct)
    axes[1].axhline(y=avg_improvement, color='blue', linestyle='--', linewidth=1.2,
                    label=f'Average: {avg_improvement:.1f}%')
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "04_per_episode_improvement.png"))
    fig.savefig(os.path.join(output_dir, "04_per_episode_improvement.pdf"))
    plt.close(fig)
    print("  -> 04_per_episode_improvement.png/pdf")


def print_summary_table(fixed_results, ppo_results):
    """Print statistical summary for the thesis."""
    metrics = {
        "Final Error (RMS rad)": ("final_eq_rms", "{:.4f}"),
        "Total Reward": ("total_reward", "{:.2f}"),
        "Mean Control Energy": ("mean_u_energy", "{:.2f}"),
        "Mean Torque Variation": ("mean_du_energy", "{:.4f}"),
        "Success Rate": ("success", "{:.1%}"),
    }

    print("\n" + "=" * 75)
    print(f"{'Metric':<28} {'Fixed LQR':>14} {'PPO Adaptive':>16} {'Improvement':>12}")
    print("=" * 75)

    for name, (key, fmt) in metrics.items():
        if key == "success":
            fixed_val = np.mean([1.0 if r[key] else 0.0 for r in fixed_results])
            ppo_val = np.mean([1.0 if r[key] else 0.0 for r in ppo_results])
        else:
            fixed_val = np.mean([r[key] for r in fixed_results])
            ppo_val = np.mean([r[key] for r in ppo_results])

        if key == "success":
            improvement = f"{(ppo_val - fixed_val)*100:+.1f}pp"
        elif fixed_val != 0:
            improvement = f"{(fixed_val - ppo_val) / abs(fixed_val) * 100:+.1f}%"
        else:
            improvement = "N/A"

        fixed_str = fmt.format(fixed_val)
        ppo_str = fmt.format(ppo_val)
        print(f"  {name:<26} {fixed_str:>14} {ppo_str:>16} {improvement:>12}")

    print("=" * 75)

    # Standard deviations
    fixed_eq = [r["final_eq_rms"] for r in fixed_results]
    ppo_eq = [r["final_eq_rms"] for r in ppo_results]
    print(f"\n  Final error σ:  LQR={np.std(fixed_eq):.4f}  PPO={np.std(ppo_eq):.4f}")
    print(f"  Episodes:       {len(fixed_results)}")


def save_csv_results(fixed_results, ppo_results, output_dir):
    """Save raw results as CSV for use in LaTeX tables."""
    with open(os.path.join(output_dir, "results_comparison.csv"), "w") as f:
        f.write("episode,method,final_eq_rms,total_reward,mean_u_energy,"
                "mean_du_energy,success\n")
        for i, (rf, rp) in enumerate(zip(fixed_results, ppo_results)):
            f.write(f"{i+1},fixed_lqr,{rf['final_eq_rms']:.6f},"
                    f"{rf['total_reward']:.4f},{rf['mean_u_energy']:.6f},"
                    f"{rf['mean_du_energy']:.6f},{rf['success']}\n")
            f.write(f"{i+1},ppo_adaptive,{rp['final_eq_rms']:.6f},"
                    f"{rp['total_reward']:.4f},{rp['mean_u_energy']:.6f},"
                    f"{rp['mean_du_energy']:.6f},{rp['success']}\n")
    print(f"  -> results_comparison.csv")


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Compare fixed LQR vs PPO adaptive control for UR5e"
    )
    parser.add_argument("--model", type=str, required=True,
                        help="Path to trained PPO model (.zip)")
    parser.add_argument("--url", type=str, default="http://127.0.0.1:8848",
                        help="Backend URL")
    parser.add_argument("--episodes", type=int, default=30,
                        help="Number of comparison episodes")
    parser.add_argument("--T", type=float, default=1.5,
                        help="Trajectory duration (s)")
    parser.add_argument("--dt", type=float, default=0.02,
                        help="Timestep (s)")
    parser.add_argument("--mode", type=str, default="mpc_lite")
    parser.add_argument("--N", type=int, default=20)
    parser.add_argument("--output-dir", type=str, default="thesis_plots",
                        help="Directory for output figures")
    parser.add_argument("--seed", type=int, default=42)

    args = parser.parse_args()

    setup_thesis_style()

    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 60)
    print("  Comparison: Fixed LQR vs PPO Adaptive")
    print("=" * 60)
    print(f"  PPO Model:  {args.model}")
    print(f"  Episodes:   {args.episodes}")
    print(f"  Output:     {args.output_dir}/")
    print("=" * 60)

    # Load model
    print(f"\nLoading PPO model...")
    model = PPO.load(args.model)

    # Backend
    backend = RobotArmBackendEnv(base_url=args.url)

    # Run comparison
    print(f"\nRunning {args.episodes} comparison episodes...\n")
    fixed_results, ppo_results = run_comparison(
        backend, model, args.episodes, args.T, args.dt, args.mode, args.N, args.seed
    )

    # Print summary
    print_summary_table(fixed_results, ppo_results)

    # Generate plots
    print(f"\nGenerating plots in {args.output_dir}/...\n")
    plot_time_series_comparison(fixed_results, ppo_results, args.output_dir)
    plot_box_comparison(fixed_results, ppo_results, args.output_dir)
    plot_single_trajectory(fixed_results, ppo_results, args.output_dir)
    plot_success_and_improvement(fixed_results, ppo_results, args.output_dir)
    save_csv_results(fixed_results, ppo_results, args.output_dir)

    print(f"\nDone! All results saved to: {args.output_dir}/")
    print("Archivos generados:")
    print("  - 01_time_series_comparison.png/pdf")
    print("  - 02_box_comparison.png/pdf")
    print("  - 03_single_trajectory_detail.png/pdf")
    print("  - 04_per_episode_improvement.png/pdf")
    print("  - results_comparison.csv")


if __name__ == "__main__":
    main()
