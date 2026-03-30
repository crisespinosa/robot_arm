#!/usr/bin/env python3
"""
Comparative evaluation with perturbations: Fixed LQR vs PPO adaptive.

Tests robustness under realistic disturbances:
  1. Torque noise (simulates actuator imprecision)
  2. Payload change (simulates picking up an object mid-trajectory)
  3. Sensor noise (simulates encoder noise)
  4. Combined perturbations

This script injects perturbations at the Python level by modifying
the actions/observations, simulating real-world conditions without
needing to recompile the C++ backend.

Usage:
  python compare_with_perturbations.py --model-v1 checkpoints/ur5e_ppo_final.zip \
                                        --model-v2 checkpoints_v2/ur5e_ppo_v2_final.zip \
                                        --episodes 30
"""

import argparse
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from env_backend_robot_arm import RobotArmBackendEnv


# ============================================================
# Perturbation functions
# ============================================================

class Perturbation:
    """Base class for perturbations."""
    def __init__(self, name, label):
        self.name = name
        self.label = label

    def perturb_action(self, action, step, total_steps):
        return action

    def perturb_obs(self, obs, step, total_steps):
        return obs


class NoPerturbation(Perturbation):
    def __init__(self):
        super().__init__("none", "No perturbation")


class TorqueNoise(Perturbation):
    """Adds Gaussian noise to actions, simulating actuator imprecision."""
    def __init__(self, noise_std=0.15):
        super().__init__("torque_noise", f"Actuator Noise (σ={noise_std})")
        self.noise_std = noise_std

    def perturb_action(self, action, step, total_steps):
        noise = np.random.normal(0, self.noise_std, size=action.shape)
        return np.clip(action + noise, -1.0, 1.0).astype(np.float32)


class PayloadChange(Perturbation):
    """Simulates picking up a payload mid-trajectory by biasing actions."""
    def __init__(self, bias_magnitude=0.25, onset_fraction=0.4):
        super().__init__("payload",
                         f"Payload Change (bias={bias_magnitude}, onset={int(onset_fraction*100)}%)")
        self.bias = bias_magnitude
        self.onset = onset_fraction

    def perturb_action(self, action, step, total_steps):
        if step >= int(total_steps * self.onset):
            # Bias toward higher weights (simulates needing more control effort)
            bias = np.array([self.bias, self.bias * 0.5, -self.bias * 0.3,
                             self.bias, self.bias * 0.5], dtype=np.float32)
            return np.clip(action + bias, -1.0, 1.0).astype(np.float32)
        return action


class SensorNoise(Perturbation):
    """Adds noise to observations, simulating sensor inaccuracies."""
    def __init__(self, noise_std=0.02):
        super().__init__("sensor_noise", f"Sensor Noise (σ={noise_std})")
        self.noise_std = noise_std

    def perturb_obs(self, obs, step, total_steps):
        noise = np.random.normal(0, self.noise_std, size=obs.shape).astype(np.float32)
        return obs + noise


class ImpulsePerturbation(Perturbation):
    """Simulates a sudden impulse/collision at a random time."""
    def __init__(self, impulse_magnitude=0.4, duration_steps=5):
        super().__init__("impulse",
                         f"External Impulse (mag={impulse_magnitude})")
        self.magnitude = impulse_magnitude
        self.duration = duration_steps
        self.impulse_start = None

    def perturb_action(self, action, step, total_steps):
        if self.impulse_start is None:
            self.impulse_start = np.random.randint(
                int(total_steps * 0.2), int(total_steps * 0.7))

        if self.impulse_start <= step < self.impulse_start + self.duration:
            impulse = np.random.uniform(-self.magnitude, self.magnitude,
                                        size=action.shape).astype(np.float32)
            return np.clip(action + impulse, -1.0, 1.0).astype(np.float32)
        return action


class CombinedPerturbation(Perturbation):
    """Combines torque noise + sensor noise + payload change."""
    def __init__(self):
        super().__init__("combined", "Combined Perturbations")
        self.torque = TorqueNoise(noise_std=0.10)
        self.sensor = SensorNoise(noise_std=0.015)
        self.payload = PayloadChange(bias_magnitude=0.15, onset_fraction=0.4)

    def perturb_action(self, action, step, total_steps):
        action = self.torque.perturb_action(action, step, total_steps)
        action = self.payload.perturb_action(action, step, total_steps)
        return action

    def perturb_obs(self, obs, step, total_steps):
        return self.sensor.perturb_obs(obs, step, total_steps)


# ============================================================
# Episode runner with perturbations
# ============================================================

def run_episode_perturbed(backend, q_start, q_target, T, dt, mode, N,
                          model=None, fixed_action=None, perturbation=None):
    """Run one episode with optional perturbations applied."""
    if perturbation is None:
        perturbation = NoPerturbation()

    obs, reset_info = backend.reset(q_start, q_target, T=T, dt=dt, mode=mode, N=N)
    total_steps = int(T / dt) + 5

    trajectory = {
        "t": [], "eq_rms": [], "edq_rms": [],
        "u_energy": [], "du_energy": [], "reward": [],
    }

    done = False
    total_reward = 0.0
    step = 0

    while not done:
        # Perturb observation (before policy sees it)
        obs_perturbed = perturbation.perturb_obs(obs.copy(), step, total_steps)

        if model is not None:
            action, _ = model.predict(obs_perturbed, deterministic=True)
        elif fixed_action is not None:
            action = fixed_action.copy()
        else:
            action = np.zeros(5, dtype=np.float32)

        # Perturb action (before sending to backend)
        action = perturbation.perturb_action(action, step, total_steps)

        obs, reward, done, info = backend.step(action)
        total_reward += reward
        step += 1

        trajectory["t"].append(info.get("t", 0))
        trajectory["eq_rms"].append(info.get("eq_rms", 0))
        trajectory["edq_rms"].append(info.get("edq_rms", 0))
        trajectory["u_energy"].append(info.get("u_energy", 0))
        trajectory["du_energy"].append(info.get("du_energy", 0))
        trajectory["reward"].append(reward)

    for key in trajectory:
        trajectory[key] = np.array(trajectory[key])

    trajectory["total_reward"] = total_reward
    trajectory["final_eq_rms"] = trajectory["eq_rms"][-1] if len(trajectory["eq_rms"]) > 0 else np.inf
    trajectory["success"] = info.get("success", False)
    trajectory["mean_u_energy"] = np.mean(trajectory["u_energy"]) if len(trajectory["u_energy"]) > 0 else 0
    trajectory["mean_du_energy"] = np.mean(trajectory["du_energy"]) if len(trajectory["du_energy"]) > 0 else 0

    return trajectory


# ============================================================
# Run full comparison
# ============================================================

def generate_test_configs(n_episodes, seed=42):
    """Generate reproducible start/target pairs."""
    rng = np.random.RandomState(seed)
    q_ranges = [
        (-1.5, 1.5), (-1.5, 0.5), (-2.0, 0.5),
        (-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5),
    ]

    configs = []
    for _ in range(n_episodes):
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

        configs.append((q_start, q_target))
    return configs


def run_full_comparison(backend, models, configs, perturbations, T, dt, mode, N):
    """Run all combinations of models × perturbations × episodes."""
    results = {}

    for pert in perturbations:
        results[pert.name] = {}

        for model_name, model in models.items():
            results[pert.name][model_name] = []
            print(f"\n  [{pert.label}] {model_name}:")

            for ep, (q_start, q_target) in enumerate(configs):
                # Reset perturbation state per episode
                if hasattr(pert, 'impulse_start'):
                    pert.impulse_start = None

                if model_name == "Fixed LQR":
                    traj = run_episode_perturbed(
                        backend, q_start, q_target, T, dt, mode, N,
                        fixed_action=np.zeros(5, dtype=np.float32),
                        perturbation=pert)
                else:
                    traj = run_episode_perturbed(
                        backend, q_start, q_target, T, dt, mode, N,
                        model=model, perturbation=pert)

                results[pert.name][model_name].append(traj)

                if (ep + 1) % 10 == 0:
                    avg_eq = np.mean([r["final_eq_rms"] for r in results[pert.name][model_name]])
                    sr = np.mean([r["success"] for r in results[pert.name][model_name]]) * 100
                    print(f"    Ep {ep+1}: avg_eq={avg_eq:.4f}, success={sr:.0f}%")

    return results


# ============================================================
# Plotting
# ============================================================

def setup_style():
    plt.rcParams.update({
        'font.family': 'serif', 'font.size': 11,
        'axes.labelsize': 12, 'axes.titlesize': 13,
        'xtick.labelsize': 10, 'ytick.labelsize': 10,
        'legend.fontsize': 9, 'figure.dpi': 150,
        'savefig.dpi': 300, 'savefig.bbox': 'tight',
        'axes.grid': True, 'grid.alpha': 0.3,
    })


def plot_robustness_summary(results, perturbation_names, model_names, output_dir):
    """Main figure: bar chart of error and success rate per perturbation × model."""
    n_perts = len(perturbation_names)
    n_models = len(model_names)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))

    colors = {
        "Fixed LQR": '#d62728',
        "PPO v1": '#ff7f0e',
        "PPO v2": '#1f77b4',
    }

    x = np.arange(n_perts)
    width = 0.8 / n_models

    # Error final
    for i, mn in enumerate(model_names):
        vals = []
        stds = []
        for pn in perturbation_names:
            eqs = [r["final_eq_rms"] for r in results[pn][mn]]
            vals.append(np.mean(eqs))
            stds.append(np.std(eqs))
        axes[0].bar(x + i * width - (n_models - 1) * width / 2,
                     vals, width, yerr=stds,
                     color=colors.get(mn, f'C{i}'), alpha=0.85,
                     label=mn, capsize=3)
    axes[0].set_ylabel("Final Error (RMS rad)")
    axes[0].set_title("Precision under Perturbations")
    axes[0].set_xticks(x)
    axes[0].axhline(y=0.03, color='green', linestyle='--', linewidth=1, alpha=0.5)
    axes[0].legend()

    # Success rate
    for i, mn in enumerate(model_names):
        vals = []
        for pn in perturbation_names:
            sr = np.mean([r["success"] for r in results[pn][mn]]) * 100
            vals.append(sr)
        axes[1].bar(x + i * width - (n_models - 1) * width / 2,
                     vals, width,
                     color=colors.get(mn, f'C{i}'), alpha=0.85,
                     label=mn)
    axes[1].set_ylabel("Success Rate (%)")
    axes[1].set_title("Success under Perturbations")
    axes[1].set_xticks(x)
    axes[1].set_ylim(0, 105)
    axes[1].legend()

    # Reward
    for i, mn in enumerate(model_names):
        vals = []
        for pn in perturbation_names:
            rews = [r["total_reward"] for r in results[pn][mn]]
            vals.append(np.mean(rews))
        axes[2].bar(x + i * width - (n_models - 1) * width / 2,
                     vals, width,
                     color=colors.get(mn, f'C{i}'), alpha=0.85,
                     label=mn)
    axes[2].set_ylabel("Total Reward")
    axes[2].set_title("Reward under Perturbations")
    axes[2].set_xticks(x)
    axes[2].legend()

    # x-axis labels
    pert_labels_short = {
        "none": "Sin\npert.",
        "torque_noise": "Actuator\nNoise",
        "sensor_noise": "Sensor\nNoise",
        "payload": "Payload\nChange",
        "impulse": "External\nImpulse",
        "combined": "Combined",
    }
    for ax in axes:
        ax.set_xticklabels([pert_labels_short.get(pn, pn) for pn in perturbation_names],
                           fontsize=9)

    fig.suptitle("Robustness: Fixed LQR vs PPO Adaptive under Perturbations",
                 fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "05_robustness_summary.png"))
    fig.savefig(os.path.join(output_dir, "05_robustness_summary.pdf"))
    plt.close(fig)
    print("  -> 05_robustness_summary.png/pdf")


def plot_degradation_curves(results, perturbation_names, model_names, output_dir):
    """Figure: how error degrades with increasing perturbation severity."""
    fig, ax = plt.subplots(figsize=(10, 5.5))

    colors = {
        "Fixed LQR": '#d62728',
        "PPO v1": '#ff7f0e',
        "PPO v2": '#1f77b4',
    }
    markers = {"Fixed LQR": 's', "PPO v1": '^', "PPO v2": 'o'}

    for mn in model_names:
        means = []
        stds = []
        for pn in perturbation_names:
            eqs = [r["final_eq_rms"] for r in results[pn][mn]]
            means.append(np.mean(eqs))
            stds.append(np.std(eqs))

        ax.errorbar(range(len(perturbation_names)), means, yerr=stds,
                     color=colors.get(mn, 'gray'),
                     marker=markers.get(mn, 'o'),
                     linewidth=2, markersize=8, capsize=5,
                     label=mn)

    ax.axhline(y=0.03, color='green', linestyle='--', linewidth=1,
               alpha=0.5, label='Success Threshold')

    pert_labels = {
        "none": "No Pert.",
        "torque_noise": "Actuator\nNoise",
        "sensor_noise": "Sensor\nNoise",
        "payload": "Payload\nChange",
        "impulse": "Impulse",
        "combined": "Combined",
    }
    ax.set_xticks(range(len(perturbation_names)))
    ax.set_xticklabels([pert_labels.get(pn, pn) for pn in perturbation_names])
    ax.set_ylabel("Mean Final Error (RMS rad)")
    ax.set_title("Degradation Curve: Error vs Perturbation Type")
    ax.legend(loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "06_degradation_curves.png"))
    fig.savefig(os.path.join(output_dir, "06_degradation_curves.pdf"))
    plt.close(fig)
    print("  -> 06_degradation_curves.png/pdf")


def plot_time_series_perturbed(results, perturbation_name, model_names, output_dir):
    """Time series of tracking error under worst perturbation (combined)."""
    fig, ax = plt.subplots(figsize=(10, 5))

    colors = {
        "Fixed LQR": '#d62728',
        "PPO v1": '#ff7f0e',
        "PPO v2": '#1f77b4',
    }

    for mn in model_names:
        trajs = results[perturbation_name][mn]
        max_len = max(len(t["eq_rms"]) for t in trajs)
        matrix = np.full((len(trajs), max_len), np.nan)
        for i, t in enumerate(trajs):
            matrix[i, :len(t["eq_rms"])] = t["eq_rms"]

        t_axis = np.arange(max_len) * 0.02
        mean = np.nanmean(matrix, axis=0)
        std = np.nanstd(matrix, axis=0)

        ax.plot(t_axis, mean, color=colors.get(mn, 'gray'),
                linewidth=1.8, label=mn)
        ax.fill_between(t_axis, mean - std, mean + std,
                        color=colors.get(mn, 'gray'), alpha=0.12)

    ax.axhline(y=0.03, color='green', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Tracking Error (RMS rad)")
    ax.set_title(f"Time Series Error under Combined Perturbations")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "07_time_series_combined_perturbation.png"))
    fig.savefig(os.path.join(output_dir, "07_time_series_combined_perturbation.pdf"))
    plt.close(fig)
    print("  -> 07_time_series_combined_perturbation.png/pdf")


def print_full_table(results, perturbation_names, model_names, pert_labels):
    """Print complete results table."""
    print("\n" + "=" * 90)
    print("  TABLA DE RESULTADOS COMPLETA")
    print("=" * 90)

    header = f"{'Perturbación':<22}"
    for mn in model_names:
        header += f" | {mn:>12} (eq)  {mn:>8} (SR)"
    print(header)
    print("-" * 90)

    for pn in perturbation_names:
        row = f"  {pert_labels.get(pn, pn):<20}"
        for mn in model_names:
            eqs = [r["final_eq_rms"] for r in results[pn][mn]]
            sr = np.mean([r["success"] for r in results[pn][mn]]) * 100
            row += f" | {np.mean(eqs):>10.4f}±{np.std(eqs):.4f}  {sr:>7.1f}%"
        print(row)

    print("=" * 90)


def save_full_csv(results, perturbation_names, model_names, output_dir):
    """Save all results to CSV."""
    with open(os.path.join(output_dir, "perturbation_results.csv"), "w") as f:
        f.write("perturbation,model,episode,final_eq_rms,total_reward,"
                "mean_u_energy,mean_du_energy,success\n")
        for pn in perturbation_names:
            for mn in model_names:
                for i, r in enumerate(results[pn][mn]):
                    f.write(f"{pn},{mn},{i+1},{r['final_eq_rms']:.6f},"
                            f"{r['total_reward']:.4f},{r['mean_u_energy']:.6f},"
                            f"{r['mean_du_energy']:.6f},{r['success']}\n")
    print(f"  -> perturbation_results.csv")


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Compare LQR vs PPO under perturbations"
    )
    parser.add_argument("--model-v1", type=str, default="checkpoints/ur5e_ppo_final.zip")
    parser.add_argument("--model-v2", type=str, default="checkpoints_v2/ur5e_ppo_v2_final.zip")
    parser.add_argument("--url", type=str, default="http://127.0.0.1:8848")
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--T", type=float, default=1.5)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--mode", type=str, default="mpc_lite")
    parser.add_argument("--N", type=int, default=20)
    parser.add_argument("--output-dir", type=str, default="thesis_plots_perturbations")
    parser.add_argument("--seed", type=int, default=42)

    args = parser.parse_args()
    setup_style()
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 60)
    print("  Comparison with Perturbations")
    print("=" * 60)
    print(f"  Episodios:  {args.episodes}")
    print(f"  Modelo v1:  {args.model_v1}")
    print(f"  Modelo v2:  {args.model_v2}")
    print("=" * 60)

    # Load models
    models = {"Fixed LQR": None}

    if os.path.exists(args.model_v1):
        print(f"Cargando PPO v1: {args.model_v1}")
        models["PPO v1"] = PPO.load(args.model_v1)
    else:
        print(f"[!] No se encontró v1: {args.model_v1}")

    if os.path.exists(args.model_v2):
        print(f"Cargando PPO v2: {args.model_v2}")
        models["PPO v2"] = PPO.load(args.model_v2)
    else:
        print(f"[!] No se encontró v2: {args.model_v2}")

    backend = RobotArmBackendEnv(base_url=args.url)
    configs = generate_test_configs(args.episodes, args.seed)

    # Define perturbation scenarios
    perturbations = [
        NoPerturbation(),
        TorqueNoise(noise_std=0.15),
        SensorNoise(noise_std=0.02),
        PayloadChange(bias_magnitude=0.25, onset_fraction=0.4),
        ImpulsePerturbation(impulse_magnitude=0.4, duration_steps=5),
        CombinedPerturbation(),
    ]

    pert_names = [p.name for p in perturbations]
    model_names = list(models.keys())

    # Run all comparisons
    print(f"\nRunning {args.episodes} episodes × {len(perturbations)} perturbations × {len(models)} models...")
    results = run_full_comparison(
        backend, models, configs, perturbations,
        args.T, args.dt, args.mode, args.N
    )

    # Results table
    pert_labels = {p.name: p.label for p in perturbations}
    print_full_table(results, pert_names, model_names, pert_labels)

    # Plots
    print(f"\nGenerando gráficas en {args.output_dir}/...")
    plot_robustness_summary(results, pert_names, model_names, args.output_dir)
    plot_degradation_curves(results, pert_names, model_names, args.output_dir)
    plot_time_series_perturbed(results, "combined", model_names, args.output_dir)
    save_full_csv(results, pert_names, model_names, args.output_dir)

    print(f"\n¡Listo! Resultados en: {args.output_dir}/")


if __name__ == "__main__":
    main()
