#!/usr/bin/env python3
"""
PPO training v2 for thesis-quality results.

What this version really adds:
  - real domain randomization end-to-end via /rl/reset
  - observation noise for robustness
  - 3-phase curriculum
  - local reward shaping for sustained low tracking error
  - TensorBoard logging of episode metrics
  - custom evaluation callback that saves the best model by success rate
"""

from __future__ import annotations

import argparse
import os
import time
from typing import Any, Dict, List

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

from env_backend_robot_arm import RobotArmBackendEnv


class UR5eGymEnvV2(gym.Env):
    """
    Improved UR5e Gym environment with:
      - domain randomization (friction/inertia perturbation)
      - observation noise
      - 3-phase curriculum learning
      - local reward shaping for sustained low error
    """

    metadata = {"render_modes": []}

    def __init__(self, base_url: str = "http://127.0.0.1:8848", config: dict | None = None):
        super().__init__()

        self.backend = RobotArmBackendEnv(base_url=base_url)
        self.config = config or {}

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.backend.obs_dim,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.backend.act_dim,),
            dtype=np.float32,
        )

        self.T = float(self.config.get("T", 1.5))
        self.dt = float(self.config.get("dt", 0.02))
        self.mode = str(self.config.get("mode", "mpc_lite"))
        self.N = int(self.config.get("N", 20))
        self.u_max = float(self.config.get("u_max", 8.0))
        self.max_steps = int(self.config.get("max_steps", int(self.T / self.dt) + 10))
        self.success_tol = float(self.config.get("success_tol", 0.03))

        self.rw_pos = self.config.get("rw_pos", 2.0)
        self.rw_vel = self.config.get("rw_vel", 0.15)
        self.rw_u = self.config.get("rw_u", 0.01)
        self.rw_du = self.config.get("rw_du", 0.005)
        self.rw_success = self.config.get("rw_success", 5.0)

        self.q_range = self.config.get(
            "q_range",
            [
                (-1.5, 1.5), (-1.5, 0.5), (-2.0, 0.5),
                (-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5),
            ],
        )

        self.domain_rand = bool(self.config.get("domain_rand", True))
        self.rand_friction_range = float(self.config.get("rand_friction_range", 0.2))
        self.rand_inertia_range = float(self.config.get("rand_inertia_range", 0.15))

        self.obs_noise_std = float(self.config.get("obs_noise_std", 0.005))

        self.curriculum_enabled = bool(self.config.get("curriculum", True))
        self.episode_count = 0
        self.phase1_episodes = max(1, int(self.config.get("phase1_episodes", 300)))
        self.phase2_episodes = max(self.phase1_episodes + 1, int(self.config.get("phase2_episodes", 800)))

        self.low_error_threshold = float(self.config.get("low_error_threshold", 0.05))
        self.low_error_bonus = float(self.config.get("low_error_bonus", 0.02))
        self.low_error_streak_target = max(1, int(self.config.get("low_error_streak_target", 5)))

        self._current_obs: np.ndarray | None = None
        self._step_in_episode = 0
        self._low_error_streak = 0
        self._last_randomization = {"friction_scale": 1.0, "inertia_scale": 1.0}

    def _get_curriculum_scale(self) -> float:
        if not self.curriculum_enabled:
            return 1.0
        if self.episode_count < self.phase1_episodes:
            progress = self.episode_count / self.phase1_episodes
            return 0.15 + 0.25 * progress
        if self.episode_count < self.phase2_episodes:
            progress = (self.episode_count - self.phase1_episodes) / (self.phase2_episodes - self.phase1_episodes)
            return 0.40 + 0.40 * progress
        remaining = self.episode_count - self.phase2_episodes
        return min(1.0, 0.80 + 0.20 * (remaining / 200.0))

    def _sample_config(self, scale: float) -> tuple[np.ndarray, np.ndarray]:
        q_target = np.zeros(6, dtype=np.float64)
        for i, (lo, hi) in enumerate(self.q_range):
            mid = (lo + hi) / 2.0
            half_range = (hi - lo) / 2.0 * scale
            q_target[i] = np.random.uniform(mid - half_range, mid + half_range)

        q_start = np.zeros(6, dtype=np.float64)
        if np.random.random() > 0.25:
            for i, (lo, hi) in enumerate(self.q_range):
                mid = (lo + hi) / 2.0
                half_range = (hi - lo) / 2.0 * scale * 0.5
                q_start[i] = np.random.uniform(mid - half_range, mid + half_range)

        while np.linalg.norm(q_target - q_start) < 0.1:
            for i, (lo, hi) in enumerate(self.q_range):
                mid = (lo + hi) / 2.0
                half_range = (hi - lo) / 2.0 * scale
                q_target[i] = np.random.uniform(mid - half_range, mid + half_range)

        return q_start, q_target

    def _domain_randomization_params(self) -> dict[str, float]:
        if not self.domain_rand:
            return {"friction_scale": 1.0, "inertia_scale": 1.0}

        friction_scale = 1.0 + np.random.uniform(-self.rand_friction_range, self.rand_friction_range)
        inertia_scale = 1.0 + np.random.uniform(-self.rand_inertia_range, self.rand_inertia_range)

        return {
            "friction_scale": float(max(0.2, friction_scale)),
            "inertia_scale": float(max(0.2, inertia_scale)),
        }

    def _maybe_add_obs_noise(self, obs: np.ndarray) -> np.ndarray:
        obs = np.asarray(obs, dtype=np.float32)
        if self.obs_noise_std > 0.0:
            obs = obs + np.random.normal(0.0, self.obs_noise_std, size=obs.shape).astype(np.float32)
        return obs

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        scale = self._get_curriculum_scale()
        q_start, q_target = self._sample_config(scale)
        dr_params = self._domain_randomization_params()

        try:
            obs, info = self.backend.reset(
                q_start=q_start,
                q_target=q_target,
                T=self.T,
                dt=self.dt,
                mode=self.mode,
                N=self.N,
                u_max=self.u_max,
                max_steps=self.max_steps,
                success_tol=self.success_tol,
                rw_pos=self.rw_pos,
                rw_vel=self.rw_vel,
                rw_u=self.rw_u,
                rw_du=self.rw_du,
                rw_success=self.rw_success,
                friction_scale=dr_params["friction_scale"],
                inertia_scale=dr_params["inertia_scale"],
            )
        except Exception as exc:
            print(f"[UR5eGymEnvV2] Backend reset failed: {exc}")
            obs = np.zeros(self.backend.obs_dim, dtype=np.float32)
            info = {"error": str(exc)}

        info = dict(info)
        info.update(dr_params)

        obs = self._maybe_add_obs_noise(obs)

        self._current_obs = obs
        self._step_in_episode = 0
        self._low_error_streak = 0
        self._last_randomization = dr_params
        self.episode_count += 1
        return obs, info

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        try:
            obs, reward, done, info = self.backend.step(action)
        except Exception as exc:
            print(f"[UR5eGymEnvV2] Backend step failed: {exc}")
            obs = self._current_obs if self._current_obs is not None else np.zeros(self.backend.obs_dim, dtype=np.float32)
            reward = -1.0
            done = True
            info = {"error": str(exc), "truncated": True, "success": False}

        info = dict(info)
        info.update(self._last_randomization)

        eq_rms = info.get("eq_rms")
        if isinstance(eq_rms, (int, float, np.integer, np.floating)):
            if float(eq_rms) <= self.low_error_threshold:
                self._low_error_streak += 1
                if self._low_error_streak >= self.low_error_streak_target:
                    reward += self.low_error_bonus
                    info["low_error_bonus_applied"] = True
            else:
                self._low_error_streak = 0
        else:
            self._low_error_streak = 0

        obs = self._maybe_add_obs_noise(obs)

        self._current_obs = obs
        self._step_in_episode += 1

        backend_success = bool(info.get("success", False))
        backend_truncated = bool(info.get("truncated", False))

        terminated = bool(done and backend_success)
        truncated = bool(done and not backend_success) or backend_truncated

        if not (terminated or truncated) and self._step_in_episode >= self.max_steps:
            truncated = True
            info["truncated"] = True
            info["forced_truncation"] = True

        return obs, float(reward), terminated, truncated, info

    def close(self):
        self.backend.close()
        super().close()


class TrainingMetricsCallback(BaseCallback):
    """Log extra training metrics to TensorBoard."""

    def __init__(self, log_every_steps: int = 2048, verbose: int = 0):
        super().__init__(verbose)
        self.log_every_steps = max(1, int(log_every_steps))
        self.ep_rewards: List[float] = []
        self.ep_lengths: List[int] = []

    def _on_training_start(self) -> None:
        n_envs = int(getattr(self.training_env, "num_envs", 1))
        self.ep_rewards = [0.0] * n_envs
        self.ep_lengths = [0] * n_envs

    def _on_step(self) -> bool:
        rewards = self.locals.get("rewards", [])
        dones = self.locals.get("dones", [])
        infos = self.locals.get("infos", [])

        for i in range(len(dones)):
            self.ep_rewards[i] += float(rewards[i])
            self.ep_lengths[i] += 1
            info = infos[i] if i < len(infos) else {}

            if dones[i]:
                self.logger.record("custom/episode_reward", self.ep_rewards[i])
                self.logger.record("custom/episode_length", self.ep_lengths[i])
                self.logger.record("custom/episode_success", float(bool(info.get("success", False))))
                if "eq_rms" in info:
                    self.logger.record("custom/episode_eq_rms", float(info["eq_rms"]))
                if "edq_rms" in info:
                    self.logger.record("custom/episode_edq_rms", float(info["edq_rms"]))
                if "u_energy" in info:
                    self.logger.record("custom/episode_u_energy", float(info["u_energy"]))
                if "du_energy" in info:
                    self.logger.record("custom/episode_du_energy", float(info["du_energy"]))
                if "friction_scale" in info:
                    self.logger.record("custom/friction_scale", float(info["friction_scale"]))
                if "inertia_scale" in info:
                    self.logger.record("custom/inertia_scale", float(info["inertia_scale"]))
                self.ep_rewards[i] = 0.0
                self.ep_lengths[i] = 0

        if self.num_timesteps % self.log_every_steps == 0:
            self.logger.record(
                "custom/learning_rate",
                float(self.model.lr_schedule(self.model._current_progress_remaining)),
            )
        return True


class SuccessRateEvalCallback(BaseCallback):
    """
    Evaluate the current policy on a clean environment and save the best model
    according to success rate, using average reward as a tie-breaker.
    """

    def __init__(
        self,
        eval_env: UR5eGymEnvV2,
        eval_freq: int,
        n_eval_episodes: int,
        best_model_save_path: str,
        deterministic: bool = True,
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = max(1, int(eval_freq))
        self.n_eval_episodes = max(1, int(n_eval_episodes))
        self.best_model_save_path = best_model_save_path
        self.deterministic = deterministic
        self.best_success_rate = -np.inf
        self.best_avg_reward = -np.inf
        os.makedirs(self.best_model_save_path, exist_ok=True)

    def _evaluate(self) -> tuple[float, float, float]:
        total_reward = 0.0
        successes = 0
        eq_rms_values = []

        for _ in range(self.n_eval_episodes):
            obs, info = self.eval_env.reset()
            done = False
            ep_reward = 0.0
            last_info = info

            while not done:
                action, _ = self.model.predict(obs, deterministic=self.deterministic)
                obs, reward, terminated, truncated, last_info = self.eval_env.step(action)
                ep_reward += float(reward)
                done = bool(terminated or truncated)

            total_reward += ep_reward
            if bool(last_info.get("success", False)):
                successes += 1
            eq_rms = last_info.get("eq_rms")
            if isinstance(eq_rms, (int, float, np.integer, np.floating)):
                eq_rms_values.append(float(eq_rms))

        avg_reward = total_reward / self.n_eval_episodes
        success_rate = successes / self.n_eval_episodes
        avg_eq_rms = float(np.mean(eq_rms_values)) if eq_rms_values else float("nan")
        return avg_reward, success_rate, avg_eq_rms

    def _on_step(self) -> bool:
        if self.num_timesteps % self.eval_freq != 0:
            return True

        avg_reward, success_rate, avg_eq_rms = self._evaluate()
        self.logger.record("eval_custom/avg_reward", avg_reward)
        self.logger.record("eval_custom/success_rate", success_rate)
        if not np.isnan(avg_eq_rms):
            self.logger.record("eval_custom/avg_eq_rms", avg_eq_rms)

        is_better = (
            success_rate > self.best_success_rate
            or (np.isclose(success_rate, self.best_success_rate) and avg_reward > self.best_avg_reward)
        )
        if is_better:
            self.best_success_rate = success_rate
            self.best_avg_reward = avg_reward
            model_path = os.path.join(self.best_model_save_path, "best_success_model")
            self.model.save(model_path)
            if self.verbose:
                print(
                    "[SuccessRateEvalCallback] New best model saved: "
                    f"success_rate={success_rate:.3f}, avg_reward={avg_reward:.3f}"
                )
        return True


def make_env_v2(rank: int, base_url: str, config: dict, log_dir: str | None):
    def _init():
        env = UR5eGymEnvV2(base_url=base_url, config=config)
        if log_dir is not None:
            env = Monitor(env, os.path.join(log_dir, f"env_{rank}"))
        return env

    return _init


def build_env_config(args: argparse.Namespace) -> Dict[str, Any]:
    max_steps = args.max_steps if args.max_steps is not None else int(args.T / args.dt) + 10
    return {
        "T": args.T,
        "dt": args.dt,
        "mode": args.mode,
        "N": args.N,
        "u_max": args.u_max,
        "max_steps": max_steps,
        "success_tol": args.success_tol,
        "rw_pos": args.rw_pos,
        "rw_vel": args.rw_vel,
        "rw_u": args.rw_u,
        "rw_du": args.rw_du,
        "rw_success": args.rw_success,
        "curriculum": args.curriculum,
        "phase1_episodes": args.phase1,
        "phase2_episodes": args.phase2,
        "domain_rand": args.domain_rand,
        "rand_friction_range": args.friction_range,
        "rand_inertia_range": args.inertia_range,
        "obs_noise_std": args.obs_noise_std,
        "low_error_threshold": args.low_error_threshold,
        "low_error_bonus": args.low_error_bonus,
        "low_error_streak_target": args.low_error_streak,
    }


def train(args: argparse.Namespace):
    print("=" * 60)
    print("  UR5e PPO Training v2")
    print("=" * 60)
    print(f"  Backend URL:       {args.url}")
    print(f"  Total timesteps:   {args.timesteps:,}")
    print(f"  Domain rand:       {args.domain_rand}")
    print(f"  Obs noise std:     {args.obs_noise_std}")
    print(f"  Curriculum phases: {args.phase1}/{args.phase2}/full")
    print(f"  Learning rate:     {args.lr}")
    print(f"  Save dir:          {args.save_dir}")
    print("=" * 60)

    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    monitor_dir = os.path.join(args.log_dir, "monitor")
    os.makedirs(monitor_dir, exist_ok=True)

    env_config = build_env_config(args)
    env = DummyVecEnv([make_env_v2(0, args.url, env_config, monitor_dir)])

    eval_config = dict(env_config)
    eval_config["domain_rand"] = False
    eval_config["obs_noise_std"] = 0.0
    eval_config["curriculum"] = False
    eval_monitor_dir = os.path.join(args.log_dir, "eval_monitor")
    os.makedirs(eval_monitor_dir, exist_ok=True)
    eval_env_vec = DummyVecEnv([make_env_v2(99, args.url, eval_config, eval_monitor_dir)])

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=args.lr,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        ent_coef=args.ent_coef,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256])),
        tensorboard_log=args.log_dir,
        verbose=1,
        seed=args.seed,
    )

    print(f"\nTotal parameters: {sum(p.numel() for p in model.policy.parameters()):,}")

    checkpoint_cb = CheckpointCallback(
        save_freq=max(args.checkpoint_freq, 1),
        save_path=args.save_dir,
        name_prefix="ur5e_ppo_v2",
    )
    metrics_cb = TrainingMetricsCallback()
    eval_cb = EvalCallback(
        eval_env_vec,
        best_model_save_path=os.path.join(args.save_dir, "best"),
        eval_freq=max(args.eval_freq, 1),
        n_eval_episodes=args.eval_episodes,
        deterministic=True,
        verbose=1,
    )
    callbacks = CallbackList([checkpoint_cb, metrics_cb, eval_cb])

    print("\nStarting training...\n")
    t0 = time.time()

    try:
        model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    except KeyboardInterrupt:
        print("\n[!] Training interrupted by user.")

    elapsed = time.time() - t0
    print(f"\nTraining completed in {elapsed:.1f}s ({elapsed/60:.1f} min)")

    final_path = os.path.join(args.save_dir, "ur5e_ppo_v2_final")
    model.save(final_path)
    print(f"Final model saved to: {final_path}.zip")

    env.close()
    eval_env_vec.close()


def evaluate(args: argparse.Namespace):
    print(f"Loading model from: {args.model_path}")

    env_config = {
        "T": args.T,
        "dt": args.dt,
        "mode": args.mode,
        "N": args.N,
        "u_max": args.u_max,
        "max_steps": args.max_steps if args.max_steps is not None else int(args.T / args.dt) + 10,
        "success_tol": args.success_tol,
        "rw_pos": args.rw_pos,
        "rw_vel": args.rw_vel,
        "rw_u": args.rw_u,
        "rw_du": args.rw_du,
        "rw_success": args.rw_success,
        "curriculum": False,
        "domain_rand": False,
        "obs_noise_std": 0.0,
        "low_error_threshold": args.low_error_threshold,
        "low_error_bonus": args.low_error_bonus,
        "low_error_streak_target": args.low_error_streak,
    }

    env = UR5eGymEnvV2(base_url=args.url, config=env_config)
    model = PPO.load(args.model_path)

    total_reward = 0.0
    successes = 0

    for ep in range(args.eval_episodes):
        obs, info = env.reset()
        ep_reward = 0.0
        done = False

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += float(reward)
            done = bool(terminated or truncated)

        total_reward += ep_reward
        success = bool(info.get("success", False))
        if success:
            successes += 1

        eq_rms = info.get("eq_rms")
        eq_str = f"{eq_rms:.4f}" if isinstance(eq_rms, (int, float, np.integer, np.floating)) else "?"
        print(
            f"  Episode {ep + 1}/{args.eval_episodes}: "
            f"reward={ep_reward:.3f}, success={success}, eq_rms={eq_str}"
        )

    avg_reward = total_reward / max(args.eval_episodes, 1)
    success_rate = successes / max(args.eval_episodes, 1) * 100.0
    print(f"\nAverage reward: {avg_reward:.3f}")
    print(f"Success rate:   {success_rate:.1f}% ({successes}/{args.eval_episodes})")


def add_common_env_args(parser: argparse.ArgumentParser):
    parser.add_argument("--url", type=str, default="http://127.0.0.1:8848")
    parser.add_argument("--T", type=float, default=1.5)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--mode", type=str, default="mpc_lite")
    parser.add_argument("--N", type=int, default=20)
    parser.add_argument("--u-max", type=float, default=8.0)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--success-tol", type=float, default=0.03)
    parser.add_argument("--rw-pos", type=float, default=2.0)
    parser.add_argument("--rw-vel", type=float, default=0.15)
    parser.add_argument("--rw-u", type=float, default=0.01)
    parser.add_argument("--rw-du", type=float, default=0.005)
    parser.add_argument("--rw-success", type=float, default=5.0)


def main():
    parser = argparse.ArgumentParser(description="PPO v2 training for UR5e with real domain randomization")
    subparsers = parser.add_subparsers(dest="command")

    tp = subparsers.add_parser("train")
    tp.add_argument("--timesteps", type=int, default=1_000_000)
    tp.add_argument("--seed", type=int, default=42)
    tp.add_argument("--lr", type=float, default=3e-4)
    tp.add_argument("--n-steps", type=int, default=2048)
    tp.add_argument("--batch-size", type=int, default=256)
    tp.add_argument("--n-epochs", type=int, default=10)
    tp.add_argument("--gamma", type=float, default=0.99)
    tp.add_argument("--gae-lambda", type=float, default=0.95)
    tp.add_argument("--clip-range", type=float, default=0.2)
    tp.add_argument("--ent-coef", type=float, default=0.005)

    tp.add_argument("--curriculum", action="store_true", default=True)
    tp.add_argument("--no-curriculum", dest="curriculum", action="store_false")
    tp.add_argument("--phase1", type=int, default=300)
    tp.add_argument("--phase2", type=int, default=800)

    tp.add_argument("--domain-rand", action="store_true", default=True)
    tp.add_argument("--no-domain-rand", dest="domain_rand", action="store_false")
    tp.add_argument("--friction-range", type=float, default=0.2)
    tp.add_argument("--inertia-range", type=float, default=0.15)
    tp.add_argument("--obs-noise-std", type=float, default=0.005)

    tp.add_argument("--low-error-threshold", type=float, default=0.05)
    tp.add_argument("--low-error-bonus", type=float, default=0.02)
    tp.add_argument("--low-error-streak", type=int, default=5)

    tp.add_argument("--save-dir", type=str, default="checkpoints_v2")
    tp.add_argument("--log-dir", type=str, default="logs_v2")
    tp.add_argument("--checkpoint-freq", type=int, default=10_000)
    tp.add_argument("--eval-freq", type=int, default=5_000)
    tp.add_argument("--eval-episodes", type=int, default=10)
    add_common_env_args(tp)

    ep = subparsers.add_parser("eval")
    ep.add_argument("model_path", type=str)
    ep.add_argument("--eval-episodes", type=int, default=30)
    ep.add_argument("--low-error-threshold", type=float, default=0.05)
    ep.add_argument("--low-error-bonus", type=float, default=0.02)
    ep.add_argument("--low-error-streak", type=int, default=5)
    add_common_env_args(ep)

    args = parser.parse_args()
    if args.command == "train":
        train(args)
    elif args.command == "eval":
        evaluate(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
