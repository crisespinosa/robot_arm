#!/usr/bin/env python3
"""
PPO training script for the UR5e robot arm controller.

Uses the Drogon backend (/rl/reset, /rl/step) as the environment.
The PPO agent learns to tune the LQR/MPC-lite weights in real time:
  action[5] in [-1,1] -> [wq, wdq, wu, wqN, wdqN]

This version is aligned with the current backend:
  - forwards success_tol and max_steps to /rl/reset
  - can forward reward weights to the backend
  - uses backend-provided truncated/success info when available
  - includes a local safety truncation if the backend does not stop an episode
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


class UR5eGymEnv(gym.Env):
    """Gymnasium-compatible wrapper for the UR5e backend RL environment."""

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

        self.rw_pos = self.config.get("rw_pos")
        self.rw_vel = self.config.get("rw_vel")
        self.rw_u = self.config.get("rw_u")
        self.rw_du = self.config.get("rw_du")
        self.rw_success = self.config.get("rw_success")

        self.q_range = self.config.get(
            "q_range",
            [
                (-1.5, 1.5),
                (-1.5, 0.5),
                (-2.0, 0.5),
                (-1.5, 1.5),
                (-1.5, 1.5),
                (-1.5, 1.5),
            ],
        )

        self.curriculum_enabled = bool(self.config.get("curriculum", True))
        self.episode_count = 0
        self.curriculum_warmup = max(1, int(self.config.get("curriculum_warmup", 500)))

        self._current_obs: np.ndarray | None = None
        self._step_in_episode = 0

    def _sample_target(self) -> np.ndarray:
        q_target = np.zeros(6, dtype=np.float64)

        if self.curriculum_enabled and self.episode_count < self.curriculum_warmup:
            progress = self.episode_count / self.curriculum_warmup
            scale = 0.2 + 0.8 * progress
        else:
            scale = 1.0

        for i, (lo, hi) in enumerate(self.q_range):
            mid = (lo + hi) / 2.0
            half_range = (hi - lo) / 2.0 * scale
            q_target[i] = np.random.uniform(mid - half_range, mid + half_range)

        return q_target

    def _sample_start(self) -> np.ndarray:
        if np.random.random() < 0.3:
            return np.zeros(6, dtype=np.float64)
        return self._sample_target() * 0.5

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        q_start = self._sample_start()
        q_target = self._sample_target()

        while np.linalg.norm(q_target - q_start) < 0.1:
            q_target = self._sample_target()

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
            )
        except Exception as exc:
            print(f"[UR5eGymEnv] Backend reset failed: {exc}")
            obs = np.zeros(self.backend.obs_dim, dtype=np.float32)
            info = {"error": str(exc)}

        self._current_obs = np.asarray(obs, dtype=np.float32)
        self._step_in_episode = 0
        self.episode_count += 1
        return self._current_obs, info

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        try:
            obs, reward, done, info = self.backend.step(action)
        except Exception as exc:
            print(f"[UR5eGymEnv] Backend step failed: {exc}")
            obs = self._current_obs if self._current_obs is not None else np.zeros(self.backend.obs_dim, dtype=np.float32)
            reward = -1.0
            done = True
            info = {"error": str(exc), "truncated": True, "success": False}

        obs = np.asarray(obs, dtype=np.float32)
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
    """Log basic per-episode training metrics to TensorBoard."""

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
                if "eq_rms" in info:
                    self.logger.record("custom/episode_eq_rms", float(info["eq_rms"]))
                if "u_energy" in info:
                    self.logger.record("custom/episode_u_energy", float(info["u_energy"]))
                if "du_energy" in info:
                    self.logger.record("custom/episode_du_energy", float(info["du_energy"]))
                self.logger.record("custom/episode_success", float(bool(info.get("success", False))))
                self.ep_rewards[i] = 0.0
                self.ep_lengths[i] = 0

        if self.num_timesteps % self.log_every_steps == 0:
            self.logger.record(
                "custom/learning_rate",
                float(self.model.lr_schedule(self.model._current_progress_remaining)),
            )
        return True


def make_env(rank: int, base_url: str = "http://127.0.0.1:8848", config: dict | None = None, log_dir: str | None = None):
    def _init():
        env = UR5eGymEnv(base_url=base_url, config=config)
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
        "curriculum": args.curriculum,
        "curriculum_warmup": args.curriculum_warmup,
        "rw_pos": args.rw_pos,
        "rw_vel": args.rw_vel,
        "rw_u": args.rw_u,
        "rw_du": args.rw_du,
        "rw_success": args.rw_success,
    }


def train(args: argparse.Namespace):
    print("=" * 60)
    print("  UR5e PPO Training")
    print("=" * 60)
    print(f"  Backend URL:     {args.url}")
    print(f"  Total timesteps: {args.timesteps:,}")
    print(f"  Num envs:        {args.num_envs}")
    print(f"  Batch size:      {args.batch_size}")
    print(f"  Learning rate:   {args.lr}")
    print(f"  Save dir:        {args.save_dir}")
    print(f"  Log dir:         {args.log_dir}")
    print("=" * 60)

    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    monitor_dir = os.path.join(args.log_dir, "monitor")
    os.makedirs(monitor_dir, exist_ok=True)

    env_config = build_env_config(args)

    env = DummyVecEnv([
        make_env(i, args.url, env_config, monitor_dir)
        for i in range(args.num_envs)
    ])

    eval_monitor_dir = os.path.join(args.log_dir, "eval_monitor")
    os.makedirs(eval_monitor_dir, exist_ok=True)
    eval_config = dict(env_config)
    eval_config["curriculum"] = False
    eval_env = DummyVecEnv([make_env(99, args.url, eval_config, eval_monitor_dir)])

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

    print(f"\nModel architecture: {model.policy}")
    print(f"Total parameters: {sum(p.numel() for p in model.policy.parameters()):,}")

    checkpoint_cb = CheckpointCallback(
        save_freq=max(args.checkpoint_freq // max(args.num_envs, 1), 1),
        save_path=args.save_dir,
        name_prefix="ur5e_ppo",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(args.save_dir, "best"),
        log_path=os.path.join(args.log_dir, "eval"),
        eval_freq=max(args.eval_freq // max(args.num_envs, 1), 1),
        n_eval_episodes=args.eval_episodes,
        deterministic=True,
    )

    metrics_cb = TrainingMetricsCallback()
    callbacks = CallbackList([checkpoint_cb, eval_cb, metrics_cb])

    print("\nStarting training...\n")
    t0 = time.time()

    try:
        model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    except KeyboardInterrupt:
        print("\n[!] Training interrupted by user.")

    elapsed = time.time() - t0
    print(f"\nTraining completed in {elapsed:.1f}s ({elapsed/60:.1f} min)")

    final_path = os.path.join(args.save_dir, "ur5e_ppo_final")
    model.save(final_path)
    print(f"Final model saved to: {final_path}.zip")

    env.close()
    eval_env.close()


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
        "curriculum": False,
        "rw_pos": args.rw_pos,
        "rw_vel": args.rw_vel,
        "rw_u": args.rw_u,
        "rw_du": args.rw_du,
        "rw_success": args.rw_success,
    }

    env = UR5eGymEnv(base_url=args.url, config=env_config)
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
    parser.add_argument("--url", type=str, default="http://127.0.0.1:8848", help="Backend URL")
    parser.add_argument("--T", type=float, default=1.5, help="Trajectory duration (s)")
    parser.add_argument("--dt", type=float, default=0.02, help="Timestep (s)")
    parser.add_argument("--mode", type=str, default="mpc_lite", help="Control mode (mpc_lite, lqr, pd)")
    parser.add_argument("--N", type=int, default=20, help="LQR horizon")
    parser.add_argument("--u-max", type=float, default=8.0, help="Control saturation limit used by backend")
    parser.add_argument("--max-steps", type=int, default=None, help="Episode max steps forwarded to backend")
    parser.add_argument("--success-tol", type=float, default=0.03, help="Success tolerance (rad RMS)")
    parser.add_argument("--rw-pos", type=float, default=2.0, help="Reward weight for position tracking error")
    parser.add_argument("--rw-vel", type=float, default=0.15, help="Reward weight for velocity tracking error")
    parser.add_argument("--rw-u", type=float, default=0.01, help="Reward weight for control energy")
    parser.add_argument("--rw-du", type=float, default=0.005, help="Reward weight for control smoothness")
    parser.add_argument("--rw-success", type=float, default=5.0, help="Terminal success bonus")


def main():
    parser = argparse.ArgumentParser(description="PPO training for UR5e robot arm LQR weight tuning")
    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    train_p = subparsers.add_parser("train", help="Train a PPO agent")
    train_p.add_argument("--timesteps", type=int, default=500_000, help="Total training timesteps")
    train_p.add_argument("--num-envs", type=int, default=1, help="Number of environments")
    train_p.add_argument("--seed", type=int, default=42, help="Random seed")
    train_p.add_argument("--lr", type=float, default=3e-4, help="Learning rate")
    train_p.add_argument("--n-steps", type=int, default=2048, help="Steps per rollout")
    train_p.add_argument("--batch-size", type=int, default=256, help="Minibatch size")
    train_p.add_argument("--n-epochs", type=int, default=10, help="PPO update epochs per rollout")
    train_p.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    train_p.add_argument("--gae-lambda", type=float, default=0.95, help="GAE lambda")
    train_p.add_argument("--clip-range", type=float, default=0.2, help="PPO clip range")
    train_p.add_argument("--ent-coef", type=float, default=0.005, help="Entropy coefficient")
    train_p.add_argument("--curriculum", action="store_true", default=True, help="Enable curriculum learning")
    train_p.add_argument("--no-curriculum", dest="curriculum", action="store_false")
    train_p.add_argument("--curriculum-warmup", type=int, default=500, help="Episodes for curriculum warmup")
    train_p.add_argument("--save-dir", type=str, default="checkpoints", help="Directory for model checkpoints")
    train_p.add_argument("--log-dir", type=str, default="logs", help="Directory for tensorboard logs")
    train_p.add_argument("--checkpoint-freq", type=int, default=10_000, help="Save checkpoint every N steps")
    train_p.add_argument("--eval-freq", type=int, default=5_000, help="Evaluate every N steps")
    train_p.add_argument("--eval-episodes", type=int, default=10, help="Episodes per evaluation")
    add_common_env_args(train_p)

    eval_p = subparsers.add_parser("eval", help="Evaluate a trained model")
    eval_p.add_argument("model_path", type=str, help="Path to .zip model file")
    eval_p.add_argument("--eval-episodes", type=int, default=20)
    add_common_env_args(eval_p)

    args = parser.parse_args()
    if args.command == "train":
        train(args)
    elif args.command == "eval":
        evaluate(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
