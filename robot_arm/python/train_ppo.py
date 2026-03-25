#!/usr/bin/env python3
"""
PPO training script for the UR5e robot arm controller.

Uses the Drogon backend (/rl/reset, /rl/step) as the environment.
The PPO agent learns to tune the LQR/MPC-lite weights in real-time:
  action[5] in [-1,1] -> [wq, wdq, wu, wqN, wdqN]

Requirements:
  pip install stable-baselines3 gymnasium numpy requests tensorboard

Usage:
  1. Start the backend:  cd robot_arm/build && ./robot_arm
  2. Run training:       python train_ppo.py
  3. Monitor:            tensorboard --logdir logs/

The trained model is saved to checkpoints/ and can be loaded for inference.
"""

import argparse
import os
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    CheckpointCallback, EvalCallback, CallbackList
)
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor

from env_backend_robot_arm import RobotArmBackendEnv


# ============================================================
# Gymnasium wrapper around the backend HTTP environment
# ============================================================

class UR5eGymEnv(gym.Env):
    """
    Gymnasium-compatible wrapper for the UR5e backend RL environment.

    Observation space: Box(25,)  [q, dq, eq, edq, phase]
    Action space:      Box(5,)   [-1, 1]^5 -> mapped to LQR weights by backend
    """

    metadata = {"render_modes": []}

    def __init__(self, base_url="http://127.0.0.1:8848", config=None):
        super().__init__()

        self.backend = RobotArmBackendEnv(base_url=base_url)
        self.config = config or {}

        # Spaces
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self.backend.obs_dim,),
            dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(self.backend.act_dim,),
            dtype=np.float32
        )

        # Training configuration
        self.T = self.config.get("T", 1.5)
        self.dt = self.config.get("dt", 0.02)
        self.mode = self.config.get("mode", "mpc_lite")
        self.N = self.config.get("N", 20)
        self.max_steps = self.config.get("max_steps", int(self.T / self.dt) + 10)
        self.success_tol = self.config.get("success_tol", 0.03)

        # Target generation
        self.q_range = self.config.get("q_range", [
            (-1.5, 1.5),   # q1: shoulder pan
            (-1.5, 0.5),   # q2: shoulder lift (mostly negative/neutral for UR5e)
            (-2.0, 0.5),   # q3: elbow
            (-1.5, 1.5),   # q4: wrist 1
            (-1.5, 1.5),   # q5: wrist 2
            (-1.5, 1.5),   # q6: wrist 3
        ])

        # Curriculum: start with small movements, increase over time
        self.curriculum_enabled = self.config.get("curriculum", True)
        self.episode_count = 0
        self.curriculum_warmup = self.config.get("curriculum_warmup", 500)

        self._current_obs = None

    def _sample_target(self):
        """Sample a random target configuration within the allowed range."""
        q_target = np.zeros(6, dtype=np.float64)

        # Curriculum: start with smaller movements
        if self.curriculum_enabled and self.episode_count < self.curriculum_warmup:
            progress = self.episode_count / self.curriculum_warmup
            scale = 0.2 + 0.8 * progress  # ramp from 20% to 100% of range
        else:
            scale = 1.0

        for i, (lo, hi) in enumerate(self.q_range):
            mid = (lo + hi) / 2.0
            half_range = (hi - lo) / 2.0 * scale
            q_target[i] = np.random.uniform(mid - half_range, mid + half_range)

        return q_target

    def _sample_start(self):
        """Sample a random start configuration (or zero)."""
        if np.random.random() < 0.3:
            # 30% of the time start from home (zero)
            return np.zeros(6, dtype=np.float64)
        else:
            return self._sample_target() * 0.5  # start from a random config

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        q_start = self._sample_start()
        q_target = self._sample_target()

        # Ensure start != target (minimum displacement)
        while np.linalg.norm(q_target - q_start) < 0.1:
            q_target = self._sample_target()

        try:
            obs, info = self.backend.reset(
                q_start=q_start,
                q_target=q_target,
                T=self.T,
                dt=self.dt,
                mode=self.mode,
                N=self.N
            )
        except Exception as e:
            print(f"[UR5eGymEnv] Backend reset failed: {e}")
            obs = np.zeros(self.backend.obs_dim, dtype=np.float32)
            info = {}

        self._current_obs = obs
        self.episode_count += 1
        return obs, info

    def step(self, action):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        try:
            obs, reward, done, info = self.backend.step(action)
        except Exception as e:
            print(f"[UR5eGymEnv] Backend step failed: {e}")
            obs = self._current_obs if self._current_obs is not None else \
                  np.zeros(self.backend.obs_dim, dtype=np.float32)
            reward = -1.0
            done = True
            info = {"error": str(e)}

        self._current_obs = obs

        # SB3 expects (obs, reward, terminated, truncated, info)
        terminated = done and info.get("success", False)
        truncated = done and not terminated

        return obs, float(reward), terminated, truncated, info


# ============================================================
# Environment factory
# ============================================================

def make_env(rank, base_url="http://127.0.0.1:8848", config=None, log_dir=None):
    """Factory function for creating monitored environments."""
    def _init():
        env = UR5eGymEnv(base_url=base_url, config=config)
        if log_dir is not None:
            env = Monitor(env, os.path.join(log_dir, f"env_{rank}"))
        return env
    return _init


# ============================================================
# Main training loop
# ============================================================

def train(args):
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

    # Directories
    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    monitor_dir = os.path.join(args.log_dir, "monitor")
    os.makedirs(monitor_dir, exist_ok=True)

    # Environment config
    env_config = {
        "T": args.T,
        "dt": args.dt,
        "mode": args.mode,
        "N": args.N,
        "max_steps": int(args.T / args.dt) + 10,
        "success_tol": args.success_tol,
        "curriculum": args.curriculum,
        "curriculum_warmup": args.curriculum_warmup,
    }

    # Create vectorized environment
    # NOTE: With HTTP backend, SubprocVecEnv won't help since the backend
    # is single-threaded. Use DummyVecEnv for simplicity.
    if args.num_envs == 1:
        env = DummyVecEnv([make_env(0, args.url, env_config, monitor_dir)])
    else:
        # Multiple envs only useful if you run multiple backend instances
        # on different ports (e.g., 8848, 8849, ...)
        env = DummyVecEnv([
            make_env(i, args.url, env_config, monitor_dir)
            for i in range(args.num_envs)
        ])

    # Eval environment (separate instance)
    eval_env = DummyVecEnv([make_env(99, args.url, env_config, None)])

    # PPO hyperparameters tuned for this task
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
        policy_kwargs=dict(
            net_arch=dict(pi=[256, 256], vf=[256, 256]),
        ),
        tensorboard_log=args.log_dir,
        verbose=1,
        seed=args.seed,
    )

    print(f"\nModel architecture: {model.policy}")
    print(f"Total parameters: {sum(p.numel() for p in model.policy.parameters()):,}")

    # Callbacks
    checkpoint_cb = CheckpointCallback(
        save_freq=max(args.checkpoint_freq // args.num_envs, 1),
        save_path=args.save_dir,
        name_prefix="ur5e_ppo",
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(args.save_dir, "best"),
        log_path=os.path.join(args.log_dir, "eval"),
        eval_freq=max(args.eval_freq // args.num_envs, 1),
        n_eval_episodes=args.eval_episodes,
        deterministic=True,
    )

    callbacks = CallbackList([checkpoint_cb, eval_cb])

    # Train
    print("\nStarting training...\n")
    t0 = time.time()

    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=callbacks,
            progress_bar=True,
        )
    except KeyboardInterrupt:
        print("\n[!] Training interrupted by user.")

    elapsed = time.time() - t0
    print(f"\nTraining completed in {elapsed:.1f}s ({elapsed/60:.1f} min)")

    # Save final model
    final_path = os.path.join(args.save_dir, "ur5e_ppo_final")
    model.save(final_path)
    print(f"Final model saved to: {final_path}.zip")

    env.close()
    eval_env.close()


# ============================================================
# Inference / evaluation
# ============================================================

def evaluate(args):
    """Load a trained model and run evaluation episodes."""
    print(f"Loading model from: {args.model_path}")

    env_config = {
        "T": args.T,
        "dt": args.dt,
        "mode": args.mode,
        "N": args.N,
        "curriculum": False,
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
            ep_reward += reward
            done = terminated or truncated

        total_reward += ep_reward
        success = info.get("success", False)
        if success:
            successes += 1

        eq_rms = info.get('eq_rms')
        eq_str = f"{eq_rms:.4f}" if isinstance(eq_rms, (int, float)) else "?"
        print(f"  Episode {ep+1}/{args.eval_episodes}: "
              f"reward={ep_reward:.3f}, "
              f"success={success}, "
              f"eq_rms={eq_str}")

    avg_reward = total_reward / args.eval_episodes
    success_rate = successes / args.eval_episodes * 100
    print(f"\nAverage reward: {avg_reward:.3f}")
    print(f"Success rate:   {success_rate:.1f}% ({successes}/{args.eval_episodes})")


# ============================================================
# CLI
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="PPO training for UR5e robot arm LQR weight tuning"
    )
    subparsers = parser.add_subparsers(dest="command", help="Command to run")

    # --- Train ---
    train_p = subparsers.add_parser("train", help="Train a PPO agent")
    train_p.add_argument("--url", type=str, default="http://127.0.0.1:8848",
                         help="Backend URL")
    train_p.add_argument("--timesteps", type=int, default=500_000,
                         help="Total training timesteps")
    train_p.add_argument("--num-envs", type=int, default=1,
                         help="Number of parallel environments")
    train_p.add_argument("--seed", type=int, default=42, help="Random seed")

    # PPO hyperparameters
    train_p.add_argument("--lr", type=float, default=3e-4,
                         help="Learning rate")
    train_p.add_argument("--n-steps", type=int, default=2048,
                         help="Steps per rollout")
    train_p.add_argument("--batch-size", type=int, default=256,
                         help="Minibatch size")
    train_p.add_argument("--n-epochs", type=int, default=10,
                         help="PPO update epochs per rollout")
    train_p.add_argument("--gamma", type=float, default=0.99,
                         help="Discount factor")
    train_p.add_argument("--gae-lambda", type=float, default=0.95,
                         help="GAE lambda")
    train_p.add_argument("--clip-range", type=float, default=0.2,
                         help="PPO clip range")
    train_p.add_argument("--ent-coef", type=float, default=0.005,
                         help="Entropy coefficient")

    # Environment
    train_p.add_argument("--T", type=float, default=1.5,
                         help="Trajectory duration (s)")
    train_p.add_argument("--dt", type=float, default=0.02,
                         help="Timestep (s)")
    train_p.add_argument("--mode", type=str, default="mpc_lite",
                         help="Control mode (mpc_lite, lqr, pd)")
    train_p.add_argument("--N", type=int, default=20,
                         help="LQR horizon")
    train_p.add_argument("--success-tol", type=float, default=0.03,
                         help="Success tolerance (rad RMS)")
    train_p.add_argument("--curriculum", action="store_true", default=True,
                         help="Enable curriculum learning")
    train_p.add_argument("--no-curriculum", dest="curriculum", action="store_false")
    train_p.add_argument("--curriculum-warmup", type=int, default=500,
                         help="Episodes for curriculum warmup")

    # Saving
    train_p.add_argument("--save-dir", type=str, default="checkpoints",
                         help="Directory for model checkpoints")
    train_p.add_argument("--log-dir", type=str, default="logs",
                         help="Directory for tensorboard logs")
    train_p.add_argument("--checkpoint-freq", type=int, default=10_000,
                         help="Save checkpoint every N steps")
    train_p.add_argument("--eval-freq", type=int, default=5_000,
                         help="Evaluate every N steps")
    train_p.add_argument("--eval-episodes", type=int, default=10,
                         help="Episodes per evaluation")

    # --- Eval ---
    eval_p = subparsers.add_parser("eval", help="Evaluate a trained model")
    eval_p.add_argument("model_path", type=str, help="Path to .zip model file")
    eval_p.add_argument("--url", type=str, default="http://127.0.0.1:8848")
    eval_p.add_argument("--eval-episodes", type=int, default=20)
    eval_p.add_argument("--T", type=float, default=1.5)
    eval_p.add_argument("--dt", type=float, default=0.02)
    eval_p.add_argument("--mode", type=str, default="mpc_lite")
    eval_p.add_argument("--N", type=int, default=20)

    args = parser.parse_args()

    if args.command == "train":
        train(args)
    elif args.command == "eval":
        evaluate(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
