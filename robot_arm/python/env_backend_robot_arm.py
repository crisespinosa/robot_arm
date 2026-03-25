import requests
import numpy as np


class RobotArmBackendEnv:
    """
    Minimal Python env wrapper for the current Drogon backend.

    Observation dim: 25
      [q(6), dq(6), eq(6), edq(6), phase(1)]

    Action dim: 5
      action in [-1,1]^5 -> backend maps to weights:
      [wq, wdq, wu, wqN, wdqN]
    """

    def __init__(self, base_url="http://127.0.0.1:8848"):
        self.base_url = base_url.rstrip("/")
        self.obs_dim = 25
        self.act_dim = 5

    def reset(self, q_start, q_target, T=1.5, dt=0.02, mode="mpc_lite", N=20):
        payload = {
            "q_start": list(map(float, q_start)),
            "q_target": list(map(float, q_target)),
            "T": float(T),
            "dt": float(dt),
            "mode": mode,
            "N": int(N),
        }
        r = requests.post(f"{self.base_url}/rl/reset", json=payload, timeout=30)
        r.raise_for_status()
        data = r.json()
        return np.asarray(data["obs"], dtype=np.float32), data

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] != self.act_dim:
            raise ValueError(f"action must have shape ({self.act_dim},)")
        payload = {"action": action.tolist()}
        r = requests.post(f"{self.base_url}/rl/step", json=payload, timeout=30)
        r.raise_for_status()
        data = r.json()
        obs = np.asarray(data["obs"], dtype=np.float32)
        reward = float(data["reward"])
        done = bool(data["done"])
        return obs, reward, done, data
