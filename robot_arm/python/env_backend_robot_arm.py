#!/usr/bin/env python3
"""HTTP wrapper for the Drogon robot-arm backend."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional

import numpy as np
import requests


class RobotArmBackendEnv:
    """
    Minimal Python wrapper around the Drogon backend.

    Observation dim: 25
      [q(6), dq(6), eq(6), edq(6), phase(1)]

    Action dim: 5
      action in [-1,1]^5 -> backend maps to weights:
      [wq, wdq, wu, wqN, wdqN]
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8848", timeout: float = 30.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = float(timeout)
        self.obs_dim = 25
        self.act_dim = 5
        self.session = requests.Session()

    @staticmethod
    def _float_list(values: Iterable[float]) -> list[float]:
        return [float(v) for v in values]

    def reset(
        self,
        q_start,
        q_target,
        T: float = 1.5,
        dt: float = 0.02,
        mode: str = "mpc_lite",
        N: int = 20,
        *,
        u_max: Optional[float] = None,
        max_steps: Optional[int] = None,
        success_tol: Optional[float] = None,
        rw_pos: Optional[float] = None,
        rw_vel: Optional[float] = None,
        rw_u: Optional[float] = None,
        rw_du: Optional[float] = None,
        rw_success: Optional[float] = None,
        friction_scale: Optional[float] = None,
        inertia_scale: Optional[float] = None,
    ):
        payload: Dict[str, Any] = {
            "q_start": self._float_list(q_start),
            "q_target": self._float_list(q_target),
            "T": float(T),
            "dt": float(dt),
            "mode": str(mode),
            "N": int(N),
        }

        optional_fields = {
            "u_max": u_max,
            "max_steps": max_steps,
            "success_tol": success_tol,
            "rw_pos": rw_pos,
            "rw_vel": rw_vel,
            "rw_u": rw_u,
            "rw_du": rw_du,
            "rw_success": rw_success,
            "friction_scale": friction_scale,
            "inertia_scale": inertia_scale,
        }
        for key, value in optional_fields.items():
            if value is not None:
                payload[key] = float(value) if isinstance(value, (int, float, np.floating, np.integer)) else value

        response = self.session.post(
            f"{self.base_url}/rl/reset",
            json=payload,
            timeout=self.timeout,
        )
        response.raise_for_status()
        data = response.json()

        if "obs" not in data:
            raise KeyError("Backend /rl/reset response is missing 'obs'.")

        return np.asarray(data["obs"], dtype=np.float32), data

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] != self.act_dim:
            raise ValueError(f"action must have shape ({self.act_dim},)")

        response = self.session.post(
            f"{self.base_url}/rl/step",
            json={"action": action.tolist()},
            timeout=self.timeout,
        )
        response.raise_for_status()
        data = response.json()

        if "obs" not in data or "reward" not in data or "done" not in data:
            raise KeyError("Backend /rl/step response must include 'obs', 'reward', and 'done'.")

        obs = np.asarray(data["obs"], dtype=np.float32)
        reward = float(data["reward"])
        done = bool(data["done"])

        info = dict(data)
        nested_info = data.get("info")
        if isinstance(nested_info, dict):
            info.update(nested_info)

        return obs, reward, done, info

    def close(self):
        self.session.close()
