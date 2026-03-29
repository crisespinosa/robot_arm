#!/usr/bin/env python3
"""
PPO inference server for Unity integration.

Endpoints:
  POST /ppo/predict  - Given obs[25], return action[5]
  POST /ppo/reset    - Stateless acknowledgement for compatibility
  GET  /ppo/health   - Health check
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import numpy as np
from stable_baselines3 import PPO


class PPOModel:
    """Small thread-safe wrapper around a trained PPO model."""

    def __init__(self, model_path: str):
        print(f"Loading PPO model from: {model_path}")
        self.model = PPO.load(model_path)
        self.lock = threading.Lock()
        print("Model loaded successfully")
        print(f"  Obs space: {self.model.observation_space}")
        print(f"  Act space: {self.model.action_space}")

    def predict(self, obs, deterministic: bool = True) -> list[float]:
        obs = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        with self.lock:
            action, _ = self.model.predict(obs, deterministic=deterministic)
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        return action.tolist()


ppo_model: PPOModel | None = None


class PPORequestHandler(BaseHTTPRequestHandler):
    server_version = "PPOInferenceServer/1.1"

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: int, data: dict[str, Any]) -> None:
        payload = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        self._send_json(200, {"ok": True})

    def do_GET(self):
        if self.path == "/ppo/health":
            loaded = ppo_model is not None
            self._send_json(
                200,
                {
                    "ok": True,
                    "model_loaded": loaded,
                    "obs_dim": 25,
                    "act_dim": 5,
                },
            )
            return
        self._send_json(404, {"ok": False, "error": f"Unknown path: {self.path}"})

    def do_POST(self):
        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            data = json.loads(body) if body else {}
        except Exception as exc:
            self._send_json(400, {"ok": False, "error": f"Bad JSON: {exc}"})
            return

        if self.path == "/ppo/predict":
            self._handle_predict(data)
        elif self.path == "/ppo/reset":
            self._send_json(200, {"ok": True, "message": "Reset acknowledged"})
        else:
            self._send_json(404, {"ok": False, "error": f"Unknown path: {self.path}"})

    def _handle_predict(self, data: dict[str, Any]) -> None:
        global ppo_model

        if ppo_model is None:
            self._send_json(503, {"ok": False, "error": "Model not loaded"})
            return

        obs = data.get("obs")
        if not isinstance(obs, list) or len(obs) != 25:
            self._send_json(400, {"ok": False, "error": "obs must be an array of length 25"})
            return

        deterministic = bool(data.get("deterministic", True))

        try:
            t0 = time.perf_counter()
            action = ppo_model.predict(obs, deterministic=deterministic)
            latency_ms = (time.perf_counter() - t0) * 1000.0
            self._send_json(
                200,
                {
                    "ok": True,
                    "action": action,
                    "latency_ms": latency_ms,
                },
            )
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": f"Prediction failed: {exc}"})


def main():
    global ppo_model

    parser = argparse.ArgumentParser(description="PPO inference server for Unity")
    parser.add_argument("--model", type=str, required=True, help="Path to trained PPO model (.zip)")
    parser.add_argument("--port", type=int, default=8849, help="Server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Server host")
    args = parser.parse_args()

    ppo_model = PPOModel(args.model)

    server = ThreadingHTTPServer((args.host, args.port), PPORequestHandler)
    print(f"\nPPO Inference Server running on http://{args.host}:{args.port}")
    print("Endpoints:")
    print("  POST /ppo/predict  - Get action from observation")
    print("  POST /ppo/reset    - Reset acknowledgement")
    print("  GET  /ppo/health   - Health check")
    print("\nWaiting for Unity requests...\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
