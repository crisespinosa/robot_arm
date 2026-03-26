#!/usr/bin/env python3
"""
PPO Inference Server for Unity integration.

Loads a trained PPO model and exposes a simple HTTP API that Unity can call
to get PPO actions from observations. This runs alongside the Drogon backend.

Architecture:
  Unity <-> Drogon backend (/rl/reset, /rl/step) for physics
  Unity <-> This server (/ppo/predict) for PPO inference

Endpoints:
  POST /ppo/predict  - Given obs[25], return action[5]
  POST /ppo/reset    - Reset internal state (optional)
  GET  /ppo/health   - Health check

Usage:
  python ppo_inference_server.py --model checkpoints_v2/ur5e_ppo_v2_final.zip --port 8849
"""

import argparse
import json
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from stable_baselines3 import PPO


class PPOModel:
    """Wrapper around the trained PPO model for thread-safe inference."""

    def __init__(self, model_path):
        print(f"Loading PPO model from: {model_path}")
        self.model = PPO.load(model_path)
        print(f"Model loaded successfully")
        print(f"  Obs space: {self.model.observation_space}")
        print(f"  Act space: {self.model.action_space}")

    def predict(self, obs, deterministic=True):
        """Get action from observation."""
        obs = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        action, _ = self.model.predict(obs, deterministic=deterministic)
        return action.flatten().tolist()


# Global model instance
ppo_model = None


class PPORequestHandler(BaseHTTPRequestHandler):
    """HTTP request handler for PPO inference."""

    def log_message(self, format, *args):
        # Suppress default logging for cleaner output
        pass

    def _send_json(self, status, data):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_OPTIONS(self):
        """Handle CORS preflight."""
        self._send_json(200, {"ok": True})

    def do_GET(self):
        if self.path == "/ppo/health":
            self._send_json(200, {
                "ok": True,
                "model_loaded": ppo_model is not None,
                "obs_dim": 25,
                "act_dim": 5
            })
        else:
            self._send_json(404, {"error": f"Unknown path: {self.path}"})

    def do_POST(self):
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            data = json.loads(body) if body else {}
        except Exception as e:
            self._send_json(400, {"error": f"Bad JSON: {str(e)}"})
            return

        if self.path == "/ppo/predict":
            self._handle_predict(data)
        elif self.path == "/ppo/reset":
            self._handle_reset(data)
        else:
            self._send_json(404, {"error": f"Unknown path: {self.path}"})

    def _handle_predict(self, data):
        """Given observation, return PPO action."""
        global ppo_model

        if ppo_model is None:
            self._send_json(503, {"error": "Model not loaded"})
            return

        obs = data.get("obs")
        if obs is None or len(obs) != 25:
            self._send_json(400, {"error": "obs must be array of length 25"})
            return

        deterministic = data.get("deterministic", True)

        try:
            action = ppo_model.predict(obs, deterministic=deterministic)
            self._send_json(200, {
                "ok": True,
                "action": action
            })
        except Exception as e:
            self._send_json(500, {"error": f"Prediction failed: {str(e)}"})

    def _handle_reset(self, data):
        """Reset any internal state (placeholder for future use)."""
        self._send_json(200, {"ok": True, "message": "Reset acknowledged"})


def main():
    global ppo_model

    parser = argparse.ArgumentParser(description="PPO Inference Server for Unity")
    parser.add_argument("--model", type=str, required=True,
                        help="Path to trained PPO model (.zip)")
    parser.add_argument("--port", type=int, default=8849,
                        help="Server port (default: 8849)")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                        help="Server host")
    args = parser.parse_args()

    # Load model
    ppo_model = PPOModel(args.model)

    # Start server
    server = HTTPServer((args.host, args.port), PPORequestHandler)
    print(f"\nPPO Inference Server running on http://{args.host}:{args.port}")
    print(f"Endpoints:")
    print(f"  POST /ppo/predict  - Get action from observation")
    print(f"  POST /ppo/reset    - Reset internal state")
    print(f"  GET  /ppo/health   - Health check")
    print(f"\nWaiting for Unity requests...\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
