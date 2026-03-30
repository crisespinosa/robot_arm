#!/usr/bin/env python3
"""
Сервер для инференса PPO модели с интеграцией Unity.

Обеспечивает HTTP API для предсказания действий управления на основе наблюдений.
Используется для адаптивной настройки весов LQR в реальном времени.

Endpoints:
  POST /ppo/predict  - Получить действие по наблюдению [25] -> действие [5]
  POST /ppo/reset    - Подтверждение сброса (без состояния)
  GET  /ppo/health   - Проверка здоровья сервера

Архитектура:
  - Загружает обученную PPO модель (stable-baselines3)
  - Предоставляет потокобезопасный интерфейс предсказания
  - Поддерживает детерминированные предсказания для инференса
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
    """
    Потокобезопасная оболочка вокруг обученной PPO модели.

    Управляет загрузкой модели и обеспечивает синхронизированный доступ
    к методу предсказания для избежания состояния гонки в многопоточной среде.

    Атрибуты:
        model: Загруженная PPO модель из stable-baselines3
        lock: Мьютекс для синхронизации доступа к модели
    """

    def __init__(self, model_path: str) -> None:
        """
        Инициализирует PPOModel и загружает модель из файла.

        Аргументы:
            model_path: Путь к сохраненной PPO модели (.zip файл)
        """
        print(f"Loading PPO model from: {model_path}")
        self.model: PPO = PPO.load(model_path)
        self.lock: threading.Lock = threading.Lock()
        print("Model loaded successfully")
        print(f"  Obs space: {self.model.observation_space}")
        print(f"  Act space: {self.model.action_space}")

    def predict(self, obs: np.ndarray | list, deterministic: bool = True) -> list[float]:
        """
        Предсказывает действие для данного наблюдения.

        Используется потокобезопасный доступ к модели. Наблюдение
        преобразуется в правильный формат, подается на модель,
        и результат возвращается как список.

        Аргументы:
            obs: Наблюдение размера [25] (q, dq, eq, edq, phase)
            deterministic: Использовать ли детерминированное предсказание
                          (без стохастичности)

        Возвращает:
            Список из 5 действий в диапазоне [-1, 1]
        """
        # Преобразуем наблюдение в numpy массив и переформатируем в батч
        obs = np.asarray(obs, dtype=np.float32).reshape(1, -1)

        # Потокобезопасное предсказание
        with self.lock:
            action, _ = self.model.predict(obs, deterministic=deterministic)

        # Преобразуем действие в список и возвращаем
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        return action.tolist()


# Глобальный объект модели (инициализируется в main())
ppo_model: PPOModel | None = None


class PPORequestHandler(BaseHTTPRequestHandler):
    """
    HTTP обработчик запросов для инференса PPO.

    Обрабатывает три типа запросов:
    1. GET /ppo/health - проверка статуса сервера
    2. POST /ppo/predict - предсказание действия
    3. POST /ppo/reset - подтверждение сброса (stateless)

    Все ответы отправляются в формате JSON с поддержкой CORS.
    """

    server_version = "PPOInferenceServer/1.1"

    def log_message(self, format: str, *args: Any) -> None:
        """
        Подавляет стандартное логирование HTTP запросов.

        Аргументы:
            format: Формат сообщения
            *args: Аргументы формата
        """
        # Не логируем стандартные HTTP сообщения
        return

    def _send_json(self, status: int, data: dict[str, Any]) -> None:
        """
        Отправляет JSON ответ с правильными заголовками.

        Устанавливает статус код, Content-Type и CORS заголовки,
        затем отправляет JSON данные.

        Аргументы:
            status: HTTP статус код (200, 400, 500 и т.д.)
            data: Словарь данных для кодирования в JSON
        """
        # Кодируем данные в JSON
        payload = json.dumps(data).encode("utf-8")

        # Отправляем HTTP ответ с заголовками
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))

        # Добавляем CORS заголовки для интеграции с Unity
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

        # Отправляем тело ответа
        self.wfile.write(payload)

    def do_OPTIONS(self) -> None:
        """
        Обрабатывает preflight CORS запросы.

        Используется браузерами/Unity для проверки CORS перед
        отправкой фактического запроса.
        """
        self._send_json(200, {"ok": True})

    def do_GET(self) -> None:
        """
        Обрабатывает GET запросы (только health check).

        GET /ppo/health возвращает статус сервера и информацию о модели.
        """
        if self.path == "/ppo/health":
            # Проверяем загруженность модели
            loaded = ppo_model is not None
            self._send_json(
                200,
                {
                    "ok": True,
                    "model_loaded": loaded,
                    "obs_dim": 25,      # Размерность наблюдения
                    "act_dim": 5,       # Размерность действия
                },
            )
            return

        # Неизвестный путь
        self._send_json(404, {"ok": False, "error": f"Unknown path: {self.path}"})

    def do_POST(self) -> None:
        """
        Обрабатывает POST запросы (predict и reset).

        Читает JSON тело запроса, разбирает его и маршрутизирует
        на соответствующий обработчик.
        """
        try:
            # Читаем тело запроса
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            data = json.loads(body) if body else {}
        except Exception as exc:
            # Ошибка парсинга JSON
            self._send_json(400, {"ok": False, "error": f"Bad JSON: {exc}"})
            return

        # Маршрутизируем запрос
        if self.path == "/ppo/predict":
            self._handle_predict(data)
        elif self.path == "/ppo/reset":
            # Reset - просто подтверждаем (без состояния)
            self._send_json(200, {"ok": True, "message": "Reset acknowledged"})
        else:
            self._send_json(404, {"ok": False, "error": f"Unknown path: {self.path}"})

    def _handle_predict(self, data: dict[str, Any]) -> None:
        """
        Обрабатывает запрос предсказания /ppo/predict.

        Проверяет формат наблюдения (должно быть массив из 25 элементов),
        вызывает модель и возвращает действие с метриками задержки.

        Аргументы:
            data: Разобранное JSON тело запроса с полями:
                  - obs: Массив наблюдения размера [25]
                  - deterministic: Булево значение (опционально)
        """
        global ppo_model

        # Проверяем загруженность модели
        if ppo_model is None:
            self._send_json(503, {"ok": False, "error": "Model not loaded"})
            return

        # Получаем наблюдение из запроса
        obs = data.get("obs")
        if not isinstance(obs, list) or len(obs) != 25:
            # Неверный формат наблюдения
            self._send_json(400, {"ok": False, "error": "obs must be an array of length 25"})
            return

        # Получаем флаг детерминированности (по умолчанию True)
        deterministic = bool(data.get("deterministic", True))

        try:
            # Выполняем предсказание с измерением задержки
            t0 = time.perf_counter()
            action = ppo_model.predict(obs, deterministic=deterministic)
            latency_ms = (time.perf_counter() - t0) * 1000.0

            # Отправляем успешный ответ
            self._send_json(
                200,
                {
                    "ok": True,
                    "action": action,           # Действие [5]
                    "latency_ms": latency_ms,   # Задержка в миллисекундах
                },
            )
        except Exception as exc:
            # Ошибка при предсказании
            self._send_json(500, {"ok": False, "error": f"Prediction failed: {exc}"})


def main() -> None:
    """
    Главная функция: парсит аргументы, загружает модель и запускает сервер.

    Создает ThreadingHTTPServer с PPORequestHandler и запускает его
    в блокирующем режиме (serve_forever).
    """
    global ppo_model

    # Парсим аргументы командной строки
    parser = argparse.ArgumentParser(description="PPO inference server for Unity")
    parser.add_argument("--model", type=str, required=True,
                        help="Path to trained PPO model (.zip)")
    parser.add_argument("--port", type=int, default=8849,
                        help="Server port")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                        help="Server host")
    args = parser.parse_args()

    # Загружаем модель
    ppo_model = PPOModel(args.model)

    # Создаем и запускаем сервер
    server = ThreadingHTTPServer((args.host, args.port), PPORequestHandler)
    print(f"\nPPO Inference Server running on http://{args.host}:{args.port}")
    print("Endpoints:")
    print("  POST /ppo/predict  - Get action from observation")
    print("  POST /ppo/reset    - Reset acknowledgement")
    print("  GET  /ppo/health   - Health check")
    print("\nWaiting for Unity requests...\n")

    try:
        # Запускаем сервер (блокирует до прерывания)
        server.serve_forever()
    except KeyboardInterrupt:
        # Обработка Ctrl+C
        print("\nServer stopped.")
        server.server_close()


if __name__ == "__main__":
    main()
