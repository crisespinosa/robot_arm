#!/usr/bin/env python3
"""
HTTP обертка для взаимодействия с бэкендом робототехники Drogon.

Предоставляет Python интерфейс к C++ бэкенду управления UR5e,
реализующему модель LQR со динамической подстройкой весов через PPO.

Пространства:
  Наблюдение (25-D):  [q(6), dq(6), eq(6), edq(6), phase(1)]
    - q: текущие углы суставов
    - dq: угловые скорости
    - eq: ошибки позиции
    - edq: ошибки скорости
    - phase: фаза траектории [0, 1]

  Действие (5-D):    [-1, 1]^5 -> преобразуется в веса LQR:
    - action -> [wq, wdq, wu, wqN, wdqN]
    - wq: вес отслеживания позиции (начальное)
    - wdq: вес отслеживания скорости (начальное)
    - wu: вес энергии управления
    - wqN: вес позиции в конечном состоянии
    - wdqN: вес скорости в конечном состоянии
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional

import numpy as np
import requests


class RobotArmBackendEnv:
    """
    Минимальная Python оболочка вокруг бэкенда управления UR5e.

    Обеспечивает стандартный Gym-подобный интерфейс (reset/step) для
    взаимодействия с C++ бэкендом через HTTP API.

    Наблюдение (25D):
      [q(6), dq(6), eq(6), edq(6), phase(1)]
      - суставные углы, скорости, ошибки позиции, ошибки скорости, фаза

    Действие (5D):
      Массив в [-1, 1]^5, преобразуется бэкендом в веса LQR

    Атрибуты:
        base_url: URL бэкенда (по умолчанию localhost:8848)
        timeout: Таймаут HTTP запросов в секундах
        obs_dim: Размерность пространства наблюдения (всегда 25)
        act_dim: Размерность пространства действия (всегда 5)
        session: Переиспользуемая сессия requests для эффективности
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8848",
                 timeout: float = 30.0) -> None:
        """
        Инициализирует среду с подключением к бэкенду.

        Аргументы:
            base_url: URL бэкенда (например, "http://127.0.0.1:8848")
            timeout: Таймаут для HTTP запросов в секундах
        """
        self.base_url: str = base_url.rstrip("/")
        self.timeout: float = float(timeout)
        self.obs_dim: int = 25
        self.act_dim: int = 5
        # Переиспользуемая сессия для лучшей производительности
        self.session: requests.Session = requests.Session()

    @staticmethod
    def _float_list(values: Iterable[float]) -> list[float]:
        """
        Преобразует итерируемый объект значений в список float.

        Используется для нормализации входных данных перед отправкой
        в JSON запросе на бэкенд.

        Аргументы:
            values: Итерируемый объект числовых значений

        Возвращает:
            Список Python float значений
        """
        return [float(v) for v in values]

    def reset(
        self,
        q_start: Iterable[float],
        q_target: Iterable[float],
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
    ) -> tuple[np.ndarray, Dict[str, Any]]:
        """
        Сбрасывает среду и начинает новый эпизод траектории.

        Отправляет начальную позицию, целевую позицию и параметры
        на бэкенд, инициализирует траекторию и возвращает начальное наблюдение.

        Аргументы:
            q_start: Начальные углы суставов (6D)
            q_target: Целевые углы суставов (6D)
            T: Длительность траектории в секундах (по умолчанию 1.5)
            dt: Временной шаг в секундах (по умолчанию 0.02)
            mode: Режим управления ("mpc_lite", "lqr" и т.д.)
            N: Горизонт прогнозирования для MPC
            u_max: Максимальный момент в Nm (опционально)
            max_steps: Максимум шагов в эпизоде (опционально)
            success_tol: Допуск успеха в радианах (опционально)
            rw_pos: Вес отслеживания позиции (опционально)
            rw_vel: Вес отслеживания скорости (опционально)
            rw_u: Вес энергии управления (опционально)
            rw_du: Вес гладкости управления (опционально)
            rw_success: Бонус за успех (опционально)
            friction_scale: Масштаб трения для domain randomization
            inertia_scale: Масштаб инерции для domain randomization

        Возвращает:
            Кортеж (наблюдение, информация):
            - наблюдение: numpy массив формы (25,) с типом float32
            - информация: словарь с дополнительной информацией от бэкенда
        """
        # Строим основной payload
        payload: Dict[str, Any] = {
            "q_start": self._float_list(q_start),
            "q_target": self._float_list(q_target),
            "T": float(T),
            "dt": float(dt),
            "mode": str(mode),
            "N": int(N),
        }

        # Опциональные поля (добавляем только если не None)
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
                # Конвертируем в float если числовое значение
                payload[key] = (
                    float(value)
                    if isinstance(value, (int, float, np.floating, np.integer))
                    else value
                )

        # Отправляем POST запрос на /rl/reset
        response = self.session.post(
            f"{self.base_url}/rl/reset",
            json=payload,
            timeout=self.timeout,
        )
        response.raise_for_status()
        data = response.json()

        # Проверяем наличие наблюдения в ответе
        if "obs" not in data:
            raise KeyError("Backend /rl/reset response is missing 'obs'.")

        # Возвращаем наблюдение и всю информацию от бэкенда
        return np.asarray(data["obs"], dtype=np.float32), data

    def step(self, action: np.ndarray | list) -> tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """
        Выполняет один шаг в окружении с заданным действием.

        Отправляет действие на бэкенд, получает новое наблюдение,
        награду, флаг завершения и информацию о шаге.

        Аргументы:
            action: Действие размера [5] в диапазоне [-1, 1]

        Возвращает:
            Кортеж (наблюдение, награда, завершено, информация):
            - наблюдение: numpy массив формы (25,) с типом float32
            - награда: float скалярная награда
            - завершено: bool флаг окончания эпизода
            - информация: dict с дополнительными метриками

        Поднимает:
            ValueError: Если размер действия неверный
            KeyError: Если ответ бэкенда некорректен
        """
        # Преобразуем и валидируем действие
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] != self.act_dim:
            raise ValueError(f"action must have shape ({self.act_dim},)")

        # Отправляем POST запрос на /rl/step
        response = self.session.post(
            f"{self.base_url}/rl/step",
            json={"action": action.tolist()},
            timeout=self.timeout,
        )
        response.raise_for_status()
        data = response.json()

        # Проверяем обязательные поля в ответе
        if "obs" not in data or "reward" not in data or "done" not in data:
            raise KeyError(
                "Backend /rl/step response must include 'obs', 'reward', and 'done'."
            )

        # Извлекаем основные компоненты с правильными типами
        obs = np.asarray(data["obs"], dtype=np.float32)
        reward = float(data["reward"])
        done = bool(data["done"])

        # Извлекаем информацию (может содержать вложенные поля)
        info = dict(data)
        nested_info = data.get("info")
        if isinstance(nested_info, dict):
            # Если есть вложенный "info", добавляем его элементы в основной info
            info.update(nested_info)

        return obs, reward, done, info

    def close(self) -> None:
        """
        Закрывает HTTP сессию.

        Вызывается при завершении работы с окружением.
        Освобождает сетевые ресурсы.
        """
        self.session.close()
