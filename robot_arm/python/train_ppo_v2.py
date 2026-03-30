#!/usr/bin/env python3
"""
Обучение PPO v2 для получения результатов качества диссертации.

Реализует систему адаптивной подстройки весов LQR с использованием PPO.

Ключевые особенности:
  - Domain randomization (возмущение трения/инерции через /rl/reset)
  - Шум наблюдений для повышения робастности
  - 3-фазное curriculum learning (постепенное усложнение задачи)
  - Локальное reward shaping для поддержания низкой ошибки отслеживания
  - Логирование метрик в TensorBoard
  - Кастомный callback для сохранения лучшей модели по успешности

Архитектура PPO:
  - Входные данные: 25-D наблюдение [q, dq, eq, edq, phase]
  - Выходные данные: 5-D действие [-1, 1]
  - Сеть: MLP с двумя слоями по 256 нейронов
  - Оптимизатор: Adam с schedulerom learning rate

Curriculum phases:
  Phase 1: Легкие задачи (малый диапазон конфигураций)
  Phase 2: Средние задачи (расширенный диапазон)
  Phase 3: Полные задачи (весь диапазон с максимальной сложностью)
"""

from __future__ import annotations

import argparse
import os
import time
from typing import Any, Callable, Dict, List

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
    Улучшенная Gym среда для обучения UR5e с адаптивным управлением.

    Включает:
      - Domain randomization (возмущения трения/инерции на каждом reset)
      - Шум наблюдений (имитация шума датчиков)
      - 3-фазное curriculum learning (постепенное усложнение)
      - Локальное reward shaping для отслеживания низких ошибок
      - Гибкие конфигурации через словарь config

    Наблюдение: 25-D вектор [q(6), dq(6), eq(6), edq(6), phase(1)]
    Действие: 5-D вектор [-1, 1] (преобразуется в веса LQR)

    Атрибуты:
        backend: Объект RobotArmBackendEnv для взаимодействия с бэкендом
        config: Словарь параметров конфигурации
        observation_space: Box пространство наблюдения (25,)
        action_space: Box пространство действия (5,)
        episode_count: Счетчик завершенных эпизодов (для curriculum)
        _current_obs: Последнее наблюдение
        _step_in_episode: Текущий шаг в эпизоде
        _low_error_streak: Число последовательных шагов с низкой ошибкой
        _last_randomization: Последние параметры domain randomization
    """

    metadata = {"render_modes": []}

    def __init__(self, base_url: str = "http://127.0.0.1:8848",
                 config: dict | None = None) -> None:
        """
        Инициализирует UR5eGymEnvV2 с заданными параметрами.

        Аргументы:
            base_url: URL бэкенда C++ (по умолчанию localhost:8848)
            config: Словарь параметров конфигурации, включая:
                    - T, dt, mode, N, u_max, max_steps, success_tol: параметры управления
                    - rw_pos, rw_vel, rw_u, rw_du, rw_success: веса награды
                    - domain_rand, rand_friction_range, rand_inertia_range: DR параметры
                    - obs_noise_std: стандартное отклонение шума наблюдений
                    - curriculum, phase1_episodes, phase2_episodes: параметры curriculum
                    - low_error_threshold, low_error_bonus, low_error_streak_target: shaping
        """
        super().__init__()

        # Инициализируем бэкенд
        self.backend = RobotArmBackendEnv(base_url=base_url)
        self.config = config or {}

        # Определяем пространства наблюдений и действий
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

        # Параметры управления траекторией
        self.T: float = float(self.config.get("T", 1.5))
        self.dt: float = float(self.config.get("dt", 0.02))
        self.mode: str = str(self.config.get("mode", "mpc_lite"))
        self.N: int = int(self.config.get("N", 20))
        self.u_max: float = float(self.config.get("u_max", 8.0))
        self.max_steps: int = int(self.config.get("max_steps", int(self.T / self.dt) + 10))
        self.success_tol: float = float(self.config.get("success_tol", 0.03))

        # Веса для функции награды
        self.rw_pos: float = self.config.get("rw_pos", 2.0)
        self.rw_vel: float = self.config.get("rw_vel", 0.15)
        self.rw_u: float = self.config.get("rw_u", 0.01)
        self.rw_du: float = self.config.get("rw_du", 0.005)
        self.rw_success: float = self.config.get("rw_success", 5.0)

        # Диапазоны суставов для sampling конфигураций
        self.q_range: list[tuple[float, float]] = self.config.get(
            "q_range",
            [
                (-1.5, 1.5), (-1.5, 0.5), (-2.0, 0.5),
                (-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5),
            ],
        )

        # Domain randomization параметры
        self.domain_rand: bool = bool(self.config.get("domain_rand", True))
        self.rand_friction_range: float = float(self.config.get("rand_friction_range", 0.2))
        self.rand_inertia_range: float = float(self.config.get("rand_inertia_range", 0.15))

        # Шум наблюдений
        self.obs_noise_std: float = float(self.config.get("obs_noise_std", 0.005))

        # Curriculum learning параметры
        self.curriculum_enabled: bool = bool(self.config.get("curriculum", True))
        self.episode_count: int = 0
        self.phase1_episodes: int = max(1, int(self.config.get("phase1_episodes", 300)))
        self.phase2_episodes: int = max(
            self.phase1_episodes + 1, int(self.config.get("phase2_episodes", 800))
        )

        # Параметры reward shaping для низкой ошибки
        self.low_error_threshold: float = float(self.config.get("low_error_threshold", 0.05))
        self.low_error_bonus: float = float(self.config.get("low_error_bonus", 0.02))
        self.low_error_streak_target: int = max(
            1, int(self.config.get("low_error_streak_target", 5))
        )

        # Внутреннее состояние
        self._current_obs: np.ndarray | None = None
        self._step_in_episode: int = 0
        self._low_error_streak: int = 0
        self._last_randomization: Dict[str, float] = {
            "friction_scale": 1.0,
            "inertia_scale": 1.0,
        }

    def _get_curriculum_scale(self) -> float:
        """
        Вычисляет масштаб сложности на основе текущей фазы curriculum.

        3 фазы:
        - Phase 1 (0 к phase1_episodes): масштаб от 0.15 к 0.40
        - Phase 2 (phase1 к phase2): масштаб от 0.40 к 0.80
        - Phase 3 (после phase2): масштаб от 0.80 к 1.0

        Возвращает:
            float: Масштаб сложности в [0, 1], используется для конфигураций
        """
        if not self.curriculum_enabled:
            return 1.0

        # Phase 1: постепенный переход от 0.15 к 0.40
        if self.episode_count < self.phase1_episodes:
            progress = self.episode_count / self.phase1_episodes
            return 0.15 + 0.25 * progress

        # Phase 2: постепенный переход от 0.40 к 0.80
        if self.episode_count < self.phase2_episodes:
            progress = (self.episode_count - self.phase1_episodes) / (
                self.phase2_episodes - self.phase1_episodes
            )
            return 0.40 + 0.40 * progress

        # Phase 3: постепенный переход от 0.80 к 1.0
        remaining = self.episode_count - self.phase2_episodes
        return min(1.0, 0.80 + 0.20 * (remaining / 200.0))

    def _sample_config(self, scale: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Семплирует случайные начальную и целевую конфигурации.

        Использует масштаб сложности для определения диапазонов.
        Гарантирует, что начальная и целевая позиции достаточно далеки.

        Аргументы:
            scale: Масштаб сложности [0, 1], определяет размер диапазонов

        Возвращает:
            Кортеж (q_start, q_target) - начальная и целевая конфигурации
        """
        # Целевая конфигурация: полный диапазон, масштабированный
        q_target = np.zeros(6, dtype=np.float64)
        for i, (lo, hi) in enumerate(self.q_range):
            mid = (lo + hi) / 2.0
            half_range = (hi - lo) / 2.0 * scale
            q_target[i] = np.random.uniform(mid - half_range, mid + half_range)

        # Начальная конфигурация: часто в начале, редко = целевое
        q_start = np.zeros(6, dtype=np.float64)
        if np.random.random() > 0.25:  # 75% вероятность samplerить начало
            for i, (lo, hi) in enumerate(self.q_range):
                mid = (lo + hi) / 2.0
                half_range = (hi - lo) / 2.0 * scale * 0.5  # Меньше диапазон для начала
                q_start[i] = np.random.uniform(mid - half_range, mid + half_range)

        # Убедимся, что начало и цель достаточно далеки
        while np.linalg.norm(q_target - q_start) < 0.1:
            for i, (lo, hi) in enumerate(self.q_range):
                mid = (lo + hi) / 2.0
                half_range = (hi - lo) / 2.0 * scale
                q_target[i] = np.random.uniform(mid - half_range, mid + half_range)

        return q_start, q_target

    def _domain_randomization_params(self) -> Dict[str, float]:
        """
        Генерирует случайные параметры domain randomization.

        Если domain_rand отключен, возвращает neutral значения (1.0, 1.0).
        Иначе семплирует возмущения для трения и инерции.

        Возвращает:
            Словарь с ключами:
            - friction_scale: масштаб трения [0.8, 1.2] примерно
            - inertia_scale: масштаб инерции [0.85, 1.15] примерно
        """
        if not self.domain_rand:
            return {"friction_scale": 1.0, "inertia_scale": 1.0}

        # Семплируем возмущения в диапазоне [-range, +range] от 1.0
        friction_scale = 1.0 + np.random.uniform(
            -self.rand_friction_range, self.rand_friction_range
        )
        inertia_scale = 1.0 + np.random.uniform(
            -self.rand_inertia_range, self.rand_inertia_range
        )

        # Убедимся, что масштабы > 0.2 (разумный минимум)
        return {
            "friction_scale": float(max(0.2, friction_scale)),
            "inertia_scale": float(max(0.2, inertia_scale)),
        }

    def _maybe_add_obs_noise(self, obs: np.ndarray) -> np.ndarray:
        """
        Добавляет Гауссовский шум к наблюдению (если включен).

        Имитирует шум датчиков для повышения робастности политики.

        Аргументы:
            obs: Наблюдение

        Возвращает:
            Наблюдение с добавленным шумом (если obs_noise_std > 0)
        """
        obs = np.asarray(obs, dtype=np.float32)
        if self.obs_noise_std > 0.0:
            # Добавляем Гауссовский шум с заданным std
            obs = obs + np.random.normal(0.0, self.obs_noise_std, size=obs.shape).astype(
                np.float32
            )
        return obs

    def reset(self, seed: int | None = None,
              options: dict | None = None) -> tuple[np.ndarray, Dict[str, Any]]:
        """
        Сбрасывает среду для нового эпизода.

        Семплирует новую конфигурацию, параметры DR, вызывает бэкенд reset,
        добавляет шум, и инициализирует внутреннее состояние.

        Аргументы:
            seed: Seed для PRNG (соответствует Gym API)
            options: Опции (соответствует Gym API)

        Возвращает:
            Кортеж (наблюдение, информация)
        """
        super().reset(seed=seed)

        # Получаем масштаб для curriculum текущей фазы
        scale = self._get_curriculum_scale()

        # Семплируем начальную и целевую конфигурации
        q_start, q_target = self._sample_config(scale)

        # Генерируем параметры domain randomization
        dr_params = self._domain_randomization_params()

        try:
            # Вызываем бэкенд reset с полными параметрами
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
            # Обработка ошибок бэкенда - возвращаем нулевое наблюдение
            print(f"[UR5eGymEnvV2] Backend reset failed: {exc}")
            obs = np.zeros(self.backend.obs_dim, dtype=np.float32)
            info = {"error": str(exc)}

        # Добавляем параметры DR в info
        info = dict(info)
        info.update(dr_params)

        # Добавляем шум наблюдениям
        obs = self._maybe_add_obs_noise(obs)

        # Инициализируем внутреннее состояние
        self._current_obs = obs
        self._step_in_episode = 0
        self._low_error_streak = 0
        self._last_randomization = dr_params
        self.episode_count += 1

        return obs, info

    def step(self, action: np.ndarray | list) -> tuple[
        np.ndarray, float, bool, bool, Dict[str, Any]
    ]:
        """
        Выполняет один шаг управления в окружении.

        Обрезает действие, отправляет на бэкенд, обрабатывает reward shaping
        для низкой ошибки, добавляет шум, проверяет termination/truncation.

        Аргументы:
            action: Действие [5] в [-1, 1]

        Возвращает:
            Кортеж (obs, reward, terminated, truncated, info):
            - obs: новое наблюдение с шумом
            - reward: скалярная награда с возможным бонусом за низкую ошибку
            - terminated: True если успешно завершено (success=True)
            - truncated: True если превышен max_steps или ошибка бэкенда
            - info: словарь с метриками и параметрами DR
        """
        # Обрезаем действие в допустимый диапазон
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        try:
            # Отправляем действие на бэкенд
            obs, reward, done, info = self.backend.step(action)
        except Exception as exc:
            # Обработка ошибок бэкенда
            print(f"[UR5eGymEnvV2] Backend step failed: {exc}")
            obs = (
                self._current_obs
                if self._current_obs is not None
                else np.zeros(self.backend.obs_dim, dtype=np.float32)
            )
            reward = -1.0
            done = True
            info = {"error": str(exc), "truncated": True, "success": False}

        # Добавляем параметры DR в info
        info = dict(info)
        info.update(self._last_randomization)

        # Reward shaping: бонус за удержание низкой ошибки на протяжении streak
        eq_rms = info.get("eq_rms")
        if isinstance(eq_rms, (int, float, np.integer, np.floating)):
            if float(eq_rms) <= self.low_error_threshold:
                # Ошибка низкая - увеличиваем streak
                self._low_error_streak += 1
                if self._low_error_streak >= self.low_error_streak_target:
                    # Достигли целевого streak - добавляем бонус
                    reward += self.low_error_bonus
                    info["low_error_bonus_applied"] = True
            else:
                # Ошибка не низкая - сбрасываем streak
                self._low_error_streak = 0
        else:
            self._low_error_streak = 0

        # Добавляем шум к наблюдению
        obs = self._maybe_add_obs_noise(obs)

        # Обновляем внутреннее состояние
        self._current_obs = obs
        self._step_in_episode += 1

        # Определяем terminated/truncated на основе информации от бэкенда
        backend_success = bool(info.get("success", False))
        backend_truncated = bool(info.get("truncated", False))

        terminated = bool(done and backend_success)  # Успешное завершение
        truncated = bool(done and not backend_success) or backend_truncated  # Неудача или truncate

        # Проверяем превышение max_steps
        if not (terminated or truncated) and self._step_in_episode >= self.max_steps:
            truncated = True
            info["truncated"] = True
            info["forced_truncation"] = True

        return obs, float(reward), terminated, truncated, info

    def close(self) -> None:
        """
        Закрывает окружение и освобождает ресурсы.

        Закрывает сессию бэкенда.
        """
        self.backend.close()
        super().close()


class TrainingMetricsCallback(BaseCallback):
    """
    Callback для логирования дополнительных метрик обучения в TensorBoard.

    Отслеживает награды эпизодов, длины, успешность, ошибки и параметры.
    """

    def __init__(self, log_every_steps: int = 2048, verbose: int = 0) -> None:
        """
        Инициализирует callback.

        Аргументы:
            log_every_steps: Логировать learning rate каждые N шагов
            verbose: Уровень выводимой информации
        """
        super().__init__(verbose)
        self.log_every_steps: int = max(1, int(log_every_steps))
        self.ep_rewards: List[float] = []
        self.ep_lengths: List[int] = []

    def _on_training_start(self) -> None:
        """
        Инициализирует аккумуляторы при начале обучения.

        Создает отдельные счетчики для каждого окружения в vectorized setup.
        """
        n_envs = int(getattr(self.training_env, "num_envs", 1))
        self.ep_rewards = [0.0] * n_envs
        self.ep_lengths = [0] * n_envs

    def _on_step(self) -> bool:
        """
        Вызывается на каждом шаге обучения.

        Логирует метрики завершенных эпизодов и периодически learning rate.

        Возвращает:
            True для продолжения обучения
        """
        rewards = self.locals.get("rewards", [])
        dones = self.locals.get("dones", [])
        infos = self.locals.get("infos", [])

        # Обновляем счетчики для каждого окружения
        for i in range(len(dones)):
            self.ep_rewards[i] += float(rewards[i])
            self.ep_lengths[i] += 1
            info = infos[i] if i < len(infos) else {}

            # Если эпизод завершен, логируем метрики
            if dones[i]:
                self.logger.record("custom/episode_reward", self.ep_rewards[i])
                self.logger.record("custom/episode_length", self.ep_lengths[i])
                self.logger.record(
                    "custom/episode_success", float(bool(info.get("success", False)))
                )

                # Логируем метрики из info если доступны
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

                # Сбрасываем счетчики
                self.ep_rewards[i] = 0.0
                self.ep_lengths[i] = 0

        # Периодически логируем learning rate
        if self.num_timesteps % self.log_every_steps == 0:
            self.logger.record(
                "custom/learning_rate",
                float(self.model.lr_schedule(self.model._current_progress_remaining)),
            )

        return True


class SuccessRateEvalCallback(BaseCallback):
    """
    Callback для оценки политики и сохранения лучшей модели по успешности.

    Оценивает на чистой среде (без DR и шума) и сохраняет модель,
    которая максимизирует success rate, используя среднюю награду как tiebreaker.
    """

    def __init__(
        self,
        eval_env: UR5eGymEnvV2,
        eval_freq: int,
        n_eval_episodes: int,
        best_model_save_path: str,
        deterministic: bool = True,
        verbose: int = 0,
    ) -> None:
        """
        Инициализирует callback.

        Аргументы:
            eval_env: Окружение для оценки (чистое, без DR)
            eval_freq: Оценивать каждые N шагов
            n_eval_episodes: Количество эпизодов для оценки
            best_model_save_path: Директория для сохранения лучшей модели
            deterministic: Использовать детерминированное действие при оценке
            verbose: Уровень выводимой информации
        """
        super().__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = max(1, int(eval_freq))
        self.n_eval_episodes = max(1, int(n_eval_episodes))
        self.best_model_save_path = best_model_save_path
        self.deterministic = deterministic
        self.best_success_rate: float = -np.inf
        self.best_avg_reward: float = -np.inf
        os.makedirs(self.best_model_save_path, exist_ok=True)

    def _evaluate(self) -> tuple[float, float, float]:
        """
        Выполняет оценку политики на n_eval_episodes.

        Собирает статистику успешности, средней награды и ошибок.

        Возвращает:
            Кортеж (avg_reward, success_rate, avg_eq_rms)
        """
        total_reward: float = 0.0
        successes: int = 0
        eq_rms_values: List[float] = []

        for _ in range(self.n_eval_episodes):
            obs, info = self.eval_env.reset()
            done = False
            ep_reward: float = 0.0
            last_info = info

            # Выполняем эпизод
            while not done:
                action, _ = self.model.predict(obs, deterministic=self.deterministic)
                obs, reward, terminated, truncated, last_info = self.eval_env.step(action)
                ep_reward += float(reward)
                done = bool(terminated or truncated)

            # Собираем статистику
            total_reward += ep_reward
            if bool(last_info.get("success", False)):
                successes += 1

            eq_rms = last_info.get("eq_rms")
            if isinstance(eq_rms, (int, float, np.integer, np.floating)):
                eq_rms_values.append(float(eq_rms))

        # Вычисляем средние значения
        avg_reward = total_reward / self.n_eval_episodes
        success_rate = successes / self.n_eval_episodes
        avg_eq_rms = float(np.mean(eq_rms_values)) if eq_rms_values else float("nan")

        return avg_reward, success_rate, avg_eq_rms

    def _on_step(self) -> bool:
        """
        Вызывается на каждом шаге обучения.

        Проверяет, пришло ли время для оценки, и сохраняет лучшую модель.

        Возвращает:
            True для продолжения обучения
        """
        if self.num_timesteps % self.eval_freq != 0:
            return True

        # Выполняем оценку
        avg_reward, success_rate, avg_eq_rms = self._evaluate()

        # Логируем результаты
        self.logger.record("eval_custom/avg_reward", avg_reward)
        self.logger.record("eval_custom/success_rate", success_rate)
        if not np.isnan(avg_eq_rms):
            self.logger.record("eval_custom/avg_eq_rms", avg_eq_rms)

        # Проверяем улучшение: сначала success_rate, затем avg_reward
        is_better = (
            success_rate > self.best_success_rate
            or (np.isclose(success_rate, self.best_success_rate) and avg_reward > self.best_avg_reward)
        )

        if is_better:
            # Сохраняем новую лучшую модель
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


def make_env_v2(rank: int, base_url: str, config: dict,
                log_dir: str | None) -> Callable[[], gym.Env]:
    """
    Factory функция для создания окружений в vectorized setup.

    Позволяет создавать независимые окружения для DummyVecEnv.

    Аргументы:
        rank: Индекс окружения (для uniqueness)
        base_url: URL бэкенда
        config: Конфигурация окружения
        log_dir: Директория для логирования (или None)

    Возвращает:
        Функция которая создает новое UR5eGymEnvV2
    """
    def _init() -> gym.Env:
        env = UR5eGymEnvV2(base_url=base_url, config=config)
        if log_dir is not None:
            env = Monitor(env, os.path.join(log_dir, f"env_{rank}"))
        return env

    return _init


def build_env_config(args: argparse.Namespace) -> Dict[str, Any]:
    """
    Строит словарь конфигурации из аргументов командной строки.

    Аргументы:
        args: Распарсенные аргументы

    Возвращает:
        Словарь с параметрами конфигурации для UR5eGymEnvV2
    """
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


def train(args: argparse.Namespace) -> None:
    """
    Выполняет обучение PPO модели.

    Создает окружения (training + evaluation), инициализирует модель PPO,
    настраивает callbacks, и запускает обучение.

    Аргументы:
        args: Распарсенные аргументы командной строки
    """
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

    # Создаем директории
    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    monitor_dir = os.path.join(args.log_dir, "monitor")
    os.makedirs(monitor_dir, exist_ok=True)

    # Строим конфигурацию для training окружения
    env_config = build_env_config(args)

    # Создаем training окружение с DR и шумом
    env = DummyVecEnv([make_env_v2(0, args.url, env_config, monitor_dir)])

    # Строим конфигурацию для evaluation окружения (без DR и шума)
    eval_config = dict(env_config)
    eval_config["domain_rand"] = False
    eval_config["obs_noise_std"] = 0.0
    eval_config["curriculum"] = False

    # Создаем evaluation окружение
    eval_monitor_dir = os.path.join(args.log_dir, "eval_monitor")
    os.makedirs(eval_monitor_dir, exist_ok=True)
    eval_env_vec = DummyVecEnv([make_env_v2(99, args.url, eval_config, eval_monitor_dir)])

    # Инициализируем PPO модель
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

    # Создаем callbacks
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
        # Запускаем обучение
        model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    except KeyboardInterrupt:
        print("\n[!] Training interrupted by user.")

    # Выводим статистику обучения
    elapsed = time.time() - t0
    print(f"\nTraining completed in {elapsed:.1f}s ({elapsed/60:.1f} min)")

    # Сохраняем финальную модель
    final_path = os.path.join(args.save_dir, "ur5e_ppo_v2_final")
    model.save(final_path)
    print(f"Final model saved to: {final_path}.zip")

    # Закрываем окружения
    env.close()
    eval_env_vec.close()


def evaluate(args: argparse.Namespace) -> None:
    """
    Выполняет оценку обученной модели на заданном количестве эпизодов.

    Загружает модель, запускает эпизоды в чистой среде (без DR),
    и выводит статистику успешности и метрик.

    Аргументы:
        args: Распарсенные аргументы командной строки
    """
    print(f"Loading model from: {args.model_path}")

    # Строим конфигурацию для evaluation окружения
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

    # Создаем окружение и загружаем модель
    env = UR5eGymEnvV2(base_url=args.url, config=env_config)
    model = PPO.load(args.model_path)

    # Выполняем оценку
    total_reward: float = 0.0
    successes: int = 0

    for ep in range(args.eval_episodes):
        obs, info = env.reset()
        ep_reward: float = 0.0
        done = False

        # Запускаем эпизод
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_reward += float(reward)
            done = bool(terminated or truncated)

        # Собираем статистику
        total_reward += ep_reward
        success = bool(info.get("success", False))
        if success:
            successes += 1

        # Выводим информацию об эпизоде
        eq_rms = info.get("eq_rms")
        eq_str = (
            f"{eq_rms:.4f}"
            if isinstance(eq_rms, (int, float, np.integer, np.floating))
            else "?"
        )
        print(
            f"  Episode {ep + 1}/{args.eval_episodes}: "
            f"reward={ep_reward:.3f}, success={success}, eq_rms={eq_str}"
        )

    # Выводим финальную статистику
    avg_reward = total_reward / max(args.eval_episodes, 1)
    success_rate = successes / max(args.eval_episodes, 1) * 100.0
    print(f"\nAverage reward: {avg_reward:.3f}")
    print(f"Success rate:   {success_rate:.1f}% ({successes}/{args.eval_episodes})")


def add_common_env_args(parser: argparse.ArgumentParser) -> None:
    """
    Добавляет общие аргументы управления окружением в parser.

    Аргументы:
        parser: ArgumentParser для добавления аргументов
    """
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


def main() -> None:
    """
    Главная функция: парсит аргументы и запускает обучение или оценку.

    Поддерживает два режима:
    - train: Обучение новой модели
    - eval: Оценка существующей модели
    """
    parser = argparse.ArgumentParser(
        description="PPO v2 training for UR5e with real domain randomization"
    )
    subparsers = parser.add_subparsers(dest="command")

    # Subparser для training
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

    # Subparser для evaluation
    ep = subparsers.add_parser("eval")
    ep.add_argument("model_path", type=str)
    ep.add_argument("--eval-episodes", type=int, default=30)
    ep.add_argument("--low-error-threshold", type=float, default=0.05)
    ep.add_argument("--low-error-bonus", type=float, default=0.02)
    ep.add_argument("--low-error-streak", type=int, default=5)
    add_common_env_args(ep)

    args = parser.parse_args()

    # Запускаем команду
    if args.command == "train":
        train(args)
    elif args.command == "eval":
        evaluate(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
