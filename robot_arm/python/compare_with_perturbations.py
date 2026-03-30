#!/usr/bin/env python3
"""
Сравнительная оценка робастности под действием возмущений.

Тестирует адаптивность и робастность фиксированного LQR и PPO
под реалистичными возмущениями:

  1. Шум момента (имитация неточности актуаторов)
  2. Изменение нагрузки (имитация поднятия объекта в процессе)
  3. Шум датчиков (имитация шума энкодеров)
  4. Внешние импульсы (имитация столкновений)
  5. Комбинированные возмущения (все вместе)

Возмущения применяются на Python уровне путем модификации действий/наблюдений.
Это имитирует реальные условия без необходимости перекомпиляции C++ бэкенда.

Использование:
  python compare_with_perturbations.py --model-v1 checkpoints/ur5e_ppo_final.zip \
                                        --model-v2 checkpoints_v2/ur5e_ppo_v2_final.zip \
                                        --episodes 30
"""

import argparse
import os
from typing import Any, Dict, List, Optional

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend для сохранения фигур
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from env_backend_robot_arm import RobotArmBackendEnv


# ============================================================
# Классы возмущений
# ============================================================

class Perturbation:
    """
    Базовый класс для возмущений.

    Определяет интерфейс для применения детерминированных или стохастических
    возмущений к действиям и наблюдениям.

    Атрибуты:
        name: Идентификатор возмущения (для логирования)
        label: Описание для визуализации
    """

    def __init__(self, name: str, label: str) -> None:
        """
        Инициализирует возмущение.

        Аргументы:
            name: Идентификатор возмущения
            label: Описание для графиков
        """
        self.name = name
        self.label = label

    def perturb_action(self, action: np.ndarray, step: int,
                      total_steps: int) -> np.ndarray:
        """
        Применяет возмущение к действию.

        Аргументы:
            action: Исходное действие [5]
            step: Текущий шаг эпизода
            total_steps: Общее число шагов в эпизоде

        Возвращает:
            Возмущенное действие
        """
        return action

    def perturb_obs(self, obs: np.ndarray, step: int,
                   total_steps: int) -> np.ndarray:
        """
        Применяет возмущение к наблюдению.

        Аргументы:
            obs: Исходное наблюдение [25]
            step: Текущий шаг эпизода
            total_steps: Общее число шагов в эпизоде

        Возвращает:
            Возмущенное наблюдение
        """
        return obs


class NoPerturbation(Perturbation):
    """Отсутствие возмущений (базовый случай)."""

    def __init__(self) -> None:
        super().__init__("none", "No perturbation")


class TorqueNoise(Perturbation):
    """
    Гауссовский шум к действиям, имитирующий неточность актуаторов.

    Имитирует шум в системе управления момента (Drive uncertainty).
    """

    def __init__(self, noise_std: float = 0.15) -> None:
        """
        Инициализирует возмущение шума момента.

        Аргументы:
            noise_std: Стандартное отклонение шума (примерно 15% диапазона)
        """
        super().__init__("torque_noise", f"Actuator Noise (σ={noise_std})")
        self.noise_std = noise_std

    def perturb_action(self, action: np.ndarray, step: int,
                      total_steps: int) -> np.ndarray:
        """
        Добавляет Гауссовский шум к действию.

        Аргументы:
            action: Исходное действие
            step: Текущий шаг (не используется)
            total_steps: Общее число шагов (не используется)

        Возвращает:
            Действие с добавленным шумом, обрезанное в [-1, 1]
        """
        noise = np.random.normal(0, self.noise_std, size=action.shape)
        return np.clip(action + noise, -1.0, 1.0).astype(np.float32)


class PayloadChange(Perturbation):
    """
    Имитирует поднятие нагрузки в процессе траектории.

    Добавляет смещение к действиям в определенный момент времени,
    требуя увеличения управляющих усилий.
    """

    def __init__(self, bias_magnitude: float = 0.25,
                 onset_fraction: float = 0.4) -> None:
        """
        Инициализирует возмущение изменения нагрузки.

        Аргументы:
            bias_magnitude: Величина смещения при появлении нагрузки
            onset_fraction: Доля эпизода, когда появляется нагрузка (0.0-1.0)
        """
        super().__init__(
            "payload",
            f"Payload Change (bias={bias_magnitude}, onset={int(onset_fraction*100)}%)"
        )
        self.bias = bias_magnitude
        self.onset = onset_fraction

    def perturb_action(self, action: np.ndarray, step: int,
                      total_steps: int) -> np.ndarray:
        """
        Добавляет смещение к действию после момента onset.

        Аргументы:
            action: Исходное действие
            step: Текущий шаг эпизода
            total_steps: Общее число шагов

        Возвращает:
            Смещенное действие (после момента появления нагрузки)
        """
        if step >= int(total_steps * self.onset):
            # Смещение к более высоким весам (нужна большая коррекция)
            bias = np.array([
                self.bias, self.bias * 0.5, -self.bias * 0.3,
                self.bias, self.bias * 0.5
            ], dtype=np.float32)
            return np.clip(action + bias, -1.0, 1.0).astype(np.float32)
        return action


class SensorNoise(Perturbation):
    """
    Гауссовский шум к наблюдениям, имитирующий неточность датчиков.

    Имитирует шум энкодеров суставов и других датчиков.
    """

    def __init__(self, noise_std: float = 0.02) -> None:
        """
        Инициализирует возмущение шума датчиков.

        Аргументы:
            noise_std: Стандартное отклонение шума наблюдений
        """
        super().__init__("sensor_noise", f"Sensor Noise (σ={noise_std})")
        self.noise_std = noise_std

    def perturb_obs(self, obs: np.ndarray, step: int,
                   total_steps: int) -> np.ndarray:
        """
        Добавляет Гауссовский шум к наблюдению.

        Аргументы:
            obs: Исходное наблюдение
            step: Текущий шаг (не используется)
            total_steps: Общее число шагов (не используется)

        Возвращает:
            Наблюдение с добавленным шумом
        """
        noise = np.random.normal(0, self.noise_std, size=obs.shape).astype(np.float32)
        return obs + noise


class ImpulsePerturbation(Perturbation):
    """
    Внезапный импульс/столкновение в случайный момент времени.

    Имитирует внешнее воздействие (столкновение, толчок).
    """

    def __init__(self, impulse_magnitude: float = 0.4,
                 duration_steps: int = 5) -> None:
        """
        Инициализирует возмущение импульса.

        Аргументы:
            impulse_magnitude: Амплитуда импульса
            duration_steps: Длительность импульса в шагах
        """
        super().__init__(
            "impulse",
            f"External Impulse (mag={impulse_magnitude})"
        )
        self.magnitude = impulse_magnitude
        self.duration = duration_steps
        self.impulse_start: Optional[int] = None

    def perturb_action(self, action: np.ndarray, step: int,
                      total_steps: int) -> np.ndarray:
        """
        Добавляет импульс к действию в определенное время.

        Аргументы:
            action: Исходное действие
            step: Текущий шаг эпизода
            total_steps: Общее число шагов

        Возвращает:
            Действие с импульсом (если в нужное время)
        """
        # Инициализируем момент начала импульса при первом вызове
        if self.impulse_start is None:
            # Выбираем случайный момент между 20% и 70% эпизода
            self.impulse_start = np.random.randint(
                int(total_steps * 0.2), int(total_steps * 0.7)
            )

        # Если находимся в периоде импульса, добавляем возмущение
        if self.impulse_start <= step < self.impulse_start + self.duration:
            impulse = np.random.uniform(
                -self.magnitude, self.magnitude,
                size=action.shape
            ).astype(np.float32)
            return np.clip(action + impulse, -1.0, 1.0).astype(np.float32)
        return action


class CombinedPerturbation(Perturbation):
    """
    Комбинированное возмущение: шум момента + шум датчиков + изменение нагрузки.

    Имитирует реальные условия с несколькими одновременными возмущениями.
    """

    def __init__(self) -> None:
        """Инициализирует комбинированное возмущение."""
        super().__init__("combined", "Combined Perturbations")
        # Создаем компоненты возмущений
        self.torque = TorqueNoise(noise_std=0.10)
        self.sensor = SensorNoise(noise_std=0.015)
        self.payload = PayloadChange(bias_magnitude=0.15, onset_fraction=0.4)

    def perturb_action(self, action: np.ndarray, step: int,
                      total_steps: int) -> np.ndarray:
        """
        Применяет шум момента и изменение нагрузки.

        Аргументы:
            action: Исходное действие
            step: Текущий шаг эпизода
            total_steps: Общее число шагов

        Возвращает:
            Возмущенное действие (шум + нагрузка)
        """
        action = self.torque.perturb_action(action, step, total_steps)
        action = self.payload.perturb_action(action, step, total_steps)
        return action

    def perturb_obs(self, obs: np.ndarray, step: int,
                   total_steps: int) -> np.ndarray:
        """
        Применяет шум датчиков.

        Аргументы:
            obs: Исходное наблюдение
            step: Текущий шаг эпизода
            total_steps: Общее число шагов

        Возвращает:
            Возмущенное наблюдение (с шумом датчиков)
        """
        return self.sensor.perturb_obs(obs, step, total_steps)


# ============================================================
# Запуск эпизода с возмущениями
# ============================================================

def run_episode_perturbed(backend: RobotArmBackendEnv, q_start: np.ndarray,
                          q_target: np.ndarray, T: float, dt: float,
                          mode: str, N: int,
                          model: Optional[PPO] = None,
                          fixed_action: Optional[np.ndarray] = None,
                          perturbation: Optional[Perturbation] = None) -> Dict[str, Any]:
    """
    Запускает один эпизод управления с применением возмущений.

    Возмущения применяются к наблюдению перед политикой
    и к действию перед отправкой на бэкенд.

    Аргументы:
        backend: RobotArmBackendEnv
        q_start: Начальная конфигурация
        q_target: Целевая конфигурация
        T: Длительность траектории
        dt: Временной шаг
        mode: Режим управления
        N: Горизонт прогнозирования
        model: PPO модель (или None для zero action)
        fixed_action: Фиксированное действие (или None)
        perturbation: Объект возмущения (или None для отсутствия)

    Возвращает:
        Словарь с собранными данными траектории и финальными метриками
    """
    # Инициализируем возмущение (если не задано, используем NoPerturbation)
    if perturbation is None:
        perturbation = NoPerturbation()

    # Инициализируем эпизод на бэкенде
    obs, reset_info = backend.reset(q_start, q_target, T=T, dt=dt, mode=mode, N=N)
    total_steps = int(T / dt) + 5

    # Структура для сбора данных траектории
    trajectory: Dict[str, List[Any]] = {
        "t": [], "eq_rms": [], "edq_rms": [],
        "u_energy": [], "du_energy": [], "reward": [],
    }

    done = False
    total_reward: float = 0.0
    step = 0

    # Главный цикл эпизода
    while not done:
        # Применяем возмущение к наблюдению (перед политикой)
        obs_perturbed = perturbation.perturb_obs(obs.copy(), step, total_steps)

        # Выбираем действие
        if model is not None:
            action, _ = model.predict(obs_perturbed, deterministic=True)
        elif fixed_action is not None:
            action = fixed_action.copy()
        else:
            action = np.zeros(5, dtype=np.float32)

        # Применяем возмущение к действию (перед отправкой на бэкенд)
        action = perturbation.perturb_action(action, step, total_steps)

        # Выполняем шаг на бэкенде
        obs, reward, done, info = backend.step(action)
        total_reward += reward
        step += 1

        # Собираем метрики из info
        trajectory["t"].append(info.get("t", 0))
        trajectory["eq_rms"].append(info.get("eq_rms", 0))
        trajectory["edq_rms"].append(info.get("edq_rms", 0))
        trajectory["u_energy"].append(info.get("u_energy", 0))
        trajectory["du_energy"].append(info.get("du_energy", 0))
        trajectory["reward"].append(reward)

    # Преобразуем списки в numpy массивы
    for key in trajectory:
        trajectory[key] = np.array(trajectory[key])

    # Добавляем финальные метрики
    trajectory["total_reward"] = total_reward
    trajectory["final_eq_rms"] = (
        trajectory["eq_rms"][-1] if len(trajectory["eq_rms"]) > 0 else np.inf
    )
    trajectory["success"] = info.get("success", False)
    trajectory["mean_u_energy"] = (
        np.mean(trajectory["u_energy"]) if len(trajectory["u_energy"]) > 0 else 0
    )
    trajectory["mean_du_energy"] = (
        np.mean(trajectory["du_energy"]) if len(trajectory["du_energy"]) > 0 else 0
    )

    return trajectory


# ============================================================
# Полное сравнение
# ============================================================

def generate_test_configs(n_episodes: int, seed: int = 42) -> List[tuple[np.ndarray, np.ndarray]]:
    """
    Генерирует воспроизводимые пары (начало, цель) для тестирования.

    Аргументы:
        n_episodes: Количество конфигураций для генерации
        seed: Random seed для воспроизводимости

    Возвращает:
        Список кортежей (q_start, q_target)
    """
    rng = np.random.RandomState(seed)

    # Диапазоны суставов
    q_ranges = [
        (-1.5, 1.5), (-1.5, 0.5), (-2.0, 0.5),
        (-1.5, 1.5), (-1.5, 1.5), (-1.5, 1.5),
    ]

    configs: List[tuple[np.ndarray, np.ndarray]] = []

    for _ in range(n_episodes):
        # Генерируем начальную конфигурацию (часто около центра)
        q_start = np.zeros(6, dtype=np.float64)
        if rng.random() > 0.3:
            for i, (lo, hi) in enumerate(q_ranges):
                q_start[i] = rng.uniform(lo * 0.5, hi * 0.5)

        # Генерируем целевую конфигурацию (случайно в диапазоне)
        q_target = np.zeros(6, dtype=np.float64)
        for i, (lo, hi) in enumerate(q_ranges):
            q_target[i] = rng.uniform(lo, hi)

        # Убедимся, что начало и цель достаточно далеко
        while np.linalg.norm(q_target - q_start) < 0.1:
            for i, (lo, hi) in enumerate(q_ranges):
                q_target[i] = rng.uniform(lo, hi)

        configs.append((q_start, q_target))

    return configs


def run_full_comparison(backend: RobotArmBackendEnv,
                        models: Dict[str, Optional[PPO]],
                        configs: List[tuple[np.ndarray, np.ndarray]],
                        perturbations: List[Perturbation],
                        T: float, dt: float, mode: str, N: int) -> Dict[str, Dict[str, List[Dict[str, Any]]]]:
    """
    Запускает все комбинации моделей × возмущений × эпизодов.

    Аргументы:
        backend: RobotArmBackendEnv
        models: Словарь моделей {имя: модель или None}
        configs: Список конфигураций для эпизодов
        perturbations: Список возмущений для тестирования
        T, dt, mode, N: Параметры управления

    Возвращает:
        Вложенный словарь результатов:
        results[возмущение][модель] = [траектория_1, траектория_2, ...]
    """
    results: Dict[str, Dict[str, List[Dict[str, Any]]]] = {}

    for pert in perturbations:
        results[pert.name] = {}

        for model_name, model in models.items():
            results[pert.name][model_name] = []
            print(f"\n  [{pert.label}] {model_name}:")

            for ep, (q_start, q_target) in enumerate(configs):
                # Сбрасываем состояние возмущения для нового эпизода
                if hasattr(pert, 'impulse_start'):
                    pert.impulse_start = None

                # Выбираем способ управления
                if model_name == "Fixed LQR":
                    traj = run_episode_perturbed(
                        backend, q_start, q_target, T, dt, mode, N,
                        fixed_action=np.zeros(5, dtype=np.float32),
                        perturbation=pert
                    )
                else:
                    traj = run_episode_perturbed(
                        backend, q_start, q_target, T, dt, mode, N,
                        model=model, perturbation=pert
                    )

                results[pert.name][model_name].append(traj)

                # Выводим прогресс каждые 10 эпизодов
                if (ep + 1) % 10 == 0:
                    avg_eq = np.mean([r["final_eq_rms"] for r in results[pert.name][model_name]])
                    sr = np.mean([r["success"] for r in results[pert.name][model_name]]) * 100
                    print(f"    Ep {ep+1}: avg_eq={avg_eq:.4f}, success={sr:.0f}%")

    return results


# ============================================================
# Визуализация
# ============================================================

def setup_style() -> None:
    """
    Настраивает matplotlib стиль для публикационных графиков.
    """
    plt.rcParams.update({
        'font.family': 'serif', 'font.size': 11,
        'axes.labelsize': 12, 'axes.titlesize': 13,
        'xtick.labelsize': 10, 'ytick.labelsize': 10,
        'legend.fontsize': 9, 'figure.dpi': 150,
        'savefig.dpi': 300, 'savefig.bbox': 'tight',
        'axes.grid': True, 'grid.alpha': 0.3,
    })


def plot_robustness_summary(results: Dict[str, Dict[str, List[Dict[str, Any]]]],
                            perturbation_names: List[str],
                            model_names: List[str],
                            output_dir: str) -> None:
    """
    Основная фигура: bar chart ошибки и успешности для каждого возмущения × модель.

    Показывает три метрики рядом: финальная ошибка, успешность, награда.

    Аргументы:
        results: Результаты всех экспериментов
        perturbation_names: Названия возмущений
        model_names: Названия моделей
        output_dir: Директория для сохранения
    """
    n_perts = len(perturbation_names)
    n_models = len(model_names)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))

    # Цвета для каждой модели
    colors = {
        "Fixed LQR": '#d62728',
        "PPO v1": '#ff7f0e',
        "PPO v2": '#1f77b4',
    }

    x = np.arange(n_perts)
    width = 0.8 / n_models

    # График 1: Финальная ошибка
    for i, mn in enumerate(model_names):
        vals: List[float] = []
        stds: List[float] = []
        for pn in perturbation_names:
            eqs = [r["final_eq_rms"] for r in results[pn][mn]]
            vals.append(np.mean(eqs))
            stds.append(np.std(eqs))
        axes[0].bar(x + i * width - (n_models - 1) * width / 2,
                     vals, width, yerr=stds,
                     color=colors.get(mn, f'C{i}'), alpha=0.85,
                     label=mn, capsize=3)
    axes[0].set_ylabel("Final Error (RMS rad)")
    axes[0].set_title("Precision under Perturbations")
    axes[0].set_xticks(x)
    axes[0].axhline(y=0.03, color='green', linestyle='--', linewidth=1, alpha=0.5)
    axes[0].legend()

    # График 2: Успешность
    for i, mn in enumerate(model_names):
        vals = []
        for pn in perturbation_names:
            sr = np.mean([r["success"] for r in results[pn][mn]]) * 100
            vals.append(sr)
        axes[1].bar(x + i * width - (n_models - 1) * width / 2,
                     vals, width,
                     color=colors.get(mn, f'C{i}'), alpha=0.85,
                     label=mn)
    axes[1].set_ylabel("Success Rate (%)")
    axes[1].set_title("Success under Perturbations")
    axes[1].set_xticks(x)
    axes[1].set_ylim(0, 105)
    axes[1].legend()

    # График 3: Награда
    for i, mn in enumerate(model_names):
        vals = []
        for pn in perturbation_names:
            rews = [r["total_reward"] for r in results[pn][mn]]
            vals.append(np.mean(rews))
        axes[2].bar(x + i * width - (n_models - 1) * width / 2,
                     vals, width,
                     color=colors.get(mn, f'C{i}'), alpha=0.85,
                     label=mn)
    axes[2].set_ylabel("Total Reward")
    axes[2].set_title("Reward under Perturbations")
    axes[2].set_xticks(x)
    axes[2].legend()

    # Подписи x-оси
    pert_labels_short = {
        "none": "Sin\npert.",
        "torque_noise": "Actuator\nNoise",
        "sensor_noise": "Sensor\nNoise",
        "payload": "Payload\nChange",
        "impulse": "External\nImpulse",
        "combined": "Combined",
    }
    for ax in axes:
        ax.set_xticklabels([pert_labels_short.get(pn, pn) for pn in perturbation_names],
                           fontsize=9)

    fig.suptitle("Robustness: Fixed LQR vs PPO Adaptive under Perturbations",
                 fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "05_robustness_summary.png"))
    fig.savefig(os.path.join(output_dir, "05_robustness_summary.pdf"))
    plt.close(fig)
    print("  -> 05_robustness_summary.png/pdf")


def plot_degradation_curves(results: Dict[str, Dict[str, List[Dict[str, Any]]]],
                            perturbation_names: List[str],
                            model_names: List[str],
                            output_dir: str) -> None:
    """
    График: Как ошибка деградирует при усилении возмущений.

    Показывает тренд деградации производительности для каждой модели.

    Аргументы:
        results: Результаты всех экспериментов
        perturbation_names: Названия возмущений
        model_names: Названия моделей
        output_dir: Директория для сохранения
    """
    fig, ax = plt.subplots(figsize=(10, 5.5))

    colors = {
        "Fixed LQR": '#d62728',
        "PPO v1": '#ff7f0e',
        "PPO v2": '#1f77b4',
    }
    markers = {"Fixed LQR": 's', "PPO v1": '^', "PPO v2": 'o'}

    for mn in model_names:
        means: List[float] = []
        stds: List[float] = []
        for pn in perturbation_names:
            eqs = [r["final_eq_rms"] for r in results[pn][mn]]
            means.append(np.mean(eqs))
            stds.append(np.std(eqs))

        ax.errorbar(range(len(perturbation_names)), means, yerr=stds,
                     color=colors.get(mn, 'gray'),
                     marker=markers.get(mn, 'o'),
                     linewidth=2, markersize=8, capsize=5,
                     label=mn)

    ax.axhline(y=0.03, color='green', linestyle='--', linewidth=1,
               alpha=0.5, label='Success Threshold')

    pert_labels = {
        "none": "No Pert.",
        "torque_noise": "Actuator\nNoise",
        "sensor_noise": "Sensor\nNoise",
        "payload": "Payload\nChange",
        "impulse": "Impulse",
        "combined": "Combined",
    }
    ax.set_xticks(range(len(perturbation_names)))
    ax.set_xticklabels([pert_labels.get(pn, pn) for pn in perturbation_names])
    ax.set_ylabel("Mean Final Error (RMS rad)")
    ax.set_title("Degradation Curve: Error vs Perturbation Type")
    ax.legend(loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "06_degradation_curves.png"))
    fig.savefig(os.path.join(output_dir, "06_degradation_curves.pdf"))
    plt.close(fig)
    print("  -> 06_degradation_curves.png/pdf")


def plot_time_series_perturbed(results: Dict[str, Dict[str, List[Dict[str, Any]]]],
                               perturbation_name: str,
                               model_names: List[str],
                               output_dir: str) -> None:
    """
    Временной ряд отслеживания ошибки при худшем возмущении (combined).

    Показывает как ошибка развивается во времени для каждой модели.

    Аргументы:
        results: Результаты всех экспериментов
        perturbation_name: Название возмущения для анализа
        model_names: Названия моделей
        output_dir: Директория для сохранения
    """
    fig, ax = plt.subplots(figsize=(10, 5))

    colors = {
        "Fixed LQR": '#d62728',
        "PPO v1": '#ff7f0e',
        "PPO v2": '#1f77b4',
    }

    for mn in model_names:
        trajs = results[perturbation_name][mn]
        # Находим максимальную длину траектории
        max_len = max(len(t["eq_rms"]) for t in trajs)

        # Заполняем матрицу NaN для выравнивания
        matrix = np.full((len(trajs), max_len), np.nan)
        for i, t in enumerate(trajs):
            matrix[i, :len(t["eq_rms"])] = t["eq_rms"]

        # Вычисляем среднее и std
        t_axis = np.arange(max_len) * 0.02
        mean = np.nanmean(matrix, axis=0)
        std = np.nanstd(matrix, axis=0)

        # Рисуем линию с затенением
        ax.plot(t_axis, mean, color=colors.get(mn, 'gray'),
                linewidth=1.8, label=mn)
        ax.fill_between(t_axis, mean - std, mean + std,
                        color=colors.get(mn, 'gray'), alpha=0.12)

    # Добавляем линию успеха
    ax.axhline(y=0.03, color='green', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Tracking Error (RMS rad)")
    ax.set_title(f"Time Series Error under Combined Perturbations")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "07_time_series_combined_perturbation.png"))
    fig.savefig(os.path.join(output_dir, "07_time_series_combined_perturbation.pdf"))
    plt.close(fig)
    print("  -> 07_time_series_combined_perturbation.png/pdf")


def print_full_table(results: Dict[str, Dict[str, List[Dict[str, Any]]]],
                     perturbation_names: List[str],
                     model_names: List[str],
                     pert_labels: Dict[str, str]) -> None:
    """
    Выводит полную таблицу результатов со статистикой.

    Аргументы:
        results: Результаты всех экспериментов
        perturbation_names: Названия возмущений
        model_names: Названия моделей
        pert_labels: Словарь описаний возмущений
    """
    print("\n" + "=" * 90)
    print("  TABLA DE RESULTADOS COMPLETA")
    print("=" * 90)

    # Заголовок таблицы
    header = f"{'Perturbación':<22}"
    for mn in model_names:
        header += f" | {mn:>12} (eq)  {mn:>8} (SR)"
    print(header)
    print("-" * 90)

    # Строки таблицы для каждого возмущения
    for pn in perturbation_names:
        row = f"  {pert_labels.get(pn, pn):<20}"
        for mn in model_names:
            eqs = [r["final_eq_rms"] for r in results[pn][mn]]
            sr = np.mean([r["success"] for r in results[pn][mn]]) * 100
            row += f" | {np.mean(eqs):>10.4f}±{np.std(eqs):.4f}  {sr:>7.1f}%"
        print(row)

    print("=" * 90)


def save_full_csv(results: Dict[str, Dict[str, List[Dict[str, Any]]]],
                  perturbation_names: List[str],
                  model_names: List[str],
                  output_dir: str) -> None:
    """
    Сохраняет все результаты в CSV для анализа.

    Аргументы:
        results: Результаты всех экспериментов
        perturbation_names: Названия возмущений
        model_names: Названия моделей
        output_dir: Директория для сохранения
    """
    with open(os.path.join(output_dir, "perturbation_results.csv"), "w") as f:
        # Заголовок CSV
        f.write("perturbation,model,episode,final_eq_rms,total_reward,"
                "mean_u_energy,mean_du_energy,success\n")

        # Записываем данные для каждой комбинации
        for pn in perturbation_names:
            for mn in model_names:
                for i, r in enumerate(results[pn][mn]):
                    f.write(f"{pn},{mn},{i+1},{r['final_eq_rms']:.6f},"
                            f"{r['total_reward']:.4f},{r['mean_u_energy']:.6f},"
                            f"{r['mean_du_energy']:.6f},{r['success']}\n")

    print(f"  -> perturbation_results.csv")


# ============================================================
# Главная функция
# ============================================================

def main() -> None:
    """
    Главная функция: парсит аргументы, запускает эксперименты и генерирует графики.

    Процесс:
    1. Загружает PPO модели (v1 и v2)
    2. Генерирует конфигурации для тестирования
    3. Запускает все комбинации моделей × возмущений × эпизодов
    4. Выводит статистику в консоль
    5. Генерирует публикационные графики
    6. Сохраняет результаты в CSV
    """
    parser = argparse.ArgumentParser(
        description="Compare LQR vs PPO under perturbations"
    )
    parser.add_argument("--model-v1", type=str, default="checkpoints/ur5e_ppo_final.zip")
    parser.add_argument("--model-v2", type=str, default="checkpoints_v2/ur5e_ppo_v2_final.zip")
    parser.add_argument("--url", type=str, default="http://127.0.0.1:8848")
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--T", type=float, default=1.5)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--mode", type=str, default="mpc_lite")
    parser.add_argument("--N", type=int, default=20)
    parser.add_argument("--output-dir", type=str, default="thesis_plots_perturbations")
    parser.add_argument("--seed", type=int, default=42)

    args = parser.parse_args()

    # Настраиваем стиль
    setup_style()
    os.makedirs(args.output_dir, exist_ok=True)

    print("=" * 60)
    print("  Comparison with Perturbations")
    print("=" * 60)
    print(f"  Episodios:  {args.episodes}")
    print(f"  Modelo v1:  {args.model_v1}")
    print(f"  Modelo v2:  {args.model_v2}")
    print("=" * 60)

    # Загружаем модели
    models: Dict[str, Optional[PPO]] = {"Fixed LQR": None}

    if os.path.exists(args.model_v1):
        print(f"Cargando PPO v1: {args.model_v1}")
        models["PPO v1"] = PPO.load(args.model_v1)
    else:
        print(f"[!] No se encontró v1: {args.model_v1}")

    if os.path.exists(args.model_v2):
        print(f"Cargando PPO v2: {args.model_v2}")
        models["PPO v2"] = PPO.load(args.model_v2)
    else:
        print(f"[!] No se encontró v2: {args.model_v2}")

    # Создаем бэкенд и генерируем конфигурации
    backend = RobotArmBackendEnv(base_url=args.url)
    configs = generate_test_configs(args.episodes, args.seed)

    # Определяем возмущения для тестирования
    perturbations: List[Perturbation] = [
        NoPerturbation(),
        TorqueNoise(noise_std=0.15),
        SensorNoise(noise_std=0.02),
        PayloadChange(bias_magnitude=0.25, onset_fraction=0.4),
        ImpulsePerturbation(impulse_magnitude=0.4, duration_steps=5),
        CombinedPerturbation(),
    ]

    pert_names = [p.name for p in perturbations]
    model_names = list(models.keys())

    # Запускаем полное сравнение
    print(f"\nRunning {args.episodes} episodes × {len(perturbations)} perturbations × {len(models)} models...")
    results = run_full_comparison(
        backend, models, configs, perturbations,
        args.T, args.dt, args.mode, args.N
    )

    # Выводим таблицу результатов
    pert_labels = {p.name: p.label for p in perturbations}
    print_full_table(results, pert_names, model_names, pert_labels)

    # Генерируем графики
    print(f"\nGenerando gráficas en {args.output_dir}/...")
    plot_robustness_summary(results, pert_names, model_names, args.output_dir)
    plot_degradation_curves(results, pert_names, model_names, args.output_dir)
    plot_time_series_perturbed(results, "combined", model_names, args.output_dir)
    save_full_csv(results, pert_names, model_names, args.output_dir)

    print(f"\n¡Listo! Resultados en: {args.output_dir}/")


if __name__ == "__main__":
    main()
