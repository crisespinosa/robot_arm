#pragma once

/**
 * @file ArmController.h
 * @brief REST-контроллер для управления роботом-манипулятором UR5e.
 *
 * Предоставляет HTTP-эндпоинты для:
 *   - Планирования опорной траектории minimum-jerk (квинтический полином).
 *     Endpoint: POST /arm/plan_minjerk_q. Реализация — замкнутая формула
 *     квинтика, а НЕ численный решатель принципа максимума Понтрягина.
 *   - Установки опорной траектории для контура управления.
 *   - Шагов управления (LQR / MPC-lite + обратная динамика) — ЯДРО ЭТАПА 1.
 *   - RL-эндпоинтов (reset/step) — задел для будущих этапов, не ядро.
 *
 * Оценка состояния: фильтр Калмана 2-го порядка (KF2) по измерениям q
 * для восстановления скорости dq. Основа управления: конечногоризонтный
 * LQR с обратной рекурсией Риккати на локальной модели ошибки
 * (двойной интегратор по каждому суставу).
 */

#include <drogon/HttpController.h>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "dynamics.hpp"    // ArmDynamics — модель динамики UR5e
#include "trajectory.hpp"  // RefPoint, plan_minjerk_traj — планирование траекторий

class ArmController : public drogon::HttpController<ArmController> {
public:
    ArmController();

    // ---- Регистрация HTTP-эндпоинтов ----
    METHOD_LIST_BEGIN
        // Планирование опорной траектории minimum-jerk (квинтический полином) в суставных координатах.
        ADD_METHOD_TO(ArmController::handlePlanMinJerkQ,  "/arm/plan_minjerk_q", drogon::Post);
        // Установка опорной траектории для контроллера
        ADD_METHOD_TO(ArmController::handleSetReference,  "/arm/set_reference", drogon::Post);
        // Один шаг управления (LQR + обратная динамика + интегрирование)
        ADD_METHOD_TO(ArmController::handleStep,          "/arm/step",          drogon::Post);
        // RL: сброс эпизода (инициализация окружения для PPO)
        ADD_METHOD_TO(ArmController::handleRLReset,       "/rl/reset",          drogon::Post);
        // RL: один шаг (PPO-агент передаёт действие, получает наблюдение и награду)
        ADD_METHOD_TO(ArmController::handleRLStep,        "/rl/step",           drogon::Post);
    METHOD_LIST_END

    /// Планирование опорной траектории minimum-jerk (квинтический полином).
    void handlePlanMinJerkQ(const drogon::HttpRequestPtr&,
                            std::function<void (const drogon::HttpResponsePtr&)>&&);

    /// Установка опорной траектории для контура управления.
    void handleSetReference(const drogon::HttpRequestPtr&,
                            std::function<void (const drogon::HttpResponsePtr&)>&&);

    /// Шаг управления: вычислить момент и обновить состояние.
    void handleStep(const drogon::HttpRequestPtr&,
                    std::function<void (const drogon::HttpResponsePtr&)>&&);

    /// Сброс эпизода RL: задать q_start, q_target, параметры рандомизации.
    void handleRLReset(const drogon::HttpRequestPtr&,
                       std::function<void (const drogon::HttpResponsePtr&)>&&);

    /// Шаг RL: принять действие PPO, вычислить управление, вернуть obs/reward/done.
    void handleRLStep(const drogon::HttpRequestPtr&,
                      std::function<void (const drogon::HttpResponsePtr&)>&&);

private:
    ArmDynamics dyn_;  ///< Модель динамики UR5e (инерция, трение, гравитация)

    std::mutex mtx_;                    ///< Мьютекс для потокобезопасного доступа к траектории
    std::vector<RefPoint> ref_traj_;    ///< Опорная траектория minimum-jerk (квинтик).
    double ref_dt_{0.02};               ///< Шаг дискретизации опорной траектории (с)
    double ref_T_{0.0};                 ///< Длительность опорной траектории (с)

    std::vector<double> last_q_meas_{std::vector<double>(6, 0.0)};  ///< Последнее измерение q
    bool have_last_q_{false};           ///< Флаг: получено ли первое измерение

    /**
     * @brief Фильтр Калмана 2-го порядка (KF2) для одного сустава.
     *
     * Модель состояния: x = [q, dq]^T
     * Измерение: z = q (только положение)
     * Оценивает скорость dq из последовательных измерений q.
     */
    struct KF2 {
        double x0{0.0};   ///< Оценка положения (рад)
        double x1{0.0};   ///< Оценка скорости (рад/с)
        double P00{1.0}, P01{0.0}, P10{0.0}, P11{1.0};  ///< Ковариационная матрица P
        bool initialized{false};  ///< Флаг инициализации
    };
    std::vector<KF2> kf_{std::vector<KF2>(6)};  ///< Фильтры Калмана для 6 суставов

    // ---- Параметры эпизода RL ----
    bool rl_active_{false};              ///< Активен ли эпизод RL
    double rl_t_{0.0};                   ///< Текущее время в эпизоде (с)
    double rl_dt_{0.02};                 ///< Шаг по времени (с)
    double rl_T_{0.0};                   ///< Длительность эпизода (с)
    int rl_step_count_{0};               ///< Текущий номер шага
    int rl_max_steps_{300};              ///< Максимальное число шагов в эпизоде
    double rl_success_tol_{0.03};        ///< Порог успеха (RMS ошибки < tol, рад)
    std::string rl_mode_{"mpc_lite"};    ///< Режим управления: "mpc_lite" или "pd"
    int rl_horizonN_{20};                ///< Горизонт LQR (число шагов Риккати)
    double rl_u_max_{8.0};               ///< Макс. допустимое ускорение (рад/с²)
    std::vector<double> rl_last_tau_cmd_{std::vector<double>(6, 0.0)};  ///< Предыдущая команда момента
    std::vector<double> rl_q_start_{std::vector<double>(6, 0.0)};       ///< Начальная конфигурация
    std::vector<double> rl_q_target_{std::vector<double>(6, 0.0)};      ///< Целевая конфигурация
    double rl_inertia_scale_{1.0};       ///< Масштаб инерции (доменная рандомизация)
    double rl_friction_scale_{1.0};      ///< Масштаб трения (доменная рандомизация)

    // ---- Веса для функции награды RL ----
    double rw_pos_{2.0};      ///< Вес ошибки положения (β_q)
    double rw_vel_{0.15};     ///< Вес ошибки скорости (β_dq)
    double rw_u_{0.002};      ///< Вес энергии управления (β_u)
    double rw_du_{0.0005};    ///< Вес изменения управления (β_Δu)
    double rw_success_{5.0};  ///< Бонус за успешное достижение цели

    // ============================================================
    // Метрики эпизода для тезиса
    // ============================================================
    //
    // Все ошибки отслеживания считаются между РЕАЛЬНЫМ состоянием
    // манипулятора (dyn_.state()) и опорной траекторией PMP.
    // J_total вычисляется в векторной форме по суставам:
    //     J_step = (Σ_i wq·eq_i² + Σ_i wdq·edq_i² + Σ_i wu·tau_i²) · dt
    // ------------------------------------------------------------
    struct EpisodeMetrics {
        // --- Tracker (зависят от реального состояния q_real, dq_real) ---
        double sum_eq_sq    {0.0};   ///< Σ ‖eq‖²  (сумма квадратов по всем суставам и шагам)
        double sum_edq_sq   {0.0};   ///< Σ ‖edq‖²
        double sum_u_sq_dt  {0.0};   ///< ∫‖τ‖²dt (энергия управления)
        double sum_J        {0.0};   ///< ∫(eqᵀQ eq + edqᵀQd edq + τᵀR τ) dt (LQR-стоимость)
        double max_abs_eq   {0.0};   ///< пиковая |eq_i| по всем суставам и шагам
        int    n_samples    {0};     ///< число шагов
        double eq_final     {0.0};   ///< RMS ошибки положения на последнем шаге
        double reward_total {0.0};   ///< сумма reward PPO
        bool   success      {false}; ///< eq_final ≤ success_tol и time_done

        // --- PMP план (не зависят от трекера) ---
        double ref_T           {0.0};  ///< длительность плана (с)
        double ref_max_dq      {0.0};  ///< пик ‖dq_ref‖∞ на всём плане
        double ref_max_ddq     {0.0};  ///< пик ‖ddq_ref‖∞
        double ref_jerk_energy {0.0};  ///< ∫‖jerk_ref‖² dt (критерий minimum-jerk)
        double ref_path_length {0.0};  ///< ∫‖dq_ref‖₁ dt (длина пути в пространстве суставов)
    };
    EpisodeMetrics ep_metrics_{};     ///< метрики текущего эпизода
    bool           csv_log_enabled_{true};  ///< писать ли пошаговый лог CSV
    std::ofstream  csv_log_;          ///< файл CSV текущего эпизода
};
