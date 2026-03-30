#pragma once

/**
 * @file ArmController.h
 * @brief REST-контроллер для управления роботом-манипулятором UR5e.
 *
 * Предоставляет HTTP-эндпоинты для:
 *   - Планирования траекторий (PMP minimum-jerk)
 *   - Установки опорной траектории
 *   - Шагов управления (LQR/MPC-lite + обратная динамика)
 *   - Обучения с подкреплением (RL: reset/step для PPO-агента)
 *
 * Использует фильтр Калмана 2-го порядка (KF2) для оценки состояния
 * и конечногоризонтный LQR с рекурсией Риккати для вычисления усилений.
 */

#include <drogon/HttpController.h>
#include <functional>
#include <mutex>
#include <vector>
#include <string>

#include "dynamics.hpp"    // ArmDynamics — модель динамики UR5e
#include "trajectory.hpp"  // PMPPoint, plan_pmp_minimum_jerk — планирование траекторий

class ArmController : public drogon::HttpController<ArmController> {
public:
    ArmController();

    // ---- Регистрация HTTP-эндпоинтов ----
    METHOD_LIST_BEGIN
        // Планирование траектории minimum-jerk в суставных координатах
        ADD_METHOD_TO(ArmController::handlePlanPMP_Q,     "/arm/plan_pmp_q",    drogon::Post);
        // Установка опорной траектории для контроллера
        ADD_METHOD_TO(ArmController::handleSetReference,  "/arm/set_reference", drogon::Post);
        // Один шаг управления (LQR + обратная динамика + интегрирование)
        ADD_METHOD_TO(ArmController::handleStep,          "/arm/step",          drogon::Post);
        // RL: сброс эпизода (инициализация окружения для PPO)
        ADD_METHOD_TO(ArmController::handleRLReset,       "/rl/reset",          drogon::Post);
        // RL: один шаг (PPO-агент передаёт действие, получает наблюдение и награду)
        ADD_METHOD_TO(ArmController::handleRLStep,        "/rl/step",           drogon::Post);
    METHOD_LIST_END

    /// Планирование PMP minimum-jerk траектории.
    void handlePlanPMP_Q(const drogon::HttpRequestPtr&,
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
    std::vector<PMPPoint> ref_traj_;    ///< Опорная траектория (PMP minimum-jerk)
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
};
