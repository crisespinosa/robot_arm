#pragma once

/**
 * @file ArmController.h
 * @brief REST-контроллер для управления роботом-манипулятором UR5e.
 *
 * Предоставляет HTTP-эндпоинты для:
 *   - Планирования опорной траектории minimum-jerk.
 *   - Установки опорной траектории.
 *   - Выполнения одного шага управления LQR + inverse dynamics.
 *
 * Оценка состояния: фильтр Калмана 2-го порядка (KF2) по измерениям q
 * для восстановления скорости dq. Основа управления: конечногоризонтный
 * LQR с обратной рекурсией Риккати на локальной модели ошибки
 * (двойной интегратор по каждому суставу).
 */

#include <drogon/HttpController.h>
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

    // ============================================================
    // Метрики эпизода (накапливаются по шагам, печатаются один раз
    // в конце траектории — когда t достигает T).
    // ============================================================
    struct EpisodeStats {
        double sum_eq_sq{0.0};       ///< Σ ‖eq‖² по шагам и суставам
        double sum_edq_sq{0.0};      ///< Σ ‖edq‖²
        double sum_u_sq_dt{0.0};     ///< ∫ ‖τ‖² dt
        double max_abs_eq{0.0};      ///< пиковая |eq_i| по всему эпизоду
        int    n_samples{0};         ///< количество накопленных шагов
        double last_eq_rms{0.0};     ///< RMS ошибки положения на последнем шаге
        double last_t{0.0};          ///< последнее t

        // Снимок последних использованных параметров шага
        std::string mode{"lqr"};
        double wq{0.0}, wdq{0.0}, wu{0.0}, wqN{0.0}, wdqN{0.0};
        int    horizonN{20};
        double u_max{8.0};
        bool   summary_printed{false};
    };
    EpisodeStats stats_;
    int    step_count_{0};            ///< счётчик шагов внутри текущего эпизода
    int    log_every_n_{10};          ///< периодичность вывода [step] лога
    double success_tol_{0.03};        ///< RMS-порог "успеха" в финале (рад)
};
