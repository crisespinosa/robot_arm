#pragma once

/**
 * @file dynamics.hpp
 * @brief Модель динамики 6-осевого манипулятора UR5e в суставных координатах.
 *
 * Реализует уравнение движения:
 *   M(q) * ddq + n(q, dq) = tau
 * где:
 *   M(q)     — диагональная матрица инерции (зависит от конфигурации),
 *   n(q, dq) = C(q,dq)*dq + g(q) + F(dq)  — нелинейные члены
 *              (Кориолис, гравитация, трение),
 *   tau      — вектор крутящих моментов в суставах.
 *
 * Параметры откалиброваны для Universal Robots UR5e:
 *   - Массы звеньев из URDF: 3.761, 8.393, 2.275, 1.219, 1.219, 0.1879 кг
 *   - DH-параметры: a2=0.425м, a3=0.3922м, d4=0.1333м, d5=0.0997м, d6=0.0996м
 *   - Суставы 1-3: макс. момент 150 Нм, макс. скорость pi рад/с
 *   - Суставы 4-6: макс. момент 28 Нм, макс. скорость 2*pi рад/с
 *   - Гармонические редукторы 100:1 (HFUS-2SH)
 *
 * Интегрирование: полунеявный метод Эйлера (semi-implicit Euler).
 * Поддержка доменной рандомизации: масштабирование инерции и трения (0.2–3.0).
 */

#include <vector>
#include <algorithm>
#include <cassert>
#include <cmath>

// ---------------------------------------------------------------
// ArmState — состояние 6-осевого манипулятора в суставных координатах.
//   q   — положения суставов (рад)
//   dq  — скорости суставов  (рад/с)
//   ddq — ускорения суставов (рад/с²), последнее вычисленное значение
// ---------------------------------------------------------------
struct ArmState {
    std::vector<double> q;    ///< Угловые положения суставов (рад)
    std::vector<double> dq;   ///< Угловые скорости суставов (рад/с)
    std::vector<double> ddq;  ///< Угловые ускорения суставов (рад/с²)
};

// ---------------------------------------------------------------
// ArmDynamics — класс динамики манипулятора UR5e.
//
// Приближённая модель в суставном пространстве:
//   M(q) ddq + n(q, dq) = tau
// M(q) — диагональная, зависящая от конфигурации матрица инерции,
// n(q, dq) — вектор нелинейных сил: Кориолис + гравитация + трение.
// ---------------------------------------------------------------
class ArmDynamics {
public:
    /**
     * @brief Конструктор. Инициализирует модель с заданным числом степеней свободы.
     * @param dof Число степеней свободы (по умолчанию 6 для UR5e).
     */
    explicit ArmDynamics(size_t dof = 6) : dof_(dof) {
        state_.q.assign(dof_, 0.0);
        state_.dq.assign(dof_, 0.0);
        state_.ddq.assign(dof_, 0.0);
        tau_.assign(dof_, 0.0);

        const double PI = 3.14159265358979323846;

        // Пределы положений суставов UR5e (рад) — из URDF
        // Все суставы вращательные: от -2*pi до +2*pi
        qmin_ = {-2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI};
        qmax_ = { 2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI};

        // Пределы скоростей суставов UR5e (рад/с)
        // Суставы 1-3 («размер 3»): pi рад/с ≈ 3.14
        // Суставы 4-6 («размер 1»): 2*pi рад/с ≈ 6.28
        dqmax_ = {PI, PI, PI, 2.0*PI, 2.0*PI, 2.0*PI};

        // Пределы крутящих моментов суставов UR5e (Нм)
        // Суставы 1-3: 150 Нм,  Суставы 4-6: 28 Нм
        taumax_ = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};

        // Эффективные инерции в суставном пространстве (кг·м²)
        // Вычислены из масс звеньев URDF и геометрии:
        //   m1=3.761, m2=8.393, m3=2.275, m4=1.219, m5=1.219, m6=0.1879 кг
        //   DH: a2=0.425м, a3=0.3922м
        // Включена отражённая инерция ротора через редукторы 100:1
        // Jbase — базовая инерция (домашняя конфигурация), Jvar — конфигурационная вариация
        Jbase_ = {5.5, 10.8, 3.6, 0.45, 0.20, 0.08};
        Jvar_  = {0.6,  2.5, 1.2, 0.08, 0.03, 0.01};

        // Вязкое трение (Нм·с/рад) — идентифицировано из данных движения UR5e
        Bvisc_ = {3.5, 4.8, 3.2, 0.70, 0.40, 0.18};

        // Кулоновское трение (Нм) — идентифицировано из UR5e
        Fcoul_ = {0.55, 0.80, 0.60, 0.14, 0.10, 0.05};

        // Если DOF ≠ 6 — задаём параметры по умолчанию
        if (dof_ != 6) {
            qmin_.assign(dof_, -PI);
            qmax_.assign(dof_, PI);
            dqmax_.assign(dof_, 3.0);
            taumax_.assign(dof_, 20.0);
            Jbase_.assign(dof_, 1.0);
            Jvar_.assign(dof_, 0.1);
            Bvisc_.assign(dof_, 0.2);
            Fcoul_.assign(dof_, 0.02);
        }
    }

    /// Возвращает текущее состояние манипулятора (q, dq, ddq).
    const ArmState& state() const { return state_; }

    /**
     * @brief Установить масштабные коэффициенты для доменной рандомизации.
     * @param inertia_scale  Масштаб инерции (допустимый диапазон: 0.2–3.0).
     * @param friction_scale Масштаб трения  (допустимый диапазон: 0.2–3.0).
     */
    void setParamScales(double inertia_scale, double friction_scale) {
        inertia_scale_ = std::clamp(inertia_scale, 0.2, 3.0);
        friction_scale_ = std::clamp(friction_scale, 0.2, 3.0);
    }

    /// Сброс масштабов инерции и трения к номинальным значениям (1.0).
    void resetParamScales() {
        inertia_scale_ = 1.0;
        friction_scale_ = 1.0;
    }

    /// Возвращает текущий масштаб инерции.
    double inertiaScale() const { return inertia_scale_; }

    /// Возвращает текущий масштаб трения.
    double frictionScale() const { return friction_scale_; }

    /**
     * @brief Задать состояние манипулятора (положения и скорости суставов).
     * @param q  Вектор угловых положений (рад), размерность = dof.
     * @param dq Вектор угловых скоростей (рад/с), размерность = dof.
     */
    void setState(const std::vector<double>& q,
                  const std::vector<double>& dq) {
        assert(q.size() == dof_ && dq.size() == dof_);
        state_.q = q;
        state_.dq = dq;
        state_.ddq.assign(dof_, 0.0);
        clampState();
    }

    /**
     * @brief Обратная динамика: вычисляет вектор моментов tau для желаемых ускорений.
     *
     * Формула:  tau = M(q) * ddq_des + n(q, dq)
     * Результат ограничивается допустимыми пределами моментов.
     *
     * @param q       Текущие положения суставов (рад).
     * @param dq      Текущие скорости суставов (рад/с).
     * @param ddq_des Желаемые ускорения суставов (рад/с²).
     * @return Вектор крутящих моментов (Нм), ограниченный пределами.
     */
    std::vector<double> inverseDynamics(const std::vector<double>& q,
                                        const std::vector<double>& dq,
                                        const std::vector<double>& ddq_des) const {
        assert(q.size() == dof_ && dq.size() == dof_ && ddq_des.size() == dof_);
        std::vector<double> M = computeMassDiag(q);
        std::vector<double> n = computeNonlinear(q, dq);
        std::vector<double> tau(dof_, 0.0);
        for (size_t i = 0; i < dof_; ++i) {
            double ti = M[i] * ddq_des[i] + n[i];
            tau[i] = std::clamp(ti, -taumax_[i], taumax_[i]);
        }
        return tau;
    }

    /**
     * @brief Шаг интегрирования: применить крутящий момент и обновить состояние.
     *
     * Используется полунеявный метод Эйлера (semi-implicit Euler):
     *   1. Вычисляем ddq = (tau - n(q, dq)) / M(q)
     *   2. Обновляем dq += dt * ddq
     *   3. Обновляем q  += dt * dq  (используем уже обновлённую скорость)
     *   4. Применяем ограничения на положения и скорости.
     *
     * @param tau Вектор крутящих моментов (Нм).
     * @param dt  Шаг по времени (с).
     */
    void stepWithTorque(const std::vector<double>& tau, double dt) {
        assert(tau.size() == dof_);
        if (dt <= 1e-9) dt = 1e-3;

        // Ограничиваем входные моменты допустимыми пределами
        for (size_t i = 0; i < dof_; ++i) {
            tau_[i] = std::clamp(tau[i], -taumax_[i], taumax_[i]);
        }

        // Вычисляем диагональную матрицу масс и нелинейные силы
        const std::vector<double> M = computeMassDiag(state_.q);
        const std::vector<double> n = computeNonlinear(state_.q, state_.dq);

        for (size_t i = 0; i < dof_; ++i) {
            const double Mi = std::max(1e-6, M[i]);
            double ddq = (tau_[i] - n[i]) / Mi;
            // Мягкое численное ограничение на ускорения
            ddq = std::clamp(ddq, -25.0, 25.0);
            state_.ddq[i] = ddq;

            // Полунеявный метод Эйлера: сначала обновляем скорость, потом положение
            state_.dq[i] += dt * ddq;
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
            state_.q[i]  += dt * state_.dq[i];

            // Ограничение положений: если за пределами, обнуляем скорость в этом направлении
            if (state_.q[i] < qmin_[i]) {
                state_.q[i] = qmin_[i];
                if (state_.dq[i] < 0.0) state_.dq[i] = 0.0;
            }
            if (state_.q[i] > qmax_[i]) {
                state_.q[i] = qmax_[i];
                if (state_.dq[i] > 0.0) state_.dq[i] = 0.0;
            }
        }
    }

    /// Возвращает вектор допустимых пределов крутящих моментов (Нм).
    const std::vector<double>& torqueLimits() const { return taumax_; }

private:
    /**
     * @brief Сглаженная функция знака (sgn) через гиперболический тангенс.
     * Используется для моделирования кулоновского трения без разрыва.
     */
    static double sgn_soft(double x) {
        return std::tanh(8.0 * x);
    }

    /**
     * @brief Вычисляет диагональные элементы матрицы инерции M(q).
     *
     * Каждый элемент зависит от конфигурации: M_i = Jbase_i + Jvar_i * f(q).
     * Учитывается масштаб инерции (доменная рандомизация).
     */
    std::vector<double> computeMassDiag(const std::vector<double>& q) const {
        std::vector<double> M(dof_, 1.0);
        if (dof_ != 6) return M;

        // UR5e: a2=0.425м, a3=0.3922м
        // Сустав 1 (поворот основания): вертикальное вращение, несёт всю дистальную массу
        //   Вариация зависит от q[1] (насколько разогнута рука)
        M[0] = inertia_scale_ * (Jbase_[0] + Jvar_[0] * (1.0 + 0.22 * std::cos(q[1])));

        // Сустав 2 (подъём плеча): самый тяжело нагруженный
        //   m2*a2²/3 + (m3+m4+m5+m6)*a2² доминируется звеном 2 (8.393 кг)
        //   Зависит от q[1] и суммы q[1]+q[2]
        M[1] = inertia_scale_ * (Jbase_[1] + Jvar_[1] * (1.0 + 0.45 * std::cos(q[1]) + 0.30 * std::cos(q[1] + q[2])));

        // Сустав 3 (локоть): несёт предплечье (2.275 кг) + кисти (2.63 кг)
        M[2] = inertia_scale_ * (Jbase_[2] + Jvar_[2] * (1.0 + 0.55 * std::cos(q[2]) + 0.22 * std::cos(q[1] + q[2])));

        // Сустав 4 (кисть 1): лёгкий, d4=0.1333м, m4=1.219 кг
        M[3] = inertia_scale_ * (Jbase_[3] + Jvar_[3] * (1.0 + 0.12 * std::cos(q[3])));

        // Сустав 5 (кисть 2): d5=0.0997м, m5=1.219 кг
        M[4] = inertia_scale_ * (Jbase_[4] + Jvar_[4] * (1.0 + 0.10 * std::cos(q[4])));

        // Сустав 6 (кисть 3): самый лёгкий, d6=0.0996м, m6=0.1879 кг
        M[5] = inertia_scale_ * (Jbase_[5] + Jvar_[5] * (1.0 + 0.08 * std::cos(q[5])));

        // Минимальное значение для численной устойчивости
        for (double& x : M) x = std::max(0.03, x);
        return M;
    }

    /**
     * @brief Вычисляет вектор нелинейных сил n(q, dq).
     *
     * Включает:
     *   - Вязкое трение:   B_visc * dq
     *   - Кулоновское трение: F_coul * sgn(dq)
     *   - Гравитационные силы g(q)
     *   - Кориолисово сцепление между суставами
     */
    std::vector<double> computeNonlinear(const std::vector<double>& q,
                                         const std::vector<double>& dq) const {
        std::vector<double> n(dof_, 0.0);
        if (dof_ != 6) return n;

        // --- Вязкое + кулоновское трение ---
        for (size_t i = 0; i < dof_; ++i) {
            n[i] += friction_scale_ * (Bvisc_[i] * dq[i] + Fcoul_[i] * sgn_soft(dq[i]));
        }

        // --- Гравитационные члены (g = 9.81 м/с²) ---
        // UR5e: a2=0.425м, a3=0.3922м, d4=0.1333м, d5=0.0997м, d6=0.0996м
        // Массы звеньев: m2=8.393, m3=2.275, m4=1.219, m5=1.219, m6=0.1879 кг
        //
        // Сустав 2 (подъём плеча) — наибольшая гравитационная нагрузка:
        //   g2 ≈ 37.9*sin(q1) + 14.5*sin(q1+q2)
        n[1] += 37.9 * std::sin(q[1]) + 14.5 * std::sin(q[1] + q[2]);

        // Сустав 3 (локоть):  g3 ≈ 14.5*sin(q1+q2)
        n[2] += 14.5 * std::sin(q[1] + q[2]);

        // Сустав 4 (кисть 1): g4 ≈ 2.6*sin(q1+q2+q3)
        n[3] += 2.6 * std::sin(q[1] + q[2] + q[3]);

        // Сустав 5 (кисть 2): g5 ≈ 1.4*sin(q4)
        n[4] += 1.4 * std::sin(q[4]);

        // Сустав 6 (кисть 3): g6 ≈ 0.18*sin(q5)
        n[5] += 0.18 * std::sin(q[5]);

        // --- Кориолисово сцепление между суставами ---
        // Основное сцепление — между суставами 2-3 (плечо-локоть)
        //   h23 ≈ 2.55 * sin(q2)
        const double h23 = 2.55 * std::sin(q[2]);
        n[1] += h23 * (2.0 * dq[1] * dq[2] + dq[2] * dq[2]);
        n[2] -= h23 * (dq[1] * dq[1]);

        // Сцепление суставов 1-2 (основание-плечо)
        const double h12 = 0.50 * std::sin(q[1]);
        n[0] += h12 * dq[1] * dq[1];
        n[1] -= 0.25 * std::sin(q[1]) * dq[0] * dq[0];

        // Сцепление суставов 3-4 (локоть-кисть1)
        const double h34 = 0.18 * std::sin(q[3]);
        n[2] += h34 * (2.0 * dq[2] * dq[3] + dq[3] * dq[3]);
        n[3] -= h34 * (dq[2] * dq[2]);

        return n;
    }

    /// Ограничивает текущее состояние допустимыми пределами.
    void clampState() {
        for (size_t i = 0; i < dof_; ++i) {
            state_.q[i]  = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
        }
    }

    size_t dof_;              ///< Число степеней свободы
    ArmState state_;          ///< Текущее состояние манипулятора
    std::vector<double> tau_; ///< Буфер команды крутящих моментов

    std::vector<double> qmin_, qmax_;   ///< Пределы положений суставов (рад)
    std::vector<double> dqmax_;         ///< Пределы скоростей суставов (рад/с)
    std::vector<double> taumax_;        ///< Пределы крутящих моментов (Нм)

    std::vector<double> Jbase_, Jvar_;  ///< Базовая и вариационная инерция (кг·м²)
    std::vector<double> Bvisc_, Fcoul_; ///< Вязкое (Нм·с/рад) и кулоновское (Нм) трение
    double inertia_scale_{1.0};         ///< Масштаб инерции (доменная рандомизация)
    double friction_scale_{1.0};        ///< Масштаб трения (доменная рандомизация)
};
