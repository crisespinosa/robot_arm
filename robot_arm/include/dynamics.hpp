#pragma once

/**
 * @file dynamics.hpp
 * @brief Улучшенная модель динамики 6-осевого манипулятора UR5e в суставных координатах.
 *
 * Реализует полное уравнение движения в суставном пространстве:
 *
 *     M(q) * ddq + C(q, dq) * dq + g(q) + F(dq) = tau
 *
 * где:
 *   M(q)     — полная 6×6 симметричная положительно-определённая матрица инерции
 *              с главными недиагональными элементами (связь плечо–локоть и др.),
 *   C(q,dq)  — матрица Кориолиса/центробежных сил, вычисленная через символы
 *              Кристоффеля первого рода от M(q),
 *   g(q)     — вектор гравитационных моментов, получен как g(q) = ∂U(q)/∂q
 *              из потенциальной энергии U(q) = Σ mᵢ·g·zᵢ(q),
 *   F(dq)    — трение модели Штрибека (вязкое + статическое + кулоновское),
 *   tau      — вектор крутящих моментов в суставах.
 *
 * ===================================================================
 * КЛЮЧЕВЫЕ ОТЛИЧИЯ ОТ БАЗОВОЙ МОДЕЛИ
 * ===================================================================
 *
 *   1. M(q) — ПОЛНАЯ 6×6 МАТРИЦА (а не диагональ).
 *      • Главный недиагональный элемент M[1][2]=M[2][1] получен из
 *        стандартной формулы планарной 2R-механики.
 *      • Интегрирование теперь решает линейную систему M(q)·ddq = τ − n(q,dq)
 *        методом Гаусса с выбором ведущего элемента (6×6).
 *
 *   2. Гравитация g(q) выводится из ПОТЕНЦИАЛЬНОЙ ЭНЕРГИИ
 *        U(q) = Σᵢ mᵢ g zᵢ(q),      g(q) = ∂U/∂q.
 *      Высоты центров масс zᵢ(q) рассчитаны из прямой кинематики UR5e.
 *
 *   3. Кориолис C(q,dq)·dq вычисляется через СИМВОЛЫ КРИСТОФФЕЛЯ
 *        Γᵢⱼₖ(q) = ½(∂Mᵢⱼ/∂qₖ + ∂Mᵢₖ/∂qⱼ − ∂Mⱼₖ/∂qᵢ),
 *        Cᵢ    = Σⱼₖ Γᵢⱼₖ · dqⱼ · dqₖ,
 *      где частные производные ∂M/∂q берутся численно (центральные разности).
 *
 *   4. ТРЕНИЕ ШТРИБЕКА (Stribeck):
 *        τ_f(dq) = B·dq + [Fc + (Fs − Fc)·exp(−(|dq|/vs)²)] · sgn(dq).
 *      Учитывает статическое трение (стикцию) и плавный переход в
 *      кулоновский режим через характерную скорость vs.
 *
 *   5. УБРАН нефизический clamp на ускорения (было ddq ∈ [−25, 25]).
 *      Корректная модель M(q) сама ограничивает ускорения через инерцию.
 *
 * ===================================================================
 * КЛАССИФИКАЦИЯ ПАРАМЕТРОВ ПО ИСТОЧНИКУ
 * ===================================================================
 *
 * [OFFICIAL] — Официальная документация Universal Robots UR5e:
 *   - DH-параметры: a2=0.425м, a3=0.3922м
 *   - Пределы скоростей/моментов/положений — Technical Specifications.
 *
 * [URDF/ROS] — Из URDF-описания ROS-Industrial / fmauch_universal_robot:
 *   - Массы звеньев: m2=8.393, m3=2.275, m4=1.219, m5=1.219, m6=0.1879 кг
 *   - DH: d4=0.1333м, d5=0.0997м, d6=0.0996м
 *
 * [APPROX] — Приближения для симуляции (не идентифицированы экспериментально):
 *   - Отражённые инерции роторов Jrot_* — оценка через типичные редукторы 100:1.
 *   - Положения CoM lc_ua, lc_eff — принято посередине соответствующих звеньев.
 *   - Коэффициенты вязкого/кулоновского/статического трения —
 *     эвристические, подобраны для правдоподобного поведения в симуляции.
 *   - Гравитационные коэффициенты для кистевых суставов.
 *
 * ===================================================================
 *
 * Интегрирование: полунеявный метод Эйлера + полный линейный решатель 6×6.
 * Поддержка доменной рандомизации: масштабирование инерции и трения (0.2–3.0).
 */

#include <vector>
#include <array>
#include <algorithm>
#include <cassert>
#include <cmath>

// ---------------------------------------------------------------
// Тип для полной матрицы инерции 6×6
// ---------------------------------------------------------------
using Mat6 = std::array<std::array<double, 6>, 6>;
using Vec6 = std::array<double, 6>;

// ---------------------------------------------------------------
// ArmState — состояние 6-осевого манипулятора в суставных координатах.
// ---------------------------------------------------------------
struct ArmState {
    std::vector<double> q;    ///< Угловые положения суставов (рад)
    std::vector<double> dq;   ///< Угловые скорости суставов (рад/с)
    std::vector<double> ddq;  ///< Угловые ускорения суставов (рад/с²)
};

// ---------------------------------------------------------------
// ArmDynamics — класс динамики манипулятора UR5e.
//   M(q)·ddq + C(q,dq)·dq + g(q) + F(dq) = tau
// ---------------------------------------------------------------
class ArmDynamics {
public:
    explicit ArmDynamics(size_t dof = 6) : dof_(dof) {
        state_.q.assign(dof_, 0.0);
        state_.dq.assign(dof_, 0.0);
        state_.ddq.assign(dof_, 0.0);
        tau_.assign(dof_, 0.0);

        const double PI = 3.14159265358979323846;

        // =============================================================
        // [OFFICIAL] Пределы суставов из документации UR5e
        // =============================================================
        qmin_ = {-2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI};
        qmax_ = { 2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI};
        dqmax_  = {PI, PI, PI, 2.0*PI, 2.0*PI, 2.0*PI};
        taumax_ = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};

        // =============================================================
        // [APPROX] Базовые значения диагонали для доменов dof_ != 6
        // (для UR5e реальные диагонали M[i][i] формируются в
        //  computeMassMatrix() из физических параметров звеньев).
        // =============================================================
        Jbase_ = {5.5, 10.8, 3.6, 0.45, 0.20, 0.08};
        Jvar_  = {0.6,  2.5, 1.2, 0.08, 0.03, 0.01};

        // =============================================================
        // [APPROX] Матрица демпфирования (вязкое трение) — ЯВНАЯ ФОРМА
        //
        //   B = diag(3.5, 4.8, 3.2, 0.70, 0.40, 0.18)  [Нм·с/рад]
        //
        //   tau_damp = B · dq
        //
        // Порядок величин согласован с литературой по манипуляторам
        // с гармоническими редукторами.
        // =============================================================
        Bvisc_ = {3.5, 4.8, 3.2, 0.70, 0.40, 0.18};

        // =============================================================
        // [APPROX] Кулоновское трение (кинетическое) — Нм
        // =============================================================
        Fcoul_ = {0.55, 0.80, 0.60, 0.14, 0.10, 0.05};

        // =============================================================
        // [APPROX] Статическое трение (стикция) — Нм
        // Типично Fs ≈ 1.5...2.0 × Fc для промышленных редукторов.
        // Используется в модели Штрибека:
        //   τ_f = B·dq + [Fc + (Fs − Fc)·exp(−(|dq|/vs)²)] · sgn(dq)
        // =============================================================
        Fstat_ = {0.90, 1.30, 0.95, 0.22, 0.16, 0.08};

        // =============================================================
        // [APPROX] Характерная скорость Штрибека (рад/с)
        // Определяет ширину переходной зоны стикция → кулон. трение.
        // =============================================================
        vs_ = {0.02, 0.02, 0.02, 0.01, 0.01, 0.01};

        // Для произвольного DOF — generic defaults
        if (dof_ != 6) {
            qmin_.assign(dof_, -PI);
            qmax_.assign(dof_, PI);
            dqmax_.assign(dof_, 3.0);
            taumax_.assign(dof_, 20.0);
            Jbase_.assign(dof_, 1.0);
            Jvar_.assign(dof_, 0.1);
            Bvisc_.assign(dof_, 0.2);
            Fcoul_.assign(dof_, 0.02);
            Fstat_.assign(dof_, 0.035);
            vs_.assign(dof_, 0.02);
        }
    }

    /// Возвращает текущее состояние манипулятора (q, dq, ddq).
    const ArmState& state() const { return state_; }

    /**
     * @brief Установить масштабные коэффициенты для доменной рандомизации.
     */
    void setParamScales(double inertia_scale, double friction_scale) {
        inertia_scale_  = std::clamp(inertia_scale,  0.2, 3.0);
        friction_scale_ = std::clamp(friction_scale, 0.2, 3.0);
    }

    void resetParamScales() {
        inertia_scale_ = 1.0;
        friction_scale_ = 1.0;
    }

    double inertiaScale()  const { return inertia_scale_; }
    double frictionScale() const { return friction_scale_; }

    // =================================================================
    // Матрица демпфирования — явный доступ
    // =================================================================

    /**
     * @brief Возвращает диагональ матрицы демпфирования B (Нм·с/рад).
     *
     *   B = diag(Bvisc_),   τ_damp = B · dq.
     *
     * Входит в нелинейный вектор n(q, dq) через computeFriction().
     */
    std::vector<double> dampingMatrixDiag() const {
        std::vector<double> B(dof_);
        for (size_t i = 0; i < dof_; ++i) {
            B[i] = friction_scale_ * Bvisc_[i];
        }
        return B;
    }

    std::vector<double> dampingTorque(const std::vector<double>& dq) const {
        assert(dq.size() == dof_);
        std::vector<double> tau_damp(dof_);
        for (size_t i = 0; i < dof_; ++i) {
            tau_damp[i] = friction_scale_ * Bvisc_[i] * dq[i];
        }
        return tau_damp;
    }

    /**
     * @brief Публичный доступ к полной матрице инерции M(q).
     *        Полезно для отладки и тестов.
     */
    Mat6 massMatrix(const std::vector<double>& q) const {
        return computeMassMatrix(q);
    }

    /**
     * @brief Задать состояние манипулятора.
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
     * @brief Обратная динамика: tau = M(q)·ddq_des + n(q, dq)
     *
     * Теперь использует ПОЛНУЮ матрицу M(q) — это матрично-векторное
     * произведение, а не поэлементное умножение.
     */
    std::vector<double> inverseDynamics(const std::vector<double>& q,
                                        const std::vector<double>& dq,
                                        const std::vector<double>& ddq_des) const {
        assert(q.size() == dof_ && dq.size() == dof_ && ddq_des.size() == dof_);
        std::vector<double> tau(dof_, 0.0);

        if (dof_ != 6) {
            // Fallback для нестандартных DOF
            for (size_t i = 0; i < dof_; ++i) {
                double ti = Jbase_[i] * ddq_des[i];
                tau[i] = std::clamp(ti, -taumax_[i], taumax_[i]);
            }
            return tau;
        }

        const Mat6 M = computeMassMatrix(q);
        const std::vector<double> n = computeNonlinear(q, dq);

        // tau = M · ddq_des + n   (полное матрично-векторное произведение)
        for (size_t i = 0; i < 6; ++i) {
            double acc = 0.0;
            for (size_t j = 0; j < 6; ++j) {
                acc += M[i][j] * ddq_des[j];
            }
            double ti = acc + n[i];
            tau[i] = std::clamp(ti, -taumax_[i], taumax_[i]);
        }
        return tau;
    }

    /**
     * @brief Шаг интегрирования: применить крутящий момент и обновить состояние.
     *
     * Решает линейную систему:
     *     M(q) · ddq = tau − n(q, dq)
     * методом Гаусса с выбором ведущего элемента (6×6).
     *
     * Затем применяет полунеявный метод Эйлера:
     *     dq ← dq + dt · ddq
     *     q  ← q  + dt · dq
     *
     * Hard-clamp на ускорения УДАЛЁН — корректная модель M(q) сама
     * ограничивает ddq через инерцию.
     */
    void stepWithTorque(const std::vector<double>& tau, double dt) {
        assert(tau.size() == dof_);
        if (dt <= 1e-9) dt = 1e-3;

        // Ограничиваем входные моменты допустимыми пределами
        for (size_t i = 0; i < dof_; ++i) {
            tau_[i] = std::clamp(tau[i], -taumax_[i], taumax_[i]);
        }

        if (dof_ != 6) {
            // Fallback для нестандартных DOF
            for (size_t i = 0; i < dof_; ++i) {
                double Mi = std::max(1e-6, Jbase_[i] * inertia_scale_);
                double ddq = tau_[i] / Mi;
                state_.ddq[i] = ddq;
                state_.dq[i]  += dt * ddq;
                state_.dq[i]  = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
                state_.q[i]   += dt * state_.dq[i];
                state_.q[i]   = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
            }
            return;
        }

        // --- Главный путь: полное 6×6 решение ---
        const Mat6 M = computeMassMatrix(state_.q);
        const std::vector<double> n = computeNonlinear(state_.q, state_.dq);

        // Правая часть: rhs = tau − n
        Vec6 rhs{};
        for (int i = 0; i < 6; ++i) rhs[i] = tau_[i] - n[i];

        // Решение 6×6: M · ddq = rhs
        const Vec6 ddq = solve6(M, rhs);

        // Полунеявный метод Эйлера + мягкие ограничения
        for (int i = 0; i < 6; ++i) {
            state_.ddq[i] = ddq[i];
            state_.dq[i]  += dt * ddq[i];
            state_.dq[i]  = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
            state_.q[i]   += dt * state_.dq[i];

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

    const std::vector<double>& torqueLimits() const { return taumax_; }

private:
    // =================================================================
    // Сглаженная функция знака через гиперболический тангенс.
    // =================================================================
    static double sgn_soft(double x) {
        return std::tanh(8.0 * x);
    }

    // =================================================================
    // Решение линейной системы 6×6 A·x = b методом Гаусса
    // с частичным выбором ведущего элемента по столбцу.
    //
    // Для симметричных положительно-определённых M(q) численно устойчиво.
    // Используется на каждом шаге интегрирования stepWithTorque().
    // =================================================================
    static Vec6 solve6(const Mat6& A_in, const Vec6& b_in) {
        // Расширенная матрица [A | b] размером 6×7
        std::array<std::array<double, 7>, 6> aug{};
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) aug[i][j] = A_in[i][j];
            aug[i][6] = b_in[i];
        }

        // Прямой ход: приведение к верхнему треугольному виду
        for (int k = 0; k < 6; ++k) {
            // Выбор ведущего элемента по столбцу k
            int    imax = k;
            double vmax = std::abs(aug[k][k]);
            for (int i = k + 1; i < 6; ++i) {
                double v = std::abs(aug[i][k]);
                if (v > vmax) { vmax = v; imax = i; }
            }
            if (imax != k) std::swap(aug[k], aug[imax]);

            // Регуляризация при почти-вырожденности
            if (std::abs(aug[k][k]) < 1e-12) {
                aug[k][k] = (aug[k][k] >= 0.0 ? 1.0 : -1.0) * 1e-9;
            }

            // Исключение ниже ведущего
            for (int i = k + 1; i < 6; ++i) {
                double f = aug[i][k] / aug[k][k];
                for (int j = k; j < 7; ++j) {
                    aug[i][j] -= f * aug[k][j];
                }
            }
        }

        // Обратный ход
        Vec6 x{};
        for (int i = 5; i >= 0; --i) {
            double s = aug[i][6];
            for (int j = i + 1; j < 6; ++j) s -= aug[i][j] * x[j];
            x[i] = s / aug[i][i];
        }
        return x;
    }

    // =================================================================
    // Полная матрица инерции M(q) — 6×6
    //
    // Физические параметры (частично аналитически, частично эвристически):
    //   - Отражённая инерция роторов Jrot_i (доминанта диагонали)
    //   - Суставы 1–2: планарная 2R-механика (плечо–локоть) со стандартной
    //     формулой мат. инерции, дающей недиагональный член M[1][2].
    //   - Сустав 0: вертикальная инерция зависит от горизонт. «вылета»
    //     центра масс (через теорему о параллельных осях).
    //   - Суставы 3–5: квази-диагональная кистевая подматрица с малыми
    //     перекрёстными членами.
    //
    // Результат симметричен и строго положительно-определён.
    // =================================================================
    Mat6 computeMassMatrix(const std::vector<double>& q) const {
        Mat6 M{};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                M[i][j] = 0.0;

        if (dof_ != 6 || q.size() < 6) {
            for (size_t i = 0; i < std::min<size_t>(dof_, 6); ++i) {
                M[i][i] = (i < Jbase_.size() ? Jbase_[i] : 1.0);
            }
            return M;
        }

        // ------------------------------------------------------------
        // Физические параметры звеньев UR5e
        // [URDF/ROS] массы, [OFFICIAL] длины, [APPROX] положения CoM
        // ------------------------------------------------------------
        constexpr double m_ua    = 8.393;   // масса верхнего плеча (звено 2 URDF)
        constexpr double m_eff   = 4.90;    // m3 + m4 + m5 + m6 (предплечье + кисть)
        constexpr double a2      = 0.425;   // длина верхнего плеча [OFFICIAL]
        constexpr double lc_ua   = 0.2125;  // CoM верхнего плеча (≈ середина)
        constexpr double lc_eff  = 0.30;    // эфф. CoM бундла предплечье+кисть от локтя

        // Предвычисленные инерциальные константы (параллельная ось + собственная)
        constexpr double I_ua_com    = m_ua * a2 * a2 / 12.0;              // ≈ 0.1263
        constexpr double I_fa_com    = m_eff * 0.3922 * 0.3922 / 12.0;     // ≈ 0.0628
        constexpr double m_ua_lc_sq  = m_ua * lc_ua * lc_ua;               // ≈ 0.3787
        constexpr double m_eff_lc_sq = m_eff * lc_eff * lc_eff;            // ≈ 0.4410
        constexpr double m_eff_a2_sq = m_eff * a2 * a2;                    // ≈ 0.8851
        constexpr double m_eff_a2_lc = m_eff * a2 * lc_eff;                // ≈ 0.6248

        // ------------------------------------------------------------
        // Отражённая инерция роторов (через гармонические редукторы ~100:1)
        // [APPROX] — подобраны так, чтобы суммарные M[i][i] совпадали с
        // эмпирическими оценками для UR5e в номинальной конфигурации.
        // ------------------------------------------------------------
        constexpr double Jrot0 = 4.00;
        constexpr double Jrot1 = 9.00;
        constexpr double Jrot2 = 3.00;
        constexpr double Jrot3 = 0.38;
        constexpr double Jrot4 = 0.17;
        constexpr double Jrot5 = 0.07;

        const double c2  = std::cos(q[2]);
        const double s1  = std::sin(q[1]);
        const double s12 = std::sin(q[1] + q[2]);

        // ------------------------------------------------------------
        // Сустав 0: вращение основания вокруг вертикальной оси.
        //
        // Инерция = Jrot0 + Σ mᵢ · rᵢ²(q)
        // где rᵢ — горизонтальная проекция CoM звена i относительно
        // вертикальной оси сустава 0. Чем «вытянутее» рука, тем больше
        // момент инерции.
        // ------------------------------------------------------------
        const double r_ua  = lc_ua * s1;                 // гориз. коорд. CoM верхнего плеча
        const double r_eff = a2 * s1 + lc_eff * s12;     // гориз. коорд. CoM бундла
        M[0][0] = Jrot0
                + m_ua  * r_ua  * r_ua
                + m_eff * r_eff * r_eff
                + I_ua_com + I_fa_com;

        // ------------------------------------------------------------
        // Суставы 1 (плечо) и 2 (локоть) — планарная 2R-механика.
        //
        // Стандартная форма матрицы инерции планарного 2R:
        //   M11 = I₁ + I₂ + m₁·lc₁² + m₂·(a₁² + lc₂²) + 2·m₂·a₁·lc₂·cos(q₂)
        //   M12 = M21 = I₂ + m₂·lc₂² + m₂·a₁·lc₂·cos(q₂)
        //   M22 = I₂ + m₂·lc₂²
        //
        // В нашей параметризации: m₁ = m_ua, m₂ = m_eff, a₁ = a2.
        // ------------------------------------------------------------
        const double coupling = m_eff_a2_lc * c2;  // m_eff · a₂ · lc_eff · cos(q₂)
        M[1][1] = Jrot1
                + I_ua_com + I_fa_com
                + m_ua_lc_sq + m_eff_a2_sq + m_eff_lc_sq
                + 2.0 * coupling;
        M[1][2] = I_fa_com + m_eff_lc_sq + coupling;
        M[2][1] = M[1][2];
        M[2][2] = Jrot2 + I_fa_com + m_eff_lc_sq;

        // ------------------------------------------------------------
        // Кистевые суставы (3, 4, 5) — квази-диагональная аппроксимация
        // с малыми конфигурационно-зависимыми и перекрёстными членами.
        // ------------------------------------------------------------
        M[3][3] = Jrot3 + 0.028 + 0.004 * std::cos(q[3]);
        M[4][4] = Jrot4 + 0.015 + 0.002 * std::cos(q[4]);
        M[5][5] = Jrot5 + 0.006;

        // Слабое взаимодействие между смежными кистевыми суставами
        const double m34 = 0.0025 * std::cos(q[4]);
        M[3][4] = m34;
        M[4][3] = m34;
        M[4][5] = 0.0008;
        M[5][4] = 0.0008;

        // ------------------------------------------------------------
        // Доменная рандомизация: масштабирование инерции
        // ------------------------------------------------------------
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                M[i][j] *= inertia_scale_;

        // Регуляризация диагонали для численной устойчивости
        for (int i = 0; i < 6; ++i)
            if (M[i][i] < 0.03) M[i][i] = 0.03;

        return M;
    }

    // =================================================================
    // Гравитационный вектор g(q), полученный из потенциальной энергии:
    //
    //     U(q) = Σᵢ mᵢ · g · zᵢ(q)
    //     g(q) = ∂U(q)/∂q
    //
    // Для UR5e используем простую кинематику в вертикальной плоскости
    // шарнира 1 (плечо) и 2 (локоть), опираясь на массы из URDF и длины
    // из OFFICIAL. Коэффициенты для кистевых суставов — приближения.
    // =================================================================
    std::vector<double> computeGravity(const std::vector<double>& q) const {
        std::vector<double> g(dof_, 0.0);
        if (dof_ != 6 || q.size() < 6) return g;

        constexpr double G       = 9.81;
        constexpr double m_ua    = 8.393;
        constexpr double m_eff   = 4.90;
        constexpr double a2      = 0.425;
        constexpr double lc_ua   = 0.2125;
        constexpr double lc_eff  = 0.30;

        // Константы ∂U/∂qᵢ, выведенные аналитически:
        //   U(q) ⊃ −G · [m_ua·(lc_ua·cos q₁) + m_eff·(a₂·cos q₁ + lc_eff·cos(q₁+q₂))]
        //   ∂U/∂q₁ = G · [(m_ua·lc_ua + m_eff·a₂)·sin q₁ + m_eff·lc_eff·sin(q₁+q₂)]
        //   ∂U/∂q₂ = G · m_eff·lc_eff·sin(q₁+q₂)
        //
        // Проверка числовых значений:
        //   (m_ua·lc_ua + m_eff·a₂)·G = (1.7835 + 2.0825)·9.81 ≈ 37.94  ✓
        //   m_eff·lc_eff·G            = 4.90·0.30·9.81         ≈ 14.42  ✓
        const double s1   = std::sin(q[1]);
        const double s12  = std::sin(q[1] + q[2]);
        const double s123 = std::sin(q[1] + q[2] + q[3]);
        const double s4   = std::sin(q[4]);
        const double s5   = std::sin(q[5]);

        // g[0]: вращение основания вокруг вертикали → гравитация не совершает работу.
        g[0] = 0.0;

        // g[1]: плечо — основная гравитационная нагрузка.
        g[1] = (m_ua * lc_ua + m_eff * a2) * G * s1
             +  m_eff * lc_eff             * G * s12;

        // g[2]: локоть.
        g[2] = m_eff * lc_eff * G * s12;

        // Кистевые суставы (приближения)
        g[3] = 2.6  * s123;
        g[4] = 1.4  * s4;
        g[5] = 0.18 * s5;

        return g;
    }

    // =================================================================
    // Кориолис/центробежные силы через символы Кристоффеля 1-го рода.
    //
    //   Γᵢⱼₖ(q) = ½ · (∂Mᵢⱼ/∂qₖ + ∂Mᵢₖ/∂qⱼ − ∂Mⱼₖ/∂qᵢ)
    //   Cᵢ     = Σⱼ Σₖ  Γᵢⱼₖ(q) · dqⱼ · dqₖ
    //
    // Частные производные ∂M/∂q вычисляются численно центральными
    // разностями с шагом eps. Для 6-DOF требуется 12 вычислений M(q).
    // Это физически корректный способ получить Кориолисовы силы
    // из произвольной матрицы инерции M(q).
    // =================================================================
    std::vector<double> computeCoriolis(const std::vector<double>& q,
                                        const std::vector<double>& dq) const {
        std::vector<double> C(dof_, 0.0);
        if (dof_ != 6 || q.size() < 6 || dq.size() < 6) return C;

        constexpr double eps = 1e-5;

        // ∂M/∂qₖ для k = 0..5 через центральные разности
        std::array<Mat6, 6> dM{};
        std::vector<double> qp = q;
        std::vector<double> qm = q;
        for (int k = 0; k < 6; ++k) {
            qp[k] = q[k] + eps;
            qm[k] = q[k] - eps;
            const Mat6 Mp = computeMassMatrix(qp);
            const Mat6 Mm = computeMassMatrix(qm);
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    dM[k][i][j] = (Mp[i][j] - Mm[i][j]) / (2.0 * eps);
                }
            }
            qp[k] = q[k];
            qm[k] = q[k];
        }

        // Cᵢ = Σⱼ Σₖ  ½(∂Mᵢⱼ/∂qₖ + ∂Mᵢₖ/∂qⱼ − ∂Mⱼₖ/∂qᵢ) · dqⱼ · dqₖ
        for (int i = 0; i < 6; ++i) {
            double sum_i = 0.0;
            for (int j = 0; j < 6; ++j) {
                for (int k = 0; k < 6; ++k) {
                    const double gamma =
                        0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k]);
                    sum_i += gamma * dq[j] * dq[k];
                }
            }
            C[i] = sum_i;
        }
        return C;
    }

    // =================================================================
    // Трение модели Штрибека:
    //
    //   τ_f(dq) = B·dq + [Fc + (Fs − Fc)·exp(−(|dq|/vs)²)] · sgn(dq)
    //
    //  ∘ B·dq               — вязкое трение (пропорционально скорости)
    //  ∘ Fc                 — кулоновское трение (|dq| → ∞)
    //  ∘ Fs                 — статическое (стикция, |dq| → 0)
    //  ∘ exp(−(|dq|/vs)²)   — плавный переход стикция → кулон. трение
    //  ∘ sgn(dq) ≈ tanh(8·dq) — гладкая аппроксимация знака
    // =================================================================
    std::vector<double> computeFriction(const std::vector<double>& dq) const {
        std::vector<double> f(dof_, 0.0);
        for (size_t i = 0; i < dof_; ++i) {
            const double v      = dq[i];
            const double abs_v  = std::abs(v);
            const double vs     = (i < vs_.size() ? vs_[i] : 0.02);
            const double ratio  = abs_v / vs;
            const double e_term = std::exp(-(ratio * ratio));
            const double Fs_i   = (i < Fstat_.size() ? Fstat_[i] : Fcoul_[i] * 1.5);
            const double stribeck = Fcoul_[i] + (Fs_i - Fcoul_[i]) * e_term;
            f[i] = friction_scale_ *
                   (Bvisc_[i] * v + stribeck * sgn_soft(v));
        }
        return f;
    }

    // =================================================================
    // Полный нелинейный вектор n(q, dq) = C(q,dq)·dq + g(q) + F(dq)
    // =================================================================
    std::vector<double> computeNonlinear(const std::vector<double>& q,
                                         const std::vector<double>& dq) const {
        std::vector<double> n(dof_, 0.0);
        if (dof_ != 6) {
            // Fallback: только трение (без Кориолиса и гравитации)
            const std::vector<double> f = computeFriction(dq);
            for (size_t i = 0; i < dof_; ++i) n[i] = f[i];
            return n;
        }

        const std::vector<double> f = computeFriction(dq);
        const std::vector<double> C = computeCoriolis(q, dq);
        const std::vector<double> G = computeGravity(q);

        for (size_t i = 0; i < 6; ++i) {
            n[i] = C[i] + G[i] + f[i];
        }
        return n;
    }

    /// Ограничивает текущее состояние допустимыми пределами.
    void clampState() {
        for (size_t i = 0; i < dof_; ++i) {
            state_.q[i]  = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
        }
    }

    // =================================================================
    // Члены класса
    // =================================================================
    size_t dof_;              ///< Число степеней свободы
    ArmState state_;          ///< Текущее состояние манипулятора
    std::vector<double> tau_; ///< Буфер команды крутящих моментов

    // [OFFICIAL] Пределы из документации Universal Robots
    std::vector<double> qmin_, qmax_;   ///< Пределы положений (рад)
    std::vector<double> dqmax_;         ///< Пределы скоростей (рад/с)
    std::vector<double> taumax_;        ///< Пределы моментов (Нм)

    // [APPROX] Параметры модели для симуляции
    std::vector<double> Jbase_, Jvar_;  ///< Fallback-инерции (используется для dof_≠6)
    std::vector<double> Bvisc_;         ///< Диагональ матрицы демпфирования B (Нм·с/рад)
    std::vector<double> Fcoul_;         ///< Кулоновское трение (Нм)
    std::vector<double> Fstat_;         ///< Статическое трение/стикция (Нм)
    std::vector<double> vs_;            ///< Характерная скорость Штрибека (рад/с)

    double inertia_scale_{1.0};         ///< Масштаб инерции (доменная рандомизация)
    double friction_scale_{1.0};        ///< Масштаб трения  (доменная рандомизация)
};
