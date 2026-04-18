#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

/*
  Генератор опорной (эталонной) траектории minimum-jerk.

  Реализация: ЗАМКНУТАЯ АНАЛИТИЧЕСКАЯ ФОРМУЛА — квинтический полином
  (полином 5-й степени), являющийся известным оптимальным решением
  задачи minimum-jerk на тройном интеграторе с фиксированными граничными
  условиями. Нет двухточечной краевой задачи, нет пристрелки, нет
  итераций по сопряжённым переменным. Это НЕ численный решатель
  принципа максимума Понтрягина.

  Модель (для каждого сустава):
    x1 = q,  x2 = dq,  x3 = ddq
    u  = dddq (jerk, рывок)

  Функционал (minimum-jerk):
    J = ∫_0^T (1/2) ||u(t)||^2 dt

  Аналитическая форма траектории (замкнутое решение):
    q(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
*/

struct RefPoint {
    double t;
    std::vector<double> q;     // положение
    std::vector<double> dq;    // скорость
    std::vector<double> ddq;   // ускорение
    std::vector<double> u;     // jerk = dddq (производная тройного интегратора)

    // Диагностические поля. Вычисляются пост-фактум как производные квинтика
    // и НЕ используются в контуре управления (см. ArmController.cc). Оставлены
    // для совместимости с внешними скриптами анализа.
    std::vector<double> lambda1;
    std::vector<double> lambda2;
    std::vector<double> lambda3; // справочно: при данной форме квинтика u = -lambda3

    double J_acc = 0.0; // накопленная стоимость ≈ ∫_0^t (1/2)||u||^2 dt
};

// ------------------------------------------------------------
// Решение СЛАУ 6x6: A a = b (метод Гаусса с частичным выбором ведущего элемента)
// Используется для нахождения коэффициентов [a0..a5] квинтического полинома.
// ------------------------------------------------------------
inline std::vector<double> solve6(std::vector<std::vector<double>> A,
                                 std::vector<double> b)
{
    const int n = 6;

    // Проверка размерностей
    if ((int)A.size() != n || (int)b.size() != n) {
        throw std::runtime_error("solve6: bad dimensions");
    }
    for (int i = 0; i < n; ++i) {
        if ((int)A[i].size() != n) throw std::runtime_error("solve6: bad matrix row");
        A[i].push_back(b[i]); // формируем расширенную матрицу [A|b]
    }

    // Прямой ход (обнуление элементов под диагональю) + частичный выбор ведущего элемента
    for (int col = 0; col < n; ++col) {
        int piv = col;
        double best = std::fabs(A[col][col]);

        // Ищем максимальный по модулю элемент в текущем столбце (для устойчивости)
        for (int r = col + 1; r < n; ++r) {
            double v = std::fabs(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }

        // Если ведущий элемент слишком мал — система вырожденная/почти вырожденная
        if (best < 1e-12) throw std::runtime_error("solve6: singular system");

        // Меняем строки местами, чтобы ведущий элемент оказался на диагонали
        if (piv != col) std::swap(A[piv], A[col]);

        // Нормируем ведущую строку (делаем диагональный элемент равным 1)
        double diag = A[col][col];
        for (int c = col; c <= n; ++c) A[col][c] /= diag;

        // Обнуляем элементы ниже ведущего в этом столбце
        for (int r = col + 1; r < n; ++r) {
            double f = A[r][col];
            for (int c = col; c <= n; ++c) A[r][c] -= f * A[col][c];
        }
    }

    // Обратный ход (подстановка) для получения решения
    std::vector<double> x(n, 0.0);
    for (int r = n - 1; r >= 0; --r) {
        double s = A[r][n]; // правая часть
        for (int c = r + 1; c < n; ++c) s -= A[r][c] * x[c];
        x[r] = s; // на диагонали уже 1
    }

    return x; // возвращает [a0..a5]
}

// ------------------------------------------------------------
// Коэффициенты квинтика при общих граничных условиях:
//   q(0)=q0, dq(0)=v0, ddq(0)=a0
//   q(T)=q1, dq(T)=v1, ddq(T)=a1
// Возвращает a = [a0..a5] для q(t)
// ------------------------------------------------------------
inline std::vector<double> quintic_coeffs(double q0, double v0, double a0,
                                         double q1, double v1, double a1,
                                         double T)
{
    // Защита: слишком малое T приведёт к плохой обусловленности/делению на ~0
    if (T <= 1e-9) throw std::runtime_error("quintic_coeffs: T too small");

    // Предвычисляем степени T (ускоряет формирование матрицы)
    const double TT  = T;
    const double TT2 = TT*TT;
    const double TT3 = TT2*TT;
    const double TT4 = TT3*TT;
    const double TT5 = TT4*TT;

    // Строим систему A*a = b для неизвестных коэффициентов a0..a5
    std::vector<std::vector<double>> A(6, std::vector<double>(6, 0.0));
    std::vector<double> b(6, 0.0);

    // Условие q(0)=q0:  a0 = q0
    A[0][0] = 1.0; b[0] = q0;

    // Условие dq(0)=v0: a1 = v0
    A[1][1] = 1.0; b[1] = v0;

    // Условие ddq(0)=a0: 2*a2 = a0
    A[2][2] = 2.0; b[2] = a0;

    // Условие q(T)=q1: a0 + a1 T + a2 T^2 + a3 T^3 + a4 T^4 + a5 T^5 = q1
    A[3][0] = 1.0;  A[3][1] = TT;   A[3][2] = TT2;
    A[3][3] = TT3;  A[3][4] = TT4;  A[3][5] = TT5;
    b[3] = q1;

    // Условие dq(T)=v1: a1 + 2 a2 T + 3 a3 T^2 + 4 a4 T^3 + 5 a5 T^4 = v1
    A[4][1] = 1.0;
    A[4][2] = 2.0*TT;
    A[4][3] = 3.0*TT2;
    A[4][4] = 4.0*TT3;
    A[4][5] = 5.0*TT4;
    b[4] = v1;

    // Условие ddq(T)=a1: 2 a2 + 6 a3 T + 12 a4 T^2 + 20 a5 T^3 = a1
    A[5][2] = 2.0;
    A[5][3] = 6.0*TT;
    A[5][4] = 12.0*TT2;
    A[5][5] = 20.0*TT3;
    b[5] = a1;

    // Решаем систему и получаем коэффициенты квинтического полинома
    return solve6(A, b);
}


// ------------------------------------------------------------
// Планирование траектории minimum-jerk с помощью квинтика (только q)
// Улучшение: “устойчивая” сетка времени, где t_N = T строго (без ошибок округления)
// ------------------------------------------------------------
// Теория (смысл):
//  - Хотим получить максимально плавное движение, минимизируя рывок (jerk):
//        J = ∫_0^T (1/2) ||u(t)||^2 dt,   где u(t) = dddq(t)
//  - При заданных граничных условиях по q, dq и ddq на концах отрезка
//    оптимальная траектория для каждого сустава имеет вид квинтика (полином 5-й степени):
//        q(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
//  - Эта функция вычисляет ТОЛЬКО q(t) и возвращает таблицу строк вида:
//        [t, q1(t), q2(t), ..., q_dof(t)]
inline std::vector<std::vector<double>> plan_minjerk(
    const std::vector<double>& q0,   // начальные положения суставов (размер dof)
    const std::vector<double>& q1,   // конечные положения суставов   (размер dof)
    double T,                        // длительность движения
    double dt)                       // желаемый шаг (будет скорректирован)
{
    // dof = число степеней свободы (кол-во суставов)
    const size_t dof = q0.size();

    // Проверка: q0 и q1 должны быть одной размерности (одинаковое число суставов)
    if (q1.size() != dof) throw std::runtime_error("plan_minjerk: size mismatch");

    // Проверка: время движения должно быть положительным
    if (T <= 0.0) throw std::runtime_error("plan_minjerk: T must be > 0");

    // Защита от нулевого/слишком малого шага (чисто для численной устойчивости)
    dt = std::max(dt, 1e-9);

    // Выбираем число интервалов N на [0, T].
    // ceil(T/dt) гарантирует шаг не хуже требуемого (по плотности сетки).
    // max(2, ...) — чтобы минимум было 2 интервала (хотя бы 3 точки).
    int N = std::max(2, (int)std::ceil(T / dt));

    // Эффективный шаг сетки: dt_eff = T/N.
    // Важная идея: тогда t_N = N*dt_eff = T ТОЧНО.
    // Это избавляет от накопления ошибки округления и гарантирует попадание в финальный момент.
    double dt_eff = T / N;

    // Выход: вектор строк, каждая строка будет [t, q[0], q[1], ..., q[dof-1]]
    std::vector<std::vector<double>> out;
    out.reserve((size_t)N + 1); // N+1 точек: k = 0..N

    // Коэффициенты квинтика по каждому суставу:
    // coeffs[i] = [a0,a1,a2,a3,a4,a5] для i-го сустава
    std::vector<std::vector<double>> coeffs(dof);

    // Стандартный случай “покой → покой”:
    //   dq(0)=ddq(0)=dq(T)=ddq(T)=0
    // Поэтому передаём нули для скоростей и ускорений на концах.
    for (size_t i = 0; i < dof; ++i) {
        coeffs[i] = quintic_coeffs(q0[i], 0.0, 0.0, q1[i], 0.0, 0.0, T);
    }

    // Проходим по сетке времени t_k = k*dt_eff, k=0..N
    for (int k = 0; k <= N; ++k) {
        double t = k * dt_eff;

        // Формируем строку: сначала время, далее значения q(t) по всем суставам
        std::vector<double> row(1 + (int)dof, 0.0);
        row[0] = t;

        // Предвычисляем степени t, чтобы не пересчитывать их внутри цикла по dof
        const double tt  = t;
        const double tt2 = tt * tt;
        const double tt3 = tt2 * tt;
        const double tt4 = tt3 * tt;
        const double tt5 = tt4 * tt;

        // Вычисляем q_i(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
        for (size_t i = 0; i < dof; ++i) {
            const auto& a = coeffs[i];
            row[1 + (int)i] = a[0] + a[1]*tt + a[2]*tt2 + a[3]*tt3 + a[4]*tt4 + a[5]*tt5;
        }

        // Сохраняем строку в выход
        out.push_back(std::move(row));
    }

    // Возвращаем таблицу [t, q1(t), q2(t), ...]
    return out;
}

// ------------------------------------------------------------
// Генератор опорной траектории minimum-jerk (ОБЩИЙ случай):
// допускает произвольные граничные условия по dq и ddq.
// Устойчивая сетка + корректное накопление стоимости с реальным шагом dt_step.
// Реализация — аналитический квинтик (замкнутая формула), а НЕ численный ПМП.
// ------------------------------------------------------------
// Теория (смысл):
//  - Модель по каждому суставу — «тройной интегратор»:
//        x1 = q,  x2 = dq,  x3 = ddq
//        x1' = x2
//        x2' = x3
//        x3' = u    (u = dddq, jerk)
//  - Функционал minimum-jerk:
//        J = ∫_0^T (1/2) ||u(t)||^2 dt
//  - Известный результат: при фиксированных q, dq, ddq на концах
//    оптимальная по этому функционалу траектория — квинтический полином.
//    Мы его и строим напрямую, без итераций.
//  - Возвращаем: q, dq, ddq, u (=jerk), диагностические λ (не используются
//    контуром управления), и J_acc — накопленную стоимость.
inline std::vector<RefPoint> plan_minjerk_traj(
    const std::vector<double>& q0,    // q(0)
    const std::vector<double>& dq0,   // dq(0)
    const std::vector<double>& ddq0,  // ddq(0)
    const std::vector<double>& q1,    // q(T)
    const std::vector<double>& dq1,   // dq(T)
    const std::vector<double>& ddq1,  // ddq(T)
    double T,
    double dt)
{
    // dof = число суставов
    const size_t dof = q0.size();

    // Проверка: все векторы должны иметь одинаковую размерность dof
    if (dq0.size()!=dof || ddq0.size()!=dof || q1.size()!=dof || dq1.size()!=dof || ddq1.size()!=dof)
        throw std::runtime_error("plan_minjerk_traj(general): size mismatch");

    // Проверка времени
    if (T <= 0.0) throw std::runtime_error("plan_minjerk_traj: T must be > 0");

    // Защита от dt ~ 0
    dt = std::max(dt, 1e-9);

    // Устойчивая сетка: N = ceil(T/dt), затем dt_eff = T/N, чтобы t_N = T ровно
    int N = std::max(2, (int)std::ceil(T / dt));
    double dt_eff = T / N;

    // Выход: N+1 точек
    std::vector<RefPoint> out;
    out.reserve((size_t)N + 1);

    // Коэффициенты квинтика для каждого сустава, удовлетворяющие:
    //  q(0)=q0, dq(0)=dq0, ddq(0)=ddq0
    //  q(T)=q1, dq(T)=dq1, ddq(T)=ddq1
    std::vector<std::vector<double>> coeffs(dof);
    for (size_t i = 0; i < dof; ++i) {
        coeffs[i] = quintic_coeffs(q0[i], dq0[i], ddq0[i], q1[i], dq1[i], ddq1[i], T);
    }

    // J_acc — дискретное приближение интеграла стоимости:
    //   J ≈ Σ (1/2)||u(t_k)||^2 * Δt_k
    double J_acc = 0.0;
    double prev_t = 0.0;

    // Проходим по t_k = k*dt_eff
    for (int k = 0; k <= N; ++k) {
        double t = k * dt_eff;

        // Реальный шаг между соседними точками (для k=0 шага нет)
        double dt_step = (k == 0) ? 0.0 : (t - prev_t);
        prev_t = t;

        // Степени времени для вычисления q(t) и производных
        const double tt  = t;
        const double tt2 = tt * tt;
        const double tt3 = tt2 * tt;
        const double tt4 = tt3 * tt;
        const double tt5 = tt4 * tt;

        // Создаём точку траектории
        RefPoint p;
        p.t = t;
        p.q.assign(dof, 0.0);
        p.dq.assign(dof, 0.0);
        p.ddq.assign(dof, 0.0);
        p.u.assign(dof, 0.0);
        p.lambda1.assign(dof, 0.0);
        p.lambda2.assign(dof, 0.0);
        p.lambda3.assign(dof, 0.0);
        p.J_acc = 0.0;

        // Для каждого сустава:
        //  1) считаем q, dq, ddq из квинтика
        //  2) считаем u = dddq (jerk) — управляющее воздействие тройного интегратора
        //  3) считаем λ для “видимости PMP” (согласовано с условием u* = -λ3)
        for (size_t i = 0; i < dof; ++i) {
            const auto& a = coeffs[i];

            // q_i(t) — положение
            p.q[i] = a[0] + a[1]*tt + a[2]*tt2 + a[3]*tt3 + a[4]*tt4 + a[5]*tt5;

            // dq_i(t) = d/dt q_i(t) — скорость
            p.dq[i] = a[1]
                    + 2.0*a[2]*tt
                    + 3.0*a[3]*tt2
                    + 4.0*a[4]*tt3
                    + 5.0*a[5]*tt4;

            // ddq_i(t) = d^2/dt^2 q_i(t) — ускорение
            p.ddq[i] = 2.0*a[2]
                     + 6.0*a[3]*tt
                     + 12.0*a[4]*tt2
                     + 20.0*a[5]*tt3;

            // u_i(t) = d^3/dt^3 q_i(t) — jerk (управление)
            p.u[i] = 6.0*a[3]
                   + 24.0*a[4]*tt
                   + 60.0*a[5]*tt2;

            // Диагностические величины (НЕ используются контуром управления).
            // Считаются пост-фактум как производные уже построенного квинтика.
            // Выбор формул согласован с классической постановкой minimum-jerk,
            // но сами они здесь не решают никакую задачу, а лишь сохраняются
            // для внешних скриптов анализа.
            p.lambda3[i] = -p.u[i];                       // по построению u = -λ3
            const double du_dt   = 24.0*a[4] + 120.0*a[5]*tt;  // du/dt
            const double d2u_dt2 = 120.0*a[5];                 // d²u/dt²
            p.lambda2[i] = du_dt;
            p.lambda1[i] = -d2u_dt2;
        }

        // ||u||^2 = Σ u_i^2 — квадрат нормы вектора jerk по всем суставам
        double u2 = 0.0;
        for (size_t i = 0; i < dof; ++i) u2 += p.u[i] * p.u[i];

        // Накопление стоимости:
        //  J_acc += (1/2) ||u||^2 * Δt
        J_acc += 0.5 * u2 * dt_step;
        p.J_acc = J_acc;

        // Сохраняем точку
        out.push_back(std::move(p));
    }

    // Возвращаем всю дискретизированную траекторию (по точкам)
    return out;
}

// ------------------------------------------------------------
// Wrapper (совместимость): текущий ArmController вызывает именно эту сигнатуру.
// Предполагаем dq0=ddq0=dq1=ddq1=0 (покой → покой).
// ------------------------------------------------------------
// Эта перегрузка нужна, чтобы не менять старый код.
// Она просто подставляет нулевые скорости и ускорения и вызывает общий вариант.
inline std::vector<RefPoint> plan_minjerk_traj(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    double T, double dt)
{
    const size_t dof = q0.size();
    if (q1.size() != dof) throw std::runtime_error("plan_minjerk_traj: size mismatch");

    // z — вектор нулей для скоростей и ускорений на концах
    std::vector<double> z(dof, 0.0);

    // Вызов общего варианта с “покой → покой”
    return plan_minjerk_traj(q0, z, z, q1, z, z, T, dt);
}

