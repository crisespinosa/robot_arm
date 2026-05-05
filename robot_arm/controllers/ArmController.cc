/**
 * @file ArmController.cc
 * @brief REST контроллер для UR5e робота-манипулятора.
 *
 * Реализация HTTP-контроллера на базе Drogon. Ядро текущего этапа проекта —
 * конечногоризонтный LQR с рекурсией Риккати на локальной модели ошибки
 * (двойной интегратор по каждому суставу).
 *
 * Состав модуля:
 *   - Генерация опорной траектории minimum-jerk (квинтический полином,
 *     замкнутая формула). Это НЕ численный решатель принципа максимума
 *     Понтрягина. HTTP endpoint: POST /arm/plan_minjerk_q.
 *   - Управление LQR с конечным горизонтом (основа этапа 1) и упрощённый
 *     PD-режим для сравнения.
 *   - Фильтр Калмана 2-го порядка (KF2) — оценка скорости по измерениям q.
 *
 * Математическая основа LQR-контура:
 *   Локальная модель:      x[k+1] = A x[k] + B u[k],  x = [eq, edq]^T
 *   Рекурсия Риккати:      P[k]  = Q + A^T P[k+1] A
 *                                   - A^T P[k+1] B (R + B^T P[k+1] B)^{-1} B^T P[k+1] A
 *   Закон управления:      u[k]  = -K[k] x[k],
 *                          K[k]  = (R + B^T P[k+1] B)^{-1} B^T P[k+1] A
 *
 * @see ArmDynamics — полная нелинейная модель UR5e, компенсируется через
 *      inverseDynamics.
 * @see RefPoint   — точка опорной траектории minimum-jerk.
 */

#include "ArmController.h"

#include <drogon/HttpAppFramework.h>
#include <json/json.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace drogon;

// ============================================================
// JSON вспомогательные функции
// ============================================================

/**
 * @brief Преобразует вектор в JSON массив размером ровно 6 элементов
 * @param q_in Входной вектор
 * @return JSON массив, дополненный нулями до размера 6
 */
static Json::Value to_q6_json(const std::vector<double>& q_in) {
    Json::Value q(Json::arrayValue);
    for (int i = 0; i < 6; ++i) {
        q.append(i < (int)q_in.size() ? q_in[i] : 0.0);
    }
    return q;
}

/**
 * @brief Преобразует вектор в JSON массив произвольного размера
 * @param v Входной вектор
 * @return JSON массив
 */
static Json::Value to_json_array(const std::vector<double>& v) {
    Json::Value out(Json::arrayValue);
    for (double x : v) out.append(x);
    return out;
}

/**
 * @brief Читает JSON массив с проверкой типа и размера
 * @param root Корневой JSON объект
 * @param key Ключ для чтения
 * @param out Выходной вектор (будет переменного размера)
 * @param err Сообщение об ошибке (если ошибка)
 * @param min_len Минимальная длина (если exact_len < 0)
 * @param exact_len Точная длина (-1 = не проверять)
 * @return true если успешно прочитано, иначе false
 */
static bool read_array_double(const Json::Value& root,
                              const char* key,
                              std::vector<double>& out,
                              std::string& err,
                              int min_len = 0,
                              int exact_len = -1) {
    if (!root.isMember(key)) {
        err = std::string("Missing key: ") + key;
        return false;
    }
    const auto& arr = root[key];
    if (!arr.isArray()) {
        err = std::string(key) + " must be an array";
        return false;
    }
    if (exact_len >= 0 && (int)arr.size() != exact_len) {
        err = std::string(key) + " must have length " + std::to_string(exact_len);
        return false;
    }
    if (exact_len < 0 && (int)arr.size() < min_len) {
        err = std::string(key) + " must have at least " + std::to_string(min_len) + " values";
        return false;
    }
    out.resize(arr.size());
    for (int i = 0; i < (int)arr.size(); ++i) out[i] = arr[i].asDouble();
    return true;
}

/**
 * @brief Опционально читает 6-мерный вектор конфигурации из JSON
 * @param root Корневой JSON объект
 * @param key Ключ для чтения
 * @param out6 Выходной 6-мерный вектор
 * @param err Сообщение об ошибке (если ошибка)
 * @return true если ключ присутствует и успешно прочитан, иначе false
 */
static bool read_q6_optional(const Json::Value& root,
                             const char* key,
                             std::vector<double>& out6,
                             std::string& err) {
    if (!root.isMember(key)) return false;
    const auto& arr = root[key];
    if (!arr.isArray() || (int)arr.size() < 6) {
        err = std::string(key) + " must be array length >= 6";
        return false;
    }
    out6 = {arr[0].asDouble(), arr[1].asDouble(), arr[2].asDouble(),
            arr[3].asDouble(), arr[4].asDouble(), arr[5].asDouble()};
    return true;
}

/**
 * @brief Создает JSON объект ошибки
 * @param msg Сообщение об ошибке
 * @return JSON объект с полями: {"ok": false, "error": msg}
 */
static Json::Value make_error(const std::string& msg) {
    Json::Value out(Json::objectValue);
    out["ok"] = false;
    out["error"] = msg;
    return out;
}

/**
 * @brief Парсит JSON тело HTTP запроса
 *
 * Сначала пытается получить встроенный JSON из запроса, затем парсит
 * сырое тело запроса если встроенный JSON недоступен.
 *
 * @param req HTTP запрос
 * @param fallback_root Fallback JSON объект для сохранения результата парсинга
 * @return Указатель на JSON объект, или nullptr если парсинг не удался
 */
static std::shared_ptr<Json::Value> parse_json_body(const HttpRequestPtr& req, Json::Value& fallback_root) {
    auto json = req->getJsonObject();
    if (json) return json;

    const auto& body = req->getBody();
    Json::CharReaderBuilder b;
    std::string errs;
    std::unique_ptr<Json::CharReader> reader(b.newCharReader());
    if (!reader->parse(body.data(), body.data() + body.size(), &fallback_root, &errs)) {
        return nullptr;
    }
    return std::make_shared<Json::Value>(fallback_root);
}

// ============================================================
// Вспомогательные функции линейной алгебры (2x2 матрицы)
// ============================================================

/**
 * @struct Mat2
 * @brief 2x2 матрица для математических операций в Риккати рекурсии
 *
 * Используется в конечном горизонте LQR для рекурсивного вычисления
 * матрицы Риккати P[k] и коэффициентов обратной связи K[k].
 */
struct Mat2 {
    double a00{0}, a01{0}, a10{0}, a11{0};
};

/**
 * @brief Сложение матриц: C = A + B
 */
static Mat2 mat2_add(const Mat2& A, const Mat2& B) {
    return {A.a00+B.a00, A.a01+B.a01, A.a10+B.a10, A.a11+B.a11};
}

/**
 * @brief Вычитание матриц: C = A - B
 */
static Mat2 mat2_sub(const Mat2& A, const Mat2& B) {
    return {A.a00-B.a00, A.a01-B.a01, A.a10-B.a10, A.a11-B.a11};
}

/**
 * @brief Умножение матриц: C = A * B
 *
 * Вычисляет произведение двух 2x2 матриц по формуле:
 * C[i,j] = sum_k A[i,k] * B[k,j]
 */
static Mat2 mat2_mul(const Mat2& A, const Mat2& B) {
    return {
        A.a00*B.a00 + A.a01*B.a10,
        A.a00*B.a01 + A.a01*B.a11,
        A.a10*B.a00 + A.a11*B.a10,
        A.a10*B.a01 + A.a11*B.a11
    };
}

/**
 * @brief Транспонирование матрицы: C = A^T
 */
static Mat2 mat2_T(const Mat2& A) {
    return {A.a00, A.a10, A.a01, A.a11};
}

/**
 * @brief Внешнее произведение: C = v * w^T
 *
 * Для двумерных векторов v и w вычисляет матрицу:
 * C[i,j] = v[i] * w[j]
 */
static Mat2 mat2_outer2(const std::array<double,2>& v, const std::array<double,2>& w) {
    return {v[0]*w[0], v[0]*w[1], v[1]*w[0], v[1]*w[1]};
}

// ============================================================
// Дискретизация сохранённой опорной траектории (minimum-jerk)
// ============================================================

/**
 * @struct RefSample
 * @brief Образец отсчета опорной траектории в момент времени t
 *
 * Содержит желаемые позицию (q), скорость (dq) и ускорение (ddq)
 * из опорной траектории, полученные линейной интерполяцией двух
 * соседних её точек.
 */
struct RefSample {
    std::vector<double> q, dq, ddq;
    double t{0.0};
};

/**
 * @brief Линейная интерполяция между двумя опорными точками
 *
 * Выполняет линейную интерполяцию всех компонентов (q, dq, ddq) между
 * точками a и b на момент времени t. Если b.t == a.t, возвращает точку a.
 *
 * @param a Первая точка траектории
 * @param b Вторая точка траектории
 * @param t Момент времени интерполяции
 * @return Интерполированный образец с q[i] = qa[i] + alpha * (qb[i] - qa[i])
 */
static RefSample lerp_ref(const RefPoint& a, const RefPoint& b, double t) {
    double alpha = 0.0;
    if (b.t > a.t) alpha = (t - a.t) / (b.t - a.t);
    alpha = std::clamp(alpha, 0.0, 1.0);

    RefSample s;
    s.t = t;
    s.q.resize(6); s.dq.resize(6); s.ddq.resize(6);
    for (int i = 0; i < 6; ++i) {
        const double qa = (i < (int)a.q.size()) ? a.q[i] : 0.0;
        const double qb = (i < (int)b.q.size()) ? b.q[i] : 0.0;
        const double dqa = (i < (int)a.dq.size()) ? a.dq[i] : 0.0;
        const double dqb = (i < (int)b.dq.size()) ? b.dq[i] : 0.0;
        const double ddqa = (i < (int)a.ddq.size()) ? a.ddq[i] : 0.0;
        const double ddqb = (i < (int)b.ddq.size()) ? b.ddq[i] : 0.0;

        s.q[i]   = qa   + alpha * (qb   - qa);
        s.dq[i]  = dqa  + alpha * (dqb  - dqa);
        s.ddq[i] = ddqa + alpha * (ddqb - ddqa);
    }
    return s;
}

/**
 * @brief Дискретизирует опорную траекторию в заданный момент времени
 *
 * Производит дискретизацию сохранённой опорной траектории (набор точек) в
 * точку времени t с шагом dt_ref. Использует линейную интерполяцию между
 * точками. Пустая траектория возвращает нулевой образец.
 *
 * @param traj_copy Копия сохраненной траектории (вектор RefPoint)
 * @param t Желаемый момент времени
 * @param dt_ref Шаг дискретизации исходной траектории (для индексирования)
 * @return RefSample с q, dq, ddq в момент t
 */
static RefSample sample_ref_from_traj(const std::vector<RefPoint>& traj_copy, double t, double dt_ref) {
    RefSample ref;
    if (traj_copy.empty()) {
        ref.q.assign(6, 0.0);
        ref.dq.assign(6, 0.0);
        ref.ddq.assign(6, 0.0);
        ref.t = t;
        return ref;
    }
    if (traj_copy.size() == 1) {
        ref.t = t;
        ref.q = traj_copy[0].q;
        ref.dq = traj_copy[0].dq;
        ref.ddq = traj_copy[0].ddq;
        if ((int)ref.q.size() < 6) ref.q.resize(6, 0.0);
        if ((int)ref.dq.size() < 6) ref.dq.resize(6, 0.0);
        if ((int)ref.ddq.size() < 6) ref.ddq.resize(6, 0.0);
        return ref;
    }

    const int n = (int)traj_copy.size();
    int idx = (int)std::floor(t / dt_ref);
    if (idx < 0) idx = 0;
    if (idx >= n - 1) idx = n - 2;
    return lerp_ref(traj_copy[idx], traj_copy[idx + 1], t);
}

/**
 * @brief Вычисляет RMS ошибку между двумя векторами
 *
 * RMS(a, b) = sqrt(sum_i (a[i] - b[i])^2 / n)
 *
 * @param a Первый вектор
 * @param b Второй вектор
 * @return RMS норма разности (берется минимум длин векторов)
 */
static double rms_between(const std::vector<double>& a, const std::vector<double>& b) {
    double s = 0.0;
    const int n = std::min((int)a.size(), (int)b.size());
    if (n == 0) return 0.0;
    for (int i = 0; i < n; ++i) {
        double e = a[i] - b[i];
        s += e * e;
    }
    return std::sqrt(s / (double)n);
}

/**
 * @brief Вычисляет среднее квадратичное значение вектора
 *
 * mean_sq(a) = sum_i a[i]^2 / n
 *
 * @param a Входной вектор
 * @return Средний квадрат компонент вектора
 */
static double mean_sq(const std::vector<double>& a) {
    if (a.empty()) return 0.0;
    double s = 0.0;
    for (double x : a) s += x * x;
    return s / (double)a.size();
}

// ============================================================
// LQR: коэффициенты обратной связи для локальной модели двойного
// интегратора (управление предсказывает желаемое ddq, а растение —
// полная нелинейная модель ArmDynamics, момент считается через
// inverse dynamics)
// ============================================================

/**
 * @brief Вычисляет коэффициент обратной связи LQR для конечного горизонта
 *
 * Решает задачу конечного горизонта LQR с использованием обратной Риккати рекурсии.
 *
 * Система:
 *   x[k+1] = A*x[k] + B*u[k]
 *   где x = [q, dq]^T - состояние двойного интегратора
 *   A = [1 dt; 0 1], B = [0.5*dt^2; dt]
 *
 * Стоимость:
 *   J = sum_{k=0}^{N-1} (x[k]'*Q*x[k] + u[k]'*R*u[k]) + x[N]'*P_N*x[N]
 *   где Q = diag(wq, wdq), R = wu, P_N = diag(wqN, wdqN)
 *
 * Обратная Риккати рекурсия (из шага N-1 к шагу 0):
 *   P[k] = Q + A'*P[k+1]*A - A'*P[k+1]*B*(R + B'*P[k+1]*B)^{-1}*B'*P[k+1]*A
 *   K[k] = (R + B'*P[k+1]*B)^{-1} * B' * P[k+1] * A
 *
 * Управление:
 *   u[k] = -K[k]*x[k]
 *
 * @param dt Период дискретизации
 * @param N Горизонт предсказания (количество шагов)
 * @param wq Вес целевой функции на позиции (Q диагональ [0,0])
 * @param wdq Вес целевой функции на скорости (Q диагональ [1,1])
 * @param wu Вес целевой функции на управлении (матрица R)
 * @param wqN Терминальный вес на позиции (P_N диагональ [0,0])
 * @param wdqN Терминальный вес на скорости (P_N диагональ [1,1])
 * @return K0 = [K0[0], K0[1]] - коэффициенты обратной связи шага 0
 *         Управление: u[0] = -K0[0]*q - K0[1]*dq
 */
static std::array<double,2> finite_horizon_lqr_gain(double dt, int N,
                                                    double wq, double wdq, double wu,
                                                    double wqN, double wdqN) {
    // Матрица системы: x[k+1] = A*x[k] + B*u[k]
    // x = [eq, edq]^T
    Mat2 A{1.0, dt, 0.0, 1.0};
    std::array<double,2> B{0.5*dt*dt, dt};

    // Матрица стоимости промежуточных шагов
    Mat2 Q{wq, 0.0, 0.0, wdq};

    // Матрица стоимости терминального шага
    Mat2 Pnext{wqN, 0.0, 0.0, wdqN};

    std::array<double,2> K0{0.0, 0.0};

    // Обратная Риккати рекурсия: от шага N-1 к 0
    for (int i = N - 1; i >= 0; --i) {
        // Вычисляем S = R + B'*P*B
        double BtPB =
            B[0]*(Pnext.a00*B[0] + Pnext.a01*B[1]) +
            B[1]*(Pnext.a10*B[0] + Pnext.a11*B[1]);
        double S = wu + BtPB;
        if (S < 1e-12) S = 1e-12;

        // Вычисляем K[i] = (R + B'*P*B)^{-1} * B' * P * A = S^{-1} * B'*P*A
        Mat2 PA = mat2_mul(Pnext, A);
        std::array<double,2> BtPA = {
            B[0]*PA.a00 + B[1]*PA.a10,
            B[0]*PA.a01 + B[1]*PA.a11
        };
        std::array<double,2> K = {BtPA[0]/S, BtPA[1]/S};
        if (i == 0) K0 = K;

        // Вычисляем P[i] = Q + A'*P*A - A'*P*B*S^{-1}*B'*P*A
        Mat2 At = mat2_T(A);
        Mat2 AtPA = mat2_mul(At, PA);
        std::array<double,2> PB = {
            Pnext.a00*B[0] + Pnext.a01*B[1],
            Pnext.a10*B[0] + Pnext.a11*B[1]
        };
        std::array<double,2> AtPB = {
            At.a00*PB[0] + At.a01*PB[1],
            At.a10*PB[0] + At.a11*PB[1]
        };

        // Матрица Риккати обновляется как: P := Q + A'*P*A - (A'*P*B) * S^{-1} * (B'*P*A)
        Mat2 term2 = mat2_outer2(AtPB, BtPA);
        term2.a00 /= S; term2.a01 /= S; term2.a10 /= S; term2.a11 /= S;

        Mat2 P = mat2_add(Q, mat2_sub(AtPA, term2));
        // Симметризация
        P.a01 = 0.5*(P.a01 + P.a10);
        P.a10 = P.a01;
        Pnext = P;
    }

    return K0;
}

// ============================================================
// Ядро управления используется /arm/step
// ============================================================

/**
 * @struct ControlResult
 * @brief Результат вычисления одного шага управления
 *
 * Содержит желаемые команды управления (позиция, скорость, ускорение, момент)
 * и информацию об ошибке отслеживания траектории.
 */
struct ControlResult {
    std::vector<double> q_cmd, dq_cmd, ddq_cmd, tau_cmd;
    double eq_rms{0.0};   ///< RMS ошибка позиции: sqrt(mean((q - q_ref)^2))
    double edq_rms{0.0};  ///< RMS ошибка скорости: sqrt(mean((dq - dq_ref)^2))
    RefSample ref;        ///< Образец опорной траектории в текущий момент времени
};

/**
 * @brief Вычисляет команды управления на один шаг
 *
 * Основной алгоритм управления. На основе текущего состояния (q, dq) и опорной
 * траектории выбирает одно из трех управлений:
 *
 * 1) PD управление:
 *    ddq_des = ddq_ref - kp*(q - q_ref) - kd*(dq - dq_ref)
 *    где kp = wq, kd = wdq
 *
 * 2) LQR управление (ядро этапа 1):
 *    Конечногоризонтная рекурсия Риккати даёт коэффициенты обратной связи K[0]
 *    для локальной модели ошибки (двойной интегратор по каждому суставу):
 *    ddq_des = ddq_ref - K[0][0]*(q - q_ref) - K[0][1]*(dq - dq_ref)
 *
 * Затем используется inverse dynamics для вычисления моментов на основе ddq_des.
 * Система интегрируется вперед на один шаг dt, и возвращаются команды.
 *
 * @param q_use Текущие позиции суставов [6]
 * @param dq_use Текущие скорости суставов [6]
 * @param traj_copy Копия опорной траектории (вектор RefPoint)
 * @param dt_ref Шаг дискретизации траектории
 * @param t Текущий момент времени
 * @param dt Шаг интегрирования динамики
 * @param mode Режим управления: "pd" (сравнение) или "lqr" / "mpc_lite"
 *             (оба режима соответствуют конечногоризонтному LQR)
 * @param horizonN Горизонт LQR (число шагов обратной рекурсии Риккати)
 * @param wq Вес на позиции
 * @param wdq Вес на скорости
 * @param wu Вес на управлении
 * @param wqN Терминальный вес на позиции
 * @param wdqN Терминальный вес на скорости
 * @param u_max Максимальное ускорение (для ограничения)
 * @param dyn Объект динамики робота (используется для inverse dynamics и интеграции)
 * @return ControlResult с командами управления
 */
static ControlResult compute_control_step(const std::vector<double>& q_use,
                                          const std::vector<double>& dq_use,
                                          const std::vector<RefPoint>& traj_copy,
                                          double dt_ref,
                                          double t,
                                          double dt,
                                          const std::string& mode,
                                          int horizonN,
                                          double wq,
                                          double wdq,
                                          double wu,
                                          double wqN,
                                          double wdqN,
                                          double u_max,
                                          ArmDynamics& dyn) {
    ControlResult res;

    // Дискретизируем опорную траекторию в текущий момент времени
    res.ref = sample_ref_from_traj(traj_copy, t, dt_ref);

    res.ddq_cmd.assign(6, 0.0);
    res.tau_cmd.assign(6, 0.0);
    res.dq_cmd.assign(6, 0.0);
    res.q_cmd.assign(6, 0.0);

    if (mode == "pd") {
        // PD управление
        const double kp = wq;
        const double kd = wdq;
        for (int i = 0; i < 6; ++i) {
            double eq = q_use[i] - res.ref.q[i];
            double edq = dq_use[i] - res.ref.dq[i];
            double ddq_des = res.ref.ddq[i] - kp * eq - kd * edq;
            ddq_des = std::clamp(ddq_des, -u_max, u_max);
            res.ddq_cmd[i] = ddq_des;
        }
    } else {
        // LQR управление: вычисляем коэффициент обратной связи K[0]
        int Nuse = (mode == "mpc_lite") ? horizonN : 20;
        auto K = finite_horizon_lqr_gain(dt, Nuse, wq, wdq, wu, wqN, wdqN);
        for (int i = 0; i < 6; ++i) {
            double eq = q_use[i] - res.ref.q[i];
            double edq = dq_use[i] - res.ref.dq[i];
            double ddq_des = res.ref.ddq[i] - (K[0] * eq + K[1] * edq);
            ddq_des = std::clamp(ddq_des, -u_max, u_max);
            res.ddq_cmd[i] = ddq_des;
        }
    }

    // Вычисляем моменты с помощью обратной динамики
    dyn.setState(q_use, dq_use);
    res.tau_cmd = dyn.inverseDynamics(q_use, dq_use, res.ddq_cmd);

    // Интегрируем динамику вперед на один шаг
    dyn.stepWithTorque(res.tau_cmd, dt);

    // Получаем результирующее состояние после интеграции
    const auto& st = dyn.state();
    res.q_cmd = st.q;
    res.dq_cmd = st.dq;

    // Вычисляем RMS ошибки отслеживания
    res.eq_rms = rms_between(q_use, res.ref.q);
    res.edq_rms = rms_between(dq_use, res.ref.dq);
    return res;
}

// ============================================================
// Реализация ArmController
// ============================================================

/**
 * @brief Конструктор ArmController
 *
 * Инициализирует 6-DOF динамику робота и устанавливает масштабы параметров.
 */
ArmController::ArmController()
    : dyn_(6) {
    dyn_.resetParamScales();
    dyn_.setState({0,0,0,0,0,0}, {0,0,0,0,0,0});
}

/**
 * @brief HTTP обработчик для планирования опорной траектории minimum-jerk
 *
 * Реализует замкнутую формулу квинтика (полинома 5-й степени) для задачи
 * minimum-jerk с фиксированными граничными условиями. Это НЕ принцип
 * максимума Понтрягина.
 *
 * Эндпоинт: POST /arm/plan_minjerk_q
 *
 * JSON запрос:
 * {
 *   "q_target": [q0, q1, q2, q3, q4, q5],  // целевая конфигурация (обязательно)
 *   "q_start": [q0, q1, q2, q3, q4, q5],   // начальная конфигурация (опционально, по умолчанию текущее состояние)
 *   "T": 1.5,                               // время траектории (опционально, по умолчанию 1.0 сек)
 *   "dt": 0.02                              // шаг дискретизации (опционально, по умолчанию 0.02 сек)
 * }
 *
 * JSON ответ:
 * {
 *   "dt": 0.02,
 *   "unit": "rad",
 *   "q_start_used": [6 элементов],
 *   "q_target_used": [6 элементов],
 *   "trajectory": [
 *     {"t": 0.0, "q": [6 элементов]},
 *     {"t": 0.02, "q": [6 элементов]},
 *     ...
 *   ]
 * }
 *
 * @param req HTTP запрос
 * @param callback Функция обратного вызова для отправки ответа
 */
void ArmController::handlePlanMinJerkQ(const HttpRequestPtr& req,
                                   std::function<void (const HttpResponsePtr&)>&& callback) {
    Json::Value root;
    auto json = parse_json_body(req, root);
    if (!json) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("Bad JSON body"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    std::vector<double> q_target;
    std::string err;
    if (!read_array_double(*json, "q_target", q_target, err, 6)) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    std::vector<double> q_target6 = {q_target[0],q_target[1],q_target[2],q_target[3],q_target[4],q_target[5]};

    std::vector<double> q_start6;
    bool have_q_start = false;
    err.clear();
    if (read_q6_optional(*json, "q_start", q_start6, err)) {
        have_q_start = true;
        dyn_.setState(q_start6, {0,0,0,0,0,0});
    } else if (json->isMember("q_start")) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    double T  = json->isMember("T")  ? (*json)["T"].asDouble()  : 1.0;
    double dt = json->isMember("dt") ? (*json)["dt"].asDouble() : 0.02;
    if (T <= 0.0) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("T must be > 0"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    if (dt <= 0.0) dt = 0.02;

    std::vector<double> q0_6(6, 0.0);
    if (have_q_start) q0_6 = q_start6;
    else {
        const auto& st = dyn_.state();
        for (int i = 0; i < 6; ++i) q0_6[i] = st.q[i];
    }

    auto ref_traj = plan_minjerk_traj(q0_6, q_target6, T, dt);
    dyn_.setState(q_target6, {0,0,0,0,0,0});

    Json::Value out(Json::objectValue);
    out["dt"] = dt;
    out["unit"] = "rad";
    out["q_start_used"] = to_q6_json(q0_6);
    out["q_target_used"] = to_q6_json(q_target6);
    out["trajectory"] = Json::arrayValue;
    for (const auto& p : ref_traj) {
        Json::Value item(Json::objectValue);
        item["t"] = p.t;
        item["q"] = to_q6_json(p.q);
        out["trajectory"].append(item);
    }

    callback(HttpResponse::newHttpJsonResponse(out));
}

/**
 * @brief HTTP обработчик для установки опорной траектории
 *
 * Эндпоинт POST /arm/set_reference
 *
 * Планирует minimum-jerk траекторию (квинтический полином) и сохраняет её
 * как опорную для последующих вызовов /arm/step. Также сбрасывает фильтры
 * Калмана.
 *
 * JSON запрос:
 * {
 *   "q_target": [q0, q1, q2, q3, q4, q5],  // целевая конфигурация (обязательно)
 *   "q_start": [q0, q1, q2, q3, q4, q5],   // начальная конфигурация (опционально, по умолчанию текущее состояние)
 *   "T": 1.5,                               // время траектории (опционально, по умолчанию 1.0 сек)
 *   "dt": 0.02                              // шаг дискретизации (опционально, по умолчанию 0.02 сек)
 * }
 *
 * JSON ответ:
 * {
 *   "ok": true,
 *   "dt": 0.02,
 *   "T": 1.5,
 *   "points": 75,  // количество точек в траектории
 *   "q_start_used": [6 элементов],
 *   "q_target_used": [6 элементов]
 * }
 *
 * @param req HTTP запрос
 * @param callback Функция обратного вызова для отправки ответа
 */
void ArmController::handleSetReference(const HttpRequestPtr& req,
                                       std::function<void (const HttpResponsePtr&)>&& callback) {
    Json::Value root;
    auto json = parse_json_body(req, root);
    if (!json) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("Bad JSON body"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    std::string err;
    std::vector<double> q_target;
    if (!read_array_double(*json, "q_target", q_target, err, 6)) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    std::vector<double> q_target6 = {q_target[0],q_target[1],q_target[2],q_target[3],q_target[4],q_target[5]};

    std::vector<double> q_start6;
    bool have_q_start = false;
    err.clear();
    if (read_q6_optional(*json, "q_start", q_start6, err)) {
        have_q_start = true;
    } else if (json->isMember("q_start")) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    double T  = json->isMember("T")  ? (*json)["T"].asDouble()  : 1.0;
    double dt = json->isMember("dt") ? (*json)["dt"].asDouble() : 0.02;
    if (T <= 0.0) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("T must be > 0"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    if (dt <= 0.0) dt = 0.02;

    std::vector<double> q0_6(6, 0.0);
    if (have_q_start) q0_6 = q_start6;
    else {
        const auto& st = dyn_.state();
        for (int i = 0; i < 6; ++i) q0_6[i] = st.q[i];
    }

    std::vector<RefPoint> traj;
    try {
        traj = plan_minjerk_traj(q0_6, q_target6, T, dt);
    } catch (const std::exception& e) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(std::string("Planner error: ") + e.what()));
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        dyn_.resetParamScales();
        ref_traj_ = std::move(traj);
        ref_dt_ = dt;
        ref_T_ = T;
        have_last_q_ = false;
        for (auto& f : kf_) f = KF2{};
    }

    Json::Value out(Json::objectValue);
    out["ok"] = true;
    out["dt"] = dt;
    out["T"] = T;
    out["points"] = (int)ref_traj_.size();
    out["q_start_used"] = to_q6_json(q0_6);
    out["q_target_used"] = to_q6_json(q_target6);

    // Логируем установку опорной траектории — пригодится для отладки шагов.
    std::cout << std::fixed << std::setprecision(4)
              << "\n[set_reference] T=" << T << "s  dt=" << dt
              << "s  points=" << ref_traj_.size() << "\n"
              << "  q_start  = ["
              << q0_6[0] << ", " << q0_6[1] << ", " << q0_6[2] << ", "
              << q0_6[3] << ", " << q0_6[4] << ", " << q0_6[5] << "]\n"
              << "  q_target = ["
              << q_target6[0] << ", " << q_target6[1] << ", " << q_target6[2] << ", "
              << q_target6[3] << ", " << q_target6[4] << ", " << q_target6[5] << "]"
              << std::endl;

    callback(HttpResponse::newHttpJsonResponse(out));
}

/**
 * @brief HTTP обработчик для шага управления (PD/LQR/MPC)
 *
 * Эндпоинт POST /arm/step
 *
 * Требует предварительного вызова /arm/set_reference для установки опорной траектории.
 * На основе измеренного состояния (q, dq) вычисляет команды управления.
 *
 * Поддерживает опциональный фильтр Калмана для оценки состояния из шумных измерений позиции.
 *
 * JSON запрос:
 * {
 *   "q": [q0, q1, q2, q3, q4, q5],          // измеренные позиции (обязательно)
 *   "dq": [dq0, dq1, dq2, dq3, dq4, dq5],   // измеренные скорости (опционально)
 *   "t": 1.5,                                // текущий момент времени (опционально, по умолчанию 0.0)
 *   "dt": 0.02,                              // шаг времени (опционально, используется если dq отсутствует)
 *   "mode": "lqr",                           // режим управления (опционально, по умолчанию "lqr")
 *   "N": 20,                                 // горизонт предсказания (опционально, по умолчанию 20)
 *   "u_max": 8.0,                            // максимальное ускорение (опционально, по умолчанию 8.0)
 *   "use_kalman": false,                     // использовать фильтр Калмана (опционально)
 *   "meas_var": 1e-4,                        // дисперсия измерения (опционально)
 *   "proc_var_q": 1e-6,                      // дисперсия процесса для q (опционально)
 *   "proc_var_dq": 1e-4,                     // дисперсия процесса для dq (опционально)
 *   "weights": {
 *     "wq": 30.0,                            // вес на позиции
 *     "wdq": 2.0,                            // вес на скорости
 *     "wu": 0.1,                             // вес на управлении
 *     "wqN": 30.0,                           // терминальный вес на позиции
 *     "wdqN": 2.0                            // терминальный вес на скорости
 *   }
 * }
 *
 * JSON ответ:
 * {
 *   "ok": true,
 *   "t": 1.5,
 *   "dt": 0.02,
 *   "mode": "lqr",
 *   "q_cmd": [6 элементов],                 // желаемые позиции
 *   "dq_cmd": [6 элементов],                // желаемые скорости
 *   "ddq_cmd": [6 элементов],               // желаемые ускорения
 *   "tau_cmd": [6 элементов],               // команды моментов
 *   "q_ref": [6 элементов],                 // опорные позиции
 *   "dq_ref": [6 элементов],                // опорные скорости
 *   "ddq_ref": [6 элементов],               // опорные ускорения
 *   "weights_used": {...},                  // использованные веса
 *   "debug": {
 *     "eq_rms": 0.05,
 *     "edq_rms": 0.02,
 *     "use_kalman": false
 *   }
 * }
 *
 * @param req HTTP запрос
 * @param callback Функция обратного вызова для отправки ответа
 */
void ArmController::handleStep(const HttpRequestPtr& req,
                               std::function<void (const HttpResponsePtr&)>&& callback) {
    Json::Value root;
    auto json = parse_json_body(req, root);
    if (!json) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("Bad JSON body"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    std::vector<RefPoint> traj_copy;
    double dt_ref = 0.02;
    double T_ref = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (ref_traj_.empty()) {
            auto resp = HttpResponse::newHttpJsonResponse(make_error("No reference. Call /arm/set_reference first."));
            resp->setStatusCode(k409Conflict);
            callback(resp);
            return;
        }
        traj_copy = ref_traj_;
        dt_ref = ref_dt_;
        T_ref = ref_T_;
    }

    std::string err;
    std::vector<double> q_meas;
    if (!read_array_double(*json, "q", q_meas, err, 6)) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    std::vector<double> q6 = {q_meas[0],q_meas[1],q_meas[2],q_meas[3],q_meas[4],q_meas[5]};

    std::vector<double> dq6(6, 0.0);
    bool have_dq = false;
    if (json->isMember("dq")) {
        std::vector<double> dq;
        if (!read_array_double(*json, "dq", dq, err, 6)) {
            auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }
        dq6 = {dq[0],dq[1],dq[2],dq[3],dq[4],dq[5]};
        have_dq = true;
    }

    double dt = json->isMember("dt") ? (*json)["dt"].asDouble() : dt_ref;
    if (dt <= 1e-6) dt = dt_ref;

    if (!have_dq) {
        // Если скорость не передана, вычисляем из разности конфигураций
        std::lock_guard<std::mutex> lk(mtx_);
        if (have_last_q_) {
            for (int i = 0; i < 6; ++i) dq6[i] = (q6[i] - last_q_meas_[i]) / dt;
        } else {
            std::fill(dq6.begin(), dq6.end(), 0.0);
        }
        last_q_meas_ = q6;
        have_last_q_ = true;
    } else {
        std::lock_guard<std::mutex> lk(mtx_);
        last_q_meas_ = q6;
        have_last_q_ = true;
    }

    double t = json->isMember("t") ? (*json)["t"].asDouble() : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > T_ref) t = T_ref;

    // Веса по умолчанию
    double wq  = 30.0;
    double wdq = 2.0;
    double wu  = 0.1;
    double wqN = wq;
    double wdqN = wdq;
    if (json->isMember("weights") && (*json)["weights"].isObject()) {
        const auto& w = (*json)["weights"];
        if (w.isMember("wq"))   wq   = w["wq"].asDouble();
        if (w.isMember("wdq"))  wdq  = w["wdq"].asDouble();
        if (w.isMember("wu"))   wu   = w["wu"].asDouble();
        if (w.isMember("wqN"))  wqN  = w["wqN"].asDouble();
        if (w.isMember("wdqN")) wdqN = w["wdqN"].asDouble();
    }

    double u_max = json->isMember("u_max") ? (*json)["u_max"].asDouble() : 8.0;
    if (u_max <= 0) u_max = 8.0;

    std::string mode = json->isMember("mode") ? (*json)["mode"].asString() : "lqr";
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    int horizonN = json->isMember("N") ? (*json)["N"].asInt() : 20;
    if (horizonN < 1) horizonN = 1;
    if (horizonN > 200) horizonN = 200;

    bool use_kalman = json->isMember("use_kalman") ? (*json)["use_kalman"].asBool() : false;
    double meas_var = json->isMember("meas_var") ? (*json)["meas_var"].asDouble() : 1e-4;
    double proc_var_q  = json->isMember("proc_var_q")  ? (*json)["proc_var_q"].asDouble()  : 1e-6;
    double proc_var_dq = json->isMember("proc_var_dq") ? (*json)["proc_var_dq"].asDouble() : 1e-4;

    std::vector<double> q_use = q6;
    std::vector<double> dq_use = dq6;

    if (use_kalman) {
        // Фильтр Калмана для оценки состояния (q, dq) из шумных измерений q
        // Динамика: x[k+1] = A*x[k] + w[k], x = [q, dq]^T
        // Измерение: z[k] = [1 0] * x[k] + v[k]
        Mat2 A{1.0, dt, 0.0, 1.0};
        Mat2 Qw{proc_var_q, 0.0, 0.0, proc_var_dq};
        const double Rv = std::max(1e-12, meas_var);

        std::lock_guard<std::mutex> lk(mtx_);
        for (int i = 0; i < 6; ++i) {
            auto& f = kf_[i];
            if (!f.initialized) {
                // Инициализируем фильтр при первом вызове
                f.x0 = q6[i];
                f.x1 = dq6[i];
                f.P00 = 1.0; f.P01 = 0.0; f.P10 = 0.0; f.P11 = 1.0;
                f.initialized = true;
            }

            // Прогноз: x[k|k-1] = A * x[k-1|k-1]
            double x0m = A.a00*f.x0 + A.a01*f.x1;
            double x1m = A.a10*f.x0 + A.a11*f.x1;

            // Коварианция прогноза: P[k|k-1] = A * P[k-1|k-1] * A^T + Q
            Mat2 P{f.P00, f.P01, f.P10, f.P11};
            Mat2 AP = mat2_mul(A, P);
            Mat2 APAt = mat2_mul(AP, mat2_T(A));
            Mat2 Pm = mat2_add(APAt, Qw);

            // Инновация: y = z[k] - C * x[k|k-1], где C = [1 0]
            double y = q6[i];
            double S = Pm.a00 + Rv;  // S = C * P * C^T + R
            if (S < 1e-12) S = 1e-12;

            // Коэффициент Калмана: K = P * C^T * S^{-1}
            double K0 = Pm.a00 / S;
            double K1 = Pm.a10 / S;

            // Корректировка: x[k|k] = x[k|k-1] + K * y
            double innov = y - x0m;
            double x0p = x0m + K0 * innov;
            double x1p = x1m + K1 * innov;

            // Коварианция корректировки: P[k|k] = (I - K*C) * P[k|k-1]
            Mat2 I_KC{1.0-K0, 0.0, -K1, 1.0};
            Mat2 Pp = mat2_mul(I_KC, Pm);

            f.x0 = x0p; f.x1 = x1p;
            f.P00 = Pp.a00; f.P01 = Pp.a01; f.P10 = Pp.a10; f.P11 = Pp.a11;
            q_use[i] = f.x0;
            dq_use[i] = f.x1;
        }
    }

    ControlResult ctrl = compute_control_step(
        q_use, dq_use, traj_copy, dt_ref, t, dt, mode, horizonN,
        wq, wdq, wu, wqN, wdqN, u_max, dyn_
    );

    Json::Value out(Json::objectValue);
    out["ok"] = true;
    out["t"] = t;
    out["dt"] = dt;
    out["mode"] = mode;
    out["q_cmd"] = to_q6_json(ctrl.q_cmd);
    out["dq_cmd"] = to_q6_json(ctrl.dq_cmd);
    out["ddq_cmd"] = to_q6_json(ctrl.ddq_cmd);
    out["tau_cmd"] = to_q6_json(ctrl.tau_cmd);
    out["q_ref"] = to_q6_json(ctrl.ref.q);
    out["dq_ref"] = to_q6_json(ctrl.ref.dq);
    out["ddq_ref"] = to_q6_json(ctrl.ref.ddq);

    Json::Value weights(Json::objectValue);
    weights["wq"] = wq; weights["wdq"] = wdq; weights["wu"] = wu;
    weights["wqN"] = wqN; weights["wdqN"] = wdqN;
    out["weights_used"] = weights;

    Json::Value dbg(Json::objectValue);
    dbg["eq_rms"] = ctrl.eq_rms;
    dbg["edq_rms"] = ctrl.edq_rms;
    dbg["use_kalman"] = use_kalman;
    out["debug"] = dbg;

    // Компактный лог шага: время, ошибки, пиковая ошибка по суставам, RMS момента.
    double max_abs_eq = 0.0;
    double sum_tau_sq = 0.0;
    for (int i = 0; i < 6; ++i) {
        const double abs_eq = std::abs(q_use[i] - ctrl.ref.q[i]);
        if (abs_eq > max_abs_eq) max_abs_eq = abs_eq;
        sum_tau_sq += ctrl.tau_cmd[i] * ctrl.tau_cmd[i];
    }
    const double tau_rms = std::sqrt(sum_tau_sq / 6.0);
    std::cout << std::fixed << std::setprecision(4)
              << "[step] t=" << t
              << "s  eq_rms=" << ctrl.eq_rms
              << "  edq_rms=" << ctrl.edq_rms
              << "  max|eq|=" << max_abs_eq
              << "  |tau|_rms=" << std::setprecision(2) << tau_rms
              << "  mode=" << mode
              << std::endl;

    callback(HttpResponse::newHttpJsonResponse(out));
}
