/**
 * @file ArmController.cc
 * @brief REST контроллер для UR5e робота-манипулятора с поддержкой RL
 *
 * Этот модуль реализует HTTP контроллер для управления робошумом UR5e на основе
 * Drogon фреймворка. Включает следующую функциональность:
 * - Планирование траектории с минимальным рывком (PMP)
 * - LQR (линейно-квадратичное управление) с конечным горизонтом
 * - Управление по PD (пропорционально-дифференциальное)
 * - MPC-lite управление
 * - Фильтр Калмана для оценки состояния
 * - RL (обучение с подкреплением) интерфейс с использованием PPO
 *
 * Математическая основа:
 * - Динамика: x[k+1] = Ax[k] + Bu[k]
 * - Риккати рекурсия: P[k] = Q + A'PA - A'PB(R+B'PB)^{-1}B'PA
 * - Управление: u[k] = -K[k]x[k], где K[k] = (R+B'PB)^{-1}B'PA
 *
 * @see ArmDynamics для динамики робота
 * @see PMPPoint для траекторий минимального рывка
 */

#include "ArmController.h"

#include <drogon/HttpAppFramework.h>
#include <json/json.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
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
// Дискретизация траектории из сохраненной PMP траектории
// ============================================================

/**
 * @struct RefSample
 * @brief Образец отсчета опорной траектории в момент времени t
 *
 * Содержит желаемые позицию (q), скорость (dq) и ускорение (ddq)
 * из опорной траектории, полученные интерполяцией двух соседних точек PMP.
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
static RefSample lerp_ref(const PMPPoint& a, const PMPPoint& b, double t) {
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
 * Производит дискретизацию сохраненной PMP траектории (набор точек) в точку
 * времени t с шагом dt_ref. Использует линейную интерполяцию между точками.
 * Пустая траектория возвращает нулевой образец.
 *
 * @param traj_copy Копия сохраненной траектории (вектор PMPPoint)
 * @param t Желаемый момент времени
 * @param dt_ref Шаг дискретизации исходной траектории (для индексирования)
 * @return RefSample с q, dq, ddq в момент t
 */
static RefSample sample_ref_from_traj(const std::vector<PMPPoint>& traj_copy, double t, double dt_ref) {
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

/**
 * @brief Собирает 25-мерный вектор наблюдений для PPO агента
 *
 * Построение вектора наблюдений для обучения с подкреплением:
 * [q[0:6], dq[0:6], q_error[0:6], dq_error[0:6], t_frac]
 * где:
 * - q = позиции суставов (6)
 * - dq = скорости суставов (6)
 * - q_error = q - q_ref (позиционная ошибка, 6)
 * - dq_error = dq - dq_ref (скоростная ошибка, 6)
 * - t_frac = t / T (нормализованное время, 1)
 * Итого: 6 + 6 + 6 + 6 + 1 = 25 измерений
 *
 * @param q Текущие позиции суставов
 * @param dq Текущие скорости суставов
 * @param ref Опорный образец траектории
 * @param t Текущий момент времени
 * @param T Полное время траектории
 * @return Вектор размером 25 с наблюдениями
 */
static std::vector<double> build_obs(const std::vector<double>& q,
                                     const std::vector<double>& dq,
                                     const RefSample& ref,
                                     double t,
                                     double T) {
    std::vector<double> obs;
    obs.reserve(25);
    // q[0:6]
    for (int i = 0; i < 6; ++i) obs.push_back(i < (int)q.size() ? q[i] : 0.0);
    // dq[0:6]
    for (int i = 0; i < 6; ++i) obs.push_back(i < (int)dq.size() ? dq[i] : 0.0);
    // q_error[0:6]
    for (int i = 0; i < 6; ++i) obs.push_back((i < (int)q.size() ? q[i] : 0.0) - ref.q[i]);
    // dq_error[0:6]
    for (int i = 0; i < 6; ++i) obs.push_back((i < (int)dq.size() ? dq[i] : 0.0) - ref.dq[i]);
    // t_frac
    obs.push_back(T > 1e-9 ? std::clamp(t / T, 0.0, 1.0) : 0.0);
    return obs;
}

/**
 * @brief Преобразует действия PPO в веса LQR
 *
 * Преобразует 5-мерный вектор действий агента (в диапазоне [-1, 1])
 * в веса целевой функции LQR (неотрицательные, в физических диапазонах).
 * Использует линейную интерполяцию (lerp) для отображения.
 *
 * Отображение действие -> вес:
 * - action[0] -> wq   (0 имеет вес на позиции): [5.0, 80.0]
 * - action[1] -> wdq  (вес на скорости):           [0.2, 10.0]
 * - action[2] -> wu   (вес на управлении):         [0.01, 1.0]
 * - action[3] -> wqN  (терминальный вес позиции): [5.0, 120.0]
 * - action[4] -> wdqN (терминальный вес скорости):[0.2, 12.0]
 *
 * @param action Вектор действий из PPO агента, размер 5, диапазон [-1, 1]
 * @return Вектор весов [wq, wdq, wu, wqN, wdqN], размер 5
 */
static std::vector<double> map_action_to_weights(const std::vector<double>& action) {
    // Expected action in [-1,1]^5 from PPO.
    // Output weights: [wq, wdq, wu, wqN, wdqN]
    auto lerp = [](double a, double lo, double hi) {
        double alpha = 0.5 * (std::clamp(a, -1.0, 1.0) + 1.0);
        return lo + alpha * (hi - lo);
    };

    std::vector<double> w(5, 0.0);
    w[0] = lerp(action.size() > 0 ? action[0] : 0.0, 5.0, 80.0);   // wq
    w[1] = lerp(action.size() > 1 ? action[1] : 0.0, 0.2, 10.0);   // wdq
    w[2] = lerp(action.size() > 2 ? action[2] : 0.0, 0.01, 1.0);   // wu
    w[3] = lerp(action.size() > 3 ? action[3] : 0.0, 5.0, 120.0);  // wqN
    w[4] = lerp(action.size() > 4 ? action[4] : 0.0, 0.2, 12.0);   // wdqN
    return w;
}

// ============================================================
// LQR / MPC-lite коэффициенты для локальной модели двойного интегратора
// (управление предсказывает желаемое ddq, но растение - это ArmDynamics)
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
    // x = [q, dq]^T
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
// Ядро управления используется /arm/step и /rl/step
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
 * 2) MPC-lite / LQR управление:
 *    Использует конечный горизонт Риккати для вычисления коэффициентов K[0]
 *    ddq_des = ddq_ref - K[0][0]*(q - q_ref) - K[0][1]*(dq - dq_ref)
 *
 * Затем используется inverse dynamics для вычисления моментов на основе ddq_des.
 * Система интегрируется вперед на один шаг dt, и возвращаются команды.
 *
 * @param q_use Текущие позиции суставов [6]
 * @param dq_use Текущие скорости суставов [6]
 * @param traj_copy Копия опорной траектории (вектор PMPPoint)
 * @param dt_ref Шаг дискретизации траектории
 * @param t Текущий момент времени
 * @param dt Шаг интегрирования динамики
 * @param mode Режим управления: "pd", "mpc_lite", "lqr"
 * @param horizonN Горизонт предсказания (для MPC-lite/LQR)
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
                                          const std::vector<PMPPoint>& traj_copy,
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
 * @brief HTTP обработчик для планирования траектории с минимальным рывком (PMP)
 *
 * Эндпоинт POST /arm/plan_pmp_q
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
void ArmController::handlePlanPMP_Q(const HttpRequestPtr& req,
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

    auto pmp_traj = plan_pmp_minimum_jerk(q0_6, q_target6, T, dt);
    dyn_.setState(q_target6, {0,0,0,0,0,0});

    Json::Value out(Json::objectValue);
    out["dt"] = dt;
    out["unit"] = "rad";
    out["q_start_used"] = to_q6_json(q0_6);
    out["q_target_used"] = to_q6_json(q_target6);
    out["trajectory"] = Json::arrayValue;
    for (const auto& p : pmp_traj) {
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
 * Запланировать и установить PMP траекторию как опорную для последующих вызовов /arm/step.
 * Также сбрасывает фильтры Калмана и состояние RL.
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

    std::vector<PMPPoint> traj;
    try {
        traj = plan_pmp_minimum_jerk(q0_6, q_target6, T, dt);
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

    std::vector<PMPPoint> traj_copy;
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
    callback(HttpResponse::newHttpJsonResponse(out));
}

/**
 * @brief HTTP обработчик для инициализации RL эпизода
 *
 * Эндпоинт POST /rl/reset
 *
 * Инициализирует новый эпизод обучения с подкреплением. Планирует траекторию,
 * сбрасывает состояние робота, инициализирует счетчик шагов и возвращает
 * начальное наблюдение.
 *
 * JSON запрос:
 * {
 *   "q_target": [q0, q1, q2, q3, q4, q5],      // целевая конфигурация (обязательно)
 *   "q_start": [q0, q1, q2, q3, q4, q5],       // начальная конфигурация (опционально)
 *   "T": 1.5,                                   // время эпизода (опционально, по умолчанию 1.5 сек)
 *   "dt": 0.02,                                 // шаг времени (опционально, по умолчанию 0.02 сек)
 *   "mode": "mpc_lite",                         // режим управления (опционально)
 *   "N": 20,                                    // горизонт предсказания (опционально)
 *   "u_max": 8.0,                               // максимальное ускорение (опционально)
 *   "max_steps": 100,                           // максимально шагов до truncation (опционально)
 *   "success_tol": 0.03,                        // допуск для успешного завершения (опционально)
 *   "friction_scale": 1.0,                      // масштабирование трения (опционально)
 *   "inertia_scale": 1.0,                       // масштабирование инерции (опционально)
 *   "rw_pos": 2.0,                              // награда за позицию (опционально)
 *   "rw_vel": 0.15,                             // награда за скорость (опционально)
 *   "rw_u": 0.01,                               // награда за управление (опционально)
 *   "rw_du": 0.005,                             // награда за дельта управления (опционально)
 *   "rw_success": 5.0                           // награда за успех (опционально)
 * }
 *
 * JSON ответ:
 * {
 *   "ok": true,
 *   "obs": [25 элементов],                      // начальное наблюдение
 *   "obs_dim": 25,                              // размерность наблюдения
 *   "act_dim": 5,                               // размерность действия
 *   "q_start": [6 элементов],
 *   "q_target": [6 элементов],
 *   "mode": "mpc_lite",
 *   "dt": 0.02,
 *   "T": 1.5,
 *   "max_steps": 100,
 *   "success_tol": 0.03,
 *   "friction_scale": 1.0,
 *   "inertia_scale": 1.0
 * }
 *
 * @param req HTTP запрос
 * @param callback Функция обратного вызова для отправки ответа
 */
void ArmController::handleRLReset(const HttpRequestPtr& req,
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

    std::vector<double> q_start6(6, 0.0);
    if (read_q6_optional(*json, "q_start", q_start6, err)) {
        // ok
    } else if (json->isMember("q_start")) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    } else {
        const auto& st = dyn_.state();
        for (int i = 0; i < 6; ++i) q_start6[i] = st.q[i];
    }

    double T  = json->isMember("T")  ? (*json)["T"].asDouble()  : 1.5;
    double dt = json->isMember("dt") ? (*json)["dt"].asDouble() : 0.02;
    if (T <= 0.0) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("T must be > 0"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    if (dt <= 1e-6) dt = 0.02;

    std::vector<PMPPoint> traj;
    try {
        traj = plan_pmp_minimum_jerk(q_start6, q_target6, T, dt);
    } catch (const std::exception& e) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error(std::string("Planner error: ") + e.what()));
        resp->setStatusCode(k500InternalServerError);
        callback(resp);
        return;
    }

    std::string mode = json->isMember("mode") ? (*json)["mode"].asString() : "mpc_lite";
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
    int horizonN = json->isMember("N") ? (*json)["N"].asInt() : 20;
    horizonN = std::clamp(horizonN, 1, 200);
    double u_max = json->isMember("u_max") ? (*json)["u_max"].asDouble() : 8.0;
    if (u_max <= 0.0) u_max = 8.0;
    int max_steps = json->isMember("max_steps") ? (*json)["max_steps"].asInt() : (int)std::ceil(T / dt) + 10;
    if (max_steps < 1) max_steps = 1;
    double success_tol = json->isMember("success_tol") ? (*json)["success_tol"].asDouble() : 0.03;
    if (success_tol <= 0.0) success_tol = 0.03;
    double friction_scale = json->isMember("friction_scale") ? (*json)["friction_scale"].asDouble() : 1.0;
    double inertia_scale = json->isMember("inertia_scale") ? (*json)["inertia_scale"].asDouble() : 1.0;
    friction_scale = std::clamp(friction_scale, 0.2, 3.0);
    inertia_scale = std::clamp(inertia_scale, 0.2, 3.0);

    double rw_pos = json->isMember("rw_pos") ? (*json)["rw_pos"].asDouble() : 2.0;
    double rw_vel = json->isMember("rw_vel") ? (*json)["rw_vel"].asDouble() : 0.15;
    double rw_u = json->isMember("rw_u") ? (*json)["rw_u"].asDouble() : 0.01;
    double rw_du = json->isMember("rw_du") ? (*json)["rw_du"].asDouble() : 0.005;
    double rw_success = json->isMember("rw_success") ? (*json)["rw_success"].asDouble() : 5.0;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        ref_traj_ = std::move(traj);
        ref_dt_ = dt;
        ref_T_ = T;
        have_last_q_ = false;
        for (auto& f : kf_) f = KF2{};

        // Инициализируем состояние RL
        rl_active_ = true;
        rl_t_ = 0.0;
        rl_dt_ = dt;
        rl_T_ = T;
        rl_step_count_ = 0;
        rl_max_steps_ = max_steps;
        rl_success_tol_ = success_tol;
        rl_mode_ = mode;
        rl_horizonN_ = horizonN;
        rl_u_max_ = u_max;
        rl_last_tau_cmd_.assign(6, 0.0);
        rl_q_start_ = q_start6;
        rl_q_target_ = q_target6;
        rl_inertia_scale_ = inertia_scale;
        rl_friction_scale_ = friction_scale;
        dyn_.setParamScales(inertia_scale, friction_scale);
        rw_pos_ = rw_pos;
        rw_vel_ = rw_vel;
        rw_u_ = rw_u;
        rw_du_ = rw_du;
        rw_success_ = rw_success;
    }

    dyn_.setState(q_start6, std::vector<double>(6, 0.0));
    RefSample ref0 = sample_ref_from_traj(ref_traj_, 0.0, dt);
    std::vector<double> obs = build_obs(q_start6, std::vector<double>(6, 0.0), ref0, 0.0, T);

    // ============================================================
    // Сброс метрик эпизода и расчёт характеристик плана PMP
    // ============================================================
    ep_metrics_ = EpisodeMetrics{};
    ep_metrics_.ref_T = T;
    {
        // Характеристики плана PMP, вычисляются один раз при reset.
        // Не зависят от трекера — оценивают качество ТОЛЬКО планировщика.
        double ref_max_dq      = 0.0;
        double ref_max_ddq     = 0.0;
        double ref_jerk_energy = 0.0;
        double ref_path_length = 0.0;
        for (const auto& pt : ref_traj_) {
            for (int i = 0; i < 6; ++i) {
                if (i < (int)pt.dq.size()) {
                    const double v = std::abs(pt.dq[i]);
                    if (v > ref_max_dq) ref_max_dq = v;
                    ref_path_length += v * dt;
                }
                if (i < (int)pt.ddq.size()) {
                    const double a = std::abs(pt.ddq[i]);
                    if (a > ref_max_ddq) ref_max_ddq = a;
                }
                if (i < (int)pt.u.size()) {
                    // pt.u содержит jerk (минимизируемая величина в PMP minimum-jerk)
                    ref_jerk_energy += pt.u[i] * pt.u[i] * dt;
                }
            }
        }
        ep_metrics_.ref_max_dq      = ref_max_dq;
        ep_metrics_.ref_max_ddq     = ref_max_ddq;
        ep_metrics_.ref_jerk_energy = ref_jerk_energy;
        ep_metrics_.ref_path_length = ref_path_length;
    }

    // Открываем CSV-лог эпизода (имя по timestamp, уникальное на эпизод)
    if (csv_log_enabled_) {
        if (csv_log_.is_open()) csv_log_.close();
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const long long ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        const std::string fname = "episode_" + std::to_string(ms) + ".csv";
        csv_log_.open(fname);
        if (csv_log_.is_open()) {
            csv_log_ << "t,eq_rms,edq_rms,u_energy,J_step,reward,success";
            for (int i = 0; i < 6; ++i) csv_log_ << ",q_real"   << i;
            for (int i = 0; i < 6; ++i) csv_log_ << ",dq_real"  << i;
            for (int i = 0; i < 6; ++i) csv_log_ << ",q_ref"    << i;
            for (int i = 0; i < 6; ++i) csv_log_ << ",dq_ref"   << i;
            for (int i = 0; i < 6; ++i) csv_log_ << ",eq"       << i;
            for (int i = 0; i < 6; ++i) csv_log_ << ",tau"      << i;
            csv_log_ << "\n";
        }
    }

    Json::Value out(Json::objectValue);
    out["ok"] = true;
    out["obs"] = to_json_array(obs);
    out["obs_dim"] = (int)obs.size();
    out["act_dim"] = 5;
    out["q_start"] = to_q6_json(q_start6);
    out["q_target"] = to_q6_json(q_target6);
    out["mode"] = mode;
    out["dt"] = dt;
    out["T"] = T;
    out["max_steps"] = max_steps;
    out["success_tol"] = success_tol;
    out["friction_scale"] = friction_scale;
    out["inertia_scale"] = inertia_scale;
    callback(HttpResponse::newHttpJsonResponse(out));
}

/**
 * @brief HTTP обработчик для шага RL агента
 *
 * Эндпоинт POST /rl/step
 *
 * Выполняет один шаг эпизода обучения с подкреплением. Требует активного RL эпизода
 * (инициализированного /rl/reset).
 *
 * Агент может предоставить либо действие (5-мерный вектор из PPO), либо веса LQR.
 * Действие преобразуется в веса LQR с помощью функции map_action_to_weights.
 *
 * На основе текущего состояния вычисляются команды управления, робот интегрируется
 * вперед на один шаг, и вычисляется награда на основе позиционной ошибки,
 * скоростной ошибки, энергии управления и энергии изменений управления.
 *
 * JSON запрос:
 * {
 *   "action": [a0, a1, a2, a3, a4],       // действие PPO (опционально, если нет weights)
 *   "weights": {
 *     "wq": 30.0,                          // вес на позиции (опционально)
 *     "wdq": 2.0,                          // вес на скорости (опционально)
 *     "wu": 0.1,                           // вес на управлении (опционально)
 *     "wqN": 30.0,                         // терминальный вес (опционально)
 *     "wdqN": 2.0                          // терминальный вес (опционально)
 *   }
 * }
 *
 * JSON ответ:
 * {
 *   "ok": true,
 *   "obs": [25 элементов],                // наблюдение следующего состояния
 *   "reward": 3.5,                        // награда за этот шаг
 *   "done": false,                        // терминальный флаг
 *   "truncated": false,                   // флаг truncation (max_steps достигнут)
 *   "t": 0.04,                            // время текущего шага
 *   "step_count": 2,                      // счетчик шагов
 *   "q_cmd": [6 элементов],               // желаемые позиции
 *   "dq_cmd": [6 элементов],              // желаемые скорости
 *   "ddq_cmd": [6 элементов],             // желаемые ускорения
 *   "tau_cmd": [6 элементов],             // команды моментов
 *   "weights_used": {...},                // использованные веса
 *   "action_used": [5 элементов],         // если был использован action
 *   "info": {
 *     "eq_rms": 0.05,                     // ошибка позиции
 *     "edq_rms": 0.02,                    // ошибка скорости
 *     "u_energy": 1.5,                    // энергия управления
 *     "du_energy": 0.3,                   // энергия изменения управления
 *     "success": false,                   // достигнута ли целевая конфигурация
 *     "time_done": false,                 // истекло ли время эпизода
 *     "friction_scale": 1.0,
 *     "inertia_scale": 1.0
 *   }
 * }
 *
 * Функция вознаграждения:
 * reward = -rw_pos * (eq_rms_next)^2
 *        - rw_vel * (edq_rms_next)^2
 *        - rw_u * ||tau||^2_mean
 *        - rw_du * ||d(tau)/dt||^2_mean
 *        + rw_success * (если success)
 *
 * Условия завершения:
 * - done = success или time_done или truncated
 * - success = time_done && eq_rms <= success_tol
 * - time_done = t >= T
 * - truncated = step_count >= max_steps
 *
 * @param req HTTP запрос
 * @param callback Функция обратного вызова для отправки ответа
 */
void ArmController::handleRLStep(const HttpRequestPtr& req,
                                 std::function<void (const HttpResponsePtr&)>&& callback) {
    Json::Value root;
    auto json = parse_json_body(req, root);
    if (!json) {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("Bad JSON body"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    std::vector<PMPPoint> traj_copy;
    double dt_ref = 0.02;
    double T_ref = 0.0;
    double t = 0.0;
    int step_count = 0;
    int max_steps = 1;
    std::string mode = "mpc_lite";
    int horizonN = 20;
    double u_max = 8.0;
    double success_tol = 0.03;
    std::vector<double> last_tau(6, 0.0);
    double rl_inertia_scale = 1.0;
    double rl_friction_scale = 1.0;
    double rw_pos = 2.0, rw_vel = 0.15, rw_u = 0.01, rw_du = 0.005, rw_success = 5.0;

    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!rl_active_ || ref_traj_.empty()) {
            auto resp = HttpResponse::newHttpJsonResponse(make_error("No active RL episode. Call /rl/reset first."));
            resp->setStatusCode(k409Conflict);
            callback(resp);
            return;
        }
        traj_copy = ref_traj_;
        dt_ref = ref_dt_;
        T_ref = ref_T_;
        t = rl_t_;
        step_count = rl_step_count_;
        max_steps = rl_max_steps_;
        mode = rl_mode_;
        horizonN = rl_horizonN_;
        u_max = rl_u_max_;
        success_tol = rl_success_tol_;
        last_tau = rl_last_tau_cmd_;
        rl_inertia_scale = rl_inertia_scale_;
        rl_friction_scale = rl_friction_scale_;
        rw_pos = rw_pos_;
        rw_vel = rw_vel_;
        rw_u = rw_u_;
        rw_du = rw_du_;
        rw_success = rw_success_;
    }

    const auto& st = dyn_.state();
    std::vector<double> q = st.q;
    std::vector<double> dq = st.dq;

    std::vector<double> weights_vec(5, 0.0);
    std::vector<double> action_vec(5, 0.0);
    bool have_action = false;

    std::string err;
    if (json->isMember("action")) {
        // Агент предоставил действие PPO
        if (!read_array_double(*json, "action", action_vec, err, 5, 5)) {
            auto resp = HttpResponse::newHttpJsonResponse(make_error(err));
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }
        weights_vec = map_action_to_weights(action_vec);
        have_action = true;
    } else if (json->isMember("weights") && (*json)["weights"].isObject()) {
        // Агент предоставил веса LQR напрямую
        const auto& w = (*json)["weights"];
        weights_vec[0] = w.isMember("wq") ? w["wq"].asDouble() : 30.0;
        weights_vec[1] = w.isMember("wdq") ? w["wdq"].asDouble() : 2.0;
        weights_vec[2] = w.isMember("wu") ? w["wu"].asDouble() : 0.1;
        weights_vec[3] = w.isMember("wqN") ? w["wqN"].asDouble() : weights_vec[0];
        weights_vec[4] = w.isMember("wdqN") ? w["wdqN"].asDouble() : weights_vec[1];
    } else {
        auto resp = HttpResponse::newHttpJsonResponse(make_error("Provide either action[5] or weights object."));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    // Вычисляем команды управления
    ControlResult ctrl = compute_control_step(
        q, dq, traj_copy, dt_ref, t, dt_ref, mode, horizonN,
        weights_vec[0], weights_vec[1], weights_vec[2], weights_vec[3], weights_vec[4], u_max, dyn_
    );

    // Дискретизируем опорную траекторию в следующий момент времени
    double t_next = std::min(t + dt_ref, T_ref);
    RefSample ref_next = sample_ref_from_traj(traj_copy, t_next, dt_ref);

    // Вычисляем ошибки отслеживания в следующем состоянии
    double eq_rms_next = rms_between(ctrl.q_cmd, ref_next.q);
    double edq_rms_next = rms_between(ctrl.dq_cmd, ref_next.dq);

    // Вычисляем энергии управления
    std::vector<double> du_vec(6, 0.0);
    for (int i = 0; i < 6; ++i) du_vec[i] = ctrl.tau_cmd[i] - last_tau[i];
    double u_energy = mean_sq(ctrl.tau_cmd);
    double du_energy = mean_sq(du_vec);

    // Функция вознаграждения
    double reward =
        -rw_pos * (eq_rms_next * eq_rms_next)
        -rw_vel * (edq_rms_next * edq_rms_next)
        -rw_u * u_energy
        -rw_du * du_energy;

    bool time_done = (t_next >= T_ref - 1e-9);
    bool success = time_done && (eq_rms_next <= success_tol);
    if (success) reward += rw_success;

    // ============================================================
    // Обновление метрик эпизода для тезиса
    // ============================================================
    // Используем РЕАЛЬНОЕ состояние манипулятора из dyn_.state()
    // (после интеграции stepWithTorque внутри compute_control_step).
    // J_step считается в векторной форме по суставам:
    //     J_step = (Σ_i wq·eq_i² + Σ_i wdq·edq_i² + Σ_i wu·τ_i²) · dt
    {
        const auto&  st_real = dyn_.state();
        const auto&  q_real  = st_real.q;
        const auto&  dq_real = st_real.dq;
        const double dt_step = dt_ref;

        // Векторные ошибки по суставам (реальное состояние против плана PMP)
        std::array<double, 6> eq_vec{}, edq_vec{};
        double sum_eq_sq_step  = 0.0;
        double sum_edq_sq_step = 0.0;
        double sum_tau_sq_step = 0.0;
        for (int i = 0; i < 6; ++i) {
            eq_vec[i]  = q_real[i]  - ref_next.q[i];
            edq_vec[i] = dq_real[i] - ref_next.dq[i];
            sum_eq_sq_step  += eq_vec[i]  * eq_vec[i];
            sum_edq_sq_step += edq_vec[i] * edq_vec[i];
            sum_tau_sq_step += ctrl.tau_cmd[i] * ctrl.tau_cmd[i];

            const double abs_eq_i = std::abs(eq_vec[i]);
            if (abs_eq_i > ep_metrics_.max_abs_eq) ep_metrics_.max_abs_eq = abs_eq_i;
        }

        // Квадратичная стоимость LQR (векторная форма по суставам)
        const double J_step =
            (weights_vec[0] * sum_eq_sq_step   // eqᵀ Q  eq
           + weights_vec[1] * sum_edq_sq_step  // edqᵀ Qd edq
           + weights_vec[2] * sum_tau_sq_step  // τᵀ  R  τ
            ) * dt_step;

        ep_metrics_.sum_eq_sq    += sum_eq_sq_step;
        ep_metrics_.sum_edq_sq   += sum_edq_sq_step;
        ep_metrics_.sum_u_sq_dt  += sum_tau_sq_step * dt_step;
        ep_metrics_.sum_J        += J_step;
        ep_metrics_.n_samples    += 1;
        ep_metrics_.reward_total += reward;

        // Построчный CSV-лог (по одному шагу на строку)
        if (csv_log_enabled_ && csv_log_.is_open()) {
            csv_log_ << std::fixed << std::setprecision(6)
                     << t_next       << ','
                     << eq_rms_next  << ','
                     << edq_rms_next << ','
                     << u_energy     << ','
                     << J_step       << ','
                     << reward       << ','
                     << (success ? 1 : 0);
            for (int i = 0; i < 6; ++i) csv_log_ << ',' << q_real[i];
            for (int i = 0; i < 6; ++i) csv_log_ << ',' << dq_real[i];
            for (int i = 0; i < 6; ++i) csv_log_ << ',' << ref_next.q[i];
            for (int i = 0; i < 6; ++i) csv_log_ << ',' << ref_next.dq[i];
            for (int i = 0; i < 6; ++i) csv_log_ << ',' << eq_vec[i];
            for (int i = 0; i < 6; ++i) csv_log_ << ',' << ctrl.tau_cmd[i];
            csv_log_ << '\n';
        }
    }

    int next_step_count = step_count + 1;
    bool truncated = next_step_count >= max_steps;
    bool done = success || time_done || truncated;

    // Собираем наблюдение для следующего шага
    std::vector<double> obs = build_obs(ctrl.q_cmd, ctrl.dq_cmd, ref_next, t_next, T_ref);

    {
        std::lock_guard<std::mutex> lk(mtx_);
        rl_t_ = t_next;
        rl_step_count_ = next_step_count;
        rl_last_tau_cmd_ = ctrl.tau_cmd;
        if (done) rl_active_ = false;
    }

    // ============================================================
    // Финализация эпизода: печать сводки и закрытие CSV
    // ============================================================
    if (done) {
        ep_metrics_.eq_final = eq_rms_next;
        ep_metrics_.success  = success;

        const int    N = std::max(1, ep_metrics_.n_samples);
        // eq_rms_global и edq_rms_global усредняются по (N · 6)
        // потому что sum_eq_sq уже суммирует по 6 суставам на каждом шаге.
        const double eq_rms_global  = std::sqrt(ep_metrics_.sum_eq_sq  / (N * 6.0));
        const double edq_rms_global = std::sqrt(ep_metrics_.sum_edq_sq / (N * 6.0));

        std::cout
            << "\n========== EPISODE METRICS ==========\n"
            << "  steps             : " << N << "\n"
            << "  mode              : " << mode << "\n"
            << "  weights (wq/wdq/wu): "
                << weights_vec[0] << " / "
                << weights_vec[1] << " / "
                << weights_vec[2] << "\n"
            << "  --- Tracker (real state vs PMP) ---\n"
            << "  eq_rms_global     : " << eq_rms_global    << " rad\n"
            << "  edq_rms_global    : " << edq_rms_global   << " rad/s\n"
            << "  max_abs_eq        : " << ep_metrics_.max_abs_eq << " rad\n"
            << "  eq_final          : " << ep_metrics_.eq_final   << " rad\n"
            << "  u_energy_total    : " << ep_metrics_.sum_u_sq_dt << " Nm^2*s\n"
            << "  J_total (LQR)     : " << ep_metrics_.sum_J       << "\n"
            << "  reward_total      : " << ep_metrics_.reward_total << "\n"
            << "  success           : " << (success ? "YES" : "no") << "\n"
            << "  --- PMP plan ---\n"
            << "  T_plan            : " << ep_metrics_.ref_T        << " s\n"
            << "  ref_max_|dq|      : " << ep_metrics_.ref_max_dq   << " rad/s\n"
            << "  ref_max_|ddq|     : " << ep_metrics_.ref_max_ddq  << " rad/s^2\n"
            << "  jerk_energy       : " << ep_metrics_.ref_jerk_energy << "\n"
            << "  path_length       : " << ep_metrics_.ref_path_length << " rad\n"
            << "  --- Domain randomization ---\n"
            << "  friction_scale    : " << rl_friction_scale << "\n"
            << "  inertia_scale     : " << rl_inertia_scale  << "\n"
            << "=====================================\n" << std::endl;

        if (csv_log_.is_open()) csv_log_.close();
    }

    Json::Value out(Json::objectValue);
    out["ok"] = true;
    out["obs"] = to_json_array(obs);
    out["reward"] = reward;
    out["done"] = done;
    out["truncated"] = truncated && !success && !time_done;
    out["t"] = t_next;
    out["step_count"] = next_step_count;
    out["q_cmd"] = to_q6_json(ctrl.q_cmd);
    out["dq_cmd"] = to_q6_json(ctrl.dq_cmd);
    out["ddq_cmd"] = to_q6_json(ctrl.ddq_cmd);
    out["tau_cmd"] = to_q6_json(ctrl.tau_cmd);

    Json::Value weights(Json::objectValue);
    weights["wq"] = weights_vec[0];
    weights["wdq"] = weights_vec[1];
    weights["wu"] = weights_vec[2];
    weights["wqN"] = weights_vec[3];
    weights["wdqN"] = weights_vec[4];
    out["weights_used"] = weights;
    if (have_action) out["action_used"] = to_json_array(action_vec);

    Json::Value info(Json::objectValue);
    info["eq_rms"] = eq_rms_next;
    info["edq_rms"] = edq_rms_next;
    info["u_energy"] = u_energy;
    info["du_energy"] = du_energy;
    info["success"] = success;
    info["time_done"] = time_done;
    info["friction_scale"] = rl_friction_scale;
    info["inertia_scale"] = rl_inertia_scale;
    out["info"] = info;

    callback(HttpResponse::newHttpJsonResponse(out));
}
