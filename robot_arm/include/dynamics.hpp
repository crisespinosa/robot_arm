#pragma once

#include <vector>
#include <algorithm>
#include <cassert>

// ------------------------------------------------------------
// ArmState — структура состояния манипулятора.
// Хранит текущие значения для всех суставов (размер = dof):
//   q   — положения (рад)
//   dq  — скорости  (рад/с)
//   ddq — ускорения (рад/с^2), последнее вычисленное значение
// ------------------------------------------------------------
struct ArmState {
    std::vector<double> q;    // положения (rad)
    std::vector<double> dq;   // скорости (rad/s)
    std::vector<double> ddq;  // ускорения (rad/s^2) (последнее значение)
};

// ------------------------------------------------------------
// SimpleDynamics — очень упрощённая динамика “для отладки/демо”.
// Здесь предполагается модель вида:
//   ddq = tau
// то есть момент tau напрямую задаёт угловое ускорение (без инерции/Кориолиса/гравитации).
//
// Интегрирование по времени (явный Эйлер):
//   dq(t+dt) = dq(t) + dt * ddq
//   q (t+dt) = q (t) + dt * dq(t+dt)
//
// Также добавлены ограничения:
//   q ∈ [qmin, qmax]
//   |dq| ≤ dqmax
// ------------------------------------------------------------
class SimpleDynamics {
public:
    // Конструктор: задаём число степеней свободы (суставов) dof
    explicit SimpleDynamics(size_t dof) : dof_(dof) {
        // Инициализация состояния нулями
        state_.q.assign(dof_, 0.0);
        state_.dq.assign(dof_, 0.0);
        state_.ddq.assign(dof_, 0.0);

        // Вектор управляющих моментов tau (по суставам)
        tau_.assign(dof_, 0.0);

        // Ограничения по углу (по умолчанию [-pi, pi])
        qmin_.assign(dof_, -3.14159);
        qmax_.assign(dof_,  3.14159);

        // Ограничения по модулю скорости (по умолчанию 4 рад/с)
        dqmax_.assign(dof_, 4.0);
    }

    // Доступ к текущему состоянию (read-only)
    const ArmState& state() const { return state_; }

    // Установка состояния (q, dq). Ускорения сбрасываем в 0,
    // затем “зажимаем” состояние в допустимых пределах.
    void setState(const std::vector<double>& q,
                  const std::vector<double>& dq)
    {
        assert(q.size() == dof_ && dq.size() == dof_);
        state_.q = q;
        state_.dq = dq;
        state_.ddq.assign(dof_, 0.0);
        clampState();
    }

    // Установка управляющих моментов tau (по суставам)
    void setTorque(const std::vector<double>& tau) {
        assert(tau.size() == dof_);
        tau_ = tau;
    }

    // Один шаг моделирования на dt (сек).
    //
    // Модель:
    //   ddq = tau
    //
    // Интегрирование (явный Эйлер):
    //   dq += dt * ddq
    //   q  += dt * dq
    //
    // После каждого обновления применяем ограничения:
    //   dq в пределах [-dqmax, dqmax]
    //   q  в пределах [qmin, qmax]
    void step(double dt) {
        for (size_t i = 0; i < dof_; ++i) {
            // По упрощённой модели момент напрямую равен ускорению
            double ddq = tau_[i];
            state_.ddq[i] = ddq;

            // Обновляем скорость
            state_.dq[i] += dt * ddq;
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);

            // Обновляем положение
            state_.q[i] += dt * state_.dq[i];
            state_.q[i] = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
        }
    }

private:
    // Внутренняя функция “зажима” состояния в допустимые пределы
    void clampState() {
        for (size_t i = 0; i < dof_; ++i) {
            state_.q[i]  = std::clamp(state_.q[i],  qmin_[i], qmax_[i]);
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
        }
    }

    size_t dof_;                 // число суставов
    ArmState state_;             // текущее состояние
    std::vector<double> tau_;    // управляющие моменты (в этой модели = ускорения)

    // Ограничения: углы и скорости
    std::vector<double> qmin_, qmax_, dqmax_;
};

