#pragma once
#include <vector>
#include <algorithm>
#include <cassert>

struct ArmState {
    std::vector<double> q;   // (rad)
    std::vector<double> dq;  // (rad/s)
};

class SimpleDynamics {
public:
    explicit SimpleDynamics(size_t dof) : dof_(dof) {
        state_.q.assign(dof_, 0.0);
        state_.dq.assign(dof_, 0.0);
        tau_.assign(dof_, 0.0);

        qmin_.assign(dof_, -3.14159); // -180°
        qmax_.assign(dof_,  3.14159); //  180°
        dqmax_.assign(dof_,  4.0);    // rad/s máx.
    }

    const ArmState& state() const { return state_; }

    void setState(const std::vector<double>& q,
                  const std::vector<double>& dq)
    {
        assert(q.size() == dof_ && dq.size() == dof_);
        state_.q = q;
        state_.dq = dq;
        clampState();
    }

    void setTorque(const std::vector<double>& tau){
        assert(tau.size() == dof_);
        tau_ = tau;
    }

    // Minimum model: ddq = tau;  dq += dt*ddq;  q += dt*dq
    void step(double dt){
        for(size_t i = 0; i < dof_; ++i){
            double ddq = tau_[i];
            state_.dq[i] += dt * ddq;
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
            state_.q[i]  += dt * state_.dq[i];
            state_.q[i]  = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
        }
    }

private:
    void clampState(){
        for(size_t i = 0; i < dof_; ++i){
            state_.q[i]  = std::clamp(state_.q[i],  qmin_[i], qmax_[i]);
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
        }
    }

    size_t dof_;
    ArmState state_;
    std::vector<double> tau_;
    std::vector<double> qmin_, qmax_, dqmax_;
};

