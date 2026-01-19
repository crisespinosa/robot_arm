#pragma once
// Ensures this header file is included only once during compilation

#include <vector>
#include <algorithm>
#include <cassert>

struct ArmState {
    std::vector<double> q;   // Joint positions (rad)
    std::vector<double> dq;  // Joint velocities (rad/s)
};

class SimpleDynamics {
public:
    // Constructor: initializes a robot model with given degrees of freedom
    explicit SimpleDynamics(size_t dof) : dof_(dof) {
        state_.q.assign(dof_, 0.0);   // Initialize joint positions
        state_.dq.assign(dof_, 0.0);  // Initialize joint velocities
        tau_.assign(dof_, 0.0);       // Initialize control torques

        // Joint limits
        qmin_.assign(dof_, -3.14159); // -180 degrees
        qmax_.assign(dof_, 3.14159); // +180 degrees
        dqmax_.assign(dof_, 4.0);    // Max joint speed (rad/s)
    }

    // Returns the current state of the arm
    const ArmState& state() const { return state_; }

    // Sets the robot state (positions and velocities)
    void setState(const std::vector<double>& q,
        const std::vector<double>& dq)
    {
        assert(q.size() == dof_ && dq.size() == dof_);
        state_.q = q;
        state_.dq = dq;
        clampState(); // Enforce limits
    }

    // Sets the control torques
    void setTorque(const std::vector<double>& tau) {
        assert(tau.size() == dof_);
        tau_ = tau;
    }

    // Very simple dynamic model:
    // ddq = tau
    // dq += dt * ddq
    // q  += dt * dq
    void step(double dt) {
        for (size_t i = 0; i < dof_; ++i) {
            double ddq = tau_[i];                    // Acceleration = torque
            state_.dq[i] += dt * ddq;               // Integrate velocity
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
            state_.q[i] += dt * state_.dq[i];      // Integrate position
            state_.q[i] = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
        }
    }

private:
    // Enforces joint and velocity limits
    void clampState() {
        for (size_t i = 0; i < dof_; ++i) {
            state_.q[i] = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
        }
    }

    size_t dof_;                     // Number of degrees of freedom
    ArmState state_;                 // Current robot state
    std::vector<double> tau_;        // Control torques
    std::vector<double> qmin_, qmax_, dqmax_; // Joint and velocity limits
};
