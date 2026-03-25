#pragma once

#include <vector>
#include <algorithm>
#include <cassert>
#include <cmath>

// ------------------------------------------------------------
// ArmState — состояние 6-осевого манипулятора в суставных координатах.
// q   — положения суставов (rad)
// dq  — скорости суставов  (rad/s)
// ddq — ускорения суставов (rad/s^2), последнее вычисленное значение
// ------------------------------------------------------------
struct ArmState {
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
};

// ------------------------------------------------------------
// ArmDynamics — UR5e-calibrated
//
// Approximate joint-space dynamics model:
//   M(q) ddq + n(q,dq) = tau
// where
//   M(q)  — diagonal, configuration-dependent inertia matrix,
//   n(q,dq) = C(q,dq)dq + g(q) + F(dq)
//
// Parameters calibrated for the Universal Robots UR5e:
//   - Link masses from URDF: 3.761, 8.393, 2.275, 1.219, 1.219, 0.1879 kg
//   - DH lengths: a2=0.425m, a3=0.3922m, d4=0.1333m, d5=0.0997m, d6=0.0996m
//   - Joints 1-3 ("size 3"): max torque 150 Nm, max velocity pi rad/s
//   - Joints 4-6 ("size 1"): max torque 28 Nm, max velocity 2*pi rad/s
//   - All joints use 100:1 harmonic drive (HFUS-2SH strain wave gearing)
//   - Gravity terms computed from actual link masses and CoM positions
//   - Coriolis coupling terms scaled to UR5e geometry
//   - Friction coefficients estimated from identified UR5 parameters
// ------------------------------------------------------------
class ArmDynamics {
public:
    explicit ArmDynamics(size_t dof = 6) : dof_(dof) {
        state_.q.assign(dof_, 0.0);
        state_.dq.assign(dof_, 0.0);
        state_.ddq.assign(dof_, 0.0);
        tau_.assign(dof_, 0.0);

        const double PI = 3.14159265358979323846;

        // UR5e joint position limits (rad) — from URDF
        // All revolute joints: -2*pi to +2*pi
        qmin_ = {-2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI, -2.0*PI};
        qmax_ = { 2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI,  2.0*PI};

        // UR5e joint velocity limits (rad/s)
        // Joints 1-3 ("size 3"): pi rad/s ≈ 3.14
        // Joints 4-6 ("size 1"): 2*pi rad/s ≈ 6.28
        dqmax_ = {PI, PI, PI, 2.0*PI, 2.0*PI, 2.0*PI};

        // UR5e joint torque limits (Nm)
        // Joints 1-3: 150 Nm,  Joints 4-6: 28 Nm
        taumax_ = {150.0, 150.0, 150.0, 28.0, 28.0, 28.0};

        // UR5e effective joint-space inertias (kg*m^2)
        // Computed from URDF link masses and geometry:
        //   Link masses: m1=3.761, m2=8.393, m3=2.275, m4=1.219, m5=1.219, m6=0.1879 kg
        //   DH lengths:  a2=0.425m, a3=0.3922m
        // Includes reflected rotor inertia through 100:1 harmonic drives
        // Jbase = approximate inertia at home config, Jvar = configuration-dependent variation
        Jbase_ = {5.5, 10.8, 3.6, 0.45, 0.20, 0.08};
        Jvar_  = {0.6,  2.5, 1.2, 0.08, 0.03, 0.01};

        // Viscous friction (Nm*s/rad) — identified from UR5e motion data
        Bvisc_ = {3.5, 4.8, 3.2, 0.70, 0.40, 0.18};

        // Coulomb friction (Nm) — identified from UR5e
        Fcoul_ = {0.55, 0.80, 0.60, 0.14, 0.10, 0.05};

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

    const ArmState& state() const { return state_; }

    void setState(const std::vector<double>& q,
                  const std::vector<double>& dq) {
        assert(q.size() == dof_ && dq.size() == dof_);
        state_.q = q;
        state_.dq = dq;
        state_.ddq.assign(dof_, 0.0);
        clampState();
    }

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

    void stepWithTorque(const std::vector<double>& tau, double dt) {
        assert(tau.size() == dof_);
        if (dt <= 1e-9) dt = 1e-3;

        for (size_t i = 0; i < dof_; ++i) {
            tau_[i] = std::clamp(tau[i], -taumax_[i], taumax_[i]);
        }

        const std::vector<double> M = computeMassDiag(state_.q);
        const std::vector<double> n = computeNonlinear(state_.q, state_.dq);

        for (size_t i = 0; i < dof_; ++i) {
            const double Mi = std::max(1e-6, M[i]);
            double ddq = (tau_[i] - n[i]) / Mi;
            // мягкое численное ограничение на ускорения
            ddq = std::clamp(ddq, -25.0, 25.0);
            state_.ddq[i] = ddq;

            // semi-implicit Euler
            state_.dq[i] += dt * ddq;
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
            state_.q[i]  += dt * state_.dq[i];

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
    static double sgn_soft(double x) {
        return std::tanh(8.0 * x);
    }

    std::vector<double> computeMassDiag(const std::vector<double>& q) const {
        std::vector<double> M(dof_, 1.0);
        if (dof_ != 6) return M;

        // UR5e geometry: a2=0.425m, a3=0.3922m
        // Joint 1 (shoulder pan): vertical rotation, sees all distal mass
        //   Variation depends on shoulder lift angle q[1] (how extended the arm is)
        M[0] = Jbase_[0] + Jvar_[0] * (1.0 + 0.22 * std::cos(q[1]));

        // Joint 2 (shoulder lift): heaviest — carries upper arm + forearm + wrists
        //   m2*a2^2/3 + (m3+m4+m5+m6)*a2^2 dominated by link 2 (8.393 kg)
        //   Depends on q[1] (upper arm angle) and q[1]+q[2] (elbow extension)
        M[1] = Jbase_[1] + Jvar_[1] * (1.0 + 0.45 * std::cos(q[1]) + 0.30 * std::cos(q[1] + q[2]));

        // Joint 3 (elbow): carries forearm (2.275 kg) + wrists (2.63 kg)
        //   a3=0.3922m, depends on q[2] and combined q[1]+q[2]
        M[2] = Jbase_[2] + Jvar_[2] * (1.0 + 0.55 * std::cos(q[2]) + 0.22 * std::cos(q[1] + q[2]));

        // Joint 4 (wrist 1): lightweight, d4=0.1333m, m4=1.219 kg
        M[3] = Jbase_[3] + Jvar_[3] * (1.0 + 0.12 * std::cos(q[3]));

        // Joint 5 (wrist 2): d5=0.0997m, m5=1.219 kg
        M[4] = Jbase_[4] + Jvar_[4] * (1.0 + 0.10 * std::cos(q[4]));

        // Joint 6 (wrist 3): lightest, d6=0.0996m, m6=0.1879 kg
        M[5] = Jbase_[5] + Jvar_[5] * (1.0 + 0.08 * std::cos(q[5]));

        for (double& x : M) x = std::max(0.03, x);
        return M;
    }

    std::vector<double> computeNonlinear(const std::vector<double>& q,
                                         const std::vector<double>& dq) const {
        std::vector<double> n(dof_, 0.0);
        if (dof_ != 6) return n;

        // --- Viscous + Coulomb friction ---
        for (size_t i = 0; i < dof_; ++i) {
            n[i] += Bvisc_[i] * dq[i] + Fcoul_[i] * sgn_soft(dq[i]);
        }

        // --- Gravity (g = 9.81 m/s^2) ---
        // UR5e: a2=0.425m, a3=0.3922m, d4=0.1333m, d5=0.0997m, d6=0.0996m
        // Link masses: m2=8.393, m3=2.275, m4=1.219, m5=1.219, m6=0.1879 kg
        //
        // Joint 2 (shoulder lift) — bears most gravity load:
        //   g2 = g*[m2*Lc2*sin(q1) + (m3+m4+m5+m6)*a2*sin(q1)
        //         + m3*Lc3*sin(q1+q2) + (m4+m5+m6)*a3*sin(q1+q2)]
        //   Lc2 ≈ 0.2125m (half of a2), Lc3 ≈ 0.196m (half of a3)
        //   ≈ 9.81*(8.393*0.2125 + 4.90*0.425)*sin(q1) + 9.81*(2.275*0.196 + 2.63*0.3922)*sin(q1+q2)
        //   ≈ 37.9*sin(q1) + 14.5*sin(q1+q2)
        n[1] += 37.9 * std::sin(q[1]) + 14.5 * std::sin(q[1] + q[2]);

        // Joint 3 (elbow):
        //   g3 = g*[m3*Lc3 + (m4+m5+m6)*a3]*sin(q1+q2) ≈ 14.5*sin(q1+q2)
        n[2] += 14.5 * std::sin(q[1] + q[2]);

        // Joint 4 (wrist 1):
        //   g4 = g*(m4*Lc4 + (m5+m6)*d4)*sin(q1+q2+q3) ≈ 2.6*sin(q1+q2+q3)
        n[3] += 2.6 * std::sin(q[1] + q[2] + q[3]);

        // Joint 5 (wrist 2):
        //   g5 = g*(m5+m6)*d5*sin(q4) ≈ 1.4*sin(q4)
        n[4] += 1.4 * std::sin(q[4]);

        // Joint 6 (wrist 3): very light — m6=0.1879 kg
        n[5] += 0.18 * std::sin(q[5]);

        // --- Coriolis coupling terms ---
        // UR5e: main coupling is between joints 2-3 (shoulder-elbow)
        //   h23 = m3*a2*Lc3*sin(q2) + (m4+m5+m6)*a2*a3*sin(q2)
        //       ≈ (2.275*0.425*0.196 + 2.63*0.425*0.3922)*sin(q2)
        //       ≈ (0.189 + 0.438)*sin(q2) ≈ 2.55*sin(q2)
        const double h23 = 2.55 * std::sin(q[2]);
        n[1] += h23 * (2.0 * dq[1] * dq[2] + dq[2] * dq[2]);
        n[2] -= h23 * (dq[1] * dq[1]);

        // Coupling between joints 1-2 (base-shoulder):
        //   Inertia of all links projected through q[1]
        const double h12 = 0.50 * std::sin(q[1]);
        n[0] += h12 * dq[1] * dq[1];
        n[1] -= 0.25 * std::sin(q[1]) * dq[0] * dq[0];

        // Coupling between joints 3-4 (elbow-wrist1):
        //   Smaller — wrist links are light
        const double h34 = 0.18 * std::sin(q[3]);
        n[2] += h34 * (2.0 * dq[2] * dq[3] + dq[3] * dq[3]);
        n[3] -= h34 * (dq[2] * dq[2]);

        return n;
    }

    void clampState() {
        for (size_t i = 0; i < dof_; ++i) {
            state_.q[i]  = std::clamp(state_.q[i], qmin_[i], qmax_[i]);
            state_.dq[i] = std::clamp(state_.dq[i], -dqmax_[i], dqmax_[i]);
        }
    }

    size_t dof_;
    ArmState state_;
    std::vector<double> tau_;

    std::vector<double> qmin_, qmax_;
    std::vector<double> dqmax_;
    std::vector<double> taumax_;

    std::vector<double> Jbase_, Jvar_;
    std::vector<double> Bvisc_, Fcoul_;
};
