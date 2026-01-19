#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

/*
  
    - State:  x1=q, x2=dq, x3=ddq
    - Control u = dddq (jerk)
    - Cost:   J = ∫ (1/2) u^2 dt

      q(t)   = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
      dq(t)  = derivative
      ddq(t) = second derivative
      u(t)   = third derivative  (jerk)

  
*/

struct PMPPoint {
    double t;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> u;      // jerk

    // PMP costates 
    std::vector<double> lambda1; //  costate associated with position q
    std::vector<double> lambda2; //costate associated with velocity dq
    std::vector<double> lambda3; // satisfies u = -lambda3

    double J_acc = 0.0; // J_acc: accumulated value of the cost functional
    // J_acc(t_k) ≈ ∫_0^{t_k} (1/2) ||u(t)||^2 dt

};  


// ------------------------------------------------------------
// Helper: solve 6x6 linear system (Gaussian elimination with pivoting) Ax=b
// ------------------------------------------------------------
inline std::vector<double> solve6(std::vector<std::vector<double>> A,
                                 std::vector<double> b)
{   //Verification of dimensions A=6, b=6
    const int n = 6;
    if ((int)A.size() != n || (int)b.size() != n) {
        throw std::runtime_error("solve6: bad dimensions");
    }
    for (int i = 0; i < n; ++i) {
        if ((int)A[i].size() != n) throw std::runtime_error("solve6: bad matrix row");
        A[i].push_back(b[i]); // augment [A∣b]
    }

    // Forward elimination with partial pivoting
    for (int col = 0; col < n; ++col) {
        int piv = col;
        double best = std::fabs(A[col][col]);
        for (int r = col + 1; r < n; ++r) {
            double v = std::fabs(A[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-12) throw std::runtime_error("solve6: singular system");
        if (piv != col) std::swap(A[piv], A[col]);

        double diag = A[col][col];
        for (int c = col; c <= n; ++c) A[col][c] /= diag;

        for (int r = col + 1; r < n; ++r) {
            double f = A[r][col];
            for (int c = col; c <= n; ++c) A[r][c] -= f * A[col][c];
        }
    }

    // Back substitution
    std::vector<double> x(n, 0.0);
    for (int r = n - 1; r >= 0; --r) {
        double s = A[r][n]; // RHS
        for (int c = r + 1; c < n; ++c) s -= A[r][c] * x[c];
        x[r] = s; // since diagonal is 1
    }
    return x;
}

// ------------------------------------------------------------
// Quintic coefficients for general boundary conditions:
//   q(0)=q0, dq(0)=v0, ddq(0)=a0
//   q(T)=q1, dq(T)=v1, ddq(T)=a1
//
// Returns a = [a0..a5] for q(t) = a0 + a1 t + ... + a5 t^5
// ------------------------------------------------------------
inline std::vector<double> quintic_coeffs(double q0, double v0, double a0,
                                         double q1, double v1, double a1,
                                         double T)
{
    if (T <= 1e-9) throw std::runtime_error("quintic_coeffs: T too small");

    const double TT  = T;
    const double TT2 = TT*TT;
    const double TT3 = TT2*TT;
    const double TT4 = TT3*TT;
    const double TT5 = TT4*TT;
    
    // Aa=b
    std::vector<std::vector<double>> A(6, std::vector<double>(6, 0.0));
    std::vector<double> b(6, 0.0);

    // q(0)=q0
    A[0][0] = 1.0; b[0] = q0;

    // dq(0)=v0
    A[1][1] = 1.0; b[1] = v0;

    // ddq(0)=a0 -> 2*a2 = a0
    A[2][2] = 2.0; b[2] = a0;

    // q(T)=q1
    A[3][0] = 1.0;
    A[3][1] = TT;
    A[3][2] = TT2;
    A[3][3] = TT3;
    A[3][4] = TT4;
    A[3][5] = TT5;
    b[3] = q1;

    // dq(T)=v1
    A[4][1] = 1.0;
    A[4][2] = 2.0*TT;
    A[4][3] = 3.0*TT2;
    A[4][4] = 4.0*TT3;
    A[4][5] = 5.0*TT4;
    b[4] = v1;

    // ddq(T)=a1
    A[5][2] = 2.0;
    A[5][3] = 6.0*TT;
    A[5][4] = 12.0*TT2;
    A[5][5] = 20.0*TT3;
    b[5] = a1;

    return solve6(A, b);
}

// ------------------------------------------------------------
// Plan min-jerk using quintic.
// Default boundary velocities/accelerations are zero.
// Output table rows: [t, q1, q2, ...]
// ------------------------------------------------------------
inline std::vector<std::vector<double>> plan_minjerk(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    double T, double dt)
{
    const size_t dof = q0.size();
    if (q1.size() != dof) throw std::runtime_error("plan_minjerk: size mismatch");

    int N = std::max(2, (int)std::round(T / std::max(dt, 1e-9)));
    std::vector<std::vector<double>> out;
    out.reserve((size_t)N + 1);

    // Standard: v0=a0=v1=a1=0
    std::vector<std::vector<double>> coeffs(dof);
    for (size_t i = 0; i < dof; ++i) {
        coeffs[i] = quintic_coeffs(q0[i], 0.0, 0.0, q1[i], 0.0, 0.0, T);
    }

    for (int k = 0; k <= N; ++k) {
        double t = k * dt;
        if (t > T) t = T;

        std::vector<double> row(1 + (int)dof, 0.0);
        row[0] = t;

        const double tt  = t;
        const double tt2 = tt * tt;
        const double tt3 = tt2 * tt;
        const double tt4 = tt3 * tt;
        const double tt5 = tt4 * tt;

        for (size_t i = 0; i < dof; ++i) {
            const auto& a = coeffs[i];
            row[1 + (int)i] = a[0] + a[1]*tt + a[2]*tt2 + a[3]*tt3 + a[4]*tt4 + a[5]*tt5;
        }
        out.push_back(std::move(row));
    }
    return out;
}

// ------------------------------------------------------------
// Plan PMP minimum-jerk trajectory explicitly (quintic + derivatives).
// Returns q, dq, ddq, u(=jerk), costates, and J_acc (accumulated cost).
// Default: zero boundary velocities/accelerations.
// Model (triple integrator per joint):
//   x1 = q,  x2 = dq,  x3 = ddq
//   x1' = x2
//   x2' = x3
//   x3' = u   , where u = dddq (jerk)
//
// Minimum-jerk cost functional:
//   J = ∫_0^T (1/2) ||u(t)||^2 dt
//
// Known solution under standard boundary conditions:
//   dq(0)=ddq(0)=dq(T)=ddq(T)=0  ⇒ q(t) is a quintic polynomial
//
// Quintic trajectory for each DOF i:
//   q_i(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
// Derivatives:
//   dq_i(t)  = a1 + 2 a2 t + 3 a3 t^2 + 4 a4 t^3 + 5 a5 t^4
//   ddq_i(t) = 2 a2 + 6 a3 t + 12 a4 t^2 + 20 a5 t^3
//   u_i(t)   = dddq_i(t) = 6 a3 + 24 a4 t + 60 a5 t^2
//
// PMP (for explicit visibility):
//   Hamiltonian: H = (1/2)u^2 + λ1 x2 + λ2 x3 + λ3 u
//   Optimality:  ∂H/∂u = u + λ3 = 0  ⇒  u* = -λ3
//   Adjoint equations (for this system):
//     dλ3/dt = -λ2
//     dλ2/dt = -λ1
// (Here we show one consistent choice derived from u(t)):
//   λ3 = -u
//   λ2 = -dλ3/dt = du/dt
//   λ1 = -dλ2/dt = -d²u/dt²
//
// Accumulated cost (numerical approximation):
//   J_acc(t_k) ≈ Σ_{j=0..k} (1/2) ||u(t_j)||^2 dt
// ------------------------------------------------------------
inline std::vector<PMPPoint> plan_pmp_minimum_jerk(
    const std::vector<double>& q0,
    const std::vector<double>& q1,
    double T, double dt)
{
    const size_t dof = q0.size(); // DOF = degrees of freedom = number of joints
    if (q1.size() != dof) throw std::runtime_error("plan_pmp_minimum_jerk: size mismatch");

    // Number of samples N (at least 2) on [0, T] with step dt:
    //   N ≈ round(T/dt)
    int N = std::max(2, (int)std::round(T / std::max(dt, 1e-9)));

    std::vector<PMPPoint> out;
    out.reserve((size_t)N + 1);

    // ------------------------------------------------------------
    //   For each joint i, compute quintic coefficients enforcing:
    //    q(0)=q0, dq(0)=0, ddq(0)=0
    //    q(T)=q1, dq(T)=0, ddq(T)=0
    //
    // This builds a 6x6 linear system and solves:
    //    A a = b   ⇒ a = [a0..a5]
    // ------------------------------------------------------------
    std::vector<std::vector<double>> coeffs(dof);
    for (size_t i = 0; i < dof; ++i) {
        coeffs[i] = quintic_coeffs(q0[i], 0.0, 0.0, q1[i], 0.0, 0.0, T);
    }

    // ------------------------------------------------------------
    //    Initialize accumulated cost:
    //    J_acc(0) = 0
    // ------------------------------------------------------------
    double J_acc = 0.0;

    // ------------------------------------------------------------
    //    Sample the trajectory at t_k = k*dt, k=0..N
    // ------------------------------------------------------------
    for (int k = 0; k <= N; ++k) {
        double t = k * dt;
        if (t > T) t = T; // clamp last sample to exactly T

        // Precompute powers for polynomial evaluation
        const double tt  = t;
        const double tt2 = tt * tt;
        const double tt3 = tt2 * tt;
        const double tt4 = tt3 * tt;
        const double tt5 = tt4 * tt;

        PMPPoint p;
        p.t = t;
        p.q.assign(dof, 0.0);
        p.dq.assign(dof, 0.0);
        p.ddq.assign(dof, 0.0);
        p.u.assign(dof, 0.0);
        p.lambda1.assign(dof, 0.0);
        p.lambda2.assign(dof, 0.0);
        p.lambda3.assign(dof, 0.0);
        p.J_acc = 0.0;

        // ------------------------------------------------------------
        //   For each joint i evaluate:
        //    q_i(t), dq_i(t), ddq_i(t), u_i(t)
        //    and then (λ1, λ2, λ3) for PMP visibility
        // ------------------------------------------------------------
        for (size_t i = 0; i < dof; ++i) {
            const auto& a = coeffs[i]; // a[0]..a[5]

            // q_i(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
            p.q[i] = a[0] + a[1]*tt + a[2]*tt2 + a[3]*tt3 + a[4]*tt4 + a[5]*tt5;

            // dq_i(t) = a1 + 2a2 t + 3a3 t^2 + 4a4 t^3 + 5a5 t^4
            p.dq[i] = a[1]
                    + 2.0*a[2]*tt
                    + 3.0*a[3]*tt2
                    + 4.0*a[4]*tt3
                    + 5.0*a[5]*tt4;

            // ddq_i(t) = 2a2 + 6a3 t + 12a4 t^2 + 20a5 t^3
            p.ddq[i] = 2.0*a[2]
                     + 6.0*a[3]*tt
                     + 12.0*a[4]*tt2
                     + 20.0*a[5]*tt3;

            // u_i(t) = dddq_i(t) = 6a3 + 24a4 t + 60a5 t^2
            p.u[i] = 6.0*a[3]
                   + 24.0*a[4]*tt
                   + 60.0*a[5]*tt2;

            // PMP: u* = -λ3  ⇒ λ3 = -u
            p.lambda3[i] = -p.u[i];

            // du/dt = 24a4 + 120a5 t
            const double du_dt = 24.0*a[4] + 120.0*a[5]*tt;

            // d²u/dt² = 120a5 (constant)
            const double d2u_dt2 = 120.0*a[5];

            // From adjoint relations (consistent choice):
            //   λ2 = du/dt
            //   λ1 = -d²u/dt²
            p.lambda2[i] = du_dt;
            p.lambda1[i] = -d2u_dt2;
        }

        // ------------------------------------------------------------
        // 5) Accumulate cost using squared norm of the jerk vector:
        //    ||u||^2 = Σ_i u_i^2
        //    J_acc += (1/2) * ||u(t_k)||^2 * dt
        // ------------------------------------------------------------
        double u2 = 0.0;
        for (size_t i = 0; i < dof; ++i) {
            u2 += p.u[i] * p.u[i];
        }
        J_acc += 0.5 * u2 * dt;
        p.J_acc = J_acc;

        out.push_back(std::move(p));
    }

    return out;
}