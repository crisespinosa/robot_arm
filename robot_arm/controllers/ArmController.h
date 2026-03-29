#pragma once

#include <drogon/HttpController.h>
#include <functional>
#include <mutex>
#include <vector>
#include <string>

#include "dynamics.hpp"    // ArmDynamics
#include "trajectory.hpp"  // PMPPoint, plan_pmp_minimum_jerk

class ArmController : public drogon::HttpController<ArmController> {
public:
    ArmController();

    METHOD_LIST_BEGIN
        ADD_METHOD_TO(ArmController::handlePlanPMP_Q,     "/arm/plan_pmp_q",    drogon::Post);
        ADD_METHOD_TO(ArmController::handleSetReference,  "/arm/set_reference", drogon::Post);
        ADD_METHOD_TO(ArmController::handleStep,          "/arm/step",          drogon::Post);
        ADD_METHOD_TO(ArmController::handleRLReset,       "/rl/reset",          drogon::Post);
        ADD_METHOD_TO(ArmController::handleRLStep,        "/rl/step",           drogon::Post);
    METHOD_LIST_END

    void handlePlanPMP_Q(const drogon::HttpRequestPtr&,
                         std::function<void (const drogon::HttpResponsePtr&)>&&);

    void handleSetReference(const drogon::HttpRequestPtr&,
                            std::function<void (const drogon::HttpResponsePtr&)>&&);

    void handleStep(const drogon::HttpRequestPtr&,
                    std::function<void (const drogon::HttpResponsePtr&)>&&);

    void handleRLReset(const drogon::HttpRequestPtr&,
                       std::function<void (const drogon::HttpResponsePtr&)>&&);

    void handleRLStep(const drogon::HttpRequestPtr&,
                      std::function<void (const drogon::HttpResponsePtr&)>&&);

private:
    ArmDynamics dyn_;

    std::mutex mtx_;
    std::vector<PMPPoint> ref_traj_;
    double ref_dt_{0.02};
    double ref_T_{0.0};

    std::vector<double> last_q_meas_{std::vector<double>(6, 0.0)};
    bool have_last_q_{false};

    struct KF2 {
        double x0{0.0};
        double x1{0.0};
        double P00{1.0}, P01{0.0}, P10{0.0}, P11{1.0};
        bool initialized{false};
    };
    std::vector<KF2> kf_{std::vector<KF2>(6)};

    bool rl_active_{false};
    double rl_t_{0.0};
    double rl_dt_{0.02};
    double rl_T_{0.0};
    int rl_step_count_{0};
    int rl_max_steps_{300};
    double rl_success_tol_{0.03};
    std::string rl_mode_{"mpc_lite"};
    int rl_horizonN_{20};
    double rl_u_max_{8.0};
    std::vector<double> rl_last_tau_cmd_{std::vector<double>(6, 0.0)};
    std::vector<double> rl_q_start_{std::vector<double>(6, 0.0)};
    std::vector<double> rl_q_target_{std::vector<double>(6, 0.0)};
    double rl_inertia_scale_{1.0};
    double rl_friction_scale_{1.0};

    double rw_pos_{2.0};
    double rw_vel_{0.15};
    double rw_u_{0.002};
    double rw_du_{0.0005};
    double rw_success_{5.0};
};
