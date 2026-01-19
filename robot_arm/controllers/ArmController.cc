#include "ArmController.h"
#include <drogon/HttpAppFramework.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <json/json.h>
#include <iostream>

#include "trajectory.hpp"         // plan_pmp_minimum_jerk(...)

using namespace drogon;

// Helper: takes q of size 3/4/6 and returns JSON array always of size 6 (pads missing with zeros)
static Json::Value to_q6_json(const std::vector<double> &q_in)
{
    Json::Value q(Json::arrayValue);
    for (int i = 0; i < 6; ++i) {
        double v = (i < (int)q_in.size()) ? q_in[i] : 0.0;
        q.append(v);
    }
    return q;
}

// Constructor: initializes internal dynamics model for 6 DOF and sets state to zeros
ArmController::ArmController()
    : dyn_(6)
{
    dyn_.setState({0,0,0,0,0,0}, {0,0,0,0,0,0});
}

// HTTP handler: POST /arm/plan_pmp_q
void ArmController::handlePlanPMP_Q(const HttpRequestPtr &req,
                                   std::function<void (const HttpResponsePtr &)> &&callback)
{
    // Try to get JSON directly from request (if Content-Type is application/json)
    auto json = req->getJsonObject();

    // Fallback: manually parse body if getJsonObject() returned null
    Json::Value root;
    if (!json) {
        const auto& body = req->getBody();
        Json::CharReaderBuilder b;
        std::string errs;
        std::unique_ptr<Json::CharReader> reader(b.newCharReader());
        if (!reader->parse(body.data(), body.data() + body.size(), &root, &errs)) {
            auto resp = HttpResponse::newHttpJsonResponse(Json::Value("Bad JSON body"));
            resp->setStatusCode(k400BadRequest);
            callback(resp);
            return;
        }
        json = std::make_shared<Json::Value>(root);
    }

    // Validate that q_target exists and is an array
    if (!json->isMember("q_target") || !(*json)["q_target"].isArray()) {
        auto resp = HttpResponse::newHttpJsonResponse(Json::Value("Not enough parameters: q_target (array)"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    // Validate q_target has at least 6 values
    const auto& arr = (*json)["q_target"];
    if (arr.size() < 6) {
        auto resp = HttpResponse::newHttpJsonResponse(Json::Value("q_target must have 6 values"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    // Read 6-DOF target configuration in radians
    std::vector<double> q_target6 = {
        arr[0].asDouble(),
        arr[1].asDouble(),
        arr[2].asDouble(),
        arr[3].asDouble(),
        arr[4].asDouble(),
        arr[5].asDouble()
    };

    // Read optional parameters (defaults if missing)
    double T  = json->isMember("T")  ? (*json)["T"].asDouble()  : 1.0;
    double dt = json->isMember("dt") ? (*json)["dt"].asDouble() : 0.02;

    // Ensure internal state vectors are 6 DOF (safety for older / inconsistent state)
    auto st = dyn_.state();
    if (st.q.size() < 6) dyn_.setState({0,0,0,0,0,0}, {0,0,0,0,0,0});

    // Current joint state q0 (rad) as start point for planning
    auto q0 = dyn_.state().q;
    std::vector<double> q0_6 = { q0[0], q0[1], q0[2], q0[3], q0[4], q0[5] };

    // Compute PMP + minimum-jerk trajectory: returns list of points {t, q}
    auto pmp_traj = plan_pmp_minimum_jerk(q0_6, q_target6, T, dt);

    // Update internal dynamics state to final pose (so next request starts from last target)
    auto st2 = dyn_.state();
    std::vector<double> q6  = st2.q;
    std::vector<double> dq6 = st2.dq;
    if (q6.size()  < 6) q6  = {0,0,0,0,0,0};
    if (dq6.size() < 6) dq6 = {0,0,0,0,0,0};

    for (int i = 0; i < 6; ++i) {
        q6[i]  = q_target6[i];
        dq6[i] = 0.0; // stop at the end
    }
    dyn_.setState(q6, dq6);

    // Build JSON response: { dt, unit, trajectory: [ {t, q[6]}, ... ] }
    Json::Value out(Json::objectValue);
    out["dt"] = dt;
    out["unit"] = "rad";
    out["trajectory"] = Json::arrayValue;

    for (const auto &p : pmp_traj) {
        Json::Value item(Json::objectValue);
        item["t"] = p.t;
        item["q"] = to_q6_json(p.q); // ensures always 6 values (pads if needed)
        out["trajectory"].append(item);
    }

    // Send response
    auto resp = HttpResponse::newHttpJsonResponse(out);
    callback(resp);
}


