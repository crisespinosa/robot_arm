#include "ArmController.h"
#include <drogon/HttpAppFramework.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <json/json.h>
#include <iostream>

#include "trajectory.hpp" // plan_pmp_minimum_jerk(...)

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

// Helper: read an array of >=6 doubles from JSON key into a size-6 vector
static bool read_q6_array(const Json::Value& root, const char* key, std::vector<double>& out6, std::string& err)
{
    if (!root.isMember(key)) return false; // "optional" behavior
    const auto& arr = root[key];
    if (!arr.isArray()) {
        err = std::string(key) + " must be an array";
        return false;
    }
    if ((int)arr.size() < 6) {
        err = std::string(key) + " must have at least 6 values";
        return false;
    }

    out6 = {
        arr[0].asDouble(),
        arr[1].asDouble(),
        arr[2].asDouble(),
        arr[3].asDouble(),
        arr[4].asDouble(),
        arr[5].asDouble()
    };
    return true;
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

    // ---- Required: q_target ----
    if (!json->isMember("q_target") || !(*json)["q_target"].isArray() || (int)(*json)["q_target"].size() < 6) {
        auto resp = HttpResponse::newHttpJsonResponse(Json::Value("Not enough parameters: q_target (array length >= 6)"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }

    const auto& arrT = (*json)["q_target"];
    std::vector<double> q_target6 = {
        arrT[0].asDouble(),
        arrT[1].asDouble(),
        arrT[2].asDouble(),
        arrT[3].asDouble(),
        arrT[4].asDouble(),
        arrT[5].asDouble()
    };

    // ---- Optional: T, dt ----
    double T  = json->isMember("T")  ? (*json)["T"].asDouble()  : 1.0;
    double dt = json->isMember("dt") ? (*json)["dt"].asDouble() : 0.02;

    if (T <= 0.0) {
        auto resp = HttpResponse::newHttpJsonResponse(Json::Value("T must be > 0"));
        resp->setStatusCode(k400BadRequest);
        callback(resp);
        return;
    }
    if (dt <= 0.0) dt = 0.02;

    // ---- Optional (IMPORTANT): q_start ----
    // If Unity sends q_start, we use it as the true start.
    std::vector<double> q_start6;
    bool have_q_start = false;
    {
        std::string err;
        std::vector<double> tmp;
        bool present = read_q6_array(*json, "q_start", tmp, err);
        if (present) {
            // present can mean "exists and parsed OK"
            q_start6 = tmp;
            have_q_start = true;

            // Keep internal state consistent (not strictly required, but useful)
            dyn_.setState(q_start6, {0,0,0,0,0,0});
        } else {
            // If key exists but invalid (read_q6_array would have returned false with err),
            // we need to detect that: simplest is check member + isArray here.
            if (json->isMember("q_start")) {
                // It exists but was invalid
                // Try get detailed error using a strict check:
                if (!(*json)["q_start"].isArray() || (int)(*json)["q_start"].size() < 6) {
                    auto resp = HttpResponse::newHttpJsonResponse(Json::Value("q_start must be an array length >= 6"));
                    resp->setStatusCode(k400BadRequest);
                    callback(resp);
                    return;
                }
            }
        }
    }

    // ---- Decide q0 (start state) ----
    std::vector<double> q0_6(6, 0.0);
    if (have_q_start) {
        q0_6 = q_start6;
    } else {
        // fallback to internal state if Unity didn't send q_start
        auto st = dyn_.state();
        if (st.q.size() < 6) {
            dyn_.setState({0,0,0,0,0,0}, {0,0,0,0,0,0});
            st = dyn_.state();
        }
        for (int i = 0; i < 6; ++i) q0_6[i] = st.q[i];
    }

    LOG_INFO << "[ArmController] plan_pmp_q:"
             << " have_q_start=" << (have_q_start ? "true" : "false")
             << " q0[0]=" << q0_6[0] << " qT[0]=" << q_target6[0]
             << " T=" << T << " dt=" << dt;

    // ---- Plan trajectory (quintic / min-jerk) ----
    auto pmp_traj = plan_pmp_minimum_jerk(q0_6, q_target6, T, dt);

    // Update internal state to final target (kept for backward compatibility)
    {
        std::vector<double> q6  = q_target6;
        std::vector<double> dq6 = {0,0,0,0,0,0};
        dyn_.setState(q6, dq6);
    }

    // ---- Build JSON response ----
    Json::Value out(Json::objectValue);
    out["dt"] = dt;
    out["unit"] = "rad";

    // Extra debug fields (Unity can ignore them)
    out["q_start_used"] = to_q6_json(q0_6);
    out["q_target_used"] = to_q6_json(q_target6);

    out["trajectory"] = Json::arrayValue;
    for (const auto &p : pmp_traj) {
        Json::Value item(Json::objectValue);
        item["t"] = p.t;
        item["q"] = to_q6_json(p.q);
        out["trajectory"].append(item);
    }

    auto resp = HttpResponse::newHttpJsonResponse(out);
    callback(resp);
}

