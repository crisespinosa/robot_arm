#pragma once

#include <drogon/HttpController.h>
#include <functional>
#include "dynamics.hpp"   // SimpleDynamics

class ArmController : public drogon::HttpController<ArmController> {
public:
    ArmController();

    METHOD_LIST_BEGIN
        ADD_METHOD_TO(ArmController::handlePlanPMP_Q,   "/arm/plan_pmp_q",drogon::Post);
    METHOD_LIST_END


    void handlePlanPMP_Q(const drogon::HttpRequestPtr &,
                    std::function<void (const drogon::HttpResponsePtr &)> &&);
                    

private:
    SimpleDynamics dyn_;  
};


