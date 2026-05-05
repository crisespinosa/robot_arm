#include <drogon/drogon.h>
#include <iostream>
#include "controllers/ArmController.h"

int main() {
    std::cout << "[main] robot_arm backend listening on 0.0.0.0:8848" << std::endl;
    drogon::app().addListener("0.0.0.0", 8848);
    drogon::app().run();
    return 0;
}
