#include <drogon/drogon.h>
#include "controllers/ArmController.h"

int main() {
    drogon::app().addListener("0.0.0.0", 8848);
    drogon::app().run();
    return 0;
}




