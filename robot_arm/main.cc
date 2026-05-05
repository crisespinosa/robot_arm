/**
 * @file main.cc
 * @brief Точка входа Drogon-сервера для управления роботом-манипулятором UR5e.
 *
 * Загружает конфигурацию из config.json (в корне проекта) и запускает
 * HTTP-сервер. Все маршруты зарегистрированы декларативно через
 * METHOD_LIST в контроллерах (см. controllers/ArmController.h).
 *
 * Запуск:
 *   cd build && ./robot_arm
 *
 * Сервер по умолчанию слушает порт, указанный в config.json (8848).
 */

#include <drogon/drogon.h>

#include <filesystem>
#include <iostream>

int main() {
    using namespace drogon;

    // Ищем config.json: сначала рядом с исполняемым файлом, затем на уровень
    // выше (типичный сценарий — запуск из build/).
    namespace fs = std::filesystem;
    const std::array<fs::path, 3> candidates = {
        fs::path("config.json"),
        fs::path("../config.json"),
        fs::path("../../config.json"),
    };

    fs::path config_path;
    for (const auto& p : candidates) {
        if (fs::exists(p)) { config_path = p; break; }
    }

    if (config_path.empty()) {
        std::cerr << "[main] config.json не найден (искал: ./, ../, ../../). "
                  << "Запусти бинарник из каталога, где доступен config.json."
                  << std::endl;
        return 1;
    }

    std::cout << "[main] Загружаю конфигурацию: " << config_path << std::endl;
    app().loadConfigFile(config_path.string());

    std::cout << "[main] Запуск HTTP-сервера. Ctrl+C для остановки." << std::endl;
    app().run();

    return 0;
}
