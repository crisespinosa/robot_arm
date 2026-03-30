/**
 * @file test_main.cc
 * @brief Точка входа для модульных тестов на базе фреймворка Drogon Test.
 *
 * Запускает цикл событий Drogon в отдельном потоке, выполняет
 * зарегистрированные тесты и корректно завершает работу.
 *
 * Для добавления тестов: используйте макрос DROGON_TEST(ИмяТеста) { ... }
 */

#define DROGON_TEST_MAIN
#include <drogon/drogon_test.h>
#include <drogon/drogon.h>

/// Базовый тест — проверяет, что фреймворк запускается без ошибок.
DROGON_TEST(BasicTest)
{
    // Здесь можно добавить проверки: например, создать ArmDynamics
    // и убедиться, что inverseDynamics возвращает корректные значения.
}

int main(int argc, char** argv)
{
    using namespace drogon;

    // Промис для синхронизации: ждём, пока цикл событий запустится
    std::promise<void> p1;
    std::future<void> f1 = p1.get_future();

    // Запускаем основной цикл событий Drogon в отдельном потоке
    std::thread thr([&]() {
        // Ставим промис в очередь — он выполнится после старта цикла
        app().getLoop()->queueInLoop([&p1]() { p1.set_value(); });
        app().run();
    });

    // Ждём, пока цикл событий действительно запустится
    f1.get();

    // Выполняем все зарегистрированные тесты
    int status = test::run(argc, argv);

    // Завершаем цикл событий и ждём завершения потока
    app().getLoop()->queueInLoop([]() { app().quit(); });
    thr.join();

    return status;
}
