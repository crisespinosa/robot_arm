# ==============================================================================
# CMakeLists.txt — сборка проекта robot_arm (бэкенд управления UR5e)
#
# Проект: Субоптимальное управление роботом-манипулятором
#         на основе мультимодальных входных данных
#
# Требования:
#   - CMake >= 3.5
#   - C++17 или C++20
#   - Фреймворк Drogon (HTTP-сервер)
#
# Сборка:
#   mkdir build && cd build
#   cmake .. && make -j$(nproc)
#   ./robot_arm
# ==============================================================================

cmake_minimum_required(VERSION 3.5)
project(robot_arm CXX)

# --- Определение стандарта C++ ---
# Автоматически выбираем C++20, если доступны <any>, <string_view>, <coroutine>,
# иначе C++17 (минимальное требование).
include(CheckIncludeFileCXX)

check_include_file_cxx(any HAS_ANY)
check_include_file_cxx(string_view HAS_STRING_VIEW)
check_include_file_cxx(coroutine HAS_COROUTINE)
if (NOT "${CMAKE_CXX_STANDARD}" STREQUAL "")
    # Стандарт задан явно — не переопределяем
elseif (HAS_ANY AND HAS_STRING_VIEW AND HAS_COROUTINE)
    set(CMAKE_CXX_STANDARD 20)
elseif (HAS_ANY AND HAS_STRING_VIEW)
    set(CMAKE_CXX_STANDARD 17)
else ()
    set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Основной исполняемый файл ---
add_executable(${PROJECT_NAME} main.cc)

# --- Подключение Drogon ---
# Используем системную установку Drogon через find_package.
# Для локальной сборки Drogon: раскомментируйте add_subdirectory(drogon)
# и закомментируйте find_package/target_link_libraries ниже.
find_package(Drogon CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Drogon::Drogon)

# --- Проверка минимального стандарта ---
if (CMAKE_CXX_STANDARD LESS 17)
    message(FATAL_ERROR "Требуется C++17 или выше")
elseif (CMAKE_CXX_STANDARD LESS 20)
    message(STATUS "Используется C++17")
else ()
    message(STATUS "Используется C++20")
endif ()

# --- Автоматический сбор исходных файлов ---
aux_source_directory(controllers CTL_SRC)    # Контроллеры (ArmController)
aux_source_directory(filters FILTER_SRC)      # Фильтры Drogon (если есть)
aux_source_directory(plugins PLUGIN_SRC)      # Плагины Drogon (если есть)
aux_source_directory(models MODEL_SRC)        # Модели данных (если есть)

# --- Представления (views) Drogon ---
drogon_create_views(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/views
                    ${CMAKE_CURRENT_BINARY_DIR})

# --- Пути включения заголовочных файлов ---
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/models
        ${CMAKE_CURRENT_SOURCE_DIR}/include    # dynamics.hpp, trajectory.hpp
)

# --- Подключение собранных исходников ---
target_sources(${PROJECT_NAME}
               PRIVATE
               ${SRC_DIR}
               ${CTL_SRC}
               ${FILTER_SRC}
               ${PLUGIN_SRC}
               ${MODEL_SRC})

# --- Тесты ---
add_subdirectory(test)
