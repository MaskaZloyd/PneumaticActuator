cmake_minimum_required(VERSION 4.0 FATAL_ERROR)
project(PneumaticActuator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable vcpkg integration
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/.vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/.vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(implot CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
# find_package(spdlog CONFIG REQUIRED)

add_executable(${PROJECT_NAME} 
src/main.cpp
src/platform/window_manager.cpp
src/core/application.cpp
src/core/logger.cpp
src/gui/pneumatic_parameters_module.cpp
src/gui/chart_module.cpp
src/gui/statistics_module.cpp
src/model/pneumatic_model.cpp
src/model/friction/lugre_friction.cpp
src/model/stop/stop_force.cpp
src/model/physics/pneumatic_physics.cpp
src/solver/ode_solver_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE glfw glad::glad imgui::imgui implot::implot Eigen3::Eigen Boost::boost)
target_include_directories(${PROJECT_NAME} PRIVATE src)
