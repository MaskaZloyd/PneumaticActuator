#include <cstdlib>
#include <iostream>
#include <memory>
#include <print>

#include "core/application.hpp"
#include "core/logger.hpp"
#include "gui/pneumatic_parameters_module.hpp"

auto& g_logger = mz::core::Logger::getInstance();

int
main()
{
  try {
    auto& app = mz::core::createApplication(
      "PneumaticActuator - GLFW + GLAD + ImGui", 1200, 800);

    app.addModule(std::make_shared<mz::gui::PneumaticParametersModule>());
    return app.run();

  } catch (const mz::platform::WindowManagerException& e) {
    return -1;
  } catch (const std::exception& e) {
    return -1;
  }
}