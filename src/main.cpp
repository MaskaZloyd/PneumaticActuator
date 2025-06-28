#include <cstdlib>
#include <iostream>
#include <memory>
#include <print>

#include "core/application.hpp"
#include "core/logger.hpp"
#include "gui/chart_module.hpp"
#include "gui/pneumatic_parameters_module.hpp"
#include "gui/statistics_module.hpp"

auto& g_logger = mz::core::Logger::getInstance();

int
main()
{
  try {
    auto& app = mz::core::createApplication(
      "PneumaticActuator - Simulation & Analysis Tool", 1600, 1000);

    // Create the pneumatic model (shared between all modules)
    auto pneumatic_model = std::make_shared<mz::model::PneumaticModel>();

    // Add all GUI modules
    app.addModule(
      std::make_shared<mz::gui::PneumaticParametersModule>(pneumatic_model));
    app.addModule(std::make_shared<mz::gui::ChartModule>(pneumatic_model));
    app.addModule(std::make_shared<mz::gui::StatisticsModule>(pneumatic_model));

    MZ_LOG_INFO("PneumaticActuator application initialized with all modules");
    MZ_LOG_INFO("Ready to simulate pneumatic actuator dynamics");

    return app.run();

  } catch (const mz::platform::WindowManagerException& e) {
    MZ_LOG_ERROR("WindowManager error during application startup");
    return -1;
  } catch (const std::exception& e) {
    MZ_LOG_ERROR("Unexpected error during application startup");
    return -1;
  }
}