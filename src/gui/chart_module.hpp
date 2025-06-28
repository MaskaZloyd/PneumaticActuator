#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <memory>
#include <vector>

namespace mz::gui {

/**
 * @brief Chart module for visualizing pneumatic actuator simulation results
 *
 * Displays three main plots:
 * - Position vs Time: y(t)
 * - Velocity vs Time: v(t)
 * - Phase Portrait: v(x) - velocity vs position
 */
class ChartModule final : public BaseModule
{
public:
  explicit ChartModule(std::shared_ptr<model::PneumaticModel> pneumatic_model);

  void render() override;

private:
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model;

  // Cached data for plotting (avoid constant memory allocation)
  std::vector<double> m_time_data;
  std::vector<double> m_position_data;
  std::vector<double> m_velocity_data;

  // Plot configuration
  bool m_auto_fit{ true };
  bool m_show_grid{ true };
  bool m_show_legend{ true };

  // Last result hash to detect changes
  std::size_t m_last_result_hash{ 0 };

  void updatePlotData();
  void renderPositionTimeChart();
  void renderVelocityTimeChart();
  void renderPhasePortraitChart();
  void renderPlotControls();

  [[nodiscard]] std::size_t calculateResultHash() const;
  [[nodiscard]] bool        hasValidData() const noexcept;
};

}