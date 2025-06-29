#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <memory>
#include <vector>

namespace mz::gui {

/**
 * @brief Chart module for visualizing pneumatic actuator simulation results.
 *
 * Displays three main plots:
 * - Position vs Time: y(t)
 * - Velocity vs Time: v(t)
 * - Phase Portrait: v(x) - velocity vs position
 */
class ChartModule final : public BaseModule
{
public:
  /**
   * @brief Construct a new ChartModule object.
   * @param pneumatic_model Shared pointer to the PneumaticModel.
   */
  explicit ChartModule(std::shared_ptr<model::PneumaticModel> pneumatic_model);

  /**
   * @brief Render the chart module UI.
   */
  void render() override;

private:
  std::shared_ptr<model::PneumaticModel>
    m_pneumatic_model; ///< Pneumatic model reference

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

  /**
   * @brief Update cached plot data from the model.
   */
  void update_plot_data();
  /**
   * @brief Render the position vs time chart.
   */
  void render_position_time_chart();
  /**
   * @brief Render the velocity vs time chart.
   */
  void render_velocity_time_chart();
  /**
   * @brief Render the phase portrait chart (velocity vs position).
   */
  void render_phase_portrait_chart();
  /**
   * @brief Render plot controls UI.
   */
  void render_plot_controls();

  /**
   * @brief Calculate a hash of the current result for change detection.
   * @return Hash value.
   */
  [[nodiscard]] std::size_t calculate_result_hash() const;
  /**
   * @brief Check if cached data is valid.
   * @return True if valid, false otherwise.
   */
  [[nodiscard]] bool has_valid_data() const noexcept;
};

}