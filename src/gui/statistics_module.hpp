#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace mz::gui {

/**
 * @brief Statistics module for displaying calculation metrics and solver
 * information.
 *
 * Shows comprehensive information about the simulation including:
 * - Calculation timing
 * - Solver statistics (steps, convergence)
 * - Result summary (max/min values, final state)
 * - System information
 */
class StatisticsModule final : public BaseModule
{
public:
  /**
   * @brief Construct a new StatisticsModule object.
   * @param pneumatic_model Shared pointer to the PneumaticModel.
   */
  explicit StatisticsModule(
    std::shared_ptr<model::PneumaticModel> pneumatic_model);

  /**
   * @brief Render the statistics module UI.
   */
  void render() override;

private:
  std::shared_ptr<model::PneumaticModel>
    m_pneumatic_model; ///< Pneumatic model reference

  // Display configuration
  bool m_show_timing{ true };
  bool m_show_solver_info{ true };
  bool m_show_result_summary{ true };
  bool m_show_system_info{ true };
  bool m_auto_refresh{ true };

  // Cached statistics to avoid frequent updates
  model::CalculationStatistics               m_cached_stats;
  std::chrono::steady_clock::time_point      m_last_update;
  static constexpr std::chrono::milliseconds UPDATE_INTERVAL{ 100 };

  /**
   * @brief Render timing information.
   * @param stats Calculation statistics.
   */
  void render_timing_info(const model::CalculationStatistics& stats);
  /**
   * @brief Render solver information.
   * @param stats Calculation statistics.
   */
  void render_solver_info(const model::CalculationStatistics& stats);
  /**
   * @brief Render result summary.
   * @param stats Calculation statistics.
   */
  void render_result_summary(const model::CalculationStatistics& stats);
  /**
   * @brief Render system information.
   */
  void render_system_info();
  /**
   * @brief Render module controls.
   */
  void render_controls();

  /**
   * @brief Update the cached statistics if needed.
   */
  void update_cached_stats();
  /**
   * @brief Check if statistics should be updated.
   * @return True if update is needed, false otherwise.
   */
  [[nodiscard]] bool should_update_stats() const noexcept;
  /**
   * @brief Format a duration as a string.
   * @param duration Duration in milliseconds.
   * @return Formatted string.
   */
  [[nodiscard]] std::string format_duration(
    std::chrono::milliseconds duration) const;
  /**
   * @brief Format a number as a string with given precision.
   * @param value The value to format.
   * @param precision Number of decimal places.
   * @return Formatted string.
   */
  [[nodiscard]] std::string format_number(double value,
                                          int    precision = 6) const;
};

}