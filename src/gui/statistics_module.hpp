#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace mz::gui {

/**
 * @brief Statistics module for displaying calculation metrics and solver
 * information
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
  explicit StatisticsModule(
    std::shared_ptr<model::PneumaticModel> pneumatic_model);

  void render() override;

private:
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model;

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

  void renderTimingInfo(const model::CalculationStatistics& stats);
  void renderSolverInfo(const model::CalculationStatistics& stats);
  void renderResultSummary(const model::CalculationStatistics& stats);
  void renderSystemInfo();
  void renderControls();

  void                      updateCachedStats();
  [[nodiscard]] bool        shouldUpdateStats() const noexcept;
  [[nodiscard]] std::string formatDuration(
    std::chrono::milliseconds duration) const;
  [[nodiscard]] std::string formatNumber(double value, int precision = 6) const;
};

}