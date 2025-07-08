#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace mz::gui {

/**
 * @brief Advanced statistics module for comprehensive simulation analysis
 *
 * Displays detailed information about:
 * - Motion statistics (position, velocity, acceleration)
 * - Thermodynamic analysis (pressures, temperatures, mass flows)
 * - Solver performance (timing, steps, convergence)
 * - System configuration summary
 * - Real-time simulation monitoring
 */
class StatisticsModule final : public BaseModule
{
public:
  /**
   * @brief Construct statistics module
   * @param pneumatic_model Shared pointer to the advanced PneumaticModel
   */
  explicit StatisticsModule(
    std::shared_ptr<model::PneumaticModel> pneumatic_model);

  /**
   * @brief Render the comprehensive statistics UI
   */
  void render() override;

private:
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model; ///< Model reference

  // Display configuration
  bool m_show_motion_stats{ true };
  bool m_show_thermodynamic_stats{ true };
  bool m_show_solver_performance{ true };
  bool m_show_system_config{ false };
  bool m_show_real_time_monitor{ true };
  bool m_auto_refresh{ true };

  // Update control
  std::chrono::steady_clock::time_point      m_last_update;
  static constexpr std::chrono::milliseconds UPDATE_INTERVAL{ 100 };

  // Cached data for performance
  model::SimulationResult     m_cached_result{};
  model::SimulationStatistics m_cached_stats{};
  model::SimulationConfig     m_cached_config{};
  bool                        m_has_cached_data{ false };

  /**
   * @brief Render display control toggles
   */
  void renderDisplayControls();

  /**
   * @brief Render motion statistics section
   */
  void renderMotionStatistics();

  /**
   * @brief Render thermodynamic statistics section
   */
  void renderThermodynamicStatistics();

  /**
   * @brief Render solver performance metrics
   */
  void renderSolverPerformance();

  /**
   * @brief Render system configuration summary
   */
  void renderSystemConfiguration();

  /**
   * @brief Render real-time simulation monitor
   */
  void renderRealTimeMonitor();

  /**
   * @brief Update cached data if needed
   */
  void updateCachedData();

  /**
   * @brief Check if data should be refreshed
   * @return true if refresh is needed
   */
  [[nodiscard]] bool shouldRefreshData() const noexcept;

  /**
   * @brief Format a duration for display
   * @param duration Duration in milliseconds
   * @return Formatted string
   */
  [[nodiscard]] std::string formatDuration(
    std::chrono::milliseconds duration) const;

  /**
   * @brief Format a value with units and appropriate precision
   * @param value Numerical value
   * @param unit Unit string
   * @param precision Decimal places
   * @return Formatted string
   */
  [[nodiscard]] std::string formatValue(double             value,
                                        const std::string& unit,
                                        int                precision = 3) const;

  /**
   * @brief Get solver type description
   * @param config Solver configuration
   * @return Human-readable solver description
   */
  [[nodiscard]] std::string getSolverDescription(
    const solver::SolverConfig& config) const;

  /**
   * @brief Render a statistics table section
   * @param title Section title
   * @param stats Vector of (label, value) pairs
   */
  void renderStatsTable(
    const std::string&                                      title,
    const std::vector<std::pair<std::string, std::string>>& stats);

  /**
   * @brief Create motion statistics data
   * @return Vector of formatted statistics
   */
  [[nodiscard]] std::vector<std::pair<std::string, std::string>>
  createMotionStatsData() const;

  /**
   * @brief Create thermodynamic statistics data
   * @return Vector of formatted statistics
   */
  [[nodiscard]] std::vector<std::pair<std::string, std::string>>
  createThermodynamicStatsData() const;

  /**
   * @brief Create solver performance data
   * @return Vector of formatted statistics
   */
  [[nodiscard]] std::vector<std::pair<std::string, std::string>>
  createSolverPerformanceData() const;
};

} // namespace mz::gui