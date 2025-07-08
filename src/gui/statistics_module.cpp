#include "statistics_module.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <format>
#include <imgui.h>

namespace mz::gui {

StatisticsModule::StatisticsModule(
  std::shared_ptr<model::PneumaticModel> pneumatic_model)
  : m_pneumatic_model(pneumatic_model)
  , m_last_update(std::chrono::steady_clock::now())
{
  MZ_LOG_INFO("Initialized advanced statistics module");
}

void
StatisticsModule::render()
{
  ImGui::Begin("Simulation Statistics");

  // Update cached data if needed
  if (m_auto_refresh) {
    updateCachedData();
  }

  renderDisplayControls();
  ImGui::Separator();

  // Show different statistics sections based on user preferences
  if (m_show_real_time_monitor) {
    renderRealTimeMonitor();
    ImGui::Separator();
  }

  if (m_show_motion_stats) {
    renderMotionStatistics();
  }

  if (m_show_thermodynamic_stats) {
    renderThermodynamicStatistics();
  }

  if (m_show_solver_performance) {
    renderSolverPerformance();
  }

  if (m_show_system_config) {
    renderSystemConfiguration();
  }

  ImGui::End();
}

void
StatisticsModule::renderDisplayControls()
{
  ImGui::Text("Display Options:");

  if (ImGui::Button("Refresh Now")) {
    updateCachedData();
  }
  ImGui::SameLine();
  ImGui::Checkbox("Auto Refresh", &m_auto_refresh);

  ImGui::Separator();

  ImGui::Checkbox("Motion Statistics", &m_show_motion_stats);
  ImGui::SameLine();
  ImGui::Checkbox("Thermodynamics", &m_show_thermodynamic_stats);

  ImGui::Checkbox("Solver Performance", &m_show_solver_performance);
  ImGui::SameLine();
  ImGui::Checkbox("System Config", &m_show_system_config);

  ImGui::Checkbox("Real-time Monitor", &m_show_real_time_monitor);
}

void
StatisticsModule::renderRealTimeMonitor()
{
  if (ImGui::CollapsingHeader("Real-time Monitor",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    bool is_simulating = m_pneumatic_model->isCalculating();
    bool has_results   = m_pneumatic_model->hasResults();

    // Simulation status
    ImGui::Text("Simulation Status:");
    ImGui::SameLine();
    if (is_simulating) {
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "RUNNING");
    } else if (has_results) {
      ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "COMPLETED");
    } else {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "IDLE");
    }

    // Basic info
    ImGui::Text("Has Results: %s", has_results ? "Yes" : "No");

    if (has_results && m_has_cached_data) {
      ImGui::Text("Result Status: %s",
                  m_cached_result.converged ? "Converged" : "Failed");
      ImGui::Text("Data Points: %zu", m_cached_result.time_points.size());
      ImGui::Text("Solver Used: %s", m_cached_result.solver_name.c_str());

      if (!m_cached_result.time_points.empty()) {
        double sim_time = m_cached_result.time_points.back() -
                          m_cached_result.time_points.front();
        ImGui::Text("Simulated Time: %.3f s", sim_time);
      }
    }

    // Solver info
    try {
      std::string solver_info = m_pneumatic_model->getSolverInfo();
      ImGui::Text("Current Solver: %s", solver_info.c_str());
    } catch (...) {
      ImGui::Text("Solver Info: Not available");
    }
  }
}

void
StatisticsModule::renderMotionStatistics()
{
  if (ImGui::CollapsingHeader("Motion Statistics",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (m_has_cached_data && m_cached_result.isValid()) {
      auto stats_data = createMotionStatsData();
      renderStatsTable("Motion Analysis", stats_data);
    } else {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No simulation data available");
    }
  }
}

void
StatisticsModule::renderThermodynamicStatistics()
{
  if (ImGui::CollapsingHeader("Thermodynamic Statistics",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (m_has_cached_data && m_cached_result.isValid()) {
      auto stats_data = createThermodynamicStatsData();
      renderStatsTable("Thermodynamic Analysis", stats_data);
    } else {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No simulation data available");
    }
  }
}

void
StatisticsModule::renderSolverPerformance()
{
  if (ImGui::CollapsingHeader("Solver Performance",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (m_has_cached_data) {
      auto stats_data = createSolverPerformanceData();
      renderStatsTable("Performance Metrics", stats_data);
    } else {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No performance data available");
    }
  }
}

void
StatisticsModule::renderSystemConfiguration()
{
  if (ImGui::CollapsingHeader("System Configuration")) {
    if (m_has_cached_data) {
      std::vector<std::pair<std::string, std::string>> config_data;

      // Geometry
      config_data.emplace_back(
        "Chamber 1 Area",
        formatValue(m_cached_config.geometry.A1 * 1e6, "mm²", 1));
      config_data.emplace_back(
        "Chamber 2 Area",
        formatValue(m_cached_config.geometry.A2 * 1e6, "mm²", 1));
      config_data.emplace_back(
        "Stroke Length",
        formatValue(m_cached_config.geometry.L * 1000, "mm", 0));
      config_data.emplace_back(
        "Total Mass", formatValue(m_cached_config.geometry.M, "kg", 1));

      // Fluid system
      config_data.emplace_back(
        "Supply Pressure",
        formatValue(m_cached_config.fluid.p_s * 1e-5, "bar", 1));
      config_data.emplace_back(
        "Atm. Pressure",
        formatValue(m_cached_config.fluid.p_a * 1e-5, "bar", 2));
      config_data.emplace_back(
        "Supply Temp.",
        formatValue(m_cached_config.fluid.T_s - 273.15, "°C", 1));

      // Friction
      config_data.emplace_back(
        "Coulomb Friction", formatValue(m_cached_config.friction.Fc, "N", 1));
      config_data.emplace_back(
        "Static Friction", formatValue(m_cached_config.friction.Fs, "N", 1));
      config_data.emplace_back(
        "Stribeck Velocity",
        formatValue(m_cached_config.friction.vs * 1000, "mm/s", 3));

      // Solver
      config_data.emplace_back("Solver Type",
                               getSolverDescription(m_cached_config.solver));
      config_data.emplace_back(
        "Abs. Tolerance",
        std::format("{:.0e}", m_cached_config.solver.abs_tolerance));
      config_data.emplace_back(
        "Rel. Tolerance",
        std::format("{:.0e}", m_cached_config.solver.rel_tolerance));

      renderStatsTable("Configuration Summary", config_data);
    } else {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No configuration data available");
    }
  }
}

void
StatisticsModule::updateCachedData()
{
  if (!shouldRefreshData()) {
    return;
  }

  try {
    m_cached_config = m_pneumatic_model->getConfiguration();
    m_cached_stats  = m_pneumatic_model->getStatistics();

    if (m_pneumatic_model->hasResults()) {
      m_cached_result   = m_pneumatic_model->getResults();
      m_has_cached_data = true;
    }

    m_last_update = std::chrono::steady_clock::now();
  } catch (const std::exception& e) {
    MZ_LOG_WARN(
      std::format("Failed to update cached statistics: {}", e.what()));
  }
}

bool
StatisticsModule::shouldRefreshData() const noexcept
{
  auto now = std::chrono::steady_clock::now();
  return (now - m_last_update) >= UPDATE_INTERVAL;
}

std::string
StatisticsModule::formatDuration(std::chrono::milliseconds duration) const
{
  auto ms = duration.count();
  if (ms < 1000) {
    return std::format("{} ms", ms);
  } else if (ms < 60000) {
    return std::format("{:.2f} s", ms / 1000.0);
  } else {
    auto minutes = ms / 60000;
    auto seconds = (ms % 60000) / 1000.0;
    return std::format("{}m {:.1f}s", minutes, seconds);
  }
}

std::string
StatisticsModule::formatValue(double             value,
                              const std::string& unit,
                              int                precision) const
{
  return std::format("{:.{}f} {}", value, precision, unit);
}

std::string
StatisticsModule::getSolverDescription(const solver::SolverConfig& config) const
{
  const char* solver_names[] = { "Runge-Kutta 4",  "Dormand-Prince 5",
                                 "Cash-Karp 5(4)", "Fehlberg 7(8)",
                                 "Bulirsch-Stoer", "Rosenbrock 4" };

  int type_index             = static_cast<int>(config.type);
  if (type_index >= 0 && type_index < 6) {
    return solver_names[type_index];
  }
  return "Unknown";
}

void
StatisticsModule::renderStatsTable(
  const std::string&                                      title,
  const std::vector<std::pair<std::string, std::string>>& stats)
{
  if (ImGui::BeginTable(
        title.c_str(), 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn(
      "Parameter", ImGuiTableColumnFlags_WidthFixed, 180.0f);
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();

    for (const auto& [label, value] : stats) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("%s", label.c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%s", value.c_str());
    }

    ImGui::EndTable();
  }
}

std::vector<std::pair<std::string, std::string>>
StatisticsModule::createMotionStatsData() const
{
  std::vector<std::pair<std::string, std::string>> stats;

  if (!m_cached_result.isValid()) {
    return stats;
  }

  // Basic motion statistics
  stats.emplace_back("Max Position",
                     formatValue(m_cached_stats.max_position * 1000, "mm", 2));
  stats.emplace_back(
    "Max Velocity", formatValue(m_cached_stats.max_velocity * 1000, "mm/s", 1));
  stats.emplace_back("Max Acceleration",
                     formatValue(m_cached_stats.max_acceleration, "m/s²", 2));
  stats.emplace_back(
    "Final Position",
    formatValue(m_cached_stats.final_position * 1000, "mm", 2));
  stats.emplace_back(
    "Final Velocity",
    formatValue(m_cached_stats.final_velocity * 1000, "mm/s", 1));

  // Calculate additional motion metrics
  if (!m_cached_result.positions.empty() &&
      !m_cached_result.time_points.empty()) {
    double total_displacement = std::abs(m_cached_result.positions.back() -
                                         m_cached_result.positions.front());
    stats.emplace_back("Total Displacement",
                       formatValue(total_displacement * 1000, "mm", 2));

    double simulation_time =
      m_cached_result.time_points.back() - m_cached_result.time_points.front();
    double avg_velocity = total_displacement / simulation_time;
    stats.emplace_back("Average Velocity",
                       formatValue(avg_velocity * 1000, "mm/s", 1));
  }

  return stats;
}

std::vector<std::pair<std::string, std::string>>
StatisticsModule::createThermodynamicStatsData() const
{
  std::vector<std::pair<std::string, std::string>> stats;

  if (!m_cached_result.isValid()) {
    return stats;
  }

  // Pressure statistics
  stats.emplace_back(
    "Max Pressure 1",
    formatValue(m_cached_stats.max_pressure1 * 1e-5, "bar", 2));
  stats.emplace_back(
    "Max Pressure 2",
    formatValue(m_cached_stats.max_pressure2 * 1e-5, "bar", 2));

  // Temperature statistics
  stats.emplace_back(
    "Max Temperature 1",
    formatValue(m_cached_stats.max_temperature1 - 273.15, "°C", 1));
  stats.emplace_back(
    "Max Temperature 2",
    formatValue(m_cached_stats.max_temperature2 - 273.15, "°C", 1));

  // Mass flow statistics
  stats.emplace_back(
    "Total Mass Flow",
    formatValue(m_cached_stats.total_mass_flow * 1e6, "mg", 3));

  // Calculate additional thermodynamic metrics
  if (!m_cached_result.pressures1.empty() &&
      !m_cached_result.pressures2.empty()) {
    // Pressure ranges
    auto [min_p1, max_p1] = std::minmax_element(
      m_cached_result.pressures1.begin(), m_cached_result.pressures1.end());
    auto [min_p2, max_p2] = std::minmax_element(
      m_cached_result.pressures2.begin(), m_cached_result.pressures2.end());

    double pressure_range_1 = (*max_p1 - *min_p1) * 1e-5;
    double pressure_range_2 = (*max_p2 - *min_p2) * 1e-5;

    stats.emplace_back("Pressure Range 1",
                       formatValue(pressure_range_1, "bar", 2));
    stats.emplace_back("Pressure Range 2",
                       formatValue(pressure_range_2, "bar", 2));

    // Final pressures
    stats.emplace_back(
      "Final Pressure 1",
      formatValue(m_cached_result.pressures1.back() * 1e-5, "bar", 2));
    stats.emplace_back(
      "Final Pressure 2",
      formatValue(m_cached_result.pressures2.back() * 1e-5, "bar", 2));
  }

  if (!m_cached_result.temperatures1.empty() &&
      !m_cached_result.temperatures2.empty()) {
    // Temperature ranges
    auto [min_t1, max_t1] =
      std::minmax_element(m_cached_result.temperatures1.begin(),
                          m_cached_result.temperatures1.end());
    auto [min_t2, max_t2] =
      std::minmax_element(m_cached_result.temperatures2.begin(),
                          m_cached_result.temperatures2.end());

    double temp_range_1 = *max_t1 - *min_t1;
    double temp_range_2 = *max_t2 - *min_t2;

    stats.emplace_back("Temperature Range 1",
                       formatValue(temp_range_1, "K", 1));
    stats.emplace_back("Temperature Range 2",
                       formatValue(temp_range_2, "K", 1));

    // Final temperatures
    stats.emplace_back(
      "Final Temperature 1",
      formatValue(m_cached_result.temperatures1.back() - 273.15, "°C", 1));
    stats.emplace_back(
      "Final Temperature 2",
      formatValue(m_cached_result.temperatures2.back() - 273.15, "°C", 1));
  }

  return stats;
}

std::vector<std::pair<std::string, std::string>>
StatisticsModule::createSolverPerformanceData() const
{
  std::vector<std::pair<std::string, std::string>> stats;

  // Basic performance metrics
  stats.emplace_back("Calculation Time",
                     formatDuration(m_cached_stats.calculation_time));
  stats.emplace_back("Integration Steps",
                     std::format("{}", m_cached_stats.steps_taken));
  stats.emplace_back("Convergence",
                     m_cached_stats.converged ? "Success" : "Failed");
  stats.emplace_back(
    "Average Step Size",
    formatValue(m_cached_stats.average_step_size * 1000, "ms", 3));

  if (m_has_cached_data && m_cached_result.isValid()) {
    // Additional solver metrics
    stats.emplace_back("Solver Algorithm", m_cached_result.solver_name);
    stats.emplace_back("Wall Clock Time",
                       formatDuration(m_cached_result.solve_time));

    if (m_cached_stats.steps_taken > 0 &&
        m_cached_stats.calculation_time.count() > 0) {
      double steps_per_second = m_cached_stats.steps_taken * 1000.0 /
                                m_cached_stats.calculation_time.count();
      stats.emplace_back("Steps per Second",
                         std::format("{:.0f}", steps_per_second));
    }

    if (!m_cached_result.time_points.empty()) {
      double simulated_time = m_cached_result.time_points.back() -
                              m_cached_result.time_points.front();
      double real_time_ratio =
        simulated_time * 1000.0 / m_cached_result.solve_time.count();
      stats.emplace_back("Real-time Ratio",
                         std::format("{:.1f}x", real_time_ratio));
    }
  }

  return stats;
}

}