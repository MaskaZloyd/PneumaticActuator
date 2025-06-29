#include "statistics_module.hpp"
#include "core/logger.hpp"

#include <format>
#include <imgui.h>
#include <iomanip>
#include <sstream>

namespace mz::gui {

StatisticsModule::StatisticsModule(
  std::shared_ptr<model::PneumaticModel> pneumatic_model)
  : m_pneumatic_model(pneumatic_model)
  , m_last_update(std::chrono::steady_clock::now())
{
}

void
StatisticsModule::render()
{
  ImGui::Begin("Statistics & Information");

  // Update cached statistics if needed
  if (should_update_stats()) {
    update_cached_stats();
  }

  // Render control buttons
  render_controls();

  ImGui::Separator();

  // Check if we have any results
  if (!m_pneumatic_model->hasResults()) {
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                       "No simulation results available");
    ImGui::Text("Run a calculation to see statistics here.");

    // Still show system info even without results
    if (m_show_system_info) {
      ImGui::Separator();
      render_system_info();
    }

    ImGui::End();
    return;
  }

  // Render statistics sections
  if (m_show_timing) {
    render_timing_info(m_cached_stats);
  }

  if (m_show_solver_info) {
    render_solver_info(m_cached_stats);
  }

  if (m_show_result_summary) {
    render_result_summary(m_cached_stats);
  }

  if (m_show_system_info) {
    render_system_info();
  }

  ImGui::End();
}

void
StatisticsModule::render_timing_info(const model::CalculationStatistics& stats)
{
  if (ImGui::CollapsingHeader("Timing Information",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Indent();

    ImGui::Text("Calculation Time:");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f),
                       "%s",
                       format_duration(stats.calculation_time).c_str());

    // Calculate performance metrics
    if (stats.steps_taken > 0 && stats.calculation_time.count() > 0) {
      double steps_per_ms =
        static_cast<double>(stats.steps_taken) / stats.calculation_time.count();
      double steps_per_sec = steps_per_ms * 1000.0;

      ImGui::Text("Performance:");
      ImGui::SameLine();
      ImGui::TextColored(
        ImVec4(0.6f, 0.6f, 1.0f, 1.0f), "%.0f steps/sec", steps_per_sec);
    }

    // Show calculation status
    ImGui::Text("Status:");
    ImGui::SameLine();
    if (m_pneumatic_model->isCalculating()) {
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Calculating...");
    } else {
      ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.4f, 1.0f), "Completed");
    }

    ImGui::Unindent();
  }
}

void
StatisticsModule::render_solver_info(const model::CalculationStatistics& stats)
{
  if (ImGui::CollapsingHeader("Solver Information",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Indent();

    auto solver_config = m_pneumatic_model->getSolverConfig();

    ImGui::Text("Solver Type:");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.4f, 1.0f), "Explicit Euler");

    ImGui::Text("Step Size:");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 0.6f, 1.0f),
                       "%s s",
                       format_number(solver_config.step_size, 6).c_str());

    ImGui::Text("Steps Taken:");
    ImGui::SameLine();
    ImGui::TextColored(
      ImVec4(0.4f, 0.8f, 0.8f, 1.0f), "%zu", stats.steps_taken);

    ImGui::Text("Convergence:");
    ImGui::SameLine();
    ImGui::TextColored(stats.converged ? ImVec4(0.4f, 0.8f, 0.4f, 1.0f)
                                       : ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                       stats.converged ? "Yes" : "No");

    ImGui::Unindent();
  }
}

void
StatisticsModule::render_result_summary(
  const model::CalculationStatistics& stats)
{
  if (ImGui::CollapsingHeader("Result Summary",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Indent();

    // Position statistics
    ImGui::Text("Position:");
    ImGui::Indent();
    ImGui::Text("Maximum: %s m", format_number(stats.max_position, 4).c_str());
    ImGui::Text("Final:   %s m",
                format_number(stats.final_position, 4).c_str());
    ImGui::Unindent();

    // Velocity statistics
    ImGui::Text("Velocity:");
    ImGui::Indent();
    ImGui::Text("Maximum: %s m/s",
                format_number(stats.max_velocity, 4).c_str());
    ImGui::Text("Final:   %s m/s",
                format_number(stats.final_velocity, 4).c_str());
    ImGui::Unindent();

    // Calculate additional metrics
    auto result = m_pneumatic_model->getCalculationResult();
    if (!result.time.empty()) {
      double simulation_time = result.time.back() - result.time.front();

      ImGui::Text("Simulation:");
      ImGui::Indent();
      ImGui::Text("Duration: %s s", format_number(simulation_time, 3).c_str());
      ImGui::Text("Data Points: %zu", result.time.size());

      if (simulation_time > 0) {
        double avg_frequency =
          static_cast<double>(result.time.size()) / simulation_time;
        ImGui::Text("Avg. Frequency: %s Hz",
                    format_number(avg_frequency, 1).c_str());
      }
      ImGui::Unindent();
    }

    ImGui::Unindent();
  }
}

void
StatisticsModule::render_system_info()
{
  if (ImGui::CollapsingHeader("System Information")) {
    ImGui::Indent();

    auto params = m_pneumatic_model->getParameters();

    ImGui::Text("Pneumatic Parameters:");
    ImGui::Indent();
    ImGui::Text("Piston Ø: %.1f mm", params.piston_diameter);
    ImGui::Text("Rod Ø:    %.1f mm", params.rod_diameter);
    ImGui::Text("Stroke:   %.1f mm", params.stroke);
    ImGui::Text("P_in:     %.1f bar", params.in_pressure);
    ImGui::Text("P_out:    %.1f bar", params.out_pressure);
    ImGui::Text("Mass:     %.1f kg", params.mass);
    ImGui::Unindent();

    // Calculate derived values
    constexpr double pi = 3.14159265358979323846;
    double           piston_area =
      pi * std::pow(params.piston_diameter / 1000.0, 2) / 4.0;
    double rod_area = pi * std::pow(params.rod_diameter / 1000.0, 2) / 4.0;
    double effective_area = piston_area - rod_area;
    double pressure_diff =
      (params.in_pressure - params.out_pressure) * 100000.0; // Convert to Pa
    double theoretical_force = pressure_diff * effective_area;

    ImGui::Text("Calculated Values:");
    ImGui::Indent();
    ImGui::Text("Piston Area:    %s m²", format_number(piston_area, 6).c_str());
    ImGui::Text("Rod Area:       %s m²", format_number(rod_area, 6).c_str());
    ImGui::Text("Effective Area: %s m²",
                format_number(effective_area, 6).c_str());
    ImGui::Text("Pressure Diff:  %s Pa",
                format_number(pressure_diff, 0).c_str());
    ImGui::Text("Theoretical Force: %s N",
                format_number(theoretical_force, 2).c_str());
    if (params.mass > 0) {
      double theoretical_accel = theoretical_force / params.mass;
      ImGui::Text("Theoretical Accel: %s m/s²",
                  format_number(theoretical_accel, 2).c_str());
    }
    ImGui::Unindent();

    ImGui::Unindent();
  }
}

void
StatisticsModule::render_controls()
{
  ImGui::Text("Display Options:");

  ImGui::Checkbox("Timing", &m_show_timing);
  ImGui::SameLine();
  ImGui::Checkbox("Solver", &m_show_solver_info);
  ImGui::SameLine();
  ImGui::Checkbox("Results", &m_show_result_summary);
  ImGui::SameLine();
  ImGui::Checkbox("System", &m_show_system_info);

  ImGui::SameLine();
  ImGui::Checkbox("Auto Refresh", &m_auto_refresh);

  ImGui::SameLine();
  if (ImGui::Button("Refresh Now")) {
    update_cached_stats();
  }
}

void
StatisticsModule::update_cached_stats()
{
  try {
    if (m_pneumatic_model->hasResults()) {
      m_cached_stats = m_pneumatic_model->getCalculationStatistics();
    }
    m_last_update = std::chrono::steady_clock::now();
  } catch (const std::exception& e) {
    MZ_LOG_ERROR(std::format("Failed to update statistics: {}", e.what()));
  }
}

bool
StatisticsModule::should_update_stats() const noexcept
{
  if (!m_auto_refresh) {
    return false;
  }

  auto now = std::chrono::steady_clock::now();
  return (now - m_last_update) >= UPDATE_INTERVAL;
}

std::string
StatisticsModule::format_duration(std::chrono::milliseconds duration) const
{
  auto ms = duration.count();

  if (ms < 1000) {
    return std::format("{} ms", ms);
  } else if (ms < 60000) {
    return std::format("{:.2f} s", ms / 1000.0);
  } else {
    int minutes = static_cast<int>(ms / 60000);
    int seconds = static_cast<int>((ms % 60000) / 1000);
    return std::format("{}m {}s", minutes, seconds);
  }
}

std::string
StatisticsModule::format_number(double value, int precision) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

}