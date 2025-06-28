#include "chart_module.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <format>
#include <functional>
#include <imgui.h>
#include <implot.h>

namespace mz::gui {

ChartModule::ChartModule(std::shared_ptr<model::PneumaticModel> pneumatic_model)
  : m_pneumatic_model(pneumatic_model)
{
  // Reserve space for typical data sizes to avoid frequent reallocations
  m_time_data.reserve(10000);
  m_position_data.reserve(10000);
  m_velocity_data.reserve(10000);
}

void
ChartModule::render()
{
  ImGui::Begin("Simulation Results");

  if (!m_pneumatic_model->hasResults()) {
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                       "No simulation results available");
    ImGui::Text("Run a calculation to see results here.");
    ImGui::End();
    return;
  }

  // Check if we need to update plot data
  std::size_t current_hash = calculateResultHash();
  if (current_hash != m_last_result_hash) {
    updatePlotData();
    m_last_result_hash = current_hash;
  }

  if (!hasValidData()) {
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f),
                       "Invalid or empty simulation data");
    ImGui::End();
    return;
  }

  // Render plot controls
  renderPlotControls();

  ImGui::Separator();

  // Create a layout with plots
  const float plot_height = (ImGui::GetContentRegionAvail().y - 20) / 3.0f;

  // Position vs Time plot
  renderPositionTimeChart();

  // Velocity vs Time plot
  renderVelocityTimeChart();

  // Phase Portrait (Velocity vs Position)
  renderPhasePortraitChart();

  ImGui::End();
}

void
ChartModule::updatePlotData()
{
  try {
    auto result = m_pneumatic_model->getCalculationResult();

    if (result.time.empty() || result.state.empty()) {
      MZ_LOG_WARN("Empty calculation results received");
      return;
    }

    // Clear previous data
    m_time_data.clear();
    m_position_data.clear();
    m_velocity_data.clear();

    // Copy data for plotting
    m_time_data = result.time;
    m_position_data.reserve(result.state.size());
    m_velocity_data.reserve(result.state.size());

    for (const auto& state : result.state) {
      m_position_data.push_back(state[0]);
      m_velocity_data.push_back(state[1]);
    }

    MZ_LOG_DEBUG(
      std::format("Updated plot data: {} points", m_time_data.size()));

  } catch (const std::exception& e) {
    MZ_LOG_ERROR(std::format("Failed to update plot data: {}", e.what()));
  }
}

void
ChartModule::renderPositionTimeChart()
{
  if (ImPlot::BeginPlot("Position vs Time", ImVec2(-1, 0))) {

    ImPlot::SetupAxes("Time [s]",
                      "Position [m]",
                      ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    if (m_show_grid) {
      ImPlot::SetupAxisTicks(ImAxis_X1, nullptr, 0);
      ImPlot::SetupAxisTicks(ImAxis_Y1, nullptr, 0);
    }

    ImPlot::SetupLegend(ImPlotLocation_NorthEast,
                        m_show_legend ? 0 : ImPlotLegendFlags_NoButtons);

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.2f, 0.6f, 1.0f, 1.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);

    ImPlot::PlotLine("Position",
                     m_time_data.data(),
                     m_position_data.data(),
                     static_cast<int>(m_time_data.size()));

    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
  }
}

void
ChartModule::renderVelocityTimeChart()
{
  if (ImPlot::BeginPlot("Velocity vs Time", ImVec2(-1, 0))) {

    ImPlot::SetupAxes("Time [s]",
                      "Velocity [m/s]",
                      ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    if (m_show_grid) {
      ImPlot::SetupAxisTicks(ImAxis_X1, nullptr, 0);
      ImPlot::SetupAxisTicks(ImAxis_Y1, nullptr, 0);
    }

    ImPlot::SetupLegend(ImPlotLocation_NorthEast,
                        m_show_legend ? 0 : ImPlotLegendFlags_NoButtons);

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1.0f, 0.4f, 0.2f, 1.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);

    ImPlot::PlotLine("Velocity",
                     m_time_data.data(),
                     m_velocity_data.data(),
                     static_cast<int>(m_time_data.size()));

    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
  }
}

void
ChartModule::renderPhasePortraitChart()
{
  if (ImPlot::BeginPlot("Phase Portrait (Velocity vs Position)",
                        ImVec2(-1, 0))) {

    ImPlot::SetupAxes("Position [m]",
                      "Velocity [m/s]",
                      ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    if (m_show_grid) {
      ImPlot::SetupAxisTicks(ImAxis_X1, nullptr, 0);
      ImPlot::SetupAxisTicks(ImAxis_Y1, nullptr, 0);
    }

    ImPlot::SetupLegend(ImPlotLocation_NorthEast,
                        m_show_legend ? 0 : ImPlotLegendFlags_NoButtons);

    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.2f, 0.8f, 0.2f, 1.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);

    ImPlot::PlotLine("Phase Portrait",
                     m_position_data.data(),
                     m_velocity_data.data(),
                     static_cast<int>(m_position_data.size()));

    // Mark start and end points
    if (!m_position_data.empty()) {
      ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                             ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
      ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                             ImVec4(0.0f, 1.0f, 0.0f, 0.8f));
      ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 8.0f);

      double start_pos[] = { m_position_data.front() };
      double start_vel[] = { m_velocity_data.front() };
      ImPlot::PlotScatter("Start", start_pos, start_vel, 1);

      ImPlot::PopStyleVar();
      ImPlot::PopStyleColor(2);

      ImPlot::PushStyleColor(ImPlotCol_MarkerOutline,
                             ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
      ImPlot::PushStyleColor(ImPlotCol_MarkerFill,
                             ImVec4(1.0f, 0.0f, 0.0f, 0.8f));
      ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 8.0f);

      double end_pos[] = { m_position_data.back() };
      double end_vel[] = { m_velocity_data.back() };
      ImPlot::PlotScatter("End", end_pos, end_vel, 1);

      ImPlot::PopStyleVar();
      ImPlot::PopStyleColor(2);
    }

    ImPlot::PopStyleVar();
    ImPlot::PopStyleColor();

    ImPlot::EndPlot();
  }
}

void
ChartModule::renderPlotControls()
{
  ImGui::Text("Plot Controls:");

  ImGui::SameLine();
  ImGui::Checkbox("Auto Fit", &m_auto_fit);

  ImGui::SameLine();
  ImGui::Checkbox("Show Grid", &m_show_grid);

  ImGui::SameLine();
  ImGui::Checkbox("Show Legend", &m_show_legend);

  ImGui::SameLine();
  if (ImGui::Button("Refresh")) {
    m_last_result_hash = 0; // Force refresh
  }
}

std::size_t
ChartModule::calculateResultHash() const
{
  if (!m_pneumatic_model->hasResults()) {
    return 0;
  }

  try {
    auto result = m_pneumatic_model->getCalculationResult();

    // Simple hash based on data size and some sample values
    std::size_t hash = std::hash<std::size_t>{}(result.time.size());

    if (!result.time.empty()) {
      hash ^= std::hash<double>{}(result.time.front()) << 1;
      hash ^= std::hash<double>{}(result.time.back()) << 2;
    }

    if (!result.state.empty()) {
      hash ^= std::hash<double>{}(result.state.front()[0]) << 3;
      hash ^= std::hash<double>{}(result.state.back()[0]) << 4;
    }

    return hash;

  } catch (const std::exception&) {
    return 0;
  }
}

bool
ChartModule::hasValidData() const noexcept
{
  return !m_time_data.empty() && !m_position_data.empty() &&
         !m_velocity_data.empty() &&
         (m_time_data.size() == m_position_data.size()) &&
         (m_time_data.size() == m_velocity_data.size());
}

}