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
  // Initialize valve data vectors
  m_valve_data.resize(4);
  MZ_LOG_INFO("Initialized advanced chart module");
}

void
ChartModule::render()
{
  ImGui::Begin("Simulation Charts");

  // Update plot data if needed
  if (needsDataUpdate()) {
    updatePlotData();
  }

  renderPlotControls();
  ImGui::Separator();

  // Render different plot sections based on user preferences
  if (m_show_motion_plots) {
    renderMotionPlots();
  }

  if (m_show_pressure_plots) {
    renderPressurePlots();
  }

  if (m_show_temperature_plots) {
    renderTemperaturePlots();
  }

  if (m_show_mass_plots) {
    renderMassPlots();
  }

  if (m_show_valve_plots) {
    renderValvePlots();
  }

  if (m_show_phase_portraits) {
    renderPhasePortraits();
  }

  if (m_show_combined_plots) {
    renderCombinedPlots();
  }

  ImGui::End();
}

void
ChartModule::renderPlotControls()
{
  ImGui::Text("Plot Selection:");

  ImGui::Checkbox("Motion", &m_show_motion_plots);
  ImGui::SameLine();
  ImGui::Checkbox("Pressure", &m_show_pressure_plots);
  ImGui::SameLine();
  ImGui::Checkbox("Temperature", &m_show_temperature_plots);
  ImGui::SameLine();
  ImGui::Checkbox("Mass", &m_show_mass_plots);

  ImGui::Checkbox("Valves", &m_show_valve_plots);
  ImGui::SameLine();
  ImGui::Checkbox("Phase Portraits", &m_show_phase_portraits);
  ImGui::SameLine();
  ImGui::Checkbox("Combined", &m_show_combined_plots);

  ImGui::Separator();
  ImGui::Text("Display Options:");

  ImGui::Checkbox("Auto Fit", &m_auto_fit);
  ImGui::SameLine();
  ImGui::Checkbox("Grid", &m_show_grid);
  ImGui::SameLine();
  ImGui::Checkbox("Legend", &m_show_legend);
  ImGui::SameLine();
  ImGui::Checkbox("Sync Axes", &m_sync_axes);

  ImGui::SliderFloat("Line Width", &m_line_width, 0.5f, 3.0f, "%.1f");

  ImGui::Separator();
  ImGui::Text("Units:");

  ImGui::Checkbox("Metric Units (mm, bar)", &m_use_metric_units);
  ImGui::SameLine();
  ImGui::Checkbox("Celsius", &m_use_celsius);

  if (ImGui::Button("Refresh Data")) {
    updatePlotData();
  }
  ImGui::SameLine();
  ImGui::Text("Data Valid: %s", m_data_valid ? "Yes" : "No");
}

void
ChartModule::renderMotionPlots()
{
  if (ImGui::CollapsingHeader("Motion Plots", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (!m_data_valid || m_time_data.empty()) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No motion data available");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 200);

    // Position vs Time
    if (ImPlot::BeginPlot("Position vs Time", plot_size)) {
      setupPlotStyle(
        "Position vs Time", "Time [s]", "Position [" + getPositionUnit() + "]");

      std::vector<double> pos_display(m_position_data.size());
      std::transform(m_position_data.begin(),
                     m_position_data.end(),
                     pos_display.begin(),
                     [this](double pos) { return convertPosition(pos); });

      ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, m_line_width);
      ImPlot::PlotLine("Position",
                       m_time_data.data(),
                       pos_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::EndPlot();
    }

    // Velocity vs Time
    if (ImPlot::BeginPlot("Velocity vs Time", plot_size)) {
      setupPlotStyle(
        "Velocity vs Time", "Time [s]", "Velocity [" + getVelocityUnit() + "]");

      std::vector<double> vel_display(m_velocity_data.size());
      std::transform(m_velocity_data.begin(),
                     m_velocity_data.end(),
                     vel_display.begin(),
                     [this](double vel) { return convertVelocity(vel); });

      ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, m_line_width);
      ImPlot::PlotLine("Velocity",
                       m_time_data.data(),
                       vel_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::EndPlot();
    }

    // Acceleration vs Time (if available)
    if (!m_acceleration_data.empty()) {
      if (ImPlot::BeginPlot("Acceleration vs Time", plot_size)) {
        setupPlotStyle(
          "Acceleration vs Time", "Time [s]", "Acceleration [m/s²]");

        ImPlot::SetNextLineStyle(IMPLOT_AUTO_COL, m_line_width);
        ImPlot::PlotLine("Acceleration",
                         m_time_data.data(),
                         m_acceleration_data.data(),
                         static_cast<int>(m_time_data.size()));

        ImPlot::EndPlot();
      }
    }
  }
}

void
ChartModule::renderPressurePlots()
{
  if (ImGui::CollapsingHeader("Pressure Plots",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    if (!m_data_valid || m_pressure1_data.empty()) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No pressure data available");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 250);

    // Combined pressure plot
    if (ImPlot::BeginPlot("Chamber Pressures vs Time", plot_size)) {
      setupPlotStyle("Chamber Pressures vs Time",
                     "Time [s]",
                     "Pressure [" + getPressureUnit() + "]");

      std::vector<double> p1_display(m_pressure1_data.size());
      std::vector<double> p2_display(m_pressure2_data.size());

      std::transform(m_pressure1_data.begin(),
                     m_pressure1_data.end(),
                     p1_display.begin(),
                     [this](double p) { return convertPressure(p); });
      std::transform(m_pressure2_data.begin(),
                     m_pressure2_data.end(),
                     p2_display.begin(),
                     [this](double p) { return convertPressure(p); });

      ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), m_line_width);
      ImPlot::PlotLine("Chamber 1",
                       m_time_data.data(),
                       p1_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.2f, 1.0f, 1.0f), m_line_width);
      ImPlot::PlotLine("Chamber 2",
                       m_time_data.data(),
                       p2_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::EndPlot();
    }

    // Pressure difference plot
    if (m_pressure1_data.size() == m_pressure2_data.size() &&
        !m_pressure1_data.empty()) {
      std::vector<double> pressure_diff(m_pressure1_data.size());
      for (std::size_t i = 0; i < m_pressure1_data.size(); ++i) {
        pressure_diff[i] =
          convertPressure(m_pressure1_data[i] - m_pressure2_data[i]);
      }

      if (ImPlot::BeginPlot("Pressure Difference (P1 - P2)", plot_size)) {
        setupPlotStyle("Pressure Difference",
                       "Time [s]",
                       "ΔPressure [" + getPressureUnit() + "]");

        ImPlot::SetNextLineStyle(ImVec4(0.8f, 0.4f, 0.8f, 1.0f), m_line_width);
        ImPlot::PlotLine("P1 - P2",
                         m_time_data.data(),
                         pressure_diff.data(),
                         static_cast<int>(m_time_data.size()));

        ImPlot::EndPlot();
      }
    }
  }
}

void
ChartModule::renderTemperaturePlots()
{
  if (ImGui::CollapsingHeader("Temperature Plots")) {
    if (!m_data_valid || m_temperature1_data.empty()) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No temperature data available");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 200);

    if (ImPlot::BeginPlot("Chamber Temperatures vs Time", plot_size)) {
      setupPlotStyle("Chamber Temperatures vs Time",
                     "Time [s]",
                     "Temperature [" + getTemperatureUnit() + "]");

      std::vector<double> t1_display(m_temperature1_data.size());
      std::vector<double> t2_display(m_temperature2_data.size());

      std::transform(m_temperature1_data.begin(),
                     m_temperature1_data.end(),
                     t1_display.begin(),
                     [this](double t) { return convertTemperature(t); });
      std::transform(m_temperature2_data.begin(),
                     m_temperature2_data.end(),
                     t2_display.begin(),
                     [this](double t) { return convertTemperature(t); });

      ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.4f, 0.0f, 1.0f), m_line_width);
      ImPlot::PlotLine("Chamber 1",
                       m_time_data.data(),
                       t1_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::SetNextLineStyle(ImVec4(0.0f, 0.6f, 1.0f, 1.0f), m_line_width);
      ImPlot::PlotLine("Chamber 2",
                       m_time_data.data(),
                       t2_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::EndPlot();
    }
  }
}

void
ChartModule::renderMassPlots()
{
  if (ImGui::CollapsingHeader("Mass Plots")) {
    if (!m_data_valid || m_mass1_data.empty()) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No mass data available");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 200);

    if (ImPlot::BeginPlot("Air Mass vs Time", plot_size)) {
      setupPlotStyle("Air Mass vs Time", "Time [s]", "Mass [g]");

      // Convert kg to g for display
      std::vector<double> m1_display(m_mass1_data.size());
      std::vector<double> m2_display(m_mass2_data.size());

      std::transform(m_mass1_data.begin(),
                     m_mass1_data.end(),
                     m1_display.begin(),
                     [](double m) { return m * 1000.0; }); // kg to g
      std::transform(m_mass2_data.begin(),
                     m_mass2_data.end(),
                     m2_display.begin(),
                     [](double m) { return m * 1000.0; });

      ImPlot::SetNextLineStyle(ImVec4(0.6f, 1.0f, 0.2f, 1.0f), m_line_width);
      ImPlot::PlotLine("Chamber 1",
                       m_time_data.data(),
                       m1_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), m_line_width);
      ImPlot::PlotLine("Chamber 2",
                       m_time_data.data(),
                       m2_display.data(),
                       static_cast<int>(m_time_data.size()));

      ImPlot::EndPlot();
    }
  }
}

void
ChartModule::renderValvePlots()
{
  if (ImGui::CollapsingHeader("Valve Control Plots")) {
    if (!m_data_valid || m_valve_data.empty() || m_valve_data[0].empty()) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No valve data available");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 200);

    if (ImPlot::BeginPlot("Valve Openings vs Time", plot_size)) {
      setupPlotStyle("Valve Openings vs Time", "Time [s]", "Opening [0-1]");

      const char* valve_names[] = {
        "Inlet 1", "Outlet 1", "Inlet 2", "Outlet 2"
      };
      ImVec4 valve_colors[] = {
        ImVec4(1.0f, 0.0f, 0.0f, 1.0f), // Red
        ImVec4(1.0f, 0.5f, 0.5f, 1.0f), // Light Red
        ImVec4(0.0f, 0.0f, 1.0f, 1.0f), // Blue
        ImVec4(0.5f, 0.5f, 1.0f, 1.0f)  // Light Blue
      };

      for (int i = 0; i < 4; ++i) {
        if (i < static_cast<int>(m_valve_data.size()) &&
            !m_valve_data[i].empty()) {
          ImPlot::SetNextLineStyle(valve_colors[i], m_line_width);
          ImPlot::PlotLine(valve_names[i],
                           m_time_data.data(),
                           m_valve_data[i].data(),
                           static_cast<int>(m_time_data.size()));
        }
      }

      ImPlot::EndPlot();
    }
  }
}

void
ChartModule::renderPhasePortraits()
{
  if (ImGui::CollapsingHeader("Phase Portraits")) {
    if (!m_data_valid || m_position_data.empty() || m_velocity_data.empty()) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No phase portrait data available");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 300);

    // Velocity vs Position phase portrait
    if (ImPlot::BeginPlot("Phase Portrait: Velocity vs Position", plot_size)) {
      setupPlotStyle("Phase Portrait",
                     "Position [" + getPositionUnit() + "]",
                     "Velocity [" + getVelocityUnit() + "]");

      std::vector<double> pos_display(m_position_data.size());
      std::vector<double> vel_display(m_velocity_data.size());

      std::transform(m_position_data.begin(),
                     m_position_data.end(),
                     pos_display.begin(),
                     [this](double pos) { return convertPosition(pos); });
      std::transform(m_velocity_data.begin(),
                     m_velocity_data.end(),
                     vel_display.begin(),
                     [this](double vel) { return convertVelocity(vel); });

      ImPlot::SetNextLineStyle(ImVec4(0.8f, 0.2f, 0.8f, 1.0f), m_line_width);
      ImPlot::PlotLine("Trajectory",
                       pos_display.data(),
                       vel_display.data(),
                       static_cast<int>(pos_display.size()));

      // Mark start and end points
      if (!pos_display.empty()) {
        ImPlot::SetNextMarkerStyle(
          ImPlotMarker_Circle, 6.0f, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
        ImPlot::PlotScatter("Start", &pos_display[0], &vel_display[0], 1);

        ImPlot::SetNextMarkerStyle(
          ImPlotMarker_Square, 6.0f, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
        ImPlot::PlotScatter("End", &pos_display.back(), &vel_display.back(), 1);
      }

      ImPlot::EndPlot();
    }

    // Pressure vs Volume phase portrait (if both chambers have data)
    if (!m_pressure1_data.empty() && !m_position_data.empty()) {
      // Calculate volumes based on position and geometry
      std::vector<double> volume1, volume2;

      // Get geometry from model configuration
      try {
        auto   config = m_pneumatic_model->getConfiguration();
        double A1     = config.geometry.A1;
        double A2     = config.geometry.A2;
        double V1_0   = config.geometry.V1_0;
        double V2_0   = config.geometry.V2_0;
        double L      = config.geometry.L;

        volume1.reserve(m_position_data.size());
        volume2.reserve(m_position_data.size());

        for (double pos : m_position_data) {
          volume1.push_back((V1_0 + A1 * pos) * 1e6); // Convert to ml
          volume2.push_back((V2_0 + A2 * (L - pos)) * 1e6);
        }

        // Plot P-V diagram for chamber 1
        if (ImPlot::BeginPlot("P-V Diagram: Chamber 1", plot_size)) {
          setupPlotStyle("P-V Diagram Chamber 1",
                         "Volume [ml]",
                         "Pressure [" + getPressureUnit() + "]");

          std::vector<double> p1_display(m_pressure1_data.size());
          std::transform(m_pressure1_data.begin(),
                         m_pressure1_data.end(),
                         p1_display.begin(),
                         [this](double p) { return convertPressure(p); });

          ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.2f, 0.2f, 1.0f),
                                   m_line_width);
          ImPlot::PlotLine("P-V Curve",
                           volume1.data(),
                           p1_display.data(),
                           static_cast<int>(volume1.size()));

          ImPlot::EndPlot();
        }

      } catch (...) {
        ImGui::TextColored(
          ImVec4(0.8f, 0.8f, 0.0f, 1.0f),
          "Cannot calculate P-V diagram: configuration unavailable");
      }
    }
  }
}

void
ChartModule::renderCombinedPlots()
{
  if (ImGui::CollapsingHeader("Combined Analysis")) {
    if (!m_data_valid) {
      ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f),
                         "No data available for combined plots");
      return;
    }

    ImVec2 plot_size = ImVec2(-1, 350);

    // Multi-axis plot with position and pressure
    if (!m_position_data.empty() && !m_pressure1_data.empty()) {
      if (ImPlot::BeginPlot("Position and Pressure vs Time", plot_size)) {
        std::string pos_label = "Position [" + getPositionUnit() + "]";
        ImPlot::SetupAxes("Time [s]", pos_label.c_str());
        ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0, INFINITY);

        // Plot position on primary y-axis
        std::vector<double> pos_display(m_position_data.size());
        std::transform(m_position_data.begin(),
                       m_position_data.end(),
                       pos_display.begin(),
                       [this](double pos) { return convertPosition(pos); });

        ImPlot::SetNextLineStyle(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), m_line_width);
        ImPlot::PlotLine("Position",
                         m_time_data.data(),
                         pos_display.data(),
                         static_cast<int>(m_time_data.size()));

        // Setup secondary y-axis for pressure
        std::string pressure_label = "Pressure [" + getPressureUnit() + "]";
        ImPlot::SetupAxis(
          ImAxis_Y2, pressure_label.c_str(), ImPlotAxisFlags_AuxDefault);
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);

        std::vector<double> p1_display(m_pressure1_data.size());
        std::transform(m_pressure1_data.begin(),
                       m_pressure1_data.end(),
                       p1_display.begin(),
                       [this](double p) { return convertPressure(p); });

        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), m_line_width);
        ImPlot::PlotLine("Pressure 1",
                         m_time_data.data(),
                         p1_display.data(),
                         static_cast<int>(m_time_data.size()));

        ImPlot::EndPlot();
      }
    }
  }
}

void
ChartModule::updatePlotData()
{
  if (!m_pneumatic_model->hasResults()) {
    m_data_valid = false;
    return;
  }

  try {
    auto result = m_pneumatic_model->getResults();
    if (!result.isValid()) {
      m_data_valid = false;
      return;
    }

    // Update basic trajectory data
    m_time_data     = result.time_points;
    m_position_data = result.positions;
    m_velocity_data = result.velocities;

    // Calculate acceleration if we have velocity data
    m_acceleration_data.clear();
    if (m_velocity_data.size() > 1 && m_time_data.size() > 1) {
      m_acceleration_data.reserve(m_velocity_data.size());
      m_acceleration_data.push_back(0.0); // First point

      for (std::size_t i = 1; i < m_velocity_data.size(); ++i) {
        double dt = m_time_data[i] - m_time_data[i - 1];
        if (dt > 0) {
          double accel = (m_velocity_data[i] - m_velocity_data[i - 1]) / dt;
          m_acceleration_data.push_back(accel);
        } else {
          m_acceleration_data.push_back(0.0);
        }
      }
    }

    // Update thermodynamic data
    m_pressure1_data    = result.pressures1;
    m_pressure2_data    = result.pressures2;
    m_temperature1_data = result.temperatures1;
    m_temperature2_data = result.temperatures2;
    m_mass1_data        = result.masses1;
    m_mass2_data        = result.masses2;

    // Update valve data
    m_valve_data.clear();
    m_valve_data.resize(4);

    for (const auto& valve_state : result.valve_openings) {
      for (int i = 0; i < 4; ++i) {
        m_valve_data[i].push_back(valve_state[i]);
      }
    }

    m_data_valid       = true;
    m_last_result_hash = calculateResultHash();

    MZ_LOG_INFO(std::format("Updated chart data with {} data points",
                            m_time_data.size()));

  } catch (const std::exception& e) {
    MZ_LOG_ERROR(std::format("Failed to update chart data: {}", e.what()));
    m_data_valid = false;
  }
}

bool
ChartModule::needsDataUpdate() const
{
  if (!m_pneumatic_model->hasResults()) {
    return false;
  }

  auto current_hash = calculateResultHash();
  return current_hash != m_last_result_hash || !m_data_valid;
}

std::size_t
ChartModule::calculateResultHash() const
{
  try {
    auto result = m_pneumatic_model->getResults();
    if (!result.isValid()) {
      return 0;
    }

    std::hash<std::size_t> hasher;
    std::size_t            seed = 0;

    // Hash based on result size and some key values
    seed ^= hasher(result.time_points.size());
    seed ^= hasher(result.integration_steps);

    if (!result.time_points.empty()) {
      seed ^= std::hash<double>{}(result.time_points.back());
    }
    if (!result.positions.empty()) {
      seed ^= std::hash<double>{}(result.positions.back());
    }

    return seed;
  } catch (...) {
    return 0;
  }
}

void
ChartModule::setupPlotStyle(const std::string& title,
                            const std::string& x_label,
                            const std::string& y_label)
{
  ImPlot::SetupAxes(x_label.c_str(), y_label.c_str());

  if (m_auto_fit) {
    ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0, INFINITY);
    ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, -INFINITY, INFINITY);
  }

  if (m_show_grid) {
    ImPlot::SetupAxesLimits(0, 1, 0, 1, ImPlotCond_Once);
  }

  if (m_show_legend) {
    ImPlot::SetupLegend(ImPlotLocation_NorthEast);
  }
}

double
ChartModule::convertPosition(double position_m) const
{
  return m_use_metric_units ? position_m * 1000.0 : position_m; // mm or m
}

double
ChartModule::convertVelocity(double velocity_ms) const
{
  return m_use_metric_units ? velocity_ms * 1000.0 : velocity_ms; // mm/s or m/s
}

double
ChartModule::convertPressure(double pressure_pa) const
{
  return m_use_metric_units ? pressure_pa * 1e-5 : pressure_pa; // bar or Pa
}

double
ChartModule::convertTemperature(double temperature_k) const
{
  return m_use_celsius ? temperature_k - 273.15 : temperature_k; // deg C or K
}

std::string
ChartModule::getPositionUnit() const
{
  return m_use_metric_units ? "mm" : "m";
}

std::string
ChartModule::getVelocityUnit() const
{
  return m_use_metric_units ? "mm/s" : "m/s";
}

std::string
ChartModule::getPressureUnit() const
{
  return m_use_metric_units ? "bar" : "Pa";
}

std::string
ChartModule::getTemperatureUnit() const
{
  return m_use_celsius ? "°C" : "K";
}

}