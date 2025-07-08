#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <memory>
#include <string>
#include <vector>

namespace mz::gui {

/**
 * @brief Advanced chart module for comprehensive visualization
 *
 * Provides multiple visualization modes for the pneumatic simulation:
 * - Motion plots (position, velocity, acceleration vs time)
 * - Thermodynamic plots (pressure, temperature, mass vs time)
 * - Valve control plots (valve openings vs time)
 * - Phase portraits (velocity vs position, pressure vs volume)
 * - 3D visualizations (optional advanced plots)
 */
class ChartModule final : public BaseModule
{
public:
  /**
   * @brief Construct chart module
   * @param pneumatic_model Shared pointer to the advanced PneumaticModel
   */
  explicit ChartModule(std::shared_ptr<model::PneumaticModel> pneumatic_model);

  /**
   * @brief Render the comprehensive chart visualization UI
   */
  void render() override;

private:
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model; ///< Model reference

  // Plot data cache (extracted from simulation results)
  std::vector<double>              m_time_data;
  std::vector<double>              m_position_data;
  std::vector<double>              m_velocity_data;
  std::vector<double>              m_acceleration_data;
  std::vector<double>              m_pressure1_data;
  std::vector<double>              m_pressure2_data;
  std::vector<double>              m_temperature1_data;
  std::vector<double>              m_temperature2_data;
  std::vector<double>              m_mass1_data;
  std::vector<double>              m_mass2_data;
  std::vector<std::vector<double>> m_valve_data; // 4 valves

  // Plot configuration
  bool m_show_motion_plots{ true };
  bool m_show_pressure_plots{ true };
  bool m_show_temperature_plots{ false };
  bool m_show_mass_plots{ false };
  bool m_show_valve_plots{ false };
  bool m_show_phase_portraits{ false };
  bool m_show_combined_plots{ false };

  // Display options
  bool  m_auto_fit{ true };
  bool  m_show_grid{ true };
  bool  m_show_legend{ true };
  bool  m_sync_axes{ true };
  float m_line_width{ 1.5f };

  // Unit conversion options
  bool m_use_metric_units{ true }; // mm/s vs m/s, bar vs Pa, etc.
  bool m_use_celsius{ true };      // °C vs K

  // Data management
  std::size_t m_last_result_hash{ 0 };
  bool        m_data_valid{ false };

  /**
   * @brief Render plot control panel
   */
  void renderPlotControls();

  /**
   * @brief Render motion plots section
   */
  void renderMotionPlots();

  /**
   * @brief Render pressure plots section
   */
  void renderPressurePlots();

  /**
   * @brief Render temperature plots section
   */
  void renderTemperaturePlots();

  /**
   * @brief Render mass plots section
   */
  void renderMassPlots();

  /**
   * @brief Render valve control plots section
   */
  void renderValvePlots();

  /**
   * @brief Render phase portrait plots section
   */
  void renderPhasePortraits();

  /**
   * @brief Render combined multi-variable plots
   */
  void renderCombinedPlots();

  /**
   * @brief Update plot data from simulation results
   */
  void updatePlotData();

  /**
   * @brief Check if plot data needs updating
   * @return true if update is needed
   */
  [[nodiscard]] bool needsDataUpdate() const;

  /**
   * @brief Calculate hash of current simulation results
   * @return Hash value for change detection
   */
  [[nodiscard]] std::size_t calculateResultHash() const;

  /**
   * @brief Setup ImPlot style for current plot
   * @param title Plot title
   * @param x_label X-axis label
   * @param y_label Y-axis label
   */
  void setupPlotStyle(const std::string& title,
                      const std::string& x_label,
                      const std::string& y_label);

  /**
   * @brief Convert position to display units
   * @param position_m Position in meters
   * @return Position in display units (mm or m)
   */
  [[nodiscard]] double convertPosition(double position_m) const;

  /**
   * @brief Convert velocity to display units
   * @param velocity_ms Velocity in m/s
   * @return Velocity in display units (mm/s or m/s)
   */
  [[nodiscard]] double convertVelocity(double velocity_ms) const;

  /**
   * @brief Convert pressure to display units
   * @param pressure_pa Pressure in Pa
   * @return Pressure in display units (bar or Pa)
   */
  [[nodiscard]] double convertPressure(double pressure_pa) const;

  /**
   * @brief Convert temperature to display units
   * @param temperature_k Temperature in K
   * @return Temperature in display units (°C or K)
   */
  [[nodiscard]] double convertTemperature(double temperature_k) const;

  /**
   * @brief Get position unit string
   * @return Unit string for current position display mode
   */
  [[nodiscard]] std::string getPositionUnit() const;

  /**
   * @brief Get velocity unit string
   * @return Unit string for current velocity display mode
   */
  [[nodiscard]] std::string getVelocityUnit() const;

  /**
   * @brief Get pressure unit string
   * @return Unit string for current pressure display mode
   */
  [[nodiscard]] std::string getPressureUnit() const;

  /**
   * @brief Get temperature unit string
   * @return Unit string for current temperature display mode
   */
  [[nodiscard]] std::string getTemperatureUnit() const;
};

}