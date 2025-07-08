#pragma once

#include "base_module.hpp"
#include "model/pneumatic_model.hpp"
#include <atomic>
#include <chrono>
#include <memory>

namespace mz::gui {

/**
 * @brief Advanced GUI module for configuring pneumatic system simulation
 *
 * Provides comprehensive control over:
 * - Thermodynamic parameters (air properties)
 * - Geometry parameters (cylinder dimensions and mass)
 * - Fluid system parameters (pressures, temperatures, valve characteristics)
 * - Friction model parameters (LuGre friction)
 * - Solver configuration (algorithm, tolerances, step sizes)
 * - Initial conditions (position, velocity, pressures, temperatures)
 * - Valve control strategy
 * - Time span settings
 */
class PneumaticParametersModule final : public BaseModule
{
public:
  /**
   * @brief Construct parameters module
   * @param pneumatic_model Shared pointer to the advanced PneumaticModel
   */
  explicit PneumaticParametersModule(
    std::shared_ptr<model::PneumaticModel> pneumatic_model);

  /**
   * @brief Render the complete parameters configuration UI
   */
  void render() override;

private:
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model;

  // Configuration structures
  model::SimulationConfig   m_config{};
  model::InitialConditions  m_initial_conditions{};
  std::pair<double, double> m_time_span{ 0.0, 1.0 };

  // Valve controller configuration
  model::ConfigurableValveController::ControlMode m_valve_mode{
    model::ConfigurableValveController::ControlMode::ConstantOpening
  };
  std::array<float, 4> m_constant_valve_openings{ 0.5f, 0.0f, 0.0f, 0.5f };
  float                m_step_time{ 0.5f };
  std::array<float, 4> m_before_openings{ 0.0f, 1.0f, 1.0f, 0.0f };
  std::array<float, 4> m_after_openings{ 1.0f, 0.0f, 0.0f, 1.0f };
  float                m_target_pressure1{ 2e5f };
  float                m_target_pressure2{ 1e5f };
  float                m_control_gain{ 0.1f };

  // Simulation state
  std::atomic<bool> m_is_simulating{ false };

  // UI state and helpers
  int  m_solver_type_index{ 1 }; // Default to Dormand-Prince
  bool m_show_advanced_thermodynamics{ false };
  bool m_show_advanced_friction{ false };
  bool m_show_advanced_stop_force{ false };
  bool m_show_advanced_solver{ false };
  bool m_show_valve_details{ false };

  // Quick preset configurations
  enum class SystemPreset
  {
    Default,
    HighPressure,
    LowFriction,
    HighAccuracy,
    FastSimulation
  };
  int m_selected_preset{ static_cast<int>(SystemPreset::Default) };

  /**
   * @brief Render main control buttons (start/stop simulation, reset, presets)
   */
  void renderControlButtons();

  /**
   * @brief Render geometry parameters section
   */
  void renderGeometryParameters();

  /**
   * @brief Render fluid system parameters section
   */
  void renderFluidParameters();

  /**
   * @brief Render thermodynamic parameters section
   */
  void renderThermodynamicParameters();

  /**
   * @brief Render friction model parameters section
   */
  void renderFrictionParameters();

  /**
   * @brief Render stop force model parameters section
   */
  void renderStopForceParameters();

  /**
   * @brief Render solver configuration section
   */
  void renderSolverConfiguration();

  /**
   * @brief Render initial conditions section
   */
  void renderInitialConditions();

  /**
   * @brief Render valve control configuration section
   */
  void renderValveControlConfiguration();

  /**
   * @brief Render time span configuration
   */
  void renderTimeSpanConfiguration();

  /**
   * @brief Apply a system preset configuration
   * @param preset The preset to apply
   */
  void applySystemPreset(SystemPreset preset);

  /**
   * @brief Update model configuration from current UI state
   */
  void updateModelFromUI();

  /**
   * @brief Update UI state from current model configuration
   */
  void updateUIFromModel();

  /**
   * @brief Configure and apply valve controller to model
   */
  void configureValveController();

  /**
   * @brief Validate current configuration and show warnings if needed
   * @return true if configuration is valid
   */
  bool validateConfiguration();

  /**
   * @brief Get solver type name for display
   * @param type Solver type enum value
   * @return Human-readable name
   */
  static const char* getSolverTypeName(int type);

  /**
   * @brief Get preset name for display
   * @param preset Preset enum value
   * @return Human-readable name
   */
  static const char* getPresetName(SystemPreset preset);
};

} // namespace mz::gui