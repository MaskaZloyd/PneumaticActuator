#pragma once

#include "base_module.hpp"
#include "core/util/solver.hpp"
#include "model/pneumatic_model.hpp"
#include <atomic>
#include <chrono>

namespace mz::gui {

/**
 * @brief GUI module for editing pneumatic actuator and solver parameters.
 *
 * Allows the user to configure pneumatic, friction, stop force, and solver
 * parameters, as well as initial state and simulation time span.
 */
class PneumaticParametersModule final : public BaseModule
{
public:
  /**
   * @brief Construct a new PneumaticParametersModule object.
   * @param pneumatic_model Shared pointer to the PneumaticModel.
   */
  explicit PneumaticParametersModule(
    std::shared_ptr<model::PneumaticModel> pneumatic_model);

  /**
   * @brief Render the parameters module UI.
   */
  void render() override;

private:
  model::PneumaticParameters             m_pneumatic_parameters;
  model::FrictionParameters              m_friction_parameters;
  model::StopForceParameters             m_stop_force_parameters;
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model;
  core::util::SolverConfig               m_solver_config;
  Eigen::VectorXd                        m_initial_state;
  std::pair<double, double>              m_t_span;

  float m_step_size_ui;
  float m_initial_position_ui;
  float m_initial_velocity_ui;
  float m_t_start_ui;
  float m_t_end_ui;

  // Calculation state
  std::atomic<bool> m_is_calculating;

  /**
   * @brief Update solver configuration from UI state.
   */
  void update_solver_config_from_ui();
  /**
   * @brief Update initial state from UI state.
   */
  void update_initial_state_from_ui();
  /**
   * @brief Update simulation time span from UI state.
   */
  void update_t_span_from_ui();
  /**
   * @brief Initialize UI state from current configuration.
   */
  void initialize_ui_from_config();
};
}