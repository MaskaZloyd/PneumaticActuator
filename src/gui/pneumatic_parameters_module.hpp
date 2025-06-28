#pragma once

#include "base_module.hpp"
#include "core/util/solver.hpp"
#include "model/pneumatic_model.hpp"
#include <atomic>
#include <chrono>

namespace mz::gui {
class PneumaticParametersModule final : public BaseModule
{
public:
  explicit PneumaticParametersModule(
    std::shared_ptr<model::PneumaticModel> pneumatic_model);

  void render() override;

private:
  model::PneumaticParameters             m_pneumatic_parameters;
  std::shared_ptr<model::PneumaticModel> m_pneumatic_model;
  core::util::SolverConfig               m_solver_config;
  Eigen::VectorXd                        m_initial_state;
  std::pair<double, double>              m_t_span;

  // UI state variables (float for ImGui compatibility)
  float m_step_size_ui;
  float m_initial_position_ui;
  float m_initial_velocity_ui;
  float m_t_start_ui;
  float m_t_end_ui;

  // Calculation state
  std::atomic<bool> m_is_calculating;

  void updateSolverConfigFromUI();
  void updateInitialStateFromUI();
  void updateTSpanFromUI();
  void initializeUIFromConfig();
};
}