#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "core/util/solver.hpp"

namespace mz::model {
struct PneumaticParameters
{
  float piston_diameter{ 32.0 };
  float rod_diameter{ 11.0 };
  float in_pressure{ 6.0 };
  float out_pressure{ 1.0 };
  float stroke{ 300.0 };
  float mass{ 6.0 };
};

struct CalculationStatistics
{
  std::chrono::milliseconds calculation_time{ 0 };
  std::size_t               steps_taken{ 0 };
  bool                      converged{ false };
  double                    max_position{ 0.0 };
  double                    max_velocity{ 0.0 };
  double                    final_position{ 0.0 };
  double                    final_velocity{ 0.0 };
};

class PneumaticModel final
{
public:
  PneumaticModel();

  void startCalculation();

  void setParameters(const PneumaticParameters& parameters);
  [[nodiscard]] PneumaticParameters      getParameters() const;
  [[nodiscard]] core::util::SolveResult  getCalculationResult() const;
  [[nodiscard]] CalculationStatistics    getCalculationStatistics() const;
  [[nodiscard]] core::util::SolverConfig getSolverConfig() const;
  void setSolverConfig(const core::util::SolverConfig& solver_config);
  void setInitialState(const Eigen::VectorXd& initial_state);
  void setTspan(const std::pair<double, double>& t_span);

  [[nodiscard]] bool isCalculating() const noexcept { return m_is_calculating; }
  [[nodiscard]] bool hasResults() const noexcept { return m_has_results; }

private:
  [[nodiscard]] Eigen::VectorXd pneumatic_ode(
    double                 time,
    const Eigen::VectorXd& state) const;

  void calculateStatistics();

  PneumaticParameters       m_parameters{};
  core::util::SolveResult   m_result{};
  CalculationStatistics     m_statistics{};
  core::util::SolverConfig  m_solver_config{};
  Eigen::VectorXd           m_initial_state{ 0.0, 0.0 };
  std::pair<double, double> m_t_span{ 0.0, 1.0 };

  std::atomic<bool>  m_is_calculating{ false };
  std::atomic<bool>  m_has_results{ false };
  mutable std::mutex m_result_mutex;
};
}