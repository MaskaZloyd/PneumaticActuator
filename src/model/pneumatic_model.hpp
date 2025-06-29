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

/**
 * @brief Parameters for the pneumatic actuator.
 */
struct PneumaticParameters
{
  float piston_diameter{ 32.0 }; ///< Piston diameter in mm
  float rod_diameter{ 11.0 };    ///< Rod diameter in mm
  float in_pressure{ 6.0 };      ///< Input pressure in bar
  float out_pressure{ 1.0 };     ///< Output pressure in bar
  float stroke{ 300.0 };         ///< Stroke length in mm
  float mass{ 6.0 };             ///< Mass in kg
};

/**
 * @brief Parameters for friction modeling.
 */
struct FrictionParameters
{
  float Fc{ 100.0 }; ///< Coulomb friction force
  float Fs{ 100.0 }; ///< Static friction force
  float vs{ 0.01 };  ///< Stribeck velocity
  float B{ 100.0 };  ///< Viscous friction coefficient
};

/**
 * @brief Parameters for stop force modeling.
 */
struct StopForceParameters
{
  float x_min{ 0.0 };   ///< Minimum position in mm
  float x_max{ 300.0 }; ///< Maximum position in mm
  float k_stop{ 1e6 };  ///< Stop spring constant in N/m
  float c_stop{ 1e3 };  ///< Stop damping coefficient in Ns/m
};

/**
 * @brief Statistics about the calculation process and results.
 */
struct CalculationStatistics
{
  std::chrono::milliseconds calculation_time{
    0
  }; ///< Time taken for calculation
  std::size_t steps_taken{ 0 };
  bool        converged{ false };
  double      max_position{ 0.0 };
  double      max_velocity{ 0.0 };
  double      final_position{ 0.0 };
  double      final_velocity{ 0.0 };
};

/**
 * @brief Pneumatic actuator model for simulation and analysis.
 *
 * This class encapsulates the parameters, state, and solver configuration for
 * simulating a pneumatic actuator, including friction and stop force effects.
 */
class PneumaticModel final
{
public:
  /**
   * @brief Construct a new PneumaticModel object.
   */
  PneumaticModel();

  /**
   * @brief Start the calculation/simulation in a separate thread.
   */
  void startCalculation();

  /**
   * @brief Set the pneumatic parameters.
   * @param parameters The pneumatic parameters to use.
   */
  void setParameters(const PneumaticParameters& parameters);

  /**
   * @brief Get the current pneumatic parameters.
   * @return The current pneumatic parameters.
   */
  [[nodiscard]] PneumaticParameters getParameters() const;

  /**
   * @brief Get the result of the last calculation.
   * @return The calculation result (time and state trajectory).
   */
  [[nodiscard]] core::util::SolveResult getCalculationResult() const;

  /**
   * @brief Get statistics about the last calculation.
   * @return Calculation statistics.
   */
  [[nodiscard]] CalculationStatistics getCalculationStatistics() const;

  /**
   * @brief Get the current solver configuration.
   * @return Solver configuration.
   */
  [[nodiscard]] core::util::SolverConfig getSolverConfig() const;

  /**
   * @brief Set the solver configuration.
   * @param solver_config The solver configuration to use.
   */
  void setSolverConfig(const core::util::SolverConfig& solver_config);

  /**
   * @brief Set the initial state for the simulation.
   * @param initial_state Initial state vector (e.g., position and velocity).
   */
  void setInitialState(const Eigen::VectorXd& initial_state);

  /**
   * @brief Set the time span for the simulation.
   * @param t_span Pair of (start time, end time).
   */
  void setTspan(const std::pair<double, double>& t_span);

  /**
   * @brief Check if a calculation is currently running.
   * @return True if calculating, false otherwise.
   */
  [[nodiscard]] bool isCalculating() const noexcept { return m_is_calculating; }

  /**
   * @brief Check if results are available from the last calculation.
   * @return True if results are available, false otherwise.
   */
  [[nodiscard]] bool hasResults() const noexcept { return m_has_results; }

private:
  /**
   * @brief The ODE function representing the pneumatic system dynamics.
   * @param time Current time.
   * @param state Current state vector.
   * @return Time derivative of the state vector.
   */
  [[nodiscard]] Eigen::VectorXd pneumatic_ode(
    double                 time,
    const Eigen::VectorXd& state) const;

  /**
   * @brief Calculate the friction force for a given velocity.
   * @param velocity The velocity.
   * @return The friction force.
   */
  double calculate_friction_force(double velocity) const;

  /**
   * @brief Calculate the stop force for a given position and velocity.
   * @param position The position.
   * @param velocity The velocity.
   * @return The stop force.
   */
  double calculate_stop_force(double position, double velocity) const;

  /**
   * @brief Calculate statistics for the last simulation.
   */
  void calculateStatistics();

  PneumaticParameters       m_parameters{};
  FrictionParameters        m_friction_parameters{};
  StopForceParameters       m_stop_force_parameters{};
  core::util::SolveResult   m_result{};
  CalculationStatistics     m_statistics{};
  core::util::SolverConfig  m_solver_config{};
  Eigen::VectorXd           m_initial_state{ 0.0, 0.0 };
  std::pair<double, double> m_t_span{ 0.0, 1.0 };
  std::atomic<bool>         m_is_calculating{ false };
  std::atomic<bool>         m_has_results{ false };
  mutable std::mutex        m_result_mutex;
};
}