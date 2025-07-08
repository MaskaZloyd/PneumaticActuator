#pragma once

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "friction/lugre_friction.hpp"
#include "physics/pneumatic_physics.hpp"
#include "solver/ode_solver_wrapper.hpp"
#include "stop/stop_force.hpp"

namespace mz::model {

/**
 * @brief Complete simulation configuration combining all subsystem parameters
 */
struct SimulationConfig
{
  physics::ThermodynamicParameters thermo{};   ///< Air properties and constants
  physics::GeometryParameters      geometry{}; ///< Cylinder dimensions and mass
  physics::FluidParameters
    fluid{}; ///< Supply/exhaust pressures and valve characteristics
  friction::LuGreParameters friction{}; ///< Friction model parameters
  stop::StopForceParameters stop{};     ///< Stop force model parameters
  solver::SolverConfig      solver{};   ///< ODE solver settings

  /**
   * @brief Validate all parameters for physical consistency
   * @return true if configuration is valid
   */
  [[nodiscard]] bool isValid() const noexcept;

  /**
   * @brief Get configuration summary as string
   * @return Human-readable description
   */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief Initial conditions for simulation start
 */
struct InitialConditions
{
  double position{ 0.0 };        ///< Initial piston position [m]
  double velocity{ 0.0 };        ///< Initial piston velocity [m/s]
  double pressure1{ 1e5 };       ///< Initial pressure in chamber 1 [Pa]
  double pressure2{ 1e5 };       ///< Initial pressure in chamber 2 [Pa]
  double temperature1{ 293.15 }; ///< Initial temperature in chamber 1 [K]
  double temperature2{ 293.15 }; ///< Initial temperature in chamber 2 [K]

  /**
   * @brief Validate initial conditions
   * @return true if conditions are physically reasonable
   */
  [[nodiscard]] bool isValid() const noexcept;
};

/**
 * @brief Comprehensive simulation results with full state information
 */
struct SimulationResult
{
  // Time series data
  std::vector<double> time_points; ///< Time values [s]
  std::vector<physics::StateVector>
    states; ///< Complete 10-component state history

  // Derived time series (computed from states for convenience)
  std::vector<double> positions;     ///< Piston positions [m]
  std::vector<double> velocities;    ///< Piston velocities [m/s]
  std::vector<double> pressures1;    ///< Chamber 1 pressures [Pa]
  std::vector<double> pressures2;    ///< Chamber 2 pressures [Pa]
  std::vector<double> temperatures1; ///< Chamber 1 temperatures [K]
  std::vector<double> temperatures2; ///< Chamber 2 temperatures [K]
  std::vector<double> masses1;       ///< Chamber 1 air masses [kg]
  std::vector<double> masses2;       ///< Chamber 2 air masses [kg]
  std::vector<std::array<double, 4>>
    valve_openings; ///< Valve opening states [0-1]

  // Performance metrics
  std::size_t integration_steps{ 0 }; ///< Number of integration steps taken
  std::chrono::milliseconds solve_time{ 0 }; ///< Wall clock time for solution
  bool converged{ false }; ///< Whether integration completed successfully
  std::string solver_name; ///< Name of solver used

  /**
   * @brief Check if result contains valid data
   * @return true if result is valid and complete
   */
  [[nodiscard]] bool isValid() const noexcept;

  /**
   * @brief Get final state values
   * @return Final state vector (empty if invalid)
   */
  [[nodiscard]] physics::StateVector getFinalState() const noexcept;

  /**
   * @brief Extract position and velocity trajectory for basic plotting
   * @return Pair of (positions, velocities) vectors
   */
  [[nodiscard]] std::pair<std::vector<double>, std::vector<double>>
  getBasicTrajectory() const noexcept;
};

/**
 * @brief Comprehensive statistics about simulation results
 */
struct SimulationStatistics
{
  // Basic motion statistics
  double max_position{ 0.0 };     ///< Maximum absolute position [m]
  double max_velocity{ 0.0 };     ///< Maximum absolute velocity [m/s]
  double max_acceleration{ 0.0 }; ///< Maximum absolute acceleration [m/s²]
  double final_position{ 0.0 };   ///< Final position [m]
  double final_velocity{ 0.0 };   ///< Final velocity [m/s]

  // Thermodynamic statistics
  double max_pressure1{ 0.0 };    ///< Maximum chamber 1 pressure [Pa]
  double max_pressure2{ 0.0 };    ///< Maximum chamber 2 pressure [Pa]
  double max_temperature1{ 0.0 }; ///< Maximum chamber 1 temperature [K]
  double max_temperature2{ 0.0 }; ///< Maximum chamber 2 temperature [K]
  double total_mass_flow{ 0.0 };  ///< Total mass flow through system [kg]

  // Performance statistics
  std::chrono::milliseconds calculation_time{ 0 };    ///< Total simulation time
  std::size_t               steps_taken{ 0 };         ///< Integration steps
  bool                      converged{ false };       ///< Convergence status
  double                    average_step_size{ 0.0 }; ///< Average step size [s]

  /**
   * @brief Get statistics summary as string
   * @return Human-readable summary
   */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief Simple valve controller that can be configured from GUI parameters
 *
 * Provides basic control strategies:
 * - Constant valve openings
 * - Step input at specified time
 * - Simple pressure regulation
 */
class ConfigurableValveController final : public physics::IValveController
{
public:
  enum class ControlMode
  {
    ConstantOpening,   ///< Fixed valve openings
    StepInput,         ///< Step change at specified time
    PressureRegulation ///< Simple pressure-based control
  };

  /**
   * @brief Construct controller with constant openings
   * @param openings Fixed valve openings [u1, u2, u3, u4]
   */
  explicit ConfigurableValveController(
    std::array<double, 4> openings = { 0.5, 0.0, 0.0, 0.5 });

  /**
   * @brief Configure step input control
   * @param step_time Time to apply step [s]
   * @param before_openings Valve openings before step
   * @param after_openings Valve openings after step
   */
  void configureStepInput(double                step_time,
                          std::array<double, 4> before_openings,
                          std::array<double, 4> after_openings);

  /**
   * @brief Configure pressure regulation control
   * @param target_pressure1 Target pressure for chamber 1 [Pa]
   * @param target_pressure2 Target pressure for chamber 2 [Pa]
   * @param gain Control gain
   */
  void configurePressureRegulation(double target_pressure1,
                                   double target_pressure2,
                                   double gain = 0.1);

  // IValveController interface
  [[nodiscard]] std::array<double, 4> getValveCommands(
    double                      time,
    const physics::StateVector& state) const override;

  void reset() override;

private:
  ControlMode           m_mode{ ControlMode::ConstantOpening };
  std::array<double, 4> m_constant_openings{};

  // Step input parameters
  double                m_step_time{ 0.0 };
  std::array<double, 4> m_before_openings{};
  std::array<double, 4> m_after_openings{};

  // Pressure regulation parameters
  double m_target_pressure1{ 1e5 };
  double m_target_pressure2{ 1e5 };
  double m_control_gain{ 0.1 };
};

/**
 * @brief Advanced pneumatic actuator simulation model
 *
 * Provides complete pneumatic system simulation using:
 * - 10-state thermodynamic model with mass and energy conservation
 * - Advanced LuGre friction model with dynamic bristle effects
 * - Multiple high-accuracy ODE solvers with adaptive stepping
 * - Configurable valve control strategies
 * - Comprehensive result analysis and statistics
 */
class PneumaticModel final
{
public:
  /**
   * @brief Construct model with default configuration
   */
  PneumaticModel();

  /**
   * @brief Destructor ensures clean shutdown
   */
  ~PneumaticModel();

  /**
   * @brief Start simulation with current configuration
   *
   * Runs simulation in background thread. Use isCalculating() and
   * hasResults() to monitor progress.
   */
  void startSimulation();

  /**
   * @brief Stop any running simulation
   */
  void stopSimulation();

  /**
   * @brief Run simulation synchronously with current configuration
   * @return Complete simulation result
   * @throw std::runtime_error if simulation fails
   */
  [[nodiscard]] SimulationResult runSimulation();

  /**
   * @brief Set complete simulation configuration
   * @param config New simulation parameters
   */
  void setConfiguration(const SimulationConfig& config);

  /**
   * @brief Get current simulation configuration
   * @return Current configuration
   */
  [[nodiscard]] const SimulationConfig& getConfiguration() const noexcept;

  /**
   * @brief Set initial conditions for simulation
   * @param conditions Initial state
   */
  void setInitialConditions(const InitialConditions& conditions);

  /**
   * @brief Get current initial conditions
   * @return Initial conditions
   */
  [[nodiscard]] const InitialConditions& getInitialConditions() const noexcept;

  /**
   * @brief Set simulation time span
   * @param t_start Start time [s]
   * @param t_end End time [s]
   */
  void setTimeSpan(double t_start, double t_end);

  /**
   * @brief Get current time span
   * @return Pair of (start_time, end_time)
   */
  [[nodiscard]] std::pair<double, double> getTimeSpan() const noexcept;

  /**
   * @brief Configure valve controller
   * @param controller Shared pointer to valve controller
   */
  void setValveController(
    std::shared_ptr<physics::IValveController> controller);

  /**
   * @brief Get results from last simulation
   * @return Simulation results (empty if no valid results)
   */
  [[nodiscard]] SimulationResult getResults() const;

  /**
   * @brief Get statistics from last simulation
   * @return Simulation statistics
   */
  [[nodiscard]] SimulationStatistics getStatistics() const;

  /**
   * @brief Check if simulation is currently running
   * @return true if calculating, false otherwise
   */
  [[nodiscard]] bool isCalculating() const noexcept;

  /**
   * @brief Check if valid results are available
   * @return true if results available, false otherwise
   */
  [[nodiscard]] bool hasResults() const noexcept;

  /**
   * @brief Get current solver information
   * @return Human-readable solver description
   */
  [[nodiscard]] std::string getSolverInfo() const;

private:
  // Core physics components
  std::unique_ptr<physics::PneumaticPhysics> m_physics;
  std::unique_ptr<friction::LuGreFriction>   m_friction;
  std::unique_ptr<stop::StopForce>           m_stop_force;
  std::unique_ptr<solver::OdeSolverWrapper>  m_solver;
  std::shared_ptr<physics::IValveController> m_valve_controller;

  // Configuration
  SimulationConfig          m_config{};
  InitialConditions         m_initial_conditions{};
  std::pair<double, double> m_time_span{ 0.0, 1.0 };

  // Results and state
  SimulationResult     m_result{};
  SimulationStatistics m_statistics{};
  std::atomic<bool>    m_is_calculating{ false };
  std::atomic<bool>    m_has_results{ false };
  std::atomic<bool>    m_stop_requested{ false };

  // Thread safety
  mutable std::mutex           m_result_mutex;
  std::unique_ptr<std::thread> m_simulation_thread;

  /**
   * @brief Main simulation thread function
   */
  void simulationThreadFunction();

  /**
   * @brief ODE system function for solver
   * @param time Current time [s]
   * @param state Current state vector
   * @return State derivatives
   */
  [[nodiscard]] physics::StateVector odeSytem(
    double                      time,
    const physics::StateVector& state) const;

  /**
   * @brief Process raw solver results into structured format
   * @param solver_result Raw solver output
   */
  void processSolverResults(const solver::SolveResult& solver_result);

  /**
   * @brief Calculate comprehensive statistics from results
   */
  void calculateStatistics();

  /**
   * @brief Validate configuration before simulation
   * @throw std::invalid_argument if configuration is invalid
   */
  void validateConfiguration() const;
};

} // namespace mz::model