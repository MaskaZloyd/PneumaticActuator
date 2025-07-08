#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace mz::solver {

/**
 * @brief Enumeration of available ODE solver types
 *
 * Provides runtime selection of different numerical integration methods
 * from Boost.Odeint with varying characteristics for accuracy and performance.
 */
enum class SolverType : std::uint8_t
{
  RungeKutta4      = 0, ///< Classic 4th order Runge-Kutta (fixed step)
  RungeKuttaDopri5 = 1, ///< Dormand-Prince 5th order (adaptive, dense output)
  RungeKuttaCashKarp54 = 2, ///< Cash-Karp 5(4) (adaptive, error control)
  RungeKuttaFehlberg78 = 3, ///< Fehlberg 7(8) (high accuracy, adaptive)
  BulirschStoer = 4, ///< Bulirsch-Stoer (very high accuracy, extrapolation)
  Rosenbrock4   = 5  ///< Rosenbrock 4th order (stiff systems)
};

/**
 * @brief Configuration parameters for ODE solvers
 *
 * Encapsulates solver settings including tolerances, step sizes, and limits.
 * Different solvers may use different subsets of these parameters.
 */
struct SolverConfig
{
  SolverType type{ SolverType::RungeKuttaDopri5 }; ///< Solver algorithm to use
  double     abs_tolerance{ 1e-10 };               ///< Absolute error tolerance
  double     rel_tolerance{ 1e-6 };                ///< Relative error tolerance
  double     initial_step_size{ 0.001 }; ///< Initial/fixed step size [s]
  double max_step_size{ 0.1 }; ///< Maximum step size for adaptive methods [s]
  std::size_t max_steps{ 1000000 }; ///< Maximum number of integration steps
};

/**
 * @brief Complete solution trajectory from ODE integration
 *
 * Contains the full time-state history along with metadata about
 * the integration process and performance metrics.
 */
struct SolveResult
{
  std::vector<double> time_points; ///< Time values at each step
  std::vector<Eigen::VectorXd>
              state_trajectory; ///< State vectors at each time point
  std::size_t steps_taken{ 0 }; ///< Total integration steps performed
  bool converged{ false };      ///< Whether integration completed successfully
  std::chrono::milliseconds solve_time{
    0
  };                       ///< Wall-clock time for integration
  std::string solver_name; ///< Name of solver used

  /// Check if result contains valid data
  [[nodiscard]] bool isValid() const noexcept
  {
    return converged && !time_points.empty() &&
           time_points.size() == state_trajectory.size();
  }

  /// Get final state (empty vector if invalid)
  [[nodiscard]] Eigen::VectorXd getFinalState() const noexcept
  {
    return isValid() ? state_trajectory.back() : Eigen::VectorXd{};
  }
};

/**
 * @brief Type-erased interface for ODE solver strategies
 *
 * Provides runtime polymorphism for different Boost.Odeint solvers
 * while maintaining a consistent interface for the client code.
 */
class ISolverStrategy
{
public:
  virtual ~ISolverStrategy() = default;

  /**
   * @brief Solve an ODE system over a time interval
   *
   * @param system Function f(t, y) -> dy/dt representing the ODE system
   * @param initial_state Initial condition y(t0)
   * @param time_span Integration interval [t_start, t_end]
   * @param config Solver configuration parameters
   * @return Complete solution trajectory and metadata
   */
  [[nodiscard]] virtual SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const = 0;

  /**
   * @brief Get human-readable solver name
   * @return Descriptive name of the solver algorithm
   */
  [[nodiscard]] virtual std::string name() const = 0;

  /**
   * @brief Check if solver supports adaptive step sizing
   * @return true if solver can adjust step size automatically
   */
  [[nodiscard]] virtual bool isAdaptive() const noexcept = 0;

  /**
   * @brief Check if solver supports dense output
   * @return true if solver provides continuous interpolation
   */
  [[nodiscard]] virtual bool hasDenseOutput() const noexcept = 0;
};

/**
 * @brief Factory for creating ODE solver strategy instances
 *
 * Provides centralized creation and metadata about available solvers.
 * Encapsulates the mapping from enum values to concrete implementations.
 */
class SolverFactory
{
public:
  /**
   * @brief Create a solver strategy instance
   * @param type Solver algorithm type
   * @return Unique pointer to solver strategy implementation
   * @throw std::invalid_argument if solver type is not supported
   */
  [[nodiscard]] static std::unique_ptr<ISolverStrategy> createSolver(
    SolverType type);

  /**
   * @brief Get list of all available solver types
   * @return Vector containing all supported solver types
   */
  [[nodiscard]] static std::vector<SolverType> availableSolvers();

  /**
   * @brief Get human-readable name for a solver type
   * @param type Solver type
   * @return Descriptive name string
   */
  [[nodiscard]] static std::string getSolverName(SolverType type);

  /**
   * @brief Get recommended solver for specific use cases
   * @param requires_high_accuracy If true, prefer accuracy over speed
   * @param is_stiff If true, prefer solvers suitable for stiff systems
   * @return Recommended solver type
   */
  [[nodiscard]] static SolverType getRecommendedSolver(
    bool requires_high_accuracy = false,
    bool is_stiff               = false) noexcept;
};

/**
 * @brief Main ODE solver wrapper class
 *
 * Provides a modern C++ interface to Boost.Odeint with runtime solver
 * selection, automatic resource management, and consistent error handling.
 */
class OdeSolverWrapper
{
public:
  /**
   * @brief Construct wrapper with default configuration
   * @param config Initial solver configuration
   */
  explicit OdeSolverWrapper(SolverConfig config = {});

  /**
   * @brief Solve an ODE system
   *
   * @param system Function representing dy/dt = f(t, y)
   * @param initial_state Initial condition y(t0)
   * @param time_span Integration interval [t_start, t_end]
   * @return Complete solution trajectory and metadata
   * @throw std::runtime_error if integration fails
   */
  [[nodiscard]] SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span) const;

  /**
   * @brief Solve with custom configuration (one-time override)
   *
   * @param system ODE system function
   * @param initial_state Initial condition
   * @param time_span Integration interval
   * @param config Configuration to use for this solve only
   * @return Solution trajectory and metadata
   */
  [[nodiscard]] SolveResult solveWithConfig(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const;

  /**
   * @brief Update solver configuration
   * @param config New configuration to use
   */
  void setConfig(const SolverConfig& config);

  /**
   * @brief Get current configuration
   * @return Current solver configuration
   */
  [[nodiscard]] const SolverConfig& getConfig() const noexcept;

  /**
   * @brief Get information about current solver
   * @return Human-readable solver description
   */
  [[nodiscard]] std::string getCurrentSolverInfo() const;

  /**
   * @brief Check if current solver supports adaptive stepping
   * @return true if adaptive step size control is available
   */
  [[nodiscard]] bool isCurrentSolverAdaptive() const;

private:
  SolverConfig m_config; ///< Current solver configuration
  mutable std::unique_ptr<ISolverStrategy>
    m_solver; ///< Current solver instance

  /**
   * @brief Update solver instance if configuration changed
   */
  void updateSolver() const;

  /**
   * @brief Validate configuration parameters
   * @param config Configuration to validate
   * @throw std::invalid_argument if parameters are invalid
   */
  static void validateConfig(const SolverConfig& config);
};

} // namespace mz::solver