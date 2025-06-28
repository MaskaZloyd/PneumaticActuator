#pragma once

#include <Eigen/Dense>
#include <Eigen/LU>
#include <concepts>
#include <cstdint>
#include <functional>
#include <optional>
#include <ranges>
#include <vector>

namespace mz::core::util {

/**
 * @brief Enumeration of available ODE solver types
 * Supports both implicit and explicit numerical integration methods
 */
enum class SolverType : uint16_t
{
  ImplicitEuler = 0,
  ExplicitEuler = 1
};

/**
 * @brief Result structure containing the complete solution trajectory
 *
 * Contains time points and corresponding state vectors for the entire
 * integration interval. Each state vector represents the solution at
 * the corresponding time point.
 */
struct SolveResult
{
  std::vector<double>          time;
  std::vector<Eigen::VectorXd> state;
};

/**
 * @brief Configuration parameters for the ODE solver
 *
 * Encapsulates solver settings including step size, tolerances for
 * implicit methods, and convergence criteria.
 */
struct SolverConfig
{
  double      step_size             = 0.001;
  double      newton_tolerance      = 1e-8;
  std::size_t max_newton_iterations = 50;
  std::size_t max_steps             = 10000;
};

/**
 * @brief Concept defining requirements for ODE function callables
 *
 * An ODE function must be callable with (double, const Eigen::VectorXd&)
 * representing (time, state) and return an Eigen::VectorXd representing
 * the time derivative dy/dt = f(t, y).
 */
template<typename F>
concept OdeFunction = requires(F f, double t, const Eigen::VectorXd& y) {
  { f(t, y) } -> std::convertible_to<Eigen::VectorXd>;
};

/**
 * @brief Concept defining requirements for Jacobian function callables
 *
 * A Jacobian function must be callable with (double, const Eigen::VectorXd&)
 * and return an Eigen::MatrixXd representing ∂f/∂y.
 */
template<typename J>
concept JacobianFunction = requires(J j, double t, const Eigen::VectorXd& y) {
  { j(t, y) } -> std::convertible_to<Eigen::MatrixXd>;
};

/**
 * @brief ODE solver supporting multiple numerical methods
 *
 * Template-based solver that can handle both implicit and explicit methods.
 * Uses Eigen for efficient linear algebra operations.
 *
 * @tparam Method The numerical integration method to use
 */
template<SolverType Method = SolverType::ImplicitEuler>
class OdeSolver
{
public:
  /**
   * @brief Construct solver with configuration parameters
   * @param config Solver configuration including step size and tolerances
   */
  explicit OdeSolver(const SolverConfig& config = {})
    : config_(config)
  {
  }

  /**
   * @brief Get the solver configuration
   * @return The solver configuration
   */
  [[nodiscard]] SolverConfig getConfig() const { return config_; }

  /**
   * @brief Set the solver configuration
   * @param config The solver configuration
   */
  void setConfig(const SolverConfig& config) { config_ = config; }

  /**
   * @brief Solve ODE system with implicit method requiring Jacobian
   *
   * Solves the initial value problem dy/dt = f(t, y) with y(t0) = y0
   * using the specified implicit method. Requires both the ODE function
   * and its Jacobian for Newton iterations.
   *
   * @param ode_func The ODE function f(t, y) returning dy/dt
   * @param jacobian_func The Jacobian function returning ∂f/∂y
   * @param t_span Time interval [t_start, t_end]
   * @param y0 Initial state vector
   * @return Complete solution trajectory
   */
  template<OdeFunction F, JacobianFunction J>
    requires(Method == SolverType::ImplicitEuler)
  [[nodiscard]] SolveResult solve(F&& ode_func,
                                  J&& jacobian_func,
                                  const std::pair<double, double>& t_span,
                                  const Eigen::VectorXd&           y0)
  {
    return solve_implicit_euler(
      std::forward<F>(ode_func), std::forward<J>(jacobian_func), t_span, y0);
  }

  /**
   * @brief Solve ODE system with explicit method
   *
   * Solves the initial value problem using explicit methods that don't
   * require Jacobian information. More efficient per step but potentially
   * less stable than implicit methods.
   *
   * @param ode_func The ODE function f(t, y) returning dy/dt
   * @param t_span Time interval [t_start, t_end]
   * @param y0 Initial state vector
   * @return Complete solution trajectory
   */
  template<OdeFunction F>
    requires(Method == SolverType::ExplicitEuler)
  [[nodiscard]] SolveResult solve(F&&                              ode_func,
                                  const std::pair<double, double>& t_span,
                                  const Eigen::VectorXd&           y0)
  {
    return solve_explicit_euler(std::forward<F>(ode_func), t_span, y0);
  }

private:
  SolverConfig config_;

  /**
   * @brief Implementation of implicit Euler method
   *
   * Uses Newton's method to solve the nonlinear system:
   * y_{n+1} - y_n - h*f(t_{n+1}, y_{n+1}) = 0
   *
   * The Newton iteration is:
   * y^{k+1} = y^k - [I - h*J(t_{n+1}, y^k)]^{-1} * G(y^k)
   * where G(y) = y - y_n - h*f(t_{n+1}, y)
   */
  template<OdeFunction F, JacobianFunction J>
  [[nodiscard]] SolveResult solve_implicit_euler(
    F&&                              ode_func,
    J&&                              jacobian_func,
    const std::pair<double, double>& t_span,
    const Eigen::VectorXd&           y0)
  {
    const auto [t_start, t_end] = t_span;
    const double      h         = config_.step_size;
    const std::size_t n_steps = static_cast<std::size_t>((t_end - t_start) / h);

    SolveResult result;
    result.time.reserve(n_steps + 1);
    result.state.reserve(n_steps + 1);

    // Initial conditions
    result.time.push_back(t_start);
    result.state.push_back(y0);

    Eigen::VectorXd y_current = y0;
    double          t_current = t_start;

    for (std::size_t step = 0; step < n_steps && step < config_.max_steps;
         ++step) {
      const double t_next = t_current + h;

      // Newton iteration for implicit Euler step
      Eigen::VectorXd y_next = newton_solve(
        [&](const Eigen::VectorXd& y) {
          // G(y) = y - y_current - h*f(t_next, y)
          return y - y_current - h * ode_func(t_next, y);
        },
        [&](const Eigen::VectorXd& y) {
          // J_G(y) = I - h*J_f(t_next, y)
          const auto identity = Eigen::MatrixXd::Identity(y.size(), y.size());
          return identity - h * jacobian_func(t_next, y);
        },
        y_current +
          h * ode_func(t_current, y_current) // Initial guess (explicit Euler)
      );

      result.time.push_back(t_next);
      result.state.push_back(y_next);

      y_current = std::move(y_next);
      t_current = t_next;
    }

    return result;
  }

  /**
   * @brief Implementation of explicit Euler method
   *
   * Simple forward Euler: y_{n+1} = y_n + h*f(t_n, y_n)
   * Fast but conditionally stable.
   */
  template<OdeFunction F>
  [[nodiscard]] SolveResult solve_explicit_euler(
    F&&                              ode_func,
    const std::pair<double, double>& t_span,
    const Eigen::VectorXd&           y0)
  {
    const auto [t_start, t_end] = t_span;
    const double      h         = config_.step_size;
    const std::size_t n_steps = static_cast<std::size_t>((t_end - t_start) / h);

    SolveResult result;
    result.time.reserve(n_steps + 1);
    result.state.reserve(n_steps + 1);

    // Initial conditions
    result.time.push_back(t_start);
    result.state.push_back(y0);

    Eigen::VectorXd y_current = y0;
    double          t_current = t_start;

    for (std::size_t step = 0; step < n_steps && step < config_.max_steps;
         ++step) {
      const double          t_next = t_current + h;
      const Eigen::VectorXd y_next =
        y_current + h * ode_func(t_current, y_current);

      result.time.push_back(t_next);
      result.state.push_back(y_next);

      y_current = std::move(y_next);
      t_current = t_next;
    }

    return result;
  }

  /**
   * @brief Newton's method for solving nonlinear systems
   *
   * Solves G(x) = 0 using Newton iteration:
   * x^{k+1} = x^k - J_G(x^k)^{-1} * G(x^k)
   *
   * Uses Eigen's LU decomposition for efficient linear system solving.
   */
  template<typename ResidualFunc, typename JacobianFunc>
  [[nodiscard]] Eigen::VectorXd newton_solve(
    ResidualFunc&&         residual,
    JacobianFunc&&         jacobian,
    const Eigen::VectorXd& initial_guess)
  {
    Eigen::VectorXd x = initial_guess;

    for (std::size_t iter = 0; iter < config_.max_newton_iterations; ++iter) {
      const Eigen::VectorXd G = residual(x);

      if (G.norm() < config_.newton_tolerance) {
        break;
      }

      const Eigen::MatrixXd J       = jacobian(x);
      const Eigen::VectorXd delta_x = J.lu().solve(-G);

      x += delta_x;
    }

    return x;
  }
};

using ImplicitEulerSolver = OdeSolver<SolverType::ImplicitEuler>;
using ExplicitEulerSolver = OdeSolver<SolverType::ExplicitEuler>;

}