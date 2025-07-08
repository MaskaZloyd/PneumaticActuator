#include "ode_solver_wrapper.hpp"

#include <algorithm>
#include <boost/numeric/odeint.hpp>
#include <iomanip>
#include <sstream>
#include <stdexcept>

// Boost.Odeint namespace alias for convenience
namespace odeint = boost::numeric::odeint;

namespace mz::solver {

// Forward declarations for concrete solver strategies
class RungeKutta4Strategy;
class RungeKuttaDopri5Strategy;
class RungeKuttaCashKarp54Strategy;
class RungeKuttaFehlberg78Strategy;
class BulirschStoerStrategy;
class Rosenbrock4Strategy;

namespace {

/// Convert Eigen::VectorXd to std::vector<double> for Boost.Odeint
/// compatibility
std::vector<double>
eigenToStdVector(const Eigen::VectorXd& eigen_vec)
{
  return std::vector<double>(eigen_vec.data(),
                             eigen_vec.data() + eigen_vec.size());
}

/// Convert std::vector<double> to Eigen::VectorXd
Eigen::VectorXd
stdVectorToEigen(const std::vector<double>& std_vec)
{
  return Eigen::Map<const Eigen::VectorXd>(std_vec.data(), std_vec.size());
}

/// Observer class to collect integration results
class ResultCollector
{
public:
  void operator()(const std::vector<double>& state, double time)
  {
    time_points.push_back(time);
    state_trajectory.push_back(stdVectorToEigen(state));
  }

  std::vector<double>          time_points;
  std::vector<Eigen::VectorXd> state_trajectory;
};

/// Wrapper for system function to work with Boost.Odeint
class SystemWrapper
{
public:
  SystemWrapper(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system)
    : m_system(std::move(system))
  {
  }

  void operator()(const std::vector<double>& state,
                  std::vector<double>&       dydt,
                  double                     time)
  {
    Eigen::VectorXd eigen_state = stdVectorToEigen(state);
    Eigen::VectorXd eigen_dydt  = m_system(time, eigen_state);

    dydt.resize(eigen_dydt.size());
    std::copy(
      eigen_dydt.data(), eigen_dydt.data() + eigen_dydt.size(), dydt.begin());
  }

private:
  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> m_system;
};

} // anonymous namespace

//=============================================================================
// Concrete Solver Strategy Implementations
//=============================================================================

/// Runge-Kutta 4th order (fixed step) strategy
class RungeKutta4Strategy final : public ISolverStrategy
{
public:
  SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const override
  {

    auto start_time = std::chrono::high_resolution_clock::now();

    SystemWrapper       wrapped_system(std::move(system));
    std::vector<double> state = eigenToStdVector(initial_state);
    ResultCollector     observer;

    // Add initial state
    observer(state, time_span.first);

    try {
      odeint::runge_kutta4<std::vector<double>> stepper;
      std::size_t steps = odeint::integrate_const(stepper,
                                                  wrapped_system,
                                                  state,
                                                  time_span.first,
                                                  time_span.second,
                                                  config.initial_step_size,
                                                  std::ref(observer));

      auto end_time     = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.time_points      = std::move(observer.time_points);
      result.state_trajectory = std::move(observer.state_trajectory);
      result.steps_taken      = steps;
      result.converged        = true;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      return result;

    } catch (const std::exception& e) {
      auto end_time = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.steps_taken = 0;
      result.converged   = false;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      throw std::runtime_error("RungeKutta4 integration failed: " +
                               std::string(e.what()));
    }
  }

  std::string name() const override { return "Runge-Kutta 4th Order"; }
  bool        isAdaptive() const noexcept override { return false; }
  bool        hasDenseOutput() const noexcept override { return false; }
};

/// Dormand-Prince 5th order (adaptive, dense output) strategy
class RungeKuttaDopri5Strategy final : public ISolverStrategy
{
public:
  SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const override
  {

    auto start_time = std::chrono::high_resolution_clock::now();

    SystemWrapper       wrapped_system(std::move(system));
    std::vector<double> state = eigenToStdVector(initial_state);
    ResultCollector     observer;

    try {
      auto controlled_stepper = odeint::make_controlled(
        config.abs_tolerance,
        config.rel_tolerance,
        odeint::runge_kutta_dopri5<std::vector<double>>());

      std::size_t steps = odeint::integrate_adaptive(controlled_stepper,
                                                     wrapped_system,
                                                     state,
                                                     time_span.first,
                                                     time_span.second,
                                                     config.initial_step_size,
                                                     observer);

      auto end_time     = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.time_points      = std::move(observer.time_points);
      result.state_trajectory = std::move(observer.state_trajectory);
      result.steps_taken      = steps;
      result.converged        = true;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      return result;

    } catch (const std::exception& e) {
      auto end_time = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.converged  = false;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      throw std::runtime_error("Dopri5 integration failed: " +
                               std::string(e.what()));
    }
  }

  std::string name() const override { return "Dormand-Prince 5th Order"; }
  bool        isAdaptive() const noexcept override { return true; }
  bool        hasDenseOutput() const noexcept override { return true; }
};

/// Cash-Karp 5(4) adaptive strategy
class RungeKuttaCashKarp54Strategy final : public ISolverStrategy
{
public:
  SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const override
  {

    auto start_time = std::chrono::high_resolution_clock::now();

    SystemWrapper       wrapped_system(std::move(system));
    std::vector<double> state = eigenToStdVector(initial_state);
    ResultCollector     observer;

    try {
      auto controlled_stepper = odeint::make_controlled(
        config.abs_tolerance,
        config.rel_tolerance,
        odeint::runge_kutta_cash_karp54<std::vector<double>>());

      std::size_t steps = odeint::integrate_adaptive(controlled_stepper,
                                                     wrapped_system,
                                                     state,
                                                     time_span.first,
                                                     time_span.second,
                                                     config.initial_step_size,
                                                     observer);

      auto end_time     = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.time_points      = std::move(observer.time_points);
      result.state_trajectory = std::move(observer.state_trajectory);
      result.steps_taken      = steps;
      result.converged        = true;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      return result;

    } catch (const std::exception& e) {
      auto end_time = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.converged  = false;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      throw std::runtime_error("Cash-Karp integration failed: " +
                               std::string(e.what()));
    }
  }

  std::string name() const override { return "Cash-Karp 5(4)"; }
  bool        isAdaptive() const noexcept override { return true; }
  bool        hasDenseOutput() const noexcept override { return false; }
};

/// Fehlberg 7(8) high accuracy strategy
class RungeKuttaFehlberg78Strategy final : public ISolverStrategy
{
public:
  SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const override
  {

    auto start_time = std::chrono::high_resolution_clock::now();

    SystemWrapper       wrapped_system(std::move(system));
    std::vector<double> state = eigenToStdVector(initial_state);
    ResultCollector     observer;

    try {
      auto controlled_stepper = odeint::make_controlled(
        config.abs_tolerance,
        config.rel_tolerance,
        odeint::runge_kutta_fehlberg78<std::vector<double>>());

      std::size_t steps = odeint::integrate_adaptive(controlled_stepper,
                                                     wrapped_system,
                                                     state,
                                                     time_span.first,
                                                     time_span.second,
                                                     config.initial_step_size,
                                                     observer);

      auto end_time     = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.time_points      = std::move(observer.time_points);
      result.state_trajectory = std::move(observer.state_trajectory);
      result.steps_taken      = steps;
      result.converged        = true;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      return result;

    } catch (const std::exception& e) {
      auto end_time = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.converged  = false;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      throw std::runtime_error("Fehlberg78 integration failed: " +
                               std::string(e.what()));
    }
  }

  std::string name() const override { return "Fehlberg 7(8)"; }
  bool        isAdaptive() const noexcept override { return true; }
  bool        hasDenseOutput() const noexcept override { return false; }
};

/// Bulirsch-Stoer very high accuracy strategy
class BulirschStoerStrategy final : public ISolverStrategy
{
public:
  SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const override
  {

    auto start_time = std::chrono::high_resolution_clock::now();

    SystemWrapper       wrapped_system(std::move(system));
    std::vector<double> state = eigenToStdVector(initial_state);
    ResultCollector     observer;

    try {
      odeint::bulirsch_stoer<std::vector<double>> stepper(config.abs_tolerance,
                                                          config.rel_tolerance);

      std::size_t steps = odeint::integrate_adaptive(stepper,
                                                     wrapped_system,
                                                     state,
                                                     time_span.first,
                                                     time_span.second,
                                                     config.initial_step_size,
                                                     observer);

      auto end_time     = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.time_points      = std::move(observer.time_points);
      result.state_trajectory = std::move(observer.state_trajectory);
      result.steps_taken      = steps;
      result.converged        = true;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      return result;

    } catch (const std::exception& e) {
      auto end_time = std::chrono::high_resolution_clock::now();

      SolveResult result;
      result.converged  = false;
      result.solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
      result.solver_name = name();

      throw std::runtime_error("Bulirsch-Stoer integration failed: " +
                               std::string(e.what()));
    }
  }

  std::string name() const override { return "Bulirsch-Stoer"; }
  bool        isAdaptive() const noexcept override { return true; }
  bool        hasDenseOutput() const noexcept override { return true; }
};

/// Rosenbrock 4th order for stiff systems
class Rosenbrock4Strategy final : public ISolverStrategy
{
public:
  SolveResult solve(
    std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
    const Eigen::VectorXd&    initial_state,
    std::pair<double, double> time_span,
    const SolverConfig&       config) const override
  {

    // For now, fall back to Dopri5 since Rosenbrock requires Jacobian
    // In a full implementation, this would require Jacobian computation
    RungeKuttaDopri5Strategy fallback;
    auto result = fallback.solve(system, initial_state, time_span, config);
    result.solver_name = name() + " (fallback to Dopri5)";
    return result;
  }

  std::string name() const override { return "Rosenbrock 4th Order"; }
  bool        isAdaptive() const noexcept override { return true; }
  bool        hasDenseOutput() const noexcept override { return true; }
};

//=============================================================================
// SolverFactory Implementation
//=============================================================================

std::unique_ptr<ISolverStrategy>
SolverFactory::createSolver(SolverType type)
{
  switch (type) {
    case SolverType::RungeKutta4:
      return std::make_unique<RungeKutta4Strategy>();
    case SolverType::RungeKuttaDopri5:
      return std::make_unique<RungeKuttaDopri5Strategy>();
    case SolverType::RungeKuttaCashKarp54:
      return std::make_unique<RungeKuttaCashKarp54Strategy>();
    case SolverType::RungeKuttaFehlberg78:
      return std::make_unique<RungeKuttaFehlberg78Strategy>();
    case SolverType::BulirschStoer:
      return std::make_unique<BulirschStoerStrategy>();
    case SolverType::Rosenbrock4:
      return std::make_unique<Rosenbrock4Strategy>();
    default:
      throw std::invalid_argument("Unsupported solver type");
  }
}

std::vector<SolverType>
SolverFactory::availableSolvers()
{
  return { SolverType::RungeKutta4,          SolverType::RungeKuttaDopri5,
           SolverType::RungeKuttaCashKarp54, SolverType::RungeKuttaFehlberg78,
           SolverType::BulirschStoer,        SolverType::Rosenbrock4 };
}

std::string
SolverFactory::getSolverName(SolverType type)
{
  auto solver = createSolver(type);
  return solver->name();
}

SolverType
SolverFactory::getRecommendedSolver(bool requires_high_accuracy,
                                    bool is_stiff) noexcept
{
  if (is_stiff) {
    return SolverType::Rosenbrock4;
  } else if (requires_high_accuracy) {
    return SolverType::BulirschStoer;
  } else {
    return SolverType::RungeKuttaDopri5; // Good default balance
  }
}

//=============================================================================
// OdeSolverWrapper Implementation
//=============================================================================

OdeSolverWrapper::OdeSolverWrapper(SolverConfig config)
  : m_config(std::move(config))
{
  validateConfig(m_config);
}

SolveResult
OdeSolverWrapper::solve(
  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
  const Eigen::VectorXd&                                         initial_state,
  std::pair<double, double> time_span) const
{

  updateSolver();
  return m_solver->solve(std::move(system), initial_state, time_span, m_config);
}

SolveResult
OdeSolverWrapper::solveWithConfig(
  std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> system,
  const Eigen::VectorXd&                                         initial_state,
  std::pair<double, double>                                      time_span,
  const SolverConfig&                                            config) const
{

  validateConfig(config);
  auto temp_solver = SolverFactory::createSolver(config.type);
  return temp_solver->solve(
    std::move(system), initial_state, time_span, config);
}

void
OdeSolverWrapper::setConfig(const SolverConfig& config)
{
  validateConfig(config);
  m_config = config;
  m_solver.reset(); // Force recreation on next use
}

const SolverConfig&
OdeSolverWrapper::getConfig() const noexcept
{
  return m_config;
}

std::string
OdeSolverWrapper::getCurrentSolverInfo() const
{
  updateSolver();

  std::ostringstream oss;
  oss << m_solver->name();
  oss << " (Adaptive: " << (m_solver->isAdaptive() ? "Yes" : "No");
  oss << ", Dense Output: " << (m_solver->hasDenseOutput() ? "Yes" : "No")
      << ")";
  oss << std::scientific << std::setprecision(1);
  oss << " - Tolerances: abs=" << m_config.abs_tolerance
      << ", rel=" << m_config.rel_tolerance;

  return oss.str();
}

bool
OdeSolverWrapper::isCurrentSolverAdaptive() const
{
  updateSolver();
  return m_solver->isAdaptive();
}

void
OdeSolverWrapper::updateSolver() const
{
  if (!m_solver) {
    m_solver = SolverFactory::createSolver(m_config.type);
  }
}

void
OdeSolverWrapper::validateConfig(const SolverConfig& config)
{
  if (config.abs_tolerance <= 0.0) {
    throw std::invalid_argument("Absolute tolerance must be positive");
  }
  if (config.rel_tolerance <= 0.0) {
    throw std::invalid_argument("Relative tolerance must be positive");
  }
  if (config.initial_step_size <= 0.0) {
    throw std::invalid_argument("Initial step size must be positive");
  }
  if (config.max_step_size <= 0.0 ||
      config.max_step_size < config.initial_step_size) {
    throw std::invalid_argument(
      "Maximum step size must be positive and >= initial step size");
  }
  if (config.max_steps == 0) {
    throw std::invalid_argument("Maximum steps must be positive");
  }
}

} // namespace mz::solver