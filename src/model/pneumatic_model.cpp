#include "pneumatic_model.hpp"

#include "core/logger.hpp"
#include <algorithm>
#include <cmath>
#include <format>

namespace mz::model {
PneumaticModel::PneumaticModel()
  : m_parameters{}
  , m_result{}
  , m_statistics{}
  , m_initial_state{ (Eigen::VectorXd(2) << 0.0, 0.0).finished() }
  , m_solver_config{}
{
}

void
PneumaticModel::startCalculation()
{
  if (m_is_calculating) {
    MZ_LOG_WARN("Calculation already in progress");
    return;
  }

  m_is_calculating = true;
  m_has_results    = false;

  auto start_time  = std::chrono::high_resolution_clock::now();

  try {
    auto solver   = core::util::ExplicitEulerSolver{ m_solver_config };
    auto ode_func = [this](double time, const Eigen::VectorXd& state) {
      return pneumatic_ode(time, state);
    };

    {
      std::lock_guard<std::mutex> lock(m_result_mutex);
      m_result = solver.solve(ode_func, m_t_span, m_initial_state);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    m_statistics.calculation_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                            start_time);

    calculateStatistics();
    m_has_results = true;

    MZ_LOG_INFO(std::format(
      "Pneumatic calculation completed successfully in {:.3f}s, {} steps",
      m_statistics.calculation_time.count() / 1000.0,
      m_statistics.steps_taken));

  } catch (const std::exception& e) {
    MZ_LOG_ERROR(std::format("Calculation failed: {}", e.what()));
    m_has_results = false;
  }

  m_is_calculating = false;
}

void
PneumaticModel::setParameters(const PneumaticParameters& parameters)
{
  m_parameters = parameters;
}

PneumaticParameters
PneumaticModel::getParameters() const
{
  return m_parameters;
}

core::util::SolveResult
PneumaticModel::getCalculationResult() const
{
  std::lock_guard<std::mutex> lock(m_result_mutex);
  return m_result;
}

CalculationStatistics
PneumaticModel::getCalculationStatistics() const
{
  return m_statistics;
}

core::util::SolverConfig
PneumaticModel::getSolverConfig() const
{
  return m_solver_config;
}

void
PneumaticModel::setSolverConfig(const core::util::SolverConfig& solver_config)
{
  m_solver_config = solver_config;
}

void
PneumaticModel::setInitialState(const Eigen::VectorXd& initial_state)
{
  m_initial_state = initial_state;
}

void
PneumaticModel::setTspan(const std::pair<double, double>& t_span)
{
  m_t_span = t_span;
}

Eigen::VectorXd
PneumaticModel::pneumatic_ode(double time, const Eigen::VectorXd& state) const
{
  Eigen::VectorXd dydt(2);

  constexpr double pi = 3.14159265358979323846;

  // Calculate effective areas (convert mm to m)
  const double piston_area =
    pi * std::pow(m_parameters.piston_diameter / 1000.0, 2) / 4.0;
  const double rod_area =
    pi * std::pow(m_parameters.rod_diameter / 1000.0, 2) / 4.0;
  const double effective_area = piston_area - rod_area;

  // Convert pressures from bar to Pa (1 bar = 100000 Pa)
  const double p_in          = m_parameters.in_pressure * 100000.0;
  const double p_out         = m_parameters.out_pressure * 100000.0;
  const double pressure_diff = p_in - p_out;

  // Friction force (Stribeck model)

  // Calculate net force and acceleration
  const double net_force = pressure_diff * effective_area -
                           calculate_friction_force(state[1]) -
                           calculate_stop_force(state[0], state[1]);
  const double acceleration =
    (m_parameters.mass > 0.0) ? net_force / m_parameters.mass : 0.0;

  dydt[0] = state[1];     // dx/dt = velocity
  dydt[1] = acceleration; // dv/dt = acceleration

  return dydt;
}

double
PneumaticModel::calculate_friction_force(double velocity) const
{
  double F_friction = 0;
  if (velocity > 0.0) {
    F_friction = static_cast<double>(m_friction_parameters.Fc) +
                 (m_friction_parameters.Fs - m_friction_parameters.Fc) *
                   std::exp(-velocity / m_friction_parameters.vs) +
                 m_friction_parameters.B * velocity;
  } else {
    F_friction = static_cast<double>(m_friction_parameters.Fc) +
                 (m_friction_parameters.Fs - m_friction_parameters.Fc) *
                   std::exp(velocity / m_friction_parameters.vs) +
                 m_friction_parameters.B * velocity;
  }
  return F_friction;
}

double
PneumaticModel::calculate_stop_force(double position, double velocity) const
{
  constexpr double mm_to_m = 1.0 / 1000.0;
  const double     x_min   = m_stop_force_parameters.x_min * mm_to_m;
  const double     x_max   = m_stop_force_parameters.x_max * mm_to_m;
  double           F_stop  = 0;
  if (position < x_min) {
    F_stop =
      static_cast<double>(m_stop_force_parameters.k_stop) * (position - x_min) +
      static_cast<double>(m_stop_force_parameters.c_stop) * velocity;
  } else if (position > x_max) {
    F_stop =
      static_cast<double>(m_stop_force_parameters.k_stop) * (position - x_max) +
      m_stop_force_parameters.c_stop * velocity;
  }
  return F_stop;
}

void
PneumaticModel::calculateStatistics()
{
  if (m_result.time.empty() || m_result.state.empty()) {
    return;
  }

  m_statistics.steps_taken = m_result.time.size();
  m_statistics.converged   = true; // For explicit methods, always "converged"

  // Find maximum position and velocity
  m_statistics.max_position = 0.0;
  m_statistics.max_velocity = 0.0;

  for (const auto& state : m_result.state) {
    m_statistics.max_position =
      std::max(m_statistics.max_position, std::abs(state[0]));
    m_statistics.max_velocity =
      std::max(m_statistics.max_velocity, std::abs(state[1]));
  }

  // Final values
  if (!m_result.state.empty()) {
    const auto& final_state     = m_result.state.back();
    m_statistics.final_position = final_state[0];
    m_statistics.final_velocity = final_state[1];
  }
}
}