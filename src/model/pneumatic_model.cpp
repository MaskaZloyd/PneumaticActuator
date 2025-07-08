#include "pneumatic_model.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>

namespace mz::model {

// SimulationConfig implementation
bool
SimulationConfig::isValid() const noexcept
{
  return thermo.isValid() && geometry.isValid() && fluid.isValid() &&
         friction.isValid() && stop.isValid();
}

std::string
SimulationConfig::toString() const
{
  return std::format(
    "SimulationConfig:\n"
    "  Geometry: A1={:.2e} m², A2={:.2e} m², L={:.3f} m, M={:.1f} kg\n"
    "  Fluid: ps={:.0f} Pa, pa={:.0f} Pa, Ts={:.1f} K, Ta={:.1f} K\n"
    "  Friction: Fc={:.1f} N, Fs={:.1f} N, vs={:.4f} m/s\n"
    "  Stop Force: k={:.1e} N/m, c={:.1e} Ns/m, limits=[{:.3f}, {:.3f}] m\n"
    "  Solver: {}, tol={:.1e}",
    geometry.A1,
    geometry.A2,
    geometry.L,
    geometry.M,
    fluid.p_s,
    fluid.p_a,
    fluid.T_s,
    fluid.T_a,
    friction.Fc,
    friction.Fs,
    friction.vs,
    stop.k_stop,
    stop.c_stop,
    stop.x_min,
    stop.x_max,
    static_cast<int>(solver.type),
    solver.abs_tolerance);
}

// InitialConditions implementation
bool
InitialConditions::isValid() const noexcept
{
  return std::isfinite(position) && std::isfinite(velocity) &&
         pressure1 > 0.0 && pressure2 > 0.0 && temperature1 > 0.0 &&
         temperature2 > 0.0;
}

// SimulationResult implementation
bool
SimulationResult::isValid() const noexcept
{
  return converged && !time_points.empty() &&
         time_points.size() == states.size() &&
         time_points.size() == positions.size();
}

physics::StateVector
SimulationResult::getFinalState() const noexcept
{
  return isValid() ? states.back() : physics::StateVector{};
}

std::pair<std::vector<double>, std::vector<double>>
SimulationResult::getBasicTrajectory() const noexcept
{
  return { positions, velocities };
}

// SimulationStatistics implementation
std::string
SimulationStatistics::toString() const
{
  return std::format(
    "Simulation completed in {:.3f}s ({} steps, avg step {:.2e}s)\n"
    "Motion: max pos={:.4f}m, max vel={:.3f}m/s, final pos={:.4f}m\n"
    "Thermodynamics: max p1={:.0f}Pa, max p2={:.0f}Pa, max T1={:.1f}K, max "
    "T2={:.1f}K\n"
    "Total mass flow: {:.6f}kg",
    calculation_time.count() / 1000.0,
    steps_taken,
    average_step_size,
    max_position,
    max_velocity,
    final_position,
    max_pressure1,
    max_pressure2,
    max_temperature1,
    max_temperature2,
    total_mass_flow);
}

// ConfigurableValveController implementation
ConfigurableValveController::ConfigurableValveController(
  std::array<double, 4> openings)
  : m_constant_openings(openings)
{
  // Clamp values to [0, 1]
  for (auto& opening : m_constant_openings) {
    opening = std::clamp(opening, 0.0, 1.0);
  }
}

void
ConfigurableValveController::configureStepInput(
  double                step_time,
  std::array<double, 4> before_openings,
  std::array<double, 4> after_openings)
{
  m_mode            = ControlMode::StepInput;
  m_step_time       = step_time;
  m_before_openings = before_openings;
  m_after_openings  = after_openings;

  // Clamp all values
  for (auto& opening : m_before_openings) {
    opening = std::clamp(opening, 0.0, 1.0);
  }
  for (auto& opening : m_after_openings) {
    opening = std::clamp(opening, 0.0, 1.0);
  }
}

void
ConfigurableValveController::configurePressureRegulation(
  double target_pressure1,
  double target_pressure2,
  double gain)
{
  m_mode             = ControlMode::PressureRegulation;
  m_target_pressure1 = target_pressure1;
  m_target_pressure2 = target_pressure2;
  m_control_gain     = std::clamp(gain, 0.0, 1.0);
}

std::array<double, 4>
ConfigurableValveController::getValveCommands(
  double                      time,
  const physics::StateVector& state) const
{
  using physics::StateIndex;

  switch (m_mode) {
    case ControlMode::ConstantOpening:
      return m_constant_openings;

    case ControlMode::StepInput:
      return (time < m_step_time) ? m_before_openings : m_after_openings;

    case ControlMode::PressureRegulation: {
      // Extract masses and temperatures from state
      double m1 = state[static_cast<std::size_t>(StateIndex::Mass1)];
      double T1 = state[static_cast<std::size_t>(StateIndex::Temperature1)];
      double m2 = state[static_cast<std::size_t>(StateIndex::Mass2)];
      double T2 = state[static_cast<std::size_t>(StateIndex::Temperature2)];
      double x  = state[static_cast<std::size_t>(StateIndex::Position)];

      // Simple pressure calculation (would need geometry params for proper
      // calculation) This is simplified - in practice would access physics
      // object
      constexpr double R       = 287.0; // Specific gas constant
      constexpr double V1_base = 1e-5, A1 = 0.000804;
      constexpr double V2_base = 1e-5, A2 = 0.000690, L = 0.35;

      double V1 = V1_base + A1 * x;
      double V2 = V2_base + A2 * (L - x);
      double p1 = m1 * R * T1 / V1;
      double p2 = m2 * R * T2 / V2;

      // Simple proportional control
      double error1 = m_target_pressure1 - p1;
      double error2 = m_target_pressure2 - p2;

      std::array<double, 4> commands{};

      // Valve 1: inlet to chamber 1
      commands[0] =
        std::clamp(0.5 + m_control_gain * error1 / 10000.0, 0.0, 1.0);
      // Valve 2: outlet from chamber 1
      commands[1] =
        std::clamp(0.5 - m_control_gain * error1 / 10000.0, 0.0, 1.0);
      // Valve 3: inlet to chamber 2
      commands[2] =
        std::clamp(0.5 + m_control_gain * error2 / 10000.0, 0.0, 1.0);
      // Valve 4: outlet from chamber 2
      commands[3] =
        std::clamp(0.5 - m_control_gain * error2 / 10000.0, 0.0, 1.0);

      return commands;
    }
  }

  return m_constant_openings;
}

void
ConfigurableValveController::reset()
{
  // No internal state to reset for this simple controller
}

// PneumaticModel implementation
PneumaticModel::PneumaticModel()
  : m_physics(std::make_unique<physics::PneumaticPhysics>())
  , m_friction(std::make_unique<friction::LuGreFriction>())
  , m_stop_force(std::make_unique<stop::StopForce>())
  , m_solver(std::make_unique<solver::OdeSolverWrapper>())
  , m_valve_controller(std::make_shared<ConfigurableValveController>())
{
  MZ_LOG_INFO("Created new PneumaticModel with advanced physics engine");
}

PneumaticModel::~PneumaticModel()
{
  stopSimulation();
}

void
PneumaticModel::startSimulation()
{
  // Ensure any previous simulation is properly stopped
  stopSimulation();

  // Double-check that we're not calculating (thread-safe)
  if (m_is_calculating.load()) {
    MZ_LOG_WARN("Simulation already in progress");
    return;
  }

  m_stop_requested = false;
  m_is_calculating = true; // Set this before creating thread to prevent race

  try {
    m_simulation_thread =
      std::make_unique<std::thread>([this]() { simulationThreadFunction(); });
  } catch (const std::exception& e) {
    // If thread creation fails, reset the calculating flag
    m_is_calculating = false;
    MZ_LOG_ERROR("Failed to create simulation thread");
    throw;
  }
}

void
PneumaticModel::stopSimulation()
{
  if (m_is_calculating.load()) {
    m_stop_requested = true;

    // Wait for the thread to finish if it exists and is joinable
    if (m_simulation_thread && m_simulation_thread->joinable()) {
      m_simulation_thread->join();
    }

    // Clean up the thread object
    m_simulation_thread.reset();

    // Ensure calculating flag is properly reset
    m_is_calculating = false;
  }
}

SimulationResult
PneumaticModel::runSimulation()
{
  // Check if simulation should be stopped before starting
  if (m_stop_requested.load()) {
    MZ_LOG_INFO("Simulation stopped before starting");
    return SimulationResult{}; // Return empty result
  }

  validateConfiguration();

  auto start_time = std::chrono::high_resolution_clock::now();

  try {
    // Update physics components with current configuration
    m_physics->setThermodynamicParams(m_config.thermo);
    m_physics->setGeometryParams(m_config.geometry);
    m_physics->setFluidParams(m_config.fluid);

    m_friction->setParameters(m_config.friction);
    m_stop_force->setParameters(m_config.stop);
    m_solver->setConfig(m_config.solver);

    // Create initial state vector
    auto initial_state =
      m_physics->createInitialState(m_initial_conditions.position,
                                    m_initial_conditions.velocity,
                                    m_initial_conditions.pressure1,
                                    m_initial_conditions.pressure2,
                                    m_initial_conditions.temperature1,
                                    m_initial_conditions.temperature2);

    // Reset friction model
    m_friction->reset();
    m_valve_controller->reset();

    // Check again before starting the expensive computation
    if (m_stop_requested.load()) {
      MZ_LOG_INFO("Simulation stopped during initialization");
      return SimulationResult{};
    }

    // Define ODE system
    auto ode_system = [this](double time, const physics::StateVector& state) {
      return odeSytem(time, state);
    };

    // Solve the system
    auto solver_result =
      m_solver->solve(ode_system, initial_state, m_time_span);

    auto end_time       = std::chrono::high_resolution_clock::now();
    auto solve_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);

    // Process results
    {
      std::lock_guard<std::mutex> lock(m_result_mutex);
      m_result = SimulationResult{};
      processSolverResults(solver_result);
      m_result.solve_time  = solve_duration;
      m_result.solver_name = solver_result.solver_name;
      m_has_results        = true;
    }

    calculateStatistics();

    MZ_LOG_INFO(
      std::format("Simulation completed successfully in {:.3f}s, {} steps",
                  solve_duration.count() / 1000.0,
                  solver_result.steps_taken));

    return m_result;

  } catch (const std::exception& e) {
    MZ_LOG_ERROR(std::format("Simulation failed: {}", e.what()));
    throw;
  }
}

void
PneumaticModel::setConfiguration(const SimulationConfig& config)
{
  if (!config.isValid()) {
    throw std::invalid_argument("Invalid simulation configuration");
  }
  m_config = config;
}

const SimulationConfig&
PneumaticModel::getConfiguration() const noexcept
{
  return m_config;
}

void
PneumaticModel::setInitialConditions(const InitialConditions& conditions)
{
  if (!conditions.isValid()) {
    throw std::invalid_argument("Invalid initial conditions");
  }
  m_initial_conditions = conditions;
}

const InitialConditions&
PneumaticModel::getInitialConditions() const noexcept
{
  return m_initial_conditions;
}

void
PneumaticModel::setTimeSpan(double t_start, double t_end)
{
  if (t_end <= t_start || !std::isfinite(t_start) || !std::isfinite(t_end)) {
    throw std::invalid_argument("Invalid time span");
  }
  m_time_span = { t_start, t_end };
}

std::pair<double, double>
PneumaticModel::getTimeSpan() const noexcept
{
  return m_time_span;
}

void
PneumaticModel::setValveController(
  std::shared_ptr<physics::IValveController> controller)
{
  if (!controller) {
    throw std::invalid_argument("Valve controller cannot be null");
  }
  m_valve_controller = controller;
}

SimulationResult
PneumaticModel::getResults() const
{
  std::lock_guard<std::mutex> lock(m_result_mutex);
  return m_result;
}

SimulationStatistics
PneumaticModel::getStatistics() const
{
  return m_statistics;
}

bool
PneumaticModel::isCalculating() const noexcept
{
  return m_is_calculating.load();
}

bool
PneumaticModel::hasResults() const noexcept
{
  return m_has_results;
}

std::string
PneumaticModel::getSolverInfo() const
{
  return m_solver->getCurrentSolverInfo();
}

void
PneumaticModel::simulationThreadFunction()
{
  // Check if stop was requested before we even started
  if (m_stop_requested.load()) {
    m_is_calculating = false;
    return;
  }

  try {
    auto result = runSimulation();
    // Result already stored by runSimulation()
  } catch (const std::exception& e) {
    MZ_LOG_ERROR(std::format("Background simulation failed: {}", e.what()));
    m_has_results = false;
  }

  // Ensure calculating flag is reset when thread function exits
  m_is_calculating = false;
}

physics::StateVector
PneumaticModel::odeSytem(double time, const physics::StateVector& state) const
{
  using physics::StateIndex;

  // Extract position and velocity
  double position = state[static_cast<std::size_t>(StateIndex::Position)];
  double velocity = state[static_cast<std::size_t>(StateIndex::Velocity)];

  // Calculate friction force using LuGre model
  // Note: We need a small time step for friction integration
  // In practice, this should be coordinated with the solver's step size
  constexpr double dt_friction = 1e-4;
  double friction_force = m_friction->calculateForce(velocity, dt_friction);

  // Calculate smooth stop force based on position and velocity
  double stop_force = m_stop_force->calculateForce(position, velocity);

  // Calculate derivatives using physics model
  return m_physics->calculateDerivatives(
    state, time, *m_valve_controller, friction_force, stop_force);
}

void
PneumaticModel::processSolverResults(const solver::SolveResult& solver_result)
{
  using physics::StateIndex;

  m_result.time_points       = solver_result.time_points;
  m_result.states            = solver_result.state_trajectory;
  m_result.integration_steps = solver_result.steps_taken;
  m_result.converged         = solver_result.converged;

  // Extract derived quantities for convenience
  const std::size_t n_points = solver_result.time_points.size();
  m_result.positions.reserve(n_points);
  m_result.velocities.reserve(n_points);
  m_result.pressures1.reserve(n_points);
  m_result.pressures2.reserve(n_points);
  m_result.temperatures1.reserve(n_points);
  m_result.temperatures2.reserve(n_points);
  m_result.masses1.reserve(n_points);
  m_result.masses2.reserve(n_points);
  m_result.valve_openings.reserve(n_points);

  for (std::size_t i = 0; i < n_points; ++i) {
    const auto&  state = solver_result.state_trajectory[i];
    const double time  = solver_result.time_points[i];

    // Extract basic states
    double x  = state[static_cast<std::size_t>(StateIndex::Position)];
    double v  = state[static_cast<std::size_t>(StateIndex::Velocity)];
    double m1 = state[static_cast<std::size_t>(StateIndex::Mass1)];
    double T1 = state[static_cast<std::size_t>(StateIndex::Temperature1)];
    double m2 = state[static_cast<std::size_t>(StateIndex::Mass2)];
    double T2 = state[static_cast<std::size_t>(StateIndex::Temperature2)];

    m_result.positions.push_back(x);
    m_result.velocities.push_back(v);
    m_result.masses1.push_back(m1);
    m_result.temperatures1.push_back(T1);
    m_result.masses2.push_back(m2);
    m_result.temperatures2.push_back(T2);

    // Calculate pressures from ideal gas law
    auto [V1, V2] = m_physics->calculateVolumes(x);
    auto [p1, p2] = m_physics->calculatePressures(m1, T1, V1, m2, T2, V2);

    m_result.pressures1.push_back(p1);
    m_result.pressures2.push_back(p2);

    // Extract valve states
    std::array<double, 4> valves = {
      state[static_cast<std::size_t>(StateIndex::Valve1)],
      state[static_cast<std::size_t>(StateIndex::Valve2)],
      state[static_cast<std::size_t>(StateIndex::Valve3)],
      state[static_cast<std::size_t>(StateIndex::Valve4)]
    };
    m_result.valve_openings.push_back(valves);
  }
}

void
PneumaticModel::calculateStatistics()
{
  if (!m_result.isValid()) {
    return;
  }

  m_statistics                  = SimulationStatistics{};
  m_statistics.calculation_time = m_result.solve_time;
  m_statistics.steps_taken      = m_result.integration_steps;
  m_statistics.converged        = m_result.converged;

  // Calculate average step size
  if (!m_result.time_points.empty()) {
    double total_time =
      m_result.time_points.back() - m_result.time_points.front();
    m_statistics.average_step_size = total_time / m_result.time_points.size();
  }

  // Find maxima in trajectory
  for (std::size_t i = 0; i < m_result.positions.size(); ++i) {
    m_statistics.max_position =
      std::max(m_statistics.max_position, std::abs(m_result.positions[i]));
    m_statistics.max_velocity =
      std::max(m_statistics.max_velocity, std::abs(m_result.velocities[i]));
    m_statistics.max_pressure1 =
      std::max(m_statistics.max_pressure1, m_result.pressures1[i]);
    m_statistics.max_pressure2 =
      std::max(m_statistics.max_pressure2, m_result.pressures2[i]);
    m_statistics.max_temperature1 =
      std::max(m_statistics.max_temperature1, m_result.temperatures1[i]);
    m_statistics.max_temperature2 =
      std::max(m_statistics.max_temperature2, m_result.temperatures2[i]);
  }

  // Calculate acceleration maxima
  for (std::size_t i = 1; i < m_result.velocities.size(); ++i) {
    if (i < m_result.time_points.size()) {
      double dt = m_result.time_points[i] - m_result.time_points[i - 1];
      if (dt > 0) {
        double accel =
          std::abs((m_result.velocities[i] - m_result.velocities[i - 1]) / dt);
        m_statistics.max_acceleration =
          std::max(m_statistics.max_acceleration, accel);
      }
    }
  }

  // Final values
  if (!m_result.positions.empty()) {
    m_statistics.final_position = m_result.positions.back();
    m_statistics.final_velocity = m_result.velocities.back();
  }

  // Estimate total mass flow (simplified)
  m_statistics.total_mass_flow = 0.0;
  for (std::size_t i = 0; i < m_result.masses1.size(); ++i) {
    m_statistics.total_mass_flow += m_result.masses1[i] + m_result.masses2[i];
  }
  if (!m_result.masses1.empty()) {
    m_statistics.total_mass_flow /= m_result.masses1.size();
  }
}

void
PneumaticModel::validateConfiguration() const
{
  if (!m_config.isValid()) {
    throw std::invalid_argument("Invalid simulation configuration");
  }
  if (!m_initial_conditions.isValid()) {
    throw std::invalid_argument("Invalid initial conditions");
  }
  if (m_time_span.second <= m_time_span.first) {
    throw std::invalid_argument("Invalid time span");
  }
  if (!m_valve_controller) {
    throw std::invalid_argument("No valve controller configured");
  }
}

}