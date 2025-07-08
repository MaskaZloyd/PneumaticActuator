#include "pneumatic_physics.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mz::model::physics {

// GeometryParameters toString implementation
std::string
GeometryParameters::toString() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "GeometryParameters:\n";
  oss << "  A1: " << A1 << " m²\n";
  oss << "  A2: " << A2 << " m²\n";
  oss << "  V1_0: " << V1_0 << " m³\n";
  oss << "  V2_0: " << V2_0 << " m³\n";
  oss << "  L: " << L << " m\n";
  oss << "  M: " << M << " kg";
  return oss.str();
}

// FluidParameters toString implementation
std::string
FluidParameters::toString() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1);
  oss << "FluidParameters:\n";
  oss << "  Supply: " << p_s / 1000.0 << " kPa @ " << T_s << " K\n";
  oss << "  Atmosphere: " << p_a / 1000.0 << " kPa @ " << T_a << " K\n";
  oss << std::setprecision(6);
  oss << "  Valve areas: [" << Av1 << ", " << Av2 << ", " << Av3 << ", " << Av4
      << "] m²\n";
  oss << "  Discharge coeffs: [" << Cd1 << ", " << Cd2 << ", " << Cd3 << ", "
      << Cd4 << "]\n";
  oss << "  Time constant: " << tau << " s";
  return oss.str();
}

// PneumaticPhysics implementation
PneumaticPhysics::PneumaticPhysics(ThermodynamicParameters thermo,
                                   GeometryParameters      geom,
                                   FluidParameters         fluid)
  : m_thermo(std::move(thermo))
  , m_geom(std::move(geom))
  , m_fluid(std::move(fluid))
{
  validateParameters();
}

double
PneumaticPhysics::calculateMassFlowRate(double valve_opening,
                                        double Cd,
                                        double Av,
                                        double p_upstream,
                                        double T_upstream,
                                        double p_downstream) const noexcept
{
  // Handle edge cases
  if (valve_opening <= 0.0 || p_upstream <= p_downstream || T_upstream <= 0.0) {
    return 0.0;
  }

  // Ensure valve opening is bounded
  const double u = std::clamp(valve_opening, 0.0, 1.0);

  // Calculate pressure ratio
  const double pi_ratio = p_downstream / p_upstream;
  const double pi_crit  = m_thermo.getPiCrit();

  // Common factor
  const double common_factor =
    u * Cd * Av * (p_upstream / std::sqrt(T_upstream));

  double mass_flow = 0.0;

  if (pi_ratio > pi_crit) {
    // Subsonic flow
    const double gamma      = m_thermo.gamma;
    const double R          = m_thermo.R;

    const double pi_2_gamma = std::pow(pi_ratio, 2.0 / gamma);
    const double pi_gamma_plus_1_gamma =
      std::pow(pi_ratio, (gamma + 1.0) / gamma);

    const double flow_coeff = std::sqrt(2.0 * gamma / (R * (gamma - 1.0)) *
                                        (pi_2_gamma - pi_gamma_plus_1_gamma));

    mass_flow               = common_factor * flow_coeff;
  } else {
    // Sonic (choked) flow
    const double gamma = m_thermo.gamma;
    const double R     = m_thermo.R;

    const double choked_coeff =
      std::sqrt(gamma / R) *
      std::pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));

    mass_flow = common_factor * choked_coeff;
  }

  return std::max(0.0, mass_flow);
}

std::pair<double, double>
PneumaticPhysics::calculateVolumes(double position) const noexcept
{
  // Constrain position to valid range
  const double x  = std::clamp(position, 0.0, m_geom.L);

  const double V1 = m_geom.V1_0 + m_geom.A1 * x;
  const double V2 = m_geom.V2_0 + m_geom.A2 * (m_geom.L - x);

  return { V1, V2 };
}

std::pair<double, double>
PneumaticPhysics::calculatePressures(double m1,
                                     double T1,
                                     double V1,
                                     double m2,
                                     double T2,
                                     double V2) const noexcept
{
  // Apply minimum constraints
  const double mass1 = std::max(m1, m_thermo.min_mass);
  const double mass2 = std::max(m2, m_thermo.min_mass);
  const double temp1 = std::max(T1, m_thermo.min_temperature);
  const double temp2 = std::max(T2, m_thermo.min_temperature);
  const double vol1  = std::max(V1, 1e-9); // Minimum volume
  const double vol2  = std::max(V2, 1e-9);

  // Ideal gas law: p = mRT/V
  const double p1 =
    std::max(mass1 * m_thermo.R * temp1 / vol1, m_thermo.min_pressure);
  const double p2 =
    std::max(mass2 * m_thermo.R * temp2 / vol2, m_thermo.min_pressure);

  return { p1, p2 };
}

StateVector
PneumaticPhysics::calculateDerivatives(const StateVector&      state,
                                       double                  time,
                                       const IValveController& valve_controller,
                                       double                  friction_force,
                                       double                  stop_force) const
{
  if (state.size() != STATE_SIZE) {
    throw std::invalid_argument("State vector must have " +
                                std::to_string(STATE_SIZE) + " components");
  }

  StateVector derivatives(STATE_SIZE);

  // Extract state variables
  const double x  = state[static_cast<std::size_t>(StateIndex::Position)];
  const double v  = state[static_cast<std::size_t>(StateIndex::Velocity)];
  const double m1 = state[static_cast<std::size_t>(StateIndex::Mass1)];
  const double T1 = state[static_cast<std::size_t>(StateIndex::Temperature1)];
  const double m2 = state[static_cast<std::size_t>(StateIndex::Mass2)];
  const double T2 = state[static_cast<std::size_t>(StateIndex::Temperature2)];
  const double u1 = state[static_cast<std::size_t>(StateIndex::Valve1)];
  const double u2 = state[static_cast<std::size_t>(StateIndex::Valve2)];
  const double u3 = state[static_cast<std::size_t>(StateIndex::Valve3)];
  const double u4 = state[static_cast<std::size_t>(StateIndex::Valve4)];

  // Calculate volumes and pressures
  const auto [V1, V2] = calculateVolumes(x);
  const auto [p1, p2] = calculatePressures(m1, T1, V1, m2, T2, V2);

  // Get valve commands from controller
  const auto valve_commands = valve_controller.getValveCommands(time, state);

  // Calculate mass flow rates
  // Chamber 1 flows
  const double m_dot_1_in = calculateMassFlowRate(
    u1, m_fluid.Cd1, m_fluid.Av1, m_fluid.p_s, m_fluid.T_s, p1);
  const double m_dot_1_out =
    calculateMassFlowRate(u2, m_fluid.Cd2, m_fluid.Av2, p1, T1, m_fluid.p_a);

  // Chamber 2 flows
  const double m_dot_2_in = calculateMassFlowRate(
    u3, m_fluid.Cd3, m_fluid.Av3, m_fluid.p_s, m_fluid.T_s, p2);
  const double m_dot_2_out =
    calculateMassFlowRate(u4, m_fluid.Cd4, m_fluid.Av4, p2, T2, m_fluid.p_a);

  // Calculate energy derivatives
  const double E_dot_1 = calculateEnergyDerivative(
    m_dot_1_in, m_dot_1_out, m_fluid.T_s, T1, p1, m_geom.A1, v);
  const double E_dot_2 = calculateEnergyDerivative(
    m_dot_2_in, m_dot_2_out, m_fluid.T_s, T2, p2, m_geom.A2, -v);

  // Position derivative
  derivatives[static_cast<std::size_t>(StateIndex::Position)] = v;

  // Velocity derivative
  const double pressure_force =
    m_geom.A1 * p1 - m_geom.A2 * p2 - (m_geom.A1 - m_geom.A2) * m_fluid.p_a;
  const double net_force = pressure_force - friction_force - stop_force;
  derivatives[static_cast<std::size_t>(StateIndex::Velocity)] =
    net_force / m_geom.M;

  // Mass derivatives
  derivatives[static_cast<std::size_t>(StateIndex::Mass1)] =
    m_dot_1_in - m_dot_1_out;
  derivatives[static_cast<std::size_t>(StateIndex::Mass2)] =
    m_dot_2_in - m_dot_2_out;

  // Temperature derivatives
  const double Cv          = m_thermo.getCv();
  const double safe_m1     = std::max(m1, m_thermo.min_mass);
  const double safe_m2     = std::max(m2, m_thermo.min_mass);

  const double m_dot_1_net = m_dot_1_in - m_dot_1_out;
  const double m_dot_2_net = m_dot_2_in - m_dot_2_out;

  derivatives[static_cast<std::size_t>(StateIndex::Temperature1)] =
    (E_dot_1 - Cv * T1 * m_dot_1_net) / (safe_m1 * Cv);
  derivatives[static_cast<std::size_t>(StateIndex::Temperature2)] =
    (E_dot_2 - Cv * T2 * m_dot_2_net) / (safe_m2 * Cv);

  // Valve dynamics
  const double inv_tau = 1.0 / m_fluid.tau;
  derivatives[static_cast<std::size_t>(StateIndex::Valve1)] =
    (valve_commands[0] - u1) * inv_tau;
  derivatives[static_cast<std::size_t>(StateIndex::Valve2)] =
    (valve_commands[1] - u2) * inv_tau;
  derivatives[static_cast<std::size_t>(StateIndex::Valve3)] =
    (valve_commands[2] - u3) * inv_tau;
  derivatives[static_cast<std::size_t>(StateIndex::Valve4)] =
    (valve_commands[3] - u4) * inv_tau;

  return derivatives;
}

StateVector
PneumaticPhysics::createInitialState(double position,
                                     double velocity,
                                     double pressure1,
                                     double pressure2,
                                     double temperature1,
                                     double temperature2) const
{
  StateVector state(STATE_SIZE);

  // Constrain position
  const double x = std::clamp(position, 0.0, m_geom.L);

  // Calculate volumes
  const auto [V1, V2] = calculateVolumes(x);

  // Calculate masses from ideal gas law
  const double m1 = pressure1 * V1 / (m_thermo.R * temperature1);
  const double m2 = pressure2 * V2 / (m_thermo.R * temperature2);

  // Set state components
  state[static_cast<std::size_t>(StateIndex::Position)] = x;
  state[static_cast<std::size_t>(StateIndex::Velocity)] = velocity;
  state[static_cast<std::size_t>(StateIndex::Mass1)] =
    std::max(m1, m_thermo.min_mass);
  state[static_cast<std::size_t>(StateIndex::Temperature1)] =
    std::max(temperature1, m_thermo.min_temperature);
  state[static_cast<std::size_t>(StateIndex::Mass2)] =
    std::max(m2, m_thermo.min_mass);
  state[static_cast<std::size_t>(StateIndex::Temperature2)] =
    std::max(temperature2, m_thermo.min_temperature);
  state[static_cast<std::size_t>(StateIndex::Valve1)] = 0.0;
  state[static_cast<std::size_t>(StateIndex::Valve2)] = 0.0;
  state[static_cast<std::size_t>(StateIndex::Valve3)] = 0.0;
  state[static_cast<std::size_t>(StateIndex::Valve4)] = 0.0;

  return state;
}

bool
PneumaticPhysics::isValidState(const StateVector& state) const noexcept
{
  if (state.size() != STATE_SIZE) {
    return false;
  }

  const double x  = state[static_cast<std::size_t>(StateIndex::Position)];
  const double v  = state[static_cast<std::size_t>(StateIndex::Velocity)];
  const double m1 = state[static_cast<std::size_t>(StateIndex::Mass1)];
  const double T1 = state[static_cast<std::size_t>(StateIndex::Temperature1)];
  const double m2 = state[static_cast<std::size_t>(StateIndex::Mass2)];
  const double T2 = state[static_cast<std::size_t>(StateIndex::Temperature2)];
  const double u1 = state[static_cast<std::size_t>(StateIndex::Valve1)];
  const double u2 = state[static_cast<std::size_t>(StateIndex::Valve2)];
  const double u3 = state[static_cast<std::size_t>(StateIndex::Valve3)];
  const double u4 = state[static_cast<std::size_t>(StateIndex::Valve4)];

  // Check position bounds
  if (x < 0.0 || x > m_geom.L)
    return false;

  // Check velocity bounds (reasonable limits)
  if (std::abs(v) > 100.0)
    return false;

  // Check mass constraints
  if (m1 < m_thermo.min_mass || m2 < m_thermo.min_mass)
    return false;

  // Check temperature constraints
  if (T1 < m_thermo.min_temperature || T2 < m_thermo.min_temperature)
    return false;
  if (T1 > 1000.0 || T2 > 1000.0)
    return false; // Reasonable upper bound

  // Check valve openings
  if (u1 < 0.0 || u1 > 1.0)
    return false;
  if (u2 < 0.0 || u2 > 1.0)
    return false;
  if (u3 < 0.0 || u3 > 1.0)
    return false;
  if (u4 < 0.0 || u4 > 1.0)
    return false;

  // Check for NaN or infinite values
  for (std::size_t i = 0; i < STATE_SIZE; ++i) {
    if (!std::isfinite(state[i]))
      return false;
  }

  return true;
}

void
PneumaticPhysics::constrainState(StateVector& state) const noexcept
{
  if (state.size() != STATE_SIZE) {
    return;
  }

  // Constrain position
  state[static_cast<std::size_t>(StateIndex::Position)] = std::clamp(
    state[static_cast<std::size_t>(StateIndex::Position)], 0.0, m_geom.L);

  // Constrain velocity (reasonable limits)
  state[static_cast<std::size_t>(StateIndex::Velocity)] = std::clamp(
    state[static_cast<std::size_t>(StateIndex::Velocity)], -100.0, 100.0);

  // Constrain masses
  state[static_cast<std::size_t>(StateIndex::Mass1)] = std::max(
    state[static_cast<std::size_t>(StateIndex::Mass1)], m_thermo.min_mass);
  state[static_cast<std::size_t>(StateIndex::Mass2)] = std::max(
    state[static_cast<std::size_t>(StateIndex::Mass2)], m_thermo.min_mass);

  // Constrain temperatures
  state[static_cast<std::size_t>(StateIndex::Temperature1)] =
    std::clamp(state[static_cast<std::size_t>(StateIndex::Temperature1)],
               m_thermo.min_temperature,
               1000.0);
  state[static_cast<std::size_t>(StateIndex::Temperature2)] =
    std::clamp(state[static_cast<std::size_t>(StateIndex::Temperature2)],
               m_thermo.min_temperature,
               1000.0);

  // Constrain valve openings
  state[static_cast<std::size_t>(StateIndex::Valve1)] =
    std::clamp(state[static_cast<std::size_t>(StateIndex::Valve1)], 0.0, 1.0);
  state[static_cast<std::size_t>(StateIndex::Valve2)] =
    std::clamp(state[static_cast<std::size_t>(StateIndex::Valve2)], 0.0, 1.0);
  state[static_cast<std::size_t>(StateIndex::Valve3)] =
    std::clamp(state[static_cast<std::size_t>(StateIndex::Valve3)], 0.0, 1.0);
  state[static_cast<std::size_t>(StateIndex::Valve4)] =
    std::clamp(state[static_cast<std::size_t>(StateIndex::Valve4)], 0.0, 1.0);

  // Replace NaN or infinite values with safe defaults
  for (std::size_t i = 0; i < STATE_SIZE; ++i) {
    if (!std::isfinite(state[i])) {
      switch (static_cast<StateIndex>(i)) {
        case StateIndex::Position:
          state[i] = m_geom.L / 2.0;
          break;
        case StateIndex::Velocity:
          state[i] = 0.0;
          break;
        case StateIndex::Mass1:
        case StateIndex::Mass2:
          state[i] = m_thermo.min_mass;
          break;
        case StateIndex::Temperature1:
        case StateIndex::Temperature2:
          state[i] = m_thermo.min_temperature;
          break;
        case StateIndex::Valve1:
        case StateIndex::Valve2:
        case StateIndex::Valve3:
        case StateIndex::Valve4:
          state[i] = 0.0;
          break;
      }
    }
  }
}

void
PneumaticPhysics::setThermodynamicParams(const ThermodynamicParameters& params)
{
  if (!params.isValid()) {
    throw std::invalid_argument("Invalid thermodynamic parameters");
  }
  m_thermo = params;
}

void
PneumaticPhysics::setGeometryParams(const GeometryParameters& params)
{
  if (!params.isValid()) {
    throw std::invalid_argument("Invalid geometry parameters");
  }
  m_geom = params;
}

void
PneumaticPhysics::setFluidParams(const FluidParameters& params)
{
  if (!params.isValid()) {
    throw std::invalid_argument("Invalid fluid parameters");
  }
  m_fluid = params;
}

double
PneumaticPhysics::calculateEnergyDerivative(double mass_flow_in,
                                            double mass_flow_out,
                                            double T_supply,
                                            double T_chamber,
                                            double pressure,
                                            double area,
                                            double velocity) const noexcept
{
  const double Cp = m_thermo.getCp();

  // Energy flow in
  const double energy_in = mass_flow_in * Cp * T_supply;

  // Energy flow out
  const double energy_out = mass_flow_out * Cp * T_chamber;

  // Work done by the gas
  const double work_done = pressure * area * velocity;

  // Total energy derivative
  return energy_in - energy_out - work_done;
}

void
PneumaticPhysics::validateParameters() const
{
  if (!m_thermo.isValid()) {
    throw std::invalid_argument("Invalid thermodynamic parameters");
  }
  if (!m_geom.isValid()) {
    throw std::invalid_argument("Invalid geometry parameters");
  }
  if (!m_fluid.isValid()) {
    throw std::invalid_argument("Invalid fluid parameters");
  }
}

}