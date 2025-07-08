#include "lugre_friction.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mz::model::friction {

//=============================================================================
// LuGreParameters Implementation
//=============================================================================

std::string
LuGreParameters::toString() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);
  oss << "LuGre Parameters: ";
  oss << "Fc=" << Fc << "N, ";
  oss << "Fs=" << Fs << "N, ";
  oss << "vs=" << vs << "m/s, ";
  oss << "σ₀=" << sigma0 << "N/m, ";
  oss << "σ₁=" << sigma1 << "Ns/m, ";
  oss << "σ₂=" << sigma2 << "Ns/m, ";
  oss << "B=" << B << "Ns/m";
  return oss.str();
}

//=============================================================================
// LuGreFriction Implementation
//=============================================================================

LuGreFriction::LuGreFriction(LuGreParameters params)
  : m_params(std::move(params))
{
  validateParameters(m_params);
}

double
LuGreFriction::calculateForce(double velocity, double dt) noexcept
{
  // Clamp dt to reasonable bounds for numerical stability
  dt = std::clamp(dt, 1e-8, 0.1);

  // Calculate bristle dynamics
  const double bristle_derivative =
    calculateBristleDerivative(velocity, m_bristle_state);

  // Integrate bristle state using explicit Euler
  m_bristle_state += bristle_derivative * dt;

  // Calculate friction force components
  const double bristle_force       = m_params.sigma0 * m_bristle_state;
  const double damping_force       = m_params.sigma1 * bristle_derivative;
  const double micro_viscous_force = m_params.sigma2 * velocity;
  const double viscous_force       = m_params.B * velocity;

  // Total friction force
  const double total_force =
    bristle_force + damping_force + micro_viscous_force + viscous_force;

  // Store velocity for next iteration
  m_last_velocity = velocity;

  return total_force;
}

void
LuGreFriction::reset() noexcept
{
  m_bristle_state = 0.0;
  m_last_velocity = 0.0;
}

double
LuGreFriction::getBristleState() const noexcept
{
  return m_bristle_state;
}

void
LuGreFriction::setBristleState(double state) noexcept
{
  m_bristle_state = state;
}

const LuGreParameters&
LuGreFriction::getParameters() const noexcept
{
  return m_params;
}

void
LuGreFriction::setParameters(const LuGreParameters& params)
{
  validateParameters(params);
  m_params = params;
}

double
LuGreFriction::calculateSteadyStateForce(double velocity) const noexcept
{
  // Solving for z

  if (std::abs(velocity) < m_params.epsilon) {
    // At zero velocity, steady state friction can be anywhere in [-Fs, +Fs]
    return 0.0; // Return zero as a reasonable default
  }

  const double g_v            = calculateStribeckFunction(velocity);
  const double sign_v         = (velocity >= 0.0) ? 1.0 : -1.0;
  const double steady_state_z = g_v * sign_v / m_params.sigma0;

  // Calculate corresponding force
  const double bristle_force       = m_params.sigma0 * steady_state_z;
  const double micro_viscous_force = m_params.sigma2 * velocity;
  const double viscous_force       = m_params.B * velocity;

  return bristle_force + micro_viscous_force + viscous_force;
}

double
LuGreFriction::getMaxStaticFriction() const noexcept
{
  return m_params.Fs;
}

bool
LuGreFriction::isSticking(double velocity_threshold) const noexcept
{
  const double abs_velocity  = std::abs(m_last_velocity);
  const double bristle_force = std::abs(m_params.sigma0 * m_bristle_state);

  return (abs_velocity < velocity_threshold) && (bristle_force < m_params.Fs);
}

double
LuGreFriction::calculateStribeckFunction(double velocity) const noexcept
{
  const double abs_velocity = std::abs(velocity);
  const double exp_term =
    std::exp(-abs_velocity / (m_params.vs + m_params.epsilon));

  return m_params.Fc + (m_params.Fs - m_params.Fc) * exp_term;
}

double
LuGreFriction::calculateBristleDerivative(double velocity,
                                          double bristle_state) const noexcept
{
  const double abs_velocity = std::abs(velocity);

  // Handle near-zero velocity to avoid division by zero
  if (abs_velocity < m_params.epsilon) {
    // When velocity is near zero, the bristle deflection rate is approximately
    // zero This models the sticking behavior where bristles deform slowly
    return velocity; // Simplified behavior: bristles follow velocity directly
  }

  const double g_v = calculateStribeckFunction(velocity);
  const double deflection_term =
    (m_params.sigma0 * abs_velocity * bristle_state) / g_v;

  return velocity - deflection_term;
}

void
LuGreFriction::validateParameters(const LuGreParameters& params)
{
  if (!params.isValid()) {
    std::ostringstream oss;
    oss << "Invalid LuGre parameters: ";

    if (params.Fc < 0.0)
      oss << "Fc must be non-negative; ";
    if (params.Fs < params.Fc)
      oss << "Fs must be >= Fc; ";
    if (params.vs <= 0.0)
      oss << "vs must be positive; ";
    if (params.sigma0 <= 0.0)
      oss << "sigma0 must be positive; ";
    if (params.sigma1 < 0.0)
      oss << "sigma1 must be non-negative; ";
    if (params.sigma2 < 0.0)
      oss << "sigma2 must be non-negative; ";
    if (params.B < 0.0)
      oss << "B must be non-negative; ";
    if (params.epsilon <= 0.0)
      oss << "epsilon must be positive; ";

    throw std::invalid_argument(oss.str());
  }

  // Additional physics-based validation
  if (params.sigma0 < 1.0) {
    throw std::invalid_argument(
      "sigma0 is suspiciously low (< 1 N/m) - check units");
  }

  if (params.vs > 10.0) {
    throw std::invalid_argument(
      "vs is suspiciously high (> 10 m/s) - check units");
  }

  if (params.Fs > 10000.0) {
    throw std::invalid_argument(
      "Fs is suspiciously high (> 10 kN) - check units");
  }
}

//=============================================================================
// Comparison Operators
//=============================================================================

bool
operator==(const LuGreParameters& lhs, const LuGreParameters& rhs) noexcept
{
  constexpr double tolerance = 1e-12;

  return std::abs(lhs.Fc - rhs.Fc) < tolerance &&
         std::abs(lhs.Fs - rhs.Fs) < tolerance &&
         std::abs(lhs.vs - rhs.vs) < tolerance &&
         std::abs(lhs.sigma0 - rhs.sigma0) < tolerance &&
         std::abs(lhs.sigma1 - rhs.sigma1) < tolerance &&
         std::abs(lhs.sigma2 - rhs.sigma2) < tolerance &&
         std::abs(lhs.B - rhs.B) < tolerance &&
         std::abs(lhs.epsilon - rhs.epsilon) < tolerance;
}

bool
operator!=(const LuGreParameters& lhs, const LuGreParameters& rhs) noexcept
{
  return !(lhs == rhs);
}

}