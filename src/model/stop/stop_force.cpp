#include "stop_force.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mz::model::stop {

// StopForceParameters toString implementation
std::string
StopForceParameters::toString() const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "StopForceParameters:\n";
  oss << "  Position limits: [" << x_min << ", " << x_max << "] m\n";
  oss << "  Stiffness: " << k_stop << " N/m\n";
  oss << "  Damping: " << c_stop << " Ns/m\n";
  oss << "  Smoothness: " << alpha << " 1/m\n";
  oss << "  Epsilon: " << epsilon << " m";
  return oss.str();
}

// StopForce implementation
StopForce::StopForce(StopForceParameters params)
  : m_params(std::move(params))
{
  validateParameters(m_params);
}

double
StopForce::calculateForce(double position, double velocity) const noexcept
{
  // Calculate forces for lower and upper limits
  const double F_lower =
    m_params.k_stop * (position - m_params.x_min) + m_params.c_stop * velocity;
  const double F_upper =
    m_params.k_stop * (position - m_params.x_max) + m_params.c_stop * velocity;

  // Calculate smooth weighting functions
  const double w_lower = smooth(m_params.x_min - position);
  const double w_upper = smooth(position - m_params.x_max);

  // Combined stop force
  return F_lower * w_lower + F_upper * w_upper;
}

const StopForceParameters&
StopForce::getParameters() const noexcept
{
  return m_params;
}

void
StopForce::setParameters(const StopForceParameters& params)
{
  validateParameters(params);
  m_params = params;
}

bool
StopForce::isWithinLimits(double position, double margin) const noexcept
{
  return position >= (m_params.x_min + margin) &&
         position <= (m_params.x_max - margin);
}

std::pair<double, double>
StopForce::getActivationWeights(double position) const noexcept
{
  const double w_lower = smooth(m_params.x_min - position);
  const double w_upper = smooth(position - m_params.x_max);
  return { w_lower, w_upper };
}

double
StopForce::getMaxForce(double velocity_limit) const noexcept
{
  // Maximum force occurs at the limits with maximum velocity
  const double max_elastic_lower = m_params.k_stop * std::abs(m_params.x_min);
  const double max_elastic_upper = m_params.k_stop * std::abs(m_params.x_max);
  const double max_damping       = m_params.c_stop * velocity_limit;

  return std::max(max_elastic_lower, max_elastic_upper) + max_damping;
}

double
StopForce::smooth(double y) const noexcept
{
  // Prevent overflow in exponential function
  const double z = std::clamp(-2.0 * m_params.alpha * y, -709.0, 709.0);
  return 1.0 / (1.0 + std::exp(z));
}

void
StopForce::validateParameters(const StopForceParameters& params)
{
  if (!params.isValid()) {
    throw std::invalid_argument("Invalid stop force parameters");
  }

  if (params.x_max <= params.x_min) {
    throw std::invalid_argument("x_max must be greater than x_min");
  }

  if (params.k_stop < 0.0) {
    throw std::invalid_argument("Stop stiffness k_stop must be non-negative");
  }

  if (params.c_stop < 0.0) {
    throw std::invalid_argument("Stop damping c_stop must be non-negative");
  }

  if (params.alpha <= 0.0) {
    throw std::invalid_argument("Smoothness parameter alpha must be positive");
  }

  if (params.epsilon <= 0.0) {
    throw std::invalid_argument("Epsilon parameter must be positive");
  }
}

}