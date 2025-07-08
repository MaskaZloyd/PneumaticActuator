#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace mz::model::stop {

/**
 * @brief Parameters for the smooth stop force model
 *
 * The StopForce model provides smooth position limits using elastic and
 * damping forces that activate near the cylinder stroke limits. It prevents
 * hard impacts and provides realistic boundary behavior.
 *
 * Mathematical model:
 * F_stop = F_lower * w_lower + F_upper * w_upper
 * where:
 * - F_lower = k_stop * (x - x_min) + c_stop * v  (for lower limit)
 * - F_upper = k_stop * (x - x_max) + c_stop * v  (for upper limit)
 * - w_lower = smooth(x_min - x)  (weight function for lower limit)
 * - w_upper = smooth(x - x_max)  (weight function for upper limit)
 * - smooth(y) = 1 / (1 + exp(-2 * alpha * y))  (sigmoid function)
 */
struct StopForceParameters
{
  double x_min{ 0.0 };    ///< Minimum position limit [m]
  double x_max{ 0.35 };   ///< Maximum position limit [m]
  double k_stop{ 1e6 };   ///< Stop stiffness coefficient [N/m]
  double c_stop{ 1e4 };   ///< Stop damping coefficient [Ns/m]
  double alpha{ 1e4 };    ///< Smoothness parameter for sigmoid function [1/m]
  double epsilon{ 1e-6 }; ///< Small regularization parameter [m]

  /**
   * @brief Validate parameter consistency
   * @return true if parameters are physically reasonable
   */
  [[nodiscard]] bool isValid() const noexcept
  {
    return x_max > x_min && k_stop >= 0.0 && c_stop >= 0.0 && alpha > 0.0 &&
           epsilon > 0.0;
  }

  /**
   * @brief Get parameter summary string
   * @return Human-readable parameter description
   */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief Smooth stop force model implementation
 *
 * Implements smooth position limiting forces that activate near the cylinder
 * stroke boundaries. The model provides:
 * - Elastic restoring force (k_stop)
 * - Velocity-dependent damping (c_stop)
 * - Smooth activation using sigmoid functions
 * - No discontinuities or hard impacts
 *
 * Usage:
 * ```cpp
 * StopForce stop_force(params);
 * double position = get_position();
 * double velocity = get_velocity();
 * double force = stop_force.calculateForce(position, velocity);
 * ```
 *
 * @note This class is stateless and thread-safe.
 */
class StopForce
{
public:
  /**
   * @brief Construct stop force model with parameters
   * @param params Stop force model parameters
   * @throw std::invalid_argument if parameters are invalid
   */
  explicit StopForce(StopForceParameters params = {});

  /**
   * @brief Calculate stop force based on position and velocity
   *
   * Computes the smooth stop force using elastic and damping components
   * that activate near the position limits using sigmoid weighting functions.
   *
   * @param position Current piston position [m]
   * @param velocity Current piston velocity [m/s]
   * @return Stop force [N] (positive opposes motion toward limits)
   */
  [[nodiscard]] double calculateForce(double position,
                                      double velocity) const noexcept;

  /**
   * @brief Get current stop force parameters
   * @return Reference to parameter structure
   */
  [[nodiscard]] const StopForceParameters& getParameters() const noexcept;

  /**
   * @brief Update stop force parameters
   * @param params New parameter set
   * @throw std::invalid_argument if parameters are invalid
   */
  void setParameters(const StopForceParameters& params);

  /**
   * @brief Check if position is within normal operating range
   * @param position Position to check [m]
   * @param margin Safety margin from limits [m]
   * @return true if position is within safe range
   */
  [[nodiscard]] bool isWithinLimits(double position,
                                    double margin = 0.01) const noexcept;

  /**
   * @brief Get the activation level for stop forces
   * @param position Current position [m]
   * @return Pair of (lower_activation, upper_activation) weights [0,1]
   */
  [[nodiscard]] std::pair<double, double> getActivationWeights(
    double position) const noexcept;

  /**
   * @brief Calculate maximum possible stop force magnitude
   * @param velocity_limit Maximum expected velocity [m/s]
   * @return Maximum force magnitude [N]
   */
  [[nodiscard]] double getMaxForce(double velocity_limit = 10.0) const noexcept;

private:
  StopForceParameters m_params; ///< Model parameters

  /**
   * @brief Calculate smooth sigmoid function
   *
   * Implements the sigmoid activation function:
   * smooth(y) = 1 / (1 + exp(-2 * alpha * y))
   *
   * @param y Input value
   * @return Sigmoid output in range [0,1]
   */
  [[nodiscard]] double smooth(double y) const noexcept;

  /**
   * @brief Validate parameters and throw if invalid
   * @param params Parameters to validate
   * @throw std::invalid_argument if parameters are invalid
   */
  static void validateParameters(const StopForceParameters& params);
};

} // namespace mz::model::stop