#pragma once

#include <cmath>
#include <string>

namespace mz::model::friction {

/**
 * @brief Parameters for the LuGre friction model
 *
 * The LuGre (Lund-Grenoble) friction model represents friction as dynamic
 * interaction between bristles. It captures stick-slip behavior, Stribeck
 * effect, and provides smooth transitions between different friction regimes.
 *
 * Mathematical model:
 * dz/dt = v - σ₀|v|z/g(v)
 * F_friction = σ₀z + σ₁(dz/dt) + σ₂v + Bv
 *
 * where:
 * - z: bristle deflection state [m]
 * - v: relative velocity [m/s]
 * - g(v): Stribeck function = Fc + (Fs - Fc)exp(-|v|/(vs + ε))
 */
struct LuGreParameters
{
  double Fc{ 34.0 };       ///< Coulomb friction force [N]
  double Fs{ 52.5 };       ///< Static friction force [N]
  double vs{ 0.024 };      ///< Stribeck velocity [m/s]
  double sigma0{ 1000.0 }; ///< Bristle stiffness [N/m]
  double sigma1{ 100.0 };  ///< Bristle damping [Ns/m]
  double sigma2{ 0.1 };    ///< Micro-viscous friction [Ns/m]
  double B{ 300.0 };       ///< Additional viscous damping [Ns/m]
  double epsilon{
    1e-6
  }; ///< Regularization parameter to avoid singularities [m/s]

  /**
   * @brief Validate parameter consistency
   * @return true if parameters are physically reasonable
   */
  [[nodiscard]] bool isValid() const noexcept
  {
    return Fc >= 0.0 && Fs >= Fc && vs > 0.0 && sigma0 > 0.0 && sigma1 >= 0.0 &&
           sigma2 >= 0.0 && B >= 0.0 && epsilon > 0.0;
  }

  /**
   * @brief Get parameter summary string
   * @return Human-readable parameter description
   */
  [[nodiscard]] std::string toString() const;
};

/**
 * @brief LuGre friction model implementation
 *
 * Implements the dynamic LuGre friction model with internal bristle state.
 * The model provides realistic friction behavior including:
 * - Static friction (stiction)
 * - Kinetic friction with Stribeck effect
 * - Viscous friction
 * - Smooth transitions between regimes
 *
 * Usage:
 * ```cpp
 * LuGreFriction friction(params);
 * double dt = 0.001; // time step
 * double velocity = get_velocity();
 * double friction_force = friction.calculateForce(velocity, dt);
 * ```
 *
 * @note This class maintains internal state (bristle deflection) and is NOT
 * thread-safe.
 * @note The time step dt should be reasonably small for numerical stability.
 */
class LuGreFriction
{
public:
  /**
   * @brief Construct LuGre friction model with parameters
   * @param params Friction model parameters
   * @throw std::invalid_argument if parameters are invalid
   */
  explicit LuGreFriction(LuGreParameters params = {});

  /**
   * @brief Update friction state and calculate friction force
   *
   * Integrates the bristle dynamics using explicit Euler and computes
   * the resulting friction force based on the LuGre model equations.
   *
   * @param velocity Current relative velocity [m/s]
   * @param dt Time step for integration [s]
   * @return Friction force opposing motion [N]
   *
   * @note dt should be small enough for numerical stability (typically < 0.01s)
   * @note Call this method at regular intervals for accurate dynamics
   */
  [[nodiscard]] double calculateForce(double velocity, double dt) noexcept;

  /**
   * @brief Reset internal bristle state to zero
   *
   * Useful when starting new simulations or when discontinuous
   * changes in velocity occur (e.g., impacts, sudden stops).
   */
  void reset() noexcept;

  /**
   * @brief Get current bristle deflection state
   * @return Current bristle state z [m]
   */
  [[nodiscard]] double getBristleState() const noexcept;

  /**
   * @brief Set bristle state manually
   * @param state New bristle deflection [m]
   *
   * @note Use with caution - normally the state evolves naturally
   */
  void setBristleState(double state) noexcept;

  /**
   * @brief Get current friction parameters
   * @return Reference to parameter structure
   */
  [[nodiscard]] const LuGreParameters& getParameters() const noexcept;

  /**
   * @brief Update friction parameters
   * @param params New parameter set
   * @throw std::invalid_argument if parameters are invalid
   */
  void setParameters(const LuGreParameters& params);

  /**
   * @brief Calculate steady-state friction force for given velocity
   *
   * Computes the friction force that would result if the system
   * were in steady state at the given velocity (i.e., dz/dt = 0).
   *
   * @param velocity Steady-state velocity [m/s]
   * @return Steady-state friction force [N]
   */
  [[nodiscard]] double calculateSteadyStateForce(
    double velocity) const noexcept;

  /**
   * @brief Get maximum static friction force
   * @return Static friction threshold [N]
   */
  [[nodiscard]] double getMaxStaticFriction() const noexcept;

  /**
   * @brief Check if current state indicates sticking
   * @param velocity_threshold Threshold below which motion is considered zero
   * [m/s]
   * @return true if velocity is below threshold and bristle force exceeds
   * static limit
   */
  [[nodiscard]] bool isSticking(
    double velocity_threshold = 1e-6) const noexcept;

private:
  LuGreParameters m_params;               ///< Model parameters
  double          m_bristle_state{ 0.0 }; ///< Current bristle deflection z [m]
  double          m_last_velocity{
    0.0
  }; ///< Previous velocity for derivative estimation [m/s]

  /**
   * @brief Calculate Stribeck function g(v)
   *
   * The Stribeck function models the velocity-dependent friction behavior:
   * g(v) = Fc + (Fs - Fc) * exp(-|v|/(vs + ε))
   *
   * @param velocity Relative velocity [m/s]
   * @return Stribeck function value [N]
   */
  [[nodiscard]] double calculateStribeckFunction(
    double velocity) const noexcept;

  /**
   * @brief Calculate bristle dynamics derivative dz/dt
   *
   * Implements the bristle deflection dynamics:
   * dz/dt = v - σ₀|v|z/g(v)
   *
   * @param velocity Current velocity [m/s]
   * @param bristle_state Current bristle deflection [m]
   * @return Time derivative of bristle state [m/s]
   */
  [[nodiscard]] double calculateBristleDerivative(
    double velocity,
    double bristle_state) const noexcept;

  /**
   * @brief Validate parameters and throw if invalid
   * @param params Parameters to validate
   * @throw std::invalid_argument if parameters are invalid
   */
  static void validateParameters(const LuGreParameters& params);
};

/**
 * @brief Comparison operators for LuGreParameters
 */
[[nodiscard]] bool
operator==(const LuGreParameters& lhs, const LuGreParameters& rhs) noexcept;
[[nodiscard]] bool
operator!=(const LuGreParameters& lhs, const LuGreParameters& rhs) noexcept;

} // namespace mz::model::friction