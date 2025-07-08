#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>

namespace mz::model::physics {

/**
 * @brief Thermodynamic constants and parameters for ideal gas calculations
 *
 * Based on the mathematical model specification for air at standard conditions.
 * These parameters define the fundamental thermodynamic relationships.
 */
struct ThermodynamicParameters
{
  double gamma{ 1.4 };           ///< Heat capacity ratio (dimensionless)
  double R{ 287.0 };             ///< Specific gas constant [J/(kg·K)]
  double min_mass{ 1e-6 };       ///< Minimum mass threshold [kg]
  double min_temperature{ 1.0 }; ///< Minimum temperature [K]
  double min_pressure{ 1e3 };    ///< Minimum pressure [Pa]

  /// Derived thermodynamic properties
  [[nodiscard]] double getCp() const noexcept
  {
    return gamma * R / (gamma - 1.0);
  }
  [[nodiscard]] double getCv() const noexcept { return R / (gamma - 1.0); }
  [[nodiscard]] double getPiCrit() const noexcept
  {
    return std::pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
  }

  [[nodiscard]] bool isValid() const noexcept
  {
    return gamma > 1.0 && R > 0.0 && min_mass > 0.0 && min_temperature > 0.0 &&
           min_pressure > 0.0;
  }
};

/**
 * @brief Geometric parameters defining the pneumatic cylinder
 *
 * Includes chamber areas, dead volumes, stroke length, and mass properties
 * as specified in the mathematical model documentation.
 */
struct GeometryParameters
{
  double A1{ 0.000804 }; ///< Chamber 1 effective area [m²]
  double A2{ 0.000690 }; ///< Chamber 2 effective area [m²]
  double V1_0{ 1e-5 };   ///< Chamber 1 dead volume [m³]
  double V2_0{ 1e-5 };   ///< Chamber 2 dead volume [m³]
  double L{ 0.35 };      ///< Total stroke length [m]
  double M{ 6.0 };       ///< Piston and load mass [kg]

  [[nodiscard]] bool isValid() const noexcept
  {
    return A1 > 0.0 && A2 > 0.0 && V1_0 > 0.0 && V2_0 > 0.0 && L > 0.0 &&
           M > 0.0;
  }

  [[nodiscard]] std::string toString() const;
};

/**
 * @brief Fluid system parameters including supply conditions and valve
 * characteristics
 *
 * Defines the pneumatic supply system, valve flow characteristics, and
 * valve dynamics as per the mathematical model specification.
 */
struct FluidParameters
{
  double p_s{ 5e5 };    ///< Supply pressure [Pa]
  double T_s{ 293.15 }; ///< Supply temperature [K]
  double p_a{ 1e5 };    ///< Atmospheric pressure [Pa]
  double T_a{ 293.15 }; ///< Atmospheric temperature [K]

  // Valve characteristics (assuming identical valves)
  double Cd1{ 0.7 }, Cd2{ 0.7 }, Cd3{ 0.7 },
    Cd4{ 0.7 }; ///< Discharge coefficients
  double Av1{ 2.4e-4 }, Av2{ 2.4e-4 }, Av3{ 2.4e-4 },
    Av4{ 2.4e-4 }; ///< Valve areas [m²]

  double tau{ 0.1 }; ///< Valve time constant [s]

  [[nodiscard]] bool isValid() const noexcept
  {
    return p_s > p_a && T_s > 0.0 && T_a > 0.0 && Cd1 > 0.0 && Cd2 > 0.0 &&
           Cd3 > 0.0 && Cd4 > 0.0 && Av1 > 0.0 && Av2 > 0.0 && Av3 > 0.0 &&
           Av4 > 0.0 && tau > 0.0;
  }

  [[nodiscard]] std::string toString() const;
};

/**
 * @brief State vector indices for the 10-component pneumatic system
 *
 * Defines the ordering of state variables in the complete system as:
 * [x, v, m₁, T₁, m₂, T₂, u₁, u₂, u₃, u₄]ᵀ
 */
enum class StateIndex : std::size_t
{
  Position     = 0, ///< x - Piston position [m]
  Velocity     = 1, ///< v - Piston velocity [m/s]
  Mass1        = 2, ///< m₁ - Air mass in chamber 1 [kg]
  Temperature1 = 3, ///< T₁ - Air temperature in chamber 1 [K]
  Mass2        = 4, ///< m₂ - Air mass in chamber 2 [kg]
  Temperature2 = 5, ///< T₂ - Air temperature in chamber 2 [K]
  Valve1       = 6, ///< u₁ - Inlet valve 1 opening [0-1]
  Valve2       = 7, ///< u₂ - Outlet valve 1 opening [0-1]
  Valve3       = 8, ///< u₃ - Inlet valve 2 opening [0-1]
  Valve4       = 9  ///< u₄ - Outlet valve 2 opening [0-1]
};

constexpr std::size_t STATE_SIZE = 10; ///< Total number of state variables
using StateVector                = Eigen::VectorXd; ///< State vector type

/**
 * @brief Valve controller interface for providing valve commands
 *
 * Abstract interface for controllers that determine valve opening commands
 * based on time and system state. Implementations can include PID controllers,
 * fuzzy controllers, or other control strategies.
 */
class IValveController
{
public:
  virtual ~IValveController() = default;

  /**
   * @brief Get valve opening commands at current time and state
   * @param time Current simulation time [s]
   * @param state Current system state vector
   * @return Array of valve openings [u₁, u₂, u₃, u₄] each in [0,1]
   */
  [[nodiscard]] virtual std::array<double, 4> getValveCommands(
    double             time,
    const StateVector& state) const = 0;

  /**
   * @brief Reset controller state (if stateful)
   */
  virtual void reset() {}
};

/**
 * @brief Simple open-loop valve controller for testing
 *
 * Provides constant valve openings - useful for system identification
 * and basic testing scenarios.
 */
class ConstantValveController final : public IValveController
{
public:
  explicit ConstantValveController(
    std::array<double, 4> openings = { 0.0, 0.0, 0.0, 0.0 })
    : m_openings(openings)
  {
  }

  [[nodiscard]] std::array<double, 4> getValveCommands(
    double /*time*/,
    const StateVector& /*state*/) const override
  {
    return m_openings;
  }

  void setOpenings(const std::array<double, 4>& openings)
  {
    m_openings = openings;
  }

private:
  std::array<double, 4> m_openings;
};

/**
 * @brief Complete pneumatic system physics implementation
 *
 * Implements the full mathematical model with:
 * - Thermodynamic equations for air masses and temperatures
 * - Choked flow through valves with critical pressure ratio
 * - Chamber volume calculations based on piston position
 * - Energy balance equations with heat capacity considerations
 * - Valve dynamics with first-order response
 *
 * The system equations follow the mathematical specification exactly.
 */
class PneumaticPhysics
{
public:
  /**
   * @brief Construct physics model with parameters
   * @param thermo Thermodynamic parameters
   * @param geom Geometric parameters
   * @param fluid Fluid system parameters
   */
  explicit PneumaticPhysics(ThermodynamicParameters thermo = {},
                            GeometryParameters      geom   = {},
                            FluidParameters         fluid  = {});

  /**
   * @brief Calculate mass flow rate through a valve using choked flow equations
   *
   * Implements the isentropic throttling formula with critical pressure ratio:
   * For Π > Π_crit: ṁ = u·Cd·Av·(pu/√Tu)·√(2γ/[R(γ-1)]·[Π^(2/γ) - Π^((γ+1)/γ)])
   * For Π ≤ Π_crit: ṁ = u·Cd·Av·(pu/√Tu)·√(γ/R)·(2/(γ+1))^((γ+1)/[2(γ-1)])
   *
   * @param valve_opening Valve opening fraction [0-1]
   * @param Cd Discharge coefficient
   * @param Av Valve area [m²]
   * @param p_upstream Upstream pressure [Pa]
   * @param T_upstream Upstream temperature [K]
   * @param p_downstream Downstream pressure [Pa]
   * @return Mass flow rate [kg/s]
   */
  [[nodiscard]] double calculateMassFlowRate(
    double valve_opening,
    double Cd,
    double Av,
    double p_upstream,
    double T_upstream,
    double p_downstream) const noexcept;

  /**
   * @brief Calculate chamber volumes based on piston position
   *
   * V₁(x) = V₁,₀ + A₁·x
   * V₂(x) = V₂,₀ + A₂·(L - x)
   *
   * @param position Piston position [m]
   * @return Pair of (V₁, V₂) volumes [m³]
   */
  [[nodiscard]] std::pair<double, double> calculateVolumes(
    double position) const noexcept;

  /**
   * @brief Calculate pressures from ideal gas law
   *
   * p₁ = m₁RT₁/V₁, p₂ = m₂RT₂/V₂
   *
   * @param m1 Mass in chamber 1 [kg]
   * @param T1 Temperature in chamber 1 [K]
   * @param V1 Volume of chamber 1 [m³]
   * @param m2 Mass in chamber 2 [kg]
   * @param T2 Temperature in chamber 2 [K]
   * @param V2 Volume of chamber 2 [m³]
   * @return Pair of (p₁, p₂) pressures [Pa]
   */
  [[nodiscard]] std::pair<double, double> calculatePressures(
    double m1,
    double T1,
    double V1,
    double m2,
    double T2,
    double V2) const noexcept;

  /**
   * @brief Calculate time derivatives for the complete system
   *
   * Implements the full system of ODEs as specified in the mathematical model:
   * ẋ = v
   * v̇ = (A₁p₁ - A₂p₂ - (A₁-A₂)pₐ - F_friction - F_stop)/M
   * ṁ₁ = ṁ₁,in - ṁ₁,out
   * Ṫ₁ = (Ė₁ - CᵥT₁ṁ₁)/(m₁Cᵥ)
   * ṁ₂ = ṁ₂,in - ṁ₂,out
   * Ṫ₂ = (Ė₂ - CᵥT₂ṁ₂)/(m₂Cᵥ)
   * u̇ₖ = (uₖ,target - uₖ)/τ
   *
   * @param state Current state vector [x, v, m₁, T₁, m₂, T₂, u₁, u₂, u₃, u₄]
   * @param time Current time [s]
   * @param valve_controller Controller providing valve commands
   * @param friction_force External friction force [N]
   * @param stop_force External stop force [N]
   * @return Time derivatives vector [ẋ, v̇, ṁ₁, Ṫ₁, ṁ₂, Ṫ₂, u̇₁, u̇₂, u̇₃, u̇₄]
   */
  [[nodiscard]] StateVector calculateDerivatives(
    const StateVector&      state,
    double                  time,
    const IValveController& valve_controller,
    double                  friction_force = 0.0,
    double                  stop_force     = 0.0) const;

  /**
   * @brief Create initial state vector with specified conditions
   * @param position Initial piston position [m]
   * @param velocity Initial piston velocity [m/s]
   * @param pressure1 Initial pressure in chamber 1 [Pa] (will compute mass)
   * @param pressure2 Initial pressure in chamber 2 [Pa] (will compute mass)
   * @param temperature1 Initial temperature in chamber 1 [K]
   * @param temperature2 Initial temperature in chamber 2 [K]
   * @return Complete 10-component initial state vector
   */
  [[nodiscard]] StateVector createInitialState(
    double position     = 0.0,
    double velocity     = 0.0,
    double pressure1    = 1e5,
    double pressure2    = 1e5,
    double temperature1 = 293.15,
    double temperature2 = 293.15) const;

  /**
   * @brief Validate state vector for physical constraints
   * @param state State vector to validate
   * @return true if state satisfies physical constraints
   */
  [[nodiscard]] bool isValidState(const StateVector& state) const noexcept;

  /**
   * @brief Apply physical constraints to state vector
   * @param state State vector to constrain (modified in place)
   */
  void constrainState(StateVector& state) const noexcept;

  // Parameter accessors
  [[nodiscard]] const ThermodynamicParameters& getThermodynamicParams()
    const noexcept
  {
    return m_thermo;
  }
  [[nodiscard]] const GeometryParameters& getGeometryParams() const noexcept
  {
    return m_geom;
  }
  [[nodiscard]] const FluidParameters& getFluidParams() const noexcept
  {
    return m_fluid;
  }

  void setThermodynamicParams(const ThermodynamicParameters& params);
  void setGeometryParams(const GeometryParameters& params);
  void setFluidParams(const FluidParameters& params);

private:
  ThermodynamicParameters m_thermo; ///< Thermodynamic constants
  GeometryParameters      m_geom;   ///< Cylinder geometry
  FluidParameters         m_fluid;  ///< Fluid system parameters

  /**
   * @brief Calculate energy flow rate for a chamber
   * @param mass_flow_in Inlet mass flow rate [kg/s]
   * @param mass_flow_out Outlet mass flow rate [kg/s]
   * @param T_supply Supply temperature [K]
   * @param T_chamber Chamber temperature [K]
   * @param pressure Chamber pressure [Pa]
   * @param area Chamber area [m²]
   * @param velocity Piston velocity [m/s]
   * @return Energy flow rate dE/dt [W]
   */
  [[nodiscard]] double calculateEnergyDerivative(
    double mass_flow_in,
    double mass_flow_out,
    double T_supply,
    double T_chamber,
    double pressure,
    double area,
    double velocity) const noexcept;

  /**
   * @brief Validate parameters and throw if invalid
   */
  void validateParameters() const;
};

} // namespace mz::model::physics