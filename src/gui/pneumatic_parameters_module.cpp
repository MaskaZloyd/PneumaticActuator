#include "pneumatic_parameters_module.hpp"
#include "core/logger.hpp"

#include <algorithm>
#include <format>
#include <imgui.h>
#include <string>
#include <thread>
#include <vector>

namespace mz::gui {

PneumaticParametersModule::PneumaticParametersModule(
  std::shared_ptr<model::PneumaticModel> pneumatic_model)
  : m_pneumatic_model(pneumatic_model)
{
  // Initialize with model's current configuration
  updateUIFromModel();
  MZ_LOG_INFO("Initialized advanced pneumatic parameters module");
}

void
PneumaticParametersModule::render()
{
  ImGui::Begin("Pneumatic System Configuration");

  // Update simulation state
  m_is_simulating = m_pneumatic_model->isCalculating();

  renderControlButtons();
  ImGui::Separator();

  // Main configuration sections
  renderGeometryParameters();
  renderFluidParameters();

  if (m_show_advanced_thermodynamics) {
    renderThermodynamicParameters();
  }

  renderFrictionParameters();
  renderStopForceParameters();
  renderSolverConfiguration();
  renderInitialConditions();
  renderValveControlConfiguration();
  renderTimeSpanConfiguration();

  ImGui::End();
}

void
PneumaticParametersModule::renderControlButtons()
{
  // Main simulation control
  if (m_is_simulating) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.4f, 0.4f, 1.0f));
    if (ImGui::Button("Stop Simulation", ImVec2(150, 35))) {
      m_pneumatic_model->stopSimulation();
      MZ_LOG_INFO("Simulation stop requested");
    }
    ImGui::PopStyleColor();
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
    if (ImGui::Button("Start Simulation", ImVec2(150, 35))) {
      if (validateConfiguration()) {
        updateModelFromUI();
        configureValveController();

        std::thread([this]() {
          m_pneumatic_model->startSimulation();
        }).detach();

        MZ_LOG_INFO("Started advanced pneumatic simulation");
      }
    }
    ImGui::PopStyleColor();
  }

  ImGui::SameLine();
  if (ImGui::Button("Reset to Defaults", ImVec2(150, 35))) {
    applySystemPreset(SystemPreset::Default);
    MZ_LOG_INFO("Reset to default configuration");
  }

  ImGui::SameLine();
  if (ImGui::Button("Validate Config", ImVec2(150, 35))) {
    validateConfiguration();
  }

  // System presets
  ImGui::Text("Quick Presets:");
  ImGui::SameLine();
  if (ImGui::Combo("##preset",
                   &m_selected_preset,
                   "Default\0High Pressure\0Low Friction\0High Accuracy\0Fast "
                   "Simulation\0")) {
    applySystemPreset(static_cast<SystemPreset>(m_selected_preset));
  }

  // Advanced options toggles
  ImGui::Checkbox("Advanced Thermodynamics", &m_show_advanced_thermodynamics);
  ImGui::SameLine();
  ImGui::Checkbox("Advanced Friction", &m_show_advanced_friction);
  ImGui::SameLine();
  ImGui::Checkbox("Advanced Stop Force", &m_show_advanced_stop_force);
  ImGui::SameLine();
  ImGui::Checkbox("Advanced Solver", &m_show_advanced_solver);
  ImGui::SameLine();
  ImGui::Checkbox("Valve Details", &m_show_valve_details);
}

void
PneumaticParametersModule::renderGeometryParameters()
{
  if (ImGui::CollapsingHeader("Cylinder Geometry",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    // Static variables for DragScalar min/max values
    static double area_min   = 10.0;
    static double area_max   = 2000.0;
    static double length_min = 10.0;
    static double length_max = 1000.0;
    static double mass_min   = 0.1;
    static double mass_max   = 100.0;
    static double volume_min = 0.1;
    static double volume_max = 100.0;

    double a1_mm2            = m_config.geometry.A1 * 1e6; // Convert m² to mm²
    if (ImGui::DragScalar("Chamber 1 Area##A1",
                          ImGuiDataType_Double,
                          &a1_mm2,
                          1.0f,
                          &area_min,
                          &area_max,
                          "%.1f mm²")) {
      m_config.geometry.A1 = a1_mm2 * 1e-6;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Effective piston area for chamber 1");
    }

    double a2_mm2 = m_config.geometry.A2 * 1e6;
    if (ImGui::DragScalar("Chamber 2 Area##A2",
                          ImGuiDataType_Double,
                          &a2_mm2,
                          1.0f,
                          &area_min,
                          &area_max,
                          "%.1f mm²")) {
      m_config.geometry.A2 = a2_mm2 * 1e-6;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Effective piston area for chamber 2 (rod side)");
    }

    double length_mm = m_config.geometry.L * 1000; // Convert m to mm
    if (ImGui::DragScalar("Stroke Length##L",
                          ImGuiDataType_Double,
                          &length_mm,
                          1.0f,
                          &length_min,
                          &length_max,
                          "%.0f mm")) {
      m_config.geometry.L = length_mm * 1e-3;
    }

    ImGui::DragScalar("Total Mass##M",
                      ImGuiDataType_Double,
                      &m_config.geometry.M,
                      0.1f,
                      &mass_min,
                      &mass_max,
                      "%.1f kg");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Combined mass of piston, rod, and load");
    }

    double v1_ml = m_config.geometry.V1_0 * 1e6; // Convert m³ to ml
    double v2_ml = m_config.geometry.V2_0 * 1e6;
    if (ImGui::DragScalar("Dead Volume 1##V1_0",
                          ImGuiDataType_Double,
                          &v1_ml,
                          0.1f,
                          &volume_min,
                          &volume_max,
                          "%.1f ml")) {
      m_config.geometry.V1_0 = v1_ml * 1e-6;
    }
    if (ImGui::DragScalar("Dead Volume 2##V2_0",
                          ImGuiDataType_Double,
                          &v2_ml,
                          0.1f,
                          &volume_min,
                          &volume_max,
                          "%.1f ml")) {
      m_config.geometry.V2_0 = v2_ml * 1e-6;
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderFluidParameters()
{
  if (ImGui::CollapsingHeader("Fluid System", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    // Static variables for DragScalar min/max values
    static double pressure_min     = 1.0;
    static double pressure_max     = 20.0;
    static double atm_pressure_min = 0.5;
    static double atm_pressure_max = 2.0;
    static double temp_min         = -20.0;
    static double temp_max         = 100.0;
    static double coeff_min        = 0.1;
    static double coeff_max        = 1.0;
    static double area_valve_min   = 0.1;
    static double area_valve_max   = 10.0;
    static double tau_min          = 0.01;
    static double tau_max          = 1.0;

    double ps_bar = m_config.fluid.p_s * 1e-5; // Convert Pa to bar
    if (ImGui::DragScalar("Supply Pressure##ps",
                          ImGuiDataType_Double,
                          &ps_bar,
                          0.1f,
                          &pressure_min,
                          &pressure_max,
                          "%.1f bar")) {
      m_config.fluid.p_s = ps_bar * 1e5;
    }

    double pa_bar = m_config.fluid.p_a * 1e-5;
    if (ImGui::DragScalar("Atmospheric Pressure##pa",
                          ImGuiDataType_Double,
                          &pa_bar,
                          0.01f,
                          &atm_pressure_min,
                          &atm_pressure_max,
                          "%.2f bar")) {
      m_config.fluid.p_a = pa_bar * 1e5;
    }

    double ts_celsius = m_config.fluid.T_s - 273.15; // Convert K to °C
    if (ImGui::DragScalar("Supply Temperature##Ts",
                          ImGuiDataType_Double,
                          &ts_celsius,
                          1.0f,
                          &temp_min,
                          &temp_max,
                          "%.1f °C")) {
      m_config.fluid.T_s = ts_celsius + 273.15;
    }

    double ta_celsius = m_config.fluid.T_a - 273.15;
    if (ImGui::DragScalar("Ambient Temperature##Ta",
                          ImGuiDataType_Double,
                          &ta_celsius,
                          1.0f,
                          &temp_min,
                          &temp_max,
                          "%.1f °C")) {
      m_config.fluid.T_a = ta_celsius + 273.15;
    }

    // Valve characteristics
    if (ImGui::TreeNode("Valve Characteristics")) {
      ImGui::Text("Discharge Coefficients:");
      ImGui::DragScalar("Cd1 (Inlet 1)##Cd1",
                        ImGuiDataType_Double,
                        &m_config.fluid.Cd1,
                        0.01f,
                        &coeff_min,
                        &coeff_max,
                        "%.2f");
      ImGui::DragScalar("Cd2 (Outlet 1)##Cd2",
                        ImGuiDataType_Double,
                        &m_config.fluid.Cd2,
                        0.01f,
                        &coeff_min,
                        &coeff_max,
                        "%.2f");
      ImGui::DragScalar("Cd3 (Inlet 2)##Cd3",
                        ImGuiDataType_Double,
                        &m_config.fluid.Cd3,
                        0.01f,
                        &coeff_min,
                        &coeff_max,
                        "%.2f");
      ImGui::DragScalar("Cd4 (Outlet 2)##Cd4",
                        ImGuiDataType_Double,
                        &m_config.fluid.Cd4,
                        0.01f,
                        &coeff_min,
                        &coeff_max,
                        "%.2f");
      ImGui::Text("Valve Areas:");
      double av1_mm2 = m_config.fluid.Av1 * 1e6;
      if (ImGui::DragScalar("Av1 (Inlet 1)##Av1",
                            ImGuiDataType_Double,
                            &av1_mm2,
                            0.01f,
                            &area_valve_min,
                            &area_valve_max,
                            "%.2f mm²")) {
        m_config.fluid.Av1 = av1_mm2 * 1e-6;
      }
      double av2_mm2 = m_config.fluid.Av2 * 1e6;
      if (ImGui::DragScalar("Av2 (Outlet 1)##Av2",
                            ImGuiDataType_Double,
                            &av2_mm2,
                            0.01f,
                            &area_valve_min,
                            &area_valve_max,
                            "%.2f mm²")) {
        m_config.fluid.Av2 = av2_mm2 * 1e-6;
      }
      double av3_mm2 = m_config.fluid.Av3 * 1e6;
      if (ImGui::DragScalar("Av3 (Inlet 2)##Av3",
                            ImGuiDataType_Double,
                            &av3_mm2,
                            0.01f,
                            &area_valve_min,
                            &area_valve_max,
                            "%.2f mm²")) {
        m_config.fluid.Av3 = av3_mm2 * 1e-6;
      }
      double av4_mm2 = m_config.fluid.Av4 * 1e6;
      if (ImGui::DragScalar("Av4 (Outlet 2)##Av4",
                            ImGuiDataType_Double,
                            &av4_mm2,
                            0.01f,
                            &area_valve_min,
                            &area_valve_max,
                            "%.2f mm²")) {
        m_config.fluid.Av4 = av4_mm2 * 1e-6;
      }

      ImGui::DragScalar("Valve Time Constant##tau",
                        ImGuiDataType_Double,
                        &m_config.fluid.tau,
                        0.01f,
                        &tau_min,
                        &tau_max,
                        "%.3f s");
      ImGui::TreePop();
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderThermodynamicParameters()
{
  if (ImGui::CollapsingHeader("Thermodynamic Properties")) {
    ImGui::PushItemWidth(200);

    // Static variables for DragScalar min/max values
    static double gamma_min       = 1.1;
    static double gamma_max       = 2.0;
    static double r_min           = 200.0;
    static double r_max           = 400.0;
    static double min_mass_mg_min = 0.001;
    static double min_mass_mg_max = 10.0;
    static double min_temp_min    = 1.0;
    static double min_temp_max    = 100.0;
    static double min_press_min   = 1.0;
    static double min_press_max   = 100.0;

    ImGui::Text("Air Properties:");
    ImGui::DragScalar("Heat Capacity Ratio##gamma",
                      ImGuiDataType_Double,
                      &m_config.thermo.gamma,
                      0.01f,
                      &gamma_min,
                      &gamma_max,
                      "%.3f");
    ImGui::DragScalar("Gas Constant##R",
                      ImGuiDataType_Double,
                      &m_config.thermo.R,
                      1.0f,
                      &r_min,
                      &r_max,
                      "%.1f J/(kg·K)");

    ImGui::Text("Minimum Thresholds:");
    double min_mass_mg = m_config.thermo.min_mass * 1e6; // Convert kg to mg
    if (ImGui::DragScalar("Min Mass##min_mass",
                          ImGuiDataType_Double,
                          &min_mass_mg,
                          0.001f,
                          &min_mass_mg_min,
                          &min_mass_mg_max,
                          "%.3f mg")) {
      m_config.thermo.min_mass = min_mass_mg * 1e-6;
    }
    ImGui::DragScalar("Min Temperature##min_temp",
                      ImGuiDataType_Double,
                      &m_config.thermo.min_temperature,
                      1.0f,
                      &min_temp_min,
                      &min_temp_max,
                      "%.1f K");
    double min_press_mbar =
      m_config.thermo.min_pressure * 1e-2; // Convert Pa to mbar
    if (ImGui::DragScalar("Min Pressure##min_press",
                          ImGuiDataType_Double,
                          &min_press_mbar,
                          1.0f,
                          &min_press_min,
                          &min_press_max,
                          "%.1f mbar")) {
      m_config.thermo.min_pressure = min_press_mbar * 1e2;
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderFrictionParameters()
{
  if (ImGui::CollapsingHeader("LuGre Friction Model",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    // Static variables for DragScalar min/max values
    static double friction_min  = 0.0;
    static double friction_max  = 500.0;
    static double velocity_min  = 0.001;
    static double velocity_max  = 1.0;
    static double stiffness_min = 100.0;
    static double stiffness_max = 10000.0;
    static double damping_min   = 0.0;
    static double damping_max   = 1000.0;
    static double viscous_min   = 0.0;
    static double viscous_max   = 10.0;

    ImGui::DragScalar("Coulomb Friction##Fc",
                      ImGuiDataType_Double,
                      &m_config.friction.Fc,
                      1.0f,
                      &friction_min,
                      &friction_max,
                      "%.1f N");
    ImGui::DragScalar("Static Friction##Fs",
                      ImGuiDataType_Double,
                      &m_config.friction.Fs,
                      1.0f,
                      &friction_min,
                      &friction_max,
                      "%.1f N");
    ImGui::DragScalar("Stribeck Velocity##vs",
                      ImGuiDataType_Double,
                      &m_config.friction.vs,
                      0.001f,
                      &velocity_min,
                      &velocity_max,
                      "%.3f m/s");

    if (m_show_advanced_friction) {
      ImGui::Separator();
      ImGui::Text("Advanced Friction Parameters:");
      ImGui::DragScalar("Bristle Stiffness##sigma0",
                        ImGuiDataType_Double,
                        &m_config.friction.sigma0,
                        10.0f,
                        &stiffness_min,
                        &stiffness_max,
                        "%.0f N/m");
      ImGui::DragScalar("Bristle Damping##sigma1",
                        ImGuiDataType_Double,
                        &m_config.friction.sigma1,
                        1.0f,
                        &damping_min,
                        &damping_max,
                        "%.1f Ns/m");
      ImGui::DragScalar("Micro-Viscous##sigma2",
                        ImGuiDataType_Double,
                        &m_config.friction.sigma2,
                        0.01f,
                        &viscous_min,
                        &viscous_max,
                        "%.2f Ns/m");
      ImGui::DragScalar("Viscous Damping##B",
                        ImGuiDataType_Double,
                        &m_config.friction.B,
                        1.0f,
                        &damping_min,
                        &damping_max,
                        "%.1f Ns/m");

      static double epsilon_min = 0.001;
      static double epsilon_max = 1.0;
      double        epsilon_mm_s =
        m_config.friction.epsilon * 1000; // Convert m/s to mm/s
      if (ImGui::DragScalar("Regularization##epsilon",
                            ImGuiDataType_Double,
                            &epsilon_mm_s,
                            0.001f,
                            &epsilon_min,
                            &epsilon_max,
                            "%.3f mm/s")) {
        m_config.friction.epsilon = epsilon_mm_s * 1e-3;
      }
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderStopForceParameters()
{
  if (ImGui::CollapsingHeader("Stop Force Model",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    // Static variables for DragScalar min/max values
    static double position_min  = -10.0;
    static double position_max  = 1000.0;
    static double stiffness_min = 1000.0;
    static double stiffness_max = 1e8;
    static double damping_min   = 100.0;
    static double damping_max   = 1e6;
    static double alpha_min     = 100.0;
    static double alpha_max     = 1e6;
    static double epsilon_min   = 1e-9;
    static double epsilon_max   = 1e-3;

    // Position limits
    double x_min_mm = m_config.stop.x_min * 1000; // Convert m to mm
    if (ImGui::DragScalar("Minimum Position##x_min",
                          ImGuiDataType_Double,
                          &x_min_mm,
                          0.1f,
                          &position_min,
                          &position_max,
                          "%.1f mm")) {
      m_config.stop.x_min = x_min_mm * 1e-3;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Lower position limit - stop force activates when "
                        "approaching this limit");
    }

    double x_max_mm = m_config.stop.x_max * 1000; // Convert m to mm
    if (ImGui::DragScalar("Maximum Position##x_max",
                          ImGuiDataType_Double,
                          &x_max_mm,
                          0.1f,
                          &position_min,
                          &position_max,
                          "%.1f mm")) {
      m_config.stop.x_max = x_max_mm * 1e-3;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Upper position limit - stop force activates when "
                        "approaching this limit");
    }

    // Basic stop force parameters
    double k_stop_kn_m = m_config.stop.k_stop / 1000; // Convert N/m to kN/m
    if (ImGui::DragScalar("Stop Stiffness##k_stop",
                          ImGuiDataType_Double,
                          &k_stop_kn_m,
                          10.0f,
                          &stiffness_min,
                          &stiffness_max,
                          "%.0f kN/m")) {
      m_config.stop.k_stop = k_stop_kn_m * 1000;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Elastic stiffness coefficient for position limits");
    }

    double c_stop_kns_m = m_config.stop.c_stop / 1000; // Convert Ns/m to kNs/m
    if (ImGui::DragScalar("Stop Damping##c_stop",
                          ImGuiDataType_Double,
                          &c_stop_kns_m,
                          1.0f,
                          &damping_min,
                          &damping_max,
                          "%.1f kNs/m")) {
      m_config.stop.c_stop = c_stop_kns_m * 1000;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
        "Velocity-dependent damping coefficient for position limits");
    }

    if (m_show_advanced_stop_force) {
      ImGui::Separator();
      ImGui::Text("Advanced Stop Force Parameters:");

      ImGui::DragScalar("Smoothness Factor##alpha",
                        ImGuiDataType_Double,
                        &m_config.stop.alpha,
                        100.0f,
                        &alpha_min,
                        &alpha_max,
                        "%.0f 1/m");
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Controls transition smoothness - higher values give "
                          "sharper transitions");
      }

      double epsilon_um = m_config.stop.epsilon * 1e6; // Convert m to mum
      if (ImGui::DragScalar("Regularization##epsilon",
                            ImGuiDataType_Double,
                            &epsilon_um,
                            0.01f,
                            &epsilon_min,
                            &epsilon_max,
                            "%.2f μm")) {
        m_config.stop.epsilon = epsilon_um * 1e-6;
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Small parameter to prevent numerical issues");
      }
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderSolverConfiguration()
{
  if (ImGui::CollapsingHeader("ODE Solver", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    const char* solver_names[] = { "Runge-Kutta 4",  "Dormand-Prince 5",
                                   "Cash-Karp 5(4)", "Fehlberg 7(8)",
                                   "Bulirsch-Stoer", "Rosenbrock 4" };

    if (ImGui::Combo(
          "Solver Type##solver", &m_solver_type_index, solver_names, 6)) {
      m_config.solver.type =
        static_cast<solver::SolverType>(m_solver_type_index);
    }

    if (m_show_advanced_solver) {
      // Static variables for DragScalar min/max values
      static double abs_tol_min   = -15.0;
      static double abs_tol_max   = -3.0;
      static double rel_tol_min   = -12.0;
      static double rel_tol_max   = -1.0;
      static double init_step_min = 0.001;
      static double init_step_max = 100.0;
      static double max_step_min  = 0.1;
      static double max_step_max  = 1000.0;

      ImGui::Text("Error Tolerances:");
      double abs_tol_log = std::log10(m_config.solver.abs_tolerance);
      if (ImGui::DragScalar("Absolute Tolerance##abs_tol",
                            ImGuiDataType_Double,
                            &abs_tol_log,
                            0.1f,
                            &abs_tol_min,
                            &abs_tol_max,
                            "1e%.0f")) {
        m_config.solver.abs_tolerance = std::pow(10.0, abs_tol_log);
      }

      double rel_tol_log = std::log10(m_config.solver.rel_tolerance);
      if (ImGui::DragScalar("Relative Tolerance##rel_tol",
                            ImGuiDataType_Double,
                            &rel_tol_log,
                            0.1f,
                            &rel_tol_min,
                            &rel_tol_max,
                            "1e%.0f")) {
        m_config.solver.rel_tolerance = std::pow(10.0, rel_tol_log);
      }

      ImGui::Text("Step Size Control:");
      double init_step_ms =
        m_config.solver.initial_step_size * 1000; // Convert s to ms
      if (ImGui::DragScalar("Initial Step##init_step",
                            ImGuiDataType_Double,
                            &init_step_ms,
                            0.01f,
                            &init_step_min,
                            &init_step_max,
                            "%.3f ms")) {
        m_config.solver.initial_step_size = init_step_ms * 1e-3;
      }

      double max_step_ms = m_config.solver.max_step_size * 1000;
      if (ImGui::DragScalar("Max Step##max_step",
                            ImGuiDataType_Double,
                            &max_step_ms,
                            1.0f,
                            &max_step_min,
                            &max_step_max,
                            "%.1f ms")) {
        m_config.solver.max_step_size = max_step_ms * 1e-3;
      }

      int max_steps_k = static_cast<int>(m_config.solver.max_steps / 1000);
      if (ImGui::DragInt(
            "Max Steps##max_steps", &max_steps_k, 1, 1, 10000, "%d k")) {
        m_config.solver.max_steps = max_steps_k * 1000;
      }
    } else {
      // Simplified controls
      ImGui::Text("Error Tolerance: %.0e", m_config.solver.abs_tolerance);
      ImGui::Text("Step Size: %.3f ms",
                  m_config.solver.initial_step_size * 1000);
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderInitialConditions()
{
  if (ImGui::CollapsingHeader("Initial Conditions",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    // Static variables for DragScalar min/max values
    static double pos_min   = -500.0;
    static double pos_max   = 500.0;
    static double vel_min   = -1000.0;
    static double vel_max   = 1000.0;
    static double press_min = 0.1;
    static double press_max = 20.0;

    double pos_mm = m_initial_conditions.position * 1000; // Convert m to mm
    if (ImGui::DragScalar("Position##init_pos",
                          ImGuiDataType_Double,
                          &pos_mm,
                          1.0f,
                          &pos_min,
                          &pos_max,
                          "%.1f mm")) {
      m_initial_conditions.position = pos_mm * 1e-3;
    }

    double vel_mm_s =
      m_initial_conditions.velocity * 1000; // Convert m/s to mm/s
    if (ImGui::DragScalar("Velocity##init_vel",
                          ImGuiDataType_Double,
                          &vel_mm_s,
                          1.0f,
                          &vel_min,
                          &vel_max,
                          "%.1f mm/s")) {
      m_initial_conditions.velocity = vel_mm_s * 1e-3;
    }

    double p1_bar = m_initial_conditions.pressure1 * 1e-5; // Convert Pa to bar
    if (ImGui::DragScalar("Pressure 1##init_p1",
                          ImGuiDataType_Double,
                          &p1_bar,
                          0.1f,
                          &press_min,
                          &press_max,
                          "%.1f bar")) {
      m_initial_conditions.pressure1 = p1_bar * 1e5;
    }

    static double temp_min = -20.0;
    static double temp_max = 100.0;

    double p2_bar          = m_initial_conditions.pressure2 * 1e-5;
    if (ImGui::DragScalar("Pressure 2##init_p2",
                          ImGuiDataType_Double,
                          &p2_bar,
                          0.1f,
                          &press_min,
                          &press_max,
                          "%.1f bar")) {
      m_initial_conditions.pressure2 = p2_bar * 1e5;
    }

    double t1_celsius =
      m_initial_conditions.temperature1 - 273.15; // Convert K to deg C
    if (ImGui::DragScalar("Temperature 1##init_T1",
                          ImGuiDataType_Double,
                          &t1_celsius,
                          1.0f,
                          &temp_min,
                          &temp_max,
                          "%.1f °C")) {
      m_initial_conditions.temperature1 = t1_celsius + 273.15;
    }

    double t2_celsius = m_initial_conditions.temperature2 - 273.15;
    if (ImGui::DragScalar("Temperature 2##init_T2",
                          ImGuiDataType_Double,
                          &t2_celsius,
                          1.0f,
                          &temp_min,
                          &temp_max,
                          "%.1f °C")) {
      m_initial_conditions.temperature2 = t2_celsius + 273.15;
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderValveControlConfiguration()
{
  if (ImGui::CollapsingHeader("Valve Control Strategy",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    const char* control_modes[] = { "Constant Opening",
                                    "Step Input",
                                    "Pressure Regulation" };
    int         mode_index      = static_cast<int>(m_valve_mode);

    if (ImGui::Combo(
          "Control Mode##valve_mode", &mode_index, control_modes, 3)) {
      m_valve_mode =
        static_cast<model::ConfigurableValveController::ControlMode>(
          mode_index);
    }

    switch (m_valve_mode) {
      case model::ConfigurableValveController::ControlMode::ConstantOpening:
        ImGui::Text("Valve Openings (0-1):");
        ImGui::DragFloat4("Valves [1,2,3,4]##const_valves",
                          m_constant_valve_openings.data(),
                          0.01f,
                          0.0f,
                          1.0f,
                          "%.2f");
        break;

      case model::ConfigurableValveController::ControlMode::StepInput:
        ImGui::DragFloat(
          "Step Time##step_time", &m_step_time, 0.01f, 0.0f, 10.0f, "%.2f s");
        ImGui::Text("Before Step:");
        ImGui::DragFloat4("Valves [1,2,3,4]##before",
                          m_before_openings.data(),
                          0.01f,
                          0.0f,
                          1.0f,
                          "%.2f");
        ImGui::Text("After Step:");
        ImGui::DragFloat4("Valves [1,2,3,4]##after",
                          m_after_openings.data(),
                          0.01f,
                          0.0f,
                          1.0f,
                          "%.2f");
        break;

      case model::ConfigurableValveController::ControlMode::PressureRegulation:
        float target_p1_bar = m_target_pressure1 * 1e-5f;
        if (ImGui::DragFloat("Target Pressure 1##target_p1",
                             &target_p1_bar,
                             0.1f,
                             1.0f,
                             20.0f,
                             "%.1f bar")) {
          m_target_pressure1 = target_p1_bar * 1e5f;
        }
        float target_p2_bar = m_target_pressure2 * 1e-5f;
        if (ImGui::DragFloat("Target Pressure 2##target_p2",
                             &target_p2_bar,
                             0.1f,
                             1.0f,
                             20.0f,
                             "%.1f bar")) {
          m_target_pressure2 = target_p2_bar * 1e5f;
        }
        ImGui::DragFloat("Control Gain##control_gain",
                         &m_control_gain,
                         0.01f,
                         0.01f,
                         1.0f,
                         "%.2f");
        break;
    }

    if (m_show_valve_details) {
      ImGui::Separator();
      ImGui::Text("Valve Layout:");
      ImGui::Text("1: Supply -> Chamber 1    2: Chamber 1 -> Exhaust");
      ImGui::Text("3: Supply -> Chamber 2    4: Chamber 2 -> Exhaust");
    }

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::renderTimeSpanConfiguration()
{
  if (ImGui::CollapsingHeader("Simulation Time",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    float t_start = static_cast<float>(m_time_span.first);
    float t_end   = static_cast<float>(m_time_span.second);

    if (ImGui::DragFloat(
          "Start Time##t_start", &t_start, 0.01f, 0.0f, 100.0f, "%.2f s")) {
      m_time_span.first = t_start;
    }

    if (ImGui::DragFloat(
          "End Time##t_end", &t_end, 0.1f, 0.1f, 100.0f, "%.2f s")) {
      m_time_span.second = t_end;
    }

    float duration = t_end - t_start;
    ImGui::Text("Duration: %.2f s", duration);

    ImGui::PopItemWidth();
  }
}

void
PneumaticParametersModule::applySystemPreset(SystemPreset preset)
{
  switch (preset) {
    case SystemPreset::Default:
      m_config             = model::SimulationConfig{};
      m_initial_conditions = model::InitialConditions{};
      m_solver_type_index  = 1; // Dormand-Prince
      break;

    case SystemPreset::HighPressure:
      m_config                       = model::SimulationConfig{};
      m_config.fluid.p_s             = 10e5; // 10 bar
      m_config.fluid.p_a             = 1e5;  // 1 bar
      m_initial_conditions.pressure1 = 8e5;
      m_initial_conditions.pressure2 = 2e5;
      break;

    case SystemPreset::LowFriction:
      m_config             = model::SimulationConfig{};
      m_config.friction.Fc = 10.0;
      m_config.friction.Fs = 15.0;
      m_config.friction.B  = 50.0;
      break;

    case SystemPreset::HighAccuracy:
      m_config                      = model::SimulationConfig{};
      m_config.solver.abs_tolerance = 1e-12;
      m_config.solver.rel_tolerance = 1e-9;
      m_config.solver.type          = solver::SolverType::RungeKuttaFehlberg78;
      m_solver_type_index           = 3;
      break;

    case SystemPreset::FastSimulation:
      m_config                      = model::SimulationConfig{};
      m_config.solver.abs_tolerance = 1e-6;
      m_config.solver.rel_tolerance = 1e-3;
      m_config.solver.type          = solver::SolverType::RungeKutta4;
      m_solver_type_index           = 0;
      break;
  }

  // Update the model with new configuration
  updateModelFromUI();
}

void
PneumaticParametersModule::updateModelFromUI()
{
  try {
    m_pneumatic_model->setConfiguration(m_config);
    m_pneumatic_model->setInitialConditions(m_initial_conditions);
    m_pneumatic_model->setTimeSpan(m_time_span.first, m_time_span.second);
  } catch (const std::exception& e) {
    MZ_LOG_ERROR(
      std::format("Failed to update model configuration: {}", e.what()));
  }
}

void
PneumaticParametersModule::updateUIFromModel()
{
  m_config             = m_pneumatic_model->getConfiguration();
  m_initial_conditions = m_pneumatic_model->getInitialConditions();
  m_time_span          = m_pneumatic_model->getTimeSpan();
  m_solver_type_index  = static_cast<int>(m_config.solver.type);
}

void
PneumaticParametersModule::configureValveController()
{
  auto controller = std::make_shared<model::ConfigurableValveController>();

  switch (m_valve_mode) {
    case model::ConfigurableValveController::ControlMode::ConstantOpening: {
      std::array<double, 4> openings;
      std::transform(m_constant_valve_openings.begin(),
                     m_constant_valve_openings.end(),
                     openings.begin(),
                     [](float f) { return static_cast<double>(f); });
      controller =
        std::make_shared<model::ConfigurableValveController>(openings);
      break;
    }

    case model::ConfigurableValveController::ControlMode::StepInput: {
      std::array<double, 4> before, after;
      std::transform(m_before_openings.begin(),
                     m_before_openings.end(),
                     before.begin(),
                     [](float f) { return static_cast<double>(f); });
      std::transform(m_after_openings.begin(),
                     m_after_openings.end(),
                     after.begin(),
                     [](float f) { return static_cast<double>(f); });
      controller->configureStepInput(m_step_time, before, after);
      break;
    }

    case model::ConfigurableValveController::ControlMode::PressureRegulation:
      controller->configurePressureRegulation(
        m_target_pressure1, m_target_pressure2, m_control_gain);
      break;
  }

  m_pneumatic_model->setValveController(controller);
}

bool
PneumaticParametersModule::validateConfiguration()
{
  bool valid = true;

  if (!m_config.isValid()) {
    MZ_LOG_WARN("Invalid simulation configuration detected");
    valid = false;
  }

  if (!m_initial_conditions.isValid()) {
    MZ_LOG_WARN("Invalid initial conditions detected");
    valid = false;
  }

  if (m_time_span.second <= m_time_span.first) {
    MZ_LOG_WARN("Invalid time span: end time must be greater than start time");
    valid = false;
  }

  if (valid) {
    MZ_LOG_INFO("Configuration validation passed");
  }

  return valid;
}

const char*
PneumaticParametersModule::getSolverTypeName(int type)
{
  const char* names[] = { "Runge-Kutta 4", "Dormand-Prince 5", "Cash-Karp 5(4)",
                          "Fehlberg 7(8)", "Bulirsch-Stoer",   "Rosenbrock 4" };
  return (type >= 0 && type < 6) ? names[type] : "Unknown";
}

const char*
PneumaticParametersModule::getPresetName(SystemPreset preset)
{
  switch (preset) {
    case SystemPreset::Default:
      return "Default";
    case SystemPreset::HighPressure:
      return "High Pressure";
    case SystemPreset::LowFriction:
      return "Low Friction";
    case SystemPreset::HighAccuracy:
      return "High Accuracy";
    case SystemPreset::FastSimulation:
      return "Fast Simulation";
    default:
      return "Unknown";
  }
}

}