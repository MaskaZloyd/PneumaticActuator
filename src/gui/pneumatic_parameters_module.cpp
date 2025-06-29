#include "pneumatic_parameters_module.hpp"
#include "core/logger.hpp"

#include <format>
#include <imgui.h>
#include <string>
#include <thread>
#include <vector>

mz::gui::PneumaticParametersModule::PneumaticParametersModule(
  std::shared_ptr<model::PneumaticModel> pneumatic_model)
  : m_pneumatic_parameters{}
  , m_friction_parameters{}
  , m_stop_force_parameters{}
  , m_pneumatic_model(pneumatic_model)
  , m_solver_config{}
  , m_initial_state{ (Eigen::VectorXd(2) << 0.0, 0.0).finished() }
  , m_t_span{ 0.0, 1.0 }
  , m_step_size_ui{ 0.001f }
  , m_initial_position_ui{ 0.0f }
  , m_initial_velocity_ui{ 0.0f }
  , m_t_start_ui{ 0.0f }
  , m_t_end_ui{ 1.0f }
  , m_is_calculating{ false }
{
  initialize_ui_from_config();
}

void
mz::gui::PneumaticParametersModule::render()
{
  ImGui::Begin("Pneumatic Parameters");

  // Control buttons
  if (m_is_calculating) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.4f, 0.4f, 1.0f));
    if (ImGui::Button("Calculating...", ImVec2(120, 30))) {
      // Could add stop functionality here
    }
    ImGui::PopStyleColor();
  } else {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.4f, 0.8f, 0.4f, 1.0f));
    if (ImGui::Button("Start Calculation", ImVec2(120, 30))) {
      update_solver_config_from_ui();
      update_initial_state_from_ui();
      update_t_span_from_ui();

      m_pneumatic_model->setParameters(m_pneumatic_parameters);
      m_pneumatic_model->setSolverConfig(m_solver_config);
      m_pneumatic_model->setInitialState(m_initial_state);
      m_pneumatic_model->setTspan(m_t_span);

      MZ_LOG_INFO(std::format(
        "Starting pneumatic actuator calculation with parameters: "
        "Piston Ø={:.1f}mm, Rod Ø={:.1f}mm, Pin={:.1f}bar, Pout={:.1f}bar, "
        "Mass={:.1f}kg, Stroke={:.1f}mm, Step={:.4f}s, Time=[{:.2f}, {:.2f}]s",
        m_pneumatic_parameters.piston_diameter,
        m_pneumatic_parameters.rod_diameter,
        m_pneumatic_parameters.in_pressure,
        m_pneumatic_parameters.out_pressure,
        m_pneumatic_parameters.mass,
        m_pneumatic_parameters.stroke,
        m_solver_config.step_size,
        m_t_span.first,
        m_t_span.second));

      m_is_calculating = true;
      std::thread([this]() {
        m_pneumatic_model->startCalculation();
        m_is_calculating = false;
      }).detach();
    }
    ImGui::PopStyleColor();
  }

  ImGui::SameLine();
  if (ImGui::Button("Reset Parameters", ImVec2(120, 30))) {
    m_pneumatic_parameters = {};
    m_solver_config        = {};
    m_initial_state        = (Eigen::VectorXd(2) << 0.0, 0.0).finished();
    m_t_span               = { 0.0, 1.0 };
    initialize_ui_from_config();
    MZ_LOG_INFO("Parameters reset to defaults");
  }

  ImGui::Separator();

  // Cylinder Parameters
  if (ImGui::CollapsingHeader("Cylinder Parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    ImGui::DragFloat("Piston Diameter##piston",
                     &m_pneumatic_parameters.piston_diameter,
                     0.1f,
                     1.0f,
                     200.0f,
                     "%.1f mm");
    ImGui::SameLine();
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Internal diameter of the cylinder piston");
    }

    ImGui::DragFloat("Rod Diameter##rod",
                     &m_pneumatic_parameters.rod_diameter,
                     0.1f,
                     1.0f,
                     100.0f,
                     "%.1f mm");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Diameter of the piston rod");
    }

    ImGui::DragFloat("Stroke Length##stroke",
                     &m_pneumatic_parameters.stroke,
                     1.0f,
                     10.0f,
                     1000.0f,
                     "%.1f mm");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Maximum travel distance of the piston");
    }

    ImGui::PopItemWidth();
  }

  // Gas Parameters
  if (ImGui::CollapsingHeader("Gas Parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    ImGui::DragFloat("Inlet Pressure##pin",
                     &m_pneumatic_parameters.in_pressure,
                     0.1f,
                     0.1f,
                     20.0f,
                     "%.1f bar");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Supply pressure to the cylinder");
    }

    ImGui::DragFloat("Outlet Pressure##pout",
                     &m_pneumatic_parameters.out_pressure,
                     0.1f,
                     0.0f,
                     10.0f,
                     "%.1f bar");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Exhaust pressure from the cylinder");
    }

    ImGui::PopItemWidth();
  }

  // Load Parameters
  if (ImGui::CollapsingHeader("Load Parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    ImGui::DragFloat("Load Mass##mass",
                     &m_pneumatic_parameters.mass,
                     0.1f,
                     0.1f,
                     100.0f,
                     "%.1f kg");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Mass of the load being moved by the actuator");
    }

    ImGui::PopItemWidth();
  }

  // Initial Conditions
  if (ImGui::CollapsingHeader("Initial Conditions",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    ImGui::DragFloat("Initial Position##init_pos",
                     &m_initial_position_ui,
                     0.001f,
                     -1.0f,
                     1.0f,
                     "%.3f m");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Starting position of the piston");
    }

    ImGui::DragFloat("Initial Velocity##init_vel",
                     &m_initial_velocity_ui,
                     0.001f,
                     -5.0f,
                     5.0f,
                     "%.3f m/s");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Starting velocity of the piston");
    }

    ImGui::PopItemWidth();
  }

  // Time Parameters
  if (ImGui::CollapsingHeader("Time Parameters",
                              ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::PushItemWidth(200);

    ImGui::DragFloat(
      "Start Time##t_start", &m_t_start_ui, 0.01f, 0.0f, 10.0f, "%.2f s");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Simulation start time");
    }

    ImGui::DragFloat(
      "End Time##t_end", &m_t_end_ui, 0.01f, 0.1f, 100.0f, "%.2f s");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Simulation end time");
    }

    // Ensure t_end > t_start
    if (m_t_end_ui <= m_t_start_ui) {
      m_t_end_ui = m_t_start_ui + 0.1f;
    }

    ImGui::PopItemWidth();
  }

  // Solver Parameters
  if (ImGui::CollapsingHeader("Solver Parameters")) {
    ImGui::PushItemWidth(200);

    ImGui::DragFloat("Step Size##step_size",
                     &m_step_size_ui,
                     0.0001f,
                     0.0001f,
                     0.1f,
                     "%.4f s");
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
        "Numerical integration step size (smaller = more accurate but slower)");
    }

    ImGui::PopItemWidth();
  }

  ImGui::End();
}

void
mz::gui::PneumaticParametersModule::update_solver_config_from_ui()
{
  m_solver_config.step_size = static_cast<double>(m_step_size_ui);
}

void
mz::gui::PneumaticParametersModule::update_initial_state_from_ui()
{
  m_initial_state(0) = static_cast<double>(m_initial_position_ui);
  m_initial_state(1) = static_cast<double>(m_initial_velocity_ui);
}

void
mz::gui::PneumaticParametersModule::update_t_span_from_ui()
{
  m_t_span.first  = static_cast<double>(m_t_start_ui);
  m_t_span.second = static_cast<double>(m_t_end_ui);
}

void
mz::gui::PneumaticParametersModule::initialize_ui_from_config()
{
  m_step_size_ui        = static_cast<float>(m_solver_config.step_size);
  m_initial_position_ui = static_cast<float>(m_initial_state(0));
  m_initial_velocity_ui = static_cast<float>(m_initial_state(1));
  m_t_start_ui          = static_cast<float>(m_t_span.first);
  m_t_end_ui            = static_cast<float>(m_t_span.second);
}