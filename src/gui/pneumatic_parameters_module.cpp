#include "pneumatic_parameters_module.hpp"
#include "core/logger.hpp"

#include <imgui.h>
#include <string>
#include <vector>

void
mz::gui::PneumaticParametersModule::render()
{
  ImGui::Begin("Pneumatic Parameters");

  ImGui::SameLine();
  if (ImGui::Button("Start", ImVec2(100, 20))) {
    MZ_LOG_INFO("Start");
  }

  ImGui::SameLine();
  if (ImGui::Button("Stop", ImVec2(100, 20))) {
    MZ_LOG_INFO("Stop");
  }

  ImGui::Separator();
  if (ImGui::CollapsingHeader("Cylinder parameter")) {
    ImGui::DragFloat(
      "Piston Diameter", &m_piston_diameter, 0.1f, 0.0f, 100.0f, "%.3f, mm");
    ImGui::DragFloat(
      "Rod Diameter", &m_rod_diameter, 0.1f, 0.0f, 100.0f, "%.3f, mm");
  }
  if (ImGui::CollapsingHeader("Gas parameter")) {
    ImGui::DragFloat(
      "Inlet Pressure", &m_in_pressure, 0.1f, 0.0f, 100.0f, "%.3f, bar");
    ImGui::DragFloat(
      "Outlet Pressure", &m_out_pressure, 0.1f, 0.0f, 100.0f, "%.3f, bar");
    ImGui::DragFloat(
      "Inlet Temperature", &m_in_temperature, 0.1f, 0.0f, 100.0f, "%.3f, C");
    ImGui::DragFloat(
      "Outlet Temperature", &m_out_temperature, 0.1f, 0.0f, 100.0f, "%.3f, C");
  }

  ImGui::End();
}