#pragma once

#include "base_module.hpp"

namespace mz::gui {
class PneumaticParametersModule final : public BaseModule
{
public:
  void render() override;

private:
  float m_piston_diameter{ 0.0 };
  float m_rod_diameter{ 0.0 };

  float m_in_pressure{ 0.0 };
  float m_out_pressure{ 0.0 };
  float m_in_temperature{ 0.0 };
  float m_out_temperature{ 0.0 };
};
}