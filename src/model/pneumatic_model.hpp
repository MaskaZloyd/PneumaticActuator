#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace mz::model {
struct PneumaticParameters
{
  double piston_diameter{ 0.0 };
  double rod_diameter{ 0.0 };
  double in_pressure{ 0.0 };
  double out_pressure{ 0.0 };
  double in_temperature{ 0.0 };
  double out_temperature{ 0.0 };
};

struct CalculationResult
{};

class PnematicModel final
{
public:
  void startCalculation();
  void stopCalculation();

  void setParameters(const PneumaticParameters& parameters);
  [[nodiscard]] PneumaticParameters getParameters() const;

private:
  PneumaticParameters m_parameters;
  CalculationResult   m_result;
};
}