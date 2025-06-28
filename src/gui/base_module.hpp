#pragma once

namespace mz::gui {
class BaseModule
{
public:
  virtual ~BaseModule() = default;

  virtual void render() = 0;
};
}