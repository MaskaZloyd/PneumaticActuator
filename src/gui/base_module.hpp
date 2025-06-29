#pragma once

namespace mz::gui {

/**
 * @brief Abstract base class for GUI modules.
 *
 * All GUI modules should inherit from this class and implement the render()
 * method.
 */
class BaseModule
{
public:
  /**
   * @brief Virtual destructor for safe polymorphic destruction.
   */
  virtual ~BaseModule() = default;

  /**
   * @brief Render the module's GUI elements.
   */
  virtual void render() = 0;
};

}