#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>
#include <vector>

#include "../gui/base_module.hpp"
#include "../platform/window_manager.hpp"

namespace mz::core {

/**
 * @brief Main application class managing the application lifecycle
 *
 * This class follows RAII principles and manages the WindowManager instance.
 * It provides a clean interface for running the application event loop.
 */
class Application final
{
public:
  using FrameCallback    = std::function<void()>;
  using ShutdownCallback = std::function<void()>;
  using BaseModulePtr    = std::shared_ptr<gui::BaseModule>;

  /**
   * @brief Get the singleton application instance
   * @param title Window title
   * @param width Window width
   * @param height Window height
   * @return Reference to the application instance
   */
  static Application& getInstance(std::string_view title  = "Application",
                                  std::uint32_t    width  = 1200,
                                  std::uint32_t    height = 800);

  // Non-copyable and non-movable (RAII + singleton pattern)
  Application(const Application&)            = delete;
  Application(Application&&)                 = delete;
  Application& operator=(const Application&) = delete;
  Application& operator=(Application&&)      = delete;

  /**
   * @brief Application destructor - handles cleanup automatically (RAII)
   */
  ~Application() = default;

  /**
   * @brief Run the main application event loop
   * @return Exit code (0 for success, non-zero for error)
   */
  int run();

  /**
   * @brief Request application shutdown
   */
  void shutdown() noexcept;

  /**
   * @brief Check if application is running
   * @return true if running, false otherwise
   */
  [[nodiscard]] bool isRunning() const noexcept;

  /**
   * @brief Get the window manager instance
   * @return Reference to the window manager
   */
  [[nodiscard]] platform::WindowManager& getWindowManager() const noexcept;

  /**
   * @brief Set callback for frame updates
   * @param callback Function to call each frame
   */
  void setFrameCallback(FrameCallback callback) noexcept;

  /**
   * @brief Set callback for application shutdown
   * @param callback Function to call on shutdown
   */
  void setShutdownCallback(ShutdownCallback callback) noexcept;

  /**
   * @brief Get application title
   * @return Current window title
   */
  [[nodiscard]] std::string_view getTitle() const noexcept;

  /**
   * @brief Set application title
   * @param title New window title
   */
  void setTitle(std::string_view title) noexcept;

  /**
   * @brief Add a module to the application
   * @param module Module to add
   */
  void addModule(const BaseModulePtr& module) noexcept;

private:
  /**
   * @brief Private constructor for singleton pattern
   */
  explicit Application(std::string_view title,
                       std::uint32_t    width,
                       std::uint32_t    height);

  /**
   * @brief Initialize the application
   */
  void initialize();

  /**
   * @brief Process single frame
   */
  void processFrame();

  /**
   * @brief Handle application cleanup
   */
  void cleanup() noexcept;

  // Core application state
  bool                       m_running{ false };
  bool                       m_shutdown_requested{ false };
  std::string                m_title{};
  platform::WindowManager*   m_window_manager{ nullptr };
  std::vector<BaseModulePtr> m_modules;

  // Callbacks
  FrameCallback    m_frame_callback;
  ShutdownCallback m_shutdown_callback;
};

/**
 * @brief Convenience function to create and get application instance
 * @param title Window title
 * @param width Window width
 * @param height Window height
 * @return Reference to application instance
 */
[[nodiscard]] Application&
createApplication(std::string_view title  = "Application",
                  std::uint32_t    width  = 1200,
                  std::uint32_t    height = 800) noexcept;

}