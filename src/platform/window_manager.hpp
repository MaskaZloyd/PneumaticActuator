#pragma once

#include <memory>
#include <stdexcept>
#include <string_view>

// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
// clang-format on

namespace mz::platform {

/**
 * @brief Exception for WindowManager errors
 */
class WindowManagerException : public std::runtime_error
{
public:
  /**
   * @brief Construct a new WindowManagerException
   * @param message Exception message
   */
  explicit WindowManagerException(const std::string& message)
    : std::runtime_error(message)
  {
  }
};

/**
 * @brief Manages window creation, rendering context, and input events.
 *
 * This class is a singleton that wraps GLFW and ImGui initialization and
 * management. It handles the window lifecycle and provides an interface for
 * frame rendering. It follows RAII principles.
 */
class WindowManager final
{
public:
  /**
   * @brief Get the singleton WindowManager instance.
   * @param title The title of the window.
   * @param width The width of the window.
   * @param height The height of the window.
   * @return Reference to the WindowManager instance.
   */
  static WindowManager& getInstance(const std::string_view title,
                                    const uint32_t         width,
                                    const uint32_t         height)
  {
    static WindowManager instance{ title, width, height };
    return instance;
  }

  /**
   * @brief Destructor that handles cleanup of window and graphics contexts.
   */
  ~WindowManager();
  WindowManager()                                = delete;
  WindowManager(const WindowManager&)            = delete;
  WindowManager(WindowManager&&)                 = delete;
  WindowManager& operator=(const WindowManager&) = delete;
  WindowManager& operator=(WindowManager&&)      = delete;

  /**
   * @brief Get the unique ID for the window.
   * @return The window ID.
   */
  [[nodiscard]] uint32_t getWindowId() const { return m_window_id; }
  /**
   * @brief Get the underlying GLFW window handle.
   * @return A pointer to the GLFWwindow.
   */
  [[nodiscard]] GLFWwindow* getWindow() const;
  /**
   * @brief Get the size of the framebuffer.
   * @return A pair containing the width and height of the framebuffer.
   */
  [[nodiscard]] std::pair<int, int> getFramebufferSize() const;
  /**
   * @brief Get the size of the window.
   * @return A pair containing the width and height of the window.
   */
  [[nodiscard]] std::pair<int, int> getWindowSize() const;

  /**
   * @brief Check if the window should close.
   * @return True if the window should close, false otherwise.
   */
  [[nodiscard]] bool shouldClose() const;
  /**
   * @brief Set the window should close flag.
   * @param value The new value for the should close flag.
   */
  void setShouldClose(bool value) const;
  /**
   * @brief Set the window title.
   * @param title The new title for the window.
   */
  void setTitle(const std::string_view title) const;
  /**
   * @brief Prepares for rendering a new frame.
   * This should be called at the beginning of the frame rendering loop.
   */
  void beginFrame() const;
  /**
   * @brief Swaps buffers and polls events after rendering a frame.
   * This should be called at the end of the frame rendering loop.
   */
  void endFrame() const;

private:
  /**
   * @brief Generates a unique ID for the window.
   */
  void generate_window_id();
  /**
   * @brief Initializes GLFW and creates a window.
   * @param title The title of the window.
   * @param width The width of the window.
   * @param height The height of the window.
   */
  void init_glfw(const std::string_view title,
                 const uint32_t         width,
                 const uint32_t         height);
  /**
   * @brief Initializes GLAD.
   */
  void init_glad();
  /**
   * @brief Initializes ImGui.
   */
  void init_imgui();
  /**
   * @brief Cleans up resources.
   */
  void cleanup();

  /**
   * @brief Private constructor for singleton pattern.
   * @param title The title of the window.
   * @param width The width of the window.
   * @param height The height of the window.
   */
  explicit WindowManager(const std::string_view title,
                         const uint32_t         width,
                         const uint32_t         height);

  /**
   * @brief Deleter for the unique_ptr managing the GLFWwindow.
   */
  struct GLFWwindowDeleter
  {
    void operator()(GLFWwindow* window) const;
  };

  bool                                           m_imgui_initialized{ false };
  uint32_t                                       m_window_id{ 0 };
  std::unique_ptr<GLFWwindow, GLFWwindowDeleter> m_window = nullptr;
};

/**
 * @brief Convenience function to create and get WindowManager instance.
 * @param title Window title
 * @param width Window width
 * @param height Window height
 * @return Reference to the WindowManager instance.
 */
[[nodiscard]] WindowManager&
createWindowManager(const std::string_view title,
                    const uint32_t         width,
                    const uint32_t         height) noexcept;
}