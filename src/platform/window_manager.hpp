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

class WindowManagerException : public std::runtime_error
{
public:
  explicit WindowManagerException(const std::string& message)
    : std::runtime_error(message)
  {
  }
};

class WindowManager final
{
public:
  static WindowManager& getInstance(const std::string_view title,
                                    const uint32_t         width,
                                    const uint32_t         height)
  {
    static WindowManager instance{ title, width, height };
    return instance;
  }

  ~WindowManager();
  WindowManager()                                = delete;
  WindowManager(const WindowManager&)            = delete;
  WindowManager(WindowManager&&)                 = delete;
  WindowManager& operator=(const WindowManager&) = delete;
  WindowManager& operator=(WindowManager&&)      = delete;

  uint32_t            getWindowId() const { return m_window_id; }
  GLFWwindow*         getWindow() const;
  std::pair<int, int> getFramebufferSize() const;
  std::pair<int, int> getWindowSize() const;

  bool shouldClose() const;
  void setShouldClose(bool value) const;
  void setTitle(const std::string_view title) const;
  void beginFrame() const;
  void endFrame() const;

private:
  void generate_window_id();
  void init_glfw(const std::string_view title,
                 const uint32_t         width,
                 const uint32_t         height);
  void init_glad();
  void init_imgui();
  void cleanup();

  explicit WindowManager(const std::string_view title,
                         const uint32_t         width,
                         const uint32_t         height);

  struct GLFWwindowDeleter
  {
    void operator()(GLFWwindow* window) const;
  };

  bool                                           m_imgui_initialized{ false };
  uint32_t                                       m_window_id{ 0 };
  std::unique_ptr<GLFWwindow, GLFWwindowDeleter> m_window = nullptr;
};

WindowManager&
createWindowManager(const std::string_view title,
                    const uint32_t         width,
                    const uint32_t         height) noexcept;
}