#include "window_manager.hpp"
#include "core/logger.hpp"

#include <format>
#include <limits>
#include <print>
#include <random>

namespace {
void
glfw_error_callback(int error, const char* description)
{

  MZ_LOG_ERROR("GLFW Error");
}

void
framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
  glViewport(0, 0, width, height);
}
}

namespace mz::platform {

WindowManager::~WindowManager()
{
  cleanup();
}

GLFWwindow*
WindowManager::getWindow() const
{
  return m_window.get();
}

bool
WindowManager::shouldClose() const
{
  return glfwWindowShouldClose(m_window.get());
}

void
WindowManager::setShouldClose(bool value) const
{
  glfwSetWindowShouldClose(m_window.get(), value ? GLFW_TRUE : GLFW_FALSE);
}

void
WindowManager::beginFrame() const
{
  glfwPollEvents();

  if (m_imgui_initialized) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
  }

  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void
WindowManager::endFrame() const
{
  if (m_imgui_initialized) {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow* backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }
  }

  glfwSwapBuffers(m_window.get());
}

void
WindowManager::setTitle(const std::string_view title) const
{
  glfwSetWindowTitle(m_window.get(), title.data());
}

std::pair<int, int>
WindowManager::getFramebufferSize() const
{
  int width, height;
  glfwGetFramebufferSize(m_window.get(), &width, &height);
  return { width, height };
}

std::pair<int, int>
WindowManager::getWindowSize() const
{
  int width, height;
  glfwGetWindowSize(m_window.get(), &width, &height);
  return { width, height };
}

void
WindowManager::generate_window_id()
{
  auto random_device{ std::random_device{} };
  auto generator{ std::mt19937{ random_device() } };
  auto distribution{
    std::uniform_int_distribution<uint32_t>{
                                            1, std::numeric_limits<uint32_t>::max() }
  };

  m_window_id = distribution(generator);
}

void
WindowManager::init_glfw(const std::string_view title,
                         const uint32_t         width,
                         const uint32_t         height)
{
  glfwSetErrorCallback(glfw_error_callback);

  if (!glfwInit()) {
    throw WindowManagerException("Failed to initialize GLFW");
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4); // 4x MSAA

  GLFWwindow* raw_window =
    glfwCreateWindow(width, height, title.data(), nullptr, nullptr);

  if (!raw_window) {
    glfwTerminate();
    throw WindowManagerException("Failed to create GLFW window");
  }

  m_window = std::unique_ptr<GLFWwindow, GLFWwindowDeleter>{ raw_window };

  glfwMakeContextCurrent(m_window.get());

  glfwSwapInterval(1);

  glfwSetFramebufferSizeCallback(m_window.get(), framebuffer_size_callback);

  MZ_LOG_INFO(std::format(
    "GLFW initialized successfully. Window created: {}x{}", width, height));
}

void
WindowManager::init_glad()
{
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    throw WindowManagerException("Failed to initialize GLAD");
  }

  const auto* vendor       = glGetString(GL_VENDOR);
  const auto* renderer     = glGetString(GL_RENDERER);
  const auto* version      = glGetString(GL_VERSION);
  const auto* glsl_version = glGetString(GL_SHADING_LANGUAGE_VERSION);

  MZ_LOG_INFO("OpenGL initialized successfully");
  MZ_LOG_INFO(std::format("Vendor: {}", reinterpret_cast<const char*>(vendor)));
  MZ_LOG_INFO(
    std::format("Renderer: {}", reinterpret_cast<const char*>(renderer)));
  MZ_LOG_INFO(
    std::format("Version: {}", reinterpret_cast<const char*>(version)));
  MZ_LOG_INFO(std::format("GLSL Version: {}",
                          reinterpret_cast<const char*>(glsl_version)));

  glEnable(GL_DEPTH_TEST);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  auto [width, height] = getFramebufferSize();
  glViewport(0, 0, width, height);
}

void
WindowManager::init_imgui()
{
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  // Initialize ImPlot context
  ImPlot::CreateContext();

  ImGuiIO& io = ImGui::GetIO();

  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

  ImGui::StyleColorsLight();

  ImGuiStyle& style = ImGui::GetStyle();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    style.WindowRounding              = 0.0f;
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
  }

  if (!ImGui_ImplGlfw_InitForOpenGL(m_window.get(), true)) {
    throw WindowManagerException("Failed to initialize ImGui GLFW backend");
  }

  const char* glsl_version = "#version 330";
  if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
    ImGui_ImplGlfw_Shutdown();
    throw WindowManagerException("Failed to initialize ImGui OpenGL3 backend");
  }

  m_imgui_initialized = true;
  MZ_LOG_INFO("ImGui and ImPlot initialized successfully");
}

void
WindowManager::cleanup()
{
  if (m_imgui_initialized) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    // Cleanup ImPlot context
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    m_imgui_initialized = false;
  }

  if (m_window) {
    m_window.reset();
  }

  glfwTerminate();
}

WindowManager::WindowManager(const std::string_view title,
                             const uint32_t         width,
                             const uint32_t         height)
{
  try {
    generate_window_id();
    init_glfw(title, width, height);
    init_glad();
    init_imgui();

    MZ_LOG_INFO(std::format(
      "WindowManager initialized successfully with ID: {}", m_window_id));
  } catch (const WindowManagerException& e) {
    cleanup();
    throw;
  } catch (const std::exception& e) {
    cleanup();
    throw WindowManagerException(std::format(
      "Unexpected error during WindowManager initialization: {}", e.what()));
  }
}

void
WindowManager::GLFWwindowDeleter::operator()(GLFWwindow* window) const
{
  if (window) {
    glfwDestroyWindow(window);
  }
}

WindowManager&
createWindowManager(const std::string_view title,
                    const uint32_t         width,
                    const uint32_t         height) noexcept
{
  return WindowManager::getInstance(title, width, height);
}

}