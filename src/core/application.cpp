#include "application.hpp"
#include "core/logger.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

namespace mz::core {

Application::Application(std::string_view title,
                         std::uint32_t    width,
                         std::uint32_t    height)
  : m_title(title)
{
  try {
    initialize();

    m_window_manager = &platform::createWindowManager(title, width, height);

    MZ_LOG_INFO("Application initialized successfully");

  } catch (const std::exception& e) {
    cleanup();
    throw std::runtime_error("Failed to initialize Application: " +
                             std::string(e.what()));
  }
}

Application&
Application::getInstance(std::string_view title,
                         std::uint32_t    width,
                         std::uint32_t    height)
{
  static Application instance{ title, width, height };
  return instance;
}

int
Application::run()
{
  if (!m_window_manager) {
    MZ_LOG_ERROR("Error: WindowManager not initialized");
    return -1;
  }

  try {
    m_running            = true;
    m_shutdown_requested = false;

    MZ_LOG_INFO("Starting application event loop...");

    while (m_running && !m_window_manager->shouldClose() &&
           !m_shutdown_requested) {
      processFrame();
    }

    if (m_shutdown_callback) {
      try {
        m_shutdown_callback();
      } catch (const std::exception& e) {
        MZ_LOG_ERROR("Exception in shutdown callback");
      }
    }

    MZ_LOG_INFO("Application shutting down gracefully");
    m_running = false;

    return 0;

  } catch (const platform::WindowManagerException& e) {
    MZ_LOG_ERROR("WindowManager error");
    return -1;
  } catch (const std::exception& e) {
    MZ_LOG_ERROR("Unexpected error in application loop");
    return -1;
  }
}

void
Application::shutdown() noexcept
{
  m_shutdown_requested = true;
  MZ_LOG_INFO("Application shutdown requested");
}

bool
Application::isRunning() const noexcept
{
  return m_running;
}

platform::WindowManager&
Application::getWindowManager() const noexcept
{
  return *m_window_manager;
}

void
Application::setFrameCallback(FrameCallback callback) noexcept
{
  m_frame_callback = std::move(callback);
}

void
Application::setShutdownCallback(ShutdownCallback callback) noexcept
{
  m_shutdown_callback = std::move(callback);
}

std::string_view
Application::getTitle() const noexcept
{
  return m_title;
}

void
Application::setTitle(std::string_view title) noexcept
{
  m_title = title;
  if (m_window_manager) {
    m_window_manager->setTitle(title);
  }
}

void
Application::addModule(const BaseModulePtr& module) noexcept
{
  m_modules.push_back(std::move(module));
}

void
Application::initialize()
{
  std::cout << "Initializing Application..." << std::endl;
}

void
Application::processFrame()
{
  if (!m_window_manager) {
    return;
  }

  m_window_manager->beginFrame();

  for (const auto& module : m_modules) {
    module->render();
  }

  m_window_manager->endFrame();
}

void
Application::cleanup() noexcept
{
  try {
    MZ_LOG_INFO("Cleaning up Application resources...");

    m_running           = false;
    m_window_manager    = nullptr;

    m_frame_callback    = nullptr;
    m_shutdown_callback = nullptr;

    MZ_LOG_INFO("Application cleanup completed");

  } catch (const std::exception& e) {
    MZ_LOG_ERROR("Exception during cleanup");
  }
}

Application&
createApplication(std::string_view title,
                  std::uint32_t    width,
                  std::uint32_t    height) noexcept
{
  return Application::getInstance(title, width, height);
}

}