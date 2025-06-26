#include "application.hpp"

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

    std::cout << "Application initialized successfully: " << title << std::endl;

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
    std::cerr << "Error: WindowManager not initialized" << std::endl;
    return -1;
  }

  try {
    m_running            = true;
    m_shutdown_requested = false;

    std::cout << "Starting application event loop..." << std::endl;

    while (m_running && !m_window_manager->shouldClose() &&
           !m_shutdown_requested) {
      processFrame();
    }

    if (m_shutdown_callback) {
      try {
        m_shutdown_callback();
      } catch (const std::exception& e) {
        std::cerr << "Exception in shutdown callback: " << e.what()
                  << std::endl;
      }
    }

    std::cout << "Application shutting down gracefully" << std::endl;
    m_running = false;

    return 0;

  } catch (const platform::WindowManagerException& e) {
    std::cerr << "WindowManager error: " << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cerr << "Unexpected error in application loop: " << e.what()
              << std::endl;
    return -1;
  }
}

void
Application::shutdown() noexcept
{
  m_shutdown_requested = true;
  std::cout << "Application shutdown requested" << std::endl;
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

  if (m_frame_callback) {
    try {
      m_frame_callback();
    } catch (const std::exception& e) {
      std::cerr << "Exception in frame callback: " << e.what() << std::endl;
    }
  }

  m_window_manager->endFrame();
}

void
Application::cleanup() noexcept
{
  try {
    std::cout << "Cleaning up Application resources..." << std::endl;

    m_running           = false;
    m_window_manager    = nullptr;

    m_frame_callback    = nullptr;
    m_shutdown_callback = nullptr;

    std::cout << "Application cleanup completed" << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Exception during cleanup: " << e.what() << std::endl;
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