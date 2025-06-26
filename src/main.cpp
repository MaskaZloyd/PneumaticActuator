#include <print>

#include "core/application.hpp"

int
main()
{
  try {
    auto& app = mz::core::createApplication(
      "PneumaticActuator - GLFW + GLAD + ImGui", 1200, 800);

    std::println("Application created, window id: {}",
                 app.getWindowManager().getWindowId());

    app.setFrameCallback([]() {
      static bool show_demo_window = true;
      if (show_demo_window) {
        ImGui::ShowDemoWindow(&show_demo_window);
      }

      ImGui::Begin("PneumaticActuator Control Panel");

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate,
                  ImGui::GetIO().Framerate);

      static float f       = 0.0f;
      static int   counter = 0;
      ImGui::SliderFloat("Float", &f, 0.0f, 1.0f);
      ImGui::ColorEdit3("Clear color", reinterpret_cast<float*>(&f));

      if (ImGui::Button("Button")) {
        counter++;
      }
      ImGui::SameLine();
      ImGui::Text("counter = %d", counter);

      auto& window_manager =
        mz::core::Application::getInstance().getWindowManager();
      auto [fb_width, fb_height]   = window_manager.getFramebufferSize();
      auto [win_width, win_height] = window_manager.getWindowSize();
      ImGui::Text("Window size: %dx%d", win_width, win_height);
      ImGui::Text("Framebuffer size: %dx%d", fb_width, fb_height);

      if (ImGui::Button("Close Application")) {
        mz::core::Application::getInstance().shutdown();
      }

      ImGui::End();
    });

    app.setShutdownCallback([]() {
      std::println("Application shutting down - cleanup callback called");
    });

    return app.run();

  } catch (const mz::platform::WindowManagerException& e) {
    std::println(stderr, "WindowManager error: {}", e.what());
    return -1;
  } catch (const std::exception& e) {
    std::println(stderr, "Unexpected error: {}", e.what());
    return -1;
  }
}