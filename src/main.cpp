#include <print>

#include "platform/window_manager.hpp"

int
main()
{
  try {
    auto& window_manager{ mz::platform::createWindowManager(
      "PneumaticActuator - GLFW + GLAD + ImGui", 1200, 800) };

    std::println("Window manager created, window id: {}",
                 window_manager.getWindowId());

    while (!window_manager.shouldClose()) {
      window_manager.beginFrame();

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

      auto [fb_width, fb_height]   = window_manager.getFramebufferSize();
      auto [win_width, win_height] = window_manager.getWindowSize();
      ImGui::Text("Window size: %dx%d", win_width, win_height);
      ImGui::Text("Framebuffer size: %dx%d", fb_width, fb_height);

      if (ImGui::Button("Close Application")) {
        window_manager.setShouldClose(true);
      }

      ImGui::End();

      window_manager.endFrame();
    }

    std::println("Application shutting down gracefully");
    return 0;

  } catch (const mz::platform::WindowManagerException& e) {
    std::println(stderr, "WindowManager error: {}", e.what());
    return -1;
  } catch (const std::exception& e) {
    std::println(stderr, "Unexpected error: {}", e.what());
    return -1;
  }
}