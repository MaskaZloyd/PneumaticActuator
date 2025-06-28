#include "logger.hpp"

#include <chrono>
#include <print>

namespace mz::core {

Logger&
Logger::getInstance() noexcept
{
  static Logger instance{};
  return instance;
}

void
Logger::log(LogLevel level, const std::string_view message) noexcept
{
  // TODO: Implement logging through spdlog
  std::string level_str;
  switch (level) {
    case LogLevel::TRACE:
      level_str = "TRACE";
      break;
    case LogLevel::DEBUG:
      level_str = "DEBUG";
      break;
    case LogLevel::INFO:
      level_str = "INFO";
      break;
    case LogLevel::WARN:
      level_str = "WARN";
      break;
    case LogLevel::ERROR:
      level_str = "ERROR";
      break;
    case LogLevel::CRITICAL:
      level_str = "CRITICAL";
      break;
  }

  const std::chrono::system_clock::time_point current_time =
    std::chrono::system_clock::now();
  const std::string current_time_str =
    std::format("[{:%Y-%m-%d %H:%M:%S.%e}]", current_time);

  std::println("[{}] [{}] [{}] {}",
               current_time_str,
               level_str,
               m_config.logger_name,
               message);
}

Logger::Logger() noexcept {}

}
