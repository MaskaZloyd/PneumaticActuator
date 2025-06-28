#pragma once

#include <cstdint>
#include <string>

namespace mz::core {

/**
 * @brief Log level enum
 */
enum class LogLevel : std::uint8_t
{
  TRACE    = 0,
  DEBUG    = 1,
  INFO     = 2,
  WARN     = 3,
  ERROR    = 4,
  CRITICAL = 5
};

/**
 * @brief Logger configuration
 */
struct Config
{
  bool async_logging      = true;
  bool enable_console     = true;
  bool enable_file        = true;

  LogLevel level          = LogLevel::INFO;

  std::string logger_name = "mz";
};

/**
 * @brief Logger class
 */
class Logger final
{
public:
  /**
   * @brief Get the logger instance
   * @param config Logger configuration
   * @return Logger instance
   */
  [[nodiscard]] static Logger& getInstance() noexcept;

  /**
   * @brief Logger destructor
   */
  ~Logger()                        = default;
  Logger(const Logger&)            = delete;
  Logger(Logger&&)                 = delete;
  Logger& operator=(const Logger&) = delete;
  Logger& operator=(Logger&&)      = delete;

  /**
   * @brief Log a message
   * @param level Log level
   * @param message Message to log
   */
  void log(LogLevel level, const std::string_view message) noexcept;

private:
  Logger() noexcept;

  Config m_config{};
};
}

#define MZ_LOG_TRACE(message)                                                  \
  mz::core::Logger::getInstance().log(::mz::core::LogLevel::TRACE, message)
#define MZ_LOG_DEBUG(message)                                                  \
  mz::core::Logger::getInstance().log(::mz::core::LogLevel::DEBUG, message)
#define MZ_LOG_INFO(message)                                                   \
  mz::core::Logger::getInstance().log(::mz::core::LogLevel::INFO, message)
#define MZ_LOG_WARN(message)                                                   \
  mz::core::Logger::getInstance().log(::mz::core::LogLevel::WARN, message)
#define MZ_LOG_ERROR(message)                                                  \
  mz::core::Logger::getInstance().log(::mz::core::LogLevel::ERROR, message)
#define MZ_LOG_CRITICAL(message)                                               \
  mz::core::Logger::getInstance().log(::mz::core::LogLevel::CRITICAL, message)
