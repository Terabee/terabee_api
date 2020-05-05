/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/internal/logger/Logger.hpp"

#include <cstdlib>
#include <memory>
#include <string>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#define LOGGER_MAX_FILE_SIZE 1024*1024*64  // using 64MB log files
#define LOGGER_MAX_NUMBER_OF_FILES 10  // let's say that 10 files (64MB each) is enough

namespace terabee
{
namespace internal
{
namespace logger
{

LoggerParameterInitializer::LoggerParameterInitializer() {
  if (!std::getenv(LOGGER_ENABLE_LOGGING))
  {
    spdlog::set_level(spdlog::level::off);
    return;
  }
  if (std::getenv(LOGGER_ENABLE_DEBUG))
  {
    spdlog::set_level(spdlog::level::debug);
  }
  else
  {
    spdlog::set_level(spdlog::level::info);
  }
  spdlog::set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] <PID:%P> <Thread:%t> [%l] [%n] : %v%$");
}

Logger::Logger(const std::string& name) {
  logger_ = spdlog::get(name);
  if (logger_) return;
  if (std::getenv(LOGGER_PRINT_STDOUT)) {
    logger_ = spdlog::stdout_color_mt(name);
  }
  else
  {
    logger_ = std::make_shared<spdlog::logger>(name, file_sink_);
    spdlog::initialize_logger(logger_);
  }
}

const std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> Logger::file_sink_
  = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    LOGGER_FILENAME, LOGGER_MAX_FILE_SIZE, LOGGER_MAX_NUMBER_OF_FILES);

const LoggerParameterInitializer Logger::initializer_ = LoggerParameterInitializer();

}  // namespace logger
}  // namespace internal
}  // namespace terabee
