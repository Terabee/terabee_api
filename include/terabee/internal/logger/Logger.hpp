#ifndef LOGGER_HPP
#define LOGGER_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <memory>
#include <string>

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

#define LOGGER_FILENAME "TerabeeLog.log"
#define LOGGER_ENABLE_LOGGING "LOGGER_ENABLE_LOGGING"  // this env variable has to be set in order to enable logger at all NOLINT
#define LOGGER_PRINT_STDOUT "LOGGER_PRINT_STDOUT"  // set this env variable to 1 to enable prints on stdout NOLINT
#define LOGGER_ENABLE_DEBUG "LOGGER_ENABLE_DEBUG"  // set this env variable to 1 to enable debug logs NOLINT

namespace terabee
{
namespace internal
{
namespace logger
{

struct LoggerParameterInitializer
{
  LoggerParameterInitializer();
};

class Logger
{
public:
  typedef std::shared_ptr<spdlog::logger> loggerPtr;
  Logger() = delete;
  Logger(const Logger& l) = default;
  explicit Logger(const std::string& name);
  loggerPtr operator->() const {
    return logger_;
  }
private:
  static const std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> file_sink_;
  static const LoggerParameterInitializer initializer_;
  loggerPtr logger_;
};

}  // namespace logger
}  // namespace internal
}  // namespace terabee

#endif  // LOGGER_HPP
