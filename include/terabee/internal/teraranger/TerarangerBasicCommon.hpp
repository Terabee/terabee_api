#ifndef TERARANGER_BASIC_COMMON_HPP
#define TERARANGER_BASIC_COMMON_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 * Common implementaion for TerarangerEvo3m and TerarangerEvo60m
 * (and maybe some more in the future)
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerBasicCommon
{
public:
  TerarangerBasicCommon() = delete;
  TerarangerBasicCommon(std::shared_ptr<serial_communication::ISerial> serial_port,
    const std::string& name);
  TerarangerBasicCommon(const TerarangerBasicCommon& other) = delete;
  TerarangerBasicCommon(TerarangerBasicCommon&& other) = delete;
  ~TerarangerBasicCommon();
  bool initialize();
  bool shutDown();
  DistanceData getDistance();
  bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb);

private:
  void captureData();
  const std::array<uint8_t, 4> enable_binary_mode_cmd_;
  logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  OnDistanceDataCaptureCallback cb_;
  std::thread data_capture_thread_;
  std::atomic_bool stop_;
  std::mutex m_;
  float measurement_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_BASIC_COMMON_HPP
