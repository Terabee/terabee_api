#ifndef TERARANGER_THERMAL_COMMON_HPP
#define TERARANGER_THERMAL_COMMON_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/ThermalData.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerThermalCommon
{
public:
  TerarangerThermalCommon() = delete;
  TerarangerThermalCommon(std::shared_ptr<serial_communication::ISerial> serial_port,
    const std::string& name);
  TerarangerThermalCommon(const TerarangerThermalCommon& other) = delete;
  TerarangerThermalCommon(TerarangerThermalCommon&& other) = delete;
  ~TerarangerThermalCommon();
  bool initialize();
  bool shutDown();
  ThermalData getThermalData();
  bool registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb);
  float getSensorTemperature();
  bool setEmissivity(uint8_t emissivity);

private:
  enum AckFlag
  {
    NO_ACK = 1,
    ACK = 2,
    NACK = 3
  };
  bool isOpen();
  void captureData();
  void checkIfAckAndNotify();
  bool checkIfFrame();
  bool waitForAck();
  bool checkCrc32(const uint8_t* buff, size_t size);
  const std::array<uint8_t, 5> activate_output_cmd_;
  const std::array<uint8_t, 5> deactivate_output_cmd_;
  logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  OnThermalDataCaptureCallback cb_;
  std::thread data_capture_thread_;
  std::atomic_bool stop_;
  std::condition_variable cv_;
  std::mutex m_;
  AckFlag ack_flag_;
  ring_buffer::RingBuffer ring_buffer_;
  std::array<float, 1024> thermal_data_;
  float sensor_temperature_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_THERMAL_COMMON_HPP
