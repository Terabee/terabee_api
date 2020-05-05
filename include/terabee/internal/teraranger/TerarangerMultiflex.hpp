#ifndef TERARANGER_MULTIFLEX_HPP
#define TERARANGER_MULTIFLEX_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/ITerarangerMultiflex.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerMultiflex: public ITerarangerMultiflex
{
public:
  TerarangerMultiflex() = delete;
  TerarangerMultiflex(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerMultiflex(const TerarangerMultiflex& other) = delete;
  TerarangerMultiflex(TerarangerMultiflex&& other) = delete;
  ~TerarangerMultiflex() override;
  bool initialize() override;
  bool shutDown() override;
  DistanceData getDistance() override;
  bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) override;
  bool configureNumberOfSensors(uint8_t mask) override;

private:
  enum AckFlag
  {
    NO_ACK = 1,
    ACK = 2,
    NACK = 3
  };
  void captureData();
  bool activateBinaryMode();
  bool waitForAck();
  bool initialized();
  logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  OnDistanceDataCaptureCallback cb_;
  ring_buffer::RingBuffer ring_buffer_;
  std::thread data_capture_thread_;
  std::atomic_bool stop_;
  std::condition_variable cv_;
  std::mutex m_;
  AckFlag ack_flag_;
  DistanceData current_dist_data_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_MULTIFLEX_HPP
