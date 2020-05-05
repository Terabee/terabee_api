#ifndef TERARANGER_EVO_MINI_HPP
#define TERARANGER_EVO_MINI_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <memory>
#include <mutex>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/ITerarangerEvoMini.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvoMini: public ITerarangerEvoMini
{
public:
  TerarangerEvoMini() = delete;
  TerarangerEvoMini(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvoMini(const TerarangerEvoMini& other) = delete;
  TerarangerEvoMini(TerarangerEvoMini&& other) = delete;
  ~TerarangerEvoMini() override;
  bool initialize() override;
  bool shutDown() override;
  DistanceData getDistance() override;
  bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) override;
  PixelMode getPixelMode() const override;
  RangeMode getRangeMode() const override;
  bool setPixelMode(PixelMode mode) override;
  bool setRangeMode(RangeMode mode) override;

private:
  bool initialized();
  void captureData();
  void activateBinaryMode();
  void activateChosenPixelMode();
  void activateChosenRangeMode();
  logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  OnDistanceDataCaptureCallback cb_;
  bool stop_;
  ring_buffer::RingBuffer ring_buffer_;
  DistanceData current_distance_;
  PixelMode pixel_mode_;
  RangeMode range_mode_;
  std::thread data_capture_thread_;
  std::mutex m_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_EVO_MINI_HPP
