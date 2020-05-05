#ifndef TERARANGER_EVO_64_PX_HPP
#define TERARANGER_EVO_64_PX_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/ITerarangerEvo64px.hpp"
#include "terabee/DistanceData.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvo64px: public ITerarangerEvo64px
{
public:
  TerarangerEvo64px() = delete;
  TerarangerEvo64px(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvo64px(const TerarangerEvo64px& other) = delete;
  TerarangerEvo64px(TerarangerEvo64px&& other) = delete;
  ~TerarangerEvo64px() override;
  bool initialize() override;
  bool shutDown() override;
  OutputMode getOutputMode() const override;
  MeasurementMode getMeasurementMode() const override;
  bool setOutputMode(OutputMode mode) override;
  bool setMeasurementMode(MeasurementMode mode) override;
  DistanceData getDistanceValues() override;
  bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) override;
  DistanceData getAmbientValues() override;
private:
  bool isOpen();
  bool sendCommand(const uint8_t* cmd, size_t size);
  void captureData();
  bool checkCrc32(const uint8_t* buff, size_t size);
  std::array<uint16_t, 64> distFromRawData(const uint8_t* data);
  std::array<uint16_t, 64> ambFromRawData(const uint8_t* data);
  const std::array<uint8_t, 4> output_distance_cmd_;
  const std::array<uint8_t, 4> output_distance_ambient_cmd_;
  const std::array<uint8_t, 4> close_range_mode_cmd_;
  const std::array<uint8_t, 4> fast_mode_cmd_;
  const std::array<uint8_t, 5> activate_output_cmd_;
  const std::array<uint8_t, 5> deactivate_output_cmd_;
  logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  OnDistanceDataCaptureCallback cb_;
  std::thread data_capture_thread_;
  std::atomic_bool stop_;
  std::mutex m_;
  OutputMode output_mode_;
  MeasurementMode measurement_mode_;
  std::array<float, 64> distance_;
  std::array<float, 64> ambient_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_EVO_64_PX_HPP
