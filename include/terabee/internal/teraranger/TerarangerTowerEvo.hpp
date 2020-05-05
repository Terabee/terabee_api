#ifndef TERARANGER_TOWER_EVO_HPP
#define TERARANGER_TOWER_EVO_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/TowerDistanceData.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/ITerarangerTowerEvo.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerTowerEvo: public ITerarangerTowerEvo
{
public:
  TerarangerTowerEvo() = delete;
  TerarangerTowerEvo(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerTowerEvo(const TerarangerTowerEvo& other) = delete;
  TerarangerTowerEvo(TerarangerTowerEvo&& other) = delete;
  ~TerarangerTowerEvo() override;
  bool initialize() override;
  bool shutDown() override;
  TowerDistanceData getDistance() override;
  bool registerOnTowerDistanceDataCaptureCallback(OnTowerDistanceDataCaptureCallback cb) override;
  ImuData getImuData() override;
  OperatingMode getOperatingMode() const override;
  UpdateRate getUpdateRate() const override;
  ImuMode getImuMode() const override;
  bool setOperatingMode(OperatingMode mode) override;
  bool setUpdateRate(UpdateRate rate) override;
  bool setImuMode(ImuMode mode) override;
  bool setLedThreshold(uint8_t upper, uint8_t lower) override;

private:
  bool isInitialized();
  std::array<uint8_t, 4> operMode2Cmd(OperatingMode m);
  std::array<uint8_t, 5> updateRate2Cmd(UpdateRate m);
  std::array<uint8_t, 4> imuMode2Cmd(ImuMode m);
  std::array<uint8_t, 6> setThresholdsCmd(uint8_t uu, uint8_t ll);
  bool sendCommand(const uint8_t* cmd, size_t size);
  bool activateDevice();
  bool deactivateDevice();
  void dataCaptureLoop();
  bool checkRingBufferForDist();
  bool checkRingBufferForIM();
  logger::Logger logger_;
  std::shared_ptr<serial_communication::ISerial> serial_port_;
  OnTowerDistanceDataCaptureCallback cb_;
  ring_buffer::RingBuffer ring_buffer_;
  OperatingMode operating_mode_;
  UpdateRate update_rate_;
  ImuMode imu_mode_;
  uint8_t uu_threshold_;
  uint8_t ll_threshold_;
  std::atomic_bool stop_;
  std::thread data_capture_thread_;
  std::mutex m_;
  TowerDistanceData current_dist_data_;
  ImuData current_imu_data_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_TOWER_EVO_HPP
