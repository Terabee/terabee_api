/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "terabee/internal/common_definitions.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/endian/swapEndian.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerTowerEvo.hpp"

using terabee::internal::serial_communication::ISerial;
using terabee::internal::endian::swapEndian;

#define DIST_BUFF_SIZE 20
#define IMU1_BUFF_SIZE 12  // quaternion
#define IMU2_BUFF_SIZE 10  // euler
#define IMU3_BUFF_SIZE 18  // quaternion + xyz
#define UU_DEFAULT 80
#define LL_DEFAULT 5
#define UU_MAX 80
#define LL_MIN 5

namespace terabee
{
namespace internal
{
namespace teraranger
{

namespace
{
  const std::array<uint8_t, 5> g_activate_streaming_cmd = {0x00, 0x52, 0x02, 0x01, 0xDF};
  const std::array<uint8_t, 5> g_deactivate_streaming_cmd = {0x00, 0x52, 0x02, 0x00, 0xD8};
  const std::array<uint8_t, 4> g_binary_mode_cmd = {0x00, 0x11, 0x02, 0x4C};
  const std::array<uint8_t, 4> g_operating_mode_tower_cmd = {0x00, 0x31, 0x03, 0xE5};
  const std::array<uint8_t, 4> g_operating_mode_sequential_cmd = {0x00, 0x31, 0x02, 0xE2};
  const std::array<uint8_t, 4> g_operating_mode_simultaneous_cmd = {0x00, 0x31, 0x01, 0xEB};
  const std::array<uint8_t, 5> g_update_rate_asap_cmd = {0x00, 0x52, 0x03, 0x01, 0xca};
  const std::array<uint8_t, 5> g_update_rate_50_cmd = {0x00, 0x52, 0x03, 0x02, 0xC3};
  const std::array<uint8_t, 5> g_update_rate_100_cmd = {0x00, 0x52, 0x03, 0x03, 0xC4};
  const std::array<uint8_t, 5> g_update_rate_250_cmd = {0x00, 0x52, 0x03, 0x04, 0xD1};
  const std::array<uint8_t, 5> g_update_rate_500_cmd = {0x00, 0x52, 0x03, 0x05, 0xD6};
  const std::array<uint8_t, 5> g_update_rate_600_cmd = {0x00, 0x52, 0x03, 0x06, 0xDF};
  const std::array<uint8_t, 4> g_imu_mode_disabled_cmd = {0x00, 0x41, 0x01, 0x49};
  const std::array<uint8_t, 4> g_imu_mode_quaternion_cmd = {0x00, 0x41, 0x02, 0x40};
  const std::array<uint8_t, 4> g_imu_mode_euler_cmd = {0x00, 0x41, 0x03, 0x47};
  const std::array<uint8_t, 4> g_imu_mode_quaternionlinear_cmd = {0x00, 0x41, 0x04, 0x52};
  const std::array<uint8_t, 6> g_set_led_threshold_cmd = {0x00, 0x53, 0x01, 0x00, 0x00, 0x00};
}  // namespace

TerarangerTowerEvo::TerarangerTowerEvo(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  logger_("TerarangerTowerEvo"),
  serial_port_(serial_port),
  cb_(nullptr),
  ring_buffer_(DIST_BUFF_SIZE),
  operating_mode_(TowerMode),
  update_rate_(ASAP),
  imu_mode_(Disabled),
  uu_threshold_(UU_DEFAULT),
  ll_threshold_(LL_DEFAULT),
  stop_(true),
  data_capture_thread_(),
  m_(),
  current_dist_data_(),
  current_imu_data_()
{
  logger_->debug("CTor");
  current_dist_data_.distance = std::vector<float>(8, std::nan(""));
  current_dist_data_.mask = std::vector<bool>(8, false);
  serial_port_->setBaudrate(115200);
  serial_port_->setBytesize(ISerial::eightbits);
  serial_port_->setStopbits(ISerial::stopbits_one);
  serial_port_->setParity(ISerial::parity_none);
  serial_port_->setFlowcontrol(ISerial::flowcontrol_none);
  serial_port_->setTimeout(SERIAL_READ_TIMEOUT_MULTISENSOR);
}

TerarangerTowerEvo::~TerarangerTowerEvo()
{
  logger_->debug("DTor");
  shutDown();
}

bool TerarangerTowerEvo::initialize()
{
  logger_->debug("initialize");
  if (isInitialized())
  {
    logger_->warn("Already initialized");
    return false;
  }
  if (!serial_port_->open())
  {
    logger_->warn("Failed to open serial port");
    return false;
  }
  deactivateDevice();  // in case it was left streaming
  auto opermode_cmd = operMode2Cmd(operating_mode_);
  if (!sendCommand(opermode_cmd.data(), opermode_cmd.size()))
  {
    logger_->warn("Failed to set operating mode to {}", operating_mode_);
    return false;
  }
  auto updaterate_cmd = updateRate2Cmd(update_rate_);
  if (!sendCommand(updaterate_cmd.data(), updaterate_cmd.size()))
  {
    logger_->warn("Failed to set update rate to {}", update_rate_);
    return false;
  }
  auto imumode_cmd = imuMode2Cmd(imu_mode_);
  if (!sendCommand(imumode_cmd.data(), imumode_cmd.size()))
  {
    logger_->warn("Failed to set IMU mode to {}", imu_mode_);
    return false;
  }
  auto ledthreshold_cmd = setThresholdsCmd(uu_threshold_, ll_threshold_);
  if (!sendCommand(ledthreshold_cmd.data(), ledthreshold_cmd.size()))
  {
    logger_->warn("Failed to set LED thresholds to uu = {}, ll = {}", uu_threshold_, ll_threshold_);
    return false;
  }
  if (!activateDevice())
  {
    logger_->warn("Failed to initialize device");
    return false;
  }
  stop_ = false;
  data_capture_thread_ = std::thread([this]()
    {
      while (!stop_)
      {
        dataCaptureLoop();
      }
    });
  return true;
}

bool TerarangerTowerEvo::shutDown()
{
  logger_->debug("shutDown");
  stop_ = true;
  if (!isInitialized())
  {
    logger_->warn("Device not initialized");
    return false;
  }
  deactivateDevice();
  data_capture_thread_.join();
  serial_port_->close();
  return true;
}

TowerDistanceData TerarangerTowerEvo::getDistance()
{
  if (!isInitialized())
  {
    logger_->warn("Device not initialized");
    throw std::runtime_error("Device not initialized");
  }
  std::lock_guard<std::mutex> l(m_);
  return current_dist_data_;
}

bool TerarangerTowerEvo::registerOnTowerDistanceDataCaptureCallback(
  OnTowerDistanceDataCaptureCallback cb)
{
  if (isInitialized())
  {
    logger_->warn("Cannot change callback after device initialized");
    return false;
  }
  cb_ = cb;
  return true;
}

ImuData TerarangerTowerEvo::getImuData()
{
  if (!isInitialized())
  {
    logger_->warn("Device not initialized");
    throw std::runtime_error("Device not initialized");
  }
  std::lock_guard<std::mutex> l(m_);
  return current_imu_data_;
}

ITerarangerTowerEvo::OperatingMode TerarangerTowerEvo::getOperatingMode() const
{
  return operating_mode_;
}

ITerarangerTowerEvo::UpdateRate TerarangerTowerEvo::getUpdateRate() const
{
  return update_rate_;
}

ITerarangerTowerEvo::ImuMode TerarangerTowerEvo::getImuMode() const
{
  return imu_mode_;
}

bool TerarangerTowerEvo::setOperatingMode(OperatingMode mode)
{
  logger_->debug("setOperatingMode");
  if (isInitialized())
  {
    logger_->warn("Cannot change settings after device initialized");
    return false;
  }
  operating_mode_ = mode;
  return true;
}

bool TerarangerTowerEvo::setUpdateRate(UpdateRate rate)
{
  logger_->debug("setUpdateRate");
  if (isInitialized())
  {
    logger_->warn("Cannot change settings after device initialized");
    return false;
  }
  update_rate_ = rate;
  return true;
}

bool TerarangerTowerEvo::setImuMode(ImuMode mode)
{
  logger_->debug("setImuMode");
  if (isInitialized())
  {
    logger_->warn("Cannot change settings after device initialized");
    return false;
  }
  imu_mode_ = mode;
  current_imu_data_.data.clear();
  if (imu_mode_ == Disabled)
  {
    ring_buffer_ = ring_buffer::RingBuffer(DIST_BUFF_SIZE);
  }
  if (imu_mode_ == Quaternion)
  {
    current_imu_data_.data.resize(4, -1);
    ring_buffer_ = ring_buffer::RingBuffer(DIST_BUFF_SIZE + IMU1_BUFF_SIZE);
  }
  if (imu_mode_ == Euler)
  {
    current_imu_data_.data.resize(3, -1);
    ring_buffer_ = ring_buffer::RingBuffer(DIST_BUFF_SIZE + IMU2_BUFF_SIZE);
  }
  if (imu_mode_ == QuaternionLinearAcc)
  {
    current_imu_data_.data.resize(7, -1);
    ring_buffer_ = ring_buffer::RingBuffer(DIST_BUFF_SIZE + IMU3_BUFF_SIZE);
  }
  return true;
}

bool TerarangerTowerEvo::setLedThreshold(uint8_t upper, uint8_t lower)
{
  logger_->debug("setLedThreshold");
  if (isInitialized())
  {
    logger_->warn("Cannot change settings after device initialized");
    return false;
  }
  if (upper <= lower)
  {
    logger_->warn("setLedThreshold: {} <= {}", upper, lower);
    return false;
  }
  if (lower < LL_MIN)
  {
    logger_->warn("setLedThreshold: ll threshold too small: {}", lower);
    return false;
  }
  if (upper > UU_MAX)
  {
    logger_->warn("setLedThreshold: uu threshold too big: {}", upper);
    return false;
  }
  uu_threshold_ = upper;
  ll_threshold_ = lower;
  return true;
}

bool TerarangerTowerEvo::isInitialized()
{
  return data_capture_thread_.joinable();
}

std::array<uint8_t, 4> TerarangerTowerEvo::operMode2Cmd(OperatingMode m)
{
  if (m == TowerMode) return g_operating_mode_tower_cmd;
  else if (m == SequentialMode) return g_operating_mode_sequential_cmd;
  else if (m == SimultaneousMode) return g_operating_mode_simultaneous_cmd;
  else throw std::runtime_error("Unrecognized mode: " + std::to_string(m));
}

std::array<uint8_t, 5> TerarangerTowerEvo::updateRate2Cmd(UpdateRate m)
{
  if (m == ASAP) return g_update_rate_asap_cmd;
  else if (m == Rate50HZ) return g_update_rate_50_cmd;
  else if (m == Rate100HZ) return g_update_rate_100_cmd;
  else if (m == Rate250HZ) return g_update_rate_250_cmd;
  else if (m == Rate500HZ) return g_update_rate_500_cmd;
  else if (m == Rate600HZ) return g_update_rate_600_cmd;
  else throw std::runtime_error("Unrecognized mode: " + std::to_string(m));
}

std::array<uint8_t, 4> TerarangerTowerEvo::imuMode2Cmd(ImuMode m)
{
  if (m == Disabled) return g_imu_mode_disabled_cmd;
  else if (m == Quaternion) return g_imu_mode_quaternion_cmd;
  else if (m == Euler) return g_imu_mode_euler_cmd;
  else if (m == QuaternionLinearAcc) return g_imu_mode_quaternionlinear_cmd;
  else throw std::runtime_error("Unrecognized mode: " + std::to_string(m));
}

std::array<uint8_t, 6> TerarangerTowerEvo::setThresholdsCmd(uint8_t uu, uint8_t ll)
{
  std::array<uint8_t, 6> result = g_set_led_threshold_cmd;
  result[3] = uu;
  result[4] = ll;
  result[5] = crc::calcCrc8(result.data(), 5);
  return result;
}

bool TerarangerTowerEvo::sendCommand(const uint8_t* cmd, size_t size)
{
  logger_->debug("sendCommand");
  serial_port_->write(cmd, size);
  uint8_t reply[4];
  std::memset(reply, 0, 4);
  serial_port_->read(reply, 4);
  if (reply[0] != 0x30 || reply[2] != 0x00 || reply[3] != crc::calcCrc8(reply, 3))
  {
    logger_->warn("Did not receive ACK");
    logger_->debug("Received: [{0:x}, {1:x}, {2:x}, {3:x}]",
      reply[0], reply[1], reply[2], reply[3]);
    return false;
  }
  return true;
}

bool TerarangerTowerEvo::activateDevice()
{
  if (!sendCommand(g_binary_mode_cmd.data(), g_binary_mode_cmd.size()))
  {
    logger_->warn("Failed to set binary mode output");
    return false;
  }
  if (!sendCommand(g_activate_streaming_cmd.data(), g_activate_streaming_cmd.size()))
  {
    logger_->warn("Failed to activate data stream");
    return false;
  }
  return true;
}
bool TerarangerTowerEvo::deactivateDevice()
{
  logger_->debug("deactivateDevice");
  sendCommand(g_deactivate_streaming_cmd.data(), g_deactivate_streaming_cmd.size());
  return true;
}

void TerarangerTowerEvo::dataCaptureLoop()
{
  uint8_t byte(0);
  serial_port_->read(&byte, 1);
  ring_buffer_.push_back(byte);
  if (checkRingBufferForDist())
  {
    {
      std::lock_guard<std::mutex> l(m_);
      std::transform((uint16_t*)(ring_buffer_.data()+2),
        (uint16_t*)(ring_buffer_.data()+DIST_BUFF_SIZE-2),
        current_dist_data_.distance.begin(), [](uint16_t d) -> float
        {
          d = swapEndian(d);
          if (d == 0x0000) return -std::numeric_limits<float>::infinity();
          if (d == 0x0001) return -1;
          if (d == 0xffff) return std::numeric_limits<float>::infinity();
          return static_cast<float>(d)/1000;
        });
      uint8_t mask = ring_buffer_[DIST_BUFF_SIZE-2];
      for (size_t i = 0; i < current_dist_data_.mask.size(); i++)
      {
        if (mask & (1 << i)) current_dist_data_.mask[i] = true;
        else current_dist_data_.mask[i] = false;
      }
    }
    if (cb_)
    {
      cb_(current_dist_data_);
    }
  }
  if (imu_mode_ == Disabled) return;
  if (!checkRingBufferForIM()) return;
  const int16_t* tr_begin =
    reinterpret_cast<const int16_t*>(ring_buffer_.data()+DIST_BUFF_SIZE+3);
  const int16_t* tr_end =
    reinterpret_cast<const int16_t*>(ring_buffer_.data()+ring_buffer_.size()-1);
  std::function<float(int16_t)> transformation;
  if (imu_mode_ == Quaternion)
  {
    transformation = [](int16_t d) { return static_cast<float>(swapEndian(d))/16384; };
  }
  if (imu_mode_ == Euler)
  {
    transformation = [](int16_t d) { return static_cast<float>(swapEndian(d))/16; };
  }
  size_t index(0);
  if (imu_mode_ == QuaternionLinearAcc)
  {
    transformation = [&index](int16_t d) {
      index++;
      d = swapEndian(d);
      if (index <= 4) return static_cast<float>(d)/16384;
      else return static_cast<float>(d);
    };
  }
  std::lock_guard<std::mutex> l(m_);
  std::transform(tr_begin, tr_end, current_imu_data_.data.begin(), transformation);
}

bool TerarangerTowerEvo::checkRingBufferForDist()
{
  if (ring_buffer_[0] != 'T' || ring_buffer_[1] != 'H')
  {
    return false;
  }
  uint8_t dist_crc = crc::calcCrc8(ring_buffer_.data(), DIST_BUFF_SIZE-1);
  if (ring_buffer_[DIST_BUFF_SIZE-1] != dist_crc)
  {
    logger_->warn("Incorrect CRC {0:x} vs {1:x}",
      ring_buffer_[DIST_BUFF_SIZE-1], dist_crc);
    return false;
  }
  return true;
}

bool TerarangerTowerEvo::checkRingBufferForIM()
{
  if (ring_buffer_[DIST_BUFF_SIZE] != 'I' || ring_buffer_[DIST_BUFF_SIZE+1] != 'M')
  {
    return false;
  }
  uint8_t imu_crc = crc::calcCrc8(ring_buffer_.data() + DIST_BUFF_SIZE,
    ring_buffer_.size() - DIST_BUFF_SIZE-1);
  if (ring_buffer_[ring_buffer_.size()-1] != imu_crc)
  {
    logger_->warn("Incorrect IMU CRC {0:x} vs {1:x}",
      ring_buffer_[ring_buffer_.size()-1], imu_crc);
    return false;
  }
  return true;
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
