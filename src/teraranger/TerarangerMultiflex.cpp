/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/common_definitions.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/endian/swapEndian.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerMultiflex.hpp"

using terabee::internal::crc::calcCrc8;
using terabee::internal::endian::swapEndian;
using terabee::internal::serial_communication::ISerial;

#define DATA_FRAME_SIZE 20

namespace terabee
{
namespace internal
{
namespace teraranger
{

namespace
{
  const std::array<uint8_t, 4> g_binary_mode_cmd = {0x00, 0x11, 0x02, 0x4C};
  const std::array<uint8_t, 5> g_config_sensors_num_cmd = {0x00, 0x52, 0x03, 0x00, 0x00};
}  // namespace

TerarangerMultiflex::TerarangerMultiflex(std::shared_ptr<serial_communication::ISerial> serial_port):
  logger_("TerarangerMultiflex"),
  serial_port_(serial_port),
  cb_(nullptr),
  ring_buffer_(DATA_FRAME_SIZE),
  data_capture_thread_(),
  stop_(true),
  cv_(),
  m_(),
  ack_flag_(NO_ACK),
  current_dist_data_()
{
  logger_->debug("CTor");
  current_dist_data_.distance = std::vector<float>(8);
  std::fill(current_dist_data_.distance.begin(), current_dist_data_.distance.end(), std::nan(""));
  serial_port_->setBaudrate(115200);
  serial_port_->setBytesize(ISerial::eightbits);
  serial_port_->setStopbits(ISerial::stopbits_one);
  serial_port_->setParity(ISerial::parity_none);
  serial_port_->setFlowcontrol(ISerial::flowcontrol_none);
  serial_port_->setTimeout(SERIAL_READ_TIMEOUT_MULTISENSOR);
}

TerarangerMultiflex::~TerarangerMultiflex()
{
  logger_->debug("DTor");
  shutDown();
}

bool TerarangerMultiflex::initialize()
{
  if (initialized())
  {
    logger_->warn("Already initialized");
    return false;
  }
  if (!serial_port_->open())
  {
    logger_->error("Failed to open serial port");
    return false;
  }
  stop_ = false;
  data_capture_thread_ = std::thread([this]()
    {
      while (!stop_)
      {
        captureData();
      }
    });
  if (!activateBinaryMode())
  {
    logger_->error("Failed to activate binary mode");
    shutDown();
    return false;
  }
  return true;
}

bool TerarangerMultiflex::shutDown()
{
  if (!initialized())
  {
    logger_->warn("Not initialized");
    return false;
  }
  stop_ = true;
  data_capture_thread_.join();
  serial_port_->close();
  return true;
}

DistanceData TerarangerMultiflex::getDistance()
{
  std::lock_guard<std::mutex> l(m_);
  return current_dist_data_;
}

bool TerarangerMultiflex::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  if (initialized())
  {
    logger_->warn("Cannot change callback after device initialization");
    return false;
  }
  cb_ = cb;
  return true;
}

bool TerarangerMultiflex::configureNumberOfSensors(uint8_t mask)
{
  if (!initialized())
  {
    logger_->warn("Not initialized; You need to initialize the sensor before configureNumberOfSensors");
    return false;
  }
  logger_->info("configureNumberOfSensors");
  uint8_t cmd[5];
  std::memcpy(cmd, g_config_sensors_num_cmd.data(), g_config_sensors_num_cmd.size());
  cmd[3] = mask;
  cmd[4] = calcCrc8(cmd, 4);
  serial_port_->write(cmd, 5);
  return waitForAck();
}

void TerarangerMultiflex::captureData()
{
  uint8_t byte(0);
  serial_port_->read(&byte, 1);
  ring_buffer_.push_back(byte);
  if (ring_buffer_[0] == 0x52 && ring_buffer_[1] == 0x45
    && ring_buffer_[4] == calcCrc8(ring_buffer_.data(), 4))
  {
    if (ring_buffer_[3] == 0xff) ack_flag_ = NACK;
    if (ring_buffer_[3] == 0x00) ack_flag_ = ACK;
    logger_->debug("Got ACK: {0:x}", ring_buffer_[3]);
    cv_.notify_all();
  }
  if (ring_buffer_[0] != 0x4d || ring_buffer_[1] != 0x46) return;
  uint8_t crc = calcCrc8(ring_buffer_.data(), DATA_FRAME_SIZE-1);
  if (ring_buffer_[DATA_FRAME_SIZE-1] != crc)
  {
    logger_->warn("Incorrect CRC {0:x} vs {1:x}",
      ring_buffer_[DATA_FRAME_SIZE-1], crc);
    return;
  }
  {
    std::lock_guard<std::mutex> l(m_);
    size_t i = 0;
    uint8_t mask = ring_buffer_[DATA_FRAME_SIZE-2];
    std::transform((uint16_t*)(ring_buffer_.data()+2),
      (uint16_t*)(ring_buffer_.data()+DATA_FRAME_SIZE-2),
      current_dist_data_.distance.begin(), [mask, &i, this](uint16_t d) -> float
      {
        i++;
        if ((mask & (1 << (i-1))) == 0) return std::nanf("");
        if (d == 0xffff) return -1;
        return static_cast<float>(swapEndian(d))/1000;
      });
  }
  if (cb_)
  {
    cb_(current_dist_data_);
  }
}

bool TerarangerMultiflex::activateBinaryMode()
{
  logger_->info("activateBinaryMode");
  serial_port_->write(g_binary_mode_cmd.data(), g_binary_mode_cmd.size());
  return waitForAck();
}

bool TerarangerMultiflex::waitForAck()
{
  std::unique_lock<std::mutex> l(m_);
  cv_.wait_for(l, SERIAL_READ_TIMEOUT_MULTISENSOR, [this]()
  {
    return ack_flag_ != NO_ACK;
  });
  bool result = (ack_flag_ == ACK)? true : false;
  ack_flag_ = NO_ACK;
  return result;
}

bool TerarangerMultiflex::initialized()
{
  return data_capture_thread_.joinable();
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
