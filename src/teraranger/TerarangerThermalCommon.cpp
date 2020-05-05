/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/internal/common_definitions.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerThermalCommon.hpp"
#include "terabee/ThermalData.hpp"

#define EMISSIVITY_CMD_ADDR 0x51
#define THERMAL_FRAME_SIZE 2070

using terabee::internal::crc::calcCrc32;
using terabee::internal::crc::calcCrc8;
using terabee::internal::serial_communication::ISerial;

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerThermalCommon::TerarangerThermalCommon(std::shared_ptr<ISerial> serial_port,
  const std::string& name):
  activate_output_cmd_({0x00, 0x52, 0x02, 0x01, 0xDF}),
  deactivate_output_cmd_({0x00, 0x52, 0x02, 0x00, 0xD8}),
  logger_(name),
  serial_port_(serial_port),
  cb_(nullptr),
  data_capture_thread_(),
  stop_(true),
  cv_(),
  m_(),
  ack_flag_(NO_ACK),
  ring_buffer_(THERMAL_FRAME_SIZE),
  thermal_data_(),
  sensor_temperature_(std::nan(""))
{
  logger_->debug("CTor");
  std::fill(thermal_data_.begin(), thermal_data_.end(), std::nan(""));
  serial_port_->setBaudrate(115200);
  serial_port_->setBytesize(ISerial::eightbits);
  serial_port_->setStopbits(ISerial::stopbits_one);
  serial_port_->setParity(ISerial::parity_none);
  serial_port_->setFlowcontrol(ISerial::flowcontrol_none);
  serial_port_->setTimeout(SERIAL_READ_TIMEOUT_COMMON);
}

TerarangerThermalCommon::~TerarangerThermalCommon()
{
  logger_->debug("DTor");
  shutDown();
}

bool TerarangerThermalCommon::initialize()
{
  logger_->debug("initialize");
  if (isOpen())
  {
    logger_->warn("Already initialized");
    return false;
  }
  if (!serial_port_->open())
  {
    logger_->warn("Failed to open serial port");
    return false;
  }
  stop_ = false;
  data_capture_thread_ = std::thread(
    [this]()
    {
      while (!stop_)
      {
        captureData();
      }
    });
  serial_port_->write(activate_output_cmd_.data(), activate_output_cmd_.size());
  if (!waitForAck())
  {
    logger_->warn("Failed to activate output");
    shutDown();
    return false;
  }
  return true;
}

bool TerarangerThermalCommon::shutDown()
{
  logger_->debug("shutDown");
  if (!isOpen())
  {
    logger_->warn("Not initialized");
    return false;
  }
  stop_ = true;
  data_capture_thread_.join();
  serial_port_->write(deactivate_output_cmd_.data(), deactivate_output_cmd_.size());
  serial_port_->close();
  std::fill(thermal_data_.begin(), thermal_data_.end(), std::nan(""));
  sensor_temperature_ = std::nan("");
  return true;
}

ThermalData TerarangerThermalCommon::getThermalData()
{
  if (!isOpen())
  {
    throw std::runtime_error("Device is not initialized");
  }
  ThermalData result;
  std::lock_guard<std::mutex> l(m_);
  result.data = std::vector<float>(thermal_data_.begin(), thermal_data_.end());
  return result;
}

bool TerarangerThermalCommon::registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb)
{
  if (isOpen())
  {
    logger_->warn("Cannot register callback after device initialization");
    return false;
  }
  cb_ = cb;
  return true;
}

float TerarangerThermalCommon::getSensorTemperature()
{
  if (!isOpen())
  {
    throw std::runtime_error("Device is not initialized");
  }
  std::lock_guard<std::mutex> l(m_);
  return sensor_temperature_;
}

bool TerarangerThermalCommon::setEmissivity(uint8_t emissivity)
{
  logger_->debug("setEmissivity to {}", (int)emissivity);
  if (!isOpen())
  {
    logger_->warn("Device not initialized, cannot change settings");
    return false;
  }
  if (emissivity < 1 || emissivity > 100)
  {
    logger_->warn("Invalid emissivity value: {}", emissivity);
    return false;
  }
  std::array<uint8_t, 4> cmd = {0x00, EMISSIVITY_CMD_ADDR, emissivity, 0x00};
  cmd[3] = calcCrc8(cmd.data(), 3);
  serial_port_->write(cmd.data(), cmd.size());
  if (!waitForAck())
  {
    logger_->warn("Failed to send emissivity command");
    return false;
  }
  return true;
}

bool TerarangerThermalCommon::isOpen()
{
  return data_capture_thread_.joinable();
}

void TerarangerThermalCommon::captureData()
{
  uint8_t byte(0);
  serial_port_->read(&byte, 1);
  ring_buffer_.push_back(byte);
  checkIfAckAndNotify();
  if (!checkIfFrame()) return;
  const uint16_t* raw_thermal_ptr = reinterpret_cast<const uint16_t*>(ring_buffer_.data() + 2);
  {
    std::lock_guard<std::mutex> l(m_);
    std::transform(raw_thermal_ptr, raw_thermal_ptr + 1024, thermal_data_.data(),
      [](uint16_t val)
      {
        return static_cast<float>(val)/10;
      });
    sensor_temperature_ = static_cast<float>(raw_thermal_ptr[1024])/10;
  }
  if (cb_)
  {
    ThermalData result;
    result.data = std::vector<float>(thermal_data_.begin(), thermal_data_.end());
    cb_(result);
  }
}

void TerarangerThermalCommon::checkIfAckAndNotify()
{
  // ACK is [0x14 CMD ACK/NACK CRC] where CMD is always 0x05, ACK==0x00, NACK==0xff
  // always 0x05 because we only have 0x52 and 0x51 commands for thermal
  if (!(ring_buffer_[0] == 0x14 && ring_buffer_[1] == 0x05 &&
    (ring_buffer_[2] == 0 || ring_buffer_[2] == 0xff))) return;
  if (ring_buffer_[3] != calcCrc8(ring_buffer_.data(), 3))
  {
    logger_->debug("ACK, but wrong CRC: {0:x}", ring_buffer_[3]);
    return;
  }
  if (ring_buffer_[2] == 0xff) ack_flag_ = NACK;
  if (ring_buffer_[2] == 0x00) ack_flag_ = ACK;
  logger_->debug("Got ACK: {0:x} {1:x} {2:x} {3:x}", ring_buffer_[0], ring_buffer_[1],
    ring_buffer_[2], ring_buffer_[3]);
  cv_.notify_all();
}

bool TerarangerThermalCommon::checkIfFrame()
{
  uint16_t header(0);
  std::memcpy(&header, ring_buffer_.data(), 2);
  if (header != 0x000D) return false;
  if (!checkCrc32(ring_buffer_.data(), THERMAL_FRAME_SIZE))
  {
    logger_->warn("CRC mismatch");
    return false;
  }
  return true;
}

bool TerarangerThermalCommon::waitForAck()
{
  std::unique_lock<std::mutex> l(m_);
  cv_.wait_for(l, SERIAL_READ_TIMEOUT_COMMON, [this]()
  {
    return ack_flag_ != NO_ACK;
  });
  bool result = (ack_flag_ == ACK)? true : false;
  ack_flag_ = NO_ACK;
  return result;
}

bool TerarangerThermalCommon::checkCrc32(const uint8_t* buff, size_t size)
{
  uint32_t crc = calcCrc32(buff + 2, size - 6);  // header not included, 4 bytes CRC
  uint32_t received_crc(0);
  int index = size - 4;
  ((uint8_t*)(&received_crc))[0] = buff[index+2];
  ((uint8_t*)(&received_crc))[1] = buff[index+3];
  ((uint8_t*)(&received_crc))[2] = buff[index];
  ((uint8_t*)(&received_crc))[3] = buff[index+1];
  if (received_crc != crc)
  {
    logger_->warn("CRC mismatch {0:x} vs {1:x}", received_crc, crc);
    return false;
  }
  return true;
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
