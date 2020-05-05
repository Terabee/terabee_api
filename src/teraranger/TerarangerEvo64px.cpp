/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/common_definitions.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerEvo64px.hpp"

using terabee::internal::crc::calcCrc8;
using terabee::internal::crc::calcCrc32;
using terabee::internal::serial_communication::ISerial;

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvo64px::TerarangerEvo64px(std::shared_ptr<serial_communication::ISerial> serial_port):
  output_distance_cmd_({0x00, 0x11, 0x02, 0x4C}),
  output_distance_ambient_cmd_({0x00, 0x11, 0x03, 0x4B}),
  close_range_mode_cmd_({0x00, 0x21, 0x01, 0xBC}),
  fast_mode_cmd_({0x00, 0x21, 0x02, 0xB5}),
  activate_output_cmd_({0x00, 0x52, 0x02, 0x01, 0xDF}),
  deactivate_output_cmd_({0x00, 0x52, 0x02, 0x00, 0xD8}),
  logger_("TerarangerEvo64px"),
  serial_port_(serial_port),
  cb_(nullptr),
  data_capture_thread_(),
  stop_(true),
  m_(),
  output_mode_(OutputModeDistanceAmbient),
  measurement_mode_(MeasurementModeCloseRange),
  distance_(),
  ambient_()
{
  logger_->debug("TerarangerEvo64px::TerarangerEvo64px");
  std::fill(distance_.begin(), distance_.end(), std::nan(""));
  std::fill(ambient_.begin(), ambient_.end(), std::nan(""));
  serial_port_->setBaudrate(115200);
  serial_port_->setBytesize(ISerial::eightbits);
  serial_port_->setStopbits(ISerial::stopbits_one);
  serial_port_->setParity(ISerial::parity_none);
  serial_port_->setFlowcontrol(ISerial::flowcontrol_none);
  // Tiemout is set to 1 sec to accomodate occasional low update rate
  serial_port_->setTimeout(SERIAL_READ_TIMEOUT_COMMON);
}

TerarangerEvo64px::~TerarangerEvo64px()
{
  logger_->debug("TerarangerEvo64px::~TerarangerEvo64px");
  shutDown();
}
bool TerarangerEvo64px::initialize()
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
  // Output is deactivated for easier command processing
  sendCommand(deactivate_output_cmd_.data(), deactivate_output_cmd_.size());
  std::array<uint8_t, 4> cmd;
  if (output_mode_ == OutputModeDistanceAmbient)
  {
    cmd = output_distance_ambient_cmd_;
  }
  else if(output_mode_ == OutputModeDistance)
  {
    cmd = output_distance_cmd_;
  }
  if (!sendCommand(cmd.data(), cmd.size()))
  {
    logger_->warn("Failed to set output mode to {}", output_mode_);
    serial_port_->close();
    return false;
  }
  if (measurement_mode_ == MeasurementModeCloseRange)
  {
    cmd = close_range_mode_cmd_;
  }
  else if (measurement_mode_ == MeasurementModeFast)
  {
    cmd = fast_mode_cmd_;
  }
  if (!sendCommand(cmd.data(), cmd.size()))
  {
    logger_->warn("Failed to set measurement mode to {}", measurement_mode_);
    serial_port_->close();
    return false;
  }
  if (!sendCommand(activate_output_cmd_.data(), activate_output_cmd_.size()))
  {
    logger_->warn("Failed to activate output");
    serial_port_->close();
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
  return true;
}

bool TerarangerEvo64px::shutDown()
{
  logger_->debug("shutDown");
  if (!isOpen())
  {
    logger_->warn("Not initialized");
    return false;
  }
  stop_ = true;
  data_capture_thread_.join();
  sendCommand(deactivate_output_cmd_.data(), deactivate_output_cmd_.size());
  serial_port_->close();
  std::fill(distance_.begin(), distance_.end(), std::nan(""));
  std::fill(ambient_.begin(), ambient_.end(), std::nan(""));
  return true;
}

ITerarangerEvo64px::OutputMode TerarangerEvo64px::getOutputMode() const
{
  return output_mode_;
}

ITerarangerEvo64px::MeasurementMode TerarangerEvo64px::getMeasurementMode() const
{
  return measurement_mode_;
}

bool TerarangerEvo64px::setOutputMode(OutputMode mode)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings of initialized device");
    return false;
  }
  output_mode_ = mode;
  return true;
}

bool TerarangerEvo64px::setMeasurementMode(MeasurementMode mode)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings of initialized device");
    return false;
  }
  measurement_mode_ = mode;
  return true;
}

DistanceData TerarangerEvo64px::getDistanceValues()
{
  if (!isOpen())
  {
    throw std::runtime_error("Device is not initialized");
  }
  DistanceData result;
  std::lock_guard<std::mutex> l(m_);
  result.distance = std::vector<float>(distance_.begin(), distance_.end());
  return result;
}

bool TerarangerEvo64px::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  if (isOpen())
  {
    logger_->warn("Cannot change callback after device initialization");
    return false;
  }
  cb_ = cb;
  return true;
}

DistanceData TerarangerEvo64px::getAmbientValues()
{
  if (!isOpen())
  {
    throw std::runtime_error("Device is not initialized");
  }
  DistanceData result;
  std::lock_guard<std::mutex> l(m_);
  result.distance = std::vector<float>(ambient_.begin(), ambient_.end());
  return result;
}

bool TerarangerEvo64px::isOpen()
{
  return data_capture_thread_.joinable();
}

bool TerarangerEvo64px::sendCommand(const uint8_t* cmd, size_t size)
{
  serial_port_->write(cmd, size);
  uint8_t reply[4];
  serial_port_->read(reply, 4);
  if (!(
    reply[0] == 0x14 &&
    reply[1] == (cmd[1] >> 4) &&
    reply[2] == 0x00 &&
    reply[3] == calcCrc8(reply, 3)))
  {
    logger_->warn("Did not receive ACK");
    logger_->debug("Got: [{0:x} {1:x} {2:x} {3:x}]", reply[0], reply[1], reply[2], reply[3]);
    return false;
  }
  return true;
}

void TerarangerEvo64px::captureData()
{
  std::array<uint8_t, 269> buff;
  size_t frame_size = output_mode_ == OutputModeDistanceAmbient ? 269 : 141;
  serial_port_->read(buff.data(), frame_size);
  if (buff[0] != 0x11)
  {
    logger_->error("Wrong header of distance data: {}", buff[0]);
    std::lock_guard<std::mutex> l(m_);
    std::fill(distance_.begin(), distance_.end(), std::nan(""));
    return;
  }
  if (output_mode_ == OutputModeDistanceAmbient && buff[129] != 0x13)
  {
    logger_->error("Wrong header of ambient data: {}", buff[129]);
    std::lock_guard<std::mutex> l(m_);
    std::fill(distance_.begin(), distance_.end(), std::nan(""));
    return;
  }
  if (!checkCrc32(buff.data(), frame_size - 1))  // -1 because we drop last garbage byte 0x0A
  {
    logger_->warn("CRC mismatch");
    std::lock_guard<std::mutex> l(m_);
    std::fill(distance_.begin(), distance_.end(), std::nan(""));
    return;
  }
  auto dist = distFromRawData(buff.data() + 1);
  {
    std::lock_guard<std::mutex> l(m_);
    std::transform(dist.begin(), dist.end(), distance_.begin(),
      [](uint16_t val) -> float
      {
        if (val == 0) return -std::numeric_limits<float>::infinity();
        else if (val == 1) return -1;
        else if (val == 0x3FFF) return std::numeric_limits<float>::infinity();
        return static_cast<float>(val)/1000;
      });
    if (output_mode_ == OutputModeDistanceAmbient)
    {
      auto amb = ambFromRawData(buff.data() + 130);
      std::transform(amb.begin(), amb.end(), ambient_.begin(),
        [](uint16_t val)
        {
          return static_cast<float>(val);
        });
    }
  }
  if (cb_)
  {
    DistanceData result;
    result.distance = std::vector<float>(distance_.begin(), distance_.end());
    cb_(result);
  }
}

bool TerarangerEvo64px::checkCrc32(const uint8_t* buff, size_t size) {
  uint32_t crc = calcCrc32(buff, size - 8);
  uint32_t crc_value(0);
  int index = size - 8;
  crc_value = (buff[index] & 0x0F) << 28;
  crc_value |= (buff[index + 1] & 0x0F) << 24;
  crc_value |= (buff[index + 2] & 0x0F) << 20;
  crc_value |= (buff[index + 3] & 0x0F) << 16;
  crc_value |= (buff[index + 4] & 0x0F) << 12;
  crc_value |= (buff[index + 5] & 0x0F) << 8;
  crc_value |= (buff[index + 6] & 0x0F) << 4;
  crc_value |= (buff[index + 7] & 0x0F);
  crc_value = crc_value & 0xFFFFFFFF;
  if (crc_value != crc)
  {
    logger_->warn("CRC mismatch {0:x} vs {1:x}", crc_value, crc);
    return false;
  }
  return true;
}

std::array<uint16_t, 64> TerarangerEvo64px::distFromRawData(const uint8_t* data) {
  std::array<uint16_t, 64> result;
  for (int i = 0; i < 64; i++) {
    uint16_t val;
    // merge 7 bit numbers
    val = (data[2*i] << 7) | (data[2*i+1] & 0x7F);
    val &= 0x3FFF;
    result[i] = val;
  }
  return result;
}

std::array<uint16_t, 64> TerarangerEvo64px::ambFromRawData(const uint8_t* data) {
  std::array<uint16_t, 64> result;
  for (int i = 0; i < 64; i++) {
    uint16_t val;
    // merge 6 bit numbers
    val = (data[2*i] << 7) | (data[2*i+1] & 0x7F);
    val &= 0x3FFF;
    result[i] = val;
  }
  return result;
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
