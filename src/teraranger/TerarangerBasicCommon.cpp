/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/common_definitions.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/endian/swapEndian.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerBasicCommon.hpp"

using terabee::internal::crc::calcCrc8;
using terabee::internal::endian::swapEndian;
using terabee::internal::serial_communication::ISerial;

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerBasicCommon::TerarangerBasicCommon(std::shared_ptr<ISerial> serial_port,
  const std::string& name):
  enable_binary_mode_cmd_({0x00, 0x11, 0x02, 0x4C}),
  logger_(name),
  serial_port_(serial_port),
  cb_(nullptr),
  data_capture_thread_(),
  stop_(true),
  m_(),
  measurement_(std::nan(""))
{
  logger_->debug("CTor");
  serial_port_->setBaudrate(115200);
  serial_port_->setBytesize(ISerial::eightbits);
  serial_port_->setStopbits(ISerial::stopbits_one);
  serial_port_->setParity(ISerial::parity_none);
  serial_port_->setFlowcontrol(ISerial::flowcontrol_none);
  serial_port_->setTimeout(SERIAL_READ_TIMEOUT_COMMON);
}

TerarangerBasicCommon::~TerarangerBasicCommon()
{
  logger_->debug("DTor");
  shutDown();
}

bool TerarangerBasicCommon::initialize()
{
  logger_->debug("initialize");
  if (data_capture_thread_.joinable())
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
  serial_port_->write(enable_binary_mode_cmd_.data(), 4);
  data_capture_thread_ = std::thread(
    [this]()
    {
      serial_port_->flushInput();
      while (!stop_)
      {
        captureData();
      }
    });
  return true;
}

bool TerarangerBasicCommon::shutDown()
{
  logger_->debug("shutDown");
  if (!data_capture_thread_.joinable())
  {
    logger_->warn("Not initialized");
    return false;
  }
  measurement_ = std::nan("");
  stop_ = true;
  data_capture_thread_.join();
  serial_port_->close();
  return true;
}

DistanceData TerarangerBasicCommon::getDistance()
{
  logger_->debug("getDistance");
  DistanceData result;
  std::lock_guard<std::mutex> l(m_);
  result.distance.push_back(measurement_);
  return result;
}

bool TerarangerBasicCommon::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  if (data_capture_thread_.joinable())
  {
    logger_->warn("Cannot change callback after device initialization");
    return false;
  }
  cb_ = cb;
  return true;
}

void TerarangerBasicCommon::captureData()
{
  std::array<uint8_t, 4> data;
  if (serial_port_->read(data.data(), 4) != 4)
  {
    logger_->warn("Did not receive 4 bytes of data, setting result to NaN");
    std::lock_guard<std::mutex> l(m_);
    measurement_ = std::nan("");
    return;
  }
  if (data[0] != 0x54)
  {
    logger_->warn("Wrong data header, setting result to NaN");
    std::lock_guard<std::mutex> l(m_);
    measurement_ = std::nan("");
    return;
  }
  if (data[3] != calcCrc8(data.data(), 3))
  {
    logger_->warn("Wrong CRC, setting result to NaN");
    std::lock_guard<std::mutex> l(m_);
    measurement_ = std::nan("");
    return;
  }
  uint16_t raw = swapEndian(*((uint16_t*)&data[1]));
  std::lock_guard<std::mutex> l(m_);
  if (raw == 0)
  {
    logger_->debug("Too close, setting -Inf");
    measurement_ = -std::numeric_limits<float>::infinity();
  }
  else if (raw == 0xFFFF)
  {
    logger_->debug("Too far, setting Inf");
    measurement_ = std::numeric_limits<float>::infinity();
  }
  else if (raw == 0x0001)
  {
    logger_->debug("Measurement failure, setting -1");
    measurement_ = -1;
  }
  else
  {
    logger_->debug("Measurement in range: {}", raw);
    measurement_ = static_cast<float>(raw)/1000;
  }
  if (cb_)
  {
    DistanceData result;
    result.distance.push_back(measurement_);
    cb_(result);
  }
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
