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
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "terabee/internal/common_definitions.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/endian/swapEndian.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerEvoMini.hpp"

using terabee::internal::crc::calcCrc8;
using terabee::internal::endian::swapEndian;
using terabee::internal::serial_communication::ISerial;

#define PX1_FRAME_SIZE 4
#define PX2_FRAME_SIZE 6
#define PX4_FRAME_SIZE 10

namespace terabee
{
namespace internal
{
namespace teraranger
{

namespace
{
  const std::array<uint8_t, 4> g_binary_mode_cmd = {0x00, 0x11, 0x02, 0x4C};
  const std::array<uint8_t, 4> g_px1_mode_cmd = {0x00, 0x21, 0x01, 0xBC};
  const std::array<uint8_t, 4> g_px2_mode_cmd = {0x00, 0x21, 0x03, 0xB2};
  const std::array<uint8_t, 4> g_px4_mode_cmd = {0x00, 0x21, 0x02, 0xB5};
  const std::array<uint8_t, 4> g_short_mode_cmd = {0x00, 0x61, 0x01, 0xE7};
  const std::array<uint8_t, 4> g_long_mode_cmd = {0x00, 0x61, 0x03, 0xE9};
}  // namespace

TerarangerEvoMini::TerarangerEvoMini(std::shared_ptr<ISerial> serial_port):
  logger_("TerarangerEvoMini"),
  serial_port_(serial_port),
  cb_(nullptr),
  stop_(true),
  ring_buffer_(PX4_FRAME_SIZE),
  current_distance_(),
  pixel_mode_(Px1Mode),
  range_mode_(LongRangeMode),
  data_capture_thread_(),
  m_()
{
  logger_->debug("CTor");
  current_distance_.distance = std::vector<float>(1, std::nan(""));
  serial_port_->setBaudrate(115200);
  serial_port_->setBytesize(ISerial::eightbits);
  serial_port_->setStopbits(ISerial::stopbits_one);
  serial_port_->setParity(ISerial::parity_none);
  serial_port_->setFlowcontrol(ISerial::flowcontrol_none);
  serial_port_->setTimeout(SERIAL_READ_TIMEOUT_COMMON);
}

TerarangerEvoMini::~TerarangerEvoMini()
{
  logger_->debug("DTor");
  shutDown();
}

bool TerarangerEvoMini::initialize()
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
  activateBinaryMode();
  activateChosenPixelMode();
  activateChosenRangeMode();
  stop_ = false;
  data_capture_thread_ = std::thread([this]()
    {
      while (!stop_)
      {
        captureData();
      }
    });
  return true;
}

bool TerarangerEvoMini::shutDown()
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

DistanceData TerarangerEvoMini::getDistance()
{
  if (!initialized())
  {
    logger_->warn("Cannot getDistance is not initialized");
    throw std::runtime_error("Cannot getDistance is not initialized");
  }
  std::lock_guard<std::mutex> l(m_);
  return current_distance_;
}

bool TerarangerEvoMini::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  if (initialized())
  {
    logger_->warn("Cannot change callback after device initialization");
    return false;
  }
  cb_ = cb;
  return true;
}

ITerarangerEvoMini::PixelMode TerarangerEvoMini::getPixelMode() const
{
  return pixel_mode_;
}

ITerarangerEvoMini::RangeMode TerarangerEvoMini::getRangeMode() const
{
  return range_mode_;
}

bool TerarangerEvoMini::setPixelMode(PixelMode mode)
{
  if (initialized())
  {
    logger_->warn("Cannot change PixelMode after device initialization");
    return false;
  }
  pixel_mode_ = mode;
  if (pixel_mode_ == Px1Mode)
  {
    current_distance_.distance = std::vector<float>(1, std::nan(""));
  }
  if (pixel_mode_ == Px2Mode)
  {
    current_distance_.distance = std::vector<float>(2, std::nan(""));
  }
  if (pixel_mode_ == Px4Mode)
  {
    current_distance_.distance = std::vector<float>(4, std::nan(""));
  }
  return true;
}

bool TerarangerEvoMini::setRangeMode(RangeMode mode)
{
  if (initialized())
  {
    logger_->warn("Cannot change RangeMode after device initialization");
    return false;
  }
  range_mode_ = mode;
  return true;
}

bool TerarangerEvoMini::initialized()
{
  return data_capture_thread_.joinable();
}

void TerarangerEvoMini::captureData()
{
  uint8_t byte(0);
  serial_port_->read(&byte, 1);
  ring_buffer_.push_back(byte);
  if (ring_buffer_[0] != 0x54) return;
  size_t frame_size(0);
  if (pixel_mode_ == Px1Mode) frame_size = PX1_FRAME_SIZE;
  if (pixel_mode_ == Px2Mode) frame_size = PX2_FRAME_SIZE;
  if (pixel_mode_ == Px4Mode) frame_size = PX4_FRAME_SIZE;
  uint8_t crc = calcCrc8(ring_buffer_.data(), frame_size-1);
  if (ring_buffer_[frame_size-1] != crc)
  {
    logger_->debug("Incorrect CRC {0:x} vs {1:x}",
      ring_buffer_[frame_size-1], crc);
    return;
  }
  {
    std::lock_guard<std::mutex> l(m_);
    std::transform((uint16_t*)(ring_buffer_.data()+1),
      (uint16_t*)(ring_buffer_.data()+frame_size-1),
      current_distance_.distance.begin(), [](uint16_t d) -> float
      {
        if (d == 0x0000) return -std::numeric_limits<float>::infinity();
        if (d == 0xffff) return std::numeric_limits<float>::infinity();
        if (d == 0x0100) return -1;
        return static_cast<float>(swapEndian(d))/1000;
      });
  }
  if (cb_)
  {
    cb_(current_distance_);
  }
}

void TerarangerEvoMini::activateBinaryMode()
{
  logger_->info("activateBinaryMode");
  serial_port_->write(g_binary_mode_cmd.data(), g_binary_mode_cmd.size());
}

void TerarangerEvoMini::activateChosenPixelMode()
{
  logger_->info("activateChosenPixelMode({})", pixel_mode_);
  std::array<uint8_t, 4> cmd;
  if (pixel_mode_ == Px1Mode) cmd = g_px1_mode_cmd;
  if (pixel_mode_ == Px2Mode) cmd = g_px2_mode_cmd;
  if (pixel_mode_ == Px4Mode) cmd = g_px4_mode_cmd;
  serial_port_->write(cmd.data(), cmd.size());
}

void TerarangerEvoMini::activateChosenRangeMode()
{
  logger_->info("activateChosenRangeMode({})", range_mode_);
  std::array<uint8_t, 4> cmd;
  if (range_mode_ == ShortRangeMode) cmd = g_short_mode_cmd;
  if (range_mode_ == LongRangeMode) cmd = g_long_mode_cmd;
  serial_port_->write(cmd.data(), cmd.size());;
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
