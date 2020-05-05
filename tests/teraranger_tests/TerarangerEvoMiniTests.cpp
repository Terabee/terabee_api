#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <fstream>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/endian/swapEndian.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/teraranger/TerarangerEvoMini.hpp"

#include "mocks/MockISerial.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)
#define DATA_FRAME_SIZE 10
#define CMD_SIZE 4

using terabee::DistanceData;
using terabee::internal::crc::calcCrc8;
using terabee::internal::endian::swapEndian;
using terabee::internal::logger::Logger;
using terabee::internal::ring_buffer::RingBuffer;
using terabee::internal::teraranger::TerarangerEvoMini;
using terabee::ITerarangerEvoMini;

using terabee::internal::serial_communication::MockISerial;

using testing::_;
using testing::DoDefault;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

static const uint8_t px1_cmd[CMD_SIZE] = {0x00, 0x21, 0x01, 0xBC};
static const uint8_t px2_cmd[CMD_SIZE] = {0x00, 0x21, 0x03, 0xB2};
static const uint8_t px4_cmd[CMD_SIZE] = {0x00, 0x21, 0x02, 0xB5};

class TerarangerEvoMiniFixture: public testing::Test
{
protected:
  TerarangerEvoMiniFixture():
    logger_("TerarangerEvoMiniFixture"),
    serial_open_(false),
    current_data_idx_(0),
    data_frame_(DATA_FRAME_SIZE),
    data_read_counter_(0),
    read_cv_(),
    m_(),
    fake_measurement_({0, 0, 0, 0})
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
    serial_mock_.reset(new NiceMock<MockISerial>);
    ON_CALL(*serial_mock_, open()).WillByDefault(Invoke([this]()
      {
        if (serial_open_) return false;
        serial_open_ = true;
        return true;
      }));
    ON_CALL(*serial_mock_, close()).WillByDefault(Invoke([this]()
      {
        if (!serial_open_) return false;
        serial_open_ = false;
        return true;
      }));
    configureSerialReadWrite();
    sut_.reset(new TerarangerEvoMini(serial_mock_));
  }
  ~TerarangerEvoMiniFixture()
  {
    logger_->info("{} END with {}",
      testing::UnitTest::GetInstance()->current_test_info()->name(),
      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }
  void configureSerialReadWrite()
  {
    ON_CALL(*serial_mock_, write(_, _)).WillByDefault(Invoke(
      [this](const uint8_t* cmd, size_t size) -> size_t
      {
        if (!serial_open_) return 0;
        if (size == CMD_SIZE && cmd[0] == 0x00 && cmd[1] == 0x21)
        {
          if (std::memcmp(px1_cmd, cmd, CMD_SIZE) == 0) reconfigureDataFrame(1);
          if (std::memcmp(px2_cmd, cmd, CMD_SIZE) == 0) reconfigureDataFrame(2);
          if (std::memcmp(px4_cmd, cmd, CMD_SIZE) == 0) reconfigureDataFrame(4);
        }
        return size;
      }));
    ON_CALL(*serial_mock_, read(_, _)).WillByDefault(Invoke(
      [this](uint8_t* buff, size_t size) -> size_t
      {
        if (!serial_open_) return 0;
        size_t bytes_to_send = size;
        size_t i = 0;
        while (bytes_to_send > 0)
        {
          buff[i] = data_frame_[current_data_idx_ + i];
          current_data_idx_++;
          i++;
          bytes_to_send--;
        }
        data_read_counter_ += size;
        read_cv_.notify_all();
        return size;
      }));
  }
  void reconfigureDataFrame(uint8_t num_px)
  {
    logger_->debug("reconfigureDataFrame {}", num_px);
    data_frame_[0] = 0x54; // header
    for (size_t i = 0; i < num_px; i++)
    {
      size_t idx = 1 + 2*i;
      auto d = fake_measurement_[i];
      data_frame_[idx+1] = ((uint8_t*)(&d))[0];
      data_frame_[idx] = ((uint8_t*)(&d))[1];
    }
    data_frame_[2*num_px+1] = calcCrc8(data_frame_.data(), 2*num_px+1);
  }
  bool waitForRead()
  {
    std::unique_lock<std::mutex> l(m_);
    return read_cv_.wait_for(l, CV_TIMEOUT,
      [this]()
      {
        return data_read_counter_.load() > 30;
      });
  }
  Logger logger_;
  bool serial_open_;
  size_t current_data_idx_;
  RingBuffer data_frame_;
  std::atomic_int data_read_counter_;
  std::condition_variable read_cv_;
  std::mutex m_;
  std::array<uint16_t, 4> fake_measurement_;
  std::shared_ptr<MockISerial> serial_mock_;
  std::unique_ptr<ITerarangerEvoMini> sut_;
};

TEST_F(TerarangerEvoMiniFixture, returnTrueOnInitializeAndShutdownReturnFalseIfCalledTwice)
{
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->initialize());
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_FALSE(sut_->shutDown());
  // Second call
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->initialize());
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_FALSE(sut_->shutDown());
}

TEST_F(TerarangerEvoMiniFixture, failToInitializeIfFailedToOpenSerial)
{
  EXPECT_CALL(*serial_mock_, open()).WillRepeatedly(Return(false));
  ASSERT_FALSE(sut_->initialize());
}

TEST_F(TerarangerEvoMiniFixture, returnDefaultSettingsIfNotChamgedBefore)
{
  ASSERT_EQ(ITerarangerEvoMini::Px1Mode, sut_->getPixelMode());
  ASSERT_EQ(ITerarangerEvoMini::LongRangeMode, sut_->getRangeMode());
}

TEST_F(TerarangerEvoMiniFixture, throwExceptionIfGetDataWithoutInitialize)
{
  ASSERT_THROW(sut_->getDistance(), std::runtime_error);
  EXPECT_CALL(*serial_mock_, open()).WillRepeatedly(Return(false));
  ASSERT_FALSE(sut_->initialize());
  ASSERT_THROW(sut_->getDistance(), std::runtime_error);
}

TEST_F(TerarangerEvoMiniFixture, returnFalseIfChangeSettingsAfterInitialize)
{
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->setPixelMode(ITerarangerEvoMini::Px2Mode));
  ASSERT_FALSE(sut_->setRangeMode(ITerarangerEvoMini::ShortRangeMode));
}

TEST_F(TerarangerEvoMiniFixture, returnCorrectSettingsAfterSettingsChanges)
{
  ITerarangerEvoMini::PixelMode expected_px = ITerarangerEvoMini::Px2Mode;
  ITerarangerEvoMini::RangeMode expected_range = ITerarangerEvoMini::ShortRangeMode;
  ASSERT_TRUE(sut_->setPixelMode(expected_px));
  ASSERT_TRUE(sut_->setRangeMode(expected_range));
  ASSERT_EQ(expected_px, sut_->getPixelMode());
  ASSERT_EQ(expected_range, sut_->getRangeMode());
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->setPixelMode(ITerarangerEvoMini::Px4Mode));
  ASSERT_FALSE(sut_->setRangeMode(ITerarangerEvoMini::LongRangeMode));
  ASSERT_EQ(expected_px, sut_->getPixelMode());
  ASSERT_EQ(expected_range, sut_->getRangeMode());
}

TEST_F(TerarangerEvoMiniFixture, returnMeasurementOfCorrectSizeInEachMode)
{
  ASSERT_TRUE(sut_->initialize());
  auto result = sut_->getDistance();
  ASSERT_EQ(1, result.distance.size());  // default mode is one pixel
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_TRUE(sut_->setPixelMode(ITerarangerEvoMini::Px2Mode));
  ASSERT_TRUE(sut_->initialize());
  result = sut_->getDistance();
  ASSERT_EQ(2, result.distance.size());
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_TRUE(sut_->setPixelMode(ITerarangerEvoMini::Px4Mode));
  ASSERT_TRUE(sut_->initialize());
  result = sut_->getDistance();
  ASSERT_EQ(4, result.distance.size());
  ASSERT_TRUE(sut_->shutDown());
}

TEST_F(TerarangerEvoMiniFixture, returnCorrectCornerCasesDistanceValues)
{
  fake_measurement_[0] = 0x0000;  // too close
  fake_measurement_[1] = 0xffff;  // too far
  fake_measurement_[2] = 0x0001;  // measurement failure
  ASSERT_TRUE(sut_->setPixelMode(ITerarangerEvoMini::Px4Mode));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto result = sut_->getDistance();
  ASSERT_EQ(-std::numeric_limits<float>::infinity(), result.distance[0]);
  ASSERT_EQ(std::numeric_limits<float>::infinity(), result.distance[1]);
  ASSERT_FLOAT_EQ(-1, result.distance[2]);
}

TEST_F(TerarangerEvoMiniFixture, returnCorrectlyMeasuredDistance)
{
  float expected_distance = 0.988;
  uint16_t dist_uint = 988;
  fake_measurement_[0] = dist_uint;
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto result = sut_->getDistance();
  ASSERT_FLOAT_EQ(expected_distance, result.distance.front());
}

TEST_F(TerarangerEvoMiniFixture, invokeCallbackOnNewDataCapture)
{
  float expected_distance = 1.234;
  uint16_t dist_uint = 1234;
  fake_measurement_[0] = dist_uint;
  int num_callbacks = 0;
  ASSERT_TRUE(sut_->registerOnDistanceDataCaptureCallback(
    [this, expected_distance, &num_callbacks](const DistanceData& d)
    {
      logger_->debug("callback");
      EXPECT_FLOAT_EQ(expected_distance, d.distance.front());
      num_callbacks++;
    }));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->registerOnDistanceDataCaptureCallback([](const DistanceData&){}));
  ASSERT_FALSE(sut_->registerOnDistanceDataCaptureCallback(nullptr));
  /*
   * with current testing strategy cb will be invoked 3 times, because we publish 10-bytes frames
   * (in 1px mode only 4 bytes are significant, the rest is unused);
   * But I don't think is makes sense to check how many times exactly it is invoked
   */
  ASSERT_TRUE(waitForRead());
  ASSERT_GT(num_callbacks, 1);
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_TRUE(sut_->registerOnDistanceDataCaptureCallback(nullptr));
  num_callbacks = 0;
  data_read_counter_ = 0;
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  ASSERT_EQ(0, num_callbacks);  // to make sure the callback was canceled
}
