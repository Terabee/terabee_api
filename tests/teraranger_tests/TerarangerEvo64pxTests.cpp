#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/teraranger/TerarangerEvo64px.hpp"
#include "terabee/ITerarangerEvo64px.hpp"

#include "mocks/MockISerial.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)

using terabee::DistanceData;
using terabee::internal::crc::calcCrc8;
using terabee::internal::crc::calcCrc32;
using terabee::internal::logger::Logger;
using terabee::internal::teraranger::TerarangerEvo64px;
using terabee::ITerarangerEvo64px;

using terabee::internal::serial_communication::MockISerial;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

std::ifstream::pos_type filesize(const std::string& filename)
{
  std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
  return in.tellg();
}

class TerarangerEvo64pxFixture: public testing::Test
{
protected:
  enum ReplyMode
  {
    ReplyCommand = 1,
    ReplyData = 2,
  };
  enum ReplyDataType
  {
    ReplyDistance = 1,
    ReplyDistanceAmbient = 2
  };
  TerarangerEvo64pxFixture():
    logger_("TerarangerEvo64pxFixture"),
    test_data_dir_(__FILE__),
    m_(),
    read_cv_(),
    read_times_(0),
    reply_mode_(ReplyCommand),
    reply_data_type_(ReplyDistanceAmbient),
    data_stream_on_(false),
    last_cmd_({0, 0, 0, 0}),
    reply_cmd_({0x14, 0, 0, 0}),  // ACK [0x14, CMD, ACK/NACK, CRC]
    distance_ambient_data_(),
    distance_data_()
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
    test_data_dir_ = test_data_dir_.substr(0,
        test_data_dir_.find("TerarangerEvo64pxTests.cpp"));
    test_data_dir_ += "test_data/";
    serial_mock_.reset(new NiceMock<MockISerial>);
    ON_CALL(*serial_mock_, open()).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, close()).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setBaudrate(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setParity(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setBytesize(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setStopbits(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setFlowcontrol(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setTimeout(_)).WillByDefault(Return(true));
    populateData(test_data_dir_ + "distance_ambient_normal_range.bin");
    populateData(test_data_dir_ + "only_distance_data.bin");
    configureSerialReadWrite();
    sut_.reset(new TerarangerEvo64px(serial_mock_));
  }
  ~TerarangerEvo64pxFixture()
  {
    logger_->info("{} END with {}",
      testing::UnitTest::GetInstance()->current_test_info()->name(),
      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }
  bool waitForRead()
  {
    std::unique_lock<std::mutex> l(m_);
    return read_cv_.wait_for(l, CV_TIMEOUT,
      [this]()
      {
        return read_times_.load() > 2;
      });
  }
  void configureSerialReadWrite()
  {
    ON_CALL(*serial_mock_, write(_, _)).WillByDefault(Invoke(
      [this](const uint8_t* data, size_t size)
      {
        logger_->debug("write: [{0:x}, {1:x}, {2:x}]", data[1], data[2], data[3]);
        reply_mode_ = ReplyCommand;
        if (size < 4 || calcCrc8(data, size-1) != data[size-1])
        {
          reply_cmd_[1] = 0xff;  // NACK with ff, because wrong cmd
          reply_cmd_[2] = 0xff;
          reply_cmd_[3] = calcCrc8(reply_cmd_.data(), 3);
          return size;
        }
        std::memcpy(last_cmd_.data(), data, size);
        if (size == 5 && last_cmd_[1] == 0x52 && last_cmd_[2] == 0x02)
        {
          if (last_cmd_[3] == 0x00) data_stream_on_ = false;
          else if (last_cmd_[3] == 0x01) data_stream_on_ = true;
        }
        if (size == 4 && last_cmd_[1] == 0x11)
        {
          if (last_cmd_[2] == 0x02) reply_data_type_ = ReplyDistance;
          else if (last_cmd_[2] == 0x03) reply_data_type_ = ReplyDistanceAmbient;
        }
        reply_cmd_[1] = last_cmd_[1]>>4;
        reply_cmd_[3] = calcCrc8(reply_cmd_.data(), 3);
        return size;
      }));
    ON_CALL(*serial_mock_, read(_, _)).WillByDefault(Invoke(
      [this](uint8_t* data, size_t size)
      {
        if (size < 4) return size;
        if (reply_mode_ == ReplyCommand)
        {
          std::memcpy(data, reply_cmd_.data(), size);
        }
        else if (reply_mode_ == ReplyData && data_stream_on_)
        {
          logger_->debug("assign data: {}", reply_data_type_);
          if (reply_data_type_ == ReplyDistanceAmbient)
          {
            std::memcpy(data, distance_ambient_data_.data(), size);
          }
          else if (reply_data_type_ == ReplyDistance)
          {
            std::memcpy(data, distance_data_.data(), size);
          }
          read_times_++;
          read_cv_.notify_all();
        }
        reply_mode_ = ReplyData;
        return size;
      }));
  }
  void populateData(const std::string& filename)
  {
    if (filesize(filename) == 269)
    {
      std::ifstream in(filename, std::ifstream::binary);
      in.read(reinterpret_cast<char*>(distance_ambient_data_.data()), 269);
    }
    else if (filesize(filename) == 141)
    {
      std::ifstream in(filename, std::ifstream::binary);
      in.read(reinterpret_cast<char*>(distance_data_.data()), 141);
    }
    else
    {
      logger_->warn("Data file size must be either 269 or 141 bytes, but is {}",
        filesize(filename));
    }
  }
  Logger logger_;
  std::string test_data_dir_;
  std::mutex m_;
  std::condition_variable read_cv_;
  std::atomic_int read_times_;
  ReplyMode reply_mode_;
  ReplyDataType reply_data_type_;
  bool data_stream_on_;
  std::array<uint8_t, 5> last_cmd_;
  std::array<uint8_t, 4> reply_cmd_;
  std::array<uint8_t, 269> distance_ambient_data_;
  std::array<uint8_t, 141> distance_data_;
  std::shared_ptr<MockISerial> serial_mock_;
  std::unique_ptr<ITerarangerEvo64px> sut_;
};

TEST_F(TerarangerEvo64pxFixture, returnTrueOnInitializeAndShutdownReturnFalseIfCalledTwice)
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

TEST_F(TerarangerEvo64pxFixture, returnDefaultSettingsIfNotChamgedBefore)
{
  ASSERT_EQ(ITerarangerEvo64px::OutputModeDistanceAmbient, sut_->getOutputMode());
  ASSERT_EQ(ITerarangerEvo64px::MeasurementModeCloseRange, sut_->getMeasurementMode());
}

TEST_F(TerarangerEvo64pxFixture, throwExceptionIfGetDataWithoutInitialize)
{
  ASSERT_THROW(sut_->getDistanceValues(), std::runtime_error);
  ASSERT_THROW(sut_->getAmbientValues(), std::runtime_error);
  EXPECT_CALL(*serial_mock_, open()).WillRepeatedly(Return(false));
  ASSERT_FALSE(sut_->initialize());
  ASSERT_THROW(sut_->getDistanceValues(), std::runtime_error);
  ASSERT_THROW(sut_->getAmbientValues(), std::runtime_error);
}

TEST_F(TerarangerEvo64pxFixture, returnFalseIfAttemptToChangeSettingsAfterInitialize)
{
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->setOutputMode(ITerarangerEvo64px::OutputModeDistance));
  ASSERT_FALSE(sut_->setMeasurementMode(ITerarangerEvo64px::MeasurementModeFast));
}

TEST_F(TerarangerEvo64pxFixture, failToInitializeIfGotNack)
{
  reply_cmd_[2] = 0xFF;  // NACK
  ASSERT_FALSE(sut_->initialize());
}

TEST_F(TerarangerEvo64pxFixture, returnNanIfConfiguredToNotMeasureAmbient)
{
  ASSERT_TRUE(sut_->setOutputMode(ITerarangerEvo64px::OutputModeDistance));
  ASSERT_TRUE(sut_->setMeasurementMode(ITerarangerEvo64px::MeasurementModeFast));
  ASSERT_TRUE(sut_->initialize());
  waitForRead();
  auto ambient = sut_->getAmbientValues();
  ASSERT_TRUE(std::all_of(ambient.distance.begin(), ambient.distance.end(),
    [](float v)
    {
      return std::isnan(v);
    }));
}

TEST_F(TerarangerEvo64pxFixture, returnCorrectDistanceAmbientDataInBothModes)
{
  float expected_distance_value(0.6);  // from distance_ambient_normal_range
  float expected_ambient_value(9);
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto distance = sut_->getDistanceValues();
  auto ambient = sut_->getAmbientValues();
  ASSERT_FLOAT_EQ(expected_distance_value, *(distance.distance.end()-2));
  ASSERT_FLOAT_EQ(expected_ambient_value, ambient.distance.front());
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_TRUE(sut_->setOutputMode(ITerarangerEvo64px::OutputModeDistance));
  ASSERT_TRUE(sut_->setMeasurementMode(ITerarangerEvo64px::MeasurementModeFast));
  read_times_ = 0;
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  expected_distance_value = 1.42;  // from only_distance_data
  distance = sut_->getDistanceValues();
  ASSERT_FLOAT_EQ(expected_distance_value, distance.distance.back());
}

TEST_F(TerarangerEvo64pxFixture, returnMinusInfForObjectTooCloseMeasurement)
{
  populateData(test_data_dir_ + "distance_ambient_close_range.bin");
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto distance = sut_->getDistanceValues();
  ASSERT_TRUE(std::all_of(distance.distance.begin(), distance.distance.end(),
    [](float v)
    {
      return v == -std::numeric_limits<float>::infinity();
    }));
}

TEST_F(TerarangerEvo64pxFixture, returnInfForObjectOutOfRange)
{
  populateData(test_data_dir_ + "distance_ambient_out_of_range.bin");
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto distance = sut_->getDistanceValues();
  ASSERT_TRUE(std::all_of(distance.distance.begin(), distance.distance.end(),
    [](float v)
    {
      return v == std::numeric_limits<float>::infinity();
    }));
}

TEST_F(TerarangerEvo64pxFixture, returnMinusOneForMeasurementError)
{
  populateData(test_data_dir_ + "distance_ambient_measurement_error.bin");
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto distance = sut_->getDistanceValues();
  ASSERT_TRUE(std::all_of(distance.distance.begin(), distance.distance.end(),
    [](float v)
    {
      return v == -1;
    }));
}

TEST_F(TerarangerEvo64pxFixture, returnNanIfReadSomeGarbage)
{
  populateData(test_data_dir_ + "distance_ambient_garbage.bin");
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto distance = sut_->getDistanceValues();
  ASSERT_TRUE(std::all_of(distance.distance.begin(), distance.distance.end(),
    [](float v)
    {
      return std::isnan(v);
    }));
}

TEST_F(TerarangerEvo64pxFixture, invokeCallbackOnNewDataCapture)
{
  float expected_distance_value(0.6);  // from distance_ambient_normal_rang
  int num_callbacks = 0;
  ASSERT_TRUE(sut_->registerOnDistanceDataCaptureCallback(
    [this, expected_distance_value, &num_callbacks](const DistanceData& d)
    {
      logger_->debug("callback");
      EXPECT_FLOAT_EQ(expected_distance_value, *(d.distance.end()-2));
      num_callbacks++;
    }));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->registerOnDistanceDataCaptureCallback([](const DistanceData&){}));
  ASSERT_FALSE(sut_->registerOnDistanceDataCaptureCallback(nullptr));
  /*
   * waitForRead condition is read_times > 2, so we can expect that callback is invoked
   * at least 2 times
   */
  ASSERT_TRUE(waitForRead());
  ASSERT_GT(num_callbacks, 1);
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_TRUE(sut_->registerOnDistanceDataCaptureCallback(nullptr));
  num_callbacks = 0;
  read_times_ = 0;
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  ASSERT_EQ(0, num_callbacks);  // to make sure the callback was canceled
}
