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
#include "terabee/internal/teraranger/TerarangerMultiflex.hpp"

#include "mocks/MockISerial.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)
#define FLOAT_TOLERANCE 0.01

using terabee::DistanceData;
using terabee::internal::crc::calcCrc8;
using terabee::internal::endian::swapEndian;
using terabee::internal::logger::Logger;
using terabee::internal::ring_buffer::RingBuffer;
using terabee::internal::teraranger::TerarangerMultiflex;
using terabee::ITerarangerMultiflex;

using terabee::internal::serial_communication::MockISerial;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

#define DATA_FRAME_SIZE 20
#define REP_SIZE 5

class TerarangerMultiflexFixture: public testing::Test
{
protected:
  TerarangerMultiflexFixture():
    logger_("TerarangerMultiflexFixture"),
    current_data_idx_(0),
    ack_bytes_to_send_(0),
    ack_byte_(0x00),  // 0 is ACK
    data_frame_(DATA_FRAME_SIZE),
    data_read_counter_(0),
    read_cv_(),
    m_(),
    new_cmd_(false),
    test_data_dir_(__FILE__)
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
    std::memset(current_cmd_.data(), 0, current_cmd_.size());
    test_data_dir_ = test_data_dir_.substr(0,
      test_data_dir_.find("TerarangerMultiflexTests.cpp"));
    test_data_dir_ += "test_data/";
    loadDataFrameData(test_data_dir_ + "mf_8_sensors_one_frame.bin");
    serial_mock_.reset(new NiceMock<MockISerial>);
    ON_CALL(*serial_mock_, open()).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, close()).WillByDefault(Return(true));
    configureSerialReadWrite();
    sut_.reset(new TerarangerMultiflex(serial_mock_));
  }
  ~TerarangerMultiflexFixture()
  {
    logger_->info("{} END with {}",
      testing::UnitTest::GetInstance()->current_test_info()->name(),
      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }
  void loadDataFrameData(const std::string& data)
  {
    logger_->info("Going to read data from {}", data);
    std::ifstream in(data);
    for (size_t i = 0; i < DATA_FRAME_SIZE; i++)
    {
      uint8_t d(0);
      in.read(reinterpret_cast<char*>(&d), sizeof(d));
      data_frame_.push_back(d);
    }
  }
  void configureSerialReadWrite()
  {
    ON_CALL(*serial_mock_, write(_, _)).WillByDefault(Invoke(
      [this](const uint8_t* cmd, size_t size)
      {
        std::memcpy(current_cmd_.data(), cmd, size);
        new_cmd_ = true;
        return size;
      }));
    ON_CALL(*serial_mock_, read(_, _)).WillByDefault(Invoke(
      [this](uint8_t* buff, size_t size)
      {
        size_t bytes_to_send = size;
        if (new_cmd_)
        {
          logger_->debug("New command arrived, Going to send ACK/NACK");
          new_cmd_ = false;
          ack_bytes_to_send_ = REP_SIZE;
        }
        size_t i = 0;
        while (ack_bytes_to_send_ > 0 && bytes_to_send > 0)
        {
          buff[i] = generateReply(current_cmd_.data())[REP_SIZE - ack_bytes_to_send_];
          i++;
          ack_bytes_to_send_--;
          bytes_to_send--;
        }
        while (bytes_to_send > 0)
        {
          buff[i] = data_frame_[current_data_idx_];
          current_data_idx_++;
          i++;
          bytes_to_send--;
        }
        data_read_counter_ += size;
        read_cv_.notify_all();
        return size;
      }));
  }
  std::array<uint8_t, REP_SIZE> generateReply(const uint8_t* cmd)
  {
    std::array<uint8_t, REP_SIZE> result = {0x52, 0x45, 0x00, ack_byte_, 0x00};
    result[2] = cmd[1];
    result[4] = calcCrc8(result.data(), 4);
    return result;
  }
  bool waitForRead()
  {
    std::unique_lock<std::mutex> l(m_);
    return read_cv_.wait_for(l, CV_TIMEOUT,
      [this]()
      {
        return data_read_counter_.load() > 5*DATA_FRAME_SIZE;
      });
  }
  Logger logger_;
  size_t current_data_idx_;
  size_t ack_bytes_to_send_;
  uint8_t ack_byte_;
  RingBuffer data_frame_;
  std::atomic_int data_read_counter_;
  std::condition_variable read_cv_;
  std::mutex m_;
  bool new_cmd_;
  std::array<uint8_t, 5> current_cmd_;
  std::string test_data_dir_;
  std::shared_ptr<MockISerial> serial_mock_;
  std::unique_ptr<ITerarangerMultiflex> sut_;
};

TEST_F(TerarangerMultiflexFixture, returnTrueOnInitializeAndShutdownReturnFalseIfCalledTwice)
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

TEST_F(TerarangerMultiflexFixture, failToInitializeIfNackFromDevice)
{
  ack_byte_ = 0xff;  // ff is NACK
  ASSERT_FALSE(sut_->initialize());
}

TEST_F(TerarangerMultiflexFixture, sendInitializeCommandsWithCorrectCrc)
{
  ASSERT_TRUE(sut_->initialize());
  ASSERT_EQ(calcCrc8(current_cmd_.data(), 4), current_cmd_[4]);
}

TEST_F(TerarangerMultiflexFixture, returnNanIfReadsGarbageFromDevice)
{
  data_frame_ = RingBuffer(DATA_FRAME_SIZE);  // zero initialized buffer
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_TRUE(std::all_of(dist.distance.begin(), dist.distance.end(),
    [](float d) { return std::isnan(d); }));
}

TEST_F(TerarangerMultiflexFixture, returnFalseIfNackFromDeviceOrNotInitialized)
{
  ASSERT_FALSE(sut_->configureNumberOfSensors(0xff));
  ASSERT_TRUE(sut_->initialize());
  ack_byte_ = 0xff;
  ASSERT_FALSE(sut_->configureNumberOfSensors(0xff));
}

TEST_F(TerarangerMultiflexFixture, returnNanForDisabledSensors)
{
  // every second sensor disabled
  uint8_t mask = 0xaa;
  data_frame_[DATA_FRAME_SIZE-2] = mask;
  data_frame_[DATA_FRAME_SIZE-1] = calcCrc8(data_frame_.data(), DATA_FRAME_SIZE-1);
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(sut_->configureNumberOfSensors(mask));
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  // there must always be 8 distance values
  ASSERT_EQ(8, dist.size());
  for (size_t i = 0; i < dist.size(); i++)
  {
    if ((mask & (1 << i)) == 0)
    {
      ASSERT_TRUE(std::isnan(dist.distance[i])) << "i = " << i;
    }
    else
    {
      ASSERT_FALSE(std::isnan(dist.distance[i])) << "i = " << i;
    }
  }
}

TEST_F(TerarangerMultiflexFixture, returnMinusOneForFailedMeasurements)
{
  // last sensor reports measurement failure
  data_frame_[DATA_FRAME_SIZE-3] = 0xff;
  data_frame_[DATA_FRAME_SIZE-4] = 0xff;
  data_frame_[DATA_FRAME_SIZE-1] = calcCrc8(data_frame_.data(), DATA_FRAME_SIZE-1);
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_FLOAT_EQ(-1, dist.distance.back());
}

TEST_F(TerarangerMultiflexFixture, returnCorrectlyMeasuredDistance)
{
  float expected_distance = 1.345;
  uint16_t dist_uint = 1345;
  data_frame_[DATA_FRAME_SIZE-3] = ((uint8_t*)(&dist_uint))[0];
  data_frame_[DATA_FRAME_SIZE-4] = ((uint8_t*)(&dist_uint))[1];
  data_frame_[DATA_FRAME_SIZE-1] = calcCrc8(data_frame_.data(), DATA_FRAME_SIZE-1);
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_FLOAT_EQ(expected_distance, dist.distance.back());
}

TEST_F(TerarangerMultiflexFixture, invokeCallbackOnNewDataCapture)
{
  float expected_distance = 1.234;
  uint16_t dist_uint = 1234;
  data_frame_[DATA_FRAME_SIZE-3] = ((uint8_t*)(&dist_uint))[0];
  data_frame_[DATA_FRAME_SIZE-4] = ((uint8_t*)(&dist_uint))[1];
  data_frame_[DATA_FRAME_SIZE-1] = calcCrc8(data_frame_.data(), DATA_FRAME_SIZE-1);
  int num_callbacks = 0;
  ASSERT_TRUE(sut_->registerOnDistanceDataCaptureCallback(
    [this, expected_distance, &num_callbacks](const DistanceData& d)
    {
      logger_->debug("callback");
      EXPECT_FLOAT_EQ(expected_distance, d.distance.back());
      num_callbacks++;
    }));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->registerOnDistanceDataCaptureCallback([](const DistanceData&){}));
  ASSERT_FALSE(sut_->registerOnDistanceDataCaptureCallback(nullptr));
  /*
   * with waitForRead condition data_read_counter_.load() > 5*DATA_FRAME_SIZE;
   * callback will be called at least 5 times, we check if called at least once
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
