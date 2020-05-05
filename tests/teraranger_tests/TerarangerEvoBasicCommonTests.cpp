#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/teraranger/TerarangerEvo3m.hpp"
#include "terabee/internal/teraranger/TerarangerEvo600Hz.hpp"
#include "terabee/internal/teraranger/TerarangerEvo60m.hpp"
#include "terabee/ITerarangerEvo3m.hpp"
#include "terabee/ITerarangerEvo600Hz.hpp"
#include "terabee/ITerarangerEvo60m.hpp"

#include "mocks/MockISerial.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)

using terabee::DistanceData;
using terabee::internal::crc::calcCrc8;
using terabee::internal::logger::Logger;
using terabee::internal::teraranger::TerarangerEvo3m;
using terabee::internal::teraranger::TerarangerEvo600Hz;
using terabee::internal::teraranger::TerarangerEvo60m;
using terabee::ITerarangerEvo3m;
using terabee::ITerarangerEvo600Hz;
using terabee::ITerarangerEvo60m;

using terabee::internal::serial_communication::MockISerial;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

template <typename Sensor>
class TerarangerEvoBasicCommonFixture: public testing::Test
{
protected:
  TerarangerEvoBasicCommonFixture():
    logger_("TerarangerEvoBasicCommonFixture"),
    binary_cmd_received_(false),
    fake_data_({0, 0, 0, 0}),
    read_count_(0)
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
    serial_mock_.reset(new NiceMock<MockISerial>);
    ON_CALL(*serial_mock_, open()).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, close()).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setBaudrate(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setParity(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setBytesize(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setStopbits(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setFlowcontrol(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, setTimeout(_)).WillByDefault(Return(true));
    ON_CALL(*serial_mock_, write(_, _)).WillByDefault(Invoke(
      [this](const uint8_t* cmd, size_t size)
      {
        const uint8_t bin_cmd[4] = {0x00, 0x11, 0x02, 0x4C};
        if (std::memcmp(cmd, bin_cmd, 4) == 0) binary_cmd_received_ = true;
        return size;
      }));
    configureMockRead();
    sut_.reset(new Sensor(serial_mock_));
  }
  ~TerarangerEvoBasicCommonFixture()
  {
    logger_->info("{} END with {}",
      testing::UnitTest::GetInstance()->current_test_info()->name(),
      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }
  void configureMockRead()
  {
    ON_CALL(*serial_mock_, read(_, _)).WillByDefault(Invoke(
      [this](uint8_t *buffer, size_t size) -> uint8_t
      {
        if (!binary_cmd_received_) return size;
        if (size < 4) return size;
        std::memcpy(buffer, this->fake_data_.data(), 4);
        read_count_++;
        read_cv_.notify_all();
        return 4;
      }));
  }
  bool waitForRead()
  {
    std::unique_lock<std::mutex> l(m_);
    return read_cv_.wait_for(l, CV_TIMEOUT,
      [this]()
      {
        return read_count_.load() > 1;
      });
  }
  Logger logger_;
  bool binary_cmd_received_;
  std::array<uint8_t, 4> fake_data_;
  std::atomic_int read_count_;
  std::condition_variable read_cv_;
  std::mutex m_;
  std::shared_ptr<MockISerial> serial_mock_;
  std::unique_ptr<typename Sensor::InterfaceType> sut_;
};

using SensorTypes = ::testing::Types<
  TerarangerEvo3m,
  TerarangerEvo600Hz,
  TerarangerEvo60m
>;
TYPED_TEST_SUITE(TerarangerEvoBasicCommonFixture, SensorTypes);

/*
 * In google typed tests it is mandatory to access elements through "this" pointer
 * Otherwise compilation issues occur;
 */

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnTrueOnInitializeAndShutdownReturnFalseIfCalledTwice)
{
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_FALSE(this->sut_->initialize());
  ASSERT_TRUE(this->sut_->shutDown());
  ASSERT_FALSE(this->sut_->shutDown());
  // Second call
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_FALSE(this->sut_->initialize());
  ASSERT_TRUE(this->sut_->shutDown());
  ASSERT_FALSE(this->sut_->shutDown());
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnNanWhenGetDistanceWithoutInitialize)
{
  ASSERT_TRUE(std::isnan(this->sut_->getDistance().distance.front()));
  EXPECT_CALL(*(this->serial_mock_), open()).WillRepeatedly(Return(false));
  ASSERT_FALSE(this->sut_->initialize());
  ASSERT_TRUE(std::isnan(this->sut_->getDistance().distance.front()));
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnNanIfReadsGarbage)
{
  this->fake_data_ = {1, 2, 3, 4};
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_TRUE(std::isnan(this->sut_->getDistance().distance.front()));
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnMinusOneIfSensorCannotMeasureDistance)
{
  this->fake_data_[0] = 0x54;
  this->fake_data_[1] = 0x00;
  this->fake_data_[2] = 0x01;
  this->fake_data_[3] = calcCrc8(this->fake_data_.data(), 3);
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_EQ(-1, this->sut_->getDistance().distance.front());
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnMinusInfinityIfTooClose)
{
  this->fake_data_[0] = 0x54;
  this->fake_data_[1] = 0x00;
  this->fake_data_[2] = 0x00;
  this->fake_data_[3] = calcCrc8(this->fake_data_.data(), 3);
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_EQ(-std::numeric_limits<float>::infinity(), this->sut_->getDistance().distance.front());
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnInfinityIfObjectTooFar)
{
  this->fake_data_[0] = 0x54;
  this->fake_data_[1] = 0xFF;
  this->fake_data_[2] = 0xFF;
  this->fake_data_[3] = calcCrc8(this->fake_data_.data(), 3);
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_EQ(std::numeric_limits<float>::infinity(), this->sut_->getDistance().distance.front());
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, returnCorrectDistance)
{
  float expected_distance = 5.678;
  uint16_t dist_in_mm = 5678;
  this->fake_data_[0] = 0x54;
  this->fake_data_[1] = ((uint8_t*)(&dist_in_mm))[1];
  this->fake_data_[2] = ((uint8_t*)(&dist_in_mm))[0];
  this->fake_data_[3] = calcCrc8(this->fake_data_.data(), 3);
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_FLOAT_EQ(expected_distance, this->sut_->getDistance().distance.front());
}

TYPED_TEST(TerarangerEvoBasicCommonFixture, invokeCallbackOnNewDataCapture)
{
  float expected_distance = 1.234;
  uint16_t dist_in_mm = 1234;
  this->fake_data_[0] = 0x54;
  this->fake_data_[1] = ((uint8_t*)(&dist_in_mm))[1];
  this->fake_data_[2] = ((uint8_t*)(&dist_in_mm))[0];
  this->fake_data_[3] = calcCrc8(this->fake_data_.data(), 3);
  int num_callbacks = 0;
  ASSERT_TRUE(this->sut_->registerOnDistanceDataCaptureCallback(
    [this, expected_distance, &num_callbacks](const DistanceData& d)
    {
      this->logger_->debug("callback");
      EXPECT_FLOAT_EQ(expected_distance, d.distance.front());
      num_callbacks++;
    }));
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_FALSE(this->sut_->registerOnDistanceDataCaptureCallback([](const DistanceData&){}));
  ASSERT_FALSE(this->sut_->registerOnDistanceDataCaptureCallback(nullptr));
  /*
   * Since in this test we don't use circular buff, waitForRead just counts the times
   * a whole frame was received; With waiting condition read_count_.load() > 1 callback
   * will be executed at least once;
   */
  ASSERT_TRUE(this->waitForRead());
  ASSERT_GT(num_callbacks, 0);
  ASSERT_TRUE(this->sut_->shutDown());
  ASSERT_TRUE(this->sut_->registerOnDistanceDataCaptureCallback(nullptr));
  num_callbacks = 0;
  this->read_count_ = 0;
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_EQ(0, num_callbacks);  // to make sure the callback was canceled
}
