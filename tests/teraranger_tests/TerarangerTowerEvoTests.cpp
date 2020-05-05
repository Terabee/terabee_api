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

#include "terabee/TowerDistanceData.hpp"
#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/endian/swapEndian.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/teraranger/TerarangerTowerEvo.hpp"

#include "mocks/MockISerial.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)
#define FLOAT_TOLERANCE 0.01
#define SENSORS_MASK 0x3f  // first 6 sensors report new measurement, last 2 old measurement

using terabee::internal::crc::calcCrc8;
using terabee::internal::endian::swapEndian;
using terabee::internal::logger::Logger;
using terabee::internal::ring_buffer::RingBuffer;
using terabee::internal::teraranger::TerarangerTowerEvo;
using terabee::ITerarangerTowerEvo;
using terabee::TowerDistanceData;

using terabee::internal::serial_communication::MockISerial;

using testing::_;
using testing::Invoke;
using testing::NiceMock;
using testing::Return;

class TerarangerTowerEvoFixture: public testing::Test
{
protected:
  TerarangerTowerEvoFixture():
    logger_("TerarangerTowerEvoFixture"),
    stream_on_(false),
    stream_acked_(false),
    imu_mode_(1),
    expected_distances_({1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8}),
    expected_wxyz_({0.1, 0.2, 0.3, 0.4}),
    expected_xyz_({-100, -200, -300}),
    data_read_counter_(0),
    read_cv_(),
    m_()
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
    std::memset(current_cmd_.data(), 0, current_cmd_.size());
    std::memset(buff_, 0, sizeof(buff_));
    data_ptr_ = buff_;
    serial_mock_.reset(new NiceMock<MockISerial>);
    ON_CALL(*serial_mock_, open()).WillByDefault(Return(true));
    configureSerialReadWrite();
    sut_.reset(new TerarangerTowerEvo(serial_mock_));
  }
  ~TerarangerTowerEvoFixture()
  {
    logger_->info("{} END with {}",
      testing::UnitTest::GetInstance()->current_test_info()->name(),
      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }
  void configureSerialReadWrite()
  {
    ON_CALL(*serial_mock_, write(_, _)).WillByDefault(Invoke(
      [this](const uint8_t* cmd, size_t size)
      {
        std::memcpy(current_cmd_.data(), cmd, size);
        uint8_t streamon_cmd[5] = {0x00, 0x52, 0x02, 0x01, 0xDF};
        if (std::memcmp(current_cmd_.data(), streamon_cmd, 5) == 0)
        {
          stream_on_ = true;
          stream_acked_ = false;
          return size;
        }
        uint8_t streamoff_cmd[5] = {0x00, 0x52, 0x02, 0x00, 0xD8};
        if (std::memcmp(current_cmd_.data(), streamoff_cmd, 5) == 0)
        {
          stream_on_ = false;
          stream_acked_ = false;
          return size;
        }
        setImuMode();
        return size;
      }));
    ON_CALL(*serial_mock_, read(_, _)).WillByDefault(Invoke(
      [this](uint8_t* cmd, size_t size)
      {
        logger_->debug("Read requested with size {}", size);
        if (!stream_on_)
        {
          logger_->debug("Sending ack {}", size);
          std::memcpy(cmd, generateAck(current_cmd_.data()).data(), size);
          return size;
        }
        if (!stream_acked_)
        {
          logger_->debug("Stream ON; Sending ack {}", size);
          std::memcpy(cmd, generateAck(current_cmd_.data()).data(), size);
          stream_acked_ = true;
          return size;
        }
        return generateRespAccordingToMode(cmd, size);
      }));
  }
  void setImuMode()
  {
    uint8_t imu_cmd[2] = {0x00, 0x41};
    // compare just 2 first bytes of IMU cmd
    if (std::memcmp(current_cmd_.data(), imu_cmd, 2) == 0)
    {
      imu_mode_ = current_cmd_[2];
    }
  }

  size_t generateRespAccordingToMode(uint8_t* cmd, size_t requested_size)
  {
    buff_[0] = 'T';
    buff_[1] = 'H';
    std::transform(expected_distances_.begin(), expected_distances_.end(),
      (uint16_t*)(buff_+2), [](float n) -> uint16_t
      {
        // no need to swap endian
        if (n == std::numeric_limits<float>::infinity()) return 0xffff;
        if (n == -std::numeric_limits<float>::infinity()) return 0x0000;
        // if (std::isnan(n) // cannot be generated, because NaN means no data from sensor
        if (n == -1) return 0x0100;  // endian swapped by hand
        return swapEndian(static_cast<uint16_t>(n*1000));
      });
    buff_[18] = SENSORS_MASK;
    buff_[19] = calcCrc8(buff_, 19);
    size_t imu_idx = 20;
    buff_[imu_idx + 0] = 'I';
    buff_[imu_idx + 1] = 'M';
    buff_[imu_idx + 2] = imu_mode_ + 1;
    uint8_t* data_end = nullptr;
    if (imu_mode_ == 1) data_end = buff_ + 20;
    if (imu_mode_ == 2)  // Quaternion - additional 12 bytes
    {
      data_end = buff_ + 32;
      logger_->debug("Populating Quaternion data");
      std::transform(expected_wxyz_.begin(), expected_wxyz_.end(),
      (int16_t*)(buff_+imu_idx+3), [](float n) -> int16_t
      {
        return swapEndian(static_cast<int16_t>(n*16384));
      });
      buff_[imu_idx+11] = calcCrc8(buff_+imu_idx, 11);
    }
    if (imu_mode_ == 3)  // Euler - additional 10 bytes
    {
      data_end = buff_ + 30;
      logger_->debug("Populating Euler data");
      std::transform(expected_xyz_.begin(), expected_xyz_.end(),
      (int16_t*)(buff_+imu_idx+3), [](float n) -> int16_t
      {
        return swapEndian(static_cast<int16_t>(n*16));
      });
      buff_[imu_idx+9] = calcCrc8(buff_+imu_idx, 9);
    }
    if (imu_mode_ == 4)  // Quaternion + linacc - additional 18 bytes
    {
      data_end = buff_ + 38;
      logger_->debug("Populating Q+Llinacc data");
      std::transform(expected_wxyz_.begin(), expected_wxyz_.end(),
      (int16_t*)(buff_+imu_idx+3), [](float n) -> int16_t
      {
        return swapEndian(static_cast<int16_t>(n*16384));
      });
      std::transform(expected_xyz_.begin(), expected_xyz_.end(),
      (int16_t*)(buff_+imu_idx+11), [](float n) -> int16_t
      {
        return swapEndian(static_cast<int16_t>(n));
      });
      buff_[imu_idx+17] = calcCrc8(buff_+imu_idx, 17);
    }
    
    for (size_t i = 0; i < requested_size; i++)
    {
      if (data_ptr_ == data_end) data_ptr_ = buff_;
      cmd[i] = *data_ptr_;
      data_ptr_++;
    }
    data_read_counter_ += requested_size;
    read_cv_.notify_all();
    return requested_size;
  }
  std::array<uint8_t, 4> generateAck(const uint8_t* cmd)
  {
    std::array<uint8_t, 4> result = {0x30, 0x00, 0x00, 0x00};
    result[1] = (cmd[1] >> 4);
    result[3] = calcCrc8(result.data(), 3);
    return result;
  }
  std::array<uint8_t, 4> generateNack(const uint8_t* cmd)
  {
    std::array<uint8_t, 4> result = {0x30, 0x00, 0xff, 0x00};
    result[1] = (cmd[1] >> 4);
    result[3] = calcCrc8(result.data(), 3);
    return result;
  }
  bool waitForRead()
  {
    std::unique_lock<std::mutex> l(m_);
    return read_cv_.wait_for(l, CV_TIMEOUT,
      [this]()
      {
        return data_read_counter_.load() > 80;
      });
  }

  Logger logger_;
  bool stream_on_;
  bool stream_acked_;
  uint8_t imu_mode_;
  std::array<float, 8> expected_distances_;
  std::array<float, 4> expected_wxyz_;
  std::array<float, 3> expected_xyz_;  // used both for euler and linacc
  std::array<uint8_t, 6> current_cmd_;
  uint8_t buff_[38];
  uint8_t* data_ptr_;
  std::atomic_int data_read_counter_;
  std::condition_variable read_cv_;
  std::mutex m_;
  std::shared_ptr<MockISerial> serial_mock_;
  std::unique_ptr<ITerarangerTowerEvo> sut_;
};

TEST_F(TerarangerTowerEvoFixture, returnTrueOnInitializeAndShutdownReturnFalseIfCalledTwice)
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

TEST_F(TerarangerTowerEvoFixture, returnFalseIfNackFromDevice)
{
  EXPECT_CALL(*serial_mock_, read(_, _)).WillRepeatedly(Invoke(
    [this](uint8_t* cmd, size_t size)
    {
      std::memcpy(cmd, generateNack(cmd).data(), size);
      return size;
    }));
  ASSERT_FALSE(sut_->initialize());
}

TEST_F(TerarangerTowerEvoFixture, returnDefaultSettingsIfNotChangedBefore)
{
  ASSERT_EQ(ITerarangerTowerEvo::TowerMode, sut_->getOperatingMode());
  ASSERT_EQ(ITerarangerTowerEvo::ASAP, sut_->getUpdateRate());
  ASSERT_EQ(ITerarangerTowerEvo::Disabled, sut_->getImuMode());
}

TEST_F(TerarangerTowerEvoFixture, throwExceptionIfGetDataWithoutInitialize)
{
  ASSERT_THROW(sut_->getDistance(), std::runtime_error);
  ASSERT_THROW(sut_->getImuData(), std::runtime_error);
  EXPECT_CALL(*serial_mock_, open()).WillRepeatedly(Return(false));
  ASSERT_FALSE(sut_->initialize());
  ASSERT_THROW(sut_->getDistance(), std::runtime_error);
  ASSERT_THROW(sut_->getImuData(), std::runtime_error);
}

TEST_F(TerarangerTowerEvoFixture, returnFalseIfAttemptToChangeSettingsAfterInitialize)
{
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->setOperatingMode(ITerarangerTowerEvo::SequentialMode));
  ASSERT_FALSE(sut_->setUpdateRate(ITerarangerTowerEvo::Rate600HZ));
  ASSERT_FALSE(sut_->setImuMode(ITerarangerTowerEvo::Quaternion));
  ASSERT_FALSE(sut_->setLedThreshold(12, 15));
}

TEST_F(TerarangerTowerEvoFixture, returnCorrectSettings)
{
  ITerarangerTowerEvo::OperatingMode expected_operating_mode =
    ITerarangerTowerEvo::SequentialMode;
  ITerarangerTowerEvo::UpdateRate expected_update_rate =
    ITerarangerTowerEvo::Rate500HZ;
  ITerarangerTowerEvo::ImuMode expected_imu_mode = ITerarangerTowerEvo::Quaternion;
  ASSERT_TRUE(sut_->setOperatingMode(expected_operating_mode));
  ASSERT_TRUE(sut_->setUpdateRate(expected_update_rate));
  ASSERT_TRUE(sut_->setImuMode(expected_imu_mode));
  ASSERT_EQ(expected_operating_mode, sut_->getOperatingMode());
  ASSERT_EQ(expected_update_rate, sut_->getUpdateRate());
  ASSERT_EQ(expected_imu_mode, sut_->getImuMode());
}

// TODO: Add a way to notify specific event occurrence (timeouts, crc missmatch etc.)
TEST_F(TerarangerTowerEvoFixture, returnNanIfReadSomeGarbage)
{
  ASSERT_TRUE(sut_->initialize());
  EXPECT_CALL(*serial_mock_, read(_, _)).WillRepeatedly(Invoke([this](uint8_t*, size_t)
    {
      data_read_counter_ += 1;
      read_cv_.notify_all();
      return -1;
    }));
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  for (size_t i = 0; i < dist.size(); i++)
  {
    ASSERT_TRUE(std::isnan(dist.distance[i]));
  }
}

TEST_F(TerarangerTowerEvoFixture, returnCorrectDataInDefaultNoImuMode)
{
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_EQ(8, dist.size());  // dist data always 8 elements
  for (size_t i = 0; i < dist.size(); i++)
  {
    if (SENSORS_MASK & (1 << i)) ASSERT_TRUE(dist.mask[i]);  // new measurements
    else ASSERT_FALSE(dist.mask[i]);  // old measurements
    ASSERT_NEAR(expected_distances_[i], dist.distance[i], FLOAT_TOLERANCE);
  }
  ASSERT_TRUE(sut_->shutDown());
  // too far scenario
  expected_distances_[0] = std::numeric_limits<float>::infinity();
  // sensor not connected or measurement failure scenario
  expected_distances_[1] = -1;
  // too close scenario
  expected_distances_[2] = -std::numeric_limits<float>::infinity();
  data_read_counter_ = 0;
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  dist = sut_->getDistance();
  ASSERT_EQ(8, dist.size());
  ASSERT_FLOAT_EQ(expected_distances_[0], dist.distance[0]);
  ASSERT_FLOAT_EQ(expected_distances_[1], dist.distance[1]);
  ASSERT_FLOAT_EQ(expected_distances_[2], dist.distance[2]);
  auto imu = sut_->getImuData();
  ASSERT_EQ(0, imu.size());  // no IMU data in default mode
}

TEST_F(TerarangerTowerEvoFixture, returnCorrectQuaternionData)
{
  ASSERT_TRUE(sut_->setImuMode(ITerarangerTowerEvo::Quaternion));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_EQ(8, dist.size());
  for (size_t i = 0; i < dist.size(); i++)
  {
    ASSERT_NEAR(expected_distances_[i], dist.distance[i], FLOAT_TOLERANCE);
  }
  auto imu = sut_->getImuData();
  ASSERT_EQ(4, imu.size());  // Quaternion data should have 4 elements
  for (size_t i = 0; i < imu.size(); i++)
  {
    ASSERT_NEAR(expected_wxyz_[i], imu.data[i], FLOAT_TOLERANCE);
  }
}

TEST_F(TerarangerTowerEvoFixture, returnCorrectEulerData)
{
  ASSERT_TRUE(sut_->setImuMode(ITerarangerTowerEvo::Euler));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_EQ(8, dist.size());
  for (size_t i = 0; i < dist.size(); i++)
  {
    ASSERT_NEAR(expected_distances_[i], dist.distance[i], FLOAT_TOLERANCE);
  }
  auto imu = sut_->getImuData();
  ASSERT_EQ(3, imu.size());  // Euler data should have 3 elements
  for (size_t i = 0; i < imu.size(); i++)
  {
    ASSERT_NEAR(expected_xyz_[i], imu.data[i], FLOAT_TOLERANCE);
  }
}

TEST_F(TerarangerTowerEvoFixture, returnCorrectQuaternionLinaccData)
{
  ASSERT_TRUE(sut_->setImuMode(ITerarangerTowerEvo::QuaternionLinearAcc));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  auto dist = sut_->getDistance();
  ASSERT_EQ(8, dist.size());
  for (size_t i = 0; i < dist.size(); i++)
  {
    ASSERT_NEAR(expected_distances_[i], dist.distance[i], FLOAT_TOLERANCE);
  }
  auto imu = sut_->getImuData();
  ASSERT_EQ(7, imu.size());  // Quaternion (4 elements) + XYZ (3 elements)
  for (size_t i = 0; i < 4; i++)
  {
    ASSERT_NEAR(expected_wxyz_[i], imu.data[i], FLOAT_TOLERANCE);
  }
  for (size_t i = 0; i < 3; i++)
  {
    ASSERT_NEAR(expected_xyz_[i], imu.data[i+4], FLOAT_TOLERANCE);
  }
}

TEST_F(TerarangerTowerEvoFixture, returnFalseIfThresholdsIncorrectTrueIfCorrect)
{
  uint8_t uu_threshold(1);
  uint8_t ll_threshold(0);
  // both values below 5dm limit
  ASSERT_FALSE(sut_->setLedThreshold(uu_threshold, ll_threshold));
  uu_threshold = 50;
  // ll too small
  ASSERT_FALSE(sut_->setLedThreshold(uu_threshold, ll_threshold));
  ll_threshold = 55;
  // ll bigger than uu
  ASSERT_FALSE(sut_->setLedThreshold(uu_threshold, ll_threshold));
  uu_threshold = 90;
  // uu too high
  ASSERT_FALSE(sut_->setLedThreshold(uu_threshold, ll_threshold));
  ll_threshold = 85;
  // both too high
  ASSERT_FALSE(sut_->setLedThreshold(uu_threshold, ll_threshold));
  uu_threshold = 70;
  ll_threshold = 50;
  // OK
  ASSERT_TRUE(sut_->setLedThreshold(uu_threshold, ll_threshold));
}

TEST_F(TerarangerTowerEvoFixture, invokeCallbackOnNewDataCapture)
{
  int num_callbacks = 0;
  ASSERT_TRUE(sut_->registerOnTowerDistanceDataCaptureCallback(
    [this, &num_callbacks](const TowerDistanceData& d)
    {
      logger_->debug("callback");
      for (size_t i = 0; i < d.size(); i++)
      {
        if (SENSORS_MASK & (1 << i)) EXPECT_TRUE(d.mask[i]);  // new measurements
        else EXPECT_FALSE(d.mask[i]);  // old measurements
        EXPECT_NEAR(expected_distances_[i], d.distance[i], FLOAT_TOLERANCE);
      }
      num_callbacks++;
    }));
  ASSERT_TRUE(sut_->initialize());
  ASSERT_FALSE(sut_->registerOnTowerDistanceDataCaptureCallback([](const TowerDistanceData&){}));
  ASSERT_FALSE(sut_->registerOnTowerDistanceDataCaptureCallback(nullptr));
  /*
   * with waitForRead condition data_read_counter_.load() > 80; and default mode
   * without IMU it should be called at least 4 times, we check if at least once
   */
  ASSERT_TRUE(waitForRead());
  ASSERT_GT(num_callbacks, 1);
  ASSERT_TRUE(sut_->shutDown());
  ASSERT_TRUE(sut_->registerOnTowerDistanceDataCaptureCallback(nullptr));
  num_callbacks = 0;
  data_read_counter_ = 0;
  ASSERT_TRUE(sut_->initialize());
  ASSERT_TRUE(waitForRead());
  ASSERT_EQ(0, num_callbacks);  // to make sure the callback was canceled
}
