#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <memory>
#include <mutex>

#include "terabee/internal/crc/crc.hpp"
#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"
#include "terabee/internal/teraranger/TerarangerEvoThermal33.hpp"
#include "terabee/internal/teraranger/TerarangerEvoThermal90.hpp"
#include "terabee/ITerarangerEvoThermal33.hpp"
#include "terabee/ITerarangerEvoThermal90.hpp"
#include "terabee/ThermalData.hpp"

#include "mocks/MockISerial.hpp"

#define CV_TIMEOUT std::chrono::milliseconds(500)
#define THERMAL_FRAME_SIZE 2070
#define REP_SIZE 4

using terabee::internal::crc::calcCrc8;
using terabee::internal::logger::Logger;
using terabee::internal::ring_buffer::RingBuffer;
using terabee::internal::teraranger::TerarangerEvoThermal33;
using terabee::internal::teraranger::TerarangerEvoThermal90;
using terabee::ITerarangerEvoThermal33;
using terabee::ITerarangerEvoThermal90;
using terabee::ThermalData;

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

template<size_t N, class T>
std::ostream& operator<< (std::ostream& os, std::array<T, N> arr) {
  os << "[";
  for (size_t i = 0; i < arr.size(); i++) {
    os << arr[i] << ", ";
  }
  os << "\b\b" << " ]";
  return os;
}

template <typename Sensor>
class TerarangerEvoThermalCommonFixture: public testing::Test
{
protected:
  enum ReplyMode
  {
    ReplyCommand = 1,
    ReplyData = 2,
  };
  TerarangerEvoThermalCommonFixture():
    logger_("TerarangerEvoThermalCommonFixture"),
    test_data_dir_(__FILE__),
    current_data_idx_(0),
    ack_bytes_to_send_(0),
    read_cv_(),
    m_(),
    reply_mode_(ReplyCommand),
    data_read_counter_(0),
    data_stream_on_(false),
    thermal_frame_(THERMAL_FRAME_SIZE),
    reply_cmd_({0x14, 0, 0, 0})  // ACK
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
    reply_cmd_[3] = calcCrc8(reply_cmd_.data(), 3);
    logger_->debug("Rep: {0:x} {1:x} {2:x} {3:x}", reply_cmd_[0], reply_cmd_[1],
      reply_cmd_[2], reply_cmd_[3]);
    test_data_dir_ = test_data_dir_.substr(0,
        test_data_dir_.find("TerarangerEvoThermalCommonTests.cpp"));
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
    ON_CALL(*serial_mock_, write(_, _)).WillByDefault(Invoke(
      [](const uint8_t*, size_t size) { return size; }));
    populateData(test_data_dir_ + "evo90_thermal_data.bin");
    configureSerialReadWrite();
    sut_.reset(new Sensor(serial_mock_));
  }
  ~TerarangerEvoThermalCommonFixture()
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
        return data_read_counter_.load() > 2*THERMAL_FRAME_SIZE;
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
        if (size == 5 && data[1] == 0x52 && data[2] == 0x02)
        {
          if (data[3] == 0x00) data_stream_on_ = false;
          else if (data[3] == 0x01) data_stream_on_ = true;
        }
        reply_cmd_[1] = data[1]>>4;
        reply_cmd_[3] = calcCrc8(reply_cmd_.data(), 3);
        return size;
      }));
    ON_CALL(*serial_mock_, read(_, _)).WillByDefault(Invoke(
      [this](uint8_t* buff, size_t size)
      {
        size_t bytes_to_send = size;
        if (reply_mode_ == ReplyCommand)
        {
          logger_->debug("New command arrived, Going to send ACK/NACK");
          ack_bytes_to_send_ = REP_SIZE;
          reply_mode_ = ReplyData;
        }
        size_t i = 0;
        while (ack_bytes_to_send_ > 0 && bytes_to_send > 0)
        {
          buff[i] = reply_cmd_[REP_SIZE - ack_bytes_to_send_];
          i++;
          ack_bytes_to_send_--;
          bytes_to_send--;
        }
        if (!data_stream_on_) return size;
        while (bytes_to_send > 0)
        {
          buff[i] = thermal_frame_[current_data_idx_ ];
          current_data_idx_++;
          i++;
          bytes_to_send--;
        }
        data_read_counter_ += size;
        read_cv_.notify_all();
        return size;
      }));
  }
  void populateData(const std::string& filename)
  {
    logger_->info("Going to populate thermal frame with data: {}", filename);
    if (filesize(filename) == THERMAL_FRAME_SIZE)
    {
      std::ifstream in(filename, std::ifstream::binary);
      for (size_t i = 0; i < THERMAL_FRAME_SIZE; i++)
      {
        uint8_t d(0);
        in.read(reinterpret_cast<char*>(&d), sizeof(d));
        thermal_frame_.push_back(d);
      }
    }
    else
    {
      logger_->warn("Data file size must be {} bytes; {}", THERMAL_FRAME_SIZE, filesize(filename));
    }
  }
  Logger logger_;
  std::string test_data_dir_;
  size_t current_data_idx_;
  size_t ack_bytes_to_send_;
  std::condition_variable read_cv_;
  std::mutex m_;
  ReplyMode reply_mode_;
  std::atomic_int data_read_counter_;
  bool data_stream_on_;
  RingBuffer thermal_frame_;
  std::array<uint8_t, 4> reply_cmd_;
  std::shared_ptr<MockISerial> serial_mock_;
  std::unique_ptr<typename Sensor::InterfaceType> sut_;
};

using SensorTypes = ::testing::Types<
  TerarangerEvoThermal33,
  TerarangerEvoThermal90
>;
TYPED_TEST_SUITE(TerarangerEvoThermalCommonFixture, SensorTypes);

/*
 * In google typed tests it is mandatory to access elements through "this" pointer
 * Otherwise compilation issues occur;
 */

TYPED_TEST(TerarangerEvoThermalCommonFixture, returnTrueOnInitializeAndShutdownReturnFalseIfCalledTwice)
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

TYPED_TEST(TerarangerEvoThermalCommonFixture, throwExceptionIfGetThermalDataWithoutInitialize)
{
  ASSERT_THROW(this->sut_->getThermalData(), std::runtime_error);
  ASSERT_THROW(this->sut_->getSensorTemperature(), std::runtime_error);
  EXPECT_CALL(*(this->serial_mock_), open()).WillRepeatedly(Return(false));
  ASSERT_FALSE(this->sut_->initialize());
  ASSERT_THROW(this->sut_->getThermalData(), std::runtime_error);
  ASSERT_THROW(this->sut_->getSensorTemperature(), std::runtime_error);
}

TYPED_TEST(TerarangerEvoThermalCommonFixture, returnFalseIfChangeEmissivityWrongUsage)
{
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->sut_->setEmissivity(69));
  ASSERT_FALSE(this->sut_->setEmissivity(-1));
  ASSERT_FALSE(this->sut_->setEmissivity(101));
  ASSERT_TRUE(this->sut_->shutDown());
  ASSERT_FALSE(this->sut_->setEmissivity(69));
}

TYPED_TEST(TerarangerEvoThermalCommonFixture, returnsCorrectThermalData)
{
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  auto thermal_picture = this->sut_->getThermalData();
  float sensor_temp = this->sut_->getSensorTemperature();
  // arbitrary chosen values
  ASSERT_FLOAT_EQ(302.0, thermal_picture.data[0]);
  ASSERT_FLOAT_EQ(301.8, thermal_picture.data[1]);
  ASSERT_FLOAT_EQ(304.2, sensor_temp);
}

TYPED_TEST(TerarangerEvoThermalCommonFixture, invokeCallbackOnNewDataCapture)
{
  int num_callbacks = 0;
  ASSERT_TRUE(this->sut_->registerOnThermalDataCaptureCallback(
    [this, &num_callbacks](const ThermalData& d)
    {
      this->logger_->debug("callback");
      EXPECT_FLOAT_EQ(302.0, d.data[0]);
      EXPECT_FLOAT_EQ(301.8, d.data[1]);
      num_callbacks++;
    }));
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_FALSE(this->sut_->registerOnThermalDataCaptureCallback([](const ThermalData&){}));
  ASSERT_FALSE(this->sut_->registerOnThermalDataCaptureCallback(nullptr));
  /*
   * waitForRead waits for data_read_counter_.load() > 2*THERMAL_FRAME_SIZE;
   * so, callback is to be called at least once
   */
  ASSERT_TRUE(this->waitForRead());
  ASSERT_GT(num_callbacks, 0);
  ASSERT_TRUE(this->sut_->shutDown());
  ASSERT_TRUE(this->sut_->registerOnThermalDataCaptureCallback(nullptr));
  num_callbacks = 0;
  this->data_read_counter_ = 0;
  ASSERT_TRUE(this->sut_->initialize());
  ASSERT_TRUE(this->waitForRead());
  ASSERT_EQ(0, num_callbacks);  // to make sure the callback was canceled
}
