#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <random>

#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/ring_buffer/RingBuffer.hpp"

using terabee::internal::logger::Logger;
using terabee::internal::ring_buffer::RingBuffer;

class RingBufferFixture: public testing::Test
{
protected:
  RingBufferFixture():
    logger_("RingBufferFixture"),
    rd_(),
    gen_(rd_())
  {
    logger_->info("{} BEGIN",
      testing::UnitTest::GetInstance()->current_test_info()->name());
  }
  ~RingBufferFixture()
  {
    logger_->info("{} END with {}",
      testing::UnitTest::GetInstance()->current_test_info()->name(),
      testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
  }

  Logger logger_;
  std::random_device rd_;
  std::mt19937 gen_;
};

TEST_F(RingBufferFixture, returnCorrectBufferSize)
{
  size_t buffer_size(10);
  RingBuffer sut(buffer_size);
  ASSERT_EQ(buffer_size, sut.size());
  buffer_size = 100;
  sut = RingBuffer(buffer_size);
  ASSERT_EQ(buffer_size, sut.size());
  buffer_size = 123456;
  sut = RingBuffer(buffer_size);
  ASSERT_EQ(buffer_size, sut.size());
}

TEST_F(RingBufferFixture, returnElementsInACyclicWayIfIndexBiggerThansize)
{
  size_t buffer_size(123);
  RingBuffer sut(buffer_size);
  std::uniform_int_distribution<> dis(0, 255);
  for (size_t i = 0; i < sut.size(); i++)
  {
    sut.push_back(dis(gen_));
  }
  ASSERT_EQ(sut[0], sut[sut.size()]);
  int idx = 69;
  for (size_t n = 1; n < 10; n++)
  {
    ASSERT_EQ(sut[idx], sut[idx + n*sut.size()]);
  }
  sut[-idx];
  sut[(int)sut.size() - idx];
  ASSERT_EQ(sut[-idx], sut[(int)sut.size() - idx]);
  const RingBuffer copy = sut;
  ASSERT_EQ(copy[0], copy[copy.size()]);
  for (size_t n = 1; n < 10; n++)
  {
    ASSERT_EQ(copy[idx], copy[idx + n*copy.size()]);
  }
  copy[-idx];
  copy[(int)copy.size() - idx];
  ASSERT_EQ(copy[-idx], copy[(int)copy.size() - idx]);
}

TEST_F(RingBufferFixture, shiftElementsWhenNewElementIntroduced)
{
  std::vector<uint8_t> v = {0, 1, 2, 3, 4};
  RingBuffer sut(4);
  for (size_t i = 0; i < sut.size(); i++)
  {
    sut.push_back(v[i]);
  }
  // buffer is [0, 1, 2, 3]
  for (size_t i = 0; i < sut.size(); i++)
  {
    ASSERT_EQ(v[i], sut[i]);
  }
  ASSERT_EQ(0, std::memcmp(v.data(), sut.data(), sut.size()));
  sut.push_back(v[4]);
  // buffer is [1, 2, 3, 4]
  for (size_t i = 0; i < sut.size(); i++)
  {
    ASSERT_EQ(v[i+1], sut[i]);
  }
  ASSERT_EQ(0, std::memcmp(v.data()+1, sut.data(), sut.size()));
  sut.push_back(v[4]);
  // buffer is [2, 3, 4, 4]
  for (size_t i = 0; i < sut.size()-1; i++)
  {
    ASSERT_EQ(v[i+2], sut[i]);
  }
  ASSERT_EQ(0, std::memcmp(v.data()+2, sut.data(), sut.size()-1));
  ASSERT_EQ(v[4], sut[-1]);
}
