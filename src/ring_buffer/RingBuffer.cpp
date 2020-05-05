/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <algorithm>
#include <cstdint>
#include <vector>

#include "terabee/internal/ring_buffer/RingBuffer.hpp"

namespace terabee
{
namespace internal
{
namespace ring_buffer
{

RingBuffer::RingBuffer(size_t size): data_(size, 0) {}
size_t RingBuffer::size() const
{
  return data_.size();
}

uint8_t& RingBuffer::operator[](int idx)
{
  if (idx > 0) idx = idx % size();
  if (idx < 0)
  {
    int sub = (-idx) % size();
    idx = size() - sub;
  }
  return data_.at(idx);
}

uint8_t RingBuffer::operator[](int idx) const
{
  if (idx > 0) idx = idx % size();
  if (idx < 0)
  {
    int sub = (-idx) % size();
    idx = size() - sub;
  }
  return data_.at(idx);
}

void RingBuffer::push_back(uint8_t d)
{
  std::rotate(data_.begin(), data_.begin()+1, data_.end());
  data_.back() = d;
}

const uint8_t* RingBuffer::data() const
{
  return data_.data();
}

}  // namespace ring_buffer
}  // namespace internal
}  // namespace terabee
