#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <cstdint>
#include <vector>

namespace terabee
{
namespace internal
{
namespace ring_buffer
{

class RingBuffer
{
public:
  RingBuffer() = delete;
  explicit RingBuffer(size_t size);
  size_t size() const;
  uint8_t& operator[](int idx);
  uint8_t operator[](int idx) const;
  void push_back(uint8_t d);
  const uint8_t* data() const;

private:
  std::vector<uint8_t> data_;
};

}  // namespace ring_buffer
}  // namespace internal
}  // namespace terabee

#endif  // RING_BUFFER_HPP
