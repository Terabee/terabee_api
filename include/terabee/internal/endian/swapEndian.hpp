#ifndef SWAP_ENDIAN_HPP
#define SWAP_ENDIAN_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <cstdint>

namespace terabee
{
namespace internal
{
namespace endian
{

template <typename T>
T swapEndian(T u)
{
  union
  {
    T u;
    uint8_t u8[sizeof(T)];
  } source, dest;
  source.u = u;
  for (size_t k = 0; k < sizeof(T); k++)
    dest.u8[k] = source.u8[sizeof(T) - k - 1];
  return dest.u;
}

}  // namespace endian
}  // namespace internal
}  // namespace terabee

#endif  // SWAP_ENDIAN_HPP
