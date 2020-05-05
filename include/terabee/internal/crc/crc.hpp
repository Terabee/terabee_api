#ifndef CRC_HPP
#define CRC_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <boost/crc.hpp>

namespace terabee
{
namespace internal
{
namespace crc
{

inline uint8_t calcCrc8(const uint8_t* data, int numBytes)
{
  boost::crc_optimal<8, 0x7, 0, 0> crc;
  crc.process_bytes(data, numBytes);
  return crc.checksum();
}

inline uint32_t calcCrc32(const uint8_t* data, int numBytes)
{
  boost::crc_optimal<32, 0x4C11DB7, 0xFFFFFFFF, 0> crc;
  crc.process_bytes(data, numBytes);
  return crc.checksum();
}

}  // namespace crc
}  // namespace internal
}  // namespace terabee

#endif  // CRC_HPP
