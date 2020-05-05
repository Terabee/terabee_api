#ifndef COMMON_DEFINITIONS_HPP
#define COMMON_DEFINITIONS_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <chrono>

namespace terabee
{
namespace internal
{

const auto SERIAL_READ_TIMEOUT_COMMON = std::chrono::seconds(1);
const auto SERIAL_READ_TIMEOUT_MULTISENSOR = std::chrono::seconds(2);

}  // namespace internal
}  // namespace terabee

#endif  // COMMON_DEFINITIONS_HPP
