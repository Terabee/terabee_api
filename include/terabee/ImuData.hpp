#ifndef IMU_DATA_HPP
#define IMU_DATA_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <vector>

namespace terabee
{

/**
 * Structure to represent a Intertial Measurement Unit data returned from Terabee sensors.
 * Size of the field `data` is dependent on the sensor type.
 */
struct ImuData
{
  std::vector<float> data;
  size_t size() const { return data.size(); }
};

}  // namespace terabee

#endif  // IMU_DATA_HPP
