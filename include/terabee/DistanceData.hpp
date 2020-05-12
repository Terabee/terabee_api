#ifndef DISTANCE_DATA_HPP
#define DISTANCE_DATA_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <functional>
#include <vector>

namespace terabee
{

/**
 * Structure to represent a data returned from Terabee distance sensors.
 * Size of the field `distance` is dependent on the sensor type.
 */
struct DistanceData
{
  std::vector<float> distance;
  size_t size() const { return distance.size(); }
};

/**
 * Callback function type to be used in asynchronous distance data acquisition;
 * \see ITerarangerEvo3m::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
 */
using OnDistanceDataCaptureCallback = std::function<void(const DistanceData&)>;

}  // namespace terabee

#endif  // DISTANCE_DATA_HPP
