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

struct DistanceData
{
  std::vector<float> distance;
  size_t size() const { return distance.size(); }
};

using OnDistanceDataCaptureCallback = std::function<void(const DistanceData&)>;

}  // namespace terabee

#endif  // DISTANCE_DATA_HPP
