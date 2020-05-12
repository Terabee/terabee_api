#ifndef TOWER_DISTANCE_DATA_HPP
#define TOWER_DISTANCE_DATA_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"

#include <functional>
#include <vector>

namespace terabee
{

/**
 * Structure to represent distance data returned by Terabee Evo Tower; It is a \ref DistanceData combined
 * with `mask` vector of boolean values that indicates if the particular value was updated
 * since the last data published by sensor.
 */
struct TowerDistanceData: DistanceData
{
  std::vector<bool> mask;
  size_t size() const { return mask.size(); }
};

/**
 * Callback function type to be used in asynchronous tower distance data acquisition;
 * \see ITerarangerTowerEvo::registerOnTowerDistanceDataCaptureCallback(OnTowerDistanceDataCaptureCallback cb)
 */
using OnTowerDistanceDataCaptureCallback = std::function<void(const TowerDistanceData&)>;

}  // namespace terabee

#endif  // TOWER_DISTANCE_DATA_HPP
