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

struct TowerDistanceData: DistanceData
{
  std::vector<bool> mask;
  size_t size() const { return mask.size(); }
};

using OnTowerDistanceDataCaptureCallback = std::function<void(const TowerDistanceData&)>;

}  // namespace terabee

#endif  // TOWER_DISTANCE_DATA_HPP
