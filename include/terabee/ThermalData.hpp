#ifndef THERMAL_DATA_HPP
#define THERMAL_DATA_HPP

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

struct ThermalData
{
  std::vector<float> data;
  size_t size() const { return data.size(); }
};

using OnThermalDataCaptureCallback = std::function<void(const ThermalData&)>;

}  // namespace terabee

#endif  // THERMAL_DATA_HPP
