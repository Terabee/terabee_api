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

/**
 * Structure to represent a thermal data returned from Terabee thermal cameras.
 * Size of the field `data` is dependent on the sensor type.
 */
struct ThermalData
{
  std::vector<float> data;
  size_t size() const { return data.size(); }
};

/**
 * Callback function type to be used in asynchronous thermal data acquisition;
 * \see ITerarangerEvoThermal33::registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb)
 */
using OnThermalDataCaptureCallback = std::function<void(const ThermalData&)>;

}  // namespace terabee

#endif  // THERMAL_DATA_HPP
