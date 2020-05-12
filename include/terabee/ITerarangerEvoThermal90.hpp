#ifndef I_TERARANGER_EVO_THERMAL_90_HPP
#define I_TERARANGER_EVO_THERMAL_90_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

/**
 * \example ExampleReadEvoThermal.cpp
 */

#include <cstdint>

#include "terabee/ThermalData.hpp"

namespace terabee
{

/**
 * Interface to <a href="https://www.terabee.com/shop/thermal-cameras/teraranger-evo-thermal-90/">TeraRanger Evo Thermal 90</a> sensor.
 * \see
 *  - \ref ITerarangerFactory::createTerarangerEvoThermal90
 *  - \ref ExampleReadEvoThermal.cpp
 */
class ITerarangerEvoThermal90
{
public:
  virtual ~ITerarangerEvoThermal90() = default;
  /**
   * Performs initialization of the device.
   * \return
   *  - true on success
   *  - false on failure, e.g. device disconnected, communication failure,
   * device already initialized
   */
  virtual bool initialize() = 0;
  /**
   * Disconnects the device
   * \return
   *  - true on success
   *  - false on failure, e.g. device busy
   */
  virtual bool shutDown() = 0;
  /**
   * Method for obtaining sensor measurement in an asynchronous way.
   * \return measured 32x32 = 1024 temperature pixels in [K] or `std::nan` if for some reason
   * data acquisition was not possible
   * \throw std::runtime_error if device not initialized
   */
  virtual ThermalData getThermalData() = 0;
  /**
   * Method for obtaining sensor measurement in an asynchronous way.
   * Registers a callback to be invoked every time new data from the sensor is received
   * \param cb
   *  - The callback function must return `void` and accept one argument: const reference to ThermalData
   *  - Callback registration must be done before sensor initialization
   *  - Callbacks are invoked after sensor initialization when new data is constantly received
   *  - Callback can be set to `nullptr`; In this case no callback will be invoked when new data is received
   *  - Callback function should be short, heavy computations should be avoided, because it blocks asynchronous data capture routine
   *
   * \return
   *  - true on success
   *  - false on failure (e.g. sensor already initialized)
   */
  virtual bool registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb) = 0;
  /**
   * \return internal sensor temperature in [K] or `std::nan` if for some reason
   * data acquisition was not possible
   * \throw std::runtime_error if device not initialized
   */
  virtual float getSensorTemperature() = 0;
  /**
   * Sets device custom emissivity level
   * \param emissivity Valid values are in range <1, 100>
   * \returns
   *  - true on success
   *  - false on failure (e.g. value out of range, device not initialized)
   */
  virtual bool setEmissivity(uint8_t emissivity) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_EVO_THERMAL_90_HPP
