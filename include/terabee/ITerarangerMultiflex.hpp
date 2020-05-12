#ifndef I_TERARANGER_MULTIFLEX_HPP
#define I_TERARANGER_MULTIFLEX_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

/**
 * \example ExampleReadMultiflex.cpp
 */

#include <cstdint>

#include "terabee/DistanceData.hpp"

namespace terabee
{

/**
 * Interface to <a href="https://www.terabee.com/shop/lidar-tof-multi-directional-arrays/teraranger-multiflex/">TeraRanger Multiflex</a> sensor.
 * \see
 *  - \ref ITerarangerFactory::createTerarangerMultiflex
 *  - \ref ExampleReadMultiflex.cpp
 */
class ITerarangerMultiflex
{
public:
  virtual ~ITerarangerMultiflex() = default;
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
   * Method for obtaining sensor measurement in a synchronous way.
   * \return
   *  - returns measured distance in [m]
   *  - If measured value is out of range returns `std::numeric_limits<float>::infinity()`
   *  - If object is too close returns negative `std::numeric_limits<float>::infinity()`
   *  - If measurement cannot be done returns -1
   *  - If failed to obtain consistent data from sensor returns `std::nan("")`
   *  - If device was not successfully initialized returns `std::nan("")`
   */
  virtual DistanceData getDistance() = 0;
  /**
   * Method for obtaining sensor measurement in an asynchronous way.
   * Registers a callback to be invoked every time new data from the sensor is received
   * \param cb
   *  - The callback function must return `void` and accept one argument: const reference to DistanceData
   *  - Callback registration must be done before sensor initialization
   *  - Callbacks are invoked after sensor initialization when new data is constantly received
   *  - Callback can be set to `nullptr`; In this case no callback will be invoked when new data is received
   *  - Callback function should be short, heavy computations should be avoided, because it blocks asynchronous data capture routine
   *
   * \return
   *  - true on success
   *  - false on failure (e.g. sensor already initialized)
   */
  virtual bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) = 0;
  /**
   * Configures the number of sensors that will be queried for distance data;
   *
   * \param mask 8-bit value that defines sensor selection; to enable i-th sensor,
   * i-th bit of `mask` should be set to 1; e.g. set `mask` to 0x07
   * to enable sensors: [0, 1, 2] and disable sensors [3, 4, 5, 6, 7];
   * Measured distance value for disabled sensors is NaN
   *
   * \return
   *  - true on success
   *  - false on failure (e.g. received NACK, not initialized)
   */
  virtual bool configureNumberOfSensors(uint8_t mask) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_MULTIFLEX_HPP
