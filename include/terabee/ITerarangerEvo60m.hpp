#ifndef I_TERARANGER_EVO_60M_HPP
#define I_TERARANGER_EVO_60M_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

/**
 * \example ExampleReadEvo60m.cpp
 */

#include "terabee/DistanceData.hpp"

namespace terabee
{

/**
 * Interface to <a href="https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-60m/">TeraRanger Evo 60m</a> sensor.
 * \see
 *  - \ref ITerarangerFactory::createTerarangerEvo60m
 *  - \ref ExampleReadEvo60m.cpp
 */
class ITerarangerEvo60m
{
public:
  virtual ~ITerarangerEvo60m() = default;
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
   *  - If fails to communicate with sensor (e.g. device not initialized) returns `std::nan("")`
   *  - If measurement cannot be done (e.g. sensor malfunction) returns -1
   *  - If object is too close returns `-std::numeric_limits<float>::infinity()`
   *  - If measured value is out of range returns `std::numeric_limits<float>::infinity()`
   */
  virtual DistanceData getDistance() = 0;
  /**
   * Method for obtaining sensor measurement in an asynchronous way.
   * Registers a callback to be invoked every time new data from the sensor is received;
   * \param cb
   *  - The callback function must return `void` and accept one argument: const reference to DistanceData
   *  - Callback registration must be done before sensor initialization
   *  - Callbacks are invoked after sensor initialization when new data is constantly received
   *  - Callback can be set to `nullptr`; In this case no callback will be invoked when new data is received
   *  - Callback function should be short, heavy computations should be avoided, because it blocks asynchronous data capture routine;
   *
   * \return
   *  - true on success
   *  - false on failure (e.g. sensor already initialized)
   */
  virtual bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_EVO_60M_HPP
