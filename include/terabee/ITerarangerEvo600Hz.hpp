#ifndef I_TERARANGER_EVO_600HZ_HPP
#define I_TERARANGER_EVO_600HZ_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"

namespace terabee
{

class ITerarangerEvo600Hz
{
public:
  virtual ~ITerarangerEvo600Hz() = default;
  /**
   * Performs initialization of the device.
   * Returns true on success
   * Returns false on failure, e.g. device disconnected, communication failure,
   * device already initialized
   */
  virtual bool initialize() = 0;
  /**
   * Disconnects the device
   * Returns true on success
   * Returns false on failure, e.g. device busy
   */
  virtual bool shutDown() = 0;
  /**
   * Returns measured distance in [m]
   * If fails to communicate with sensor (e.g. device not initialized) returns std::nan("")
   * If measurement cannot be done (e.g. sensor malfunction) returns -1
   * If object is too close returns -std::numeric_limits<float>::infinity()
   * If measured value is out of range returns std::numeric_limits<float>::infinity();
   */
  virtual DistanceData getDistance() = 0;
  /**
   * Registers a callback to be invoked every time new data from the sensor is received;
   * The callback function must return `void` and accept one argument: const reference to DistanceData;
   * Callback registration must be done before sensor initialization;
   * Callbacks are invoked after sensor initialization when new data is constantly received;
   * Callback can be set to `nullptr`; In this case no callback will be invoked when new data is received;
   * Callback function should be short, heavy computations should be avoided, because it blocks
   * asynchronous data capture routine;
   *
   * Returns true on success, returns false on failure (e.g. sensor already initialized)
   */
  virtual bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_EVO_600HZ_HPP
