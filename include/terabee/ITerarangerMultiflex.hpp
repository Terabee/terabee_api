#ifndef I_TERARANGER_MULTIFLEX_HPP
#define I_TERARANGER_MULTIFLEX_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <cstdint>

#include "terabee/DistanceData.hpp"

namespace terabee
{

class ITerarangerMultiflex
{
public:
  virtual ~ITerarangerMultiflex() = default;
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
   * If measured value is out of range returns std::numeric_limits<float>::infinity();
   * If object is too close returns negative std::numeric_limits<float>::infinity();
   * If measurement cannot be done returns -1
   * If failed to obtain consistent data from sensor returns std::nan("")
   * If device was not successfully initialized returns std::nan("")
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
  /**
   * Configures the number of sensors that will be queried for distance data;
   *
   * param mask is 8-bit value that defines sensor selection; to enable i-th sensor,
   * i-th bit of `mask` should be set to 1; e.g. set `mask` to 0x07
   * to enable sensors: [0, 1, 2] and disable sensors [3, 4, 5, 6, 7]
   * Measured distance value for disabled sensors is NaN
   *
   * Returns true on success
   * Returns false on failure (e.g. received NACK, not initialized)
   */
  virtual bool configureNumberOfSensors(uint8_t mask) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_MULTIFLEX_HPP
