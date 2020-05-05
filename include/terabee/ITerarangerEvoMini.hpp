#ifndef I_TERARANGER_EVO_MINI_HPP
#define I_TERARANGER_EVO_MINI_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"

namespace terabee
{

class ITerarangerEvoMini
{
public:
  enum PixelMode
  {
    Px1Mode = 1,
    Px2Mode = 2,
    Px4Mode = 3
  };
  enum RangeMode
  {
    ShortRangeMode = 1,
    LongRangeMode = 2
  };
  virtual ~ITerarangerEvoMini() = default;
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
   * Returns measured distances in [m]
   * If fails to communicate with sensor (e.g. sensor not connected) returns std::nan("")
   * If measurement cannot be done (e.g. sensor malfunction) returns -1
   * If object is too close returns -std::numeric_limits<float>::infinity()
   * If measured value is out of range returns std::numeric_limits<float>::infinity();
   *
   * DistanceData::updated fields indicate if the corresponding measurement has been updated since last
   * call to getDistance; They are initially set to "false";
   * DistanceData::updated fields are set to true whenever new measurement is received from the sensor
   *
   * throws std::runtime_error if device not initialized
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
   * Returns current PixelMode
   * [default] Px1Mode
   */
  virtual PixelMode getPixelMode() const = 0;
  /**
   * Returns current RangeMode
   * [default] LongRangeMode
   */
  virtual RangeMode getRangeMode() const = 0;
  /**
   * Changes current PixelMode, must be done before device initialization,
   * changes are not permanent and default value is restored after device is destroyed
   * and created again;
   * Returns true on success, false on failure (e.g. device already initialized)
   */
  virtual bool setPixelMode(PixelMode mode) = 0;
  /**
   * Changes current RangeMode, must be done before device initialization,
   * changes are not permanent and default value is restored after device is destroyed
   * and created again;
   * Returns true on success, false on failure (e.g. device already initialized)
   */
  virtual bool setRangeMode(RangeMode mode) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_EVO_MINI_HPP
