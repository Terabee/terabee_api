#ifndef I_TERARANGER_EVO_MINI_HPP
#define I_TERARANGER_EVO_MINI_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

/**
 * \example ExampleReadEvoMini.cpp
 */

#include "terabee/DistanceData.hpp"

namespace terabee
{

/**
 * Interface to <a href="https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-mini/">TeraRanger Evo Mini</a> sensor.
 * \see
 *  - \ref ITerarangerFactory::createTerarangerEvoMini
 *  - \ref ExampleReadEvoMini.cpp
 */
class ITerarangerEvoMini
{
public:
  enum PixelMode
  {
    /**
     * The sensor provides one distance value
     * [default]
     */
    Px1Mode = 1,
    /**
     * The sensor provides 2 distance values
     */
    Px2Mode = 2,
    /**
     * The sensor provides 4 distance values
     */
    Px4Mode = 3
  };
  enum RangeMode
  {
    /**
     * The sensor measures distance in a short range (0.03 m - 1.35 m) with increased update rate and accuracy
     */
    ShortRangeMode = 1,
    /**
     * The sensor measures distance in a long range (0.03 m - 3.3 m)
     * [default]
     */
    LongRangeMode = 2
  };
  virtual ~ITerarangerEvoMini() = default;
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
   *
   * \throw std::runtime_error if device not initialized
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
   * \return
   *  - true on success
   *  - false on failure (e.g. device already initialized)
   */
  virtual bool setPixelMode(PixelMode mode) = 0;
  /**
   * Changes current RangeMode, must be done before device initialization,
   * changes are not permanent and default value is restored after device is destroyed
   * and created again;
   * \return
   *  - true on success
   *  - false on failure (e.g. device already initialized)
   */
  virtual bool setRangeMode(RangeMode mode) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_EVO_MINI_HPP
