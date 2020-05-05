#ifndef I_TERARANGER_TOWER_EVO_HPP
#define I_TERARANGER_TOWER_EVO_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <cstdint>

#include "terabee/TowerDistanceData.hpp"
#include "terabee/ImuData.hpp"

namespace terabee
{

class ITerarangerTowerEvo
{
public:
  enum OperatingMode
  {
    /**
     * Enables crosstalk avoidance between sensors.
     * Especially well suited for the circular
     * configuration of Tower Evo. In Tower mode
     * sensors connected to Hub Evo simultaneously
     * measure distances from 4 sensors with a 90
     * degree angle between each.
     */
    TowerMode = 1,
    /**
     * Ensures that sensors connected to Hub Evo are
     * synchronized in a sequential manner
     * (1-2-3-4-5-6-7-8), receiving a single distance
     * measurement at a time. This helps to avoid any
     * cross-talk between sensors. Sequential mode
     * provides more freedom for the physical
     * placement of sensors but can result in a
     * decrease in overall measurement repetition
     * rates.
     */
    SequentialMode = 2,
    /**
     * Supports simultaneous sensor operation,
     * enabling the highest possible sampling rates.
     * Please note that using Tower Evo with 8
     * distance sensors in simultaneous mode can
     * result in sensor crosstalk.
     */
    SimultaneousMode = 3
  };
  enum UpdateRate
  {
    ASAP = 1,
    Rate50HZ = 2,
    Rate100HZ = 3,
    Rate250HZ = 4,
    Rate500HZ = 5,
    Rate600HZ = 6
  };
  enum ImuMode
  {
    Disabled = 1,
    Quaternion = 2,
    Euler = 3,
    QuaternionLinearAcc = 4
  };
  virtual ~ITerarangerTowerEvo() = default;
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
   * Additionally TowerDistanceData::mask contains boolean values that indicate if the
   * measurement obtained from the sensor is new (true) or old (false)
   *
   * throws std::runtime_error if device not initialized
   */
  virtual TowerDistanceData getDistance() = 0;
  /**
   * Registers a callback to be invoked every time new data from the sensor is received;
   * The callback function must return `void` and accept one argument: const reference to TowerDistanceData;
   * Callback registration must be done before sensor initialization;
   * Callbacks are invoked after sensor initialization when new data is constantly received;
   * Callback can be set to `nullptr`; In this case no callback will be invoked when new data is received;
   * Callback function should be short, heavy computations should be avoided, because it blocks
   * asynchronous data capture routine;
   *
   * Returns true on success, returns false on failure (e.g. sensor already initialized)
   */
  virtual bool registerOnTowerDistanceDataCaptureCallback(OnTowerDistanceDataCaptureCallback cb) = 0;
  /**
   * Returns IMU data in the format defined by `ImuMode`
   * Returns empty data if ImuMode is Disabled
   *
   * throws std::runtime_error if device not initialized
   */
  virtual ImuData getImuData() = 0;
  /**
   * Returns current OperatingMode
   * [default] TowerMode
   */
  virtual OperatingMode getOperatingMode() const = 0;
  /**
   * Returns current UpdateRate
   * [default] ASAP
   */
  virtual UpdateRate getUpdateRate() const = 0;
  /**
   * Returns current ImuMode
   * [default] Disabled
   */
  virtual ImuMode getImuMode() const = 0;
  /**
   * Changes current OutputMode, must be done before device initialization,
   * changes are not permanent and default value is restored after device is destroyed
   * and created again;
   * returns true on success, returns false on failure (e.g. NACK from the device)
   */
  virtual bool setOperatingMode(OperatingMode mode) = 0;
  virtual bool setUpdateRate(UpdateRate rate) = 0;
  virtual bool setImuMode(ImuMode mode) = 0;
  /**
   * Changes LED upper and lower thresholds
   * Given values should be in decimeters
   * upper must be greater than lower
   * lower must be at least 5
   * upper cannot be greater than 80
   * Returns true on success
   * Returns false on failure (e.g. provided values out of range)
   */
  virtual bool setLedThreshold(uint8_t upper, uint8_t lower) = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_TOWER_EVO_HPP
