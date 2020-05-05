#ifndef I_TERARANGER_EVO_64_PX_HPP
#define I_TERARANGER_EVO_64_PX_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"

namespace terabee
{

class ITerarangerEvo64px
{
public:
  enum OutputMode
  {
    /**
     * The sensor provides 64 distance values and 64 ambient values
     * [default]
     */
    OutputModeDistanceAmbient = 1,
    /**
     * Sensor provides 64 distance values
     */
    OutputModeDistance = 2
  };
  enum MeasurementMode
  {
    /**
     * The sensor takes 2 subsequent frames at different light
     * modulation frequencies and builds the final image by
     * picking the best pixels of the 2 frames.
     * This provides distance readings starting from 0.1m to 5m
     * at a reduced sampling rate.
     * [default]
     */
    MeasurementModeCloseRange = 1,
    /**
     * If performance is driven by the reading speed, the sensor
     * can be set to work in this mode. In Fast mode,
     * obtain distance values from 0.5m to 5m.
     */
    MeasurementModeFast = 2
  };
  virtual ~ITerarangerEvo64px() = default;
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
   * Returns current OutputMode
   * [default] OutputModeDistanceAmbient
   */
  virtual OutputMode getOutputMode() const = 0;
  /**
   * Returns current MeasurementMode
   * [default] MeasurementModeCloseRange
   */
  virtual MeasurementMode getMeasurementMode() const = 0;
  /**
   * Changes current OutputMode, must be done before device initialization,
   * changes are not permanent and default value is restored after device is destroyed
   * and created again;
   * returns true on success, returns false on failure (e.g. NACK from the device)
   */
  virtual bool setOutputMode(OutputMode mode) = 0;
  /**
   * Changes current MeasurementMode, must be done before device initialization,
   * changes are not permanent and default value is restored after device is destroyed
   * and created again;
   * returns true on success, returns false on failure (e.g. NACK from the device)
   */
  virtual bool setMeasurementMode(MeasurementMode mode) = 0;
  /**
   * Returns measured distance in [m]
   * If fails to communicate with sensor returns std::nan("")
   * If measurement cannot be done (e.g. sensor malfunction) returns -1
   * If object is too close returns -std::numeric_limits<float>::infinity()
   * If measured value is out of range returns std::numeric_limits<float>::infinity();
   * Throws std::runtime_error if device not initialized
   */
  virtual DistanceData getDistanceValues() = 0;
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
   * Returns measured ambient light values, without unit
   * Returns std::nan("") if OutputMode is OutputModeDistance
   * Throws std::runtime_error if device not initialized
   */
  virtual DistanceData getAmbientValues() = 0;
};

}  // namespace terabee

#endif  // I_TERARANGER_EVO_64_PX_HPP
