#ifndef TERARANGER_EVO_THERMAL_90_HPP
#define TERARANGER_EVO_THERMAL_90_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <cstdint>
#include <memory>

#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerThermalCommon.hpp"
#include "terabee/ITerarangerEvoThermal90.hpp"
#include "terabee/ThermalData.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvoThermal90: public ITerarangerEvoThermal90
{
public:
  using InterfaceType = ITerarangerEvoThermal90;
  TerarangerEvoThermal90() = delete;
  TerarangerEvoThermal90(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvoThermal90(const TerarangerEvoThermal90& other) = delete;
  TerarangerEvoThermal90(TerarangerEvoThermal90&& other) = delete;
  ~TerarangerEvoThermal90() override;
  bool initialize() override;
  bool shutDown() override;
  ThermalData getThermalData() override;
  bool registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb) override;
  float getSensorTemperature() override;
  bool setEmissivity(uint8_t emissivity) override;

private:
  TerarangerThermalCommon impl_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_EVO_THERMAL_90_HPP
