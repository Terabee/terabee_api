#ifndef TERARANGER_EVO_THERMAL_33_HPP
#define TERARANGER_EVO_THERMAL_33_HPP

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
#include "terabee/ITerarangerEvoThermal33.hpp"
#include "terabee/ThermalData.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvoThermal33: public ITerarangerEvoThermal33
{
public:
  using InterfaceType = ITerarangerEvoThermal33;
  TerarangerEvoThermal33() = delete;
  TerarangerEvoThermal33(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvoThermal33(const TerarangerEvoThermal33& other) = delete;
  TerarangerEvoThermal33(TerarangerEvoThermal33&& other) = delete;
  ~TerarangerEvoThermal33() override;
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

#endif  // TERARANGER_EVO_THERMAL_33_HPP
