/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/internal/teraranger/TerarangerEvoThermal90.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvoThermal90::TerarangerEvoThermal90(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvoThermal90")
{}

TerarangerEvoThermal90::~TerarangerEvoThermal90()
{}

bool TerarangerEvoThermal90::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvoThermal90::shutDown()
{
  return impl_.shutDown();
}

ThermalData TerarangerEvoThermal90::getThermalData()
{
  return impl_.getThermalData();
}

bool TerarangerEvoThermal90::registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb)
{
  return impl_.registerOnThermalDataCaptureCallback(cb);
}

float TerarangerEvoThermal90::getSensorTemperature()
{
  return impl_.getSensorTemperature();
}

bool TerarangerEvoThermal90::setEmissivity(uint8_t emissivity)
{
  return impl_.setEmissivity(emissivity);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
