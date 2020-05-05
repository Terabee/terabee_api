/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/internal/teraranger/TerarangerEvoThermal33.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvoThermal33::TerarangerEvoThermal33(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvoThermal33")
{}

TerarangerEvoThermal33::~TerarangerEvoThermal33()
{}

bool TerarangerEvoThermal33::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvoThermal33::shutDown()
{
  return impl_.shutDown();
}

ThermalData TerarangerEvoThermal33::getThermalData()
{
  return impl_.getThermalData();
}

bool TerarangerEvoThermal33::registerOnThermalDataCaptureCallback(OnThermalDataCaptureCallback cb)
{
  return impl_.registerOnThermalDataCaptureCallback(cb);
}

float TerarangerEvoThermal33::getSensorTemperature()
{
  return impl_.getSensorTemperature();
}

bool TerarangerEvoThermal33::setEmissivity(uint8_t emissivity)
{
  return impl_.setEmissivity(emissivity);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
