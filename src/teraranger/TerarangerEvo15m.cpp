/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"
#include "terabee/internal/teraranger/TerarangerEvo15m.hpp"


namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvo15m::TerarangerEvo15m(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvo15m")
{}

TerarangerEvo15m::~TerarangerEvo15m()
{}

bool TerarangerEvo15m::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvo15m::shutDown()
{
  return impl_.shutDown();
}

DistanceData TerarangerEvo15m::getDistance()
{
  return impl_.getDistance();
}

bool TerarangerEvo15m::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  return impl_.registerOnDistanceDataCaptureCallback(cb);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
