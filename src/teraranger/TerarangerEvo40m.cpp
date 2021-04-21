/**
 * @Author Marcin Pilch
 *
 * @Copyright Terabee 2021
 *
 */

#include "terabee/DistanceData.hpp"
#include "terabee/internal/teraranger/TerarangerEvo40m.hpp"


namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvo40m::TerarangerEvo40m(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvo40m")
{}

TerarangerEvo40m::~TerarangerEvo40m()
{}

bool TerarangerEvo40m::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvo40m::shutDown()
{
  return impl_.shutDown();
}

DistanceData TerarangerEvo40m::getDistance()
{
  return impl_.getDistance();
}

bool TerarangerEvo40m::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  return impl_.registerOnDistanceDataCaptureCallback(cb);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
