/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"
#include "terabee/internal/teraranger/TerarangerEvo60m.hpp"


namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvo60m::TerarangerEvo60m(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvo60m")
{}

TerarangerEvo60m::~TerarangerEvo60m()
{}

bool TerarangerEvo60m::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvo60m::shutDown()
{
  return impl_.shutDown();
}

DistanceData TerarangerEvo60m::getDistance()
{
  return impl_.getDistance();
}

bool TerarangerEvo60m::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  return impl_.registerOnDistanceDataCaptureCallback(cb);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
