/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"
#include "terabee/internal/teraranger/TerarangerEvo3m.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvo3m::TerarangerEvo3m(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvo3m")
{}

TerarangerEvo3m::~TerarangerEvo3m()
{}

bool TerarangerEvo3m::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvo3m::shutDown()
{
  return impl_.shutDown();
}

DistanceData TerarangerEvo3m::getDistance()
{
  return impl_.getDistance();
}

bool TerarangerEvo3m::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  return impl_.registerOnDistanceDataCaptureCallback(cb);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
