/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include "terabee/DistanceData.hpp"
#include "terabee/internal/teraranger/TerarangerEvo600Hz.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

TerarangerEvo600Hz::TerarangerEvo600Hz(
  std::shared_ptr<serial_communication::ISerial> serial_port):
  impl_(serial_port, "TerarangerEvo600Hz")
{}

TerarangerEvo600Hz::~TerarangerEvo600Hz()
{}

bool TerarangerEvo600Hz::initialize()
{
  return impl_.initialize();
}

bool TerarangerEvo600Hz::shutDown()
{
  return impl_.shutDown();
}

DistanceData TerarangerEvo600Hz::getDistance()
{
  return impl_.getDistance();
}

bool TerarangerEvo600Hz::registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb)
{
  return impl_.registerOnDistanceDataCaptureCallback(cb);
}

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee
