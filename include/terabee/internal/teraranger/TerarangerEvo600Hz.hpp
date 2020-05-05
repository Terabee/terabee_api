#ifndef TERARANGER_EVO_600HZ_HPP
#define TERARANGER_EVO_600HZ_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <memory>

#include "terabee/DistanceData.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"
#include "terabee/internal/teraranger/TerarangerBasicCommon.hpp"
#include "terabee/ITerarangerEvo600Hz.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvo600Hz: public ITerarangerEvo600Hz
{
public:
  using InterfaceType = ITerarangerEvo600Hz;
  TerarangerEvo600Hz() = delete;
  TerarangerEvo600Hz(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvo600Hz(const TerarangerEvo600Hz& other) = delete;
  TerarangerEvo600Hz(TerarangerEvo600Hz&& other) = delete;
  ~TerarangerEvo600Hz() override;
  bool initialize() override;
  bool shutDown() override;
  DistanceData getDistance() override;
  bool registerOnDistanceDataCaptureCallback(OnDistanceDataCaptureCallback cb) override;

private:
  TerarangerBasicCommon impl_;
};

}  // namespace teraranger
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_EVO_600HZ_HPP
