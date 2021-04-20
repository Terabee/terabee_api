#ifndef TERARANGER_EVO_40M_HPP
#define TERARANGER_EVO_40M_HPP

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
#include "terabee/ITerarangerEvo40m.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvo40m: public ITerarangerEvo40m
{
public:
  using InterfaceType = ITerarangerEvo40m;
  TerarangerEvo40m() = delete;
  TerarangerEvo40m(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvo40m(const TerarangerEvo40m& other) = delete;
  TerarangerEvo40m(TerarangerEvo40m&& other) = delete;
  ~TerarangerEvo40m() override;
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

#endif  // TERARANGER_EVO_40M_HPP
