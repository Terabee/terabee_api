#ifndef TERARANGER_EVO_3M_HPP
#define TERARANGER_EVO_3M_HPP

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
#include "terabee/ITerarangerEvo3m.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvo3m: public ITerarangerEvo3m
{
public:
  using InterfaceType = ITerarangerEvo3m;
  TerarangerEvo3m() = delete;
  TerarangerEvo3m(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvo3m(const TerarangerEvo3m& other) = delete;
  TerarangerEvo3m(TerarangerEvo3m&& other) = delete;
  ~TerarangerEvo3m() override;
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

#endif  // TERARANGER_EVO_3M_HPP
