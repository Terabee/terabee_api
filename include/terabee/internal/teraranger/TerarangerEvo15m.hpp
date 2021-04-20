#ifndef TERARANGER_EVO_15M_HPP
#define TERARANGER_EVO_15M_HPP

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
#include "terabee/ITerarangerEvo15m.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvo15m: public ITerarangerEvo15m
{
public:
  using InterfaceType = ITerarangerEvo15m;
  TerarangerEvo15m() = delete;
  TerarangerEvo15m(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvo15m(const TerarangerEvo15m& other) = delete;
  TerarangerEvo15m(TerarangerEvo15m&& other) = delete;
  ~TerarangerEvo15m() override;
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

#endif  // TERARANGER_EVO_15M_HPP
