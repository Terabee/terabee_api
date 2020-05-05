#ifndef TERARANGER_EVO_60M_HPP
#define TERARANGER_EVO_60M_HPP

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
#include "terabee/ITerarangerEvo60m.hpp"

namespace terabee
{
namespace internal
{
namespace teraranger
{

class TerarangerEvo60m: public ITerarangerEvo60m
{
public:
  using InterfaceType = ITerarangerEvo60m;
  TerarangerEvo60m() = delete;
  TerarangerEvo60m(std::shared_ptr<serial_communication::ISerial> serial_port);
  TerarangerEvo60m(const TerarangerEvo60m& other) = delete;
  TerarangerEvo60m(TerarangerEvo60m&& other) = delete;
  ~TerarangerEvo60m() override;
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

#endif  // TERARANGER_EVO_60M_HPP
