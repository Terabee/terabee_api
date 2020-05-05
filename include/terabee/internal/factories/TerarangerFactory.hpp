#ifndef TERARANGER_FACTORY_HPP
#define TERARANGER_FACTORY_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <memory>
#include <string>

#include "terabee/ITerarangerEvo3m.hpp"
#include "terabee/ITerarangerEvo600Hz.hpp"
#include "terabee/ITerarangerEvo60m.hpp"
#include "terabee/ITerarangerEvo64px.hpp"
#include "terabee/ITerarangerEvoMini.hpp"
#include "terabee/ITerarangerEvoThermal33.hpp"
#include "terabee/ITerarangerEvoThermal90.hpp"
#include "terabee/ITerarangerFactory.hpp"
#include "terabee/ITerarangerMultiflex.hpp"
#include "terabee/ITerarangerTowerEvo.hpp"

namespace terabee
{
namespace internal
{
namespace factories
{
class TerarangerFactory: public ITerarangerFactory
{
public:
  ~TerarangerFactory() override = default;
  std::unique_ptr<ITerarangerEvo3m> createTerarangerEvo3m(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerEvo600Hz> createTerarangerEvo600Hz(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerEvo60m> createTerarangerEvo60m(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerEvo64px> createTerarangerEvo64px(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerEvoMini> createTerarangerEvoMini(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerEvoThermal33> createTerarangerEvoThermal33(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerEvoThermal90> createTerarangerEvoThermal90(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerMultiflex> createTerarangerMultiflex(
    const std::string& serial_port) override;
  std::unique_ptr<ITerarangerTowerEvo> createTerarangerTowerEvo(
    const std::string& serial_port) override;
};

}  // namespace factories
}  // namespace internal
}  // namespace terabee

#endif  // TERARANGER_FACTORY_HPP
