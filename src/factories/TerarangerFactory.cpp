/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <memory>
#include <string>

#include "terabee/internal/factories/TerarangerFactory.hpp"
#include "terabee/internal/serial_communication/Serial.hpp"
#include "terabee/internal/teraranger/TerarangerEvo3m.hpp"
#include "terabee/internal/teraranger/TerarangerEvo600Hz.hpp"
#include "terabee/internal/teraranger/TerarangerEvo15m.hpp"
#include "terabee/internal/teraranger/TerarangerEvo40m.hpp"
#include "terabee/internal/teraranger/TerarangerEvo60m.hpp"
#include "terabee/internal/teraranger/TerarangerEvo64px.hpp"
#include "terabee/internal/teraranger/TerarangerEvoMini.hpp"
#include "terabee/internal/teraranger/TerarangerEvoThermal33.hpp"
#include "terabee/internal/teraranger/TerarangerEvoThermal90.hpp"
#include "terabee/internal/teraranger/TerarangerMultiflex.hpp"
#include "terabee/internal/teraranger/TerarangerTowerEvo.hpp"

namespace terabee
{
namespace internal
{
namespace factories
{

std::unique_ptr<ITerarangerEvo3m> TerarangerFactory::createTerarangerEvo3m(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvo3m>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvo600Hz> TerarangerFactory::createTerarangerEvo600Hz(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvo600Hz>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvo15m> TerarangerFactory::createTerarangerEvo15m(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvo15m>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvo40m> TerarangerFactory::createTerarangerEvo40m(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvo40m>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvo60m> TerarangerFactory::createTerarangerEvo60m(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvo60m>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvo64px> TerarangerFactory::createTerarangerEvo64px(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvo64px>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvoMini> TerarangerFactory::createTerarangerEvoMini(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvoMini>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvoThermal33> TerarangerFactory::createTerarangerEvoThermal33(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvoThermal33>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerEvoThermal90> TerarangerFactory::createTerarangerEvoThermal90(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerEvoThermal90>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerMultiflex> TerarangerFactory::createTerarangerMultiflex(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerMultiflex>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

std::unique_ptr<ITerarangerTowerEvo> TerarangerFactory::createTerarangerTowerEvo(
  const std::string& serial_port)
{
  return std::make_unique<teraranger::TerarangerTowerEvo>(
    std::make_shared<serial_communication::Serial>(serial_port));
}

}  // namespace factories
}  // namespace internal
}  // namespace terabee
