#ifndef I_TERARANGER_FACTORY_HPP
#define I_TERARANGER_FACTORY_HPP

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
#include "terabee/ITerarangerMultiflex.hpp"
#include "terabee/ITerarangerTowerEvo.hpp"

namespace terabee
{

class ITerarangerFactory
{
public:
  virtual ~ITerarangerFactory() = default;
  /**
   * Returns pointer to Evo3m device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo3m> createTerarangerEvo3m(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to Evo600Hz device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo600Hz> createTerarangerEvo600Hz(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to Evo60m device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo60m> createTerarangerEvo60m(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to Evo64px device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo64px> createTerarangerEvo64px(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to EvoMini device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvoMini> createTerarangerEvoMini(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to EvoThermal33 device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvoThermal33> createTerarangerEvoThermal33(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to EvoThermal90 device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvoThermal90> createTerarangerEvoThermal90(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to Multiflex device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerMultiflex> createTerarangerMultiflex(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to TowerEvo device connected to the host USB
   * identified by `serial_port` argument
   * If serial port cannot be opened, or it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerTowerEvo> createTerarangerTowerEvo(
    const std::string& serial_port) = 0;
  /**
   * Returns pointer to factory that can create devices from Teraranger
   * family: TerarangerEvo60m
   */
  static std::unique_ptr<ITerarangerFactory> getFactory();
  /**
   * TODO: maybe also a method like this, if it is possible:
   * Returns information about all Teraranger devices connected to
   * USB ports of the host machine
   * static std::vector<DeviceInfo> listConnectedDevices() = 0;
   *
   * and then later client might do like this:
   * auto deviceList = ITerarangerFactory::listConnectedDevices();
   * auto factory = ITerarangerFactory::getFactory();
   * auto device = factory->createTerarangerEvo60m(deviceList[0].portName);
   */
};

}  // namespace terabee

#endif  // I_TERARANGER_FACTORY_HPP
