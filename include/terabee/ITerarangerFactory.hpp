#ifndef I_TERARANGER_FACTORY_HPP
#define I_TERARANGER_FACTORY_HPP

/*! \mainpage
 *
 * Application Programming Interface for Terabee sensors. The idea is to simplify sensors usage on the client side by providing abstract layer between sensor communication protocol and client algorithms.
 *
 * \section quick_start Quick Start
 *
 * To quickly connect your sensor and read the data, try one of the examples under `Examples` tab.
 *
 * \section detaled_docs Detailed interfaces documentation
 * Go to:
 *  - terabee::ITerarangerFactory to learn how to obtain a factory instance for creating sensor interfaces
 *  - One of the interfaces documentation to learn how to use a particular sensor:
 *    - terabee::ITerarangerEvo3m
 *    - terabee::ITerarangerEvo600Hz
 *    - terabee::ITerarangerEvo60m
 *    - terabee::ITerarangerEvo64px
 *    - terabee::ITerarangerEvoMini
 *    - terabee::ITerarangerEvoThermal33
 *    - terabee::ITerarangerEvoThermal90
 *    - terabee::ITerarangerMultiflex
 *    - terabee::ITerarangerTowerEvo
 */

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

/**
 * Interface to the factory that should be used to obtain interfaces to particular sensors.
 * Use ITerarangerFactory::getFactory() to get the factory instance pointer
 */
class ITerarangerFactory
{
public:
  virtual ~ITerarangerFactory() = default;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to Evo3m device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo3m> createTerarangerEvo3m(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to Evo600Hz device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo600Hz> createTerarangerEvo600Hz(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to Evo60m device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo60m> createTerarangerEvo60m(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to Evo64px device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvo64px> createTerarangerEvo64px(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to EvoMini device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvoMini> createTerarangerEvoMini(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to EvoThermal33 device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvoThermal33> createTerarangerEvoThermal33(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to EvoThermal90 device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerEvoThermal90> createTerarangerEvoThermal90(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to Multiflex device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerMultiflex> createTerarangerMultiflex(
    const std::string& serial_port) = 0;
  /**
   * \param serial_port name of the serial port device, e.g. `/dev/ttyACM0`
   * \return pointer to TowerEvo device connected to the host USB;
   * If serial port cannot be opened, or if it is impossible to communicate
   * with device, returns nullptr.
   */
  virtual std::unique_ptr<ITerarangerTowerEvo> createTerarangerTowerEvo(
    const std::string& serial_port) = 0;
  /**
   * \return pointer to the instanc of the factory; Use this function to obtain a factory
   * and create interface to any kind of sensor from Teraranger family
   */
  static std::unique_ptr<ITerarangerFactory> getFactory();
};

}  // namespace terabee

#endif  // I_TERARANGER_FACTORY_HPP
