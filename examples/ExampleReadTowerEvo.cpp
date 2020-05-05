/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <terabee/ITerarangerFactory.hpp>
#include <terabee/ITerarangerTowerEvo.hpp>
#include <terabee/TowerDistanceData.hpp>
#include <terabee/ImuData.hpp>

using terabee::ITerarangerTowerEvo;
using terabee::TowerDistanceData;
using terabee::ImuData;

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

std::ostream& operator<< (std::ostream& os, const TowerDistanceData& d) {
  os << "[";
  for (size_t i = 0; i < d.distance.size(); i++) {
    os << d.distance[i] << (d.mask[i] ? " <new>, " : " <old>, ");
  }
  os << "\b\b" << " ]";
  return os;
}

std::ostream& operator<< (std::ostream& os, const ImuData& d) {
  os << "[";
  for (size_t i = 0; i < d.data.size(); i++) {
    os << d.data[i] << ", ";
  }
  os << "\b\b" << " ]";
  return os;
}

int main(int argc, char** argv)
{
  if (argc != 2 && argc != 3)
  {
    std::cout << "usage: ./ExampleReadTowerEvo DEVICE_NAME [IMU_MODE]" << std::endl;
    std::cout << "  IMU_MODE :" << std::endl;
    std::cout << "       1 - Disabled" << std::endl;
    std::cout << "       2 - Quaternion" << std::endl;
    std::cout << "       3 - Euler" << std::endl;
    std::cout << "       4 - Quaternion + Linacc" << std::endl;
    return -1;
  }
  ITerarangerTowerEvo::ImuMode mode(ITerarangerTowerEvo::Disabled);
  if (argc == 3)
  {
    if (std::stoi(argv[2]) == 2) mode = ITerarangerTowerEvo::Quaternion;
    if (std::stoi(argv[2]) == 3) mode = ITerarangerTowerEvo::Euler;
    if (std::stoi(argv[2]) == 4) mode = ITerarangerTowerEvo::QuaternionLinearAcc;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto tower = factory->createTerarangerTowerEvo(argv[1]);
  if (!tower)
  {
    std::cout << "Failed to create device" << std::endl;
    return -1;
  }
  tower->setImuMode(mode);
  if (!tower->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << "Distance = " << tower->getDistance() << std::endl;
    std::cout << "IMU = " << tower->getImuData() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return tower->shutDown() ? 0 : -1;
}
