/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <array>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <terabee/ITerarangerFactory.hpp>
#include <terabee/ITerarangerEvo64px.hpp>
#include <terabee/DistanceData.hpp>

using terabee::ITerarangerEvo64px;
using terabee::DistanceData;

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

std::ostream& operator<< (std::ostream& os, const DistanceData& d) {
  os << "[";
  for (size_t i = 0; i < d.distance.size(); i++) {
    os << d.distance[i] << ", ";
  }
  os << "\b\b" << " ]";
  return os;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "usage: ./ExampleReadEvo64PX DEVICE_NAME" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto evo64PX = factory->createTerarangerEvo64px(argv[1]);
  if (!evo64PX)
  {
    std::cout << "Failed to create device" << std::endl;
    return -1;
  }
  if (!evo64PX->setOutputMode(ITerarangerEvo64px::OutputModeDistanceAmbient)
    || !evo64PX->setMeasurementMode(ITerarangerEvo64px::MeasurementModeFast))
  {
    std::cout << "Failed to change default settings of evo64PX device" << std::endl;
    return -1;
  }
  if (!evo64PX->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << "Distance = " << evo64PX->getDistanceValues() << std::endl;
    std::cout << "Ambient = " << evo64PX->getAmbientValues() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return evo64PX->shutDown() ? 0 : -1;
}
