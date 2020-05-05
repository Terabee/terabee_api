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
#include <terabee/ITerarangerMultiflex.hpp>
#include <terabee/DistanceData.hpp>

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
    std::cout << "usage: ./ExampleReadMultiflex DEVICE_NAME" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto multiflex = factory->createTerarangerMultiflex(argv[1]);
  if (!multiflex)
  {
    std::cout << "Failed to create device" << std::endl;
    return -1;
  }
  if (!multiflex->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  // let's read from first 6 sensors
  if (!multiflex->configureNumberOfSensors(0x3f))
  {
    std::cout << "Failed to configure number of sensors" << std::endl;
    return -1;
  }
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << "Distance = " << multiflex->getDistance() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return multiflex->shutDown() ? 0 : -1;
}
