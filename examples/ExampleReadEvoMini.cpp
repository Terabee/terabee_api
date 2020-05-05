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
#include <terabee/ITerarangerEvoMini.hpp>
#include <terabee/DistanceData.hpp>

using terabee::DistanceData;
using terabee::ITerarangerEvoMini;

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
    std::cout << "usage: ./ExampleReadEvoMini DEV" << std::endl;
    std::cout << "  - DEV is the serial device name e.g. /dev/ttyACM0" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto evo_mini = factory->createTerarangerEvoMini(argv[1]);
  if (!evo_mini)
  {
    std::cout << "Failed to create device" << std::endl;
  }
  evo_mini->setPixelMode(ITerarangerEvoMini::Px4Mode);
  if (!evo_mini->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << "Distance = " << evo_mini->getDistance() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return evo_mini->shutDown() ? 0 : -1;
}
