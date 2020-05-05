/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

/**
 * In the example Evo64 is used, but the same logic applies to all other Terabee sensors
 * that support callback registration;
 */

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <terabee/ITerarangerFactory.hpp>
#include <terabee/ITerarangerEvo64px.hpp>
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
    std::cout << "usage: ./ExampleAsyncSimple DEVICE_NAME" << std::endl;
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
  /*
   * For simplicity we do not check the return values
   */
  evo64PX->registerOnDistanceDataCaptureCallback([](const DistanceData& d)
    {
      std::cout << "New distance data; Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count() << std::endl;
      std::cout << d << std::endl;
    });
  evo64PX->initialize();
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return evo64PX->shutDown() ? 0 : -1;
}
