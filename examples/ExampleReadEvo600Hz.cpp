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
#include <terabee/ITerarangerEvo600Hz.hpp>

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "usage: ./ExampleReadEvo600Hz DEVICE_NAME" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto evo600Hz = factory->createTerarangerEvo600Hz(argv[1]);
  if (!evo600Hz)
  {
    std::cout << "Failed to create device" << std::endl;
  }
  if (!evo600Hz->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << "Distance = " << evo600Hz->getDistance().distance.front() << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(1600));
  }
  return evo600Hz->shutDown() ? 0 : -1;
}
