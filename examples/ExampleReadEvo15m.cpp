/**
 * @Author Marcin Pilch
 *
 * @Copyright Terabee 2021
 *
 */

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <terabee/ITerarangerFactory.hpp>
#include <terabee/ITerarangerEvo15m.hpp>

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "usage: ./ExampleReadEvo15 DEVICE_NAME" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto evo15 = factory->createTerarangerEvo15m(argv[1]);
  if (!evo15)
  {
    std::cout << "Failed to create device" << std::endl;
  }
  if (!evo15->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    std::cout << "Distance = " << evo15->getDistance().distance.front() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return evo15->shutDown() ? 0 : -1;
}
