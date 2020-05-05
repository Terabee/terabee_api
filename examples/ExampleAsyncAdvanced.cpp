/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

/**
 * In the example TowerEvo is used, because it also gives the information about updated status
 * of the particular sensor in the array; Similar logic applies to all other Terabee sensors
 * that support callback registration;
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

#include <terabee/ITerarangerFactory.hpp>
#include <terabee/ITerarangerTowerEvo.hpp>
#include <terabee/TowerDistanceData.hpp>

using terabee::TowerDistanceData;

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

void expensiveOperation(const TowerDistanceData& d)
{
  std::cout << "expensiveOperation BEGIN" << std::endl;
  std::cout << "measurement: " << d << std::endl;
  /*
   * Here some expensive calculations might take place
   * If they happen to last longer than it takes for the sensor to read new data from the device,
   * some notifications from the callbacks might be missed, but the data reading cycle will not be disturbed
   */
  std::cout << "expensiveOperation END" << std::endl;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cout << "usage: ./ExampleAsyncAdvanced DEVICE_NAME" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  auto tower = factory->createTerarangerTowerEvo(argv[1]);
  if (!tower)
  {
    std::cout << "Failed to create device" << std::endl;
    return -1;
  }
  std::condition_variable cv;
  std::mutex m;
  std::atomic_bool data_flag(false);
  TowerDistanceData data;
  /*
   * For simplicity we do not check the return values
   */
  tower->registerOnTowerDistanceDataCaptureCallback(
    [&cv, &m, &data_flag, &data](const TowerDistanceData& d)
    {
      std::cout << "New tower data; Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count() << std::endl;
      std::lock_guard<std::mutex> l(m);
      data = d;
      data_flag = true;
      cv.notify_all();
    });
  tower->initialize();
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    TowerDistanceData copied_data;
    {  // new scope for RAII mutex unlock, alternatively you can call l.unlock();
      std::unique_lock<std::mutex> l(m);
      // 1 sec timeout to prevent infinite locks in case of sensor failure
      cv.wait_for(l, std::chrono::seconds(1), [&data_flag]() { return data_flag.load(); });
      // safe copy under lock to make sure data will not be overwritten during the long operation
      copied_data = data;
    }
    expensiveOperation(copied_data);
    // alternatively, you can set the flag before the operation, depends on the required behavior
    data_flag = false;
  }
  return tower->shutDown() ? 0 : -1;
}
