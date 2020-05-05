/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <array>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <terabee/ITerarangerFactory.hpp>
#include <terabee/ThermalData.hpp>

using terabee::ThermalData;

volatile std::sig_atomic_t g_signal_status;
void signal_handler(int signal) {
  g_signal_status = signal;
}

std::ostream& operator<< (std::ostream& os, const ThermalData& t) {
  os << "[";
  for (size_t i = 0; i < 32; i++)
  {
    for (size_t j = 0; j < 32; j++)
    {
      os << t.data[i*32 + j];
      if (j == 31) os << ";";
      else os << ", ";
    }
    if (i == 31) os << "]";
    else os << "\n";
  }
  return os;
}

template<class Sensor>
int example(std::unique_ptr<Sensor> evo_thermal)
{
  if (!evo_thermal)
  {
    std::cout << "Failed to create device" << std::endl;
    return -1;
  }
  if (!evo_thermal->initialize())
  {
    std::cout << "Failed to initialize device" << std::endl;
    return -1;
  }
  // we save the data to a file instead of printing to the console
  std::ofstream data_out("thermal_data.dat");
  size_t iteration(0);
  std::cout << "Press C-z to quit" << std::endl;
  while (g_signal_status != SIGTSTP)
  {
    auto t = evo_thermal->getThermalData();
    data_out << "Frame[" << iteration << "] = " << t << std::endl;
    cv::Mat frame(32, 32, CV_32F, t.data.data());
    cv::Mat normalized_frame;
    cv::normalize(frame, normalized_frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat color_frame;
    cv::applyColorMap(normalized_frame, color_frame, cv::COLORMAP_JET);
    cv::resize(color_frame, color_frame, cv::Size(), 10, 10);
    cv::imshow("thermal data", color_frame);
    cv::waitKey(10);
  }
  std::cout << "Internal sensor temperature = " << evo_thermal->getSensorTemperature() << std::endl;
  return evo_thermal->shutDown() ? 0 : -1;
}

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cout << "usage: ./ExampleReadEvoThermal DEV SENSOR_TYPE" << std::endl;
    std::cout << "  - DEV is the serial device name e.g. /dev/ttyACM0" << std::endl;
    std::cout << "  - SENSOR_TYPE must be either 90 or 33" << std::endl;
    return -1;
  }
  std::signal(SIGTSTP, signal_handler);
  auto factory = terabee::ITerarangerFactory::getFactory();
  if (std::string(argv[2]) == "90")
  {
    return example(factory->createTerarangerEvoThermal90(argv[1]));
  }
  else if (std::string(argv[2]) == "33")
  {
    return example(factory->createTerarangerEvoThermal33(argv[1]));
  }
  else
  {
    std::cout << "Wrong device type: " << argv[2] << std::endl;
    return -1;
  }
}
