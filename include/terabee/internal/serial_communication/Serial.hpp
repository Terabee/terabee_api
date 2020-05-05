#ifndef SERIAL_HPP
#define SERIAL_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <chrono>
#include <cstdint>
#include <string>

#include "terabee/internal/logger/Logger.hpp"
#include "terabee/internal/serial_communication/ISerial.hpp"

namespace terabee
{
namespace internal
{
namespace serial_communication
{

class Serial: public ISerial
{
public:
  Serial() = delete;
  /**
   * Constructs Serial from given port, e.g. /dev/ttyUSB0
   *
   * with default settings:
   *    - baud 9600
   *    - parity none
   *    - bytesize 8
   *    - 1 stop bit
   *    - no flow control
   *    - timeout == 0 (non-blocking)
   */
  Serial(const std::string &port);
  Serial(const Serial &other) = delete;
  Serial(Serial &&other) = delete;
  ~Serial() override;
  bool open() override;
  bool close() override;
  bool isOpen() override;
  size_t available() override;
  bool setBaudrate(uint32_t baudrate) override;
  bool setParity(parity_t iparity) override;
  bool setBytesize(bytesize_t ibytesize) override;
  bool setStopbits(stopbits_t istopbits) override;
  bool setFlowcontrol(flowcontrol_t iflowcontrol) override;
  bool setTimeout(std::chrono::duration<float> timeout) override;
  std::string readline(size_t max_buffer_size, char eol) override;
  bool flushInput() override;
  bool flushOutput() override;
  size_t write(const uint8_t *data, size_t size) override;
  size_t read(uint8_t *buffer, size_t size) override;

private:
  logger::Logger logger_;
  std::string port_;
  uint32_t baud_rate_;
  parity_t parity_;
  bytesize_t byte_size_;
  stopbits_t stop_bits_;
  flowcontrol_t flow_control_;
  std::chrono::duration<float> timeout_;
  int serial_fd_;
  bool blocking_;
};

}  // namespace serial_communication
}  // namespace internal
}  // namespace terabee

#endif // SERIAL_HPP
