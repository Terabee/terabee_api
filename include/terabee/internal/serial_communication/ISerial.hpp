#ifndef ISERIAL_HPP
#define ISERIAL_HPP

/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <chrono>
#include <cstdint>
#include <string>

namespace terabee
{
namespace internal
{
namespace serial_communication
{

class ISerial
{
public:
  enum bytesize_t
  {
    fivebits = 5,
    sixbits = 6,
    sevenbits = 7,
    eightbits = 8
  };

  enum parity_t
  {
    parity_none = 0,
    parity_odd = 1,
    parity_even = 2,
    parity_mark = 3,
    parity_space = 4
  };

  enum stopbits_t
  {
    stopbits_one = 1,
    stopbits_two = 2,
    stopbits_one_point_five = 3
  };

  enum flowcontrol_t
  {
    flowcontrol_none = 0,
    flowcontrol_software = 1,
    flowcontrol_hardware = 2
  };

  virtual ~ISerial() {}
  virtual bool open() = 0;
  virtual bool close() = 0;
  virtual bool isOpen() = 0;
  virtual size_t available() = 0;
  virtual bool setBaudrate(uint32_t baudrate) = 0;
  virtual bool setParity(parity_t iparity) = 0;
  virtual bool setBytesize(bytesize_t ibytesize) = 0;
  virtual bool setStopbits(stopbits_t istopbits) = 0;
  virtual bool setFlowcontrol(flowcontrol_t iflowcontrol) = 0;
  virtual bool setTimeout(std::chrono::duration<float> timeout) = 0;
  virtual std::string readline(size_t max_buffer_size = 65536, char eol = '\n') = 0;
  virtual bool flushInput() = 0;
  virtual bool flushOutput() = 0;
  virtual size_t write(const uint8_t *data, size_t size) = 0;
  virtual size_t read(uint8_t *buffer, size_t size) = 0;
};

}  // namespace serial_communication
}  // namespace internal
}  // namespace terabee

#endif // ISERIAL_HPP
