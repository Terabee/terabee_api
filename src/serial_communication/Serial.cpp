/**
 * @Author Pawel Ptasznik
 *
 * @Copyright Terabee 2020
 *
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

#include "terabee/internal/serial_communication/Serial.hpp"

namespace terabee
{
namespace internal
{
namespace serial_communication
{

Serial::Serial(const std::string &port):
  logger_("Serial(" + port + ")"),
  port_(port),
  baud_rate_(9600),
  parity_(parity_none),
  byte_size_(eightbits),
  stop_bits_(stopbits_one),
  flow_control_(flowcontrol_none),
  timeout_(0),
  serial_fd_(0),
  blocking_(false)
{
  logger_->debug("Serial::Serial()");
}

Serial::~Serial()
{
  logger_->debug("Serial::~Serial()");
  close();
}

bool Serial::open()
{
  if (serial_fd_ != 0)
  {
    logger_->warn("Already initialized");
    ::close(serial_fd_);
  }
  int fileFlags = O_RDWR | O_NOCTTY | O_NDELAY;
  if (!blocking_)
  {
    fileFlags |= O_NONBLOCK;
  }
  serial_fd_ = ::open(port_.c_str(), fileFlags);
  if (serial_fd_ == -1)
  {
    logger_->warn("Failed to open serial port: ", std::strerror(errno));
    serial_fd_ = 0;
    return false;
  }
  struct termios portSettings{};
  switch (baud_rate_)
  {
    case 9600:
      portSettings.c_cflag = B9600;
      break;
    case 19200:
      portSettings.c_cflag = B19200;
      break;
    case 38400:
      portSettings.c_cflag = B38400;
      break;
    case 57600:
      portSettings.c_cflag = B57600;
      break;
    case 115200:
      portSettings.c_cflag = B115200;
      break;
    default:
      logger_->warn("Unsupported baudrate: {}", baud_rate_);
      return false;
  }
  switch (byte_size_)
  {
    case eightbits:
      portSettings.c_cflag |= CS8;
      break;
    case sevenbits:
      portSettings.c_cflag |= CS7;
      break;
    case sixbits:
      portSettings.c_cflag |= CS6;
      break;
    case fivebits:
      portSettings.c_cflag |= CS5;
      break;
  }
  switch (parity_)
  {
    case parity_none:
      break;
    case parity_odd:
      portSettings.c_cflag |= PARENB | PARODD;
      break;
    case parity_even:
      portSettings.c_cflag |= PARENB;
      break;
    case parity_mark:
      portSettings.c_cflag |= PARENB | PARODD | CMSPAR;
      break;
    case parity_space:
      portSettings.c_cflag |= PARENB | CMSPAR;
      break;
  }
  // TODO: take a look if that 1.5 stopbits can be supported
  if (stop_bits_ == stopbits_two)
  {
    portSettings.c_cflag |= CSTOPB;
  }
  if (flow_control_ == flowcontrol_software)
  {
    portSettings.c_iflag |= IXON | IXOFF;
  }
  else if (flow_control_ == flowcontrol_hardware)
  {
    portSettings.c_cflag |= CRTSCTS;
  }
  portSettings.c_cflag |= CREAD | CLOCAL;
  portSettings.c_iflag |= IGNPAR;
  portSettings.c_oflag = 0;
  portSettings.c_lflag = 0;
  portSettings.c_cc[VMIN] = blocking_ ? 1 : 0;
  portSettings.c_cc[VTIME] = 5;
  tcsetattr(serial_fd_, TCSANOW, &portSettings);
  tcflush(serial_fd_, TCIFLUSH);
  return true;
}

bool Serial::close()
{
  if (!isOpen())
  {
    logger_->warn("Not initialized");
    return false;
  }
  ::close(serial_fd_);
  serial_fd_ = 0;
  return true;
}

bool Serial::isOpen()
{
  return (serial_fd_ == 0) ? false : true;
}

size_t Serial::available()
{
  int bytes(0);
  ioctl(serial_fd_, TIOCINQ, &bytes);
  return static_cast<size_t>(bytes);
}

bool Serial::setBaudrate(uint32_t baudrate)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings on the open port");
    return false;
  }
  baud_rate_ = baudrate;
  return true;
}

bool Serial::setParity(parity_t iparity)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings on the open port");
    return false;
  }
  parity_ = iparity;
  return true;
}

bool Serial::setBytesize(bytesize_t ibytesize)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings on the open port");
    return false;
  }
  byte_size_ = ibytesize;
  return true;
}

bool Serial::setStopbits(stopbits_t istopbits)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings on the open port");
    return false;
  }
  stop_bits_ = istopbits;
  return true;
}

bool Serial::setFlowcontrol(flowcontrol_t iflowcontrol)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings on the open port");
    return false;
  }
  flow_control_ = iflowcontrol;
  return true;
}

bool Serial::setTimeout(std::chrono::duration<float> timeout)
{
  if (isOpen())
  {
    logger_->warn("Cannot change settings on the open port");
    return false;
  }
  timeout_ = timeout;
  if (timeout_.count() > 0)
  {
    blocking_ = true;
  }
  return true;
}

std::string Serial::readline(size_t max_buffer_size, char eol)
{
  std::string result;
  int retval(0);
  char c(0);
  for (size_t i = 0; i < max_buffer_size; i++)
  {
    retval = ::read(serial_fd_, &c, 1);
    if (retval <= -1 && errno != EAGAIN)
    {
      logger_->warn("Failed to read: {}", std::strerror(errno));
      return "";
    }
    if (c == eol) break;
    result += c;
  }
  return result;
}

bool Serial::flushInput()
{
  return (tcflush(serial_fd_, TCIFLUSH) == -1) ? false : true;
}

bool Serial::flushOutput()
{
  return (tcflush(serial_fd_, TCOFLUSH) == -1) ? false : true;
}

size_t Serial::write(const uint8_t *data, size_t size)
{
  if (!isOpen())
  {
    logger_->warn("Cannot write because not open");
    return 0;
  }
  int numBytesSent = ::write(serial_fd_, data, size);
  if (numBytesSent == -1)
  {
    logger_->warn("Failed to write: {}", std::strerror(errno));
  }
  return (numBytesSent >= 0) ? numBytesSent : 0;
}

size_t Serial::read(uint8_t *buffer, size_t size)
{
  if (!isOpen())
  {
    logger_->warn("Cannot read because not open");
    return 0;
  }
  size_t numBytesRead = 0;
  if (blocking_)
  {
    auto start = std::chrono::high_resolution_clock::now();
    do
    {
      int retval = ::read(serial_fd_, buffer + numBytesRead, size - numBytesRead);
      if (retval > -1) {
        numBytesRead += retval;
      }
      if (retval <= -1 && errno != EAGAIN)
      {
        logger_->warn("Failed to read: {}", std::strerror(errno));
        return numBytesRead;
      }
      int timeToWait = 1e6*(size-numBytesRead)/(baud_rate_);
      std::this_thread::sleep_for(std::chrono::microseconds(timeToWait));
      if ((timeout_.count() != 0) && (std::chrono::high_resolution_clock::now() - start > timeout_))
      {
        logger_->warn("Read timeout");
        return numBytesRead;
      }
    } while (numBytesRead < size);
  }
  else
  {
    numBytesRead = ::read(serial_fd_, buffer, size);
  }
  return numBytesRead;
}

}  // namespace serial_communication
}  // namespace internal
}  // namespace terabee
