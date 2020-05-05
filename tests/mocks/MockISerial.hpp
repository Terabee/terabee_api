#include <gmock/gmock.h>

#include "terabee/internal/serial_communication/ISerial.hpp"

namespace terabee {
namespace internal {
namespace serial_communication {

class MockISerial : public ISerial {
 public:
  MOCK_METHOD0(open,
      bool());
  MOCK_METHOD0(close,
      bool());
  MOCK_METHOD0(isOpen,
      bool());
  MOCK_METHOD0(available,
      size_t());
  MOCK_METHOD1(setBaudrate,
      bool(uint32_t baudrate));
  MOCK_METHOD1(setParity,
      bool(parity_t iparity));
  MOCK_METHOD1(setBytesize,
      bool(bytesize_t ibytesize));
  MOCK_METHOD1(setStopbits,
      bool(stopbits_t istopbits));
  MOCK_METHOD1(setFlowcontrol,
      bool(flowcontrol_t iflowcontrol));
  MOCK_METHOD1(setTimeout,
      bool(std::chrono::duration<float> timeout));
  MOCK_METHOD2(readline,
      std::string(size_t, char));
  MOCK_METHOD0(flushInput,
      bool());
  MOCK_METHOD0(flushOutput,
      bool());
  MOCK_METHOD2(write,
      size_t(const uint8_t *data, size_t size));
  MOCK_METHOD2(read,
      size_t(uint8_t *buffer, size_t size));
};

}  // namespace serial_communication
}  // namespace internal
}  // namespace terabee
