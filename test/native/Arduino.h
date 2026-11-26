#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <deque>
#include <string>
#include <vector>
#include <sstream>

#define HIGH 1
#define LOW 0
#define UART_HW_FLOWCTRL_CTS_RTS 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define strcasecmp _stricmp
#define strncasecmp _strnicmp

inline uint32_t testMillis = 0;
inline uint32_t millis() { return testMillis; }
inline void delay(uint32_t ms) { testMillis += ms; }
inline long random(long maximum) { return maximum == 0 ? 0 : maximum - 1; }

class TestSerial {
 public:
  std::vector<uint8_t> output;
  std::deque<uint8_t> input;
  void begin(uint32_t) {}
  void updateBaudRate(uint32_t) {}
  void setPins(int, int, int, int) {}
  void setHwFlowCtrlMode(int, int = 0) {}
  void flush() {}
  int available() { return static_cast<int>(input.size()); }
  int availableForWrite() { return 4096; }
  int read() {
    if (input.empty()) return -1;
    const uint8_t byte = input.front(); input.pop_front(); return byte;
  }
  size_t write(uint8_t byte) { output.push_back(byte); return 1; }
  size_t write(const uint8_t* bytes, size_t length) {
    output.insert(output.end(), bytes, bytes + length); return length;
  }
  size_t print(const char* text) {
    return write(reinterpret_cast<const uint8_t*>(text), strlen(text));
  }
  template <typename T> size_t print(const T& value) {
    std::ostringstream stream; stream << value; return print(stream.str().c_str());
  }
  size_t println() { return print("\r\n"); }
  template <typename T> size_t println(const T& value) { return print(value) + println(); }
  size_t printf(const char* format, ...) {
    char buffer[4096]{};
    va_list arguments;
    va_start(arguments, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);
    return length < 0 ? 0 : print(buffer);
  }
};

inline TestSerial Serial;
struct TestEsp {
  bool restarted = false;
  void restart() { restarted = true; }
  unsigned getFreeHeap() const { return 200000; }
};
inline TestEsp ESP;
