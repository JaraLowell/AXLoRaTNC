#pragma once

#include <stdint.h>
#include <string.h>
#include <vector>

inline constexpr uint8_t U8G2_R0 = 0;
inline constexpr uint8_t u8g2_font_6x10_tf[] = {0};

struct TestOled {
  bool beginSuccess = true;
  unsigned buffersSent = 0;
  std::vector<uint8_t> powerCommands;
};

inline TestOled testOled;

class TestU8g2 {
 public:
  TestU8g2(uint8_t, uint8_t) {}
  bool begin() { return testOled.beginSuccess; }
  void setContrast(uint8_t) {}
  void clearBuffer() {}
  void setFont(const uint8_t*) {}
  void drawStr(int, int, const char*) {}
  int getStrWidth(const char* text) { return static_cast<int>(strlen(text) * 6); }
  void drawHLine(int, int, int) {}
  void sendBuffer() { ++testOled.buffersSent; }
  void setPowerSave(uint8_t enabled) { testOled.powerCommands.push_back(enabled); }
};

using U8G2_SH1106_128X64_NONAME_F_HW_I2C = TestU8g2;
using U8G2_SSD1306_128X64_NONAME_F_HW_I2C = TestU8g2;
