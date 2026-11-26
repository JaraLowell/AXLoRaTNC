#include "native/Arduino.h"
#include "native/U8g2lib.h"
#include "display/display.h"
#include <cassert>
#include <iostream>

namespace axlora::util {
bool logEnabled() { return true; }
void setLogEnabled(bool) {}
}

using axlora::display::DisplayInfo;

void reset(uint32_t now) {
  testOled = TestOled{};
  testMillis = now;
  axlora::display::init();
}

void update(uint32_t now, const DisplayInfo& info) {
  testMillis = now;
  axlora::display::update(info);
}

void testIdleAndRadioWake() {
  reset(10000);
  DisplayInfo info{};
  update(69999, info);
  assert(testOled.powerCommands.empty());
  const unsigned rendered = testOled.buffersSent;
  update(70000, info);
  assert(testOled.powerCommands == std::vector<uint8_t>{1});
  assert(testOled.buffersSent == rendered);

  info.anyConnected = true;
  info.dedHostMode = true;
  info.freeHeapBytes = 100000;
  update(80000, info);
  assert(testOled.powerCommands == std::vector<uint8_t>{1});
  assert(testOled.buffersSent == rendered);

  ++info.rxCount;
  update(80001, info);
  assert((testOled.powerCommands == std::vector<uint8_t>{1, 0}));
  assert(testOled.buffersSent == rendered + 1);
  update(140000, info);
  assert(testOled.powerCommands.size() == 2);
  update(140001, info);
  assert((testOled.powerCommands == std::vector<uint8_t>{1, 0, 1}));

  ++info.txCount;
  update(140002, info);
  assert((testOled.powerCommands == std::vector<uint8_t>{1, 0, 1, 0}));
  update(200002, info);
  assert((testOled.powerCommands == std::vector<uint8_t>{1, 0, 1, 0, 1}));
}

void testActivityRefreshesTimeout() {
  reset(0);
  DisplayInfo info{};
  ++info.txCount;
  update(50000, info);
  update(109999, info);
  assert(testOled.powerCommands.empty());
  ++info.rxCount;
  update(110000, info);
  assert(testOled.powerCommands.empty());
  update(169999, info);
  assert(testOled.powerCommands.empty());
  update(170000, info);
  assert(testOled.powerCommands == std::vector<uint8_t>{1});
}

void testMillisWraparound() {
  const uint32_t start = UINT32_MAX - 10000;
  reset(start);
  DisplayInfo info{};
  update(start + 59999U, info);
  assert(testOled.powerCommands.empty());
  update(start + 60000U, info);
  assert(testOled.powerCommands == std::vector<uint8_t>{1});
  ++info.rxCount;
  update(start + 60001U, info);
  assert((testOled.powerCommands == std::vector<uint8_t>{1, 0}));
  update(start + 120001U, info);
  assert((testOled.powerCommands == std::vector<uint8_t>{1, 0, 1}));
}

void testInitializationFailure() {
  testOled = TestOled{};
  testOled.beginSuccess = false;
  Serial.output.clear();
  axlora::display::init();
  const std::string message(Serial.output.begin(), Serial.output.end());
  assert(message.find("OLED initialization failed") != std::string::npos);
  DisplayInfo info{};
  info.rxCount = 1;
  update(100000, info);
  assert(testOled.powerCommands.empty());
  assert(testOled.buffersSent == 0);
}

int main() {
  testIdleAndRadioWake();
  testActivityRefreshesTimeout();
  testMillisWraparound();
  testInitializationFailure();
  std::cout << "Display idle/wake tests passed\n";
}
