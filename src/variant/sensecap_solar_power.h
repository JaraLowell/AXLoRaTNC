#pragma once

#include <stdint.h>

namespace axlora::variant {

bool initializeBatteryProtection();
bool serviceBatteryProtection(bool radioReady);
bool readBatteryMillivolts(uint16_t& millivolts);

constexpr uint8_t batteryPercentageFromMillivolts(uint16_t millivolts) {
  return millivolts <= 3300 ? 0 :
         millivolts >= 4200 ? 100 :
         static_cast<uint8_t>((static_cast<uint32_t>(millivolts) - 3300) * 100 / 900);
}

}
