#pragma once

#include "pins.h"
#include "radio_config.h"

namespace axlora::variant {

static constexpr const char* NAME = "sensecap_solar";
static constexpr bool HAS_OLED = false;
static constexpr bool HAS_GPS = true;
static constexpr bool HAS_BLE = true;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr const char* DEFAULT_CALLSIGN = "N0CALL";

static constexpr float BATTERY_ADC_REFERENCE_V = 3.0f;
static constexpr float BATTERY_DIVIDER_MULTIPLIER = 3.0f;
static constexpr uint16_t BATTERY_BOOTLOCK_MV = 3300;
static constexpr uint16_t BATTERY_RUNTIME_CUTOFF_MV = 3400;

}
