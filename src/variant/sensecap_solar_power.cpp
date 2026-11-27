#include "sensecap_solar_power.h"

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)

#include <Arduino.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_soc.h>

#include "axlora_config.h"
#include "radio/radio.h"

namespace axlora::variant {
namespace {

static constexpr uint8_t SHUTDOWN_REASON_LOW_VOLTAGE = 0x4C;
static constexpr uint8_t SHUTDOWN_REASON_BOOT_PROTECT = 0x42;
static constexpr uint8_t LPCOMP_AIN_CHANNEL = 7;
static constexpr uint8_t LPCOMP_REFERENCE = 2;
static constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 15000;
static constexpr uint8_t LOW_BATTERY_CONFIRMATIONS = 2;
static constexpr uint16_t BATTERY_MIN_VALID_MV = 1000;
static constexpr uint16_t BATTERY_MAX_VALID_MV = 5000;

uint32_t lastBatteryCheckMs = 0;
uint8_t lowBatteryChecks = 0;
bool batterySenseInitialized = false;
bool batterySenseVerified = false;

bool softDeviceEnabled() {
  uint8_t enabled = 0;
  return sd_softdevice_is_enabled(&enabled) == NRF_SUCCESS && enabled != 0;
}

bool externalUsbPowerPresent() {
  if (softDeviceEnabled()) {
    uint32_t status = 0;
    return sd_power_usbregstatus_get(&status) == NRF_SUCCESS &&
           (status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
  }
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

uint16_t sampleBatteryMillivolts() {
  digitalWrite(PIN_BATTERY_ENABLE, LOW);
  delay(20);

  int readings[9]{};
  for (int& reading : readings) {
    reading = analogRead(PIN_BATTERY_ADC);
    delay(2);
  }

  int sum = 0;
  int minimum = readings[0];
  int maximum = readings[0];
  for (const int reading : readings) {
    sum += reading;
    if (reading < minimum) minimum = reading;
    if (reading > maximum) maximum = reading;
  }
  const int adcAverage = (sum - minimum - maximum) / 7;
  digitalWrite(PIN_BATTERY_ENABLE, HIGH);

  const float voltage = static_cast<float>(adcAverage) *
                        BATTERY_ADC_REFERENCE_V *
                        BATTERY_DIVIDER_MULTIPLIER * 1000.0f / 4096.0f;
  return voltage > 65535.0f ? 65535 : static_cast<uint16_t>(voltage);
}

bool armBatteryRecoveryWake() {
  digitalWrite(PIN_BATTERY_ENABLE, LOW);
  delay(10);
  if (digitalRead(PIN_USER_BUTTON) == HIGH) {
    nrf_gpio_cfg_sense_input(g_ADigitalPinMap[PIN_USER_BUTTON],
                             NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  }

  if (softDeviceEnabled()) {
    if (sd_power_usbdetected_enable(1) != NRF_SUCCESS) return false;
  } else {
    NRF_POWER->EVENTS_USBDETECTED = 0;
    NRF_POWER->INTENSET = POWER_INTENSET_USBDETECTED_Msk;
  }

  NRF_LPCOMP->TASKS_STOP = 1;
  NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;
  NRF_LPCOMP->PSEL =
      (static_cast<uint32_t>(LPCOMP_AIN_CHANNEL) << LPCOMP_PSEL_PSEL_Pos) &
      LPCOMP_PSEL_PSEL_Msk;
  NRF_LPCOMP->REFSEL =
      (static_cast<uint32_t>(LPCOMP_REFERENCE) << LPCOMP_REFSEL_REFSEL_Pos) &
      LPCOMP_REFSEL_REFSEL_Msk;
  NRF_LPCOMP->ANADETECT = LPCOMP_ANADETECT_ANADETECT_Up;
  NRF_LPCOMP->EVENTS_READY = 0;
  NRF_LPCOMP->EVENTS_DOWN = 0;
  NRF_LPCOMP->EVENTS_UP = 0;
  NRF_LPCOMP->EVENTS_CROSS = 0;
  NRF_LPCOMP->INTENCLR = 0xFFFFFFFF;
  NRF_LPCOMP->INTENSET = LPCOMP_INTENSET_UP_Msk;
  NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Enabled;
  NRF_LPCOMP->TASKS_START = 1;
  for (uint8_t attempt = 0; attempt < 20 && !NRF_LPCOMP->EVENTS_READY; ++attempt) {
    delayMicroseconds(50);
  }

  if (!NRF_LPCOMP->EVENTS_READY) {
    NRF_LPCOMP->TASKS_STOP = 1;
    NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;
    return false;
  }

  NRF_LPCOMP->TASKS_SAMPLE = 1;
  delayMicroseconds(50);
  const bool belowWakeThreshold =
      (NRF_LPCOMP->RESULT & LPCOMP_RESULT_RESULT_Msk) == LPCOMP_RESULT_RESULT_Below;
  if (!belowWakeThreshold) {
    NRF_LPCOMP->TASKS_STOP = 1;
    NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Disabled;
  }
  return belowWakeThreshold;
}

bool enterSystemOff(uint8_t reason) {
  if (!armBatteryRecoveryWake()) return false;

  if (softDeviceEnabled()) {
    if (sd_power_gpregret_clr(1, 0xFF) != NRF_SUCCESS ||
        sd_power_gpregret_set(1, reason) != NRF_SUCCESS) {
      return false;
    }
    if (sd_power_system_off() == NRF_SUCCESS) {
      for (;;) {
        __WFE();
      }
    }
    return false;
  }

  NRF_POWER->GPREGRET2 = reason;
  NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
  for (;;) {
    __WFE();
  }
}

bool protectAgainstLowBattery(uint8_t shutdownReason, bool radioReady) {
  if (radioReady) {
    radio::driver().sleep();
  }
  return enterSystemOff(shutdownReason);
}

void enableDcdc() {
  if (softDeviceEnabled()) {
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
  } else {
    NRF_POWER->DCDCEN = 1;
  }
}

}

bool initializeBatteryProtection() {
  enableDcdc();
  pinMode(PIN_BATTERY_ENABLE, OUTPUT);
  digitalWrite(PIN_BATTERY_ENABLE, HIGH);
  pinMode(PIN_BATTERY_ADC, INPUT);
  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0);

  pinMode(PIN_USER_BUTTON, INPUT_PULLUP);
  batterySenseInitialized = true;

  if (externalUsbPowerPresent()) {
    batterySenseVerified = true;
    return true;
  }

  const uint16_t batteryMv = sampleBatteryMillivolts();
  batterySenseVerified = batteryMv >= BATTERY_MIN_VALID_MV &&
                         batteryMv <= BATTERY_MAX_VALID_MV;
  if (!batterySenseVerified) {
    if (batteryMv < BATTERY_MIN_VALID_MV &&
        !protectAgainstLowBattery(SHUTDOWN_REASON_BOOT_PROTECT, false)) {
      return false;
    }
    return false;
  }
  if (batteryMv < BATTERY_BOOTLOCK_MV) {
    if (!protectAgainstLowBattery(SHUTDOWN_REASON_BOOT_PROTECT, false)) {
      batterySenseVerified = false;
      return false;
    }
  }
  return true;
}

bool serviceBatteryProtection(bool radioReady) {
  if (!batterySenseInitialized) {
    return false;
  }
  if (millis() - lastBatteryCheckMs < BATTERY_CHECK_INTERVAL_MS) {
    return batterySenseVerified;
  }
  lastBatteryCheckMs = millis();

  if (externalUsbPowerPresent()) {
    lowBatteryChecks = 0;
    batterySenseVerified = true;
    return true;
  }

  const uint16_t batteryMv = sampleBatteryMillivolts();
  batterySenseVerified = batteryMv >= BATTERY_MIN_VALID_MV &&
                         batteryMv <= BATTERY_MAX_VALID_MV;
  if (!batterySenseVerified) {
    lowBatteryChecks = 0;
    if (batteryMv < BATTERY_MIN_VALID_MV &&
        !protectAgainstLowBattery(SHUTDOWN_REASON_LOW_VOLTAGE, radioReady)) {
      return false;
    }
    return false;
  }

  if (batteryMv < BATTERY_BOOTLOCK_MV) {
    if (!protectAgainstLowBattery(SHUTDOWN_REASON_LOW_VOLTAGE, radioReady)) {
      batterySenseVerified = false;
      return false;
    }
  }
  if (batteryMv < BATTERY_RUNTIME_CUTOFF_MV) {
    if (++lowBatteryChecks >= LOW_BATTERY_CONFIRMATIONS) {
      if (!protectAgainstLowBattery(SHUTDOWN_REASON_LOW_VOLTAGE, radioReady)) {
        batterySenseVerified = false;
        return false;
      }
    }
    batterySenseVerified = false;
    return false;
  } else {
    lowBatteryChecks = 0;
  }
  return true;
}

bool readBatteryMillivolts(uint16_t& millivolts) {
  if (!batterySenseInitialized) return false;
  const uint16_t batteryMv = sampleBatteryMillivolts();
  if (batteryMv < BATTERY_MIN_VALID_MV || batteryMv > BATTERY_MAX_VALID_MV) return false;
  millivolts = batteryMv;
  return true;
}

}

#else

namespace axlora::variant {

bool initializeBatteryProtection() { return true; }
bool serviceBatteryProtection(bool) { return true; }
bool readBatteryMillivolts(uint16_t&) { return false; }

}

#endif
