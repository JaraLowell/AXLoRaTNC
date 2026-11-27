#include <Arduino.h>
#include "axlora_config.h"
#include "display/display.h"
#include "radio/radio.h"
#include "tnc/tnc.h"
#include "util/log.h"
#include "util/timer.h"
#include "variant/sensecap_solar_power.h"

void serviceLoop();

namespace {

axlora::tnc::Tnc tnc;
bool radioReady = false;
bool batteryProtectionVerified = true;
uint32_t lastRadioInitMs = 0;
uint32_t lastDisplayMs   = 0;
static constexpr uint32_t RADIO_INIT_RETRY_MS  = 5000;
static constexpr uint32_t DISPLAY_UPDATE_MS    = 1000;

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
static constexpr uint32_t SERVICE_STACK_WORDS = 4096;

void serviceTask(void*) {
  for (;;) serviceLoop();
}
#endif

void serviceRadioInit(bool force) {
  const uint32_t now = axlora::util::nowMs();
  if (radioReady || (!force && !axlora::util::elapsed(now, lastRadioInitMs, RADIO_INIT_RETRY_MS))) {
    return;
  }

  lastRadioInitMs = now;
  const axlora::radio::Result radioResult = axlora::radio::driver().init();
  radioReady = radioResult == axlora::radio::Result::Ok;
  LOG_RADIO("init result=%s", axlora::radio::resultName(radioResult));
}

}

void setup() {
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  batteryProtectionVerified = axlora::variant::initializeBatteryProtection();
#endif
#if defined(ARDUINO_ARCH_ESP32)
  const size_t serialRxBufferSize = Serial.setRxBufferSize(512);
#endif
  Serial.begin(axlora::variant::SERIAL_BAUD);
#if defined(ARDUINO_ARCH_ESP32)
  if (serialRxBufferSize != 512) {
    LOG_ERR("serial RX buffer allocation failed: %u", static_cast<unsigned>(serialRxBufferSize));
  }
#endif
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  if (!batteryProtectionVerified) {
    LOG_ERR("battery voltage unverified; radio disabled");
  }
#endif
  if (axlora::variant::PIN_LED_TX >= 0) {
    pinMode(axlora::variant::PIN_LED_TX, OUTPUT);
  }
  if (axlora::variant::PIN_LED_RX >= 0) {
    pinMode(axlora::variant::PIN_LED_RX, OUTPUT);
  }
  axlora::display::init();
  tnc.begin(axlora::variant::DEFAULT_CALLSIGN);
  LOG_INFO("boot AXLoRaTNC");

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  if (!tnc.settingsReady()) {
    batteryProtectionVerified = false;
    LOG_ERR("settings storage unavailable; radio disabled until restart");
  }
#endif
  if (batteryProtectionVerified) {
    serviceRadioInit(true);
  }
  if (!tnc.quietSerialMode()) {
    Serial.println("AXLoRaTNC commands: help, info, mode, mode console|kiss|ded, radio, ax25, connect <CALLSIGN-SSID>, disconnect, sendui <DEST> <message>, send <message>, stats");
    tnc.printInfo();
  }
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  if (xTaskCreate(serviceTask, "axlora", SERVICE_STACK_WORDS, nullptr,
                  TASK_PRIO_LOW, nullptr) != pdPASS) {
    LOG_ERR("service task allocation failed; radio disabled");
    if (radioReady) {
      axlora::radio::driver().sleep();
      radioReady = false;
    }
  }
#endif
}

void serviceLoop() {
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  const bool wasBatteryVerified = batteryProtectionVerified;
  batteryProtectionVerified = axlora::variant::serviceBatteryProtection(radioReady) &&
                              tnc.settingsReady();
  if (!batteryProtectionVerified && radioReady) {
    axlora::radio::driver().sleep();
    radioReady = false;
  }
  if (wasBatteryVerified && !batteryProtectionVerified) {
    LOG_ERR("battery voltage unverified or low; radio disabled");
  } else if (!wasBatteryVerified && batteryProtectionVerified) {
    LOG_INFO("battery voltage verified; radio may resume");
  }
#endif
  if (batteryProtectionVerified) {
    serviceRadioInit(false);
  }
  tnc.loop(radioReady && batteryProtectionVerified);

  if constexpr (axlora::variant::HAS_OLED) {
    const uint32_t now = axlora::util::nowMs();
    if (axlora::util::elapsed(now, lastDisplayMs, DISPLAY_UPDATE_MS)) {
      lastDisplayMs = now;
      axlora::display::DisplayInfo info{};
      tnc.fillDisplayInfo(info);
      info.radioReady = radioReady;
      axlora::display::update(info);
    }
  }

  yield();
}

void loop() {
#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
  delay(1000);
#else
  serviceLoop();
#endif
}
