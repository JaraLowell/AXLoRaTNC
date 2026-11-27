#include <stdint.h>

#include "variant.h"
#include "WVariant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

const uint32_t g_ADigitalPinMap[] = {
    2,  3,  28, 29, 4,  5,  43, 44, 45, 46, 47,
    15, 19, 33, 9,  10, 31, 35, 37, 14, 39,
    21, 25, 20, 24, 22, 23,
};

void initVariant() {
  pinMode(PIN_QSPI_CS, OUTPUT);
  digitalWrite(PIN_QSPI_CS, HIGH);
  pinMode(LED_WHITE, OUTPUT);
  digitalWrite(LED_WHITE, LOW);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, HIGH);
}
