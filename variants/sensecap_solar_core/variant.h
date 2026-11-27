#ifndef SENSECAP_SOLAR_CORE_VARIANT_H
#define SENSECAP_SOLAR_CORE_VARIANT_H

#define VARIANT_MCK             (64000000ul)
#define USE_LFXO

#include "WVariant.h"

#define PINS_COUNT              (27)
#define NUM_DIGITAL_PINS        (27)
#define NUM_ANALOG_INPUTS       (8)
#define NUM_ANALOG_OUTPUTS      (0)

#define PIN_LED                 (12)
#define LED_BUILTIN             (PIN_LED)
#define LED_WHITE               (11)
#define LED_BLUE                (12)
#define LED_STATE_ON            (1)

#define PIN_BUTTON1             (13)
#define PIN_BUTTON2             (20)
#define PIN_USER_BTN            PIN_BUTTON1

#define VBAT_ENABLE             (19)
#define BATTERY_PIN             (16)
#define AREF_VOLTAGE            (3.0F)
#define ADC_MULTIPLIER          (3.0F)
#define ADC_RESOLUTION          (12)
#define PWRMGT_VOLTAGE_BOOTLOCK (3300)
#define PWRMGT_LPCOMP_AIN       (7)
#define PWRMGT_LPCOMP_REFSEL    (2)

#define PIN_SERIAL1_RX          (7)
#define PIN_SERIAL1_TX          (6)

#define SPI_INTERFACES_COUNT    (1)
#define PIN_SPI_MISO            (9)
#define PIN_SPI_MOSI            (10)
#define PIN_SPI_SCK             (8)

#define WIRE_INTERFACES_COUNT   (1)
#define PIN_WIRE_SDA            (14)
#define PIN_WIRE_SCL            (15)

#define GPS_BAUDRATE            (9600)
#define PIN_GPS_TX              (PIN_SERIAL1_RX)
#define PIN_GPS_RX              (PIN_SERIAL1_TX)
#define GPS_EN                  (18)

#define PIN_QSPI_SCK            (21)
#define PIN_QSPI_CS             (22)
#define PIN_QSPI_IO0            (23)
#define PIN_QSPI_IO1            (24)
#define PIN_QSPI_IO2            (25)
#define PIN_QSPI_IO3            (26)

#endif
