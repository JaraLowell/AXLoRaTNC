#pragma once

namespace axlora::variant {

static constexpr int PIN_RADIO_SPI_SCK = 8;
static constexpr int PIN_RADIO_SPI_MISO = 9;
static constexpr int PIN_RADIO_SPI_MOSI = 10;
static constexpr int PIN_RADIO_CS = 4;
static constexpr int PIN_RADIO_DIO1 = 1;
static constexpr int PIN_RADIO_BUSY = 3;
static constexpr int PIN_RADIO_RST = 2;
static constexpr int PIN_RADIO_DIO2 = -1;
static constexpr int PIN_RADIO_RXEN = 5;
static constexpr int PIN_RADIO_TXEN = -1;

static constexpr int PIN_LED_TX = 12;
static constexpr int PIN_LED_RX = -1;
static constexpr int PIN_BATTERY_ADC = 16;
static constexpr int PIN_BATTERY_ENABLE = 19;
static constexpr int PIN_USER_BUTTON = 13;

static constexpr int PIN_OLED_SDA = 14;
static constexpr int PIN_OLED_SCL = 15;
static constexpr int PIN_OLED_RST = -1;
static constexpr int PIN_OLED_VEXT = -1;
static constexpr int PIN_GPS_UART_RX = 7;
static constexpr int PIN_GPS_UART_TX = 6;
static constexpr int PIN_DS18B20_ONEWIRE = -1;
static constexpr int PIN_AM2302_DATA = -1;
static constexpr int PIN_UART_RTS = -1;
static constexpr int PIN_UART_CTS = -1;

}
