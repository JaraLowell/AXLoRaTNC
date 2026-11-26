#pragma once

namespace axlora::variant {

// Pinout: MeshCore variants/station_g2/platformio.ini.
static constexpr int PIN_SPI_SCK = 12;
static constexpr int PIN_SPI_MISO = 14;
static constexpr int PIN_SPI_MOSI = 13;
static constexpr int PIN_RADIO_CS = 11;
static constexpr int PIN_RADIO_DIO1 = 48;
static constexpr int PIN_RADIO_BUSY = 47;
static constexpr int PIN_RADIO_RST = 21;
static constexpr int PIN_RADIO_DIO2 = -1;
static constexpr int PIN_RADIO_RXEN = -1;
static constexpr int PIN_RADIO_TXEN = -1;
static constexpr int PIN_LED_TX = -1;
static constexpr int PIN_LED_RX = -1;
static constexpr int PIN_BATTERY_ADC = -1;
static constexpr int PIN_OLED_SDA = 5;
static constexpr int PIN_OLED_SCL = 6;
static constexpr int PIN_OLED_RST = -1;
static constexpr int PIN_OLED_VEXT = -1;
static constexpr int PIN_GPS_RX = 15;
static constexpr int PIN_GPS_TX = 7;
static constexpr int PIN_DS18B20_ONEWIRE = -1;
static constexpr int PIN_AM2302_DATA = -1;
static constexpr int PIN_UART_RTS = -1;
static constexpr int PIN_UART_CTS = -1;

}
