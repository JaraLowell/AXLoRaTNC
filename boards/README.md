# AXLoRa Boards

Board JSON overrides can live here when a target needs settings that are not covered by PlatformIO's stock board definitions.

## Seeed SenseCAP Solar

Build with `pio run -e sensecap_solar`. The target uses PlatformIO's `nordicnrf52`
platform and the Adafruit nRF52 Arduino framework, with the MeshCore-pinned
framework fork and nRF52840 S140 7.3.0 SoftDevice. Its internal-flash
application limit is 811,008 bytes.

The TNC service runs in a dedicated FreeRTOS task with a 16 KiB stack. The
framework's default loop stack is too small for connected AX.25 processing.
If the service task cannot be allocated, the firmware reports an error and
disables the radio.

The board's SX1262 is wired to SPI SCK/MISO/MOSI pins 8/9/10, NSS 4, DIO1 1,
reset 2, BUSY 3, and RX-enable 5. DIO2 controls the RF switch; the module has
a 1.8 V TCXO and a 140 mA current limit. The blue LED on pin 12 indicates TX.
The default 869.4625 MHz, 62.5 kHz, SF7, CR 4/5, and sync word 0x12 match the
Station G2 configuration. This board supports 22 dBm output; that limit is
specific to this SX1262 path and must not be used for Station G2's amplified
RF output.

Battery voltage is sampled on pin 16 through the divider enabled by driving
pin 19 LOW. Boot protection enters nRF52 SYSTEMOFF below 3.3 V; runtime
protection disables radio below 3.4 V and shuts down after confirmation. The
nRF52 LPCOMP monitors AIN7 with a 3/8-VDD reference while off and wakes as the
battery rises above its threshold; USB VBUS and the user button are also armed
as wake sources. SYSTEMOFF is refused if LPCOMP is not ready or already reads
above its wake threshold. A very low ADC reading attempts protective
SYSTEMOFF; if the battery-sense path is disconnected, LPCOMP cannot track
recovery and only USB VBUS or the user button can wake the board. Other invalid
readings are marked unverified and keep the radio disabled while voltage is
retried every 15 s.

**Battery protection and recovery have not been tested on hardware.** If the
wake circuitry cannot be armed, the device stays powered with its radio
disabled rather than entering SYSTEMOFF. Do not rely on this firmware as the
sole battery protection or leave the node unattended until it has been tested.

These are Arduino D-pin indices backed by a SenseCAP-specific nRF pin map:
radio SPI D8/D9/D10 map to P1.13/P1.14/P1.15, battery ADC D16 to P0.31,
battery-sense enable D19 to P0.14, and radio-switch D5 to P0.05. The board
definition selects this custom core variant rather than the stock XIAO pin map.

The board also has a 2 MiB P25Q16H QSPI flash, but the current firmware does
not use it. Persistent settings use the Adafruit core's `InternalFileSystem`.
The linker script limits application code to `0x27000..0xED000`; InternalFS
starts at `0xED000` and spans seven 4 KiB pages through `0xF4000`. This keeps the settings region outside the application image. The firmware
formats it only when the region is entirely erased; a failed mount of
nonempty storage is reported without erasing it and disables the radio until
restart.

