# APRS

APRS frames are AX.25 UI frames with PID `0xF0`. AXLoRaTNC decodes incoming APRS frames and logs the parsed content (position, message, status, weather) to the console.

## Position beacon

```text
beacon aprs 52.0167 4.7000 /> LoRa TNC on 869.480 MHz
```

Arguments: `lat lon [symbol-table+code] [comment]`. Default symbol is `/>` (car). This sets the destination to `APRS` and formats an uncompressed APRS position info field automatically.

Then enable the beacon:

```text
beacon interval 600    # every 10 minutes
beacon on
```

## Beacon commands

```text
beacon                         # show config
beacon on / off
beacon now                     # transmit immediately
beacon text <text>             # set info field manually
beacon dest <CALLSIGN-SSID>    # set destination (default CQ)
beacon interval <seconds>      # 10–86400 s
beacon path <CALL1,CALL2|off>  # set repeater path
```

## Digipeater

```text
digi on / off
digi mode ui        # relay only UI frames (default, for APRS/beacons)
digi mode all       # relay UI and connected-mode AX.25 frames
digialias WIDE1-1   # set secondary alias
digialias off
```

The digipeater matches the first unrepeated address against the node callsign or configured alias, sets the H-bit, recalculates FCS, and retransmits. A 30-second duplicate cache suppresses UI-frame loops.

### SenseCAP Solar relay

The experimental `sensecap_solar` target enables digipeating of UI and
connected-mode frames by default. Set the station's callsign and manager over
USB:

```text
callsign N0CALL-1
manager Your name or callsign
```

Peers must use the relay's callsign in their AX.25 VIA path and use compatible
AXLoRaTNC radio settings and framing; MeshCore packets are different. A
direct connection receives a manager welcome and stays connected. Send `//MH`
or `//MHEARD` to see the in-memory heard-station table, `//I` for battery voltage
and estimated percentage, successful radio TX/RX packet counts, and uptime
since boot, or `//Q` to disconnect. These
commands are case-insensitive. Battery percentage is a voltage-based estimate
using 3.3 V as empty and 4.2 V as full, not a fuel gauge; charging can affect
the reading. An invalid battery reading is reported as unavailable. The relay does not
persist a connection log; the heard table is cleared on reboot. Leave the USB
interface in console mode for standalone relaying; KISS mode gives the host
control of received frames.

The hardware and battery shutdown/recovery behavior require on-device testing
before unattended use. Select a legal frequency, power and duty cycle for your
location.

## Mheard

```text
mheard          # list stations heard: uptime timestamp, RSSI, SNR, frame count, via flag
mheard clear    # reset the table
```

The table stores up to 20 entries with first-heard and last-heard times, RSSI, SNR, frame count, and whether the station was heard via a digipeater.
