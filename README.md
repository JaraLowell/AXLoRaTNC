# AXLoRaTNC

AXLoRaTNC is a real AX.25 packet-radio TNC for ESP32 + LoRa. AX.25 Level 2 frames stay AX.25 frames; LoRa only replaces the classic AFSK/FSK modem layer.

**[📖 Full documentation](https://michtronics.github.io/AXLoRaTNC/)  ·  [⚡ Web installer](https://michtronics.github.io/AXLoRaTNC/flash/)  ·  [📦 Releases](https://github.com/MichTronics/AXLoRaTNC/releases)**

## Flash (easiest)

Use the browser-based installer — no drivers or tools needed. Works in Chrome and Edge.

👉 **[https://michtronics.github.io/AXLoRaTNC/flash/](https://michtronics.github.io/AXLoRaTNC/flash/)**

## Target hardware

| Variant | MCU | Radio | Build env | Status |
|---|---|---|---|---|
| ESP32 DevKit V1 + EBYTE E22 | ESP32 | SX1262 | `devkitv1_e22` | ✅ tested |
| Heltec WiFi LoRa 32 V3 | ESP32-S3 | SX1262 | `heltec_v3` | ⚠ not tested |
| TTGO T-Beam | ESP32 | SX1276 | `tbeam` | ⚠ not tested |
| LilyGo T3 LoRa32 V1.6.1 | ESP32 | SX1276 | `lilygo_t3_v161` | ✅ tested |
| LILYGO T-Beam SUPREME 433MHz | ESP32-S3 | SX1262 | `tbeam_supreme_433` | ⚠ not tested |
| BQ Station G2 | ESP32-S3 | SX1262 + PA | `station_g2` | ✅ tested |
| Seeed SenseCAP Solar | nRF52840 | SX1262 | `sensecap_solar` | ⚠ experimental; hardware not tested |

On OLED-equipped devices, the screen sleeps after 60 seconds without successful
LoRa packet transmission or reception and wakes on the next radio activity.
This is checked at the existing one-second display refresh interval. Serial/TCP
polling alone does not keep it awake; radio and host communication continue
normally while the screen is off.

### Station G2

Build with `pio run -e station_g2` and flash with `pio run -e station_g2 --target upload`.
The GitHub firmware workflow also builds Station G2, publishes its web-flasher manifest, and includes a Station G2 ZIP on published releases.
The pinout, TCXO, RF switch and amplifier drive limit follow [MeshCore's Station G2 configuration](https://github.com/meshcore-dev/MeshCore/blob/main/variants/station_g2/platformio.ini).
Connect the native USB port, configure the radio and callsign in the console, then enter `mode kiss` before connecting a KISS host. The firmware starts in console mode again after reset.

The `radio power` setting is **SX1262 drive power**, not antenna output. The default 7 dBm drive produces approximately 27 dBm at 915 MHz or 28 dBm at 868 MHz; the amplifier can output several watts. Drive is limited to 19 dBm. These are typical figures, not guaranteed limits: consult the [manufacturer's measurements and operating guidance](https://wiki.bqvoy.com/en/meshtastic/station-g2), use a suitable antenna or rated dummy load, and select frequency, power and duty cycle permitted in your region before transmitting.

Station G2 defaults: 869.4625 MHz, 62.5 kHz bandwidth, SF7 and coding rate 4/5. Previously saved radio settings take precedence; use `radio reset` to restore all variant radio defaults.

Station G2 also supports **Wi-Fi KISS TCP on port 8100**, either joining a
2.4 GHz network or creating a protected access point. Wi-Fi is off by default.
Configure `wifi ssid <name>` and `wifi password <value>` over USB, then select
`wifi sta` or `wifi ap`; `wifi status` shows the IP and free memory.
Use `wifi off` before changing credentials. Settings survive reset.
See the [Wi-Fi KISS setup guide](website/guide/kiss.md#station-g2-wi-fi-kiss-tcp)
for GPNG setup and USB/TCP ownership. Keep port 8100 on a trusted network:
KISS TCP has no authentication and permits RF transmission.

This is AXLoRaTNC's AX.25-over-LoRa KISS modem, not MeshCore's KISS modem or MeshCore packet protocol. Radio peers must use compatible AXLoRaTNC framing and radio settings.

### SenseCAP Solar

The experimental `sensecap_solar` build is a standalone AX.25 digipeater with
a configurable manager welcome and an in-memory heard list. See the
[relay setup](website/guide/aprs.md#sensecap-solar-relay) and
[board notes](boards/README.md#seeed-sensecap-solar). Radio operation and
battery shutdown/recovery have not been verified on hardware; do not rely on
the battery protection for unattended use until tested.

## Build from source

**Prerequisites:** Python 3, [PlatformIO](https://platformio.org/)

```sh
git clone https://github.com/MichTronics/AXLoRaTNC.git
cd AXLoRaTNC
python3 -m venv venv
./venv/bin/pip install platformio
```

Build and flash:

```sh
./venv/bin/pio run -e devkitv1_e22                        # build only
./venv/bin/pio run -e devkitv1_e22 --target upload        # build + flash
./venv/bin/pio device monitor -b 9600                     # serial monitor
```

## LoRa defaults

The `devkitv1_e22` defaults are stored in `variants/devkitv1_e22/`. They can be overridden at runtime (see [Radio config](#radio-config)) and persist across reboots.

| Parameter | Default |
|---|---|
| Frequency | 869.4625 MHz |
| Bandwidth | 62.5 kHz |
| Spreading factor | SF7 |
| Coding rate | 4/5 |
| Sync word | 0x12 |
| TX power | 12 dBm |
| AX.25 FCS | inside LoRa payload |
| RadioLib packet CRC | enabled |

---

## Serial console

Open the serial monitor at **115 200 baud**. All settings are stored in ESP32 NVS and survive reboots.

### Callsign

```text
callsign              → show current callsign
callsign N0CALL-0      → set and persist callsign
```

### Radio config

```text
radio                         → show freq / bw / sf / cr / power + live RSSI/SNR
radio freq 869.480            → set frequency (MHz)
radio bw   125                → set bandwidth (kHz)
radio sf   7                  → set spreading factor (6–12)
radio cr   5                  → set coding rate denominator (5–8, meaning 4/5…4/8)
radio power 22                → set TX power (dBm)
radio reset                   → restore variant defaults
```

Changes are applied immediately and written to NVS.

### KISS parameters

KISS TxDelay, persistence, SlotTime, and FullDuplex are set by the host over the KISS protocol. In half-duplex mode these drive the p-persistent CSMA algorithm inside the non-blocking radio TX queue.

For fast bench testing:

```text
profile fast      → txdelay=0, p=255, slot=1, fulldup=0, duty guard off
profile normal    → txdelay=30, p=63, slot=10, fulldup=0, duty guard on
```

Use `profile fast` only on a dummy load or shielded lab setup.

### AX.25 connected mode

```text
connect N0CALL-1          → connect on channel 1
connect 2 N0CALL-2        → connect on channel 2 (1–8; Station G2: 1–10)
disconnect                → disconnect channel 1
disconnect 2              → disconnect channel 2
send hello                → send on channel 1
send 2 hello              → send on channel 2
sendui CQ hello world     → send UI frame
ax25                      → show status of all active channels
stats                     → full statistics
```

### Beacon

```text
beacon                          → show config
beacon on / off
beacon now                      → transmit immediately
beacon text <text>              → set info field
beacon dest <CALLSIGN-SSID>     → set destination (default CQ)
beacon interval <seconds>       → 10–86400 s
beacon path <CALL1,CALL2|off>   → set repeater path
```

**APRS position beacon** — auto-formats an uncompressed APRS position info field and sets the destination to `APRS`:

```text
beacon aprs 52.0167 4.7000 /> LoRa TNC
```

Arguments: `lat lon [symbol-table+code] [comment]`. Default symbol is `/>` (car). The beacon must also be enabled with `beacon on` unless it was already on.

### Mheard

```text
mheard          → list stations heard with uptime timestamps, RSSI, SNR
mheard clear    → reset the table
```

The table stores first-heard and last-heard times formatted as `HH:MM:SS` uptime since boot, plus RSSI, SNR, frame count, and whether the station was heard via a digipeater.

### Digipeater

```text
digi on / off
digi mode ui            → relay only UI frames (default)
digi mode all           → relay UI and connected-mode AX.25 frames
digialias WIDE1-1         → set secondary alias (callsign or alias matched)
digialias off
```

The digipeater matches the first unrepeated repeater address against the node callsign or configured alias, sets the H-bit, recalculates FCS, and retransmits. The default `ui` mode relays only AX.25 UI frames for APRS/beacons. `all` mode also relays connected-mode AX.25 frames, allowing connections through a LoRa digi path. A 30-second duplicate cache suppresses UI-frame loops; connected-mode retransmissions are intentionally not cached because repeated I-frames can be part of normal AX.25 recovery.

### NET/ROM node

```text
node                          → show config
node on / off
node alias AXLORA             → set NET/ROM alias (up to 6 chars)
node ident <text>             → set node identification string
node interval <seconds>       → NODES broadcast interval (60–86400 s)
node broadcast                → send NODES broadcast now
nodes                         → show learned route table
routes                        → same as nodes
```

Routes expire automatically after `interval × 6` seconds (NET/ROM obsolescence rule).

When a station connects over AX.25 connected mode the node shell answers:

```text
?         → command list
INFO      → node identification
NODES     → known NET/ROM routes
ROUTES    → same
MHEARD    → heard station table
BBS       → enter mailbox shell
BYE / B   → disconnect
```

### Mailbox (BBS)

From the connected node shell, type `BBS` to enter the mailbox. Available commands:

```text
L         → list all messages (number, timestamp, from, to, status)
LT        → list messages addressed to you
R <n>     → read message n
S <CALL>  → compose a message to CALL (end with an empty line)
K <n>     → delete message n (only from/to own callsign)
X / EXIT  → back to node shell
B / BYE   → disconnect
```

Messages are stored in NVS with sender, recipient, body, read flag, and uptime timestamp. Up to 12 messages, 200 characters each.

Sysop listing from the local console:

```text
bbs        → list all messages regardless of recipient
```

### Serial mode

```text
mode              → show current mode
mode console      → switch to console (default)
mode kiss         → switch to pure KISS
mode ded          → switch to WA8DED hostmode
```

Serial mode changes are not persisted. A KISS host's first FEND (`0xC0`) or `KISS ON` enters KISS automatically; the KISS Return frame `C0 FF C0` goes back to the console.
WA8DED clients can enter hostmode directly from the console with `ESC JHOST1 CR`.
On UART connections, the host must initially use the console baud rate (115200).

### Duty Cycle

The duty-cycle guard is enabled by default and stored in NVS.

```text
duty          → show current duty guard state
duty on       → enable guard
duty off      → disable guard for dummy-load/lab testing
duty 10       → set 10% duty cycle
duty 1        → set 1% duty cycle
duty 0.1      → set 0.1% duty cycle
profile fast  → fastest lab profile; disables duty guard
```

Use `duty off` only on a dummy load or shielded lab setup. On 869.480 MHz in EU/NL the normal guard should stay enabled.

---

## KISS

USB serial accepts standard KISS framing (`0xC0` FEND). KISS data frames are treated as complete AX.25 frames from the host; FCS is appended before LoRa transmit. Received AX.25 frames are FCS-checked and emitted as KISS data frames (without FCS), exactly as a normal KISS TNC.

KISS parameter frames (TxDelay, Persistence, SlotTime, FullDuplex) are accepted and drive the CSMA algorithm.

Compatible with: F6FBB/LinFBB, BPQ/LinBPQ, `kissattach`, Dire Wolf, APRS clients.

For a working LinFBB/F6FBB setup, use KISS through Linux AX.25 and see [docs/FBB-KISS.md](docs/FBB-KISS.md). For LinBPQ/BPQ32, see [docs/BPQ.md](docs/BPQ.md). A complete BPQ example config is in [examples/bpq32-axloratnc.cfg](examples/bpq32-axloratnc.cfg).

---

## WA8DED hostmode

Switch with `mode ded`. The binary host frame format is:

```
host → tnc:  [channel] [cmd] [count] [data…]
tnc  → host: [channel] [code] [data…]
```

**8 connected channels** (1–8), or **10 on Station G2** (1–10). Channel 0 = UI/unproto.

Connecting to a configured local callsign uses internal channel-to-channel
loopback through the first free matching channel, without RF transmission.
See [local loopback](website/guide/wa8ded.md#local-loopback) for examples.

### Commands

| Command | Function |
|---|---|
| `G` / `G0` / `G1` | Poll for all / data only / status only; channel 255 `G` lists pending channels |
| `C <CALL>` | Connect on channel n |
| `D` | Disconnect channel n |
| `I [CALL]` | Query / set own callsign |
| `L` | Numeric status/buffer counters: 2 on channel 0, 6 on connected channels |
| `M [string]` | Query / set monitor mode (`I`/`U`/`S` letters, `N` = off) |
| `V` | Firmware version string |
| `S [n]` | Terminal-only channel selection; invalid in binary hostmode |
| `U [n]` | Unattended mode (0 = off, 1/2 = on with auto-text) |
| `JHOST0` | Return to terminal mode |
| `JHOST1` | Enter binary hostmode (acknowledged) |
| `@B` | Shared free storage in 32-byte buffer equivalents |
| `@D [0/1]` | Full duplex; `@DIAG` is the diagnostics extension |
| `@I [0..256]` | I-poll retry threshold, not PACLEN |
| `QRES` | Reboot without erasing saved settings |

### Parameters (query / set)

| Cmd | Parameter | Default |
|---|---|---|
| `A` | Auto-LF | 1 |
| `B` | Processing rounds per second (read-only) | — |
| `E` | Echo | 1 |
| `F` | Initial RTT / fixed T1 (× 10 ms) | 500 |
| `K` | Timestamp in monitor | 0 |
| `N` | Retry limit (N2) | 20 |
| `O` | Max outstanding frames (MAXFRAME) | 3 |
| `P` | CSMA persistence | 255 |
| `R` | Digipeater on/off | 0 |
| `T` | TX delay (× 10 ms) | 0 |
| `W` | Slot time (× 10 ms) | 20 |
| `X` | TX enable | 1 |
| `Y` | Max incoming connections | 4 |
| `Z` | Flow control (bit 0 = RTS/CTS, bit 1 = XON/XOFF) | 3 |

Global configuration persists in NVS. Channel-specific `F`/`N`/`O` settings
are session-local; disconnected channels inherit channel-0 defaults.
Request counts are length-minus-one for commands and data alike (1–256 bytes).
See the [WA8DED guide](website/guide/wa8ded.md) for ranges and compatibility limits.

### Event codes

| Code | Meaning |
|---|---|
| 0 | Acknowledgement / no event |
| 1 | Informational text |
| 2 | Error text |
| 3 | Link status change (`CONNECTED to CALL`, `DISCONNECTED fm CALL`, `LINK FAILURE with CALL`) |
| 4 | Monitor header with no data payload |
| 5 | Monitor header followed by data on a later poll |
| 6 | Monitor I/UI data (counted binary payload) |
| 7 | Connected data (counted binary payload) |

Monitor format: `fm SRC to DST [via R1 R2*] ctl UI^ pid F0`.
Connected `L` replies end with a numeric L2 state, not a connected boolean.

Legacy hostmode compatibility target: TFPCX/TSTHOST, WinPack, BPQ32/LinBPQ (port type `DED`), JNOS, Graphic Packet, PaxTerm. For F6FBB/LinFBB, KISS through Linux AX.25 is recommended.

For full protocol details see [website/guide/wa8ded](https://michtronics.github.io/AXLoRaTNC/guide/wa8ded).

---

## APRS

APRS frames are AX.25 UI frames with PID `0xF0`. AXLoRaTNC decodes incoming APRS frames automatically and logs the parsed content (position, message, status, weather) via the console log.

To send APRS position beacons, use the `beacon aprs` shortcut which sets the destination to `APRS` and formats the info field:

```text
beacon aprs 52.0167 4.7000 /> LoRa TNC on 869.480 MHz
beacon interval 600
beacon on
```

For a custom APRS info field (compressed position, weather, etc.) set `beacon dest APRS` and `beacon text <full-info-field>` manually.

---

## Architecture

```
Console / KISS / WA8DED serial
        │
      TNC (tnc.cpp)
        │
  ┌─────┴─────┐
AX.25 L2     Beacon / Digi / Mheard / NET/ROM / BBS / APRS
  │
AX.25 frame encoder/decoder + FCS
  │
Radio driver (radio_sx1262.cpp / radio_sx1276.cpp)
  │
RadioLib → SX1262 / SX1276
```

RadioLib is isolated under `src/radio/`. The AX.25 stack under `src/ax25/` has no dependency on RadioLib. The APRS helper under `src/aprs/` is a pure C++ module with no hardware dependency.

---

## Implemented

**AX.25 framing**
- Callsign + SSID address encoding / decoding
- Destination, source, repeater address fields
- C/R (command/response) bit per AX.25 2.2 spec
- UI, I, S (RR, RNR, REJ, SREJ), U (SABM, UA, DISC, DM) frames
- AX.25 CRC-16 FCS; RadioLib packet CRC additionally enabled
- Modulo-8 sequence numbers, window size 4

**Connected-mode (Level 2)**
- Full state machine: Disconnected → Connecting → Connected → Disconnecting / Recovery
- T1 retry timer, T2 deferred-ack timer, T3 keepalive timer
- N2 retry limit with automatic Recovery state on T1 timeout
- Sliding window with retransmit after REJ and single-frame selective retransmit after SREJ
- Receive-side SREJ with short out-of-sequence frame buffering
- RNR / peer-busy flow control
- TX queue per channel (depth 6); data sent from queue after ack

**TNC**
- 8 independent connected channels (10 on Station G2)
- CSMA p-persistent listen-before-talk (TxDelay, Persistence, SlotTime, FullDuplex from KISS)
- Interrupt-driven RX on SX1262 DIO1 pin
- Duty-cycle enforcement (configurable ppm limit)
- UI or full AX.25 digipeater with H-bit update and UI-frame duplicate cache
- UI beacon with configurable destination, path, interval, and text
- APRS position beacon shortcut (`beacon aprs lat lon sym comment`)
- APRS frame detection and decode on receive
- Mheard table (20 entries) with uptime timestamps, RSSI, SNR, via flag
- NET/ROM NODES broadcast, route table (20 entries), route expiry
- Connected node shell: INFO, NODES, ROUTES, MHEARD, BBS, BYE
- NVS mailbox BBS: L, LT, R, S, K, X, B commands; timestamps
- Callsign, radio config, beacon, digi, node, BBS persistent in NVS

**Serial interfaces**
- Console (human-readable, all commands)
- KISS (compatible with all standard KISS software)
- WA8DED hostmode: 8 channels (10 on Station G2), TFPK 2.7-style command replies, selective/extended polling, event codes 0–7, canonical monitor headers with I/U/S/C and callsign filters, 256-byte connected payloads, receive backpressure, I-poll retries, local loopback and NVS-persistent global configuration. See the [WA8DED guide](website/guide/wa8ded.md) for compatibility limits.
