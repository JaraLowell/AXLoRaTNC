# KISS mode

AXLoRaTNC implements standard KISS framing over USB serial (`0xC0` FEND delimiters).

## Switch to KISS

```text
mode kiss
```

KISS hosts do not need to type a command first:

- The first KISS `0xC0` FEND from the host switches the port fully into KISS mode. This works from the console, from WA8DED terminal mode, and between WA8DED hostmode frames.
- TAPR/Kenwood-style init text `KISS ON` (followed by an ignored `RESTART`) also enters KISS. KISSINIT-style `KISS.INI` sections such as GPNG's work unchanged.
- The standard KISS Return frame `C0 FF C0` leaves KISS and returns to the console. From there, `ESC JHOST1 CR` enters WA8DED hostmode again.

The mode change is not persisted; after a reset the TNC starts in the console.

::: tip Native USB reset
On ESP32-S3 boards using native USB (such as the Station G2), dropping DTR while RTS is asserted hardware-resets the chip. Some KISS clients (GPNG, for example) pulse DTR when they open the port. The TNC reboots in under 1.5 s, sends its boot beacon if beacons are enabled, and then accepts KISS normally.
:::

## Protocol details

- KISS data frames (type 0) are treated as complete AX.25 frames from the host; AXLoRaTNC appends the FCS before LoRa transmit.
- Received AX.25 frames are FCS-checked and emitted as KISS data frames without FCS — exactly as a standard KISS TNC.
- KISS parameter frames (TxDelay=1, Persistence=2, SlotTime=3, FullDuplex=5) are accepted and drive the CSMA algorithm.

## Station G2: Wi-Fi KISS TCP

Station G2 builds include a raw KISS TCP server on **port 8100**, for use with
GPNG and other KISS TCP clients. Wi-Fi is off by default. Configure it from the
USB console; no Wi-Fi credentials belong in the firmware source.

### Join an existing network (2.4 GHz)

```text
wifi off
wifi ssid Your network name
wifi password Your network password
wifi sta
wifi status
```

SSID and password values include everything after the command's space, without
quotes. Passwords are not echoed or shown in status. Disable local echo in your
terminal as well. `wifi password clear` permits joining an open network.

Connection is asynchronous: run `wifi status` again after a few seconds to see
`ready=1` and the DHCP address. Station mode retries after losing Wi-Fi.
A DHCP reservation on your router keeps the TNC's address stable.

### Standalone access point

```text
wifi off
wifi ssid AXLoRa-G2
wifi password <choose an 8-63 byte password>
wifi ap
wifi status
```

Replace the password placeholder before sending the command. Join that Wi-Fi
network from the PC; there is no Internet service on the G2 access point. Its
default address is **192.168.4.1**. SSIDs must be 1-32 bytes; AP passwords must
be 8-63 bytes. AP and station modes share the configured SSID/password;
use `wifi off` before changing credentials.

The selected mode and credentials are stored locally in NVS and restored after
reset. `wifi off` disables Wi-Fi and the TCP listener, also after reset.
`wifi status` reports free internal heap and PSRAM for runtime memory checks.

### GPNG

Select a **KISS / TCP** connection with:

- Host: the address from `wifi status` (or `192.168.4.1` in AP mode).
- Port: **8100**.
- KISS radio port: **0** (the G2 has one radio).

For a single-link GPNG configuration, the equivalent `[tnc]` fields are
`protocol = "kiss"`, `kiss_transport = "tcp"`, `kiss_tcp_host = "<TNC IP>"`,
and `kiss_tcp_port = 8100`. If using GPNG's `links` list, configure that link's
TCP host and port instead. No KISS.INI console entry or DTR pulse is needed.

Only one TCP client is accepted. USB remains a configuration console while TCP
owns received AX.25 frames; the internal link layer does not answer them.
USB KISS/WA8DED sessions and active internal connections prevent TCP acceptance.
Starting a USB host session while TCP is connected closes the TCP client and
gives USB priority. KISS Return (`C0 FF C0`) over TCP closes only that client;
it does not change USB mode or Wi-Fi settings. Received frames and parameter
handling use the same KISS framing as USB, with no text or TX echo on TCP.
Slow clients are disconnected if the output queue fills or sending stalls for
three seconds, rather than blocking radio service.

::: warning Trusted local networks only
KISS TCP has no authentication or encryption. Anyone who can reach port 8100
can transmit using the TNC. Use a protected Wi-Fi network and do not forward
this port to the Internet. Wi-Fi does not enable a web interface.
:::

## Compatible software

| Application | Notes |
|---|---|
| kissattach (AX.25 for Linux) | Standard usage |
| Dire Wolf | Set `ADEVICE` to the ESP32 serial port |
| LinBPQ / BPQ32 | See [BPQ setup guide](/guide/bpq) |
| F6FBB / LinFBB | Recommended via Linux AX.25; see [FBB KISS guide](/guide/fbb-kiss) |
| APRS clients | Any client supporting KISS TNCs |

## kissattach example (Linux)

```sh
sudo kissattach /dev/ttyUSB0 radio 44.137.x.x
sudo kissparms -c 1 -t 0 -s 10 -r 255 -p radio
```

Replace `/dev/ttyUSB0` with the actual device path. Prefer `/dev/serial/by-id/...` for a stable name across reboots.

For connected-mode BBS traffic over LoRa, use `PACLEN` around 40-64 and
`MAXFRAME`/window 1 on the host. If `stats` shows `qdrops`, the host is sending
faster than the radio queue can drain.

## Baud rate

Always 115 200 baud. The ESP32 USB CDC port auto-negotiates; the baud rate set on the host is ignored.
