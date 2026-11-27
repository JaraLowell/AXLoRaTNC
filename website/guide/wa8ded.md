# WA8DED hostmode

AXLoRaTNC's serial host protocol targets the supplied **TFPK 2.7**
implementation of WA8DED. It does not pretend to be a PK232 or reproduce
its HF modem hardware. GPNG is a client, not the protocol specification.
For FBB using Linux AX.25, see the [FBB KISS guide](/guide/fbb-kiss).

## Enter and leave hostmode

From the console, send `ESC JHOST1 CR` directly. Alternatively, enter
`mode ded`, then send `ESC JHOST1 CR` from WA8DED terminal mode.
UART clients must initially use the console baud rate (115200); USB CDC
has no corresponding physical baud-rate constraint.

`JHOST0` acknowledges in binary before returning to terminal mode.
In terminal mode, `ESC CONSOLE CR` returns to the console.
Serial mode changes are not persisted.

`CAN ESC JHOST1 CR` resynchronizes a reconnect at a request boundary.
Escape/control bytes inside counted payloads remain data. The receive
buffer holds a complete 259-byte request; an incomplete request is
discarded after one second without another byte.

Wi-Fi port 8100 is **KISS TCP**, not WA8DED TCP; see the [KISS guide](/guide/kiss).

## Binary framing

```text
host -> TNC: [channel] [command flag] [length minus 1] [payload]
TNC -> host: [channel] [reply code] [reply-dependent payload]
```

The request count is **length minus one for both commands and data**.
Payloads are 1-256 bytes, without a required terminating NUL.
For example, the two-character command `@B` is `01 01 01 40 42`
on channel 1. Flag 0 sends data; flag 1 sends a command.

For radio transmission, a 256-byte host payload can exceed the LoRa packet
limit once AX.25 headers and FCS are included. Oversized I or UI frames are
sent with a warning instead of their original data; see
[LoRa packet size](/guide/kiss#lora-packet-size). This does not affect local
loopback, which does not use the radio.

| Reply code | Meaning | Encoding after code |
|---|---|---|
| 0 | Success / nothing pending | None |
| 1 | Command response | NUL-terminated text |
| 2 | Error | NUL-terminated text |
| 3 | Link status | NUL-terminated text |
| 4 | Monitor header, no data follows | NUL-terminated text |
| 5 | Monitor header, data follows on a later poll | NUL-terminated text |
| 6 | Monitor I/UI data | Count-minus-one, then binary bytes |
| 7 | Connected-channel data | Count-minus-one, then binary bytes |

Replies are written as complete frames with partial-write handling.
Code 6 is not connected-channel data, and code 7 is not unproto data.

## Channels and polling

Channel 0 is the unproto/monitor channel. Connected channels are 1-8,
or **1-10 on Station G2**. Channel 255 supports extended polling.

| Command | Selection |
|---|---|
| `G` | Next pending status or data |
| `G0` | Data only: monitor on channel 0, connected data on other channels |
| `G1` | Status only (code 3) |
| Channel 255 `G` | NUL-terminated **binary** list of pending channel numbers plus one |

After a code-5 header, the next channel-0 `G`/`G0` returns its code-6
payload. `G1` may intervene without losing that payload.
An empty extended poll is `FF 01 00`; pending channels 0 and 2 give
`FF 01 01 03 00`.

`L` returns plain space-separated numbers:

- Channel 0: pending status messages, pending monitor frames. A body
  awaiting delivery after its header is not counted as another frame.
- Connected channels: pending status messages, pending receive data,
  unsent transmit frames, unacknowledged frames, retry accounting, L2 state.

The last field is **not a boolean**: 0 disconnected, 1 setup, 3 disconnect
request, 4 transfer, 5 REJ sent, 6 waiting for acknowledgment, 7 local busy,
8 remote busy, 9 both busy, 10-12 waiting-ACK/busy combinations, and
13-15 REJ/busy combinations. TFPK also defines 2 for frame reject; the
current LoRa link engine disconnects on fatal frame rejection instead.

### Local loopback

Connecting to a callsign configured on a local channel pairs the outgoing
channel with the first free matching channel, lowest number first.
Connection, data, acknowledgment and disconnect exchanges stay internal:
no RF transmission, TX delay or duty-cycle waiting. This also works with
`X0`. Busy local destinations fail rather than transmitting to yourself.

For example, `I NL0MSK` on channel 0 sets disconnected channels' identities;
`C NL0MSK` on channel 1 then connects it to channel 2. `NL0MSK-11` must be
explicitly configured on a channel; it is not an automatic BBS endpoint.

## Commands

Numeric queries return the number alone, without a command prefix.
Successful settings normally return code 0.

| Command | Meaning |
|---|---|
| `C [CALL [via PATH]]` | Query peer/path or connect; channel 0 sets unproto destination/path |
| `D` | Disconnect; disconnected channels reply `CHANNEL NOT CONNECTED` (code 1) |
| `I [CALL]` | Query/set identity; channel 0 updates disconnected channels, not active sessions |
| `L` | Status/counters described above |
| `M [MODE [+/- CALLS]]` | Monitor selection/filter |
| `JHOST [0/1]` | Query/change terminal/binary mode |
| `K` | Query timestamp mode and software date/time |
| `K 0/1/2` | No timestamps / status timestamps / status and monitor timestamps |
| `K DD.MM.YY`, `K MM/DD/YY`, `K HH:MM:SS` | Set software clock; date syntax selects display format |
| `U [0/1/2 [TEXT]]` | Unattended mode and retained, case-sensitive incoming connect text |
| `V` | Honest identity: `AXLoRaTNC WA8DED` |
| `B` | Processing rounds per second, read-only |
| `Y [n]` | Incoming limit; query returns `maximum (active)` |
| `QRES` | Reboot; does not erase saved settings |
| `@B` | Estimated shared free storage in 32-byte buffer equivalents, not byte credits |
| `@D [0/1]` | Full duplex; bypasses CSMA, not a diagnostics query |
| `@I [0..256]` | I-poll retry threshold, **not PACLEN** |
| `@T2 [n]`, `@T3 [n]` | Deferred ACK / keepalive timers in 10 ms units |
| `@V [0/1]` | Disable/enable TFPK-style callsign validation |
| `@S` | Numeric L2 state on connected channels; invalid on channel 0 |
| `@DIAG` | AXLoRaTNC extension: radio/reset/serial timeout diagnostics |

`U` retains its text when only the mode changes; at most 72 characters are
accepted. Enabling without saved text returns `NO MESSAGE AVAILABLE`.
Incoming connections receive the text followed by CR. Mode 2 also handles
the terminal unattended `//Q` disconnect request.

`@I` selects a short outstanding I-frame for retransmission with P set
on the first two retries; at/above the threshold, or on later retries,
the modem polls with RR. Connected host data supports full 256-byte
frames independently of this threshold.

`H` and binary `S` are invalid in this TFPK variant. Terminal `S n`
selects a channel and reports its peer. `@Q` and `IPOLL` are not aliases
for resetting settings or setting packet length.

### Parameters and persistence

| Command | Parameter | Fresh default | Range |
|---|---|---|---|
| `A` | Auto-LF | 1 | 0-1 |
| `E` | Terminal echo | 1 | 0-1 |
| `F` | Initial RTT / fixed T1 in 10 ms units | 500 (5 s) | 1-65535 |
| `N` | Retry limit; 0 means unlimited | 20 | 0-127 |
| `O` | Outstanding-frame window | 3 | 1-7 |
| `P` | CSMA persistence | 255 | 0-255 |
| `R` | Digipeater enable | 0 | 0-1 |
| `T` | TX delay in 10 ms units | 0 | 0-64 |
| `W` | CSMA slot time in 10 ms units | 20 | 0-127 |
| `X` | RF TX enable | 1 | 0-1 |
| `Y` | Incoming connection limit | 4; Station G2 10 | 0-channel count |
| `Z` | Bit 0 RTS/CTS; bit 1 XON/XOFF | 3 | 0-3 |
| `@I` | I-poll length threshold | 128 | 0-256 |
| `@T2` | Deferred ACK in 10 ms units | 20 (200 ms) | 0-65535 |
| `@T3` | Keepalive in 10 ms units | 18000 (180 s) | 0-65535 |

Small `F` values 1-15 use TFPK's legacy conversion `value * 100 / 3`.
`F`, `N`, and `O` set on a connected channel are session-local.
Channel-0 values are saved defaults, propagated only to disconnected
channels and restored when a channel disconnects. Other configuration
settings persist in ESP32 NVS; the software clock itself does not survive
reboot. Existing saved values override fresh defaults.

Hardware-specific `@F` (flags, 0-1), `@H` (modem mode, 0-3),
`@N` (HF baud: 300/600/1200/2400/4800/9600), and `@O`
(terminal baud: the same plus 19200) accept, retain and query the values,
but do not configure a PK232 HF modem or change the actual USB/UART baud.
Use console radio/baud commands for the real hardware.
`@U 0/1` controls the UI poll bit.

## Monitor and status payloads

`M IUSC` enables I-frames, UI, supervisory/non-UI unnumbered frames,
and monitoring while the selected terminal channel is connected.
`M N` disables monitoring. `+` includes only matching source/destination
calls; `-` excludes matching calls. SSIDs are ignored. Up to six calls
are accepted; excess entries are rejected, not silently truncated.
For example: `M IU + NL0MSK N1ABC`; `M IU -` clears the filter.

```text
fm N1ABC-3 to CQ via N2DIG N3DIG* ctl UI^ pid F0
fm N1ABC to N2ABC ctl I42+ pid F0
fm N1ABC to N2ABC ctl RR3v
```

I-frame digits are N(R), then N(S). Markers are `+` command/P,
`-` response/F, `^` command without P, and `v` response without F.
Legacy frames use `!` for P/F. FRMR reject bytes are included in the
header, not delivered as a separate data payload. No RSSI/SNR decoration
is added to the protocol header.

TFPK-style link status includes the channel prefix in the text as well
as the binary channel byte:

```text
(1) CONNECTED to N1ABC via N2DIG
(1) DISCONNECTED fm N1ABC
(1) BUSY fm N1ABC
(1) LINK FAILURE with N1ABC
(1) LINK RESET fm N1ABC
(1) FRAME REJECT fm N1ABC (012008)
```

Entering retry recovery is not itself a link failure. Status events are
retained when the host event queue is full. Connected receive backpressure
sends RNR without advancing the acknowledgment until data can be retained.
Monitor traffic is bounded to protect connected data and status capacity.

## Compatibility limits and validation

The LoRa link engine is not a complete reproduction of TFPK's adaptive
SRTT/DAMA state machine: `F` reports its configured fixed RTT rather than
measured SRTT; fatal frame rejection disconnects instead of entering TFPK's
FRMR/reset sequence. HF-modem-only settings are compatibility storage.
Client interoperability still requires testing with real host software.

Native tests compile the actual TNC, framing and link code with fake
serial/radio/NVS interfaces, without accessing a modem or transmitting RF:

```powershell
.\test\run-native.ps1
```

The runner requires Visual Studio C++ Build Tools. It covers command
responses, monitor formatting/filtering, selective/extended polling,
channel events and counters, 256-byte binary payloads, full-queue status
retention, receive backpressure, I-poll retries and UI path isolation.
