# AFC Drone RF69 Base Station Relay

This project connects the AFC drone's RF69 radio link to the desktop dashboard (AFCDrone_BaseController) through a USB-connected Arduino.

```text
AFC Drone  <-- RF69 (8-byte Radio::Message) -->  Arduino + RF69  <-- USB RADIO_PACKET frames -->  Dashboard
```

The base station is a transparent relay. It does not decode telemetry or build commands. The dashboard identifies it as a relay on connect and then exchanges raw RF messages with the drone through `RADIO_PACKET` envelopes. When the drone or dashboard adds or changes a status layout, the base station does not need to be rebuilt.

## What the relay does

- Answers the dashboard's identify query (`RAW` `0x3F`) with `0x02`, meaning "base station relay".
- Forwards every RF packet received from the drone to the dashboard as a `RADIO_PACKET` with direction `0` and the downlink RSSI.
- Transmits every `RADIO_PACKET` request from the dashboard over RF, using the requested message type. This covers `COMMAND`, `CONFIG` and the 25 ms `HEARTBEAT` that feeds the drone's comm watchdog.
- Echoes sent packets back as `RADIO_PACKET` with direction `1`, except heartbeats, which would flood the link.
- Answers the drone's `SETUP` link ping (`"AFCDrone"`) with the ACK pattern (8 x `0x69`), so the drone reports `BaseStation CONNECTED`.
- Reports `RF link up` and `RF link lost` (no packets for 2 s) as debug text.
- Pendulum timing test: keeps its clock synchronised with the drone's and times each pass of the drone over a TMAG5273 Hall sensor. See [Pendulum timing test](#pendulum-timing-test).

## Hardware

- Arduino Uno (or compatible)
- RFM69HCW breakout (915 MHz) with level shifting and 3.3 V regulation, such as the Adafruit breakout
- Antenna for 915 MHz

### Wiring

The pins are set at the top of [src/main.cpp](src/main.cpp):

```cpp
static constexpr uint8_t RFM69_CS  = 4;
static constexpr uint8_t RFM69_INT = 3;
static constexpr uint8_t RFM69_RST = 2;
```

| RF69 signal | Arduino Uno pin | Purpose |
|---|---:|---|
| `CS` / `NSS` | 4 | SPI chip select |
| `DIO0` / `G0` | 3 | Packet-ready interrupt (must be pin 2 or 3 on an Uno) |
| `RST` | 2 | Radio reset |
| `SCK` | 13 | SPI clock |
| `MOSI` | 11 | SPI data to radio |
| `MISO` | 12 | SPI data from radio |
| `VIN` | 5 V (breakout) / 3.3 V (bare module) | Power |
| `GND` | GND | Common ground |

If your wiring differs, change the constants to match. If `radio.init()` fails, the relay repeats `RF69 init failed - check CS/RST wiring` as debug text once per second.

Do not use `LED_BUILTIN` as `STATUS_LED_PIN` on an Uno, because pin 13 is also SPI `SCK`.

## RF configuration

These must match the drone (`AFC-Drone/src/radio.cpp`):

| Setting | Value |
|---|---|
| Frequency | 915 MHz |
| Modem profile | `GFSK_Rb250Fd250` (the `RH_RF69::init()` default) |
| Encryption key | `01 02 03 04 05 06 07 08 01 02 03 04 05 06 07 08` |
| Payload | Exactly 8 bytes (`Radio::Message`) |
| RadioHead header ID | Packet number |
| RadioHead header flags | Message type |

RF message types (from the drone's `Radio::MessageType`):

| Value | Name | Direction |
|---:|---|---|
| 0 | `SETUP` | Drone ping and relay ACK |
| 1–7 | `STATUS0`–`STATUS6` | Drone to dashboard |
| 8 | `COMMAND` | Dashboard to drone |
| 9 | `CONFIG` | Both |
| 10 | `HEARTBEAT` | Dashboard to drone |
| 15 | `TIME_PING` | Relay to drone (clock sync; not forwarded) |
| 16 | `TIME_PONG` | Drone to relay (clock sync; not forwarded) |

## USB protocol

115200 baud. The framing is the same as the drone's direct USB link:

```text
A5 5A 01 <packetLo> <packetHi> <type> <length> <payload...> <crcLo> <crcHi>
```

The CRC is CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, little-endian). It covers the packet number, type, length and payload, but not the magic or version bytes.

| Type | Name | Relay behavior |
|---:|---|---|
| 0 | `RAW` | `0x3F` gets the reply `0x02`. Anything else is ignored. |
| 1 | `DEBUG_TEXT` | Relay status messages |
| 2 | `RADIO_PACKET` | RF traffic in both directions (see below) |

`RADIO_PACKET` payload:

| Offset | Field |
|---:|---|
| 0 | Direction: `0` received over RF, `1` sent over RF |
| 1 | RF packet number (`headerId`) |
| 2 | RF message type (`headerFlags`) |
| 3–10 | 8-byte RF message |
| 11 | Downlink RSSI magnitude, for example `62` = -62 dBm (received packets only) |

For dashboard-to-relay requests, only bytes 2–10 are used. The relay assigns its own RF packet number.

## Pendulum timing test

The drone swings on a pendulum logging its gyro to SD. A magnet on the drone passes over a TMAG5273 at the bottom of the swing, and the relay times each pass with its own `micros()`. Radio clock sync then puts those times on the drone's clock, so they can be compared with the gyro to well under 1 ms.

### Extra wiring

| TMAG5273 (SparkFun Qwiic) | Arduino Uno | Notes |
|---|---|---|
| `SDA` | A4 | Through a 3.3 V level shifter; the board is 3.3 V only |
| `SCL` | A5 | Through the level shifter |
| `3V3` | 3.3 V | |
| `GND` | GND | |

The sensor is found at 0x22, 0x35, 0x78 or 0x44, whichever answers with TI's ID, and reported at boot as `TMAG5273 at 0x22, +-80 mT, axes XYZ`. All three axes are read, and the detector uses the magnitude of the change from the resting field, |B − B_rest|. That gives one hump, symmetric about the bottom of the swing, for any magnet orientation: axial, diametric, or a stack of several. The sensor can face any way. Keep the magnet within a few cm of it at the bottom of the swing.

### Tuning

Settings are at the top of [src/main.cpp](src/main.cpp):

| Constant | Default | Purpose |
|---|---|---|
| `PASS_ENTER_UT` / `PASS_EXIT_UT` | 3000 / 1500 uT | Field change that starts a pass, and the level it must stay under for the pass to end |
| `PASS_MERGE_MS` | 60 | How long the field must stay under the exit level before the pass closes. Dips between lobes of a magnet stack stay in one pass. |
| `PASS_MIN_SAMPLES` | 3 | Fewer samples above the enter level is a glitch, and the pass is dropped |
| `PASS_MAX_MS` | 1500 | A longer "pass" is the magnet parked over the sensor. It is dropped, and nothing is counted until the field clears. |
| `BASELINE_TRACK_UT` | 300 uT | The resting field follows slow drift only while the field is within this distance of it |
| `MAG_SAMPLE_HZ` | 2000 | Field sampling rate. Reads are paced to it for even spacing. |
| `PING_INTERVAL_MS` | 200 | Clock-sync rate |

Once a second the relay prints `MAG dB <change> uT max <largest change that second> uT <rate> Hz passes <n> hb dropped <n>`. Swing the drone by hand and set `PASS_ENTER_UT` to about half the smallest `max` you see on a pass. The baseline is learnt from the first 256 samples after boot, so keep the magnet away from the sensor while the relay starts. After that it does not follow the magnet, even when the pendulum comes to rest over the sensor.

### How a pass is timed

Each sample is stamped with `micros()`. The pass time is the centroid of |B − B_rest| above the exit level, integrated over the actual sample times. For a field symmetric about the bottom of the swing, that is the bottom, independent of the threshold. No radio transmit happens while a pass is in progress, because each one stalls the loop for about 2.6 ms and would leave a hole in the samples. Clock-sync pings wait, dashboard heartbeats (every 25 ms) are dropped, and other transmit requests and the SETUP ACK are queued (up to 4) and sent when the pass ends. Dropping heartbeats is fine for the pendulum test, where the drone does not need them, but this firmware should not be used to fly.

In `BASE_MAG_PASS`, `peakRaw` is the peak field change and `baselineRaw` is the size of the resting field, both in counts and both positive. Neither gives the swing direction any more. Passes alternate direction, so tell them apart by parity, not by sign.

### Clock sync

Every 200 ms, while the drone is linked, the relay sends `TIME_PING` and the drone answers `TIME_PONG`. Both ends stamp the moments the RFM69 reports PacketSent and PayloadReady inside its interrupt. The Uno's ceramic resonator can be off by thousands of ppm and drifts with temperature, so offsets are not applied live: every exchange is logged, and `AFC-Drone/scripts/pendulum_sync.py` fits them afterwards.

### Frames to the dashboard

| Type | Payload |
|---:|---|
| 20 `BASE_TIMESYNC` (18 bytes) | `uint16 seq`, `uint32 t1`, `uint32 t2`, `uint32 t4`, `uint8 prevSeq`, `uint16 prevTxMinusRx`, `uint8 rssi` |
| 21 `BASE_MAG_PASS` (26 bytes) | `uint16 index`, `uint32 centroidUs`, `uint32 startUs`, `uint32 endUs`, `uint32 peakUs`, `int16 peakRaw` (\|ΔB\| peak), `int16 baselineRaw` (\|B_rest\|), `uint16 samples`, `uint16 maxGapUs` |

Run the dashboard server with `--pendulum-log DIR` to write these to CSV.

## Building

```sh
pio run -t upload
```

Close any serial monitor before connecting the dashboard. Only one program can hold the port.

## Operation

1. Flash and plug in the base station, then power the drone.
2. In the dashboard, connect to the base station's serial port at 115200 baud.
3. The link badge should read **Link: Relay (base station)**, and the terminal should show `RF69 relay ready`.
4. Once the drone transmits, you should see `RF link up`, the drone logs `BaseStation CONNECTED` on its own USB port, and telemetry starts updating.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Dashboard says "No identify response… assuming direct connection" | Old firmware is still flashed, or the wrong port is selected |
| `RF69 init failed` repeating | CS or RST pin mismatch, or no power to the radio |
| `RF69 relay ready` shows but never `RF link up` | `DIO0` is not on `RFM69_INT`, the drone has `radioEnabled` off, or the frequency or key differs |
| Telemetry updates but the drone will not leave SAFE | Heartbeats are not getting through. Check range and antenna, since the drone's watchdog allows only 100 ms between heartbeats |
| `RF payload length mismatch` | A sender is transmitting something other than 8 bytes |
