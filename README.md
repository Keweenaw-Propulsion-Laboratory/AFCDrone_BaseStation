# AFC Drone RF69 Dashboard Bridge

This project connects the AFC drone's RF69 telemetry and control radio link to the desktop dashboard through a USB-connected Arduino.

The Arduino acts as a bidirectional protocol bridge:

```text
AFC Drone
   |
   |  RF69 telemetry and commands
   v
Arduino + RF69 base radio
   |
   |  USB serial binary protocol
   v
Desktop Dashboard
```

The bridge does not control the drone directly and does not require a startup handshake. It listens for telemetry packets from the drone, combines them into the dashboard telemetry format, and forwards dashboard commands back to the drone.

## Features

- Receives drone telemetry over an RF69 radio.
- Sends control commands from the dashboard to the drone.
- Uses RadioHead's `RH_RF69` driver on both ends.
- Uses RF encryption with a shared 16-byte key.
- Aggregates the drone's seven status packet types into one dashboard telemetry structure.
- Frames USB messages with packet numbers, message types, payload lengths, and CRC protection.
- Reports RF connection state through an optional status LED.
- Does not send or wait for a radio handshake.

## Hardware

The base station requires:

- An Arduino-compatible board.
- An RFM69W, RFM69HW, or RFM69HCW radio module compatible with the drone's frequency.
- A USB connection between the Arduino and the dashboard computer.
- A suitable antenna for the configured radio frequency.
- A stable 3.3 V power supply for the RF69 module.

The default bridge configuration is:

```cpp
static constexpr uint8_t RFM69_CS  = 10;
static constexpr uint8_t RFM69_INT = 2;
static constexpr uint8_t RFM69_RST = 9;

static constexpr float RF69_FREQUENCY_MHZ = 915.0f;
static constexpr uint32_t USB_BAUD = 115200;
```

Change these values to match the selected Arduino and RF69 wiring.

### Default RF69 wiring

| RF69 signal | Arduino connection | Purpose |
|---|---:|---|
| `CS` / `NSS` | Pin 10 | SPI chip select |
| `DIO0` / `G0` | Pin 2 | Packet-ready interrupt |
| `RST` | Pin 9 | Radio reset |
| `SCK` | Board SPI SCK | SPI clock |
| `MOSI` | Board SPI MOSI | SPI data to radio |
| `MISO` | Board SPI MISO | SPI data from radio |
| `3.3V` | Regulated 3.3 V | Radio power |
| `GND` | Ground | Common ground |

`DIO0` must be connected to the interrupt pin configured by `RFM69_INT`. The radio may initialize over SPI even when this line is missing, but received packets will not be reported correctly.

Do not power a bare RF69 module from 5 V. Ensure the Arduino's SPI signal levels are compatible with the radio or use a breakout board with suitable regulation and level shifting.

### Status LED

The bridge leaves the status LED disabled by default:

```cpp
static constexpr int8_t STATUS_LED_PIN = -1;
```

Set this to a free output pin to indicate whether RF packets have been received recently.

Do not use `LED_BUILTIN` on boards where the built-in LED shares the SPI clock pin. On an Arduino Uno, pin 13 is both `LED_BUILTIN` and SPI `SCK`.

## Software dependencies

The bridge firmware requires:

- Arduino framework
- SPI library
- RadioHead library with `RH_RF69`

The drone and bridge must use compatible versions of the RadioHead packet format.

## RF configuration

Both radios must use the same:

- Frequency
- Modem profile
- Encryption key
- Packet structure

The bridge uses:

```cpp
radio.setModemConfig(RH_RF69::GFSK_Rb250Fd250);
radio.setFrequency(915.0f);
radio.setTxPower(20, true);
```

The encryption key is:

```cpp
static const uint8_t RF69_ENCRYPTION_KEY[16] = {
    0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08,
    0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08
};
```

The same 16-byte key must be configured on the drone.

For the high-power RFM69HW or RFM69HCW, keep:

```cpp
static constexpr bool RF69_HIGH_POWER = true;
```

Use `false` for a low-power RFM69W.

## Radio protocol

Each RF application payload is exactly eight bytes and is represented by the drone's `radio_Message` union.

Packet metadata is stored in RadioHead's packet headers rather than inside the application payload:

| RadioHead field | Use |
|---|---|
| Header ID | RF packet number |
| Header flags | Message type |
| Payload | Eight-byte `radio_Message` |

### Drone transmit behavior

The drone transmits a packet by setting the RadioHead headers and sending only the eight-byte payload:

```cpp
radio.setHeaderId(header.msgNum);
radio.setHeaderFlags(header.packetType);
radio.send(frame, sizeof(radio_Message));
```

### Base receive behavior

The bridge receives the eight-byte payload and reads its metadata separately:

```cpp
radio.recv(frame, &length);

uint8_t packetNumber = radio.headerId();
uint8_t messageType = radio.headerFlags();
```

The bridge rejects RF payloads that are not exactly eight bytes long.

## RF message types

| Value | Name | Direction | Purpose |
|---:|---|---|---|
| `0` | `SETUP` | Reserved | Currently ignored by the bridge |
| `1` | `STATUS0` | Drone to base | Timing, runtime, RSSI, and mode |
| `2` | `STATUS1` | Drone to base | Gimbal and servo values |
| `3` | `STATUS2` | Drone to base | Motor values and voltage |
| `4` | `STATUS3` | Drone to base | Quaternion |
| `5` | `STATUS4` | Drone to base | Acceleration |
| `6` | `STATUS5` | Drone to base | Velocity |
| `7` | `STATUS6` | Drone to base | Position |
| `8` | `COMMAND` | Base to drone | Target and active-slot controls |
| `9` | `CONFIG` | Base to drone | Reserved for configuration |

## Telemetry packet layout

Every RF status payload is eight bytes.

### STATUS0 — System status

| Field | Type |
|---|---|
| Loop-time average | `uint16_t` |
| Maximum loop time | `uint16_t` |
| Runtime | `uint16_t` |
| RSSI | `uint8_t` |
| Current mode | `uint8_t` |

### STATUS1 — Gimbal and servos

| Field | Type |
|---|---|
| Gimbal pitch | `int16_t` |
| Gimbal yaw | `int16_t` |
| Top servo setpoint | `int16_t` |
| Bottom servo setpoint | `int16_t` |

### STATUS2 — Motors and voltage

| Field | Type |
|---|---|
| Bottom motor setpoint | `uint8_t` |
| Top motor setpoint | `uint8_t` |
| Voltage | `uint16_t` |
| Reserved | Four bytes |

The drone currently sets voltage to zero.

### STATUS3 — Quaternion

| Field | Type |
|---|---|
| `qR` | `int16_t` |
| `qI` | `int16_t` |
| `qJ` | `int16_t` |
| `qK` | `int16_t` |

The current drone implementation sends zeros for these values.

### STATUS4 — Acceleration

| Field | Type |
|---|---|
| X acceleration | `int16_t` |
| Y acceleration | `int16_t` |
| Z acceleration | `int16_t` |
| Reserved | `int16_t` |

The current drone implementation sends zeros for these values.

### STATUS5 — Velocity

| Field | Type |
|---|---|
| X velocity | `int16_t` |
| Y velocity | `int16_t` |
| Z velocity | `int16_t` |
| Reserved | `int16_t` |

The current drone implementation sends zeros for these values.

### STATUS6 — Position

| Field | Type |
|---|---|
| X position | `int16_t` |
| Y position | `int16_t` |
| Z position | `int16_t` |
| Reserved | `int16_t` |

The current drone implementation sends zeros for these values.

## Telemetry aggregation

The drone sends telemetry as several independent RF packets. The bridge stores the most recently received value from each status packet in one `DashboardTelemetry` structure.

The bridge sends this combined structure to the dashboard when at least one field has changed, limited to one frame every 50 ms:

```cpp
static constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
```

This produces a maximum USB telemetry update rate of approximately 20 Hz. RF status packets may be transmitted more frequently, but the dashboard receives the most recent combined state at this rate.

The dashboard telemetry payload is 54 bytes. Some fields, including latitude and longitude, remain zero because the current RF protocol uses `STATUS6` for local position rather than GPS coordinates.

## Command flow

Dashboard command frames use message type `4` on the USB connection and must contain an eight-byte command payload.

The bridge forwards this payload over RF as follows:

```cpp
radio.setHeaderId(radioTxPacketNumber++);
radio.setHeaderFlags(RADIO_COMMAND);
radio.send(commandPayload, 8);
```

The drone reads the RadioHead flags to identify a `COMMAND` message and applies the payload to one of its two target slots.

The current command handler supports:

- Target gimbal X
- Target gimbal Y
- Motor 0 speed
- Motor 1 speed
- Target-slot selection
- Active-slot selection

The exact command byte and bit layout is defined by the shared `radio_Message` declaration and must remain identical on the drone, bridge, and dashboard.

## USB dashboard protocol

The bridge communicates with the dashboard at 115200 baud using binary frames.

```text
A5 5A 01 <packetLo> <packetHi> <type> <length> <payload...> <crcLo> <crcHi>
```

| Field | Size | Description |
|---|---:|---|
| Magic | 2 bytes | `0xA5 0x5A` |
| Version | 1 byte | Protocol version, currently `0x01` |
| Packet number | 2 bytes | Little-endian packet counter |
| Type | 1 byte | Dashboard message type |
| Length | 1 byte | Payload length |
| Payload | Variable | Up to 60 bytes |
| CRC | 2 bytes | Little-endian CRC-16 |

### Dashboard message types

| Value | Name | Purpose |
|---:|---|---|
| `0` | Raw | Reserved |
| `1` | Debug text | Framed status and diagnostic messages |
| `2` | Radio packet | Reserved for relayed radio data |
| `3` | Telemetry | Combined 54-byte telemetry payload |
| `4` | Command | Eight-byte command payload sent to the drone |

### CRC

The protocol uses CRC-16/CCITT-FALSE with:

- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- CRC byte order: little-endian

The CRC covers:

```text
packetLo, packetHi, type, length, payload
```

The CRC does not cover:

- The two magic bytes
- The protocol version byte

## Startup behavior

The bridge performs the following startup sequence:

1. Starts the USB serial port at 115200 baud.
2. Resets the RF69 module.
3. Initializes the RadioHead driver.
4. Configures the modem profile.
5. Configures the frequency.
6. Sets the encryption key and transmit power.
7. Enters receive mode.
8. Sends the framed dashboard debug message `RF69 bridge ready`.

The bridge does not send an RF setup packet or acknowledgement.

After the first valid telemetry packet is received, it sends the framed dashboard debug message:

```text
RF telemetry received
```

## Building and uploading

1. Install the RadioHead library in the Arduino development environment.
2. Open `afc_radio_dashboard_bridge_v6_current_status_layout.ino`.
3. Select the correct Arduino board and USB port.
4. Verify the RF69 pin definitions.
5. Verify the frequency, encryption key, and high-power setting.
6. Compile and upload the sketch.
7. Close the Arduino Serial Monitor.
8. Open the USB port from the desktop dashboard at 115200 baud.

Only one program should open the Arduino serial port at a time.

## Operation

1. Power the drone and base station.
2. Connect the base Arduino to the dashboard computer over USB.
3. Start the dashboard and connect to the correct serial port.
4. Confirm that the dashboard displays `RF69 bridge ready`.
5. Start drone telemetry transmission.
6. Confirm that the dashboard displays `RF telemetry received`.
7. Verify that telemetry fields begin updating.
8. Use the dashboard controls to send commands back to the drone.

## Troubleshooting

### Only `RF69 bridge ready` appears

The USB protocol is working, but the bridge has not accepted a telemetry packet.

Check:

- The drone is calling `radio_sendStatus0()` through `radio_sendStatus6()`.
- `radio_update()` is called continuously after setup completes.
- The drone's outgoing queue is being serviced.
- Both radios use the same frequency and encryption key.
- The RF69 `DIO0` line is connected to the configured interrupt pin.
- The bridge and drone use the same RadioHead modem profile.
- The RF payload is exactly eight bytes.
- The message type is stored in RadioHead header flags.

### `RF payload length mismatch` appears

The received RF application payload is not eight bytes.

Add a compile-time check to the shared radio message definition:

```cpp
static_assert(
    sizeof(radio_Message) == 8,
    "radio_Message must remain exactly 8 bytes"
);
```

Do not add the packet number or message type to the application payload. Those values belong in RadioHead's ID and flags headers.

### Dashboard reports CRC errors

Confirm that the CRC calculation excludes the protocol version byte. The calculation must begin with `packetLo`, not `version`.

The transmitted CRC bytes must be ordered:

```text
crcLo crcHi
```

### Telemetry fields contain incorrect values

Confirm that the bridge and drone use the same status structure layout. A field added, removed, resized, reordered, or moved to another status packet must be changed on both sides.

Always zero-initialize outgoing radio messages:

```cpp
radio_Message msg{};
```

This prevents unassigned bytes from containing stale stack data.

### A motor value matches a servo value

This indicates an outdated `STATUS1` layout. In the current protocol:

- `STATUS1` contains only gimbal and servo values.
- `STATUS2` contains motor values and voltage.

### Radio initializes but receives nothing

Check the RF69 `DIO0` connection first. SPI communication can succeed while the receive interrupt line is disconnected.

Also verify:

- Correct interrupt-capable Arduino pin
- Correct radio module frequency variant
- Antenna installed
- Common ground
- Stable 3.3 V supply
- Correct chip-select pin
- Matching encryption and modem settings

### Raw symbols appear in a serial terminal

The bridge serial stream is binary. A text terminal may display output similar to:

```text
�Z␁␀␀␁␑RF69 bridge ready
```

This is expected. Use the desktop dashboard to parse the framed binary messages.

### Dashboard commands are not applied

Check that:

- The dashboard sends message type `4`.
- The USB command payload is exactly eight bytes.
- The bridge transmits RF message type `COMMAND`, value `8`.
- The drone reads the RF message type from `radio.headerFlags()`.
- The drone's `radio_Message` command layout matches the dashboard payload.
- The bridge returns to receive mode after transmitting.

## Protocol maintenance

The drone, bridge, and dashboard depend on identical binary layouts. When changing the protocol:

1. Keep `radio_Message` exactly eight bytes unless all components are updated together.
2. Update the RF status structure on the drone and bridge.
3. Update the 54-byte dashboard telemetry parser when its layout changes.
4. Keep fields packed and use fixed-width integer types.
5. Add `static_assert` checks for structure sizes.
6. Increment the dashboard protocol version if the USB frame format changes incompatibly.
7. Test telemetry and commands in both directions.

Recommended checks include:

```cpp
static_assert(sizeof(radio_Message) == 8);
static_assert(sizeof(RadioPayload) == 8);
static_assert(sizeof(DashboardTelemetry) == 54);
```

## Current limitations

- RF startup acknowledgement is disabled and unused.
- Packet-loss detection is not yet implemented.
- Received RF packet numbers are read but not currently analyzed.
- The radio receive queue is declared on the drone but is not currently used.
- Configuration messages are reserved but not handled.
- Quaternion, acceleration, velocity, position, and voltage are currently sent as zero by the drone implementation shown here.
- GPS latitude and longitude are present in the dashboard telemetry structure but are not populated by the current RF status layout.
- USB and RF protocols require manually synchronized structure definitions.

## Main files

| File | Purpose |
|---|---|
| `afc_radio_dashboard_bridge_v6_current_status_layout.ino` | Arduino RF69-to-USB bridge firmware |
| Drone `radio.cpp` | RF69 setup, transmit queue, receive handling, status generation, and command handling |
| Shared `radio.h` | Message enums, packet structures, and eight-byte `radio_Message` definition |
| Dashboard serial parser | USB framing, CRC validation, telemetry parsing, and command generation |
