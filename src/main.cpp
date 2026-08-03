/*
 * AFC Drone RF69 <-> Desktop Dashboard Bridge
 *
 * Radio side:
 *   - RadioHead RH_RF69
 *   - 915 MHz
 *   - 16-byte encryption key matching the drone
 *   - RadioHead ID header: packet number
 *   - RadioHead FLAGS header: message type
 *   - RF application payload: 8-byte radio_Message
 *   - No RF startup handshake; the bridge only receives telemetry and sends controls
 *
 * USB side:
 *   - Binary dashboard frame:
 *       A5 5A 01 <packetLo> <packetHi> <type> <length> <payload...> <crcLo> <crcHi>
 *   - CRC-16/CCITT-FALSE is calculated over:
 *       packet number, type, length, and payload (version excluded)
 *
 * IMPORTANT:
 *   Do not print normal text to Serial. Any ASCII output will corrupt the
 *   dashboard's binary packet stream.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Board configuration
// -----------------------------------------------------------------------------

// Change these three pins to match the Arduino and RF69 breakout wiring.
static constexpr uint8_t RFM69_CS  = 4;
static constexpr uint8_t RFM69_INT = 3;
static constexpr uint8_t RFM69_RST = 2;

static constexpr float RF69_FREQUENCY_MHZ = 915.0f;

// Set false for the low-power RFM69W. Keep true for RFM69HW/RFM69HCW.
static constexpr bool RF69_HIGH_POWER = true;
static constexpr int8_t RF69_TX_POWER_DBM = 20;

// Leave this at -1 unless a status LED is wired to a pin that is not used by
// SPI. On an Arduino Uno, LED_BUILTIN is pin 13, which is also SPI SCK.
static constexpr int8_t STATUS_LED_PIN = -1;

static constexpr uint32_t USB_BAUD = 115200;

// The dashboard can accept telemetry faster than this, but 20 Hz is enough for
// graphs and keeps the serial stream comfortably below its limit.
static constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
static constexpr uint32_t RADIO_TIMEOUT_MS = 2000;

static const uint8_t RF69_ENCRYPTION_KEY[16] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
};

RH_RF69 radio(RFM69_CS, RFM69_INT);

// -----------------------------------------------------------------------------
// Radio protocol
// -----------------------------------------------------------------------------

enum RadioMessageType : uint8_t {
    RADIO_SETUP   = 0,
    RADIO_STATUS0 = 1,
    RADIO_STATUS1 = 2,
    RADIO_STATUS2 = 3,
    RADIO_STATUS3 = 4,
    RADIO_STATUS4 = 5,
    RADIO_STATUS5 = 6,
    RADIO_STATUS6 = 7,
    RADIO_COMMAND = 8,
    RADIO_CONFIG  = 9
};

struct __attribute__((packed)) RadioStatus0 {
    uint16_t loopTimeAvg;
    uint16_t loopTimeMax;
    uint16_t runTime;
    uint8_t rssi;
    uint8_t currentMode;
};

struct __attribute__((packed)) RadioStatus1 {
    int8_t gimbalPitchNorm;
    int8_t gimbalYawNorm;
    uint8_t topServoSet;
    uint8_t bottomServoSet;
    uint8_t motor1Set;
    uint8_t motor2Set;
    uint16_t voltage;
};

struct __attribute__((packed)) RadioStatus2 {
    int16_t qR;
    int16_t qI;
    int16_t qJ;
    int16_t qK;
};

struct __attribute__((packed)) RadioStatus3 {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t unused;
};

struct __attribute__((packed)) RadioStatus4 {
    int16_t velX;
    int16_t velY;
    int16_t velZ;
    int16_t unused;
};

struct __attribute__((packed)) RadioStatus5 {
    int16_t posX;
    int16_t posY;
    int16_t posZ;
    int16_t unused;
};

struct __attribute__((packed)) RadioStatus6 {
    float latitude;
    float longitude;
};

union __attribute__((packed)) RadioPayload {
    uint8_t raw[8];
    char text[8];
    RadioStatus0 status0;
    RadioStatus1 status1;
    RadioStatus2 status2;
    RadioStatus3 status3;
    RadioStatus4 status4;
    RadioStatus5 status5;
    RadioStatus6 status6;
};

static_assert(sizeof(RadioPayload) == 8, "Every RF payload must be 8 bytes");

// -----------------------------------------------------------------------------
// Dashboard protocol
// -----------------------------------------------------------------------------

enum DashboardMessageType : uint8_t {
    DASHBOARD_RAW          = 0,
    DASHBOARD_DEBUG_TEXT   = 1,
    DASHBOARD_RADIO_PACKET = 2,
    DASHBOARD_TELEMETRY    = 3,
    DASHBOARD_COMMAND      = 4
};

struct __attribute__((packed)) DashboardTelemetry {
    // Status 0
    uint16_t loopTimeAvg;
    uint16_t loopTimeMax;
    uint16_t runTime;
    uint8_t rssi;
    uint8_t currentMode;

    // Status 1
    int16_t gimbalPitch;
    int16_t gimbalYaw;
    int16_t topServoSet;
    int16_t bottomServoSet;

    // Remaining status 1 values
    uint8_t motor1Set;
    uint8_t motor2Set;
    uint16_t voltage;

    // Status 2
    int16_t qR;
    int16_t qI;
    int16_t qJ;
    int16_t qK;

    // Status 3
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;

    // Status 4
    int16_t velX;
    int16_t velY;
    int16_t velZ;

    // Status 5
    int16_t posX;
    int16_t posY;
    int16_t posZ;

    // Status 6
    float latitude;
    float longitude;
};

static_assert(sizeof(DashboardTelemetry) == 54,
              "Dashboard telemetry layout must remain 54 bytes");

static constexpr uint8_t DASHBOARD_MAGIC_0 = 0xA5;
static constexpr uint8_t DASHBOARD_MAGIC_1 = 0x5A;
static constexpr uint8_t DASHBOARD_VERSION = 0x01;
static constexpr uint8_t DASHBOARD_MAX_PAYLOAD = 60;

static DashboardTelemetry telemetry = {};
static bool telemetryDirty = false;
static uint16_t dashboardTxPacketNumber = 0;
static uint8_t radioTxPacketNumber = 0;
static uint32_t lastTelemetrySendMs = 0;
static uint32_t lastRadioPacketMs = 0;
static bool droneConnected = false;
static bool reportedFirstTelemetry = false;
static bool reportedLengthMismatch = false;

// -----------------------------------------------------------------------------
// CRC
// -----------------------------------------------------------------------------

static uint16_t crc16Update(uint16_t crc, uint8_t value) {
    crc ^= static_cast<uint16_t>(value) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
        if ((crc & 0x8000u) != 0u) {
            crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
        } else {
            crc = static_cast<uint16_t>(crc << 1);
        }
    }

    return crc;
}

static uint16_t dashboardCrc(
    const uint8_t header[5],
    const uint8_t* payload,
    uint8_t payloadLength
) {
    uint16_t crc = 0xFFFFu;

    // The dashboard excludes header[0] (protocol version) from its CRC.
    // It covers packetLo, packetHi, type, length, and payload.
    for (uint8_t i = 1; i < 5; ++i) {
        crc = crc16Update(crc, header[i]);
    }

    for (uint8_t i = 0; i < payloadLength; ++i) {
        crc = crc16Update(crc, payload[i]);
    }

    return crc;
}

// -----------------------------------------------------------------------------
// Dashboard transmit
// -----------------------------------------------------------------------------

static void sendDashboardFrame(
    DashboardMessageType type,
    const uint8_t* payload,
    uint8_t payloadLength
) {
    if (payloadLength > DASHBOARD_MAX_PAYLOAD) {
        return;
    }

    const uint16_t packetNumber = dashboardTxPacketNumber++;

    // CRC excludes the magic bytes and protocol version.
    uint8_t crcHeader[5] = {
        DASHBOARD_VERSION,
        static_cast<uint8_t>(packetNumber & 0xFFu),
        static_cast<uint8_t>((packetNumber >> 8) & 0xFFu),
        static_cast<uint8_t>(type),
        payloadLength
    };

    const uint16_t crc = dashboardCrc(crcHeader, payload, payloadLength);

    const uint8_t magic[2] = {
        DASHBOARD_MAGIC_0,
        DASHBOARD_MAGIC_1
    };

    const uint8_t crcBytes[2] = {
        static_cast<uint8_t>(crc & 0xFFu),
        static_cast<uint8_t>((crc >> 8) & 0xFFu)
    };

    // The payload write is deliberately separate and must not be omitted.
    Serial.write(magic, sizeof(magic));
    Serial.write(crcHeader, sizeof(crcHeader));

    if (payloadLength > 0 && payload != nullptr) {
        Serial.write(payload, payloadLength);
    }

    Serial.write(crcBytes, sizeof(crcBytes));
}

static void sendTelemetry() {
    sendDashboardFrame(
        DASHBOARD_TELEMETRY,
        reinterpret_cast<const uint8_t*>(&telemetry),
        sizeof(telemetry)
    );

    telemetryDirty = false;
    lastTelemetrySendMs = millis();
}

static void sendDashboardText(const char* text) {
    if (text == nullptr) {
        return;
    }

    const size_t length = strlen(text);
    const uint8_t sendLength = static_cast<uint8_t>(
        length > DASHBOARD_MAX_PAYLOAD ? DASHBOARD_MAX_PAYLOAD : length
    );

    sendDashboardFrame(
        DASHBOARD_DEBUG_TEXT,
        reinterpret_cast<const uint8_t*>(text),
        sendLength
    );
}

// -----------------------------------------------------------------------------
// RF transmit
// -----------------------------------------------------------------------------

static void restoreRadioReceiveMode() {
    radio.setModeRx();
}

static void sendDroneCommand(const uint8_t commandPayload[8]) {
    // Match the drone protocol exactly: packet metadata lives in RadioHead's
    // ID and FLAGS headers, while send() receives only the 8-byte message.
    radio.setHeaderId(radioTxPacketNumber++);
    radio.setHeaderFlags(RADIO_COMMAND);

    radio.send(commandPayload, sizeof(RadioPayload));
    radio.waitPacketSent();
    restoreRadioReceiveMode();
}

// -----------------------------------------------------------------------------
// RF receive and telemetry aggregation
// -----------------------------------------------------------------------------

static uint8_t rssiMagnitudeFromDbm(int16_t rssiDbm) {
    // RH_RF69 normally reports a negative dBm number. The current dashboard
    // telemetry layout stores one unsigned byte, so send its positive magnitude.
    int16_t magnitude = -rssiDbm;

    if (magnitude < 0) {
        magnitude = 0;
    } else if (magnitude > 255) {
        magnitude = 255;
    }

    return static_cast<uint8_t>(magnitude);
}

static void applyRadioStatus(uint8_t messageType, const RadioPayload& payload) {
    switch (messageType) {
        case RADIO_STATUS0:
            telemetry.loopTimeAvg = payload.status0.loopTimeAvg;
            telemetry.loopTimeMax = payload.status0.loopTimeMax;
            telemetry.runTime = payload.status0.runTime;
            telemetry.currentMode = payload.status0.currentMode;
            break;

        case RADIO_STATUS1:
            telemetry.gimbalPitch =
                static_cast<int16_t>(payload.status1.gimbalPitchNorm);
            telemetry.gimbalYaw =
                static_cast<int16_t>(payload.status1.gimbalYawNorm);
            telemetry.topServoSet =
                static_cast<int16_t>(payload.status1.topServoSet);
            telemetry.bottomServoSet =
                static_cast<int16_t>(payload.status1.bottomServoSet);
            telemetry.motor1Set = payload.status1.motor1Set;
            telemetry.motor2Set = payload.status1.motor2Set;
            telemetry.voltage = payload.status1.voltage;
            break;

        case RADIO_STATUS2:
            telemetry.qR = payload.status2.qR;
            telemetry.qI = payload.status2.qI;
            telemetry.qJ = payload.status2.qJ;
            telemetry.qK = payload.status2.qK;
            break;

        case RADIO_STATUS3:
            telemetry.accelX = payload.status3.accelX;
            telemetry.accelY = payload.status3.accelY;
            telemetry.accelZ = payload.status3.accelZ;
            break;

        case RADIO_STATUS4:
            telemetry.velX = payload.status4.velX;
            telemetry.velY = payload.status4.velY;
            telemetry.velZ = payload.status4.velZ;
            break;

        case RADIO_STATUS5:
            telemetry.posX = payload.status5.posX;
            telemetry.posY = payload.status5.posY;
            telemetry.posZ = payload.status5.posZ;
            break;

        case RADIO_STATUS6:
            telemetry.latitude = payload.status6.latitude;
            telemetry.longitude = payload.status6.longitude;
            break;

        default:
            return;
    }

    telemetryDirty = true;
}

static void processRadio() {
    if (!radio.available()) {
        return;
    }

    uint8_t frame[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t length = sizeof(frame);

    if (!radio.recv(frame, &length)) {
        return;
    }

    lastRadioPacketMs = millis();
    droneConnected = true;
    telemetry.rssi = rssiMagnitudeFromDbm(radio.lastRssi());

    // RadioHead removes its four transport headers before recv() returns.
    // The drone stores the packet number in ID, the message type in FLAGS, and
    // sends only the 8-byte radio_Message as the application payload.
    if (length != sizeof(RadioPayload)) {
        if (!reportedLengthMismatch) {
            sendDashboardText("RF payload length mismatch");
            reportedLengthMismatch = true;
        }
        return;
    }

    const uint8_t packetNumber = radio.headerId();
    const uint8_t messageType = radio.headerFlags();
    (void)packetNumber; // Reserved for packet-loss tracking.

    RadioPayload payload;
    memcpy(payload.raw, frame, sizeof(payload.raw));

    if (messageType >= RADIO_STATUS0 && messageType <= RADIO_STATUS6) {
        applyRadioStatus(messageType, payload);

        if (!reportedFirstTelemetry) {
            sendDashboardText("RF telemetry received");
            reportedFirstTelemetry = true;
        }
    }
}

// -----------------------------------------------------------------------------
// Dashboard receive
// -----------------------------------------------------------------------------

struct DashboardRxParser {
    enum State : uint8_t {
        WAIT_MAGIC_0,
        WAIT_MAGIC_1,
        READ_HEADER,
        READ_PAYLOAD,
        READ_CRC
    };

    State state = WAIT_MAGIC_0;
    uint8_t header[5] = {};
    uint8_t headerIndex = 0;
    uint8_t payload[DASHBOARD_MAX_PAYLOAD] = {};
    uint8_t payloadIndex = 0;
    uint8_t crcBytes[2] = {};
    uint8_t crcIndex = 0;

    void reset() {
        state = WAIT_MAGIC_0;
        headerIndex = 0;
        payloadIndex = 0;
        crcIndex = 0;
    }
};

static DashboardRxParser dashboardRx;

static void handleDashboardFrame(
    uint8_t type,
    const uint8_t* payload,
    uint8_t payloadLength
) {
    if (type == DASHBOARD_COMMAND && payloadLength == 8) {
        sendDroneCommand(payload);
    }
}

static void processDashboardByte(uint8_t value) {
    switch (dashboardRx.state) {
        case DashboardRxParser::WAIT_MAGIC_0:
            if (value == DASHBOARD_MAGIC_0) {
                dashboardRx.state = DashboardRxParser::WAIT_MAGIC_1;
            }
            break;

        case DashboardRxParser::WAIT_MAGIC_1:
            if (value == DASHBOARD_MAGIC_1) {
                dashboardRx.headerIndex = 0;
                dashboardRx.state = DashboardRxParser::READ_HEADER;
            } else if (value != DASHBOARD_MAGIC_0) {
                dashboardRx.state = DashboardRxParser::WAIT_MAGIC_0;
            }
            break;

        case DashboardRxParser::READ_HEADER:
            dashboardRx.header[dashboardRx.headerIndex++] = value;

            if (dashboardRx.headerIndex == sizeof(dashboardRx.header)) {
                const uint8_t version = dashboardRx.header[0];
                const uint8_t payloadLength = dashboardRx.header[4];

                if (version != DASHBOARD_VERSION ||
                    payloadLength > DASHBOARD_MAX_PAYLOAD) {
                    dashboardRx.reset();
                    return;
                }

                dashboardRx.payloadIndex = 0;
                dashboardRx.crcIndex = 0;
                dashboardRx.state =
                    payloadLength == 0
                        ? DashboardRxParser::READ_CRC
                        : DashboardRxParser::READ_PAYLOAD;
            }
            break;

        case DashboardRxParser::READ_PAYLOAD:
            dashboardRx.payload[dashboardRx.payloadIndex++] = value;

            if (dashboardRx.payloadIndex == dashboardRx.header[4]) {
                dashboardRx.crcIndex = 0;
                dashboardRx.state = DashboardRxParser::READ_CRC;
            }
            break;

        case DashboardRxParser::READ_CRC:
            dashboardRx.crcBytes[dashboardRx.crcIndex++] = value;

            if (dashboardRx.crcIndex == sizeof(dashboardRx.crcBytes)) {
                const uint16_t receivedCrc =
                    static_cast<uint16_t>(dashboardRx.crcBytes[0]) |
                    (static_cast<uint16_t>(dashboardRx.crcBytes[1]) << 8);

                const uint16_t calculatedCrc = dashboardCrc(
                    dashboardRx.header,
                    dashboardRx.payload,
                    dashboardRx.header[4]
                );

                if (receivedCrc == calculatedCrc) {
                    handleDashboardFrame(
                        dashboardRx.header[3],
                        dashboardRx.payload,
                        dashboardRx.header[4]
                    );
                }

                dashboardRx.reset();
            }
            break;
    }
}

static void processDashboardSerial() {
    while (Serial.available() > 0) {
        const int value = Serial.read();

        if (value >= 0) {
            processDashboardByte(static_cast<uint8_t>(value));
        }
    }
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

static void setStatusLed(bool on) {
    if (STATUS_LED_PIN >= 0) {
        digitalWrite(static_cast<uint8_t>(STATUS_LED_PIN), on ? HIGH : LOW);
    }
}

[[noreturn]] static void fatalBlink() {
    if (STATUS_LED_PIN >= 0) {
        pinMode(static_cast<uint8_t>(STATUS_LED_PIN), OUTPUT);
    }

    while (true) {
        setStatusLed(true);
        delay(100);
        setStatusLed(false);
        delay(100);
    }
}

static void resetRadioHardware() {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
}

void setup() {
    if (STATUS_LED_PIN >= 0) {
        pinMode(static_cast<uint8_t>(STATUS_LED_PIN), OUTPUT);
        setStatusLed(false);
    }

    Serial.begin(USB_BAUD);

    resetRadioHardware();

    if (!radio.init()) {
        fatalBlink();
    }

    // Use the exact modem profile that was verified by the standalone
    // receiver. Both ends must use the same profile.
    if (!radio.setModemConfig(RH_RF69::GFSK_Rb250Fd250)) {
        fatalBlink();
    }

    if (!radio.setFrequency(RF69_FREQUENCY_MHZ)) {
        fatalBlink();
    }

    radio.setEncryptionKey(RF69_ENCRYPTION_KEY);
    radio.setTxPower(RF69_TX_POWER_DBM, RF69_HIGH_POWER);
    restoreRadioReceiveMode();

    // This is a framed dashboard debug message, not raw serial text.
    sendDashboardText("RF69 bridge ready");
}

void loop() {
    processDashboardSerial();
    processRadio();

    const uint32_t now = millis();

    if (droneConnected &&
        static_cast<uint32_t>(now - lastRadioPacketMs) > RADIO_TIMEOUT_MS) {
        droneConnected = false;
    }

    setStatusLed(droneConnected);

    if (telemetryDirty &&
        static_cast<uint32_t>(now - lastTelemetrySendMs) >=
            TELEMETRY_PERIOD_MS) {
        sendTelemetry();
    }
}
