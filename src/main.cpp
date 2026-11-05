/*
 * AFC Drone RF69 <-> Desktop Dashboard Relay
 *
 * The base station is a transparent relay. It does not decode telemetry or
 * commands; the dashboard (AFCDrone_BaseController) owns the RF message
 * layouts and talks to the drone through RADIO_PACKET envelopes.
 *
 * Radio side (must match AFC-Drone src/radio.cpp):
 *   - RadioHead RH_RF69, 915 MHz, GFSK_Rb250Fd250
 *   - 16-byte encryption key matching the drone
 *   - RadioHead ID header: packet number
 *   - RadioHead FLAGS header: message type
 *   - RF application payload: 8-byte Radio::Message
 *   - Beam break: when the IR beam is interrupted the relay itself transmits a
 *     RADIO_BEAM_BREAK packet: bytes 0-3 = millis() timestamp (uint32 LE),
 *     bytes 4-5 = trigger count (uint16 LE), bytes 6-7 = 0.
 *   - The drone pings with SETUP "AFCDrone" until it receives a SETUP packet
 *     carrying the ACK pattern (8 x 0x69); this relay answers that itself.
 *
 * USB side (must match AFCDrone_BaseController src/protocol.js):
 *   - Binary frame:
 *       A5 5A 01 <packetLo> <packetHi> <type> <length> <payload...> <crcLo> <crcHi>
 *   - CRC-16/CCITT-FALSE over packet number, type, length, and payload
 *     (magic and version excluded)
 *   - RAW (0): identify query 0x3F -> reply 0x02 (base station relay)
 *   - RADIO_PACKET (2):
 *       [0] direction (0 = received over RF, 1 = sent over RF)
 *       [1] RF packet number (headerId)
 *       [2] RF message type (headerFlags)
 *       [3..10] 8-byte RF message
 *       [11] downlink RSSI magnitude (received packets only)
 *     Dashboard -> relay RADIO_PACKET frames are transmit requests; the relay
 *     assigns its own RF packet number.
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
// On an Uno, RFM69_INT must be pin 2 or 3 (the only external interrupt pins).
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

// Set false to run without an RF69 attached: radio init, RX and TX are all
// skipped, and beam breaks are only reported to the dashboard as debug text.
static constexpr bool RADIO_ENABLED = false;

// IR beam break. The LED must be on pin 9 (Timer1 OC1A generates the 38 kHz
// carrier in hardware). The receiver (TSOP-style, active-low output) can be
// on any free pin.
static constexpr uint8_t IR_LED_PIN = 9;
static constexpr uint8_t IR_RX_PIN = 7;
// Indicator LED: on while the beam is intact, off while broken.
static constexpr uint8_t BEAM_LED_PIN = 6;

// The receiver (VS1838B/TSOP1838 type) suppresses steady or periodic carrier
// as interference, so the beam is a NEC-style frame instead: 9 ms leader burst,
// 4.5 ms space, 8 random data bits (562 us burst + 562/1687 us space), a stop
// burst, then a ~12 ms gap. Frames repeat about every 40 ms.
static constexpr uint16_t IR_LEADER_ON_US = 9000;
static constexpr uint16_t IR_LEADER_OFF_US = 4500;
static constexpr uint16_t IR_BIT_ON_US = 562;
static constexpr uint16_t IR_BIT_ZERO_OFF_US = 562;
static constexpr uint16_t IR_BIT_ONE_OFF_US = 1687;
static constexpr uint16_t IR_FRAME_GAP_US = 12000;
static constexpr uint8_t IR_FRAME_BITS = 8;
// leader (2) + bits (2 each) + stop burst + frame gap
static constexpr uint8_t IR_FRAME_STEPS = 2 + 2 * IR_FRAME_BITS + 2;
// Beam counts as broken when no burst has been seen for this long.
static constexpr uint16_t BEAM_BREAK_TIMEOUT_MS = 120;

static constexpr uint32_t RADIO_TIMEOUT_MS = 2000;

static const uint8_t RF69_ENCRYPTION_KEY[16] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
};

RH_RF69 radio(RFM69_CS, RFM69_INT);

// -----------------------------------------------------------------------------
// Radio protocol
// -----------------------------------------------------------------------------

// Only the values the relay itself acts on. Every other type is forwarded
// untouched in both directions.
static constexpr uint8_t RADIO_SETUP     = 0;
static constexpr uint8_t RADIO_HEARTBEAT = 10;
// Must match the type the drone expects for beam-break events. 20 is a
// placeholder; change it to the drone's value.
static constexpr uint8_t RADIO_BEAM_BREAK = 20;

static constexpr uint8_t RADIO_MESSAGE_SIZE = 8;

// Radio::ack on the drone.
static const uint8_t RADIO_LINK_ACK[RADIO_MESSAGE_SIZE] = {
    0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69
};

// -----------------------------------------------------------------------------
// Dashboard protocol
// -----------------------------------------------------------------------------

enum DashboardMessageType : uint8_t {
    DASHBOARD_RAW          = 0,
    DASHBOARD_DEBUG_TEXT   = 1,
    DASHBOARD_RADIO_PACKET = 2
};

static constexpr uint8_t DASHBOARD_MAGIC_0 = 0xA5;
static constexpr uint8_t DASHBOARD_MAGIC_1 = 0x5A;
static constexpr uint8_t DASHBOARD_VERSION = 0x01;
static constexpr uint8_t DASHBOARD_MAX_PAYLOAD = 64;

static constexpr uint8_t IDENTIFY_QUERY_BYTE = 0x3F;
static constexpr uint8_t DEVICE_KIND_BASE_STATION_RELAY = 0x02;

static constexpr uint8_t RADIO_DIRECTION_RECEIVED = 0;
static constexpr uint8_t RADIO_DIRECTION_SENT = 1;

// direction + packet number + type + message
static constexpr uint8_t RADIO_ENVELOPE_SIZE = 3 + RADIO_MESSAGE_SIZE;

static uint16_t dashboardTxPacketNumber = 0;
static uint8_t radioTxPacketNumber = 0;
static uint32_t lastRadioPacketMs = 0;
static bool droneConnected = false;
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

static uint8_t rssiMagnitudeFromDbm(int16_t rssiDbm) {
    // RH_RF69 reports a negative dBm number. The envelope's RSSI field is one
    // unsigned byte, so send its positive magnitude.
    int16_t magnitude = -rssiDbm;

    if (magnitude < 0) {
        magnitude = 0;
    } else if (magnitude > 255) {
        magnitude = 255;
    }

    return static_cast<uint8_t>(magnitude);
}

/**
 * Mirrors one RF packet to the dashboard as a RADIO_PACKET frame. Received
 * packets carry the downlink RSSI as a 12th byte; sent packets do not.
 */
static void sendRadioEnvelope(
    uint8_t direction,
    uint8_t packetNumber,
    uint8_t messageType,
    const uint8_t message[RADIO_MESSAGE_SIZE],
    bool includeRssi,
    uint8_t rssi
) {
    uint8_t envelope[RADIO_ENVELOPE_SIZE + 1];
    envelope[0] = direction;
    envelope[1] = packetNumber;
    envelope[2] = messageType;
    memcpy(&envelope[3], message, RADIO_MESSAGE_SIZE);
    envelope[RADIO_ENVELOPE_SIZE] = rssi;

    sendDashboardFrame(
        DASHBOARD_RADIO_PACKET,
        envelope,
        includeRssi ? RADIO_ENVELOPE_SIZE + 1 : RADIO_ENVELOPE_SIZE
    );
}

// -----------------------------------------------------------------------------
// RF transmit
// -----------------------------------------------------------------------------

static void restoreRadioReceiveMode() {
    radio.setModeRx();
}

static void transmitRadio(
    uint8_t messageType,
    const uint8_t message[RADIO_MESSAGE_SIZE]
) {
    // Match the drone protocol exactly: packet metadata lives in RadioHead's
    // ID and FLAGS headers, while send() receives only the 8-byte message.
    if (!RADIO_ENABLED) {
        return;
    }

    const uint8_t packetNumber = radioTxPacketNumber++;

    radio.setHeaderId(packetNumber);
    radio.setHeaderFlags(messageType);

    radio.send(message, RADIO_MESSAGE_SIZE);
    radio.waitPacketSent();
    restoreRadioReceiveMode();

    // Heartbeats go out every 25 ms; mirroring them would flood the USB link
    // and the dashboard's packet log for no diagnostic value.
    if (messageType != RADIO_HEARTBEAT) {
        sendRadioEnvelope(
            RADIO_DIRECTION_SENT,
            packetNumber,
            messageType,
            message,
            false,
            0
        );
    }
}

// -----------------------------------------------------------------------------
// RF receive
// -----------------------------------------------------------------------------

static void processRadio() {
    if (!RADIO_ENABLED || !radio.available()) {
        return;
    }

    uint8_t frame[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t length = sizeof(frame);

    if (!radio.recv(frame, &length)) {
        return;
    }

    // RadioHead removes its four transport headers before recv() returns.
    if (length != RADIO_MESSAGE_SIZE) {
        if (!reportedLengthMismatch) {
            sendDashboardText("RF payload length mismatch");
            reportedLengthMismatch = true;
        }
        return;
    }

    const uint8_t packetNumber = radio.headerId();
    const uint8_t messageType = radio.headerFlags();
    const uint8_t rssi = rssiMagnitudeFromDbm(radio.lastRssi());

    if (!droneConnected) {
        sendDashboardText("RF link up");
    }
    lastRadioPacketMs = millis();
    droneConnected = true;

    sendRadioEnvelope(
        RADIO_DIRECTION_RECEIVED,
        packetNumber,
        messageType,
        frame,
        true,
        rssi
    );

    // The drone keeps re-sending its SETUP ping until it sees the ACK, and
    // re-pings after a reboot, so answer every one rather than only the first.
    if (messageType == RADIO_SETUP &&
        memcmp(frame, RADIO_LINK_ACK, RADIO_MESSAGE_SIZE) != 0) {
        transmitRadio(RADIO_SETUP, RADIO_LINK_ACK);
    }
}

// -----------------------------------------------------------------------------
// Beam break
// -----------------------------------------------------------------------------

static bool beamIntact = false;
static bool irCarrierOn = false;
static uint16_t beamBreakCount = 0;
static uint32_t lastBurstToggleUs = 0;
static uint16_t currentPhaseUs = 0;
static uint8_t irStep = IR_FRAME_STEPS - 1;
static uint8_t irFrameByte = 0;
static uint32_t lastBeamSeenMs = 0;

static void setupBeamBreak() {
    randomSeed(analogRead(A0));

    pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, LOW);
    pinMode(IR_RX_PIN, INPUT_PULLUP);
    pinMode(BEAM_LED_PIN, OUTPUT);
    digitalWrite(BEAM_LED_PIN, LOW);

    // Timer1 CTC, no prescaler: toggle OC1A every 210 ticks -> ~38.1 kHz.
    // The output stays disconnected (COM1A0 = 0) until a burst starts.
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10);
    OCR1A = (F_CPU / (2UL * 38000UL)) - 1;
}

static void setIrCarrier(bool on) {
    if (on) {
        TCCR1A |= _BV(COM1A0);
    } else {
        TCCR1A &= ~_BV(COM1A0);
        digitalWrite(IR_LED_PIN, LOW);
    }
    irCarrierOn = on;
}

// Duration of the given step of the frame. Even steps are bursts, odd steps
// are spaces.
static uint16_t irStepDurationUs(uint8_t step) {
    if (step == 0) return IR_LEADER_ON_US;
    if (step == 1) return IR_LEADER_OFF_US;
    if (step == IR_FRAME_STEPS - 2) return IR_BIT_ON_US;
    if (step == IR_FRAME_STEPS - 1) return IR_FRAME_GAP_US;

    const uint8_t bit = (step - 2) / 2;
    if (step % 2 == 0) return IR_BIT_ON_US;
    return ((irFrameByte >> bit) & 1u) ? IR_BIT_ONE_OFF_US : IR_BIT_ZERO_OFF_US;
}

static void sendBeamBreak() {
    const uint32_t timestamp = millis();

    uint8_t message[RADIO_MESSAGE_SIZE] = {
        static_cast<uint8_t>(timestamp & 0xFFu),
        static_cast<uint8_t>((timestamp >> 8) & 0xFFu),
        static_cast<uint8_t>((timestamp >> 16) & 0xFFu),
        static_cast<uint8_t>((timestamp >> 24) & 0xFFu),
        static_cast<uint8_t>(beamBreakCount & 0xFFu),
        static_cast<uint8_t>((beamBreakCount >> 8) & 0xFFu),
        0,
        0
    };
    ++beamBreakCount;

    if (RADIO_ENABLED) {
        transmitRadio(RADIO_BEAM_BREAK, message);
    } else {
        sendDashboardText("Beam break (radio disabled)");
    }

    // The transmit blocks the loop and delays burst sampling; don't let that
    // look like a second break.
    lastBeamSeenMs = millis();
}

static void processBeamBreak() {
    const uint32_t nowUs = micros();
    if (nowUs - lastBurstToggleUs >= currentPhaseUs) {
        lastBurstToggleUs = nowUs;
        irStep = (irStep + 1) % IR_FRAME_STEPS;
        if (irStep == 0) {
            irFrameByte = random(256);
        }
        setIrCarrier(irStep % 2 == 0);
        currentPhaseUs = irStepDurationUs(irStep);
    }

    // Receiver output is active low while it sees the 38 kHz carrier.
    const uint32_t nowMs = millis();
    if (digitalRead(IR_RX_PIN) == LOW) {
        lastBeamSeenMs = nowMs;
        beamIntact = true;
    } else if (beamIntact && nowMs - lastBeamSeenMs > BEAM_BREAK_TIMEOUT_MS) {
        beamIntact = false;
        sendBeamBreak();
    }

    digitalWrite(BEAM_LED_PIN, beamIntact ? HIGH : LOW);
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
    switch (type) {
        case DASHBOARD_RAW:
            if (payloadLength == 1 && payload[0] == IDENTIFY_QUERY_BYTE) {
                const uint8_t reply = DEVICE_KIND_BASE_STATION_RELAY;
                sendDashboardFrame(DASHBOARD_RAW, &reply, 1);
            }
            break;

        case DASHBOARD_RADIO_PACKET:
            // Transmit request: [direction hint, packet placeholder, type,
            // 8-byte message]. The relay assigns its own packet number.
            if (payloadLength >= RADIO_ENVELOPE_SIZE) {
                transmitRadio(payload[2], &payload[3]);
            }
            break;

        default:
            break;
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

[[noreturn]] static void fatalError(const char* reason) {
    if (STATUS_LED_PIN >= 0) {
        pinMode(static_cast<uint8_t>(STATUS_LED_PIN), OUTPUT);
    }

    // Keep reporting the failure so a dashboard that connects late still sees
    // why the relay is silent, instead of a port that never answers.
    uint32_t lastReportMs = 0;

    while (true) {
        if (millis() - lastReportMs >= 1000) {
            sendDashboardText(reason);
            lastReportMs = millis();
        }

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

static void setupRadio();

void setup() {
    if (STATUS_LED_PIN >= 0) {
        pinMode(static_cast<uint8_t>(STATUS_LED_PIN), OUTPUT);
        setStatusLed(false);
    }

    Serial.begin(USB_BAUD);

    setupBeamBreak();

    if (RADIO_ENABLED) {
        setupRadio();
    }

    sendDashboardText(RADIO_ENABLED ? "RF69 relay ready"
                                    : "Relay ready (radio disabled)");
}

static void setupRadio() {
    resetRadioHardware();

    if (!radio.init()) {
        fatalError("RF69 init failed - check CS/RST wiring");
    }

    // The drone relies on RH_RF69::init()'s default profile, which is this one.
    if (!radio.setModemConfig(RH_RF69::GFSK_Rb250Fd250)) {
        fatalError("RF69 modem config failed");
    }

    if (!radio.setFrequency(RF69_FREQUENCY_MHZ)) {
        fatalError("RF69 frequency set failed");
    }

    radio.setEncryptionKey(const_cast<uint8_t*>(RF69_ENCRYPTION_KEY));
    radio.setTxPower(RF69_TX_POWER_DBM, RF69_HIGH_POWER);
    restoreRadioReceiveMode();
}

void loop() {
    processDashboardSerial();
    processRadio();
    processBeamBreak();

    if (droneConnected &&
        static_cast<uint32_t>(millis() - lastRadioPacketMs) > RADIO_TIMEOUT_MS) {
        droneConnected = false;
        sendDashboardText("RF link lost");
    }

    setStatusLed(droneConnected);
}
