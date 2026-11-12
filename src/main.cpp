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
 *   - Clock sync (pendulum timing test): the relay sends TIME_PING (15) every
 *     PING_INTERVAL_MS and the drone answers TIME_PONG (16). Both ends stamp
 *     micros() in the RFM69 DIO0 interrupt; the relay reports each completed
 *     exchange to the dashboard as a BASE_TIMESYNC frame. Neither is relayed
 *     as a RADIO_PACKET.
 *   - Pendulum pass detector: a TMAG5273 Hall sensor at the bottom of the
 *     swing, a magnet on the drone. Each pass of the magnet is reported as a
 *     BASE_MAG_PASS frame timed by this Uno's micros().
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
 *   - BASE_TIMESYNC (20), BASE_MAG_PASS (21): see sendTimeSyncFrame() and
 *     sendPassFrame(). All times are this Uno's micros(), uint32 LE.
 *
 * IMPORTANT:
 *   Do not print normal text to Serial. Any ASCII output will corrupt the
 *   dashboard's binary packet stream.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
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
// skipped, and so is clock sync. Passes are still detected and reported.
static constexpr bool RADIO_ENABLED = true;

// Clock sync with the drone. 5 Hz gives the offline fit plenty of points to
// follow the Uno's ceramic resonator, which can be off by thousands of ppm.
static constexpr uint16_t PING_INTERVAL_MS = 200;

// TMAG5273 pass detector, on the Uno's I2C pins (A4 = SDA, A5 = SCL) through a
// 3.3 V level shifter. The first address in this list that answers with TI's
// manufacturer ID is used: SparkFun's board is 0x22, other variants differ.
static const uint8_t MAG_ADDRESSES[] = {0x22, 0x35, 0x78, 0x44};

// Which axis to measure. Pick the one that points from the sensor to the
// magnet at the bottom of the swing: its field peaks there and is symmetric
// about the bottom, which is what the centroid timing relies on.
enum MagAxis : uint8_t { MAG_AXIS_X = 0, MAG_AXIS_Y = 1, MAG_AXIS_Z = 2 };
static constexpr MagAxis MAG_AXIS = MAG_AXIS_Z;

// The field must move this far from the resting baseline to start a pass, and
// fall back inside the exit level to end one. In microtesla; Earth's field is
// about 50 uT and a small neodymium magnet a couple of cm away gives tens of
// mT. Tune with the "MAG" debug lines, which report the live field.
static constexpr int32_t PASS_ENTER_UT = 2000;
static constexpr int32_t PASS_EXIT_UT = 1000;

// A pass longer than this is not a swing going by (magnet parked over the
// sensor, or a baseline that moved); it is dropped and the baseline re-learnt.
static constexpr uint16_t PASS_MAX_MS = 500;

// How often the live field is reported as debug text, for tuning.
static constexpr uint16_t MAG_REPORT_MS = 1000;

static constexpr uint32_t RADIO_TIMEOUT_MS = 2000;

static const uint8_t RF69_ENCRYPTION_KEY[16] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
};

/**
 * RH_RF69 with the moment of every DIO0 interrupt recorded.
 *
 * DIO0 rises on PacketSent while transmitting and on PayloadReady while
 * receiving, at a fixed point at the end of each packet. micros() taken there
 * is the clock-sync timestamp; the drone does exactly the same, so the radio's
 * own latencies cancel in the two-way exchange.
 */
class TimedRF69 : public RH_RF69 {
public:
    TimedRF69(uint8_t cs, uint8_t irq) : RH_RF69(cs, irq), irqPin(irq) { self = this; }

    /** Call after init(), which attached RadioHead's own handler. */
    void attachTimedInterrupt() {
        attachInterrupt(digitalPinToInterrupt(irqPin), timedIsr, RISING);
    }

    uint32_t lastRxUs() const { return rxUs; }
    uint32_t lastTxDoneUs() const { return txDoneUs; }

private:
    static void timedIsr() {
        const uint32_t now = micros();
        const RHMode before = self->_mode;
        self->handleInterrupt();
        if (before == RHModeTx && self->_mode != RHModeTx) {
            self->txDoneUs = now;
        } else if (before == RHModeRx && self->_rxBufValid) {
            self->rxUs = now;
        }
    }

    static TimedRF69* self;
    const uint8_t irqPin;
    volatile uint32_t rxUs = 0;
    volatile uint32_t txDoneUs = 0;
};

TimedRF69* TimedRF69::self = nullptr;

TimedRF69 radio(RFM69_CS, RFM69_INT);

// -----------------------------------------------------------------------------
// Radio protocol
// -----------------------------------------------------------------------------

// Only the values the relay itself acts on. Every other type is forwarded
// untouched in both directions.
static constexpr uint8_t RADIO_SETUP     = 0;
static constexpr uint8_t RADIO_HEARTBEAT = 10;
// Radio::MessageType::TIME_PING / TIME_PONG on the drone.
static constexpr uint8_t RADIO_TIME_PING = 15;
static constexpr uint8_t RADIO_TIME_PONG = 16;

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
    DASHBOARD_RADIO_PACKET = 2,
    // Pendulum test frames, base station only. Clear of the drone's own USB
    // types (0-10).
    DASHBOARD_BASE_TIMESYNC = 20,
    DASHBOARD_BASE_MAG_PASS = 21
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
    // and the dashboard's packet log for no diagnostic value. Pings are
    // reported through BASE_TIMESYNC instead.
    if (messageType != RADIO_HEARTBEAT && messageType != RADIO_TIME_PING) {
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
// Clock sync
// -----------------------------------------------------------------------------

// t1 of the recent pings, indexed by the low bits of their sequence number.
// A pong older than PING_SLOTS pings is too stale to be worth matching.
static constexpr uint8_t PING_SLOTS = 16;
static uint32_t pingT1[PING_SLOTS];
static uint16_t pingSeq[PING_SLOTS];
static bool pingValid[PING_SLOTS];
static uint16_t nextPingSeq = 0;
static uint32_t lastPingMs = 0;

static void writeU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

static void writeU32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

static uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

static void sendPing() {
    const uint16_t seq = nextPingSeq++;
    uint8_t message[RADIO_MESSAGE_SIZE] = {static_cast<uint8_t>(seq), 0, 0, 0, 0, 0, 0, 0};

    // transmitRadio() blocks until PacketSent, so the stamp is this ping's.
    transmitRadio(RADIO_TIME_PING, message);

    const uint8_t slot = seq % PING_SLOTS;
    pingT1[slot] = radio.lastTxDoneUs();
    pingSeq[slot] = seq;
    pingValid[slot] = true;
}

/**
 * BASE_TIMESYNC payload, 18 bytes:
 *   [0..1]   seq           uint16, full ping sequence number
 *   [2..5]   t1            relay micros() at the ping's PacketSent
 *   [6..9]   t2            drone micros() at the ping's PayloadReady
 *   [10..13] t4            relay micros() at the pong's PayloadReady
 *   [14]     prevSeq       low byte of an earlier ping's seq...
 *   [15..16] prevTxMinusRx ...whose pong left the drone this long after its
 *                          t2 (t3 - t2, us); 0xFFFF = unknown
 *   [17]     rssi          pong RSSI magnitude, dBm
 */
static void handleTimePong(const uint8_t* pong, uint32_t t4, uint8_t rssi) {
    const uint8_t seqLow = pong[0];
    const uint8_t slot = seqLow % PING_SLOTS;
    if (!pingValid[slot] || static_cast<uint8_t>(pingSeq[slot]) != seqLow) {
        return; // Answer to a ping we no longer remember
    }
    pingValid[slot] = false;

    uint8_t payload[18];
    writeU16(&payload[0], pingSeq[slot]);
    writeU32(&payload[2], pingT1[slot]);
    writeU32(&payload[6], readU32(&pong[2]));
    writeU32(&payload[10], t4);
    payload[14] = pong[1];
    payload[15] = pong[6];
    payload[16] = pong[7];
    payload[17] = rssi;

    sendDashboardFrame(DASHBOARD_BASE_TIMESYNC, payload, sizeof(payload));
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
    // Taken before recv(), which re-arms the receiver.
    const uint32_t rxUs = radio.lastRxUs();

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

    if (messageType == RADIO_TIME_PONG) {
        handleTimePong(frame, rxUs, rssi);
        return;
    }

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
// Pendulum pass detector (TMAG5273)
// -----------------------------------------------------------------------------

// TMAG5273 registers and fields (TI datasheet SLYS045).
static constexpr uint8_t TMAG_DEVICE_CONFIG_1 = 0x00;
static constexpr uint8_t TMAG_DEVICE_CONFIG_2 = 0x01;
static constexpr uint8_t TMAG_SENSOR_CONFIG_1 = 0x02;
static constexpr uint8_t TMAG_SENSOR_CONFIG_2 = 0x03;
static constexpr uint8_t TMAG_DEVICE_ID = 0x0D;
static constexpr uint8_t TMAG_MANUFACTURER_ID_LSB = 0x0E;
static constexpr uint8_t TMAG_X_MSB_RESULT = 0x12; // Y and Z follow, 2 bytes each

static constexpr uint8_t TMAG_MODE_CONTINUOUS = 0x02;
static constexpr uint8_t TMAG_WIDE_RANGE = 0x03;   // X_Y_RANGE and Z_RANGE: +-80 / +-266 mT
static constexpr uint16_t TI_MANUFACTURER_ID = 0x5449;

static uint8_t magAddress = 0;      // 0 = no sensor found
static int32_t magRangeUt = 0;      // Full scale of the configured range
static int32_t passEnterCounts = 0;
static int32_t passExitCounts = 0;

// Resting field, in counts x 16 for a smooth fixed-point average.
static int32_t baselineX16 = 0;
static bool baselineReady = false;
static uint16_t baselineSamples = 0;

struct Pass {
    bool active;
    uint32_t startUs;
    uint32_t lastUs;
    int32_t lastWeight;    // Weight of the previous sample, counts above exit level
    float area;            // Integral of weight dt
    float moment;          // Integral of weight * (t - startUs) dt
    uint32_t peakUs;
    int16_t peakRaw;
    int32_t peakDev;
    uint16_t samples;
    uint16_t maxGapUs;
};

static Pass pass = {};
static uint16_t passCount = 0;

// For the 1 Hz MAG debug line
static uint16_t samplesThisSecond = 0;
static uint32_t lastMagReportMs = 0;
static int16_t lastRaw = 0;

static bool tmagWrite(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(magAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool tmagRead(uint8_t address, uint8_t reg, uint8_t* out, uint8_t length) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(address, length) != length) return false;
    for (uint8_t i = 0; i < length; ++i) out[i] = static_cast<uint8_t>(Wire.read());
    return true;
}

static int32_t microteslaToCounts(int32_t ut) {
    return static_cast<int32_t>((static_cast<int64_t>(ut) * 32768) / magRangeUt);
}

static void setupMag() {
    Wire.begin();
    Wire.setClock(400000);
    // A sensor that stops answering must not hang the relay forever.
    Wire.setWireTimeout(3000, true);

    for (uint8_t i = 0; i < sizeof(MAG_ADDRESSES); ++i) {
        uint8_t id[2];
        if (tmagRead(MAG_ADDRESSES[i], TMAG_MANUFACTURER_ID_LSB, id, 2) &&
            (static_cast<uint16_t>(id[0]) | (static_cast<uint16_t>(id[1]) << 8)) == TI_MANUFACTURER_ID) {
            magAddress = MAG_ADDRESSES[i];
            break;
        }
    }

    if (magAddress == 0) {
        sendDashboardText("TMAG5273 not found - check I2C wiring/level shifter");
        return;
    }

    // DEVICE_ID bits 1:0: 1 = +-40/80 mT part, 2 = +-133/266 mT part. The wide
    // range of either is selected below.
    uint8_t version = 0;
    tmagRead(magAddress, TMAG_DEVICE_ID, &version, 1);
    magRangeUt = (version & 0x03) == 2 ? 266000L : 80000L;

    const bool ok =
        tmagWrite(TMAG_DEVICE_CONFIG_1, 0x00) &&                        // 1x averaging: fastest conversion
        tmagWrite(TMAG_SENSOR_CONFIG_2, TMAG_WIDE_RANGE) &&
        tmagWrite(TMAG_SENSOR_CONFIG_1, static_cast<uint8_t>(
                      (MAG_AXIS == MAG_AXIS_X ? 0x1 : MAG_AXIS == MAG_AXIS_Y ? 0x2 : 0x4) << 4)) &&
        tmagWrite(TMAG_DEVICE_CONFIG_2, TMAG_MODE_CONTINUOUS);
    if (!ok) {
        magAddress = 0;
        sendDashboardText("TMAG5273 config write failed");
        return;
    }

    passEnterCounts = microteslaToCounts(PASS_ENTER_UT);
    passExitCounts = microteslaToCounts(PASS_EXIT_UT);

    char text[48];
    snprintf(text, sizeof(text), "TMAG5273 at 0x%02X, +-%ld mT, axis %c",
             magAddress, static_cast<long>(magRangeUt / 1000), "XYZ"[MAG_AXIS]);
    sendDashboardText(text);
}

/**
 * BASE_MAG_PASS payload, 26 bytes:
 *   [0..1]   index       uint16, +1 per pass
 *   [2..5]   centroidUs  field-weighted mean time of the pass: the bottom of
 *                        the swing, for a field symmetric about it
 *   [6..9]   startUs     first sample past the enter level
 *   [10..13] endUs       first sample back inside the exit level
 *   [14..17] peakUs      sample with the largest deviation
 *   [18..19] peakRaw     int16 raw reading at peakUs
 *   [20..21] baselineRaw int16 resting reading
 *   [22..23] samples     uint16 samples in the pass
 *   [24..25] maxGapUs    uint16 longest time between two of them
 * Field in mT = raw * range / 32768; range is in the TMAG5273 debug line.
 */
static void sendPassFrame() {
    const uint32_t centroidUs = pass.startUs + static_cast<uint32_t>(pass.moment / pass.area + 0.5f);

    uint8_t payload[26];
    writeU16(&payload[0], passCount);
    writeU32(&payload[2], centroidUs);
    writeU32(&payload[6], pass.startUs);
    writeU32(&payload[10], pass.lastUs);
    writeU32(&payload[14], pass.peakUs);
    writeU16(&payload[18], static_cast<uint16_t>(pass.peakRaw));
    writeU16(&payload[20], static_cast<uint16_t>(baselineX16 / 16));
    writeU16(&payload[22], pass.samples);
    writeU16(&payload[24], pass.maxGapUs);

    sendDashboardFrame(DASHBOARD_BASE_MAG_PASS, payload, sizeof(payload));
    ++passCount;
}

/**
 * Take one field sample and run the pass detector on it.
 *
 * The bottom of the swing is found as the centroid of the field above the
 * exit level, integrated with the trapezoid rule over the actual sample times.
 * That holds up to uneven sampling (a radio transmit pauses the loop for a
 * millisecond or so) and does not depend on the threshold, as long as the
 * field is symmetric about the bottom.
 */
static void processMag() {
    if (magAddress == 0) return;

    uint8_t bytes[2];
    // Stamp the middle of the read: the conversion it returns finished just
    // before it, a fixed latency the same on every pass.
    const uint32_t before = micros();
    if (!tmagRead(magAddress, TMAG_X_MSB_RESULT + 2 * MAG_AXIS, bytes, 2)) return;
    const uint32_t nowUs = before + (micros() - before) / 2;

    const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
    lastRaw = raw;
    ++samplesThisSecond;

    // Learn the resting field before detecting anything.
    if (!baselineReady) {
        baselineX16 = baselineSamples == 0 ? static_cast<int32_t>(raw) * 16
                                           : baselineX16 + (static_cast<int32_t>(raw) * 16 - baselineX16) / 8;
        if (++baselineSamples >= 256) baselineReady = true;
        return;
    }

    int32_t deviation = static_cast<int32_t>(raw) - baselineX16 / 16;
    if (deviation < 0) deviation = -deviation;

    if (!pass.active) {
        if (deviation >= passEnterCounts) {
            pass = Pass{};
            pass.active = true;
            pass.startUs = nowUs;
            pass.lastUs = nowUs;
            pass.lastWeight = deviation - passExitCounts;
            pass.peakUs = nowUs;
            pass.peakRaw = raw;
            pass.peakDev = deviation;
            pass.samples = 1;
        } else {
            // Track slow drift (temperature, the drone parked off-centre)
            // only while nothing is passing.
            baselineX16 += (static_cast<int32_t>(raw) * 16 - baselineX16) / 256;
        }
        return;
    }

    const uint32_t dt = nowUs - pass.lastUs;
    const int32_t weight = deviation > passExitCounts ? deviation - passExitCounts : 0;
    const float tPrev = static_cast<float>(pass.lastUs - pass.startUs);
    const float tNow = static_cast<float>(nowUs - pass.startUs);
    pass.area += 0.5f * (pass.lastWeight + weight) * dt;
    pass.moment += 0.5f * (pass.lastWeight * tPrev + weight * tNow) * dt;
    if (dt > pass.maxGapUs) pass.maxGapUs = dt > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(dt);
    pass.lastUs = nowUs;
    pass.lastWeight = weight;
    ++pass.samples;

    if (deviation > pass.peakDev) {
        pass.peakDev = deviation;
        pass.peakRaw = raw;
        pass.peakUs = nowUs;
    }

    if (deviation <= passExitCounts) {
        pass.active = false;
        if (pass.area > 0.0f) sendPassFrame();
    } else if (nowUs - pass.startUs > static_cast<uint32_t>(PASS_MAX_MS) * 1000UL) {
        pass.active = false;
        baselineReady = false;
        baselineSamples = 0;
        sendDashboardText("MAG pass too long - relearning baseline");
    }
}

static void reportMag() {
    if (magAddress == 0) return;
    const uint32_t nowMs = millis();
    if (nowMs - lastMagReportMs < MAG_REPORT_MS) return;

    const int32_t fieldUt = static_cast<int32_t>((static_cast<int64_t>(lastRaw) * magRangeUt) / 32768);
    const int32_t baseUt = static_cast<int32_t>((static_cast<int64_t>(baselineX16 / 16) * magRangeUt) / 32768);
    char text[64];
    snprintf(text, sizeof(text), "MAG %ld uT base %ld uT %u Hz passes %u",
             static_cast<long>(fieldUt), static_cast<long>(baseUt),
             static_cast<unsigned>(samplesThisSecond * 1000UL / (nowMs - lastMagReportMs)),
             passCount);
    sendDashboardText(text);

    samplesThisSecond = 0;
    lastMagReportMs = nowMs;
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

    if (RADIO_ENABLED) {
        setupRadio();
    }

    setupMag();

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
    radio.attachTimedInterrupt();
    restoreRadioReceiveMode();
}

void loop() {
    processDashboardSerial();
    processRadio();
    processMag();
    reportMag();

    // Hold pings while a pass is being sampled: a transmit stalls the loop for
    // a millisecond or so, which would leave a hole in the pass.
    if (RADIO_ENABLED && droneConnected && !pass.active &&
        static_cast<uint32_t>(millis() - lastPingMs) >= PING_INTERVAL_MS) {
        lastPingMs = millis();
        sendPing();
    }

    if (droneConnected &&
        static_cast<uint32_t>(millis() - lastRadioPacketMs) > RADIO_TIMEOUT_MS) {
        droneConnected = false;
        sendDashboardText("RF link lost");
    }

    setStatusLed(droneConnected);
}
