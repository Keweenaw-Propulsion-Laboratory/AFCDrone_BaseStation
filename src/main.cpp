#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>

// Match these to your wiring map
#define RFM69_CS   4
#define RFM69_INT  3
#define RFM69_RST  2

// Set your radio frequency (Must match your hardware: 433.0 or 915.0)
#define RF69_FREQ 915.0

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void send();

struct __attribute__((packed)) Command_t {
    struct __attribute__((packed)) flags {
        uint8_t targSlot : 1; /**The slot to be configured */
        uint8_t activeSlot : 1; /** The slot to be currently active */
        uint8_t empty : 6; // 6 Unused flags
    } flags;
    int16_t gimbalX;
    int16_t gimbalY;
    uint8_t motor0Speed;
    uint8_t motor1Speed;
    uint8_t empty0; // Unused command field

};


void setup() {
    Serial.begin(115200);
    while (!Serial); 
    
    // Hard reset the radio module (Adafruit breakout specific)
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    // Manual reset sequence
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    // Initialize the radio
    if (!rf69.init()) {
        Serial.println("RFM69 radio init failed! Check wiring.");
        while (1);
    }
    Serial.println("RFM69 radio init OK!");

    // Set frequency
    if (!rf69.setFrequency(RF69_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }

    // The RFM69HCW is a high-power module. We must set it explicitly.
    // Range is 14-20dBm. 20dBm is max power.
    rf69.setTxPower(20, true);

    // Encryption key must match the receiver (16 bytes exactly)
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    Serial.println("Setup complete! Radio is ready.");
}

bool connected = false;

void loop() {
    digitalWrite(13,connected);

    if (true) {
        send();
        delay(1000);
        return;
    }

    // Check if a radio packet has arrived
    if (rf69.available()) {
        connected = true;
        // Create a buffer to hold the incoming data
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        // Fetch the packet from the radio hardware
        if (rf69.recv(buf, &len)) {
            if (!len) return; // Ignore empty packets
            
            // Null-terminate the buffer so we can print it safely as text
            buf[len] = 0; 

            // Print the data out to the Serial Monitor
            Serial.println("--- New Packet Received ---");
            Serial.print("Data (Text):   ");
            Serial.println((char*)buf);
            
            Serial.print("Data (Hex):    ");
            for (int i = 0; i < len; i++) {
                Serial.print("0x");
                if (buf[i] < 0x10) Serial.print("0");
                Serial.print(buf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            // RSSI gives you the signal strength. Closer to 0 is stronger.
            // (e.g., -30dBm is great, -100dBm is barely hanging on)
            Serial.print("Signal (RSSI): ");
            Serial.print(rf69.lastRssi(), DEC);
            Serial.println(" dBm\n");

            uint8_t ack[8] = {0x69,0x69,0x69,0x69,0x69,0x69,0x69,0x69};
            rf69.send(ack, 8);
        } else {
            Serial.println("Receive failed");
        }
    }

    delay(500);
}

void send() {
    Serial.println("Sending packet...");
    
    // char radiopacket[] = "Hello from Uno!";

    Command_t cmd;

    cmd.flags.activeSlot = 0;
    cmd.flags.targSlot = 0;
    cmd.gimbalX = 0;
    cmd.gimbalY = 0;
    cmd.motor0Speed = 50;
    cmd.motor1Speed = 1;

    uint8_t frame[sizeof(Command_t)];

    memcpy(&frame, &cmd, sizeof(Command_t));

    static int counter = 0;

    rf69.setHeaderId(counter++);
    rf69.setHeaderFlags(8);

    rf69.send(frame, sizeof(Command_t));
    rf69.waitPacketSent();
    
    Serial.println("Packet sent successfully.");
}