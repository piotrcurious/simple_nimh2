#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static const char* AP_SSID = "ADC_RX_AP";
static const char* AP_PASS = "12345678";   // at least 8 chars

static constexpr uint16_t UDP_PORT = 12345;
static constexpr uint8_t  ADC_CH_COUNT = 6;

struct __attribute__((packed)) UdpPacket {
    uint32_t index;
    uint16_t ch[ADC_CH_COUNT];
};

WiFiUDP udp;

static uint32_t last_packet_index = 0;
static bool have_last_index = false;

static uint16_t latest_values[ADC_CH_COUNT] = {0};
static uint32_t packets_received = 0;
static uint32_t packets_lost = 0;
static uint32_t packets_bad_size = 0;

static void printState(const UdpPacket& pkt) {
    Serial.printf("idx=%lu  ", (unsigned long)pkt.index);
    for (int i = 0; i < ADC_CH_COUNT; ++i) {
        Serial.printf("ch%d=%u ", i, (unsigned)pkt.ch[i]);
    }
    Serial.printf("\n");
}

void setup() {
    Serial.begin(115200);
    delay(200);

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    bool ok = WiFi.softAP(AP_SSID, AP_PASS);
    if (!ok) {
        Serial.println("Failed to start SoftAP");
        while (true) {
            delay(1000);
        }
    }

    IPAddress apIP = WiFi.softAPIP();
    Serial.print("SoftAP IP: ");
    Serial.println(apIP);

    udp.begin(UDP_PORT);
    Serial.printf("Listening on UDP port %u\n", UDP_PORT);
    Serial.println("Expected packet format: uint32_t index + 6x uint16_t channels");
}

void loop() {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) {
        delay(1);
        return;
    }

    if (packetSize != (int)sizeof(UdpPacket)) {
        packets_bad_size++;
        while (udp.available()) {
            udp.read();
        }
        Serial.printf("Bad packet size: %d (expected %u)\n",
                      packetSize, (unsigned)sizeof(UdpPacket));
        return;
    }

    UdpPacket pkt;
    int got = udp.read((uint8_t*)&pkt, sizeof(pkt));
    if (got != (int)sizeof(pkt)) {
        packets_bad_size++;
        Serial.printf("Short read: %d\n", got);
        return;
    }

    packets_received++;

    if (have_last_index) {
        uint32_t expected = last_packet_index + 1;
        if (pkt.index != expected) {
            if (pkt.index > expected) {
                packets_lost += (pkt.index - expected);
            }
        }
    } else {
        have_last_index = true;
    }

    last_packet_index = pkt.index;

    for (int i = 0; i < ADC_CH_COUNT; ++i) {
        latest_values[i] = pkt.ch[i];
    }

    printState(pkt);

    if ((packets_received % 50) == 0) {
        Serial.printf("recv=%lu lost=%lu bad=%lu\n",
                      (unsigned long)packets_received,
                      (unsigned long)packets_lost,
                      (unsigned long)packets_bad_size);
    }
}
