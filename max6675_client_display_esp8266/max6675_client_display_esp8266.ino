#ifndef MOCK_TEST
#include "LGFX_Config_ESP8266.h"
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#else
#include "test_mock/dummy_esp8266_client.h"
#endif

// --- Screen Modes ---
enum ScreenMode {
    SCREEN_RELATIVE_BARS = 0,
    SCREEN_ABSOLUTE_BARS = 1,
    SCREEN_COMPOUND_GRAPH = 2,
    SCREEN_COUNT = 3
};

ScreenMode currentScreen = SCREEN_RELATIVE_BARS;

// --- Physical Button Configuration ---
#define BUTTON_PIN 0 // FLASH button on NodeMCU / D1 Mini (GPIO0), active LOW

// --- Hardware and Connection ---
#ifndef MOCK_TEST
LGFX tft;
WebSocketsClient webSocket;
#endif

// --- Telemetry Buffer ---
float temps[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
bool errors[4] = { true, true, true, true };

// --- Graph History (Slightly reduced for ESP8266 memory optimization) ---
static constexpr int GRAPH_WIDTH = 200; // Optimized from 280 to save ESP8266 RAM
static constexpr int GRAPH_HEIGHT = 160;
static constexpr int GRAPH_LEFT = 30;
static constexpr int GRAPH_TOP = 40;

float t1_history[GRAPH_WIDTH];
float t2_history[GRAPH_WIDTH];
float t3_history[GRAPH_WIDTH];
float t4_history[GRAPH_WIDTH];
int history_index = 0;
int total_history_points = 0;

// Colors matching our dark cyberpunk dashboard
#define COLOR_BG      0x0821 // Very dark slate blue
#define COLOR_CARD    0x0884 // Dark blue panel
#define COLOR_BORDER  0x1CAB // Cyan/Teal translucent border
#define COLOR_TEXT    0xEFFF // Light cyan
#define COLOR_MUTED   0x8E7B // Muted teal

#define COLOR_CYAN    0x07FF // T1
#define COLOR_MAG     0xF81F // T2
#define COLOR_YEL     0xFFE0 // T3
#define COLOR_RED     0xF800 // T4
#define COLOR_GRN     0x07E0

// --- WebSocket Event Handler ---
// Compiles on both host mock and device for full interoperability unit testing
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        // High optimization: Small static buffer on the stack to prevent heap fragmentation on ESP8266
        StaticJsonDocument<192> doc;
        DeserializationError err = deserializeJson(doc, payload, length);
        if (!err) {
            temps[0] = doc["t1"] | 0.0f;
            temps[1] = doc["t2"] | 0.0f;
            temps[2] = doc["t3"] | 0.0f;
            temps[3] = doc["t4"] | 0.0f;
            errors[0] = (doc["e1"] | 1) == 1;
            errors[1] = (doc["e2"] | 1) == 1;
            errors[2] = (doc["e3"] | 1) == 1;
            errors[3] = (doc["e4"] | 1) == 1;

            // Push to graph history
            t1_history[history_index] = errors[0] ? -1.0f : temps[0];
            t2_history[history_index] = errors[1] ? -1.0f : temps[1];
            t3_history[history_index] = errors[2] ? -1.0f : temps[2];
            t4_history[history_index] = errors[3] ? -1.0f : temps[3];

            history_index = (history_index + 1) % GRAPH_WIDTH;
            if (total_history_points < GRAPH_WIDTH) {
                total_history_points++;
            }
        }
    }
}

// --- EEPROM Preferences Save / Restore ---
void saveScreenMode(ScreenMode mode) {
#ifndef MOCK_TEST
    EEPROM.write(0, (uint8_t)mode);
    EEPROM.commit();
#else
    mock_eeprom_cells[0] = (uint8_t)mode;
#endif
    Serial.printf("EEPROM: Saved Screen Mode %d\n", (int)mode);
}

ScreenMode restoreScreenMode() {
    uint8_t stored = 0;
#ifndef MOCK_TEST
    stored = EEPROM.read(0);
#else
    stored = mock_eeprom_cells[0];
#endif
    if (stored >= SCREEN_COUNT) {
        stored = 0;
    }
    Serial.printf("EEPROM: Restored Screen Mode %d\n", (int)stored);
    return (ScreenMode)stored;
}

// --- Drawing Helpers ---
void drawPanelBorder(int x, int y, int w, int h) {
#ifndef MOCK_TEST
    tft.fillRect(x, y, w, h, COLOR_CARD);
    tft.drawRect(x, y, w, h, COLOR_BORDER);
#endif
}

void drawScreenHeader(const char* screenName) {
#ifndef MOCK_TEST
    tft.fillScreen(COLOR_BG);

    // Cyberpunk style line at top
    tft.fillRect(0, 0, 320, 4, COLOR_CYAN);

    tft.setTextColor(COLOR_TEXT);
    tft.setFont(&fonts::Font2);
    tft.setCursor(8, 10);
    tft.print("MAX6675 Client Display (ESP8266)");

    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(8, 25);
    tft.print("Mode: ");
    tft.setTextColor(COLOR_CYAN);
    tft.print(screenName);

    // Separator line
    tft.drawFastHLine(0, 38, 320, COLOR_BORDER);
#endif
}

// --- Screen 1: Relative Auto-Scaled Bars ---
void renderRelativeBars() {
    drawScreenHeader("Relative Auto-Scaled");

    // Compute min and max of non-error thermocouples
    float minTemp = 9999.0f;
    float maxTemp = -9999.0f;
    bool hasValid = false;

    for (int i = 0; i < 4; i++) {
        if (!errors[i]) {
            if (temps[i] < minTemp) minTemp = temps[i];
            if (temps[i] > maxTemp) maxTemp = temps[i];
            hasValid = true;
        }
    }

    if (!hasValid) {
        minTemp = 0.0f;
        maxTemp = 100.0f;
    }
    float tempRange = (maxTemp - minTemp);
    if (tempRange < 1.0f) tempRange = 1.0f;

#ifndef MOCK_TEST
    // Show scaling details
    tft.setFont(&fonts::Font0);
    tft.setTextColor(COLOR_MUTED);
    tft.setCursor(200, 25);
    tft.printf("R: %.1f-%.1f C", minTemp, maxTemp);

    // Draw four bars side to side
    int barW = 40;
    int gap = 30;
    int startX = 35;
    int baseLineY = 200;
    int maxBarH = 120;

    for (int i = 0; i < 4; i++) {
        int x = startX + i * (barW + gap);
        drawPanelBorder(x - 5, 55, barW + 10, 170);

        // Bar label T1-T4
        tft.setTextColor(COLOR_MUTED);
        tft.setFont(&fonts::Font2);
        tft.drawCentreString(String("T") + String(i + 1), x + barW/2, 212, 1);

        if (errors[i]) {
            tft.setTextColor(COLOR_RED);
            tft.drawCentreString("ERR", x + barW/2, 130, 2);
        } else {
            // Draw relative height bar
            float val = temps[i];
            float pct = (val - minTemp) / tempRange;
            int barH = (int)(pct * maxBarH);
            if (barH < 2) barH = 2;

            uint32_t colors[4] = { COLOR_CYAN, COLOR_MAG, COLOR_YEL, COLOR_RED };
            tft.fillRect(x, baseLineY - barH, barW, barH, colors[i]);

            // Draw text value above bar
            tft.setTextColor(COLOR_TEXT);
            tft.setFont(&fonts::Font0);
            tft.drawCentreString(String(val, 1), x + barW/2, baseLineY - barH - 12, 1);
        }
    }
#endif
}

// --- Screen 2: Fixed Absolute Bars ---
void renderAbsoluteBars() {
    drawScreenHeader("Absolute Range (0-1000C)");

#ifndef MOCK_TEST
    // Draw four bars side to side
    int barW = 40;
    int gap = 30;
    int startX = 35;
    int baseLineY = 200;
    int maxBarH = 120;

    for (int i = 0; i < 4; i++) {
        int x = startX + i * (barW + gap);
        drawPanelBorder(x - 5, 55, barW + 10, 170);

        // Bar label T1-T4
        tft.setTextColor(COLOR_MUTED);
        tft.setFont(&fonts::Font2);
        tft.drawCentreString(String("T") + String(i + 1), x + barW/2, 212, 1);

        if (errors[i]) {
            tft.setTextColor(COLOR_RED);
            tft.drawCentreString("ERR", x + barW/2, 130, 2);
        } else {
            // Draw absolute bar scaled strictly to 0-1000C range
            float val = temps[i];
            float pct = val / 1000.0f;
            if (pct > 1.0f) pct = 1.0f;
            if (pct < 0.0f) pct = 0.0f;

            int barH = (int)(pct * maxBarH);
            if (barH < 2) barH = 2;

            uint32_t colors[4] = { COLOR_CYAN, COLOR_MAG, COLOR_YEL, COLOR_RED };
            tft.fillRect(x, baseLineY - barH, barW, barH, colors[i]);

            // Draw text value above bar
            tft.setTextColor(COLOR_TEXT);
            tft.setFont(&fonts::Font0);
            tft.drawCentreString(String(val, 1), x + barW/2, baseLineY - barH - 12, 1);
        }
    }
#endif
}

// --- Screen 3: Compound Graph ---
void renderCompoundGraph() {
    drawScreenHeader("Compound Graph (200-800C)");

#ifndef MOCK_TEST
    drawPanelBorder(GRAPH_LEFT - 4, GRAPH_TOP - 4, GRAPH_WIDTH + 8, GRAPH_HEIGHT + 8);

    // Draw grid lines
    tft.setTextColor(COLOR_MUTED);
    tft.setFont(&fonts::Font0);

    for (int temp = 200; temp <= 800; temp += 150) {
        float pct = (temp - 200) / 600.0f;
        int y = GRAPH_TOP + GRAPH_HEIGHT - (int)(pct * GRAPH_HEIGHT);

        tft.drawFastHLine(GRAPH_LEFT, y, GRAPH_WIDTH, 0x1102); // Dark cyan grid line
        tft.setCursor(GRAPH_LEFT - 24, y - 4);
        tft.printf("%d", temp);
    }

    // Draw historical traces
    auto drawTrace = [](const float* history, uint32_t color) {
        if (total_history_points < 2) return;

        int prevX = -1, prevY = -1;
        for (int i = 0; i < total_history_points; i++) {
            int idx = (history_index - total_history_points + i + GRAPH_WIDTH) % GRAPH_WIDTH;
            float val = history[idx];
            if (val < 200.0f || val > 800.0f) {
                prevX = -1;
                prevY = -1;
                continue;
            }

            int x = GRAPH_LEFT + (int)(((float)i / (GRAPH_WIDTH - 1)) * GRAPH_WIDTH);
            float pct = (val - 200.0f) / 600.0f;
            int y = GRAPH_TOP + GRAPH_HEIGHT - (int)(pct * GRAPH_HEIGHT);

            if (prevX != -1 && prevY != -1) {
                tft.drawLine(prevX, prevY, x, y, color);
            }
            prevX = x;
            prevY = y;
        }
    };

    drawTrace(t1_history, COLOR_CYAN);
    drawTrace(t2_history, COLOR_MAG);
    drawTrace(t3_history, COLOR_YEL);
    drawTrace(t4_history, COLOR_RED);

    // Draw current values inside a legend block
    tft.setFont(&fonts::Font0);
    tft.setTextColor(COLOR_CYAN);
    tft.setCursor(GRAPH_LEFT, GRAPH_TOP + GRAPH_HEIGHT + 10);
    tft.printf("T1:%.1f", errors[0] ? 0.0f : temps[0]);

    tft.setTextColor(COLOR_MAG);
    tft.setCursor(GRAPH_LEFT + 70, GRAPH_TOP + GRAPH_HEIGHT + 10);
    tft.printf("T2:%.1f", errors[1] ? 0.0f : temps[1]);

    tft.setTextColor(COLOR_YEL);
    tft.setCursor(GRAPH_LEFT + 140, GRAPH_TOP + GRAPH_HEIGHT + 10);
    tft.printf("T3:%.1f", errors[2] ? 0.0f : temps[2]);

    tft.setTextColor(COLOR_RED);
    tft.setCursor(GRAPH_LEFT + 210, GRAPH_TOP + GRAPH_HEIGHT + 10);
    tft.printf("T4:%.1f", errors[3] ? 0.0f : temps[3]);
#endif
}

// --- Render Controller ---
void renderActiveScreen() {
    switch (currentScreen) {
        case SCREEN_RELATIVE_BARS:  renderRelativeBars(); break;
        case SCREEN_ABSOLUTE_BARS:  renderAbsoluteBars(); break;
        case SCREEN_COMPOUND_GRAPH: renderCompoundGraph(); break;
        default: break;
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);

#ifndef MOCK_TEST
    EEPROM.begin(32);
#endif

    // Restore Screen setting
    currentScreen = restoreScreenMode();

    pinMode(BUTTON_PIN, INPUT_PULLUP);

#ifndef MOCK_TEST
    tft.init();
    tft.setRotation(1); // Landscape mode
    tft.fillScreen(COLOR_BG);

    // Connect to AP
    WiFi.begin("MAX6675-Profiler", "password123");
    Serial.print("Connecting to MAX6675 AP...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Connected!");

    // Start Websocket client to read main telemetry stream
    webSocket.begin("192.168.4.1", 80, "/ws");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(2000);
#endif

    Serial.println("Display setup complete.");
}

// --- Loop ---
void loop() {
#ifndef MOCK_TEST
    webSocket.loop();
#endif

    // Non-blocking debounce-safe button detection to cycle screen modes
    static unsigned long lastDebounceTime = 0;
    static bool lastButtonState = HIGH;
    bool reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > 50) {
        static bool buttonPressedState = HIGH;
        if (reading != buttonPressedState) {
            buttonPressedState = reading;
            if (buttonPressedState == LOW) { // Button active LOW pressed
                // Toggle mode
                currentScreen = (ScreenMode)(((int)currentScreen + 1) % SCREEN_COUNT);
                saveScreenMode(currentScreen);

                // Trigger instant redraw
                renderActiveScreen();
            }
        }
    }
    lastButtonState = reading;

    // Periodic refresh
    static unsigned long lastDrawTime = 0;
    if (millis() - lastDrawTime >= 1000) {
        lastDrawTime = millis();
        renderActiveScreen();
    }
}
