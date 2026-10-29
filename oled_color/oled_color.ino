// Shows WHITE or BLACK on an SSD1306 OLED for the surface under the IR array.
// No motor control; the motor driver inputs are held low (coast).
//
// Target: ESP32-C6-DevKitM-1, arduino-esp32 3.x, USB CDC On Boot enabled.
// OLED: 128x64 SSD1306 at I2C 0x3C, SDA on GPIO23, SCL on GPIO22 (the C6 defaults).
// Sensors: VOUT3 on GPIO4, VOUT4 on GPIO5, with line_motor's thresholds and
// hysteresis. BLACK if either sensor sees black. VOUT1/VOUT2 are not used
// until their hardware is checked.
//
// Also prints "<VOUT3 mV> <VOUT4 mV> <WHITE|BLACK>" every 200 ms at 115200 baud.

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int SCREEN_W = 128;
const int SCREEN_H = 64;
const uint8_t OLED_ADDR = 0x3C;

const int NUM_SENSORS = 2;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {4, 5};              // VOUT3, VOUT4
const char *SENSOR_NAMES[NUM_SENSORS] = {"V3", "V4"};   // short to fit the 21-character bottom line
const int THRESHOLD_MV[NUM_SENSORS] = {1500, 1650};           // from the 2026-10-29 tape recordings
const int HYSTERESIS_MV = 50;
const uint8_t SAMPLES = 32;

const int IN3_PIN = 18;
const int IN4_PIN = 19;

const uint16_t UPDATE_INTERVAL_MS = 200;

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool displayOk = false;
uint32_t readingMv[NUM_SENSORS];
bool black[NUM_SENSORS];
uint32_t lastUpdate = 0;

// The first read after switching channels is discarded: the ADC's sample
// capacitor still holds charge from the previous channel.
uint32_t readAveragedMillivolts(uint8_t pin) {
  analogReadMilliVolts(pin);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    sum += analogReadMilliVolts(pin);
  }
  return sum / SAMPLES;
}

void showSurface(bool isBlack) {
  if (!displayOk) return;
  display.clearDisplay();

  // The word, centred in the top 48 px.
  const char *word = isBlack ? "BLACK" : "WHITE";
  display.setTextSize(3);
  int16_t x, y;
  uint16_t w, h;
  display.getTextBounds(word, 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_W - w) / 2, (48 - h) / 2);
  display.print(word);

  // Raw readings along the bottom, '*' marking a sensor that sees black.
  display.setTextSize(1);
  display.setCursor(0, SCREEN_H - 8);
  for (int i = 0; i < NUM_SENSORS; i++) {
    display.printf("%s%s %lu  ", black[i] ? "*" : "", SENSOR_NAMES[i], (unsigned long)readingMv[i]);
  }
  display.display();
}

void setup() {
  Serial.begin(115200);

  // Keep the motor still: both driver inputs low is coast.
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  analogReadResolution(12);
  for (int i = 0; i < NUM_SENSORS; i++) {
    // 11 dB attenuation gives a usable range of roughly 0.15-3.1 V.
    analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
    black[i] = false;
  }

  Wire.begin();   // SDA = GPIO23, SCL = GPIO22
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (displayOk) {
    display.setTextColor(SSD1306_WHITE);
    Serial.println("READY oled_color");
  } else {
    Serial.println("ERR OLED not found at 0x3C: check SDA=GPIO23, SCL=GPIO22, 3V3, GND");
  }
}

void loop() {
  bool anyBlack = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    readingMv[i] = readAveragedMillivolts(SENSOR_PINS[i]);
    if ((int)readingMv[i] < THRESHOLD_MV[i] - HYSTERESIS_MV) black[i] = true;
    else if ((int)readingMv[i] > THRESHOLD_MV[i] + HYSTERESIS_MV) black[i] = false;
    anyBlack |= black[i];
  }

  if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
    lastUpdate = millis();
    showSurface(anyBlack);
    Serial.printf("%lu %lu %s\n", (unsigned long)readingMv[0], (unsigned long)readingMv[1],
                  anyBlack ? "BLACK" : "WHITE");
  }
}
