// Streams the four sensor channels for tools/sensor_web.py at ~20 Hz as
//   G2=1408 G3=3145 G4=1860 G5=2280
// (GPIO number = averaged millivolts), at 115200 baud.
//
// Target: ESP32-C6. VOUT1-VOUT4 are on GPIO2-GPIO5 (wiring as of 2026-10-29).
// Channels are labelled by GPIO here and mapped to VOUT names in the web tool.

const uint8_t NUM_PINS = 4;
const uint8_t PINS[NUM_PINS] = {2, 3, 4, 5};

const uint8_t SAMPLES = 32;
const uint16_t PRINT_INTERVAL_MS = 50;

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

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  for (uint8_t pin : PINS) {
    // 11 dB attenuation gives a usable range of roughly 0.15-3.1 V.
    analogSetPinAttenuation(pin, ADC_11db);
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    Serial.printf("%sG%u=%lu", i ? " " : "", PINS[i], (unsigned long)readAveragedMillivolts(PINS[i]));
  }
  Serial.println();
  delay(PRINT_INTERVAL_MS);
}
