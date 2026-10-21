// Single-channel test: reads one sensor output and prints its averaged
// voltage, a calibrated 0-100% reflectance and a WHITE/BLACK decision at
// 115200 baud.
//
// Target: ESP32-C6. SENSOR_PIN is a GPIO number and must be one of the C6's
// ADC pins, GPIO0-GPIO6. GPIO8 has no ADC on the C6 (it is a strapping pin).
//
// 11 dB attenuation gives a usable range of roughly 0.15-3.1 V. If the
// sensor output turns out to stay below ~700 mV, switch to ADC_0db
// (range ~0-750 mV) for about 4x finer resolution.
//
// Serial commands (single characters):
//   w  record the current reading as the WHITE reference
//   b  record the current reading as the BLACK reference
//   r  clear the calibration
// Calibration is saved in flash and survives resets.

#include <Preferences.h>

const uint8_t SENSOR_PIN = 0;

const uint16_t SAMPLES = 64;
const uint16_t PRINT_INTERVAL_MS = 50;
const uint16_t CAL_SAMPLES = 20;          // readings averaged per calibration point
const uint16_t MIN_USEFUL_SPAN_DMV = 200; // 20.0 mV between white and black

Preferences prefs;
int32_t whiteDmv = -1;  // calibration points, in tenths of a mV
int32_t blackDmv = -1;
bool isWhite = false;

// Averaged reading in tenths of a millivolt.
int32_t readDmv() {
  uint32_t sum = 0;
  for (uint16_t i = 0; i < SAMPLES; i++) {
    sum += analogReadMilliVolts(SENSOR_PIN);
  }
  return (int32_t)(sum * 10 / SAMPLES);
}

int32_t readCalibrationPoint() {
  int64_t sum = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    sum += readDmv();
    delay(10);
  }
  return (int32_t)(sum / CAL_SAMPLES);
}

bool isCalibrated() {
  return whiteDmv >= 0 && blackDmv >= 0 && abs(whiteDmv - blackDmv) >= MIN_USEFUL_SPAN_DMV;
}

void saveCalibration() {
  prefs.putInt("white", whiteDmv);
  prefs.putInt("black", blackDmv);
}

void printCalPoint(const char *name, int32_t dmv) {
  if (dmv < 0) {
    Serial.printf(" %s=unset", name);
  } else {
    Serial.printf(" %s=%ld.%ld mV", name, dmv / 10, dmv % 10);
  }
}

void printCalibration() {
  Serial.print("# CAL");
  printCalPoint("white", whiteDmv);
  printCalPoint("black", blackDmv);
  if (whiteDmv >= 0 && blackDmv >= 0 && !isCalibrated()) {
    Serial.printf("  (span under %u.%u mV: too small to tell apart)", MIN_USEFUL_SPAN_DMV / 10,
                  MIN_USEFUL_SPAN_DMV % 10);
  }
  Serial.println();
}

void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'w') {
      whiteDmv = readCalibrationPoint();
      saveCalibration();
      printCalibration();
    } else if (c == 'b') {
      blackDmv = readCalibrationPoint();
      saveCalibration();
      printCalibration();
    } else if (c == 'r') {
      whiteDmv = blackDmv = -1;
      saveCalibration();
      printCalibration();
    }
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  prefs.begin("irsensor", false);
  whiteDmv = prefs.getInt("white", -1);
  blackDmv = prefs.getInt("black", -1);

  Serial.printf("# IR sensor readout on GPIO%u\n", SENSOR_PIN);
  printCalibration();
}

void loop() {
  handleCommands();

  int32_t dmv = readDmv();

  if (isCalibrated()) {
    int32_t pct = (dmv - blackDmv) * 100 / (whiteDmv - blackDmv);
    pct = constrain(pct, 0, 100);
    // Hysteresis: switch only after crossing 10% past the midpoint.
    if (isWhite && pct < 40) isWhite = false;
    if (!isWhite && pct > 60) isWhite = true;
    Serial.printf("GPIO%u: %5ld.%ld mV | %3ld%% | %s\n", SENSOR_PIN, dmv / 10, dmv % 10, pct,
                  isWhite ? "WHITE" : "BLACK");
  } else {
    Serial.printf("GPIO%u: %5ld.%ld mV | ---%% | UNCAL\n", SENSOR_PIN, dmv / 10, dmv % 10);
  }

  delay(PRINT_INTERVAL_MS);
}
