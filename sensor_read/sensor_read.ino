// Reads the 4-channel IR reflectance sensor (OUT1-OUT4 on J2) and prints
// each channel's averaged voltage in millivolts at 115200 baud.
//
// Wiring (sensor J2 -> ESP32):
//   OUT1 -> GPIO0    OUT2 -> GPIO1    OUT3 -> GPIO2    OUT4 -> GPIO3
//   GND  -> GND (required: without a shared ground every reading is noise)
//   3V3  -> 3V3
//
// Target: ESP32-C6, whose only ADC is on GPIO0-GPIO6. GPIO4 and GPIO5 are
// avoided because they are strapping pins sampled at reset.

const uint8_t SENSOR_PINS[4] = {0, 1, 2, 3};
const uint8_t NUM_SENSORS = 4;

const uint8_t SAMPLES_PER_CHANNEL = 16;
const uint16_t PRINT_INTERVAL_MS = 100;

// Averages several calibrated readings from one pin. The first read after
// switching channels is discarded: the ADC's sample capacitor still holds
// charge from the previous channel, which skews it when the sensor output
// has high source impedance.
uint32_t readAveragedMillivolts(uint8_t pin) {
  analogReadMilliVolts(pin);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < SAMPLES_PER_CHANNEL; i++) {
    sum += analogReadMilliVolts(pin);
  }
  return sum / SAMPLES_PER_CHANNEL;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    // 11 dB attenuation gives a usable range of roughly 0.15-3.1 V.
    analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
  }

  Serial.println("IR sensor readout (mV)");
}

void loop() {
  uint32_t mv[NUM_SENSORS];
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    mv[i] = readAveragedMillivolts(SENSOR_PINS[i]);
  }

  char line[96];
  snprintf(line, sizeof(line), "OUT1: %4lu mV | OUT2: %4lu mV | OUT3: %4lu mV | OUT4: %4lu mV",
           (unsigned long)mv[0], (unsigned long)mv[1],
           (unsigned long)mv[2], (unsigned long)mv[3]);
  Serial.println(line);

  delay(PRINT_INTERVAL_MS);
}
