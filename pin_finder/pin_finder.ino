// Pin finder for the ESP32-C6: prints the voltage on every ADC pin
// (GPIO0-GPIO6) and the digital level of the other header pins, so a
// jumper to 3V3 shows up as the pin it is really connected to.
//
// GPIO12/13 (USB), GPIO16/17 (UART0) and GPIO24-30 (flash) are skipped:
// reconfiguring them would cut the serial link or crash the chip.

const uint8_t ADC_PINS[] = {0, 1, 2, 3, 4, 5, 6};
const uint8_t DIGITAL_PINS[] = {7, 8, 9, 10, 11, 14, 15, 18, 19, 20, 21, 22, 23};

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  for (uint8_t pin : ADC_PINS) {
    analogSetPinAttenuation(pin, ADC_11db);
  }
  for (uint8_t pin : DIGITAL_PINS) {
    pinMode(pin, INPUT_PULLDOWN);
  }
}

void loop() {
  for (uint8_t pin : ADC_PINS) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) {
      sum += analogReadMilliVolts(pin);
    }
    Serial.printf("G%u=%4lu  ", pin, (unsigned long)(sum / 16));
  }
  Serial.print("| HIGH:");
  for (uint8_t pin : DIGITAL_PINS) {
    if (digitalRead(pin)) {
      Serial.printf(" G%u", pin);
    }
  }
  Serial.println();
  delay(200);
}
