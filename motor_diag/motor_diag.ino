// Motor driver diagnostic: holds IN3/IN4 at fixed digital levels so the
// wiring can be checked with a multimeter. ESP32-C6, DRV8833 channel B.
//
// Serial commands (115200 baud, single characters):
//   f  IN3=HIGH IN4=LOW   (full forward, held)
//   r  IN3=LOW  IN4=HIGH  (full reverse, held)
//   s  IN3=LOW  IN4=LOW   (coast)

const int IN3_PIN = 18;
const int IN4_PIN = 19;

void setPins(int in3, int in4, const char *label) {
  digitalWrite(IN3_PIN, in3);
  digitalWrite(IN4_PIN, in4);
  Serial.printf("%s  (IN3=%d IN4=%d)\n", label, in3, in4);
}

void setup() {
  Serial.begin(115200);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  setPins(LOW, LOW, "Coast");
  delay(2000);
  setPins(HIGH, LOW, "Forward, held - send f/r/s to change");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'f') setPins(HIGH, LOW, "Forward, held");
    else if (c == 'r') setPins(LOW, HIGH, "Reverse, held");
    else if (c == 's') setPins(LOW, LOW, "Coast");
  }
}
