// Single motor test: DRV8833 channel B (IN3/IN4 -> OUT3/OUT4)
// ESP32-C6-DevKitM-1, arduino-esp32 3.x

const int IN3_PIN   = 18;
const int IN4_PIN   = 19;
const int SLEEP_PIN = -1;      // set to the GPIO wired to the driver's sleep pin,
                               // or leave -1 if it is tied to 3.3 V on the breadboard

const int PWM_FREQ = 20000;    // 20 kHz, above audible range
const int PWM_BITS = 8;        // duty 0..255

// speed: -255 (full reverse) to +255 (full forward), 0 = coast
void setMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    ledcWrite(IN3_PIN, speed);
    ledcWrite(IN4_PIN, 0);
  } else if (speed < 0) {
    ledcWrite(IN3_PIN, 0);
    ledcWrite(IN4_PIN, -speed);
  } else {
    ledcWrite(IN3_PIN, 0);
    ledcWrite(IN4_PIN, 0);
  }
}

void setup() {
  Serial.begin(115200);

  if (SLEEP_PIN >= 0) {
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(SLEEP_PIN, HIGH);   // wake the driver
  }

  ledcAttach(IN3_PIN, PWM_FREQ, PWM_BITS);
  ledcAttach(IN4_PIN, PWM_FREQ, PWM_BITS);
  setMotor(0);

  Serial.println("Motor test starting in 3 seconds...");
  delay(3000);
}

void loop() {
  Serial.println("Forward 50%");
  setMotor(128);
  delay(2000);

  Serial.println("Stop");
  setMotor(0);
  delay(1500);

  Serial.println("Reverse 50%");
  setMotor(-128);
  delay(2000);

  Serial.println("Stop");
  setMotor(0);
  delay(1500);

  Serial.println("Forward 100%");
  setMotor(255);
  delay(2000);

  Serial.println("Stop");
  setMotor(0);
  delay(3000);
}
