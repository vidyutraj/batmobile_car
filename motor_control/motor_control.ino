// Serial-controlled motor: DRV8833 channel B (IN3/IN4 -> OUT3/OUT4).
// Driven by tools/motor_web.py, but any serial monitor works.
//
// Target: ESP32-C6-DevKitM-1, arduino-esp32 3.x, USB CDC On Boot enabled.
//
// Commands, one per line at 115200 baud:
//   M <n>   set speed, -255 (full reverse) .. 255 (full forward); 0 = coast.
//           Non-zero speeds are scaled onto MIN_DUTY..255 so every one turns the motor.
//   R <n>   set raw PWM duty, -255 .. 255, bypassing the scaling (for calibration)
//   B       brake (both inputs high, motor terminals shorted)
//   ?       report the current state
// Every command is answered with a state line:
//   STATE speed=<n> duty=<n> mode=<forward|reverse|coast|brake>

const int IN3_PIN   = 18;
const int IN4_PIN   = 19;
const int SLEEP_PIN = -1;      // GPIO wired to the driver's sleep pin, or -1 if tied to 3.3 V

const int PWM_FREQ = 20000;    // 20 kHz, above audible range
const int PWM_BITS = 8;        // duty 0..255

// Lowest duty that starts the motor from rest: 66% (168/255) at 6 V,
// measured 2026-10-16. Re-measure with the raw R command if the supply changes.
const int MIN_DUTY = 168;

int curSpeed = 0;
int curDuty = 0;
bool braking = false;
String line;

void reportState() {
  const char *mode = braking ? "brake" : curDuty > 0 ? "forward" : curDuty < 0 ? "reverse" : "coast";
  Serial.printf("STATE speed=%d duty=%d mode=%s\n", curSpeed, curDuty, mode);
}

// duty: -255 (full reverse) to +255 (full forward), 0 = coast. No scaling.
void setDuty(int duty) {
  duty = constrain(duty, -255, 255);
  ledcWrite(IN3_PIN, duty > 0 ? duty : 0);
  ledcWrite(IN4_PIN, duty < 0 ? -duty : 0);
  curDuty = duty;
  braking = false;
}

// speed: -255 (full reverse) to +255 (full forward), 0 = coast.
// Speeds 1..255 map onto MIN_DUTY..255 so any non-zero speed turns the motor.
void setMotor(int speed) {
  speed = constrain(speed, -255, 255);
  int duty = speed == 0 ? 0 : map(abs(speed), 1, 255, MIN_DUTY, 255);
  setDuty(speed < 0 ? -duty : duty);
  curSpeed = speed;
}

void brake() {
  ledcWrite(IN3_PIN, 255);
  ledcWrite(IN4_PIN, 255);
  curSpeed = curDuty = 0;
  braking = true;
}

void handleLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd[0]);
  if (c == 'M') {
    setMotor(cmd.substring(1).toInt());
  } else if (c == 'R') {
    setDuty(cmd.substring(1).toInt());
    curSpeed = 0;   // raw duty has no calibrated speed
  } else if (c == 'B') {
    brake();
  } else if (c != '?') {
    Serial.printf("ERR unknown command: %s\n", cmd.c_str());
    return;
  }
  reportState();
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
  Serial.println("READY motor_control");
  reportState();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleLine(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
}
