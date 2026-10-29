// Sensor-driven motor: the IR array decides how fast the motor runs.
// Driven by tools/line_web.py, but any serial monitor works.
//
// Target: ESP32-C6-DevKitM-1, arduino-esp32 3.x, USB CDC On Boot enabled.
// Sensors: VOUT1-VOUT4 on GPIO2, GPIO3, GPIO4, GPIO5 (wiring as of 2026-10-29).
// Motor: DRV8833 channel B (IN3/IN4 -> OUT3/OUT4), as in motor_control.
// OLED: 128x64 SSD1306 at I2C 0x3C, SDA on GPIO21, SCL on GPIO22. Shows WHITE or
// BLACK (BLACK if VOUT3 or VOUT4 sees black) and the line state. The display is
// probed every 2 s, so it can be plugged in or fixed while the sketch runs.
// Piezo: PAM8904 piezo driver, SIG on GPIO23. Plays a low tone over white and a high tone
// over black (black as on the OLED: VOUT3 or VOUT4 sees black). U 0 mutes it.
//
// Steering test: VOUT3 is the centre sensor and VOUT4 the outer one. Their
// black/white pattern gives a line state, and each state has its own motor
// speed, standing in for one wheel's speed in a turn:
//   VOUT3 black, VOUT4 white  centre   line under the middle, go straight
//   VOUT3 black, VOUT4 black  drift    line moving toward the VOUT4 side
//   VOUT3 white, VOUT4 black  turn     line on the VOUT4 side, turn hard
//   both white                lost     no line, stop
// A disabled sensor always counts as white. Auto mode is off at boot, so the
// motor never moves until it is enabled.
//
// Each sensor's black/white flips only once its reading is HYSTERESIS_MV past
// the threshold, so a reading sitting on the threshold does not chatter.
//
// Commands, one per line at 115200 baud:
//   A <0|1>      auto mode off / on (off also stops the motor)
//   V <s> <n>    speed for state s (0 lost, 1 centre, 2 drift, 3 turn), 0..255
//                (same calibrated scale as motor_control's M)
//   T <i> <mv>   threshold for sensor i (1-4) in millivolts; below = black
//   E <i> <0|1>  ignore / use sensor i
//   M <n>        manual speed -255..255 (turns auto off)
//   B            brake (turns auto off)
//   U <0|1>      speaker off / on
//   F <w> <b>    tone in Hz over white / over black, 20..20000
//   ?            report the config
// Streamed every 50 ms:
//   D <mv1> <mv2> <mv3> <mv4> black=<bits VOUT1..4> state=<s> speed=<n> duty=<n> mode=<m> auto=<0|1>
//     oled=<0|1> tone=<hz>  (oled: 1 when the display answers on I2C; tone: 0 when silent)
// Answered after each command:
//   CFG auto=<0|1> sp=<lost>,<centre>,<drift>,<turn> th=<mv1>,<mv2>,<mv3>,<mv4> en=<bits VOUT1..4> snd=<0|1> hz=<white>,<black>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int NUM_SENSORS = 4;
const uint8_t SENSOR_PINS[NUM_SENSORS] = {2, 3, 4, 5};   // VOUT1..VOUT4
const uint8_t SAMPLES = 32;

const int IN3_PIN   = 18;
const int IN4_PIN   = 19;
const int SLEEP_PIN = -1;      // GPIO wired to the driver's sleep pin, or -1 if tied to 3.3 V

const int PWM_FREQ = 20000;    // 20 kHz, above audible range
const int PWM_BITS = 8;        // duty 0..255

// Lowest duty that starts the motor from rest: 66% (168/255) at 6 V,
// measured 2026-10-16. Re-measure with motor_control's R command if the supply changes.
const int MIN_DUTY = 168;

// Fitted from the 2026-10-29 tape recordings: midway between each sensor's
// white 5th percentile and black 95th percentile. VOUT1 (inconsistent with
// tape position) and VOUT2 (saturated near 3.35 V) are left out of the rule
// until their hardware is checked.
const int DEFAULT_THRESHOLD_MV[NUM_SENSORS] = {650, 800, 1500, 1650};
const bool DEFAULT_ENABLED[NUM_SENSORS] = {false, false, true, true};
const int HYSTERESIS_MV = 50;
const uint16_t PRINT_INTERVAL_MS = 50;

const int SCREEN_W = 128;
const int SCREEN_H = 64;
const uint8_t OLED_ADDR = 0x3C;
const int OLED_SDA_PIN = 21;
const int OLED_SCL_PIN = 22;
const uint16_t OLED_PROBE_INTERVAL_MS = 2000;

const int SPEAKER_PIN = 23;
// Piezos are loudest near their 2-4 kHz resonance and faint below ~1 kHz.
const uint32_t DEFAULT_WHITE_TONE_HZ = 2000;
const uint32_t DEFAULT_BLACK_TONE_HZ = 4000;  // an octave up

const int CENTRE_SENSOR = 2;   // VOUT3
const int OUTER_SENSOR = 3;    // VOUT4

enum LineState { LOST, CENTRE, DRIFT, TURN, NUM_STATES };
const char *STATE_NAMES[NUM_STATES] = {"lost", "centre", "drift", "turn"};
const int DEFAULT_STATE_SPEED[NUM_STATES] = {0, 128, 191, 255};   // 0%, 50%, 75%, 100%

int thresholdMv[NUM_SENSORS];
bool enabled[NUM_SENSORS];
uint32_t readingMv[NUM_SENSORS];
bool black[NUM_SENSORS];
bool autoMode = false;
int stateSpeed[NUM_STATES];
LineState lineState = LOST;

int curSpeed = 0;
int curDuty = 0;
bool braking = false;
String line;
uint32_t lastPrint = 0;

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOk = false;
bool oledDirty = true;
uint32_t lastOledProbe = 0;
bool shownBlack = false;
LineState shownState = LOST;

bool soundOn = true;
uint32_t whiteToneHz = DEFAULT_WHITE_TONE_HZ;
uint32_t blackToneHz = DEFAULT_BLACK_TONE_HZ;
uint32_t toneHz = 0;

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

// Updates black[i] from the latest reading, holding it inside the hysteresis band.
void updateBlack(int i) {
  if ((int)readingMv[i] < thresholdMv[i] - HYSTERESIS_MV) black[i] = true;
  else if ((int)readingMv[i] > thresholdMv[i] + HYSTERESIS_MV) black[i] = false;
}

LineState currentState() {
  bool centre = enabled[CENTRE_SENSOR] && black[CENTRE_SENSOR];
  bool outer = enabled[OUTER_SENSOR] && black[OUTER_SENSOR];
  return centre ? (outer ? DRIFT : CENTRE) : (outer ? TURN : LOST);
}

const char *modeName() {
  return braking ? "brake" : curDuty > 0 ? "forward" : curDuty < 0 ? "reverse" : "coast";
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

void reportConfig() {
  Serial.printf("CFG auto=%d sp=%d,%d,%d,%d th=%d,%d,%d,%d en=", autoMode,
                stateSpeed[LOST], stateSpeed[CENTRE], stateSpeed[DRIFT], stateSpeed[TURN],
                thresholdMv[0], thresholdMv[1], thresholdMv[2], thresholdMv[3]);
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(enabled[i] ? '1' : '0');
  Serial.printf(" snd=%d hz=%lu,%lu\n", soundOn, (unsigned long)whiteToneHz, (unsigned long)blackToneHz);
}

void reportData() {
  Serial.printf("D %lu %lu %lu %lu black=", (unsigned long)readingMv[0], (unsigned long)readingMv[1],
                (unsigned long)readingMv[2], (unsigned long)readingMv[3]);
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(black[i] ? '1' : '0');
  Serial.printf(" state=%s speed=%d duty=%d mode=%s auto=%d oled=%d tone=%lu\n", STATE_NAMES[lineState],
                curSpeed, curDuty, modeName(), autoMode, oledOk, (unsigned long)toneHz);
}

// Checks whether the OLED answers, (re)initialising it when it appears.
void probeOled() {
  Wire.beginTransmission(OLED_ADDR);
  bool present = Wire.endTransmission() == 0;
  if (present && !oledOk) {
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.setTextColor(SSD1306_WHITE);
    oledDirty = true;
  }
  oledOk = present;
}

// Redraws only when the shown word or state changes: a full frame takes ~25 ms
// of I2C, which would otherwise stall the sensor loop.
void updateOled() {
  bool lineBlack = lineState != LOST;
  if (!oledOk || (!oledDirty && lineBlack == shownBlack && lineState == shownState)) return;
  shownBlack = lineBlack;
  shownState = lineState;
  oledDirty = false;

  display.clearDisplay();
  const char *word = lineBlack ? "BLACK" : "WHITE";
  display.setTextSize(3);
  int16_t x, y;
  uint16_t w, h;
  display.getTextBounds(word, 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_W - w) / 2, (48 - h) / 2);
  display.print(word);
  display.setTextSize(1);
  display.setCursor(0, SCREEN_H - 8);
  display.printf("state: %s", STATE_NAMES[lineState]);
  display.display();
}

// Changes the speaker tone only when it differs: retuning the timer resets the waveform.
void updateTone() {
  uint32_t want = !soundOn ? 0 : lineState == LOST ? whiteToneHz : blackToneHz;
  if (want == toneHz) return;
  ledcWriteTone(SPEAKER_PIN, want);   // 0 Hz silences the pin
  toneHz = want;
}

// Returns the sensor index (0-3) parsed from a 1-based argument, or -1.
int sensorArg(const String &arg) {
  int i = arg.toInt() - 1;
  return i >= 0 && i < NUM_SENSORS ? i : -1;
}

void handleLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd[0]);
  String args = cmd.substring(1);
  args.trim();
  int space = args.indexOf(' ');
  if (c == 'A') {
    autoMode = args.toInt() != 0;
    if (!autoMode) setMotor(0);
  } else if (c == 'V' && space > 0) {
    int s = args.substring(0, space).toInt();
    if (s >= 0 && s < NUM_STATES) stateSpeed[s] = constrain(args.substring(space + 1).toInt(), 0, 255);
  } else if ((c == 'T' || c == 'E') && space > 0 && sensorArg(args.substring(0, space)) >= 0) {
    int i = sensorArg(args.substring(0, space));
    int v = args.substring(space + 1).toInt();
    if (c == 'T') thresholdMv[i] = constrain(v, 0, 3300);
    else enabled[i] = v != 0;
  } else if (c == 'M') {
    autoMode = false;
    setMotor(args.toInt());
  } else if (c == 'B') {
    autoMode = false;
    brake();
  } else if (c == 'U') {
    soundOn = args.toInt() != 0;
  } else if (c == 'F' && space > 0) {
    whiteToneHz = constrain(args.substring(0, space).toInt(), 20, 20000);
    blackToneHz = constrain(args.substring(space + 1).toInt(), 20, 20000);
  } else if (c != '?') {
    Serial.printf("ERR unknown command: %s\n", cmd.c_str());
    return;
  }
  reportConfig();
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  for (int i = 0; i < NUM_SENSORS; i++) {
    // 11 dB attenuation gives a usable range of roughly 0.15-3.1 V.
    analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
    thresholdMv[i] = DEFAULT_THRESHOLD_MV[i];
    enabled[i] = DEFAULT_ENABLED[i];
    black[i] = false;
  }
  for (int s = 0; s < NUM_STATES; s++) stateSpeed[s] = DEFAULT_STATE_SPEED[s];

  if (SLEEP_PIN >= 0) {
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(SLEEP_PIN, HIGH);   // wake the driver
  }

  ledcAttach(IN3_PIN, PWM_FREQ, PWM_BITS);
  ledcAttach(IN4_PIN, PWM_FREQ, PWM_BITS);
  setMotor(0);
  // Attached after the motor pins so it gets its own LEDC timer: ledcWriteTone
  // retunes the timer, which must not change the motor's 20 kHz.
  ledcAttach(SPEAKER_PIN, DEFAULT_WHITE_TONE_HZ, PWM_BITS);
  ledcWriteTone(SPEAKER_PIN, 0);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  probeOled();

  Serial.println("READY line_motor");
  reportConfig();
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

  for (int i = 0; i < NUM_SENSORS; i++) {
    readingMv[i] = readAveragedMillivolts(SENSOR_PINS[i]);
    updateBlack(i);
  }
  lineState = currentState();

  if (autoMode) {
    int target = stateSpeed[lineState];
    if (target != curSpeed || braking) setMotor(target);
  }

  if (millis() - lastOledProbe >= OLED_PROBE_INTERVAL_MS) {
    lastOledProbe = millis();
    probeOled();
  }
  updateOled();
  updateTone();

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    reportData();
  }
}
