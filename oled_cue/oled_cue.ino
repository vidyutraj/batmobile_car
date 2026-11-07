// Shows a message on the SSD1306 OLED and plays a tone on the piezo when cued
// white or black from serial, and drives the two motors. Standalone bench test, no sensors: driven by tools/oled_web.py, but any serial
// monitor works.
//
// Target: ESP32-C6-DevKitM-1, arduino-esp32 3.x, USB CDC On Boot DISABLED
// (fqbn esp32:esp32:esp32c6:CDCOnBoot=default). The PCB wires motor IN1 to GPIO12, which is the C6's native USB D-,
// so the native USB port is switched off and everything (upload and serial) goes through the DevKit's UART USB port
// (USB-to-serial chip on UART0, GPIO16/17).
// OLED: 128x64 SSD1306 at I2C 0x3C. The SDA/SCL pins are looked for among
// I2C_PINS (SDA on GPIO6 and SCL on GPIO7 as of 2026-11-07; was 20/21 before) and
// the display is probed every 2 s so it can be rewired while the
// sketch runs.
// Piezo: PAM8904 piezo driver, SIG on GPIO20 (since 2026-11-07; was GPIO22, and GPIO23 before that). A steady tone for white and another
// for black, as in line_motor. U 0 mutes it.
// Motors: DRV8833-style driver, PWM at 20 kHz. Motor A on IN1 = GPIO12, IN2 = GPIO3; motor B on IN3 = GPIO18,
// IN4 = GPIO19. Forward = IN1 (IN3) driven, IN2 (IN4) low. Non-zero speeds are scaled onto MIN_DUTY..255 so every one
// turns the motor. They coast at boot, and stop if no command (P keepalive included) arrives for MOTOR_TIMEOUT_MS,
// so a closed page or unplugged USB cannot leave them running.
// Sensors: VOUT1..VOUT4 on GPIO0, GPIO1, GPIO2, GPIO5 (ADC1 channels 0, 1, 2, 5). Read only: reported every
// SENSOR_INTERVAL_MS as a SENS line, nothing acts on them yet.
//
// Each message is word-wrapped and drawn at the largest text size that fits.
//
// Commands, one per line at 115200 baud:
//   C <0|1>      cue white / black
//   W <text>     message shown for white (up to 40 characters)
//   K <text>     message shown for black
//   U <0|1>      speaker off / on
//   F <w> <b>    tone in Hz for white / for black, 20..20000
//   D <a> <b>    motor A / motor B speed, -255 (full reverse) .. 255 (full forward); 0 = coast
//   X            brake both motors (both inputs high)
//   P            keepalive for the motor timeout; no reply
//   ?            report the state
// Answered after each command and when the display appears or disappears:
//   STATE cue=<white|black> oled=<0|1> sda=<pin> scl=<pin> snd=<0|1> hz=<white>,<black> tone=<hz>
//         ma=<duty> mb=<duty> brake=<0|1>   (signed PWM duty actually applied)
//     (pins 0 if not found; tone 0 when silent)
//   MSG W <text>
//   MSG K <text>
// Every SENSOR_INTERVAL_MS:
//   SENS <mv1> <mv2> <mv3> <mv4>   (VOUT1..VOUT4 in millivolts)

#include <Wire.h>
#include "soc/usb_serial_jtag_reg.h"
#include "soc/soc.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int SCREEN_W = 128;
const int SCREEN_H = 64;
const uint8_t OLED_ADDR = 0x3C;
// {SDA, SCL} pairs, tried in order.
const int I2C_PINS[][2] = {{6, 7}, {7, 6}};
const uint16_t OLED_PROBE_INTERVAL_MS = 2000;
const unsigned MAX_MSG = 40;

const int IN1_PIN = 12;   // motor A; GPIO12 is also native USB D-
const int IN2_PIN = 3;
const int IN3_PIN = 18;   // motor B
const int IN4_PIN = 19;
const int MOTOR_PWM_FREQ = 20000;   // above audible range
const int MOTOR_PWM_BITS = 8;       // duty 0..255
// Lowest duty that starts the motor from rest: 168/255 at 6 V, measured 2026-10-16 (motor_control).
const int MIN_DUTY = 168;
const uint32_t MOTOR_TIMEOUT_MS = 1500;
// LEDC channels are paired on timers (0-1, 2-3, 4-5). The speaker retunes its timer for every tone, so it gets
// channel 0 alone and the motors take 2-5.
const int SPEAKER_CH = 0;
const int MOTOR_CH[4] = {2, 3, 4, 5};   // IN1, IN2, IN3, IN4

const uint8_t SENSOR_PINS[4] = {0, 1, 2, 5};   // VOUT1..VOUT4
const uint8_t SENSOR_SAMPLES = 16;
const uint16_t SENSOR_INTERVAL_MS = 100;

const int SPEAKER_PIN = 20;
// Piezos are loudest near their 2-4 kHz resonance and faint below ~1 kHz.
const uint32_t DEFAULT_WHITE_TONE_HZ = 2000;
const uint32_t DEFAULT_BLACK_TONE_HZ = 4000;  // an octave up
const int TONE_BITS = 8;

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);
bool oledOk = false;
int sdaPin = 0;
int sclPin = 0;
bool dirty = true;
uint32_t lastProbe = 0;

bool cueBlack = false;
String whiteMsg = "WHITE";
String blackMsg = "BLACK";
String line;

bool soundOn = true;
uint32_t whiteToneHz = DEFAULT_WHITE_TONE_HZ;
uint32_t blackToneHz = DEFAULT_BLACK_TONE_HZ;
uint32_t toneHz = 0;

int dutyA = 0, dutyB = 0;
bool braking = false;
uint32_t lastCmd = 0;
uint32_t lastSensor = 0;

void reportState() {
  Serial0.printf("STATE cue=%s oled=%d sda=%d scl=%d snd=%d hz=%lu,%lu tone=%lu ma=%d mb=%d brake=%d\n",
                cueBlack ? "black" : "white", oledOk, sdaPin, sclPin, soundOn, (unsigned long)whiteToneHz,
                (unsigned long)blackToneHz, (unsigned long)toneHz, dutyA, dutyB, braking);
  Serial0.printf("MSG W %s\n", whiteMsg.c_str());
  Serial0.printf("MSG K %s\n", blackMsg.c_str());
}

bool answers() {
  Wire.beginTransmission(OLED_ADDR);
  return Wire.endTransmission() == 0;
}

// Keeps the current pins while the display answers; otherwise tries each pair.
void probeOled() {
  bool was = oledOk;
  if (!(oledOk && answers())) {
    oledOk = false;
    for (auto &pins : I2C_PINS) {
      Wire.end();
      Wire.begin(pins[0], pins[1]);
      Wire.setClock(400000);
      if (answers()) {
        sdaPin = pins[0];
        sclPin = pins[1];
        oledOk = true;
        display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
        display.setTextColor(SSD1306_WHITE);
        dirty = true;
        break;
      }
    }
    if (!oledOk) sdaPin = sclPin = 0;
  }
  if (oledOk != was) reportState();
}

// Greedy word wrap into lines of at most `width` characters; false if it needs
// more than `maxLines` lines.
bool wrap(const String &text, int width, int maxLines, String *out, int &n) {
  n = 0;
  String cur;
  int i = 0;
  while (i < (int)text.length()) {
    int sp = text.indexOf(' ', i);
    if (sp < 0) sp = text.length();
    String word = text.substring(i, sp);
    i = sp + 1;
    if (word.length() == 0) continue;
    while ((int)word.length() > width) {   // split words longer than a line
      if (cur.length()) { if (n == maxLines) return false; out[n++] = cur; cur = ""; }
      if (n == maxLines) return false;
      out[n++] = word.substring(0, width);
      word = word.substring(width);
    }
    if (cur.length() == 0) cur = word;
    else if ((int)(cur.length() + 1 + word.length()) <= width) cur += " " + word;
    else { if (n == maxLines) return false; out[n++] = cur; cur = word; }
  }
  if (cur.length()) { if (n == maxLines) return false; out[n++] = cur; }
  return true;
}

void draw() {
  if (!oledOk || !dirty) return;
  dirty = false;
  const String &msg = cueBlack ? blackMsg : whiteMsg;
  display.clearDisplay();
  String lines[8];
  int n = 0;
  for (int size = 3; size >= 1; size--) {
    // Built-in font cells are 6x8 px at size 1.
    if (!wrap(msg, SCREEN_W / (6 * size), SCREEN_H / (8 * size), lines, n) && size > 1) continue;
    display.setTextSize(size);
    int top = (SCREEN_H - n * 8 * size) / 2;
    for (int l = 0; l < n; l++) {
      display.setCursor((SCREEN_W - (int)lines[l].length() * 6 * size) / 2, top + l * 8 * size);
      display.print(lines[l]);
    }
    break;
  }
  display.display();
}

// Changes the speaker tone only when it differs: retuning the timer resets the waveform.
void updateTone() {
  uint32_t want = !soundOn ? 0 : cueBlack ? blackToneHz : whiteToneHz;
  if (want == toneHz) return;
  ledcWriteTone(SPEAKER_PIN, want);   // 0 Hz silences the pin
  toneHz = want;
}

// Averages several calibrated readings. The first read after switching channels is discarded: the ADC's sample
// capacitor still holds charge from the previous channel.
uint32_t readMillivolts(uint8_t pin) {
  analogReadMilliVolts(pin);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < SENSOR_SAMPLES; i++) sum += analogReadMilliVolts(pin);
  return sum / SENSOR_SAMPLES;
}

void reportSensors() {
  uint32_t mv[4];
  for (int i = 0; i < 4; i++) mv[i] = readMillivolts(SENSOR_PINS[i]);
  Serial0.printf("SENS %lu %lu %lu %lu\n", (unsigned long)mv[0], (unsigned long)mv[1], (unsigned long)mv[2],
                 (unsigned long)mv[3]);
}

// speed -255..255 -> signed duty, non-zero speeds scaled onto MIN_DUTY..255.
int speedToDuty(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) return 0;
  int duty = map(abs(speed), 1, 255, MIN_DUTY, 255);
  return speed < 0 ? -duty : duty;
}

void driveMotor(int pinFwd, int pinRev, int duty) {
  ledcWrite(pinFwd, duty > 0 ? duty : 0);
  ledcWrite(pinRev, duty < 0 ? -duty : 0);
}

void setMotors(int speedA, int speedB) {
  dutyA = speedToDuty(speedA);
  dutyB = speedToDuty(speedB);
  braking = false;
  driveMotor(IN1_PIN, IN2_PIN, dutyA);
  driveMotor(IN3_PIN, IN4_PIN, dutyB);
}

void brakeMotors() {
  for (int pin : {IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN}) ledcWrite(pin, 255);
  dutyA = dutyB = 0;
  braking = true;
}

void handleLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd[0]);
  lastCmd = millis();
  if (c == 'P') return;
  String arg = cmd.substring(1);
  arg.trim();
  if (arg.length() > MAX_MSG) arg = arg.substring(0, MAX_MSG);
  if (c == 'C') cueBlack = arg.toInt() != 0;
  else if (c == 'W') whiteMsg = arg;
  else if (c == 'K') blackMsg = arg;
  else if (c == 'U') soundOn = arg.toInt() != 0;
  else if (c == 'F' && arg.indexOf(' ') > 0) {
    int space = arg.indexOf(' ');
    whiteToneHz = constrain(arg.substring(0, space).toInt(), 20, 20000);
    blackToneHz = constrain(arg.substring(space + 1).toInt(), 20, 20000);
  }
  else if (c == 'D' && arg.indexOf(' ') > 0) {
    int space = arg.indexOf(' ');
    setMotors(arg.substring(0, space).toInt(), arg.substring(space + 1).toInt());
  }
  else if (c == 'X') brakeMotors();
  else if (c != '?') {
    Serial0.printf("ERR unknown command: %s\n", cmd.c_str());
    return;
  }
  dirty = true;
  updateTone();   // so the reported tone is the one now playing
  reportState();
}

void setup() {
  // Release GPIO12/13 from the native USB PHY so GPIO12 can drive IN1. The native USB port disappears.
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
  Serial0.begin(115200);
  ledcAttachChannel(SPEAKER_PIN, DEFAULT_WHITE_TONE_HZ, TONE_BITS, SPEAKER_CH);
  const int motorPins[4] = {IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN};
  for (int i = 0; i < 4; i++) ledcAttachChannel(motorPins[i], MOTOR_PWM_FREQ, MOTOR_PWM_BITS, MOTOR_CH[i]);
  setMotors(0, 0);
  analogReadResolution(12);
  for (uint8_t pin : SENSOR_PINS) analogSetPinAttenuation(pin, ADC_11db);   // ~0.15-3.1 V usable
  ledcWriteTone(SPEAKER_PIN, 0);

  probeOled();
  updateTone();
  Serial0.println("READY oled_cue");
  reportState();
}

void loop() {
  while (Serial0.available()) {
    char c = Serial0.read();
    if (c == '\n' || c == '\r') {
      handleLine(line);
      line = "";
    } else if (line.length() < MAX_MSG + 8) {
      line += c;
    }
  }
  if (millis() - lastProbe >= OLED_PROBE_INTERVAL_MS) {
    lastProbe = millis();
    probeOled();
  }
  if ((dutyA || dutyB || braking) && millis() - lastCmd > MOTOR_TIMEOUT_MS) {
    setMotors(0, 0);
    Serial0.println("WARN motor timeout: no command, motors stopped");
    reportState();
  }
  if (millis() - lastSensor >= SENSOR_INTERVAL_MS) {
    lastSensor = millis();
    reportSensors();
  }
  draw();
  updateTone();
}
