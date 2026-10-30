// Shows a message on the SSD1306 OLED and plays a tone on the piezo when cued
// white or black from serial. Standalone audio-visual test, no sensors: driven by tools/oled_web.py, but any serial
// monitor works.
//
// Target: ESP32-C6-DevKitM-1, arduino-esp32 3.x, USB CDC On Boot enabled.
// OLED: 128x64 SSD1306 at I2C 0x3C. The SDA/SCL pins are looked for among
// I2C_PINS (on 2026-10-30 the display was found with SDA on GPIO20 and SCL on
// GPIO21), and the display is probed every 2 s so it can be rewired while the
// sketch runs.
// Piezo: PAM8904 piezo driver, SIG on GPIO22 (moved from GPIO23 on 2026-10-30). A steady tone for white and another
// for black, as in line_motor. U 0 mutes it.
// Motor driver inputs are held low (coast).
//
// Each message is word-wrapped and drawn at the largest text size that fits.
//
// Commands, one per line at 115200 baud:
//   C <0|1>      cue white / black
//   W <text>     message shown for white (up to 40 characters)
//   K <text>     message shown for black
//   U <0|1>      speaker off / on
//   F <w> <b>    tone in Hz for white / for black, 20..20000
//   ?            report the state
// Answered after each command and when the display appears or disappears:
//   STATE cue=<white|black> oled=<0|1> sda=<pin> scl=<pin> snd=<0|1> hz=<white>,<black> tone=<hz>
//     (pins 0 if not found; tone 0 when silent)
//   MSG W <text>
//   MSG K <text>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int SCREEN_W = 128;
const int SCREEN_H = 64;
const uint8_t OLED_ADDR = 0x3C;
// {SDA, SCL} pairs, tried in order.
// GPIO22 is the speaker here, so 21/22 is not tried.
const int I2C_PINS[][2] = {{20, 21}, {21, 20}};
const uint16_t OLED_PROBE_INTERVAL_MS = 2000;
const unsigned MAX_MSG = 40;

const int IN3_PIN = 18;
const int IN4_PIN = 19;

const int SPEAKER_PIN = 22;
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

void reportState() {
  Serial.printf("STATE cue=%s oled=%d sda=%d scl=%d snd=%d hz=%lu,%lu tone=%lu\n", cueBlack ? "black" : "white",
                oledOk, sdaPin, sclPin, soundOn, (unsigned long)whiteToneHz, (unsigned long)blackToneHz,
                (unsigned long)toneHz);
  Serial.printf("MSG W %s\n", whiteMsg.c_str());
  Serial.printf("MSG K %s\n", blackMsg.c_str());
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

void handleLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd[0]);
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
  else if (c != '?') {
    Serial.printf("ERR unknown command: %s\n", cmd.c_str());
    return;
  }
  dirty = true;
  updateTone();   // so the reported tone is the one now playing
  reportState();
}

void setup() {
  Serial.begin(115200);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  ledcAttach(SPEAKER_PIN, DEFAULT_WHITE_TONE_HZ, TONE_BITS);
  ledcWriteTone(SPEAKER_PIN, 0);

  probeOled();
  updateTone();
  Serial.println("READY oled_cue");
  reportState();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
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
  draw();
  updateTone();
}
