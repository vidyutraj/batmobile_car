/*
 * Batmobile Line Follower - ESP32-C6 (Arduino-ESP32 core 3.x)
 * ECE3011 Team T04
 *
 * What it does on power-up (no user input needed):
 *   1. Init motors (held off), OLED, speaker, ADC.
 *   2. 5 s countdown on the OLED with a beep each second.
 *      -> Place the car with the line between the two middle sensors before/at power-on.
 *   3. Auto-calibration: pivots left, then right, across the line (~1 s) to learn the
 *      black and white reading of EACH sensor, then pivots back until centered.
 *   4. Follows the line with simple threshold logic: each sensor is black or white (threshold halfway between
 *      its calibrated levels), and a table picks straight / gentle turn / sharp turn.
 *   5. Line lost  -> pivots toward where the line was last seen; stops after LOST_TIMEOUT_MS.
 *      All black  -> (crossing line or the car was picked up) drives straight; stops
 *                    if it lasts longer than ALL_BLACK_STOP_MS.
 *
 * ---------------- Arduino IDE settings (IMPORTANT) ----------------
 *   Board: "ESP32C6 Dev Module" (or your C6 board)
 *   USB CDC On Boot: "Disabled"   <- GPIO12 (a motor pin) is the native-USB D- pin.
 *                                     Serial must go to UART0 (TX=GPIO16, RX=GPIO17),
 *                                     i.e. the "UART" USB port on a DevKitC-1.
 *   Library: "U8g2" by olikraus (Library Manager)
 *
 * ---------------- Serial debug (115200, UART0) ----------------
 *   Type a command (Enter optional):
 *     debug   cancel the auto-start countdown and sit idle
 *     go      calibrate (sweep) then drive        run   drive using the existing calibration
 *     stop    stop the motors                     cal   calibrate only, then idle
 *     motors  test each wheel fwd/rev (lift the car!)
 *     log     toggle 20 Hz telemetry: state, sensor mV, on-line pattern, position, L/R
 *     show    print settings + calibration
 *     kp X / ki X / kd X / base X     live-tune the controller
 *   BOOT button (GPIO9): stop when moving, otherwise "go".
 *
 * ---------------- Hardware assumptions (verify!) ----------------
 *   Sensors: IR LED always on; photodiode from 3V3 to OUTx, 4.7k OUTx to GND.
 *            WHITE = more reflection = HIGHER voltage, BLACK = LOWER voltage.
 *            Breadboard measured ~0.70 V white / ~0.15 V black -> must use the ADC,
 *            digitalRead() can never see these as HIGH.
 *   Motors:  DRV8833-style 2-input H-bridge per motor (no enable pin).
 *   Audio:   PAM8904 driven by a square wave on AMP_SIG_PIN, VDD from AMP_VDD_PIN.
 *   OLED:    128x64 SSD1306 (or SH1106, see OLED_SH1106) over I2C at 0x3C/0x3D.
 */
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "driver/gpio.h"
#include "esp_system.h"
 
// ================================ PINS ================================
const int NUM_SENSORS = 4;
// OUT1..OUT4 of the sensor strip, ordered LEFT -> RIGHT as seen from behind the car
// looking forward. If steering is mirrored, reverse this array.
const uint8_t SENSOR_PINS[NUM_SENSORS] = {0, 1, 2, 5};
// false = never used. S1 (GPIO0) sits at ~3130 mV, the top of the ADC range, on white and black alike.
const bool SENSOR_ENABLED[NUM_SENSORS] = {false, true, true, true};
 
// If a wheel spins backwards on "motors", swap that motor's two numbers.
// If "LEFT forward" moves the right wheel, swap the L_ and R_ pairs.
const uint8_t L_IN1 = 18, L_IN2 = 19;   // swapped 2026-11-07: left wheel ran backwards
const uint8_t R_IN1 = 12, R_IN2 = 3;
 
const uint8_t I2C_SDA = 6, I2C_SCL = 7;  // tried swapped automatically if the OLED isn't found
const uint8_t AMP_VDD_PIN = 21;          // powers the amplifier from a GPIO (see notes)
const uint8_t AMP_SIG_PIN = 20;          // square-wave tone into the amplifier
const uint8_t BOOT_BTN_PIN = 9;          // DevKit BOOT button, active LOW
 
// The core shares an LEDC timer only between channels with identical freq/resolution, so the
// motors (20 kHz, 8-bit) share one timer and the tone (10-bit) gets its own. ledcWriteTone()
// retunes only the tone's timer and can't disturb the motor PWM.
const uint8_t CH_L1 = 0, CH_L2 = 1, CH_R1 = 2, CH_R2 = 3, CH_TONE = 4;
 
// ================================ OPTIONS ================================
#define OLED_SH1106     0   // 1 if your 1.3" OLED is an SH1106 (garbage column on the edge = try this)
#define OLED_UPSIDEDOWN 0   // 1 to rotate the display 180 degrees
#define USE_RGB_LED     1   // on-board RGB LED (GPIO8 on the DevKitC-1) as a state indicator
#define USE_BOOT_BUTTON 1
 
// ================================ TUNING ================================
const uint32_t AUTOSTART_DELAY_MS = 5000;   // after init completes
const uint32_t CONTROL_PERIOD_MS  = 5;      // 200 Hz control loop
 
// --- Sensors ---
const int   SAMPLES_PER_READ = 4;           // ADC averaging per sensor per tick
const int   MIN_SPREAD_MV    = 80;          // black/white difference a sensor needs during cal to be used
                                           // (noise is ~±3 mV; sweeps on the PCB saw 100-180 mV)
const int   MIN_GOOD_SENSORS = 2;           // cal fails only if fewer sensors than this see the line
const float DARK_FLOOR       = 0.10f;       // ignore darkness below this (white noise) in the position math
const float LINE_SEEN_DARK   = 0.40f;       // some sensor at least this dark = line is visible
const float ALL_BLACK_DARK   = 0.75f;       // every sensor at least this dark = all black
const float SENSOR_WEIGHT[NUM_SENSORS] = {-1500, -500, 500, 1500};
// Threshold logic: a sensor is BLACK once its darkness (0 = calibrated white, 1 = calibrated black) rises above
// BLACK_ON, and WHITE again once it falls below BLACK_OFF. The gap is hysteresis so it doesn't flicker at the edge.
const float BLACK_ON  = 0.60f;
const float BLACK_OFF = 0.40f;
// Inner-wheel speed as a fraction of the base speed (outer wheel runs at base).
const float SOFT_TURN_INNER = 0.30f;    // a middle sensor sees the line: gentle turn
const float HARD_TURN_INNER = -0.50f;   // an outer sensor sees the line: inner wheel reverses, sharp turn
const float POS_MAX          = 1500.0f;
 
// --- Calibration sweep ---
const int      CAL_SPEED       = 150;       // pivot speed while sweeping (130 didn't turn the wheels)
const uint32_t CAL_SWEEP_MS    = 450;       // left for this long, then right for 2x this long
const int      CENTER_SPEED    = 110;       // pivot speed while re-centering on the line
const float    CENTER_TOL      = 350.0f;    // |position| below this counts as centered
 
// --- Motors ---
const uint32_t PWM_FREQ      = 20000;       // above hearing range
const uint8_t  PWM_BITS      = 8;           // speed / duty range 0-255
const int      MIN_DUTY      = 70;          // lowest duty that turns the wheels on the floor (MEASURE). 168 was only
                                            // needed on laptop USB power; on the battery it made the car far too strong.
const int      SLEW_PER_TICK = 20;          // max speed change per 5 ms (limits current spikes / brownouts)
const uint32_t BRAKE_MS      = 150;         // short-brake time when stopping, then coast
const int      MOTOR_TEST_SPEED = 160;
 
// --- Line following (live-tunable over serial) ---
volatile int   baseSpeed = 120;             // straight-line speed, 0-255
volatile float kp = 0.10f;                  // speed units per position unit
volatile float ki = 0.0f;
volatile float kd = 0.004f;                 // speed units per (position unit / second)
const float    D_FILTER       = 0.3f;       // derivative low-pass (0-1, lower = smoother)
const float    I_LIMIT        = 2000.0f;
const float    CURVE_SLOWDOWN = 0.35f;      // base speed drops up to 35 % at full error
const uint32_t RAMP_MS        = 400;        // soft start from 0 to base speed
 
// --- Lost line / safety ---
const int      SEARCH_SPEED       = 130;
const float    SEARCH_CENTER_BAND = 400.0f; // last seen near center -> creep straight instead of pivoting
const uint32_t LOST_TIMEOUT_MS    = 1500;
const uint32_t ALL_BLACK_STOP_MS  = 300;
 
// ================================ STATE ================================
enum State : uint8_t { ST_COUNTDOWN, ST_IDLE, ST_CALIBRATING, ST_RUNNING, ST_SEARCHING,
                       ST_STOPPED, ST_ERROR, ST_MOTOR_TEST };
const char *STATE_NAMES[] = {"COUNTDOWN", "IDLE", "CALIBRATING", "RUNNING", "SEARCHING",
                             "STOPPED", "ERROR", "MOTOR_TEST"};
enum StopReason : uint8_t { SR_NONE, SR_USER, SR_LOST, SR_ALL_BLACK, SR_CAL_FAIL, SR_CENTER_FAIL };
const char *STOP_TEXT[] = {"", "Stopped by user", "Line lost", "All black / lifted",
                           "Calibration failed", "Line not found"};
 
enum Cmd : uint8_t { CMD_GO, CMD_RUN, CMD_STOP, CMD_IDLE, CMD_CAL, CMD_MOTOR_TEST, CMD_TOGGLE };
enum Event : uint8_t { EV_TICK, EV_CAL_START, EV_CAL_RETRY, EV_CAL_OK, EV_CAL_FAIL, EV_GO,
                       EV_LOST, EV_FOUND, EV_STOPPED, EV_MT_STEP, EV_NO_CAL };
 
QueueHandle_t cmdQueue, eventQueue;
 
// Written by the control task, read by loop() for display/telemetry (32-bit, so no tearing)
volatile State state = ST_COUNTDOWN;
volatile StopReason stopReason = SR_NONE;
volatile int   mv[NUM_SENSORS];
volatile float darkness[NUM_SENSORS];         // 0 = white, 1 = black (after calibration)
volatile float linePos = 0;                   // -1500 (line far left) ... +1500 (far right)
volatile bool  lineSeen = false;
volatile bool  isBlack[NUM_SENSORS];         // thresholded, with hysteresis (false for unused sensors)
volatile int   outL = 0, outR = 0;            // speeds actually applied
volatile int   countdownSec = 5;
volatile int   motorTestStep = 0;
 
int  blackMv[NUM_SENSORS], whiteMv[NUM_SENSORS], spreadMv[NUM_SENSORS];
volatile bool calValid = false;
bool sensorOk[NUM_SENSORS] = {false, true, true, true};   // false = excluded (no contrast during cal)
 
// Control-task-only state
uint32_t countdownEnd = 0;
int      tgtL = 0, tgtR = 0;
uint32_t stopTime = 0;
int      calPhase = 0, calRetries = 0;
uint32_t calPhaseStart = 0, calSweepMs = CAL_SWEEP_MS;
bool     calThenRun = true;
int      calMin[NUM_SENSORS], calMax[NUM_SENSORS];
int      lastSweepLo[NUM_SENSORS], lastSweepHi[NUM_SENSORS];   // last finished sweep, for printCal
uint32_t runStart = 0, lostSince = 0, allBlackSince = 0, motorTestStart = 0;
float    lastPos = 0, prevErr = 0, integ = 0, dFilt = 0;
 
// ================================ MOTORS ================================
void writeMotor(uint8_t in1, uint8_t in2, int spd) {
  // DRV8833 fast decay: PWM one input, hold the other low. Both low = coast.
  int duty = abs(spd);
  if (duty > 0) duty = map(duty, 1, 255, MIN_DUTY, 255);   // skip the dead zone
  ledcWrite(in1, spd > 0 ? duty : 0);
  ledcWrite(in2, spd < 0 ? duty : 0);
}
 
void brakeMotors() {                       // both inputs high = short brake
  for (uint8_t p : {L_IN1, L_IN2, R_IN1, R_IN2}) ledcWrite(p, 255);
}
 
int slewToward(int cur, int target) {
  if (target > cur + SLEW_PER_TICK) return cur + SLEW_PER_TICK;
  if (target < cur - SLEW_PER_TICK) return cur - SLEW_PER_TICK;
  return target;
}
 
bool motorsActive() {
  return state == ST_CALIBRATING || state == ST_RUNNING || state == ST_SEARCHING ||
         state == ST_MOTOR_TEST;
}
 
void applyMotors(uint32_t now) {
  if (!motorsActive()) {
    outL = 0; outR = 0;
    if ((state == ST_STOPPED || state == ST_ERROR) && now - stopTime < BRAKE_MS) brakeMotors();
    else { writeMotor(L_IN1, L_IN2, 0); writeMotor(R_IN1, R_IN2, 0); }
    return;
  }
  outL = slewToward(outL, constrain(tgtL, -255, 255));
  outR = slewToward(outR, constrain(tgtR, -255, 255));
  writeMotor(L_IN1, L_IN2, outL);
  writeMotor(R_IN1, R_IN2, outR);
}
 
// ================================ SENSORS ================================
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sum = 0;
    for (int k = 0; k < SAMPLES_PER_READ; k++) sum += analogReadMilliVolts(SENSOR_PINS[i]);
    mv[i] = sum / SAMPLES_PER_READ;
  }
}
 
// Turns mV into per-sensor darkness (0..1), a black/white decision per sensor (threshold halfway between the
// calibrated white and black, with hysteresis), and a rough line position from the black sensors.
void computeLine() {
  if (!calValid) { lineSeen = false; for (int i = 0; i < NUM_SENSORS; i++) isBlack[i] = false; return; }
  float num = 0;
  int n = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensorOk[i]) { darkness[i] = 0; isBlack[i] = false; continue; }   // unused sensor
    float d = (float)(whiteMv[i] - mv[i]) / (float)(whiteMv[i] - blackMv[i]);
    d = constrain(d, 0.0f, 1.0f);
    darkness[i] = d;
    if (isBlack[i] ? d < BLACK_OFF : d > BLACK_ON) isBlack[i] = !isBlack[i];
    if (isBlack[i]) { num += SENSOR_WEIGHT[i]; n++; }
  }
  lineSeen = n > 0;
  if (lineSeen) linePos = num / n;
}
 
bool allBlack() {
  if (!calValid) return false;
  for (int i = 0; i < NUM_SENSORS; i++) if (sensorOk[i] && !isBlack[i]) return false;
  return true;
}
 
// ================================ STATE CHANGES ================================
void postEvent(Event e) { uint8_t v = e; xQueueSend(eventQueue, &v, 0); }
 
void stopCar(StopReason r, uint32_t now, State s = ST_STOPPED) {
  state = s;
  stopReason = r;
  stopTime = now;
  tgtL = tgtR = 0;
  postEvent(EV_STOPPED);
}
 
void startRun(uint32_t now) {
  state = ST_RUNNING;
  runStart = now;
  integ = 0; dFilt = 0;
  prevErr = lineSeen ? linePos : 0;
  lastPos = prevErr;
  lostSince = 0; allBlackSince = 0;
  postEvent(EV_GO);
}
 
void startCalibration(uint32_t now, bool thenRun, uint32_t sweepMs) {
  for (int i = 0; i < NUM_SENSORS; i++) { calMin[i] = 99999; calMax[i] = 0; }
  calThenRun = thenRun;
  calSweepMs = sweepMs;
  calPhase = 0;
  calPhaseStart = now;
  state = ST_CALIBRATING;
  postEvent(EV_CAL_START);
}
 
// ================================ STATE HANDLERS ================================
void tickCountdown(uint32_t now) {
  tgtL = tgtR = 0;
  int left = (int)((countdownEnd - now + 999) / 1000);
  if ((int32_t)(countdownEnd - now) <= 0) {
    calRetries = 0;
    startCalibration(now, true, CAL_SWEEP_MS);
    return;
  }
  if (left != countdownSec) { countdownSec = left; postEvent(EV_TICK); }
}
 
// Phase 0: pivot left. Phase 1: pivot right (twice as long, so it crosses back over).
// Then validate, and phase 2: pivot back until a middle sensor is on the line.
void tickCalibration(uint32_t now) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = min(calMin[i], (int)mv[i]);
    calMax[i] = max(calMax[i], (int)mv[i]);
  }
  uint32_t t = now - calPhaseStart;
 
  switch (calPhase) {
    case 0:
      tgtL = -CAL_SPEED; tgtR = CAL_SPEED;
      if (t >= calSweepMs) { calPhase = 1; calPhaseStart = now; }
      break;
 
    case 1: {
      tgtL = CAL_SPEED; tgtR = -CAL_SPEED;
      if (t < 2 * calSweepMs) break;
      int good = 0;
      for (int i = 0; i < NUM_SENSORS; i++) {
        spreadMv[i] = calMax[i] - calMin[i];
        lastSweepLo[i] = calMin[i];
        lastSweepHi[i] = calMax[i];
        if (SENSOR_ENABLED[i] && spreadMv[i] >= MIN_SPREAD_MV) good++;
      }
      bool ok = good >= MIN_GOOD_SENSORS;
      if (!ok) {
        if (calRetries < 1) {            // one retry with a wider sweep
          calRetries++;
          postEvent(EV_CAL_RETRY);
          startCalibration(now, calThenRun, calSweepMs * 3 / 2);
        } else {
          stopCar(SR_CAL_FAIL, now, ST_ERROR);
          postEvent(EV_CAL_FAIL);
        }
        break;
      }
      for (int i = 0; i < NUM_SENSORS; i++) {
        blackMv[i] = calMin[i]; whiteMv[i] = calMax[i];
        sensorOk[i] = SENSOR_ENABLED[i] && spreadMv[i] >= MIN_SPREAD_MV;
      }
      calValid = true;
      postEvent(EV_CAL_OK);
      calPhase = 2; calPhaseStart = now;
      break;
    }
 
    case 2:
      // The car ended the sweep rotated right of where it started, so the line is most likely to its left.
      // Pivot toward the line until one of the two middle sensors (S2, S3) reads black.
      if (isBlack[1] || isBlack[2]) {
        if (calThenRun) startRun(now);
        else { state = ST_IDLE; tgtL = tgtR = 0; }
        break;
      }
      if (t > 3 * calSweepMs) {          // timeout
        if (!lineSeen) { stopCar(SR_CENTER_FAIL, now, ST_ERROR); break; }
        if (calThenRun) startRun(now); else { state = ST_IDLE; tgtL = tgtR = 0; }
        break;
      }
      if (lineSeen && linePos > 0) { tgtL = CENTER_SPEED;  tgtR = -CENTER_SPEED; }
      else                         { tgtL = -CENTER_SPEED; tgtR = CENTER_SPEED; }
      break;
  }
}
 
void tickFollow(uint32_t now) {
  // All black: crossing line (brief) or the car was lifted / hit a big black area (long).
  if (allBlack()) {
    if (allBlackSince == 0) allBlackSince = now;
    if (now - allBlackSince > ALL_BLACK_STOP_MS) { stopCar(SR_ALL_BLACK, now); return; }
    tgtL = tgtR = baseSpeed;
    return;
  }
  allBlackSince = 0;
 
  if (lineSeen) {
    if (state == ST_SEARCHING) { state = ST_RUNNING; postEvent(EV_FOUND); }
    lostSince = 0;
    lastPos = linePos;

    // Threshold table. Sensors left -> right: S1 (outer), S2, S3 (middle), S4 (outer).
    float ramp = min(1.0f, (float)(now - runStart) / RAMP_MS);
    int base = (int)(baseSpeed * ramp);
    int soft = (int)(base * SOFT_TURN_INNER), hard = (int)(base * HARD_TURN_INNER);
    bool s1 = isBlack[0], s2 = isBlack[1], s3 = isBlack[2], s4 = isBlack[3];
    if (s1)            { tgtL = hard; tgtR = base; }   // line far left: sharp left
    else if (s4)       { tgtL = base; tgtR = hard; }   // line far right: sharp right
    else if (s2 && s3) { tgtL = base; tgtR = base; }   // line under the middle: straight
    else if (s2)       { tgtL = soft; tgtR = base; }   // line a bit left: gentle left
    else               { tgtL = base; tgtR = soft; }   // s3 only, line a bit right: gentle right
    return;
  }
 
  // Line not seen
  if (state == ST_RUNNING) { state = ST_SEARCHING; lostSince = now; postEvent(EV_LOST); }
  if (now - lostSince > LOST_TIMEOUT_MS) { stopCar(SR_LOST, now); return; }
  if (fabsf(lastPos) < SEARCH_CENTER_BAND) { tgtL = tgtR = SEARCH_SPEED; }   // gap: keep going
  else if (lastPos > 0) { tgtL = SEARCH_SPEED;  tgtR = -SEARCH_SPEED; }      // pivot right
  else                  { tgtL = -SEARCH_SPEED; tgtR = SEARCH_SPEED; }       // pivot left
}
 
void tickMotorTest(uint32_t now) {
  static const int8_t steps[][2] = {{1, 0}, {0, 0}, {-1, 0}, {0, 0}, {0, 1}, {0, 0}, {0, -1}, {0, 0},
                                    {1, 1}, {0, 0}, {-1, -1}, {0, 0}};
  const int numSteps = sizeof(steps) / sizeof(steps[0]);
  int step = (now - motorTestStart) / 1000;
  if (step >= numSteps) { state = ST_IDLE; tgtL = tgtR = 0; return; }
  if (step != motorTestStep) { motorTestStep = step; postEvent(EV_MT_STEP); }
  tgtL = steps[step][0] * MOTOR_TEST_SPEED;
  tgtR = steps[step][1] * MOTOR_TEST_SPEED;
}
 
void handleCmd(uint8_t cmd, uint32_t now) {
  switch (cmd) {
    case CMD_GO:   calRetries = 0; startCalibration(now, true, CAL_SWEEP_MS); break;
    case CMD_CAL:  calRetries = 0; startCalibration(now, false, CAL_SWEEP_MS); break;
    case CMD_RUN:  if (calValid) startRun(now); else postEvent(EV_NO_CAL); break;
    case CMD_STOP: if (motorsActive() || state == ST_COUNTDOWN) stopCar(SR_USER, now); break;
    case CMD_IDLE: if (motorsActive()) stopCar(SR_USER, now); state = ST_IDLE; break;
    case CMD_MOTOR_TEST: state = ST_MOTOR_TEST; motorTestStart = now; motorTestStep = -1; break;
    case CMD_TOGGLE:
      if (motorsActive() || state == ST_COUNTDOWN) stopCar(SR_USER, now);
      else { calRetries = 0; startCalibration(now, true, CAL_SWEEP_MS); }
      break;
  }
}
 
// Runs at a fixed 200 Hz at higher priority than loop(), so slow OLED/I2C or Serial
// work in loop() can never delay steering.
void controlTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    uint32_t now = millis();
 
    uint8_t cmd;
    while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) handleCmd(cmd, now);
 
    readSensors();
    computeLine();
 
    switch (state) {
      case ST_COUNTDOWN:   tickCountdown(now); break;
      case ST_CALIBRATING: tickCalibration(now); break;
      case ST_RUNNING:
      case ST_SEARCHING:   tickFollow(now); break;
      case ST_MOTOR_TEST:  tickMotorTest(now); break;
      default:             tgtL = tgtR = 0; break;
    }
    applyMotors(now);
  }
}
 
// ================================ SOUND ================================
struct Note { uint16_t freq; uint16_t ms; };     // freq 0 = rest, ms 0 = end
// The PAM8904 drives a piezo, which is loudest around 2-4 kHz.
const Note SND_BOOT[]  = {{2093, 90}, {2637, 90}, {3136, 160}, {0, 0}};
const Note SND_TICK[]  = {{2000, 60}, {0, 0}};
const Note SND_CAL[]   = {{2637, 60}, {0, 40}, {2637, 60}, {0, 0}};
const Note SND_GO[]    = {{2093, 80}, {3136, 80}, {4186, 200}, {0, 0}};
const Note SND_LOST[]  = {{3136, 90}, {2093, 140}, {0, 0}};
const Note SND_STOP[]  = {{2637, 120}, {1568, 250}, {0, 0}};
const Note SND_ERROR[] = {{1200, 150}, {0, 80}, {1200, 150}, {0, 80}, {1200, 300}, {0, 0}};
 
const Note *sndCur = nullptr;
int sndIdx = 0;
uint32_t sndNoteStart = 0;
 
void playSound(const struct Note *s) {   // "struct" keeps Arduino's auto-prototype compiling
  sndCur = s; sndIdx = 0; sndNoteStart = millis();
  ledcWriteTone(AMP_SIG_PIN, s[0].freq);
}
 
void updateSound() {
  if (!sndCur || millis() - sndNoteStart < sndCur[sndIdx].ms) return;
  sndIdx++;
  if (sndCur[sndIdx].ms == 0) { ledcWriteTone(AMP_SIG_PIN, 0); sndCur = nullptr; return; }
  ledcWriteTone(AMP_SIG_PIN, sndCur[sndIdx].freq);
  sndNoteStart = millis();
}
 
// ================================ OLED ================================
#if OLED_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(OLED_UPSIDEDOWN ? U8G2_R2 : U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(OLED_UPSIDEDOWN ? U8G2_R2 : U8G2_R0, U8X8_PIN_NONE);
#endif
bool oledOk = false;
const char *resetText = "";
bool badReset = false;
 
bool probeI2C(uint8_t sda, uint8_t scl, uint8_t &addr) {
  Wire.begin(sda, scl, 100000);
  for (uint8_t a : {0x3C, 0x3D}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { addr = a; Wire.end(); return true; }
  }
  Wire.end();
  return false;
}
 
void initOled() {
  uint8_t addr = 0, sda = I2C_SDA, scl = I2C_SCL;
  if (!probeI2C(sda, scl, addr)) {
    if (probeI2C(I2C_SCL, I2C_SDA, addr)) {
      sda = I2C_SCL; scl = I2C_SDA;
      Serial.printf("OLED found with SDA/SCL swapped: SDA=GPIO%d SCL=GPIO%d - fix I2C_SDA/I2C_SCL\n", sda, scl);
    } else {
      Serial.println("WARNING: no OLED at 0x3C/0x3D on GPIO6/7 - running without display");
      return;
    }
  }
  u8x8_SetPin(oled.getU8x8(), U8X8_PIN_I2C_DATA, sda);
  u8x8_SetPin(oled.getU8x8(), U8X8_PIN_I2C_CLOCK, scl);
  oled.setI2CAddress(addr << 1);
  oled.setBusClock(400000);
  oledOk = oled.begin();
  Serial.printf("OLED at 0x%02X (SDA=%d SCL=%d)\n", addr, sda, scl);
}
 
void drawCentered(const char *s, int y) {
  oled.drawStr((128 - oled.getStrWidth(s)) / 2, y, s);
}
 
// Bottom strip: 4 sensor bars (filled = dark) and a line-position marker.
void drawSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x = 8 + i * 30, w = 22, h = 18, y = 44;
    oled.drawFrame(x, y, w, h);
    int fill = calValid ? (int)(darkness[i] * (h - 2)) : constrain(mv[i] * (h - 2) / 1500, 0, h - 2);
    if (fill > 0) oled.drawBox(x + 1, y + h - 1 - fill, w - 2, fill);
  }
  if (calValid && lineSeen) {
    int x = 64 + (int)(linePos / POS_MAX * 52);
    oled.drawTriangle(x - 4, 36, x + 4, 36, x, 42);
  }
}
 
void updateDisplay() {
  if (!oledOk) return;
  char buf[32];
  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB10_tr);
 
  switch (state) {
    case ST_COUNTDOWN:
      drawCentered("BATMOBILE", 12);
      oled.setFont(u8g2_font_6x10_tr);
      if (badReset) { snprintf(buf, sizeof buf, "RESET: %s", resetText); drawCentered(buf, 24); }
      else drawCentered("Place on line", 24);
      oled.setFont(u8g2_font_logisoso16_tn);
      snprintf(buf, sizeof buf, "%d", (int)countdownSec);
      drawCentered(buf, 44);
      break;
    case ST_CALIBRATING:
      drawCentered(calValid && calPhase == 2 ? "Cal OK" : "Calibrating", 12);
      oled.setFont(u8g2_font_6x10_tr);
      drawCentered(calValid && calPhase == 2 ? "Centering on line" : "Sweeping...", 26);
      drawSensors();
      break;
    case ST_RUNNING: {
      const char *where;
      if (allBlack())                          where = "All black!";
      else if (fabsf(linePos) < CENTER_TOL)    where = "Centered";
      else if (linePos < 0)                    where = "Line LEFT  <-";
      else                                     where = "Line RIGHT ->";
      drawCentered(allBlack() ? "ALL BLACK" : "ON LINE", 12);
      oled.setFont(u8g2_font_6x10_tr);
      snprintf(buf, sizeof buf, "%s L%d R%d", where, (int)outL, (int)outR);
      drawCentered(buf, 26);
      drawSensors();
      break;
    }
    case ST_SEARCHING:
      drawCentered("LINE LOST", 12);
      oled.setFont(u8g2_font_6x10_tr);
      drawCentered(fabsf(lastPos) < SEARCH_CENTER_BAND ? "Searching: ahead"
                   : lastPos < 0 ? "Searching: <- left" : "Searching: right ->", 26);
      drawSensors();
      break;
    case ST_MOTOR_TEST:  drawCentered("Motor test", 12); break;
    case ST_IDLE:
      drawCentered("Debug mode", 12);
      oled.setFont(u8g2_font_6x10_tr);
      drawCentered("UART 115200: help", 26);
      drawSensors();
      break;
    case ST_STOPPED:
    case ST_ERROR:
      drawCentered(state == ST_ERROR ? "ERROR" : "Stopped", 12);
      oled.setFont(u8g2_font_6x10_tr);
      drawCentered(STOP_TEXT[stopReason], 26);
      if (stopReason == SR_CAL_FAIL) {
        snprintf(buf, sizeof buf, "%d %d %d %d mV", spreadMv[0], spreadMv[1], spreadMv[2], spreadMv[3]);
        drawCentered(buf, 38);
      } else {
        drawSensors();
      }
      break;
  }
  oled.sendBuffer();   // ~25 ms of I2C at 400 kHz - fine, the control task preempts it
}
 
// ================================ RGB LED ================================
void updateLed() {
#if USE_RGB_LED && defined(RGB_BUILTIN)
  static int last = -1;
  if (state == last) return;
  last = state;
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case ST_COUNTDOWN:   r = 20; g = 20; break;   // yellow
    case ST_CALIBRATING: b = 30; break;          // blue
    case ST_RUNNING:     g = 30; break;          // green
    case ST_SEARCHING:   r = 30; g = 10; break;  // orange
    case ST_STOPPED:     r = 30; break;          // red
    case ST_ERROR:       r = 30; b = 30; break;  // magenta
    default:             r = g = b = 8; break;   // dim white
  }
  rgbLedWrite(RGB_BUILTIN, r, g, b);
#endif
}
 
// ================================ SERIAL / EVENTS ================================
bool telemetry = false;
 
void sendCmd(Cmd c) { uint8_t v = c; xQueueSend(cmdQueue, &v, 0); }
 
void printCal() {
  Serial.println("Sensor  pin  black_mV  white_mV  spread");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int lo = calValid ? blackMv[i] : lastSweepLo[i], hi = calValid ? whiteMv[i] : lastSweepHi[i];
    Serial.printf("  S%d   %2d    %5d     %5d    %5d %s\n", i + 1, SENSOR_PINS[i], lo, hi, hi - lo,
                  !SENSOR_ENABLED[i] ? "(disabled)" : hi - lo < MIN_SPREAD_MV ? "<-- too small: excluded" : "");
  }
}
 
void printHelp() {
  Serial.println("Commands: debug go run stop cal motors log show | kp X ki X kd X base X");
}
 
void handleLine(char *s) {
  float v;
  if      (!strcmp(s, "debug"))  { sendCmd(CMD_IDLE); Serial.println("Autostart cancelled - idle."); }
  else if (!strcmp(s, "go"))     sendCmd(CMD_GO);
  else if (!strcmp(s, "run"))    sendCmd(CMD_RUN);
  else if (!strcmp(s, "stop"))   sendCmd(CMD_STOP);
  else if (!strcmp(s, "cal"))    sendCmd(CMD_CAL);
  else if (!strcmp(s, "motors")) { Serial.println("Motor test - wheels off the ground!"); sendCmd(CMD_MOTOR_TEST); }
  else if (!strcmp(s, "log"))    { telemetry = !telemetry; if (telemetry) Serial.println("ms state S1 S2 S3 S4 [pattern] pos L R"); }
  else if (!strcmp(s, "show"))   {
    Serial.printf("state=%s base=%d kp=%.4f ki=%.4f kd=%.5f\n", STATE_NAMES[state], (int)baseSpeed, kp, ki, kd);
    printCal();
  }
  else if (sscanf(s, "kp %f", &v) == 1)   { kp = v; Serial.printf("kp=%.4f\n", v); }
  else if (sscanf(s, "ki %f", &v) == 1)   { ki = v; Serial.printf("ki=%.4f\n", v); }
  else if (sscanf(s, "kd %f", &v) == 1)   { kd = v; Serial.printf("kd=%.5f\n", v); }
  else if (sscanf(s, "base %f", &v) == 1) { baseSpeed = constrain((int)v, 0, 255); Serial.printf("base=%d\n", (int)baseSpeed); }
  else if (s[0]) { Serial.printf("Unknown '%s'. ", s); printHelp(); }
}
 
// Commands end at Enter, or after 300 ms of silence (for "No line ending" monitors).
void handleSerial() {
  static char buf[32];
  static uint8_t n = 0;
  static uint32_t lastChar = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\r' || c == '\n') { buf[n] = 0; handleLine(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = tolower(c);
  }
  if (n && millis() - lastChar > 300) { buf[n] = 0; handleLine(buf); n = 0; }
}
 
void handleEvents() {
  uint8_t e;
  while (xQueueReceive(eventQueue, &e, 0) == pdTRUE) {
    switch (e) {
      case EV_TICK:      Serial.printf("Starting in %d...\n", (int)countdownSec); playSound(SND_TICK); break;
      case EV_CAL_START: Serial.println("Calibrating (sweeping across the line)..."); playSound(SND_CAL); break;
      case EV_CAL_RETRY: Serial.println("Calibration spread too small - retrying with a wider sweep:"); printCal(); break;
      case EV_CAL_OK:    Serial.println("Calibration OK:"); printCal(); break;
      case EV_CAL_FAIL:  Serial.println("CALIBRATION FAILED - is the car on the line? sensor wiring? CAL_SWEEP_MS?"); printCal(); playSound(SND_ERROR); break;
      case EV_GO:        Serial.println("--- GO ---"); playSound(SND_GO); break;
      case EV_LOST:      Serial.println("Line lost - searching"); playSound(SND_LOST); break;
      case EV_FOUND:     Serial.println("Line found"); break;
      case EV_NO_CAL:    Serial.println("No calibration yet - use 'go' or 'cal'"); break;
      case EV_STOPPED:
        Serial.printf("--- STOP: %s ---\n", STOP_TEXT[stopReason]);
        if (stopReason != SR_CAL_FAIL) playSound(stopReason == SR_CENTER_FAIL ? SND_ERROR : SND_STOP);
        break;
      case EV_MT_STEP: {
        static const char *names[] = {"LEFT forward", "stop", "LEFT reverse", "stop",
                                      "RIGHT forward", "stop", "RIGHT reverse", "stop",
                                      "BOTH forward", "stop", "BOTH reverse", "stop"};
        if (motorTestStep >= 0 && motorTestStep < 12) Serial.printf("Motor test: %s\n", names[motorTestStep]);
        break;
      }
    }
  }
}
 
void printTelemetry() {
  char pat[NUM_SENSORS + 1];
  for (int i = 0; i < NUM_SENSORS; i++) pat[i] = !calValid || !sensorOk[i] ? '-' : isBlack[i] ? '1' : '0';
  pat[NUM_SENSORS] = 0;
  Serial.printf("%lu %s %d %d %d %d [%s] %s L%d R%d\n", (unsigned long)millis(), STATE_NAMES[state],
                mv[0], mv[1], mv[2], mv[3], pat,
                lineSeen ? String((int)linePos).c_str() : "none", (int)outL, (int)outR);
}
 
void checkBootButton() {
#if USE_BOOT_BUTTON
  static bool last = HIGH;
  static uint32_t lastChange = 0;
  bool now = digitalRead(BOOT_BTN_PIN);
  if (now != last && millis() - lastChange > 40) {
    lastChange = millis();
    last = now;
    if (now == LOW) sendCmd(CMD_TOGGLE);
  }
#endif
}
 
const char *resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_PANIC:    return "CRASH";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "WATCHDOG";
    case ESP_RST_SW:       return "software";
    case ESP_RST_EXT:      return "reset pin";
    default:               return "other";
  }
}
 
// ================================ SETUP / LOOP ================================
void setup() {
  // Motors first, held LOW, so the wheels can't twitch during boot.
  // pinMode() also hands GPIO12 over from the USB PHY to normal GPIO.
  const uint8_t motorPins[4] = {L_IN1, L_IN2, R_IN1, R_IN2};
  const uint8_t motorCh[4]   = {CH_L1, CH_L2, CH_R1, CH_R2};
  for (int i = 0; i < 4; i++) { pinMode(motorPins[i], OUTPUT); digitalWrite(motorPins[i], LOW); }
 
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    if (!ledcAttachChannel(motorPins[i], PWM_FREQ, PWM_BITS, motorCh[i]))
      Serial.printf("ERROR: PWM attach failed on GPIO%d\n", motorPins[i]);
    ledcWrite(motorPins[i], 0);
  }
 
  esp_reset_reason_t rr = esp_reset_reason();
  resetText = resetReasonText(rr);
  badReset = rr == ESP_RST_BROWNOUT || rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT ||
             rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT;
  Serial.printf("\n=== Batmobile line follower === reset reason: %s\n", resetText);
  if (rr == ESP_RST_BROWNOUT)
    Serial.println("!!! BROWNOUT: the 3V3 rail dipped (likely motor current). See power notes.");
 
  // Amplifier: power it from GPIO21 at max drive strength, tone on GPIO20
  pinMode(AMP_VDD_PIN, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)AMP_VDD_PIN, GPIO_DRIVE_CAP_3);
  digitalWrite(AMP_VDD_PIN, HIGH);
  if (!ledcAttachChannel(AMP_SIG_PIN, 2000, 10, CH_TONE)) Serial.println("ERROR: tone PWM attach failed");
  ledcWrite(AMP_SIG_PIN, 0);
 
  analogReadResolution(12);
  for (int i = 0; i < NUM_SENSORS; i++) analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);  // 0 - ~3.1 V
 
#if USE_BOOT_BUTTON
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
#endif
 
  initOled();
  printHelp();
  Serial.println("Type 'debug' within 5 s to cancel the autostart.");
 
  cmdQueue = xQueueCreate(8, sizeof(uint8_t));
  eventQueue = xQueueCreate(16, sizeof(uint8_t));
 
  countdownEnd = millis() + AUTOSTART_DELAY_MS;
  countdownSec = AUTOSTART_DELAY_MS / 1000;
  state = ST_COUNTDOWN;
  playSound(SND_BOOT);
 
  // Priority 5 > loop() (priority 1): steering always wins over display/serial work.
  xTaskCreate(controlTask, "control", 4096, nullptr, 5, nullptr);
}
 
void loop() {
  handleSerial();
  checkBootButton();
  handleEvents();
  updateSound();
  updateLed();
 
  static uint32_t lastDisp = 0, lastTele = 0;
  if (millis() - lastDisp >= 150) { lastDisp = millis(); updateDisplay(); }
  if (telemetry && millis() - lastTele >= 50) { lastTele = millis(); printTelemetry(); }
  delay(2);
}
 