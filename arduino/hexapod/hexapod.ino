/*
 * HexaPod — Arduino Mega firmware (motor controller / sensor hub)
 * ============================================================================
 * The Mega is the robot's "spine": it owns the hardware and does NO kinematics.
 * It receives 18 servo pulse targets from the Raspberry Pi over USB serial and
 * drives the servos, and it streams sensor telemetry (ultrasonic distance, IMU
 * roll/pitch, foot-contact switches) back to the Pi.
 *
 * Main loop each pass (all non-blocking):
 *   1. parse incoming servo-command frames -> drive the 18 servos
 *   2. run the ultrasonic ranging (interrupt-timed echo)
 *   3. run the IMU complementary filter (roll/pitch)
 *   4. debounce the foot-contact switches
 *   5. stream a telemetry frame back to the Pi at 50 Hz
 *   6. blink a heartbeat LED
 *
 * ── Hardware / wiring ───────────────────────────────────────────────────────
 *   Servos      : 18× MG996R, 9 per PCA9685 board (channels 0..8 used).
 *   PCA9685 #0  : I2C address 0x40 (no address jumpers) — drives legs L1,L2,L3.
 *   PCA9685 #1  : I2C address 0x41 (A0 jumper bridged)  — drives legs R1,R2,R3.
 *   I2C bus     : Mega SDA = pin 20, SCL = pin 21 (shared by both boards).
 *   Servo power : PCA9685 V+ from a separate 5–6 V / >=10 A supply. The logic
 *                 5 V comes from the Mega; ALL grounds must be common (Mega GND,
 *                 both PCA9685 GND, and the servo supply GND tied together).
 *
 * ── Build ───────────────────────────────────────────────────────────────────
 *   Libraries (Arduino IDE Library Manager, or `arduino-cli lib install`):
 *     - "Adafruit PWM Servo Driver Library"
 *     - "Adafruit MPU6050" + "Adafruit Unified Sensor"
 *   Compile/upload (CLI):
 *     arduino-cli compile --fqbn arduino:avr:mega arduino/hexapod
 *     arduino-cli upload  --fqbn arduino:avr:mega -p <PORT> arduino/hexapod
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "protocol.h"  // shared Pi<->Arduino frame format + CRC

// ── Configuration ───────────────────────────────────────────────────────────
static const uint8_t  PCA0_ADDR  = 0x40;   // board 0 (left legs)
static const uint8_t  PCA1_ADDR  = 0x41;   // board 1 (right legs)
static const uint16_t SERVO_FREQ = 50;     // analog servos want ~50 Hz
static const uint32_t SERIAL_BAUD = 115200;

// The PCA9685's internal oscillator is nominally 25 MHz but varies part-to-part;
// 27 MHz is the value Adafruit recommends measuring/using so the requested
// 50 Hz (and therefore our microsecond pulse widths) come out accurate.
static const uint32_t PCA_OSC_HZ = 27000000;

// ── Servo addressing & pulse limits (must match the Pi protocol) ────────────
static const uint8_t  NUM_LEGS         = 6;
static const uint8_t  NUM_SERVOS       = 18;   // 6 legs × 3 joints
static const uint8_t  SERVOS_PER_BOARD = 9;    // global index < 9 -> board 0
static const uint16_t SERVO_MIN_US     = 500;  // hard safety clamp (stops)
static const uint16_t SERVO_MAX_US     = 2500;
static const uint16_t NEUTRAL_PULSE_US = 1500; // boot-safe "all centred" pose

// ── HC-SR04 ultrasonic (front-facing) ───────────────────────────────────────
static const uint8_t  SONAR_TRIG_PIN    = 7;     // any digital output
static const uint8_t  SONAR_ECHO_PIN    = 2;     // MUST be interrupt-capable
                                                 //   (Mega: 2,3,18,19 — NOT the
                                                 //   I2C pins 20/21)
static const uint16_t SONAR_PING_MS     = 50;    // re-trigger every 50 ms (~20 Hz)
static const uint16_t SONAR_TIMEOUT_MS  = 30;    // no echo in 30 ms => out of range
static const uint32_t SONAR_MAX_ECHO_US = 25000; // ~4.3 m ceiling

// ── MPU6050 IMU (body tilt) ─────────────────────────────────────────────────
static const uint16_t IMU_UPDATE_MS = 10;     // run the filter at ~100 Hz
static const float    COMP_ALPHA    = 0.98f;  // complementary-filter weight

// ── Foot-contact limit switches (one per leg, indexed by LegId 0..5) ────────
// Wired pin -> GND and read with the internal pull-up, so an open (foot up)
// switch reads HIGH and a closed (foot planted) switch reads LOW.
static const uint8_t  FOOT_PINS[NUM_LEGS] = {30, 31, 32, 33, 34, 35};
static const uint16_t FOOT_DEBOUNCE_MS    = 5;  // ignore contact bounce shorter than this

// ── Telemetry ───────────────────────────────────────────────────────────────
static const uint16_t TELEMETRY_PERIOD_MS = 20;  // stream back to the Pi at 50 Hz

// Two driver objects, one per board.
Adafruit_PWMServoDriver pca0 = Adafruit_PWMServoDriver(PCA0_ADDR);
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(PCA1_ADDR);

// IMU object + filtered body angles (degrees). roll = tilt about +X (forward),
// pitch = tilt about +Y (left), matching the Pi's body-frame convention.
Adafruit_MPU6050 mpu;
static bool     imuOk        = false;
static float    imuRollDeg   = 0.0f;
static float    imuPitchDeg  = 0.0f;
static uint32_t imuLastUs    = 0;   // micros() of last filter step (for dt)
static uint32_t imuLastMs    = 0;   // millis() of last filter step (throttle)

// Last microsecond pulse commanded to each servo (index 0..17). Kept so we can
// inspect/echo current state and so a future failsafe knows the last pose.
static uint16_t servoUs[NUM_SERVOS];

// Foot-contact debounce state, per leg.
static uint8_t  footRaw[NUM_LEGS];        // most recent raw reading (1 = pressed)
static uint8_t  footStable[NUM_LEGS];     // debounced contact state (1 = planted)
static uint32_t footChangeMs[NUM_LEGS];   // when footRaw last changed

// Heartbeat LED state.
static uint32_t lastBlinkMs = 0;
static bool     ledOn       = false;

// Telemetry scheduling.
static uint32_t lastTelemetryMs = 0;

// Timestamp (ms) of the last valid servo command — a later stage uses this to
// failsafe (hold/relax) if the Pi stops talking.
static uint32_t lastCommandMs = 0;

// ── Incoming-frame parser state machine ─────────────────────────────────────
// Serial bytes dribble in a few at a time, so we parse incrementally in loop()
// (never blocking). Mirrors the Pi's FrameParser: lock onto the sync bytes,
// read TYPE/LEN/PAYLOAD, verify CRC, then dispatch.
enum RxState {
  RX_SYNC0, RX_SYNC1, RX_TYPE, RX_LEN, RX_PAYLOAD, RX_CRC
};
static const uint8_t RX_MAX_PAYLOAD = 64;  // >= largest payload (36)
static uint8_t  rxState   = RX_SYNC0;
static uint8_t  rxType    = 0;
static uint8_t  rxLen     = 0;
static uint8_t  rxIndex   = 0;
static uint8_t  rxCrc     = 0;             // running CRC over TYPE/LEN/PAYLOAD
static uint8_t  rxBuf[RX_MAX_PAYLOAD];

// Initialise one PCA9685 board to a known, accurate 50 Hz state.
static void initPca(Adafruit_PWMServoDriver &pca) {
  pca.begin();
  pca.setOscillatorFrequency(PCA_OSC_HZ);
  pca.setPWMFreq(SERVO_FREQ);
}

// Convert a servo pulse width (microseconds) to a PCA9685 12-bit "off" count.
// The chip divides each PWM period into 4096 ticks, so:
//   period_us = 1e6 / SERVO_FREQ           (20000 us at 50 Hz)
//   ticks     = us / period_us * 4096      = us * 4096 * SERVO_FREQ / 1e6
// e.g. 1500 us -> 307 ticks (= 1499 us back), matching Adafruit's servo range.
static inline uint16_t usToTicks(uint16_t us) {
  uint32_t ticks = ((uint32_t)us * 4096UL * SERVO_FREQ) / 1000000UL;
  if (ticks > 4095UL) ticks = 4095UL;
  return (uint16_t)ticks;
}

// Drive one servo by global index (0..17). Maps index -> (board, channel) with
// the same rule the Pi uses (board = index / 9, channel = index % 9) and clamps
// the pulse to the safe window as a last line of defence against bad commands.
static void writeServoUs(uint8_t index, uint16_t us) {
  if (index >= NUM_SERVOS) return;
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  servoUs[index] = us;
  const uint16_t ticks = usToTicks(us);
  if (index < SERVOS_PER_BOARD) {
    pca0.setPWM(index, 0, ticks);
  } else {
    pca1.setPWM(index - SERVOS_PER_BOARD, 0, ticks);
  }
}

// Move every servo to the boot-safe centred pose. Staggered with a small delay
// so 18 servos don't all surge at once and brown out the supply. This runs once
// at boot; the Pi commands the real calibrated stance after it connects.
static void startupNeutralPose() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    writeServoUs(i, NEUTRAL_PULSE_US);
    delay(15);  // boot-only stagger; the runtime command path never delays
  }
}

// Apply a validated servo-command frame: 18 little-endian uint16 microsecond
// pulses, in global servo-index order, straight to the servos.
static void handleServoFrame(const uint8_t *payload, uint8_t len) {
  if (len != SERVO_PAYLOAD_LEN) return;  // wrong size for this type — ignore
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    const uint16_t us = (uint16_t)payload[2 * i] | ((uint16_t)payload[2 * i + 1] << 8);
    writeServoUs(i, us);
  }
  lastCommandMs = millis();
}

// Dispatch a CRC-validated frame by type.
static void handleFrame(uint8_t type, const uint8_t *payload, uint8_t len) {
  switch (type) {
    case MSG_SERVO: handleServoFrame(payload, len); break;
    default: break;  // unknown/other types ignored for forward-compatibility
  }
}

// Feed all currently-available serial bytes through the parser state machine.
static void parseSerial() {
  while (Serial.available() > 0) {
    const uint8_t b = (uint8_t)Serial.read();
    switch (rxState) {
      case RX_SYNC0:
        if (b == SYNC0) rxState = RX_SYNC1;
        break;
      case RX_SYNC1:
        // Accept AA..AA55 runs; any other byte means this wasn't a frame start.
        if (b == SYNC1)      rxState = RX_TYPE;
        else if (b == SYNC0) rxState = RX_SYNC1;
        else                 rxState = RX_SYNC0;
        break;
      case RX_TYPE:
        rxType = b;
        rxCrc = crc8_update(0, b);  // CRC starts at TYPE
        rxState = RX_LEN;
        break;
      case RX_LEN:
        rxLen = b;
        rxCrc = crc8_update(rxCrc, b);
        rxIndex = 0;
        if (rxLen > RX_MAX_PAYLOAD) rxState = RX_SYNC0;       // bogus length: resync
        else if (rxLen == 0)        rxState = RX_CRC;
        else                        rxState = RX_PAYLOAD;
        break;
      case RX_PAYLOAD:
        rxBuf[rxIndex++] = b;
        rxCrc = crc8_update(rxCrc, b);
        if (rxIndex >= rxLen) rxState = RX_CRC;
        break;
      case RX_CRC:
        if (b == rxCrc) handleFrame(rxType, rxBuf, rxLen);
        rxState = RX_SYNC0;  // good or bad, hunt for the next frame
        break;
    }
  }
}

// ── HC-SR04: interrupt-driven, non-blocking distance measurement ────────────
// pulseIn() would block the loop for up to ~23 ms per ping — far too long for a
// 20 ms control tick. Instead we fire a 10 us trigger pulse on a timer and let a
// pin-change interrupt time the echo, so the main loop is never stalled.
volatile uint32_t sonarRiseUs  = 0;
volatile uint16_t sonarDistMm  = DISTANCE_NO_ECHO;
volatile bool     sonarRising  = false;
volatile bool     sonarGotEcho = true;   // true => not currently awaiting an echo
static   uint32_t lastSonarTrig = 0;     // loop time of last trigger
static   uint32_t sonarPingMs   = 0;     // when the outstanding ping was sent

// Echo pin CHANGE interrupt: stamp the rising edge, time the width on falling.
// distance_mm = duration_us * 10 / 58  (datasheet: 58 us of round trip per cm).
void sonarEchoIsr() {
  if (digitalRead(SONAR_ECHO_PIN)) {
    sonarRiseUs = micros();
    sonarRising = true;
  } else if (sonarRising) {
    const uint32_t dur = micros() - sonarRiseUs;
    sonarRising = false;
    sonarDistMm = (dur <= SONAR_MAX_ECHO_US) ? (uint16_t)((dur * 10UL) / 58UL)
                                             : DISTANCE_NO_ECHO;
    sonarGotEcho = true;
  }
}

static void sonarTrigger() {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);          // 10 us trigger; negligible vs. the loop
  digitalWrite(SONAR_TRIG_PIN, LOW);
  sonarGotEcho = false;
  sonarPingMs = millis();
}

// Atomic read — a 16-bit load isn't atomic on the 8-bit AVR, so guard it.
static uint16_t sonarReadMm() {
  uint16_t v;
  noInterrupts();
  v = sonarDistMm;
  interrupts();
  return v;
}

// Call every loop: schedule pings and force "no echo" if one never returns.
static void sonarUpdate() {
  const uint32_t now = millis();
  if (now - lastSonarTrig >= SONAR_PING_MS) {
    lastSonarTrig = now;
    sonarTrigger();
  }
  if (!sonarGotEcho && (now - sonarPingMs) > SONAR_TIMEOUT_MS) {
    noInterrupts();
    sonarDistMm = DISTANCE_NO_ECHO;   // nothing in range / no surface to bounce off
    sonarGotEcho = true;
    interrupts();
  }
}

// ── MPU6050: complementary filter for roll & pitch ──────────────────────────
// The accelerometer gives an absolute tilt but is noisy and corrupted by the
// robot's own motion; the gyro gives a clean rate but its integral drifts. The
// complementary filter trusts the gyro over the short term and gently pulls
// back toward the accelerometer over the long term:
//   angle = A*(angle + gyro_rate*dt) + (1-A)*accel_angle      (A ≈ 0.98)
// It is the cheap, rock-solid choice for body leveling (a full Kalman filter is
// overkill for slow tilt).
static void imuUpdate() {
  const uint32_t nowMs = millis();
  if (!imuOk || (nowMs - imuLastMs) < IMU_UPDATE_MS) return;
  imuLastMs = nowMs;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  const uint32_t nowUs = micros();
  float dt = (nowUs - imuLastUs) * 1e-6f;
  imuLastUs = nowUs;
  if (dt <= 0.0f || dt > 0.5f) dt = IMU_UPDATE_MS * 1e-3f;  // guard 1st call/wrap

  // Absolute tilt from gravity (accel in m/s^2).
  const float ay = accel.acceleration.y, az = accel.acceleration.z;
  const float ax = accel.acceleration.x;
  const float rollAcc  = atan2f(ay, az) * RAD_TO_DEG;
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  // Angular rate from the gyro (Adafruit reports rad/s) -> deg/s.
  const float rollRate  = gyro.gyro.x * RAD_TO_DEG;
  const float pitchRate = gyro.gyro.y * RAD_TO_DEG;

  imuRollDeg  = COMP_ALPHA * (imuRollDeg  + rollRate  * dt) + (1.0f - COMP_ALPHA) * rollAcc;
  imuPitchDeg = COMP_ALPHA * (imuPitchDeg + pitchRate * dt) + (1.0f - COMP_ALPHA) * pitchAcc;
}

// ── Foot-contact limit switches: time-based debounce ────────────────────────
// A mechanical switch chatters for a few ms when it makes/breaks contact. We
// only accept a new state once the raw reading has held steady for
// FOOT_DEBOUNCE_MS, so a single touchdown registers as one clean event instead
// of a burst. Non-blocking: just timestamps, no delay().
static void footUpdate() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    const uint8_t raw = (digitalRead(FOOT_PINS[i]) == LOW) ? 1 : 0;  // LOW = pressed
    if (raw != footRaw[i]) {
      footRaw[i] = raw;
      footChangeMs[i] = now;            // reading changed -> restart debounce timer
    } else if ((now - footChangeMs[i]) >= FOOT_DEBOUNCE_MS) {
      footStable[i] = raw;              // held steady long enough -> commit it
    }
  }
}

// Pack the six debounced contact states into one byte: bit i = leg i planted.
static uint8_t footContactByte() {
  uint8_t b = 0;
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    if (footStable[i]) b |= (uint8_t)(1 << i);
  }
  return b;
}

// Build and transmit one telemetry frame to the Pi. Payload (little-endian,
// matching protocol.py decode_telemetry '<HhhB'):
//   uint16 distance_mm | int16 roll_cdeg | int16 pitch_cdeg | uint8 contacts
static void sendTelemetry() {
  const uint16_t dist = sonarReadMm();   // already DISTANCE_NO_ECHO if no echo
  // Round (not truncate) degrees -> centidegrees so the Pi recovers ±0.005°.
  const float    rollC  = imuRollDeg  * 100.0f;
  const float    pitchC = imuPitchDeg * 100.0f;
  const int16_t  roll   = (int16_t)(rollC  >= 0 ? rollC  + 0.5f : rollC  - 0.5f);
  const int16_t  pitch  = (int16_t)(pitchC >= 0 ? pitchC + 0.5f : pitchC - 0.5f);
  const uint8_t  contacts = footContactByte();

  uint8_t frame[2 + 2 + TELEMETRY_PAYLOAD_LEN + 1];
  frame[0] = SYNC0;
  frame[1] = SYNC1;
  frame[2] = MSG_TELEMETRY;
  frame[3] = TELEMETRY_PAYLOAD_LEN;
  frame[4] = (uint8_t)(dist  & 0xFF);
  frame[5] = (uint8_t)(dist  >> 8);
  frame[6] = (uint8_t)(roll  & 0xFF);
  frame[7] = (uint8_t)((roll  >> 8) & 0xFF);
  frame[8] = (uint8_t)(pitch & 0xFF);
  frame[9] = (uint8_t)((pitch >> 8) & 0xFF);
  frame[10] = contacts;
  // CRC over TYPE, LEN, PAYLOAD (frame[2..10]) — not the sync bytes.
  frame[11] = crc8(&frame[2], 2 + TELEMETRY_PAYLOAD_LEN);

  Serial.write(frame, sizeof(frame));
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();          // join I2C as master (Mega: SDA=20, SCL=21)
  Wire.setClock(400000); // 400 kHz fast-mode I2C: plenty for two boards at 50 Hz

  initPca(pca0);
  initPca(pca1);

  // Bring all servos to a known centred pose before anything else commands them.
  startupNeutralPose();

  // HC-SR04 wiring + echo interrupt.
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  pinMode(SONAR_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SONAR_ECHO_PIN), sonarEchoIsr, CHANGE);

  // MPU6050 IMU (shares the I2C bus, default address 0x68).
  imuOk = mpu.begin();
  if (imuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    imuLastUs = micros();
  } else {
    Serial.println(F("WARN: MPU6050 not found — roll/pitch will read 0"));
  }

  // Foot-contact limit switches (internal pull-ups; pressed = LOW).
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    pinMode(FOOT_PINS[i], INPUT_PULLUP);
    footRaw[i] = (digitalRead(FOOT_PINS[i]) == LOW) ? 1 : 0;
    footStable[i] = footRaw[i];
    footChangeMs[i] = millis();
  }

  // One-time boot banner (ASCII; contains no 0xAA, so the Pi's frame parser
  // simply skips it before the first binary telemetry frame arrives).
  Serial.println(F("HexaPod Mega firmware: stage 17 (telemetry streaming)"));
}

void loop() {
  // 1) Drain and parse any incoming servo commands (drives the servos).
  parseSerial();

  // 2) Keep the ultrasonic ranging in the background (non-blocking).
  sonarUpdate();

  // 3) Keep the IMU filter running (throttled to ~100 Hz internally).
  imuUpdate();

  // 4) Debounce the foot-contact switches.
  footUpdate();

  const uint32_t now = millis();

  // 5) Stream the telemetry packet (distance + roll/pitch + contacts) at 50 Hz.
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }

  // 6) Heartbeat: blink the on-board LED at 2 Hz to prove the loop is running.
  // Non-blocking (millis-based) so it never stalls serial handling. No text on
  // the serial line now — it carries binary telemetry frames only.
  if (now - lastBlinkMs >= 250) {
    lastBlinkMs = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }
}
