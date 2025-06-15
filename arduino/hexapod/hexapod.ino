/*
 * HexaPod — Arduino Mega firmware (motor controller / sensor hub)
 * ============================================================================
 * The Mega is the robot's "spine": it owns the hardware and does NO kinematics.
 * It receives 18 servo pulse targets from the Raspberry Pi over USB serial and
 * drives the servos, and it streams sensor telemetry (ultrasonic distance, IMU
 * roll/pitch, foot-contact switches) back to the Pi.
 *
 * This file is built up over several stages. STAGE 11 (this commit) is just the
 * hardware bring-up: I2C, both PCA9685 PWM boards, and the serial port, plus a
 * heartbeat LED so you can confirm the board is alive before anything else.
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
 *     - "Adafruit MPU6050" + "Adafruit Unified Sensor"   (added later stage)
 *   Compile/upload (CLI):
 *     arduino-cli compile --fqbn arduino:avr:mega arduino/hexapod
 *     arduino-cli upload  --fqbn arduino:avr:mega -p <PORT> arduino/hexapod
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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
static const uint8_t  NUM_SERVOS       = 18;   // 6 legs × 3 joints
static const uint8_t  SERVOS_PER_BOARD = 9;    // global index < 9 -> board 0
static const uint16_t SERVO_MIN_US     = 500;  // hard safety clamp (stops)
static const uint16_t SERVO_MAX_US     = 2500;
static const uint16_t NEUTRAL_PULSE_US = 1500; // boot-safe "all centred" pose

// Two driver objects, one per board.
Adafruit_PWMServoDriver pca0 = Adafruit_PWMServoDriver(PCA0_ADDR);
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(PCA1_ADDR);

// Last microsecond pulse commanded to each servo (index 0..17). Kept so we can
// inspect/echo current state and so a future failsafe knows the last pose.
static uint16_t servoUs[NUM_SERVOS];

// Heartbeat LED state.
static uint32_t lastBlinkMs = 0;
static bool     ledOn       = false;

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

  // Banner so you can see the firmware booted (open the Serial Monitor @115200).
  Serial.println(F("HexaPod Mega firmware: stage 12 (servo write + neutral pose)"));
}

void loop() {
  // Heartbeat: blink the on-board LED at 2 Hz to prove the loop is running.
  // Non-blocking (millis-based) so it never stalls future serial handling.
  const uint32_t now = millis();
  if (now - lastBlinkMs >= 250) {
    lastBlinkMs = now;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
  }
}
