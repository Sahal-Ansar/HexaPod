# Arduino Mega Firmware

The Mega is the **motor controller / sensor hub**. It does no kinematics — it
just executes the servo targets the Pi sends and streams sensors back.

```
arduino/
└── hexapod/
    └── hexapod.ino   # main sketch (added in a later stage)
```

> Arduino requires the sketch's `.ino` file to live in a folder of the **same
> name** (`hexapod/hexapod.ino`), which is why the firmware lives one level down.

## Responsibilities
- Initialise I2C + both PCA9685 PWM boards and drive all 18 MG996R servos.
- Parse Pi → Arduino servo-command packets (framed + checksummed).
- Read the HC-SR04 ultrasonic, MPU6050 IMU, and 6 foot limit switches.
- Stream a telemetry packet (distance, roll/pitch, contact bits) back each loop.

## Libraries (install via Arduino IDE / arduino-cli)
- `Adafruit PWM Servo Driver Library` (PCA9685)
- `Adafruit MPU6050` + `Adafruit Unified Sensor` (IMU)
- `Wire` (I2C, bundled with the IDE)
