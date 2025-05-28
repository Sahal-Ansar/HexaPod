# HexaPod 🕷️

An autonomous six-legged robot. A **Raspberry Pi** runs the "brain" (inverse
kinematics, a tripod gait engine, IMU body-leveling, and obstacle avoidance) in
Python, and an **Arduino Mega** runs the "spine" (firmware that drives 18
servos and reads the sensors). The two talk over a simple framed serial
protocol.

The goal: the robot walks on its own with a proper tripod gait, keeps its body
roughly level on uneven ground using the IMU, knows when each foot touches down
via limit switches, and stops-and-turns away from obstacles it sees with the
ultrasonic sensor.

![HexaPod](HexaPod.jpg)

> **Status:** under construction. This codebase is being built up in small,
> reviewable stages — see the roadmap below.

---

## Hardware

| Role                | Part                          | Notes                                              |
| ------------------- | ----------------------------- | -------------------------------------------------- |
| Brain               | Raspberry Pi (Python)         | IK, gait, perception, state machine, serial master |
| Motor controller    | Arduino Mega (C++/Arduino)    | Serial slave: drives servos, reads sensors         |
| Servos              | 18× MG996R                    | 6 legs × 3 DOF (coxa, femur, tibia)                |
| Servo drivers       | 2× PCA9685 (I2C, 16-ch PWM)   | 9 servos per board                                 |
| Foot contact        | 6× limit switch               | one under each leg — detects touchdown / stance    |
| Distance sensor     | 1× HC-SR04 ultrasonic         | front-facing, for obstacle avoidance               |
| Orientation         | 1× MPU6050 IMU                | accel + gyro → roll / pitch for body leveling      |
| Link                | USB serial (Pi ↔ Mega)        | Pi sends 18 servo targets/tick; Mega streams sensors |

Each leg has **3 degrees of freedom**:

```
        coxa (yaw, horizontal swing)
          │
          ●──────► femur (lift / thigh)
                     │
                     ●──────► tibia (knee / shin) ──► foot
```

---

## Repository layout

```
HexaPod/
├── pi/                 # Raspberry Pi brain (Python)
│   ├── config.py           # robot geometry, servo limits, tuning  (stage 2)
│   ├── math_utils.py       # vector / angle helpers                (stage 3)
│   ├── kinematics/         # single-leg IK & FK, body kinematics
│   ├── comms/              # serial protocol, servo map, serial link
│   ├── gait/               # foot trajectories, tripod scheduler, engine
│   ├── control/            # control loop, stand-up, state machine
│   └── perception/         # obstacle detection / sensor filtering
├── arduino/
│   └── hexapod/            # Arduino Mega firmware (hexapod.ino)
├── tests/              # laptop-runnable unit tests (IK/FK, gait)
├── docs/               # protocol spec, architecture diagram, notes
└── requirements.txt    # Pi Python dependencies
```

---

## Architecture (high level)

```
            ┌──────────────────────────┐        servo targets (18)
            │      Raspberry Pi         │  ───────────────────────────►  ┌───────────────┐
            │  ───────────────────────  │                                │ Arduino Mega  │
            │  state machine            │  ◄───────────────────────────  │               │
            │  gait engine → IK         │     telemetry (distance,        │  2× PCA9685   │──► 18 servos
            │  perception / leveling    │      roll/pitch, 6 contacts)    │  HC-SR04      │
            └──────────────────────────┘          USB serial             │  MPU6050      │
                                                                         │  6× switch    │
                                                                         └───────────────┘
```

The Pi decides **where each foot should be**; the Mega makes it happen and
reports **what the body is feeling**. (Detailed diagram, wiring, and the serial
protocol spec are filled in as the build progresses.)

---

## Build roadmap

The project is built in 30 small stages, grouped as:

1. **Foundations** — repo scaffold, robot config, math helpers.
2. **Kinematics** — single-leg IK/FK (with round-trip tests) and body kinematics.
3. **Servo + comms** — serial protocol, joint→PWM mapping, serial link (with a mock mode).
4. **Arduino firmware** — servo driver, command parser, HC-SR04, MPU6050, limit switches, telemetry.
5. **Gait engine** — swing/stance trajectories, tripod scheduler, full gait engine.
6. **Bring-up** — control loop, velocity commands, stand-up/sit-down.
7. **Sensor feedback + autonomy** — IMU leveling, contact feedback, obstacle avoidance, state machine.
8. **Polish** — entry point, config loading, teleop, and full docs.

---

## Getting started

> Detailed setup, wiring, flashing, and run instructions are added in the final
> documentation stage. In short:

```bash
# On the Raspberry Pi (or your laptop for offline / mock testing)
python -m pip install -r requirements.txt
```

Most of the kinematics and gait logic is testable on a laptop with no hardware
attached via the included test/print harnesses and the serial link's mock mode.
