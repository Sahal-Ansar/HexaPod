"""Raspberry Pi 'brain' for the hexapod robot.

This package contains everything that runs on the Raspberry Pi:

    config        - robot geometry, servo limits, tuning constants
    math_utils    - small vector / angle helpers
    kinematics/   - single-leg IK & FK, body-pose kinematics
    comms/        - serial protocol, servo channel mapping, serial link
    gait/         - foot trajectories, tripod scheduler, gait engine
    control/      - main control loop, stand-up sequences, state machine
    perception/   - obstacle detection / sensor filtering

The Pi computes *where each foot should be* (inverse kinematics + gait) and
sends 18 servo angle targets per control tick to the Arduino Mega over USB
serial. The Arduino drives the servos and streams sensor telemetry back.
"""
