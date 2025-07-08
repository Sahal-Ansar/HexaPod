r"""Smooth stand-up and sit-down sequences.

At power-on the firmware parks every servo at a centred pose; before walking, the
robot must rise into its stance, and when done it should lower gracefully rather
than cut power and flop. Both are the *same* motion in two directions: change the
BODY HEIGHT while the feet stay planted. The legs fold to sit and extend to
stand — which body kinematics already expresses as a vertical body-pose offset:

    pose.z = (desired_body_height − stance_height)      (0 at full ride height)

We sweep that height from low (sit) to full (stand) over a fixed duration, with
**smoothstep easing** so the motion starts and ends at zero speed — no jerk at
either end. Each step is turned into 18 servo pulses, so a sequence is just a
list of pulse frames the loop streams out at the control rate.

There's also a generic pulse-space interpolator (`lerp_pulse_frames`) for the
very first move: easing from the firmware's centred boot pose to the sit pose,
so even the first motion after connecting is smooth.
"""

from __future__ import annotations

import time
from typing import Sequence

from pi.comms.serial_link import SerialLink
from pi.comms.servo_map import (
    DEFAULT_CALIBRATION,
    CalibrationTable,
    NEUTRAL_BOOT_PULSE,
    angles_to_pulses,
)
from pi.config import ROBOT, RobotConfig
from pi.kinematics.body import BodyPose, default_stance_body, solve_body
from pi.math_utils import clamp, lerp

# Body height (mm above the feet) when sitting. Low, but kept high enough that
# the deeply folded legs stay inside the reachable workspace. Tunable.
SIT_HEIGHT_MM = 35.0
# How long each transition takes, and how many frames that is at the control rate.
DEFAULT_DURATION_S = 1.5


def smoothstep(t: float) -> float:
    """Ease-in/ease-out on [0,1]: 3t^2 − 2t^3. Zero slope at both ends, so the
    motion accelerates and decelerates smoothly instead of snapping."""
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def body_height_pose(height_mm: float, config: RobotConfig = ROBOT) -> BodyPose:
    """The body pose that places the body ``height_mm`` above the planted feet."""
    return BodyPose(z=height_mm - config.stance_height_mm)


def stance_pulses_at_height(
    height_mm: float,
    config: RobotConfig = ROBOT,
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
) -> list[int]:
    """18 servo pulses for the default stance at a given body height."""
    feet = default_stance_body(config)
    angles = solve_body(feet, body_height_pose(height_mm, config), config)
    return angles_to_pulses(angles, calibration)


def _num_steps(duration_s: float, config: RobotConfig) -> int:
    return max(2, int(round(duration_s * config.control_hz)))


def height_frames(
    start_height_mm: float,
    end_height_mm: float,
    duration_s: float = DEFAULT_DURATION_S,
    config: RobotConfig = ROBOT,
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
) -> list[list[int]]:
    """Eased sequence of pulse frames sweeping body height start -> end."""
    steps = _num_steps(duration_s, config)
    frames: list[list[int]] = []
    for i in range(steps + 1):
        s = smoothstep(i / steps)
        h = lerp(start_height_mm, end_height_mm, s)
        frames.append(stance_pulses_at_height(h, config, calibration))
    return frames


def stand_up_frames(
    config: RobotConfig = ROBOT,
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
    sit_height_mm: float = SIT_HEIGHT_MM,
    duration_s: float = DEFAULT_DURATION_S,
) -> list[list[int]]:
    """Sit height -> full ride height (rise into the walking stance)."""
    return height_frames(sit_height_mm, config.stance_height_mm, duration_s, config, calibration)


def sit_down_frames(
    config: RobotConfig = ROBOT,
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
    sit_height_mm: float = SIT_HEIGHT_MM,
    duration_s: float = DEFAULT_DURATION_S,
) -> list[list[int]]:
    """Full ride height -> sit height (lower the body and rest)."""
    return height_frames(config.stance_height_mm, sit_height_mm, duration_s, config, calibration)


def lerp_pulse_frames(
    start_pulses: Sequence[int],
    end_pulses: Sequence[int],
    duration_s: float = DEFAULT_DURATION_S,
    config: RobotConfig = ROBOT,
    ease: bool = True,
) -> list[list[int]]:
    """Eased interpolation directly in pulse space between two poses. Used to
    glide from the firmware's centred boot pose to the first commanded pose."""
    steps = _num_steps(duration_s, config)
    frames: list[list[int]] = []
    for i in range(steps + 1):
        t = i / steps
        s = smoothstep(t) if ease else t
        frames.append([int(round(lerp(a, b, s))) for a, b in zip(start_pulses, end_pulses)])
    return frames


def boot_to_sit_frames(
    config: RobotConfig = ROBOT,
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
    sit_height_mm: float = SIT_HEIGHT_MM,
    duration_s: float = 1.0,
) -> list[list[int]]:
    """Glide from the all-centred boot pose to the sit pose (the first move)."""
    boot = [NEUTRAL_BOOT_PULSE] * 18
    sit = stance_pulses_at_height(sit_height_mm, config, calibration)
    return lerp_pulse_frames(boot, sit, duration_s, config)


def play(
    link: SerialLink,
    frames: Sequence[Sequence[int]],
    hz: float = ROBOT.control_hz,
    pace: bool = True,
) -> None:
    """Stream a pose sequence to the robot at ``hz`` (set pace=False in tests)."""
    period = 1.0 / hz
    for frame in frames:
        link.send_servos(frame)
        link.poll()  # keep the RX buffer drained
        if pace:
            time.sleep(period)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.standup`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    from pi.comms.serial_link import FakeArduino, open_mock
    from pi.comms.servo_map import SERVO_MAX_US, SERVO_MIN_US
    from pi.config import LEG_ORDER
    from pi.kinematics.body import foot_body_to_leg
    from pi.kinematics.leg_ik import is_reachable

    print("standup self-test")
    print("=" * 60)

    # smoothstep: pinned endpoints, midpoint, monotone, eased (slow) at ends.
    assert smoothstep(0.0) == 0.0 and smoothstep(1.0) == 1.0
    assert math.isclose(smoothstep(0.5), 0.5)
    assert smoothstep(0.1) < 0.1 and smoothstep(0.9) > 0.9  # S-curve
    assert all(smoothstep(i / 100) <= smoothstep((i + 1) / 100) for i in range(100))
    print("smoothstep eased, monotone, endpoints pinned ..... OK")

    # Every height across the sit..stand range is reachable (no IK clamping),
    # so the chosen SIT_HEIGHT is actually achievable.
    for h in range(int(SIT_HEIGHT_MM), int(ROBOT.stance_height_mm) + 1, 5):
        feet = default_stance_body()
        pose = body_height_pose(float(h))
        for leg in LEG_ORDER:
            pt = foot_body_to_leg(ROBOT.mount(leg), feet[leg], pose)
            assert is_reachable(tuple(pt)), f"height {h}mm unreachable for {leg.name}"
    print(f"sit..stand heights ({int(SIT_HEIGHT_MM)}..{int(ROBOT.stance_height_mm)}mm) "
          f"all reachable .. OK")

    # Stand-up sequence: starts at sit, ends at stand, all frames in range.
    up = stand_up_frames()
    sit_pulses = stance_pulses_at_height(SIT_HEIGHT_MM)
    stand_pulses = stance_pulses_at_height(ROBOT.stance_height_mm)
    assert up[0] == sit_pulses and up[-1] == stand_pulses
    assert all(len(f) == 18 for f in up)
    assert all(SERVO_MIN_US <= p <= SERVO_MAX_US for f in up for p in f)
    print(f"stand_up: {len(up)} frames, sit -> stand, all in range . OK")

    # Smoothness: no single servo jumps more than a small step between frames.
    max_jump = max(abs(up[i + 1][j] - up[i][j])
                   for i in range(len(up) - 1) for j in range(18))
    assert max_jump < 25, f"frame-to-frame servo jump too large: {max_jump}us"
    print(f"max per-frame servo step {max_jump}us (smooth) ......... OK")

    # Sit-down is the reverse motion.
    down = sit_down_frames()
    assert down[0] == stand_pulses and down[-1] == sit_pulses
    print("sit_down: stand -> sit (reverse) ................. OK")

    # Boot glide: from centred boot pose to sit, endpoints correct & smooth.
    boot = boot_to_sit_frames()
    assert boot[0] == [NEUTRAL_BOOT_PULSE] * 18
    assert boot[-1] == sit_pulses
    print("boot(1500us) -> sit glide endpoints correct ...... OK")

    # play() streams every frame to the (mock) robot.
    fake = FakeArduino()
    link = open_mock(fake)
    play(link, up, pace=False)
    assert fake.command_count == len(up)
    print(f"play() streamed all {len(up)} frames to the robot ..... OK")

    print("-" * 60)
    print("All standup self-tests passed.")


if __name__ == "__main__":
    _selftest()
