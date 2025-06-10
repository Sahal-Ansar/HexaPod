"""Map IK joint angles → PCA9685 (board, channel, pulse) with calibration.

The kinematics speaks in *joint angles* (degrees, in the IK convention from
config.py). The servos speak in *PWM pulse width* (microseconds). They are NOT
the same thing, for two physical reasons this module exists to bridge:

  1. A servo's 0 µs reference and its degrees-per-microsecond depend on how the
     horn was splined on at assembly — every servo needs a per-unit trim.
  2. The left and right legs are mirror images, so a positive joint angle drives
     the left-side servo one way and the right-side servo the *opposite* way.
     That is captured by a per-servo direction sign (and a matching neutral
     pulse, since flipping direction around a near-end neutral would otherwise
     drive the pulse out of range).

For each servo the pulse is:

    pulse_us = neutral_us + direction · (angle_deg + trim_deg) · us_per_deg
               then clamped to [min_us, max_us]   (mechanical-stop protection)

The Arduino derives board = index // 9 and channel = index % 9 from the global
servo index (see protocol.py); this module produces the 18 pulses in that index
order, ready for protocol.encode_servo_command().

⚠ The default calibration below is a *sane starting point*, not ground truth.
The neutral_us / trim_deg / direction values must be tuned against the real
robot with a calibration jig — that is exactly what these fields are for.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, NamedTuple

from pi.config import LEG_ORDER, Joint, LegId
from pi.comms.protocol import (
    NUM_SERVOS,
    board_of,
    channel_of,
    servo_index,
)
from pi.kinematics.leg_ik import JointAngles

# MG996R: ~500–2500 µs spans ~180°, so ≈11.11 µs per degree.
US_PER_DEG = 2000.0 / 180.0
SERVO_MIN_US = 500
SERVO_MAX_US = 2500


@dataclass(frozen=True)
class ServoCalibration:
    """Per-servo mapping from joint angle (deg) to pulse width (µs)."""

    neutral_us: float           # pulse at joint angle 0° (before trim)
    direction: int              # +1 or -1 (mirrored mounting)
    us_per_deg: float = US_PER_DEG
    trim_deg: float = 0.0       # mechanical zero correction, added to the angle
    min_us: int = SERVO_MIN_US  # clamp floor (protect against stops)
    max_us: int = SERVO_MAX_US  # clamp ceiling

    def to_pulse(self, angle_deg: float) -> int:
        """Convert a joint angle to a clamped, integer microsecond pulse."""
        us = self.neutral_us + self.direction * (angle_deg + self.trim_deg) * self.us_per_deg
        return int(round(max(self.min_us, min(self.max_us, us))))


class ServoTarget(NamedTuple):
    """A fully resolved command for one servo — handy for debugging/inspection."""

    index: int      # global servo index 0..17
    board: int      # PCA9685 board (0 or 1)
    channel: int    # channel on that board (0..8)
    leg: LegId
    joint: Joint
    angle_deg: float
    pulse_us: int


CalibrationTable = Mapping[int, ServoCalibration]


def default_calibration() -> dict[int, ServoCalibration]:
    """Build a within-range default calibration for all 18 servos.

    Chosen so that every joint's *full configured travel* — and the resting
    stance pose — lands inside [500, 2500] µs without clamping:

      * coxa  (±45°)  : neutral 1500 µs (centred); symmetric, so either sign fits.
      * femur (±90°)  : neutral 1500 µs (centred); ±90° → exactly 500..2500.
      * tibia (0..150°): asymmetric range, so the neutral pulse is pushed to the
        end the travel grows away from — 700 µs on the left (+dir), 2300 µs on
        the right (−dir) — keeping 0..150° inside the window on both sides.

    Left legs use direction +1, right legs −1 (mirror image). All trims 0.
    """
    cal: dict[int, ServoCalibration] = {}
    for leg in LEG_ORDER:
        is_left = int(leg) < 3  # legs 0,1,2 are the left side
        direction = 1 if is_left else -1
        for joint in (Joint.COXA, Joint.FEMUR, Joint.TIBIA):
            if joint is Joint.TIBIA:
                neutral = 700.0 if is_left else 2300.0
            else:
                neutral = 1500.0
            cal[servo_index(leg, joint)] = ServoCalibration(
                neutral_us=neutral, direction=direction
            )
    return cal


DEFAULT_CALIBRATION: dict[int, ServoCalibration] = default_calibration()


def angles_to_pulses(
    angles_by_leg: Mapping[LegId, JointAngles],
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
) -> list[int]:
    """Flatten per-leg joint angles into 18 µs pulses in global index order,
    ready to hand to protocol.encode_servo_command()."""
    pulses = [0] * NUM_SERVOS
    for leg in LEG_ORDER:
        angles = angles_by_leg[leg]
        triple = (angles.coxa, angles.femur, angles.tibia)
        for joint in (Joint.COXA, Joint.FEMUR, Joint.TIBIA):
            idx = servo_index(leg, joint)
            pulses[idx] = calibration[idx].to_pulse(triple[int(joint)])
    return pulses


def resolve(
    angles_by_leg: Mapping[LegId, JointAngles],
    calibration: CalibrationTable = DEFAULT_CALIBRATION,
) -> list[ServoTarget]:
    """Like angles_to_pulses but returns full (index, board, channel, …) records
    for every servo — used by the teleop/inspection tooling."""
    targets: list[ServoTarget] = []
    for leg in LEG_ORDER:
        angles = angles_by_leg[leg]
        triple = (angles.coxa, angles.femur, angles.tibia)
        for joint in (Joint.COXA, Joint.FEMUR, Joint.TIBIA):
            idx = servo_index(leg, joint)
            angle = triple[int(joint)]
            targets.append(ServoTarget(
                index=idx,
                board=board_of(idx),
                channel=channel_of(idx),
                leg=leg,
                joint=joint,
                angle_deg=angle,
                pulse_us=calibration[idx].to_pulse(angle),
            ))
    return targets


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.comms.servo_map`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    from pi.comms.protocol import decode_servo_command, encode_servo_command, FrameParser
    from pi.kinematics.body import default_stance_body, solve_body, BodyPose

    print("servo_map self-test")
    print("=" * 60)
    cal = DEFAULT_CALIBRATION

    # Angle 0 maps to the configured neutral pulse (rounded).
    coxa_l = cal[servo_index(LegId.L1, Joint.COXA)]
    coxa_r = cal[servo_index(LegId.R1, Joint.COXA)]
    assert coxa_l.to_pulse(0.0) == 1500 and coxa_r.to_pulse(0.0) == 1500
    print("angle 0 -> neutral pulse .................. OK")

    # Direction sign: +angle raises the left pulse, lowers the mirrored right.
    assert coxa_l.to_pulse(10.0) > 1500 > coxa_r.to_pulse(10.0)
    print("direction sign mirrors left vs right ..... OK")

    # Clamping protects the servo at extreme angles.
    assert coxa_l.to_pulse(1000.0) == SERVO_MAX_US
    assert coxa_l.to_pulse(-1000.0) == SERVO_MIN_US
    print("out-of-range angles clamp to limits ...... OK")

    # Every joint's FULL configured travel stays inside the pulse window for all
    # 18 servos (i.e. nothing clamps within the legal joint range).
    from pi.config import ROBOT
    for leg in LEG_ORDER:
        for joint in (Joint.COXA, Joint.FEMUR, Joint.TIBIA):
            idx = servo_index(leg, joint)
            lim = ROBOT.limits(joint)
            c = cal[idx]
            for a in (lim.min_deg, 0.0 if lim.contains(0.0) else lim.min_deg, lim.max_deg):
                raw = c.neutral_us + c.direction * (a + c.trim_deg) * c.us_per_deg
                assert SERVO_MIN_US - 1e-9 <= raw <= SERVO_MAX_US + 1e-9, \
                    f"{leg.name}/{joint.name} angle {a} -> {raw}us out of range"
    print("full joint travel fits pulse window ...... OK")

    # The resting stance solves and maps to in-range pulses; coxae sit at 1500.
    angles = solve_body(default_stance_body(), BodyPose())
    pulses = angles_to_pulses(angles)
    assert len(pulses) == NUM_SERVOS
    assert all(SERVO_MIN_US <= p <= SERVO_MAX_US for p in pulses)
    assert pulses[servo_index(LegId.L1, Joint.COXA)] == 1500
    assert pulses[servo_index(LegId.R3, Joint.COXA)] == 1500
    print("neutral stance -> in-range 18 pulses ..... OK")
    fl = resolve(angles)
    print(f"  L1: coxa={fl[0].pulse_us}us femur={fl[1].pulse_us}us "
          f"tibia={fl[2].pulse_us}us")
    print(f"  R1: coxa={fl[9].pulse_us}us femur={fl[10].pulse_us}us "
          f"tibia={fl[11].pulse_us}us")

    # End-to-end through the wire protocol.
    frame = encode_servo_command(pulses)
    decoded = decode_servo_command(FrameParser().feed(frame)[0][1])
    assert decoded == pulses
    print("pulses survive protocol encode/decode .... OK")

    # Board/channel addressing in the resolved targets.
    assert fl[0].board == 0 and fl[0].channel == 0
    assert fl[9].board == 1 and fl[9].channel == 0
    assert fl[17].board == 1 and fl[17].channel == 8
    print("board/channel addressing ................. OK")

    print("-" * 60)
    print("All servo_map self-tests passed.")


if __name__ == "__main__":
    _selftest()
