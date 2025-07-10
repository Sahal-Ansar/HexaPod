r"""IMU body-leveling — keep the body level on uneven ground.

The Arduino streams the body's measured tilt (roll, pitch) from the MPU6050. On
a slope the body tilts; we want it level. Because the feet are planted, we can
counter-rotate the BODY relative to the feet using a body-pose roll/pitch — and
body kinematics (Stage 7) already turns that pose into the right per-leg foot
targets. So leveling is a feedback controller whose *actuator* is BodyPose.

Controller: an integral (accumulating) correction that drives the measured tilt
to zero. Each tick:

    correction += sign · kp · measured_tilt        (clamped to ±max_tilt)

If the body is rolled +θ, we nudge the body-pose roll to take it back, the next
measurement is smaller, and it converges. It's an integrator (not pure
proportional) because we want *zero* steady-state tilt on a constant slope, and
an integrator parks the error at zero.

  * kp < 1 keeps it stable and smooth (small kp = gentle, slow; large = snappy,
    risk of oscillation).
  * max_tilt clamps how far the legs are asked to lean — on a slope steeper than
    that, it levels as much as it can.
  * sign depends on how the IMU is mounted vs. the body-pose roll convention. If
    the robot leans *harder* into the slope instead of leveling, flip it. (The
    absolute sign can only be pinned down on the real hardware.)
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from pi.comms.protocol import Telemetry
from pi.kinematics.body import BodyPose
from pi.math_utils import clamp


@dataclass(frozen=True)
class LevelingConfig:
    kp: float = 0.2            # integral gain per tick
    max_tilt_deg: float = 12.0  # clamp on the commanded body lean
    sign: int = 1             # +1 or -1 depending on IMU mounting
    enabled: bool = True


class BodyLeveler:
    """Holds the accumulated leveling correction and applies it to a base pose."""

    def __init__(self, config: LevelingConfig = LevelingConfig()) -> None:
        self.config = config
        self._roll_cmd = 0.0
        self._pitch_cmd = 0.0

    @property
    def correction(self) -> tuple[float, float]:
        """Current (roll, pitch) correction the controller is applying, deg."""
        return (self._roll_cmd, self._pitch_cmd)

    def reset(self) -> None:
        self._roll_cmd = 0.0
        self._pitch_cmd = 0.0

    def set_enabled(self, enabled: bool) -> None:
        self.config = replace(self.config, enabled=enabled)

    def update(self, telemetry: Telemetry | None, base_pose: BodyPose = BodyPose()) -> BodyPose:
        """Integrate the latest tilt into the correction and return the pose to
        command (base pose + leveling roll/pitch).

        With telemetry missing, the last correction is HELD (no jump). With
        leveling disabled, the base pose is returned untouched.
        """
        if not self.config.enabled:
            return base_pose

        if telemetry is not None:
            c = self.config
            self._roll_cmd = clamp(
                self._roll_cmd + c.sign * c.kp * telemetry.roll_deg,
                -c.max_tilt_deg, c.max_tilt_deg,
            )
            self._pitch_cmd = clamp(
                self._pitch_cmd + c.sign * c.kp * telemetry.pitch_deg,
                -c.max_tilt_deg, c.max_tilt_deg,
            )

        return replace(
            base_pose,
            roll=base_pose.roll + self._roll_cmd,
            pitch=base_pose.pitch + self._pitch_cmd,
        )


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.leveling`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    print("body leveling self-test")
    print("=" * 60)

    def telem(roll: float, pitch: float) -> Telemetry:
        return Telemetry(distance_mm=1000, roll_deg=roll, pitch_deg=pitch,
                         contacts=(False,) * 6)

    # Plant model for simulation: the measured tilt is the ground slope minus
    # whatever we've leaned (sign convention effect = +1, matching sign=+1):
    #   measured = ground − correction
    class Slope:
        def __init__(self, ground_roll: float, ground_pitch: float) -> None:
            self.gr = ground_roll
            self.gp = ground_pitch

        def measure(self, lev: BodyLeveler) -> Telemetry:
            rc, pc = lev.correction
            return telem(self.gr - rc, self.gp - pc)

    # 1) Convergence: on an 8°/-5° slope the body levels (measured -> 0) and the
    #    correction parks at the slope angle.
    lev = BodyLeveler(LevelingConfig(kp=0.2))
    slope = Slope(8.0, -5.0)
    last = None
    for _ in range(80):
        m = slope.measure(lev)
        lev.update(m)
        last = m
    assert abs(last.roll_deg) < 0.1 and abs(last.pitch_deg) < 0.1, "should level out"
    rc, pc = lev.correction
    assert math.isclose(rc, 8.0, abs_tol=0.1) and math.isclose(pc, -5.0, abs_tol=0.1)
    print("converges: 8/-5 deg slope leveled to ~0 ......... OK")

    # 2) Monotone, no overshoot/oscillation for kp < 1.
    lev = BodyLeveler(LevelingConfig(kp=0.25))
    slope = Slope(10.0, 0.0)
    prev = 10.0
    for _ in range(60):
        m = slope.measure(lev)
        assert m.roll_deg <= prev + 1e-9, "measured tilt must not grow"
        prev = m.roll_deg
        lev.update(m)
    print("monotone convergence, no oscillation (kp<1) ..... OK")

    # 3) Clamp: a 30° slope exceeds max_tilt, so it levels only partially.
    lev = BodyLeveler(LevelingConfig(kp=0.3, max_tilt_deg=12.0))
    slope = Slope(30.0, 0.0)
    for _ in range(100):
        lev.update(slope.measure(lev))
    rc, _ = lev.correction
    assert math.isclose(rc, 12.0, abs_tol=1e-6), "correction clamps at max_tilt"
    residual = slope.measure(lev).roll_deg
    assert math.isclose(residual, 18.0, abs_tol=0.1), "30-12 deg residual remains"
    print("steep slope clamps lean at max_tilt (12 deg) .... OK")

    # 4) Disabled -> base pose untouched. Missing telemetry -> hold correction.
    base = BodyPose(z=-10.0)
    lev = BodyLeveler(LevelingConfig(enabled=False))
    assert lev.update(telem(5, 5), base) == base
    lev = BodyLeveler(LevelingConfig(kp=0.2))
    lev.update(telem(6, 0))
    rc_before, _ = lev.correction
    held = lev.update(None, base)             # telemetry dropped
    rc_after, _ = lev.correction
    assert rc_before == rc_after, "correction held when telemetry missing"
    assert math.isclose(held.roll, base.roll + rc_after)
    print("disabled passes through; missing telem holds .... OK")

    # 5) Base pose composes: leveling adds onto an existing pose (e.g. a body
    #    height offset from standup), and the result still solves through IK.
    from pi.comms.servo_map import SERVO_MAX_US, SERVO_MIN_US, angles_to_pulses
    from pi.kinematics.body import default_stance_body, solve_body
    lev = BodyLeveler(LevelingConfig(kp=0.2))
    for _ in range(10):
        pose = lev.update(telem(6.0, -4.0), BodyPose(z=-5.0))
        pulses = angles_to_pulses(solve_body(default_stance_body(), pose))
        assert all(SERVO_MIN_US <= p <= SERVO_MAX_US for p in pulses)
    assert lev.correction[0] > 0  # leaned to correct
    print("leveling pose composes with base + solves IK .... OK")

    print("-" * 60)
    print("All body leveling self-tests passed.")


if __name__ == "__main__":
    _selftest()
