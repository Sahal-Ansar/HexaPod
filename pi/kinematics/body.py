"""Body kinematics — turn a body pose + body-frame foot targets into per-leg,
leg-frame foot targets ready for the single-leg IK.

This is the bridge between two ways of thinking about the robot:

  * The GAIT engine and the leveling logic think in the BODY frame: "this foot
    should be on the ground here relative to the body centre."
  * The single-leg IK (leg_ik.py) needs the foot expressed in that leg's own
    LEG frame (origin at the coxa, +X straight out).

It also lets us *pose the body*: translate or tilt the body while the feet stay
planted on the ground. That single capability powers three behaviours later:
  - ride-height / body-shift commands (translate the body),
  - keeping the body level on uneven terrain via the IMU (counter-roll/pitch),
  - shifting the centre of mass for stability.

────────────────────────────────────────────────────────────────────────────
THE TRANSFORM
────────────────────────────────────────────────────────────────────────────
A foot target is given in the body frame, ``f_body`` (where it sits relative to
the body centre at the *neutral* body pose). The body is then displaced from
neutral by a pose: translation ``T = (x, y, z)`` and rotation
``R = Rz(yaw)·Ry(pitch)·Rx(roll)``.

Because the foot is planted on the ground, moving the BODY by (T, R) moves the
foot the *opposite* way as seen from the (now posed) body frame. So we apply the
INVERSE body transform to the foot, then drop it into the leg frame:

    1. into the posed body frame:   v = R⁻¹ · (f_body − T)
    2. into the leg frame:          f_leg = Rz(−mount_yaw) · (v − mount_pos)

where R⁻¹ = Rx(−roll)·Ry(−pitch)·Rz(−yaw). Step 1 is why raising the body (+z
translation) makes every leg reach further DOWN, and why commanding a body roll
counter-tilts the stance — exactly what leveling needs.
"""

from __future__ import annotations

from dataclasses import dataclass

from pi.config import ROBOT, LEG_ORDER, LegId, LegMount, RobotConfig
from pi.kinematics.leg_ik import JointAngles, solve
from pi.math_utils import Vec3, rotate_x, rotate_y, rotate_z


@dataclass(frozen=True)
class BodyPose:
    """Displacement of the body from its neutral pose.

    Translation in millimetres (body frame: +x forward, +y left, +z up) and
    rotation in degrees (roll about +x, pitch about +y, yaw about +z). The
    default is the identity pose (body sitting at neutral, feet at their default
    stance).
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def translation(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    def is_identity(self) -> bool:
        return (self.x == self.y == self.z == 0.0
                and self.roll == self.pitch == self.yaw == 0.0)


def _mount_pos(mount: LegMount) -> Vec3:
    return Vec3(mount.mount_x, mount.mount_y, mount.mount_z)


def neutral_foot_body(mount: LegMount, config: RobotConfig = ROBOT) -> Vec3:
    """The default stance foot position for one leg, in the BODY frame.

    Takes the leg-frame neutral foot (straight out, below the coxa) back up into
    the body frame: rotate by the leg's mount yaw, then offset by the mount
    position. This is the inverse of the leg-frame step of the transform, so
    ``foot_body_to_leg(neutral_foot_body(m), identity) == neutral_foot``.
    """
    leg_neutral = Vec3(*config.neutral_foot())
    return _mount_pos(mount) + rotate_z(leg_neutral, mount.mount_yaw_deg)


def default_stance_body(config: RobotConfig = ROBOT) -> dict[LegId, Vec3]:
    """Default footprint for all six legs, in the BODY frame. This is the set of
    ground contact points the gait engine swings each foot around."""
    return {leg: neutral_foot_body(config.mount(leg), config) for leg in LEG_ORDER}


def foot_body_to_leg(
    mount: LegMount,
    foot_body: Vec3,
    pose: BodyPose = BodyPose(),
) -> Vec3:
    """Convert one foot target from the BODY frame to that leg's LEG frame,
    applying the (inverse) body pose. See the module header for the derivation."""
    # Step 1: express the planted foot in the *posed* body frame (inverse pose).
    v = foot_body - pose.translation
    v = rotate_z(v, -pose.yaw)
    v = rotate_y(v, -pose.pitch)
    v = rotate_x(v, -pose.roll)
    # Step 2: drop into the leg frame (undo the mount offset, then mount yaw).
    v = v - _mount_pos(mount)
    return rotate_z(v, -mount.mount_yaw_deg)


def feet_body_to_leg(
    foot_targets_body: dict[LegId, Vec3],
    pose: BodyPose = BodyPose(),
    config: RobotConfig = ROBOT,
) -> dict[LegId, Vec3]:
    """Convert every leg's body-frame foot target to its leg frame."""
    return {
        leg: foot_body_to_leg(config.mount(leg), foot_targets_body[leg], pose)
        for leg in foot_targets_body
    }


def solve_body(
    foot_targets_body: dict[LegId, Vec3],
    pose: BodyPose = BodyPose(),
    config: RobotConfig = ROBOT,
) -> dict[LegId, JointAngles]:
    """Full body IK: body-frame foot targets + body pose → joint angles per leg.

    Convenience that chains ``foot_body_to_leg`` into the single-leg IK for all
    six legs. This is what the control loop calls each tick.
    """
    return {
        leg: solve(foot_body_to_leg(config.mount(leg), foot_targets_body[leg], pose))
        for leg in foot_targets_body
    }


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.kinematics.body`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    def approx(a: float, b: float, tol: float = 1e-9) -> bool:
        return math.isclose(a, b, abs_tol=tol)

    print("body kinematics self-test")
    print("=" * 60)

    neutral_leg = Vec3(*ROBOT.neutral_foot())
    stance = default_stance_body()

    # 1) Identity pose: every leg's default body-frame foot maps straight back to
    #    the leg-frame neutral foot. Validates the mount transform round-trip.
    for leg in LEG_ORDER:
        leg_pt = foot_body_to_leg(ROBOT.mount(leg), stance[leg], BodyPose())
        assert leg_pt.distance_to(neutral_leg) < 1e-9, f"{leg.name} identity"
    print("identity pose -> neutral leg frame (all 6 legs) ... OK")

    # 2) Raising the body (+z translation) makes every leg reach further DOWN by
    #    exactly that amount (leg-frame foot z decreases by dz).
    dz = 20.0
    raised = BodyPose(z=dz)
    for leg in LEG_ORDER:
        leg_pt = foot_body_to_leg(ROBOT.mount(leg), stance[leg], raised)
        assert approx(leg_pt.z, neutral_leg.z - dz), f"{leg.name} +z"
    print(f"body +{dz:.0f}mm z -> feet reach {dz:.0f}mm lower (all 6) ... OK")

    # 3) Pure translation law: with no rotation, the leg-frame shift equals the
    #    body translation rotated into the leg frame by −mount_yaw.
    for T in (Vec3(30, 0, 0), Vec3(0, -25, 0), Vec3(15, 10, -5)):
        pose = BodyPose(x=T.x, y=T.y, z=T.z)
        for leg in LEG_ORDER:
            m = ROBOT.mount(leg)
            moved = foot_body_to_leg(m, stance[leg], pose)
            base = foot_body_to_leg(m, stance[leg], BodyPose())
            expected_delta = rotate_z(Vec3(-T.x, -T.y, -T.z), -m.mount_yaw_deg)
            assert (moved - base).distance_to(expected_delta) < 1e-9, f"{leg.name} {T}"
    print("translation delta law (3 vectors × 6 legs) ........ OK")

    # 4) Yaw is a rotation about +Z and the coxae sit on z=0, so a pure yaw must
    #    leave every leg-frame foot z unchanged (only x/y rotate).
    for yaw in (-20.0, 10.0, 35.0):
        pose = BodyPose(yaw=yaw)
        for leg in LEG_ORDER:
            leg_pt = foot_body_to_leg(ROBOT.mount(leg), stance[leg], pose)
            assert approx(leg_pt.z, neutral_leg.z), f"{leg.name} yaw {yaw}"
    print("pure yaw preserves foot height (all 6) ............ OK")

    # 5) Full pipeline: default stance at identity pose must solve to the neutral
    #    joint pose for every leg (coxa≈0, sensible femur/tibia).
    angles = solve_body(stance, BodyPose())
    for leg in LEG_ORDER:
        a = angles[leg]
        assert approx(a.coxa, 0.0, 1e-6), f"{leg.name} coxa"
        assert 0.0 < a.femur < 45.0 and 90.0 < a.tibia < 140.0, f"{leg.name} pose"
    print("solve_body(default stance) -> neutral pose (all 6)  OK")
    sample = angles[LegId.R1]
    print(f"  e.g. R1: coxa={sample.coxa:.3f} femur={sample.femur:.3f} "
          f"tibia={sample.tibia:.3f}")

    print("-" * 60)
    print("All body kinematics self-tests passed.")


if __name__ == "__main__":
    _selftest()
