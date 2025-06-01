"""Forward kinematics for a single leg — the inverse check for the IK.

FK is the easy direction: given the three joint angles, walk down the chain
(coxa → femur → tibia) and report where the foot ends up in the leg frame. Its
main job here is verification: ``foot_position(solve(p)) ≈ p`` is the round-trip
test that proves the IK is correct (Stage 6 sweeps this across the workspace).

The math is the *same convention* as leg_ik.py, run forwards:

    planar "out" axis = horizontal distance from the coxa axis, before yaw
    planar "up"  axis = z

    femur joint :  out = coxa_len,                       up = 0
    knee        :  out += femur·cos(θ_femur),            up += femur·sin(θ_femur)
    foot        :  out += tibia·cos(θ_femur − θ_tibia),  up += tibia·sin(θ_femur − θ_tibia)

θ_tibia is measured from the femur's own line (0 = straight), so the tibia's
absolute angle from horizontal is (θ_femur − θ_tibia). Finally the coxa yaw
spreads the planar "out" distance back into x/y:

    x = out·cos(θ_coxa),   y = out·sin(θ_coxa),   z = up
"""

from __future__ import annotations

import math

from pi.config import ROBOT, LegGeometry
from pi.kinematics.leg_ik import JointAngles, solve
from pi.math_utils import Vec3


def joint_positions(
    angles: JointAngles,
    geometry: LegGeometry = ROBOT.geometry,
) -> tuple[Vec3, Vec3, Vec3, Vec3]:
    """Return the leg-frame positions of (hip, femur_joint, knee, foot).

    Useful for verification (segment lengths must come out right) and for any
    future visualiser. ``hip`` is the coxa axis at the leg-frame origin.
    """
    coxa_r = math.radians(angles.coxa)
    femur_r = math.radians(angles.femur)
    # Tibia's absolute angle from horizontal = femur angle minus the knee bend.
    tibia_abs_r = math.radians(angles.femur - angles.tibia)

    cz, sz = math.cos(coxa_r), math.sin(coxa_r)

    def spread(out: float, up: float) -> Vec3:
        """Lift a planar (out, up) point into the 3D leg frame via the coxa yaw."""
        return Vec3(out * cz, out * sz, up)

    hip = Vec3(0.0, 0.0, 0.0)

    out = geometry.coxa_mm
    up = 0.0
    femur_joint = spread(out, up)

    out += geometry.femur_mm * math.cos(femur_r)
    up += geometry.femur_mm * math.sin(femur_r)
    knee = spread(out, up)

    out += geometry.tibia_mm * math.cos(tibia_abs_r)
    up += geometry.tibia_mm * math.sin(tibia_abs_r)
    foot = spread(out, up)

    return hip, femur_joint, knee, foot


def foot_position(
    angles: JointAngles,
    geometry: LegGeometry = ROBOT.geometry,
) -> Vec3:
    """Forward kinematics: joint angles → foot position (x, y, z) in leg frame."""
    return joint_positions(angles, geometry)[3]


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.kinematics.leg_fk`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    def approx(a: float, b: float, tol: float = 1e-6) -> bool:
        return math.isclose(a, b, abs_tol=tol)

    g = ROBOT.geometry
    print("leg_fk self-test")
    print("=" * 56)

    # 1) Internal consistency: the chain segments must have the right lengths
    #    for ANY joint angles, independent of the IK.
    test_angles = JointAngles(coxa=20.0, femur=30.0, tibia=80.0)
    hip, femur_joint, knee, foot = joint_positions(test_angles)
    assert approx(hip.distance_to(femur_joint), g.coxa_mm), "coxa length wrong"
    assert approx(femur_joint.distance_to(knee), g.femur_mm), "femur length wrong"
    assert approx(knee.distance_to(foot), g.tibia_mm), "tibia length wrong"
    print("segment lengths preserved ......... OK")
    print(f"  hip={hip}  femur_joint={femur_joint}")
    print(f"  knee={knee}  foot={foot}")

    # 2) FK of the IK'd neutral pose must return the neutral foot.
    neutral = ROBOT.neutral_foot()
    foot_back = foot_position(solve(neutral, limits=None))
    print(f"\nneutral foot target {neutral}")
    print(f"FK(IK(neutral))     {tuple(round(c, 6) for c in foot_back)}")
    assert foot_back.distance_to(Vec3(*neutral)) < 1e-6, "FK∘IK must recover foot"
    print("FK(IK(neutral)) round-trip ........ OK")

    # 3) A couple more round trips at points inside the workspace.
    for p in [(120.0, 30.0, -80.0), (90.0, -40.0, -110.0), (140.0, 0.0, -60.0)]:
        back = foot_position(solve(p, limits=None))
        err = back.distance_to(Vec3(*p))
        print(f"  target {p} -> err {err:.2e} mm")
        assert err < 1e-6, f"round-trip failed for {p}"

    print("-" * 56)
    print("All leg_fk self-tests passed.")


if __name__ == "__main__":
    _selftest()
