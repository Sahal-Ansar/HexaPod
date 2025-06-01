r"""3-DOF single-leg inverse kinematics.

Given a desired foot position ``(x, y, z)`` expressed in the **leg frame**, this
solves for the three joint angles (coxa, femur, tibia) that put the foot there.
Because every leg works in its own leg frame (see config.py), this one function
serves all six legs — the body-kinematics stage is what rotates a body-level
foot target into each leg's frame.

────────────────────────────────────────────────────────────────────────────
THE GEOMETRY  (this is the part to understand cold for an interview)
────────────────────────────────────────────────────────────────────────────
The leg is a 3-link chain:
    coxa  : rotates about the leg-frame +Z (vertical) — pure yaw.
    femur : rotates about a horizontal axis at the top of the leg — pitch.
    tibia : rotates about a parallel horizontal axis at the knee — pitch.

Step 1 — COXA solves the yaw on its own.
    Looking straight down, the foot sits at (x, y). The coxa simply points the
    leg at it:                       θ_coxa = atan2(y, x)
    (θ=0 → foot straight out along +X; +θ swings the foot toward +Y.)

Step 2 — Collapse femur+tibia to a 2D problem in the leg's VERTICAL plane.
    After the coxa yaw, the femur and tibia move in the vertical plane that
    contains the leg. Measure two distances in that plane, taking the FEMUR
    joint as the origin (the femur joint is one coxa-length out from the coxa
    axis, on the hip plane z=0):

        horiz = hypot(x, y)          # how far out the foot is, on the ground plane
        L = horiz - coxa_len         # horizontal femur-joint → foot
        V = z                        # vertical   femur-joint → foot  (z<0 = below)
        D = hypot(L, V)              # straight-line femur-joint → foot

    Now it's a planar 2-link arm (lengths femur=a, tibia=b) reaching a point a
    distance D away, on a line that is φ above horizontal:

        φ = atan2(V, L)

                              knee
                              /\
                       a=femur  \  b=tibia
                            /     \
        femur joint ●------/-------●  foot
                    | \   φ       /
                    |  \_________/   D = hypot(L, V)
                  (origin)

Step 3 — Law of cosines on that triangle (sides a, b, D).
    Interior knee angle (vertex at the knee):
        cos(knee) = (a² + b² − D²) / (2ab)
    Our tibia convention has θ_tibia = 0 when the shin is *in line* with the
    femur (knee straight, interior angle 180°), so:
        θ_tibia = 180° − knee
    (θ grows as the knee folds → foot tucks under: matches config.py.)

    The femur sits above the foot-line by angle α (vertex at the femur joint):
        cos(α) = (a² + D² − b²) / (2aD)
    We pick the **knee-UP** solution (standard insect posture: knee above the
    foot), so the femur is lifted α above the line to the foot:
        θ_femur = φ + α

Reachability: a real triangle needs |a−b| ≤ D ≤ a+b. Outside that, we clamp D
into range (and clamp the cosines for floating-point safety) so the solver
degrades gracefully to "reach as far as you can, straight at the target"
instead of throwing — important for a control loop that must never crash.
"""

from __future__ import annotations

import math
from typing import Mapping, NamedTuple

from pi.config import ROBOT, Joint, JointLimits, LegGeometry
from pi.math_utils import Vec3, clamp


class JointAngles(NamedTuple):
    """The three joint angles for one leg, in degrees, in the IK convention."""

    coxa: float
    femur: float
    tibia: float


def is_reachable(
    target: tuple[float, float, float],
    geometry: LegGeometry = ROBOT.geometry,
) -> bool:
    """True if the foot point is inside the leg's annular workspace, i.e. the
    femur+tibia 2-link arm can actually touch it (before joint limits)."""
    x, y, z = target
    L = math.hypot(x, y) - geometry.coxa_mm
    D = math.hypot(L, z)
    reach_max = geometry.femur_mm + geometry.tibia_mm
    reach_min = abs(geometry.femur_mm - geometry.tibia_mm)
    return reach_min <= D <= reach_max


def solve(
    target: tuple[float, float, float],
    geometry: LegGeometry = ROBOT.geometry,
    limits: Mapping[Joint, JointLimits] | None = ROBOT.joint_limits,
) -> JointAngles:
    """Inverse kinematics: foot ``(x, y, z)`` in the leg frame → joint angles.

    Args:
        target:   desired foot position in the leg frame, millimetres.
        geometry: leg segment lengths (defaults to the configured robot).
        limits:   per-joint angle limits to clamp the result to, for servo
                  safety. Pass ``None`` to get the raw solution (used by the
                  FK round-trip test so clamping doesn't corrupt the check).

    Returns:
        JointAngles(coxa, femur, tibia) in degrees. If the target is out of
        reach, the closest achievable straight-leg pose is returned rather than
        raising — call ``is_reachable`` first if you need to know.
    """
    x, y, z = target
    a = geometry.femur_mm
    b = geometry.tibia_mm

    # ── Step 1: coxa yaw points the leg at the foot in the ground plane. ──
    coxa_deg = math.degrees(math.atan2(y, x))

    # ── Step 2: reduce femur+tibia to a planar 2-link problem. ──
    horiz = math.hypot(x, y)
    L = horiz - geometry.coxa_mm   # horizontal: femur joint → foot
    V = z                          # vertical:   femur joint → foot
    D = math.hypot(L, V)           # straight-line femur joint → foot

    # Clamp the reach so an out-of-workspace target degrades gracefully to the
    # nearest straight/folded leg instead of producing a math-domain error.
    reach_max = a + b
    reach_min = abs(a - b)
    d_eff = clamp(D, reach_min, reach_max)

    # ── Step 3: law of cosines for the knee and femur angles. ──
    # Interior knee angle; clamp the cosine for floating-point safety at the
    # exact workspace boundary.
    cos_knee = clamp((a * a + b * b - d_eff * d_eff) / (2.0 * a * b), -1.0, 1.0)
    knee_interior_deg = math.degrees(math.acos(cos_knee))
    tibia_deg = 180.0 - knee_interior_deg   # 0 when straight, +ve folds the knee

    # Femur: lift the thigh by α above the line to the foot (knee-up solution).
    phi_deg = math.degrees(math.atan2(V, L))
    cos_alpha = clamp((a * a + d_eff * d_eff - b * b) / (2.0 * a * d_eff), -1.0, 1.0)
    alpha_deg = math.degrees(math.acos(cos_alpha))
    femur_deg = phi_deg + alpha_deg

    angles = JointAngles(coxa_deg, femur_deg, tibia_deg)

    # ── Optional: clamp to mechanical limits to protect the servos. ──
    if limits is not None:
        angles = JointAngles(
            coxa=limits[Joint.COXA].clamp(angles.coxa),
            femur=limits[Joint.FEMUR].clamp(angles.femur),
            tibia=limits[Joint.TIBIA].clamp(angles.tibia),
        )
    return angles


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.kinematics.leg_ik`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    def approx(a: float, b: float, tol: float = 1e-6) -> bool:
        return math.isclose(a, b, abs_tol=tol)

    print("leg_ik self-test")
    print("=" * 56)
    g = ROBOT.geometry
    print(f"geometry: coxa={g.coxa_mm} femur={g.femur_mm} tibia={g.tibia_mm}")

    # Neutral stance foot, solved without limit clamping for an exact check.
    neutral = ROBOT.neutral_foot()
    a = solve(neutral, limits=None)
    print(f"\nneutral foot {neutral} ->")
    print(f"  coxa={a.coxa:7.3f}  femur={a.femur:7.3f}  tibia={a.tibia:7.3f}  (deg)")
    # Coxa must be 0 (foot straight ahead) and the leg in a sensible knee-up pose.
    assert approx(a.coxa, 0.0), "neutral foot should need zero coxa yaw"
    assert 0.0 < a.femur < 45.0, "neutral femur should be lifted a little"
    assert 90.0 < a.tibia < 140.0, "neutral tibia should be folded into a stance"

    # Coxa yaw: a foot offset in +Y must yaw the coxa toward +Y by atan2(y,x).
    swung = solve((100.0, 50.0, -90.0), limits=None)
    expected_coxa = math.degrees(math.atan2(50.0, 100.0))
    print(f"\nswung foot (100, 50, -90) -> coxa={swung.coxa:.3f} "
          f"(expected {expected_coxa:.3f})")
    assert approx(swung.coxa, expected_coxa), "coxa must equal atan2(y, x)"

    # Reachability + graceful degradation on an impossible target.
    far = (400.0, 0.0, -50.0)
    print(f"\nfar foot {far}: reachable={is_reachable(far)}")
    assert not is_reachable(far), "400mm out is beyond max reach"
    a_far = solve(far, limits=None)
    print(f"  -> coxa={a_far.coxa:.3f} femur={a_far.femur:.3f} "
          f"tibia={a_far.tibia:.3f}")
    assert approx(a_far.tibia, 0.0, tol=1e-6), \
        "unreachable-far target should straighten the leg (tibia≈0)"

    # A point inside the workspace really is reported reachable.
    assert is_reachable(neutral), "neutral stance must be reachable"

    print("-" * 56)
    print("All leg_ik self-tests passed.")


if __name__ == "__main__":
    _selftest()
