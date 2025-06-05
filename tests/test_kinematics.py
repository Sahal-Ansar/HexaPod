"""IK ↔ FK round-trip tests, runnable on a laptop with no hardware.

The headline test sweeps a grid of foot targets across the leg's workspace and
asserts ``FK(IK(p)) ≈ p`` for every reachable point. That single property is
the strongest possible correctness check for the IK: if any branch choice, sign,
or law-of-cosines term were wrong, the foot would not land back on the target.

Run with:   pytest tests/test_kinematics.py   (or just `pytest`)
"""

from __future__ import annotations

import math

import pytest

from pi.config import ROBOT, Joint
from pi.kinematics.leg_ik import JointAngles, is_reachable, solve
from pi.kinematics.leg_fk import foot_position, joint_positions
from pi.math_utils import Vec3

# Reconstruction tolerance. IK/FK are closed-form, so the only error is
# floating-point; 1e-6 mm (a nanometre) is comfortably above that noise floor.
TOL_MM = 1e-6


# ────────────────────────────────────────────────────────────────────────────
# Build the workspace grid once, at import time, and keep only the foot targets
# the leg can actually reach (inside the femur+tibia annulus). x stays positive
# (foot out in front of the coxa), z negative (foot below the hip plane).
# ────────────────────────────────────────────────────────────────────────────
def _frange(start: float, stop: float, step: float) -> list[float]:
    n = int(round((stop - start) / step))
    return [start + i * step for i in range(n + 1)]


def _workspace_grid() -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for x in _frange(60.0, 180.0, 20.0):
        for y in _frange(-80.0, 80.0, 20.0):
            for z in _frange(-140.0, -30.0, 20.0):
                p = (x, y, z)
                if is_reachable(p):
                    pts.append(p)
    return pts


_GRID = _workspace_grid()


def test_grid_is_non_trivial() -> None:
    """Guard against the grid accidentally filtering down to nothing — that
    would make the parametrized sweep silently pass with zero cases."""
    assert len(_GRID) > 100, f"workspace grid too small ({len(_GRID)} points)"


@pytest.mark.parametrize("target", _GRID, ids=lambda p: f"x{p[0]:.0f}_y{p[1]:.0f}_z{p[2]:.0f}")
def test_ik_fk_round_trip(target: tuple[float, float, float]) -> None:
    """FK(IK(p)) must reproduce p for every reachable foot target.

    Solve with limits=None so servo-limit clamping cannot distort the geometric
    round-trip; limit behaviour is checked separately below.
    """
    angles = solve(target, limits=None)
    recovered = foot_position(angles)
    err = recovered.distance_to(Vec3(*target))
    assert err < TOL_MM, f"round-trip error {err:.3e} mm at {target} -> {angles}"


def test_neutral_pose() -> None:
    """The configured neutral stance solves to zero coxa yaw and a sane knee-up
    pose, and FK recovers the exact neutral foot."""
    neutral = ROBOT.neutral_foot()
    a = solve(neutral, limits=None)
    assert math.isclose(a.coxa, 0.0, abs_tol=TOL_MM)
    assert 0.0 < a.femur < 45.0          # thigh lifted a little
    assert 90.0 < a.tibia < 140.0        # shin folded into a stance
    assert foot_position(a).distance_to(Vec3(*neutral)) < TOL_MM


@pytest.mark.parametrize("x,y", [(120, 0), (100, 50), (100, -50), (80, 80), (140, -30)])
def test_coxa_equals_atan2(x: float, y: float) -> None:
    """The coxa joint alone resolves the ground-plane bearing to the foot."""
    a = solve((x, y, -90.0), limits=None)
    assert math.isclose(a.coxa, math.degrees(math.atan2(y, x)), abs_tol=TOL_MM)


@pytest.mark.parametrize(
    "angles",
    [
        JointAngles(0.0, 0.0, 0.0),
        JointAngles(20.0, 30.0, 80.0),
        JointAngles(-35.0, -20.0, 120.0),
        JointAngles(45.0, 60.0, 150.0),
    ],
)
def test_segment_lengths_preserved(angles: JointAngles) -> None:
    """FK invariant, independent of the IK: consecutive joints are always
    exactly one segment length apart, for any joint angles."""
    g = ROBOT.geometry
    hip, femur_joint, knee, foot = joint_positions(angles)
    assert math.isclose(hip.distance_to(femur_joint), g.coxa_mm, abs_tol=TOL_MM)
    assert math.isclose(femur_joint.distance_to(knee), g.femur_mm, abs_tol=TOL_MM)
    assert math.isclose(knee.distance_to(foot), g.tibia_mm, abs_tol=TOL_MM)


def test_unreachable_is_flagged_and_degrades_gracefully() -> None:
    """A target beyond max reach is reported unreachable and the solver returns
    a straight-leg best effort (tibia≈0) instead of raising."""
    far = (400.0, 0.0, -50.0)
    assert not is_reachable(far)
    a = solve(far, limits=None)              # must not raise
    assert math.isclose(a.tibia, 0.0, abs_tol=1e-6)


def test_joint_limits_are_enforced() -> None:
    """With limits applied, a target that demands more coxa yaw than allowed is
    clamped into the configured range (servo protection)."""
    # atan2(200, 30) ≈ 81° of coxa yaw, well beyond the ±45° limit.
    a = solve((30.0, 200.0, -60.0))   # default limits applied
    lim = ROBOT.limits(Joint.COXA)
    assert lim.contains(a.coxa)
    assert math.isclose(a.coxa, lim.max_deg, abs_tol=TOL_MM)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
