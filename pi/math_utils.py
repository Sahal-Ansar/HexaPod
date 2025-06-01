"""Small vector and angle helpers shared across kinematics and gait.

Deliberately pure-Python (only the stdlib ``math`` module) so it is cheap to
import and trivial to unit-test on a laptop with no numpy/hardware. The heavy
numerical work (IK trig, gait curves) is light enough that a dependency on
numpy isn't justified for the per-leg, per-tick math; numpy is reserved for the
bulk test sweeps.

Units convention for this whole codebase: **distances in millimetres, angles in
degrees** at every public boundary. The few helpers that need radians convert
internally, so callers never have to think about it.
"""

from __future__ import annotations

import math
from typing import NamedTuple


# ════════════════════════════════════════════════════════════════════════════
# Scalar helpers
# ════════════════════════════════════════════════════════════════════════════
def deg2rad(deg: float) -> float:
    """Degrees → radians."""
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    """Radians → degrees."""
    return rad * 180.0 / math.pi


def clamp(value: float, lo: float, hi: float) -> float:
    """Constrain ``value`` to the closed interval [lo, hi]."""
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolate from ``a`` (t=0) to ``b`` (t=1). ``t`` is NOT clamped,
    so callers can deliberately extrapolate; clamp ``t`` first if undesired."""
    return a + (b - a) * t


def wrap_deg(angle_deg: float) -> float:
    """Wrap an angle into (-180, 180]. Keeps coxa/yaw values continuous so a
    turn through ±180° doesn't blow up into huge numbers."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    # (%-based wrap yields [-180, 180); nudge the -180 edge up to +180 so the
    # range is symmetric and matches the docstring.)
    return 180.0 if wrapped == -180.0 else wrapped


def wrap_rad(angle_rad: float) -> float:
    """Wrap an angle into (-π, π]."""
    wrapped = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if wrapped == -math.pi else wrapped


# ════════════════════════════════════════════════════════════════════════════
# 3D point / vector type
# ════════════════════════════════════════════════════════════════════════════
class Vec3(NamedTuple):
    """An immutable 3D point or vector (x, y, z) in millimetres.

    Implemented as a NamedTuple so it stays lightweight and stays *compatible
    with plain (x, y, z) tuples* — e.g. ``Vec3(*ROBOT.neutral_foot())`` — while
    still offering the vector arithmetic the kinematics needs. Being immutable
    means a foot target can be passed around without fear of aliasing bugs.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    # ── arithmetic (operators return Vec3, not tuple concatenation) ──
    def __add__(self, other: "Vec3") -> "Vec3":  # type: ignore[override]
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":  # type: ignore[override]
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__  # allow scalar * Vec3

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    # ── products & magnitudes ──
    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    @property
    def norm(self) -> float:
        """Euclidean length. Uses math.hypot for numerical robustness."""
        return math.hypot(self.x, self.y, self.z)

    def normalized(self) -> "Vec3":
        """Unit vector in the same direction. Returns the zero vector unchanged
        instead of dividing by zero."""
        n = self.norm
        if n == 0.0:
            return Vec3(0.0, 0.0, 0.0)
        return Vec3(self.x / n, self.y / n, self.z / n)

    def distance_to(self, other: "Vec3") -> float:
        """Distance between this point and ``other``."""
        return (self - other).norm


# ════════════════════════════════════════════════════════════════════════════
# Rotations (active, right-handed; angle in DEGREES, CCW looking down the axis)
# ════════════════════════════════════════════════════════════════════════════
# Used by body kinematics (rotate a foot target by the leg's mount yaw) and IMU
# leveling (apply roll/pitch corrections to the body pose).
def rotate_z(p: Vec3, angle_deg: float) -> Vec3:
    """Rotate point ``p`` about the +Z axis by ``angle_deg`` (yaw)."""
    c = math.cos(deg2rad(angle_deg))
    s = math.sin(deg2rad(angle_deg))
    return Vec3(c * p.x - s * p.y, s * p.x + c * p.y, p.z)


def rotate_y(p: Vec3, angle_deg: float) -> Vec3:
    """Rotate point ``p`` about the +Y axis by ``angle_deg`` (pitch)."""
    c = math.cos(deg2rad(angle_deg))
    s = math.sin(deg2rad(angle_deg))
    return Vec3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z)


def rotate_x(p: Vec3, angle_deg: float) -> Vec3:
    """Rotate point ``p`` about the +X axis by ``angle_deg`` (roll)."""
    c = math.cos(deg2rad(angle_deg))
    s = math.sin(deg2rad(angle_deg))
    return Vec3(p.x, c * p.y - s * p.z, s * p.y + c * p.z)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.math_utils`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    def approx(a: float, b: float, tol: float = 1e-9) -> bool:
        return math.isclose(a, b, abs_tol=tol)

    def vapprox(a: Vec3, b: Vec3, tol: float = 1e-9) -> bool:
        return a.distance_to(b) < tol

    print("math_utils self-test")
    print("=" * 48)

    # Angle conversions round-trip.
    assert approx(rad2deg(deg2rad(37.0)), 37.0)
    assert approx(deg2rad(180.0), math.pi)
    print("deg/rad conversion ............ OK")

    # clamp / lerp.
    assert clamp(5, 0, 10) == 5 and clamp(-3, 0, 10) == 0 and clamp(99, 0, 10) == 10
    assert approx(lerp(10, 20, 0.25), 12.5)
    print("clamp / lerp .................. OK")

    # Angle wrapping.
    assert approx(wrap_deg(190.0), -170.0)
    assert approx(wrap_deg(-190.0), 170.0)
    assert approx(wrap_deg(180.0), 180.0)   # +180 edge preserved
    assert approx(wrap_rad(3.0 * math.pi), math.pi)
    print("wrap_deg / wrap_rad ........... OK")

    # Vec3 arithmetic.
    a = Vec3(1.0, 2.0, 2.0)
    b = Vec3(4.0, 0.0, 0.0)
    assert (a + b) == Vec3(5.0, 2.0, 2.0)
    assert (a - b) == Vec3(-3.0, 2.0, 2.0)
    assert (2.0 * a) == Vec3(2.0, 4.0, 4.0) == (a * 2.0)
    assert approx(a.norm, 3.0)                      # 1-2-2 → length 3
    assert approx(a.dot(b), 4.0)
    assert a.cross(Vec3(0, 0, 0)) == Vec3(0, 0, 0)
    # x̂ × ŷ = ẑ
    assert Vec3(1, 0, 0).cross(Vec3(0, 1, 0)) == Vec3(0, 0, 1)
    assert approx(a.normalized().norm, 1.0)
    assert Vec3(0, 0, 0).normalized() == Vec3(0, 0, 0)  # no divide-by-zero
    print("Vec3 arithmetic ............... OK")

    # Rotations: +90° about Z takes +X̂ → +Ŷ.
    assert vapprox(rotate_z(Vec3(1, 0, 0), 90.0), Vec3(0, 1, 0), 1e-12)
    # +90° about Y takes +X̂ → -Ẑ.
    assert vapprox(rotate_y(Vec3(1, 0, 0), 90.0), Vec3(0, 0, -1), 1e-12)
    # +90° about X takes +Ŷ → +Ẑ.
    assert vapprox(rotate_x(Vec3(0, 1, 0), 90.0), Vec3(0, 0, 1), 1e-12)
    # Rotations preserve length and compose to identity (full turn).
    p = Vec3(3, -4, 5)
    assert approx(rotate_z(p, 123.0).norm, p.norm)
    assert vapprox(rotate_z(rotate_z(p, 200.0), 160.0), p, 1e-9)  # 360° = identity
    print("rotate_x / rotate_y / rotate_z  OK")

    print("-" * 48)
    print("All math_utils self-tests passed.")


if __name__ == "__main__":
    _selftest()
