"""Robot geometry and configuration constants.

This module is the *single source of truth* for the hexapod's physical shape:
how long each leg segment is, where each leg is bolted onto the body, and how
far each joint is allowed to move. Every other module (kinematics, gait,
control) imports from here so there is exactly one place to change when the real
robot's measurements differ from these defaults.

It is pure Python (no numpy / hardware imports) so it is cheap and safe to
import anywhere, including from unit tests on a laptop.

────────────────────────────────────────────────────────────────────────────
COORDINATE CONVENTIONS  (right-handed, millimetres, degrees)
────────────────────────────────────────────────────────────────────────────

BODY frame — origin at the geometric centre of the body, on the "hip plane":
    +X : forward   (direction the robot walks when going straight)
    +Y : left
    +Z : up
    roll  = rotation about +X, pitch = rotation about +Y, yaw = about +Z.

LEG frame — origin at the leg's coxa joint (the vertical servo axis). The leg
frame is the body frame translated to the coxa, then rotated by the leg's
mount yaw so that:
    +X : points straight out from the body along the leg's neutral direction
    +Y : "forward" along the walking direction for that leg (swing direction)
    +Z : up
All single-leg inverse kinematics is done in this LEG frame, which is why every
leg can reuse the same IK code regardless of where it is mounted.

JOINT-ANGLE conventions (used by IK in a later stage; limits below clamp them):
    coxa  θ1 : yaw about leg +Z.  0 = foot straight out (+X). +ve swings +Y.
    femur θ2 : pitch of the thigh about the hip. 0 = femur horizontal.
               +ve lifts the foot UP.
    tibia θ3 : knee angle of the shin relative to the femur.
               0 = shin in line with femur (fully extended/straight).
               +ve bends the knee so the foot tucks DOWN/under the body.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


# ════════════════════════════════════════════════════════════════════════════
# Identifiers
# ════════════════════════════════════════════════════════════════════════════
class LegId(IntEnum):
    """The six legs, ordered so that legs 0-2 are the left side and 3-5 the
    right. This matches the servo wiring: PCA9685 board #0 drives legs 0-2
    (9 servos) and board #1 drives legs 3-5 (9 servos)."""

    L1 = 0  # front-left
    L2 = 1  # mid-left
    L3 = 2  # rear-left
    R1 = 3  # front-right
    R2 = 4  # mid-right
    R3 = 5  # rear-right


class Joint(IntEnum):
    """The three joints of every leg, ordered proximal → distal (hip → foot)."""

    COXA = 0   # horizontal swing (yaw)
    FEMUR = 1  # thigh lift (pitch)
    TIBIA = 2  # knee / shin


# ════════════════════════════════════════════════════════════════════════════
# Leg segment geometry
# ════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class LegGeometry:
    """Length of each leg segment, in millimetres. All six legs are identical.

    These are the lever arms used by the inverse kinematics. **MEASURE THESE ON
    THE REAL ROBOT** — coxa is from the coxa servo axis to the femur axis,
    femur from the femur axis to the knee axis, tibia from the knee axis to the
    very tip of the foot that touches the ground.
    """

    coxa_mm: float
    femur_mm: float
    tibia_mm: float

    @property
    def max_reach_mm(self) -> float:
        """Furthest the foot can be from the coxa axis (fully straight leg)."""
        return self.coxa_mm + self.femur_mm + self.tibia_mm


# ════════════════════════════════════════════════════════════════════════════
# Per-joint angle limits
# ════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class JointLimits:
    """Allowed travel for one joint, in degrees, in the IK joint-angle
    convention documented in the module header.

    These exist to protect the servos and the linkage from commands the IK
    solver might produce at the edge of (or outside) the reachable workspace.
    The servo *calibration* (neutral pulse, direction sign, microsecond range)
    is a separate concern handled later in comms/servo_map.py.
    """

    min_deg: float
    max_deg: float

    def clamp(self, angle_deg: float) -> float:
        """Return ``angle_deg`` limited to [min_deg, max_deg]."""
        return min(self.max_deg, max(self.min_deg, angle_deg))

    def contains(self, angle_deg: float) -> bool:
        """True if ``angle_deg`` is within the allowed range."""
        return self.min_deg <= angle_deg <= self.max_deg


# ════════════════════════════════════════════════════════════════════════════
# Per-leg mounting (where each leg attaches to the body)
# ════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class LegMount:
    """Pose of one leg's coxa joint in the BODY frame.

    (mount_x, mount_y, mount_z) is the position of the coxa servo axis relative
    to the body centre, in mm. mount_yaw_deg is the direction the leg points
    when its coxa angle is 0 — i.e. how the leg frame's +X is rotated relative
    to the body frame's +X, measured as a yaw about +Z (CCW positive).

    Body layout (viewed from above, +X = forward / up the page, +Y = left):

            L1 (front-left)            R1 (front-right)
                  \\                        /
            L2 ----[   body centre   ]---- R2   (middle legs widest)
                  /                        \\
            L3 (rear-left)             R3 (rear-right)
    """

    leg_id: LegId
    name: str
    mount_x: float
    mount_y: float
    mount_z: float
    mount_yaw_deg: float


# ════════════════════════════════════════════════════════════════════════════
# The assembled robot configuration
# ════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class RobotConfig:
    """Everything a kinematics/gait module needs to know about the body."""

    geometry: LegGeometry
    mounts: dict[LegId, LegMount]
    joint_limits: dict[Joint, JointLimits]

    # ── Default standing pose (used by stand-up sequence + gait neutral) ──
    # The neutral foot position for every leg, expressed in that leg's own
    # frame. The foot sits straight out in front of the coxa (+X) and below it
    # (-Z); X/Z here define the robot's nominal stance "footprint" and ride
    # height. Gait swings the foot around this neutral point.
    stance_reach_mm: float   # foot distance straight out from coxa axis (leg +X)
    stance_height_mm: float  # how far the body rides ABOVE the feet (foot at -Z)

    # ── Control timing ──
    control_hz: float        # control-loop tick rate (servo targets/second)

    def neutral_foot(self) -> tuple[float, float, float]:
        """Default foot target (x, y, z) in the leg frame: straight ahead of
        the coxa and below it by the ride height."""
        return (self.stance_reach_mm, 0.0, -self.stance_height_mm)

    def mount(self, leg: LegId) -> LegMount:
        return self.mounts[leg]

    def limits(self, joint: Joint) -> JointLimits:
        return self.joint_limits[joint]


# ════════════════════════════════════════════════════════════════════════════
# DEFAULT CONFIGURATION  — placeholder numbers for a small MG996R hexapod.
# Replace the dimensions below with calipered measurements of YOUR robot.
# ════════════════════════════════════════════════════════════════════════════

# Leg segment lengths (mm) — typical for an MG996R-class build. MEASURE & EDIT.
_GEOMETRY = LegGeometry(
    coxa_mm=30.0,
    femur_mm=85.0,
    tibia_mm=125.0,
)

# Body mount footprint (mm). Front/rear legs are set in from the body edge;
# the middle legs are mounted wider so swinging legs don't collide.
_FRONT_X = 60.0   # front legs this far forward of centre
_REAR_X = -60.0   # rear legs this far behind centre
_SIDE_Y = 60.0    # front/rear legs this far out to the side
_MID_Y = 75.0     # middle legs mounted wider
_MOUNT_Z = 0.0    # all coxae on the hip plane (z = 0)

# Mount yaw for each leg = the outward neutral direction in the body frame.
# Left legs point toward +Y (CCW from +X is positive), right legs toward -Y.
_MOUNTS: dict[LegId, LegMount] = {
    LegId.L1: LegMount(LegId.L1, "front-left",  _FRONT_X,  _SIDE_Y, _MOUNT_Z,  45.0),
    LegId.L2: LegMount(LegId.L2, "mid-left",        0.0,    _MID_Y, _MOUNT_Z,  90.0),
    LegId.L3: LegMount(LegId.L3, "rear-left",   _REAR_X,   _SIDE_Y, _MOUNT_Z, 135.0),
    LegId.R1: LegMount(LegId.R1, "front-right", _FRONT_X, -_SIDE_Y, _MOUNT_Z, -45.0),
    LegId.R2: LegMount(LegId.R2, "mid-right",       0.0,   -_MID_Y, _MOUNT_Z, -90.0),
    LegId.R3: LegMount(LegId.R3, "rear-right",  _REAR_X,  -_SIDE_Y, _MOUNT_Z, -135.0),
}

# Joint travel limits (deg) in the IK convention. Generous but servo-safe;
# tighten once the real linkage's mechanical stops are known.
_JOINT_LIMITS: dict[Joint, JointLimits] = {
    Joint.COXA:  JointLimits(min_deg=-45.0, max_deg=45.0),
    Joint.FEMUR: JointLimits(min_deg=-90.0, max_deg=90.0),
    Joint.TIBIA: JointLimits(min_deg=0.0,   max_deg=150.0),
}

ROBOT = RobotConfig(
    geometry=_GEOMETRY,
    mounts=_MOUNTS,
    joint_limits=_JOINT_LIMITS,
    stance_reach_mm=110.0,   # foot ~110 mm out from each coxa axis
    stance_height_mm=90.0,   # body rides ~90 mm above the feet
    control_hz=50.0,         # 50 Hz control tick (20 ms)
)

# Convenience: legs in a fixed iteration order (left side, then right side).
LEG_ORDER: tuple[LegId, ...] = (
    LegId.L1, LegId.L2, LegId.L3, LegId.R1, LegId.R2, LegId.R3,
)


# ════════════════════════════════════════════════════════════════════════════
# Self-test / inspection: `python -m pi.config` prints the resolved config so
# you can eyeball the geometry without any hardware attached.
# ════════════════════════════════════════════════════════════════════════════
def _print_summary() -> None:
    g = ROBOT.geometry
    print("HexaPod configuration")
    print("=" * 64)
    print(f"Leg segments (mm): coxa={g.coxa_mm}  femur={g.femur_mm}  "
          f"tibia={g.tibia_mm}  max reach={g.max_reach_mm:.1f}")
    print(f"Stance: reach={ROBOT.stance_reach_mm} mm, "
          f"ride height={ROBOT.stance_height_mm} mm")
    print(f"Neutral foot (leg frame): {ROBOT.neutral_foot()}")
    print(f"Control rate: {ROBOT.control_hz} Hz "
          f"({1000.0 / ROBOT.control_hz:.1f} ms/tick)")
    print("-" * 64)
    print(f"{'leg':>10} {'mount(x,y,z) mm':>22} {'yaw°':>6}")
    for leg in LEG_ORDER:
        m = ROBOT.mount(leg)
        pos = f"({m.mount_x:+.0f}, {m.mount_y:+.0f}, {m.mount_z:+.0f})"
        print(f"{m.name:>10} {pos:>22} {m.mount_yaw_deg:>6.0f}")
    print("-" * 64)
    print("Joint limits (deg):")
    for joint in (Joint.COXA, Joint.FEMUR, Joint.TIBIA):
        lim = ROBOT.limits(joint)
        print(f"  {joint.name:<6} [{lim.min_deg:+.0f}, {lim.max_deg:+.0f}]")

    # Sanity checks so a bad edit fails loudly instead of silently misbehaving.
    assert len(ROBOT.mounts) == 6, "expected exactly 6 leg mounts"
    assert ROBOT.stance_reach_mm < g.max_reach_mm, (
        "stance reach exceeds the leg's maximum reach — IK will be unsolvable")
    print("\nAll sanity checks passed.")


if __name__ == "__main__":
    _print_summary()
