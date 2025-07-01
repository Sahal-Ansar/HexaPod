r"""Gait engine — turn a desired body velocity into 6 foot targets per tick.

This is where the pieces meet:
    scheduler (tripod.py)   -> each leg's phase
    trajectory (trajectory) -> phase + step vector -> foot offset
    body stance (body.py)   -> each leg's neutral ground point

The interesting part is converting a body velocity command (vx, vy, ω) into a
per-leg STEP VECTOR. A foot planted on the ground must move opposite to the
body's motion *at that foot's location* — otherwise it scrubs/slips. The body's
velocity at a point r = (rx, ry) on the ground is the rigid-body formula:

    v_point = v_linear + ω × r
            = (vx − ω·ry,  vy + ω·rx)            (ω about +z, CCW positive)

Over one stance (duration = duty · period) the foot must cover that much ground,
so the stride for each leg is:

    step = v_point · stance_time

This automatically makes the legs do the right thing when turning: legs farther
from the turn centre (larger |r|) get a bigger stride, and legs on opposite
sides stride in opposite directions — a differential-drive turn, but with feet.

Output is foot targets in the BODY frame. The control loop then applies the body
pose (height / leveling) and runs the IK — keeping leveling decoupled from gait.

Units: vx, vy in mm/s; ω in deg/s (converted to rad/s internally).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pi.config import LEG_ORDER, LegId, RobotConfig, ROBOT
from pi.gait.trajectory import StepParams, foot_offset
from pi.gait.tripod import TripodScheduler
from pi.kinematics.body import default_stance_body
from pi.math_utils import Vec3, deg2rad


@dataclass(frozen=True)
class GaitCommand:
    """Desired body motion for this tick."""

    vx: float = 0.0          # forward speed, mm/s (+ = forward)
    vy: float = 0.0          # strafe speed, mm/s (+ = left)
    omega_deg_s: float = 0.0  # yaw rate, deg/s (+ = CCW / turn left)

    def is_zero(self, eps: float = 1e-6) -> bool:
        return (abs(self.vx) < eps and abs(self.vy) < eps
                and abs(self.omega_deg_s) < eps)


class GaitEngine:
    """Stateful walk generator. Call ``update(command, dt)`` each control tick."""

    def __init__(
        self,
        config: RobotConfig = ROBOT,
        period_s: float = 1.0,
        step_params: StepParams = StepParams(),
        max_step_mm: float = 70.0,
    ) -> None:
        self.config = config
        self.scheduler = TripodScheduler(period_s)
        self.step_params = step_params
        # Cap the stride so a fast command can't drive a foot out of its
        # workspace; exceeding it just saturates speed (safe degradation).
        self.max_step_mm = max_step_mm
        # Neutral ground contact point per leg, in the body frame.
        self._neutral: dict[LegId, Vec3] = default_stance_body(config)

    def _leg_step(self, leg: LegId, command: GaitCommand, stance_time: float) -> tuple[float, float]:
        """Stride vector (sx, sy) for one leg from the velocity command."""
        r = self._neutral[leg]
        omega = deg2rad(command.omega_deg_s)
        # Body velocity at this foot's ground location: v_linear + ω × r.
        vpx = command.vx - omega * r.y
        vpy = command.vy + omega * r.x
        sx = vpx * stance_time
        sy = vpy * stance_time
        # Clamp stride magnitude to keep the foot inside its reachable workspace.
        mag = math.hypot(sx, sy)
        if mag > self.max_step_mm:
            scale = self.max_step_mm / mag
            sx *= scale
            sy *= scale
        return sx, sy

    def update(self, command: GaitCommand, dt_s: float) -> dict[LegId, Vec3]:
        """Advance the gait and return each leg's foot target in the body frame.

        When the command is zero the robot stands: the phase is frozen and every
        foot is held at its planted neutral point. (Smoothly *coming to a stop*
        — letting airborne legs land first — is the state machine's job.)
        """
        if command.is_zero():
            return dict(self._neutral)

        self.scheduler.advance(dt_s)
        stance_time = self.step_params.duty * self.scheduler.period_s

        targets: dict[LegId, Vec3] = {}
        for leg in LEG_ORDER:
            sx, sy = self._leg_step(leg, command, stance_time)
            phase = self.scheduler.leg_phase(leg)
            offset = foot_offset(phase, sx, sy, self.step_params)
            targets[leg] = self._neutral[leg] + offset
        return targets


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.gait.engine`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    from pi.kinematics.body import BodyPose, solve_body
    from pi.kinematics.leg_ik import is_reachable
    from pi.kinematics.body import foot_body_to_leg

    print("gait engine self-test")
    print("=" * 64)

    eng = GaitEngine(period_s=1.0, step_params=StepParams(step_height_mm=40.0, duty=0.5))
    dt = 0.02  # 50 Hz
    neutral = default_stance_body()

    # 1) Pure forward: every leg gets the same straight-ahead stride.
    fwd = GaitCommand(vx=60.0)
    sx0, sy0 = eng._leg_step(LegId.L1, fwd, 0.5)
    for leg in LEG_ORDER:
        sx, sy = eng._leg_step(leg, fwd, 0.5)
        assert math.isclose(sx, sx0) and math.isclose(sy, 0.0), "forward = uniform stride"
    assert sx0 > 0, "forward stride points forward"
    print("pure forward -> identical forward stride per leg ..... OK")

    # 2) Pure turn (CCW): right legs stride forward, left legs backward (opposite
    #    signs), magnitude scales with distance from centre.
    turn = GaitCommand(omega_deg_s=40.0)
    sx_L1, _ = eng._leg_step(LegId.L1, turn, 0.5)   # front-left  (ry>0)
    sx_R1, _ = eng._leg_step(LegId.R1, turn, 0.5)   # front-right (ry<0)
    assert sx_L1 < 0 < sx_R1, "CCW turn: left back, right forward"
    assert math.isclose(sx_L1, -sx_R1, abs_tol=1e-9), "symmetric left/right"
    print("pure turn (CCW) -> differential left/right stride .... OK")

    # 3) Stop: phase frozen, all feet held at neutral (planted).
    eng.scheduler.reset(0.3)
    stopped = eng.update(GaitCommand(), dt)
    assert eng.scheduler.phase == 0.3, "phase frozen when stopped"
    for leg in LEG_ORDER:
        assert stopped[leg].distance_to(neutral[leg]) < 1e-9
    print("zero command -> stand still, all feet planted ........ OK")

    # 4) Stride saturation: a huge command clamps to max_step_mm.
    fast = GaitCommand(vx=100000.0)
    sx, sy = eng._leg_step(LegId.L1, fast, 0.5)
    assert math.isclose(math.hypot(sx, sy), eng.max_step_mm, abs_tol=1e-6)
    print("over-fast command saturates to max stride ........... OK")

    # 5) While moving: at least 3 feet are always on the ground, and the whole
    #    set of targets is reachable + solvable through body IK each tick.
    eng2 = GaitEngine(period_s=1.0)
    min_grounded = 99
    for _ in range(200):  # 4 s of walking
        targets = eng2.update(GaitCommand(vx=50.0, omega_deg_s=20.0), dt)
        grounded = sum(1 for leg in LEG_ORDER
                       if abs(targets[leg].z - neutral[leg].z) < 1e-9)
        min_grounded = min(min_grounded, grounded)
        # every foot target must be reachable in its leg frame
        for leg in LEG_ORDER:
            leg_pt = foot_body_to_leg(ROBOT.mount(leg), targets[leg], BodyPose())
            assert is_reachable(tuple(leg_pt)), f"{leg.name} target unreachable"
        solve_body(targets, BodyPose())  # full IK must not raise
    assert min_grounded >= 3, f"support dropped to {min_grounded} feet"
    print(f"walking 4 s: always >=3 feet down (min {min_grounded}), all IK ok . OK")

    # 6) Net propulsion: averaged over a cycle, stance feet move backward when
    #    walking forward (that's what drives the body forward).
    eng3 = GaitEngine(period_s=1.0)
    from pi.gait.trajectory import is_stance
    backward_motion = 0.0
    prev = None
    for _ in range(100):
        targets = eng3.update(GaitCommand(vx=50.0), dt)
        ph = eng3.scheduler.leg_phase(LegId.L1)
        if prev is not None and is_stance(ph):
            backward_motion += targets[LegId.L1].x - prev
        prev = targets[LegId.L1].x
    assert backward_motion < 0, "stance foot must net-move backward when going forward"
    print("stance feet net-move backward (propulsion) .......... OK")

    print("-" * 64)
    print("All gait engine self-tests passed.")


if __name__ == "__main__":
    _selftest()
