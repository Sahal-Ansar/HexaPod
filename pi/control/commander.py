r"""Velocity / direction command interface feeding the gait engine.

The gait engine consumes a raw GaitCommand(vx, vy, ω). This layer is what a
human (teleop) or the state machine actually drives, adding the two things a
raw command lacks:

  1. LIMITS — clamp forward/strafe/turn to speeds the robot can actually do
     (a stride can't exceed the leg workspace), so callers can't ask for the
     impossible.
  2. RAMPING — slew the live command toward the requested target at a bounded
     acceleration. A key press is an instantaneous step input; feeding that
     straight to the gait would snap the stride and lurch the body. Ramping
     turns "go!" into a smooth speed-up and "stop" into a smooth slow-down.

Typical use, each control tick:

    commander.set_normalized(fwd=1.0, strafe=0.0, turn=-0.3)  # from teleop
    loop.set_command(commander.update(dt))                    # ramped command
"""

from __future__ import annotations

from dataclasses import dataclass

from pi.gait.engine import GaitCommand
from pi.math_utils import clamp


@dataclass(frozen=True)
class VelocityLimits:
    """Top speeds and accelerations for the body.

    Defaults are sized to the gait: max forward ≈ max_step / stance_time, so the
    stride stays inside the leg workspace at top speed. Accelerations give a
    ~0.4 s ramp to full speed — brisk but not lurchy.
    """

    max_vx: float = 120.0          # mm/s forward/back
    max_vy: float = 80.0           # mm/s strafe
    max_omega_deg_s: float = 45.0  # deg/s turn
    accel_lin: float = 300.0       # mm/s^2  (linear slew rate)
    accel_omega: float = 180.0     # deg/s^2 (angular slew rate)


def _approach(current: float, target: float, max_delta: float) -> float:
    """Step ``current`` toward ``target`` by at most ``max_delta`` (>=0)."""
    if target > current:
        return min(current + max_delta, target)
    return max(current - max_delta, target)


class VelocityCommander:
    """Holds the desired and live body velocity, ramping one toward the other."""

    def __init__(self, limits: VelocityLimits = VelocityLimits()) -> None:
        self.limits = limits
        self._target = GaitCommand()   # what we've been asked for (clamped)
        self._current = GaitCommand()  # what we're actually outputting (ramped)

    # ── set the target ──
    def set_target(self, vx: float = 0.0, vy: float = 0.0, omega_deg_s: float = 0.0) -> None:
        """Set the target in physical units (mm/s, deg/s), clamped to limits."""
        lim = self.limits
        self._target = GaitCommand(
            vx=clamp(vx, -lim.max_vx, lim.max_vx),
            vy=clamp(vy, -lim.max_vy, lim.max_vy),
            omega_deg_s=clamp(omega_deg_s, -lim.max_omega_deg_s, lim.max_omega_deg_s),
        )

    def set_normalized(self, fwd: float = 0.0, strafe: float = 0.0, turn: float = 0.0) -> None:
        """Set the target from joystick-style inputs in [-1, 1], scaled to the
        max speeds. fwd = forward, strafe = left, turn = CCW."""
        lim = self.limits
        self.set_target(
            vx=clamp(fwd, -1.0, 1.0) * lim.max_vx,
            vy=clamp(strafe, -1.0, 1.0) * lim.max_vy,
            omega_deg_s=clamp(turn, -1.0, 1.0) * lim.max_omega_deg_s,
        )

    def stop(self) -> None:
        """Request zero velocity (the robot will ramp down, not snap)."""
        self._target = GaitCommand()

    # ── ramp toward the target ──
    def update(self, dt_s: float) -> GaitCommand:
        """Advance the live command toward the target by one tick and return it."""
        dv = self.limits.accel_lin * dt_s
        dw = self.limits.accel_omega * dt_s
        self._current = GaitCommand(
            vx=_approach(self._current.vx, self._target.vx, dv),
            vy=_approach(self._current.vy, self._target.vy, dv),
            omega_deg_s=_approach(self._current.omega_deg_s, self._target.omega_deg_s, dw),
        )
        return self._current

    # ── introspection ──
    @property
    def current(self) -> GaitCommand:
        return self._current

    @property
    def target(self) -> GaitCommand:
        return self._target

    def is_idle(self, eps: float = 1e-6) -> bool:
        """True once the live command has fully ramped down to a stop."""
        return self._current.is_zero(eps)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.commander`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    print("velocity commander self-test")
    print("=" * 60)
    cmd = VelocityCommander()
    lim = cmd.limits
    dt = 0.02  # 50 Hz

    # Limits: asking for too much clamps to the max.
    cmd.set_target(vx=9999.0, vy=-9999.0, omega_deg_s=9999.0)
    assert cmd.target.vx == lim.max_vx
    assert cmd.target.vy == -lim.max_vy
    assert cmd.target.omega_deg_s == lim.max_omega_deg_s
    print("set_target clamps to velocity limits ......... OK")

    # Normalized inputs scale to max speed.
    cmd.set_normalized(fwd=0.5, strafe=-1.0, turn=2.0)  # turn over-range -> clamp 1.0
    assert math.isclose(cmd.target.vx, 0.5 * lim.max_vx)
    assert math.isclose(cmd.target.vy, -1.0 * lim.max_vy)
    assert math.isclose(cmd.target.omega_deg_s, lim.max_omega_deg_s)
    print("set_normalized scales [-1,1] to max speeds ... OK")

    # Ramp UP: from rest to full forward, monotone, no overshoot, reaches target.
    cmd = VelocityCommander()
    cmd.set_target(vx=lim.max_vx)
    prev = 0.0
    reached_at = None
    for i in range(200):
        c = cmd.update(dt)
        assert c.vx >= prev - 1e-9, "ramp must be monotone"
        assert c.vx <= lim.max_vx + 1e-9, "must not overshoot"
        if reached_at is None and math.isclose(c.vx, lim.max_vx, abs_tol=1e-6):
            reached_at = i * dt
        prev = c.vx
    assert reached_at is not None
    expected = lim.max_vx / lim.accel_lin
    assert math.isclose(reached_at, expected, abs_tol=dt * 1.5), \
        f"ramp time {reached_at:.3f}s vs expected {expected:.3f}s"
    print(f"ramp up reaches full speed in ~{reached_at:.2f}s (a={lim.accel_lin}) . OK")

    # Per-tick step never exceeds the accel limit.
    cmd = VelocityCommander()
    cmd.set_target(vx=lim.max_vx)
    c0 = cmd.update(dt)
    assert c0.vx <= lim.accel_lin * dt + 1e-9
    print("per-tick change bounded by acceleration ...... OK")

    # Ramp DOWN to a stop on command.
    cmd = VelocityCommander()
    cmd.set_target(vx=lim.max_vx)
    for _ in range(200):
        cmd.update(dt)
    assert math.isclose(cmd.current.vx, lim.max_vx, abs_tol=1e-6)
    cmd.stop()
    while not cmd.is_idle():
        cmd.update(dt)
    assert cmd.current.is_zero()
    print("stop() ramps smoothly down to idle .......... OK")

    # Direction reversal passes through zero (no instantaneous flip).
    cmd = VelocityCommander()
    cmd.set_target(vx=lim.max_vx)
    for _ in range(200):
        cmd.update(dt)
    cmd.set_target(vx=-lim.max_vx)
    crossed_zero = False
    for _ in range(400):
        c = cmd.update(dt)
        if abs(c.vx) < lim.accel_lin * dt:
            crossed_zero = True
    assert crossed_zero and math.isclose(cmd.current.vx, -lim.max_vx, abs_tol=1e-6)
    print("forward->backward ramps through zero ......... OK")

    # Integration: feed the ramped command into a real (mock) control loop.
    from pi.control.loop import build_mock_loop
    loop = build_mock_loop()
    cmd = VelocityCommander()
    cmd.set_normalized(fwd=1.0, turn=-0.5)
    for _ in range(30):
        loop.set_command(cmd.update(dt))
        res = loop.tick(dt)
        assert all(500 <= p <= 2500 for p in res.pulses)
    print("ramped command drives the control loop ....... OK")

    print("-" * 60)
    print("All velocity commander self-tests passed.")


if __name__ == "__main__":
    _selftest()
