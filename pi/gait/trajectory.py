r"""Single-foot swing/stance trajectory, parameterised by phase 0..1.

One walking cycle for a single foot has two parts:

  * STANCE — the foot is on the ground, carrying the body. Relative to the body
    it slides straight BACKWARD, which is what pushes the body forward. It stays
    at ground level the whole time.
  * SWING  — the foot is lifted off the ground and swung FORWARD through the air
    in an arc, back to the front of the stride, ready for the next stance.

We express the motion as an OFFSET from the foot's neutral stance point, in the
body frame (x forward, y left, z up). The gait engine adds this offset to each
leg's neutral foot position, so this module knows nothing about where the legs
are mounted — it just shapes one foot's path.

Parameters:
  * step = (step_x, step_y): the full stride vector, in the direction the body
    travels. The foot ranges from +step/2 (front) to −step/2 (back). The gait
    engine builds a per-leg step vector from the desired (vx, vy, ω); turning
    just means the legs get different step vectors.
  * step_height: how high the foot lifts at mid-swing.
  * duty: fraction of the cycle spent in stance. A tripod gait uses 0.5 (the two
    leg groups are each in stance exactly half the time).

Phase timeline (φ ∈ [0,1)):

      φ=0        φ=duty                      φ=1
      |--------- STANCE ---------|--- SWING ---|
      foot at front              foot at back  foot back at front
      z = 0 (on ground)          z arcs up then back to 0

                       ____                z (side view)
       swing arc:    /      \
                ____/        \____
       stance (on ground): ----------------> slides backward
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pi.math_utils import Vec3, clamp


@dataclass(frozen=True)
class StepParams:
    """Shape of the step, independent of direction/speed."""

    step_height_mm: float = 40.0  # peak foot lift during swing
    duty: float = 0.5             # stance fraction (tripod = 0.5)

    def __post_init__(self) -> None:
        # duty must leave room for both phases; clamp defensively.
        object.__setattr__(self, "duty", clamp(self.duty, 0.05, 0.95))


def is_stance(phase: float, params: StepParams = StepParams()) -> bool:
    """True if this phase is in the stance (on-ground) part of the cycle."""
    return (phase % 1.0) < params.duty


def foot_offset(
    phase: float,
    step_x: float,
    step_y: float,
    params: StepParams = StepParams(),
) -> Vec3:
    """Foot offset from its neutral point at this phase, in the body frame.

    Returns (dx, dy, dz): dx/dy slide the foot within the stride, dz lifts it
    during swing (dz=0 during stance). Periodic with period 1.
    """
    phase %= 1.0
    beta = params.duty

    if phase < beta:
        # ── STANCE: linear slide from +step/2 (front) to −step/2 (back) ──
        # Linear so the foot moves at constant speed opposite the body — no
        # ground slipping. s goes 0→1 across the stance.
        s = phase / beta
        fx = step_x * (0.5 - s)
        fy = step_y * (0.5 - s)
        fz = 0.0
    else:
        # ── SWING: linear slide back from −step/2 to +step/2, lifted in an arc ──
        # u goes 0→1 across the swing. Horizontal is linear; vertical is a half
        # sine, which is 0 at lift-off and touchdown and peaks at mid-swing.
        u = (phase - beta) / (1.0 - beta)
        fx = step_x * (u - 0.5)
        fy = step_y * (u - 0.5)
        fz = params.step_height_mm * math.sin(math.pi * u)

    return Vec3(fx, fy, fz)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.gait.trajectory`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    def approx(a: float, b: float, tol: float = 1e-9) -> bool:
        return math.isclose(a, b, abs_tol=tol)

    print("trajectory self-test")
    print("=" * 60)

    params = StepParams(step_height_mm=40.0, duty=0.5)
    STEP = 80.0  # 80 mm forward stride for the checks below

    # Key positions around the cycle (forward step: step_x=STEP, step_y=0).
    front = foot_offset(0.0, STEP, 0.0, params)           # start of stance
    back_end = foot_offset(0.5 - 1e-9, STEP, 0.0, params)  # end of stance
    swing_start = foot_offset(0.5, STEP, 0.0, params)      # start of swing
    mid_swing = foot_offset(0.75, STEP, 0.0, params)       # mid swing (apex)
    cycle_end = foot_offset(1.0 - 1e-9, STEP, 0.0, params)

    assert approx(front.x, STEP / 2) and approx(front.z, 0.0)
    assert approx(back_end.x, -STEP / 2, 1e-6) and approx(back_end.z, 0.0)
    print("stance: front (+step/2) -> back (-step/2), on ground .. OK")

    assert approx(swing_start.x, -STEP / 2) and approx(swing_start.z, 0.0)
    assert approx(mid_swing.x, 0.0) and approx(mid_swing.z, params.step_height_mm)
    assert approx(cycle_end.x, STEP / 2, 1e-6) and approx(cycle_end.z, 0.0, 1e-6)
    print("swing: back -> front, apex lift at mid-swing .......... OK")

    # Phase/stance transitions are continuous (no foot teleport).
    eps = 1e-7
    a = foot_offset(0.5 - eps, STEP, 0.0, params)
    b = foot_offset(0.5 + eps, STEP, 0.0, params)
    assert a.distance_to(b) < 1e-3, "stance->swing must be continuous"
    a = foot_offset(1.0 - eps, STEP, 0.0, params)
    b = foot_offset(0.0 + eps, STEP, 0.0, params)
    assert a.distance_to(b) < 1e-3, "cycle wrap must be continuous"
    print("continuity at stance<->swing and cycle wrap .......... OK")

    # Invariants across the whole cycle.
    for i in range(1000):
        ph = i / 1000.0
        off = foot_offset(ph, STEP, 0.0, params)
        if is_stance(ph, params):
            assert approx(off.z, 0.0), "foot must stay grounded during stance"
        else:
            assert -1e-9 <= off.z <= params.step_height_mm + 1e-9, "swing lift bounds"
        # Periodicity.
        assert foot_offset(ph + 1.0, STEP, 0.0, params).distance_to(off) < 1e-9
    print("stance grounded, swing lift bounded, periodic ........ OK")

    # Strafe + turn components flow through unchanged (2D stride).
    diag = foot_offset(0.75, 40.0, 30.0, params)  # mid-swing apex
    assert approx(diag.x, 0.0) and approx(diag.y, 0.0)
    assert approx(diag.z, params.step_height_mm)
    print("2D stride (forward+strafe) handled ................... OK")

    # Side-view sketch of the foot path so you can *see* the gait.
    print("\nfoot path (side view, forward step, x = stride, ^ = height):")
    _plot_side_view(params, STEP)

    print("-" * 60)
    print("All trajectory self-tests passed.")


def _plot_side_view(params: StepParams, step: float) -> None:
    width, height = 49, 9
    grid = [[" "] * width for _ in range(height)]
    for i in range(width * 2):
        ph = i / (width * 2)
        off = foot_offset(ph, step, 0.0, params)
        col = int((off.x / step + 0.5) * (width - 1))       # -step/2..+step/2 -> 0..w
        row = height - 1 - int((off.z / params.step_height_mm) * (height - 1))
        col = max(0, min(width - 1, col))
        row = max(0, min(height - 1, row))
        grid[row][col] = "o" if off.z > 1e-6 else "."
    for r in grid:
        print("   " + "".join(r))
    print("   back" + " " * (width - 8) + "front")


if __name__ == "__main__":
    _selftest()
