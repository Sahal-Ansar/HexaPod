r"""Tripod gait scheduler — the timing layer of the walk.

The classic insect gait. The six legs are split into two interleaved tripods:

    TRIPOD A : L1 (front-left)  · R2 (mid-right) · L3 (rear-left)
    TRIPOD B : R1 (front-right) · L2 (mid-left)  · R3 (rear-right)

Each tripod is a wide triangle straddling the centreline, so three planted feet
always enclose the centre of mass — statically stable. The two tripods run the
same swing/stance cycle but **180° out of phase**: while A is planted and
pushing the body forward (stance), B is in the air swinging to its next spot,
then they swap. With a 0.5 duty factor this means *exactly three feet are on the
ground at every instant*.

This module only does TIMING: it advances one global phase and hands each leg
its phase (group A = φ, group B = φ + 0.5). It says nothing about direction or
speed — those live in the step vector (the trajectory/engine). To walk backward
you flip the step vector, not the phase; the cycle always runs forward.

    cycle:   A |--- stance ---|---  swing  ---|
             B |---  swing  ---|--- stance ---|
                φ=0          φ=0.5          φ=1
"""

from __future__ import annotations

from pi.config import LEG_ORDER, LegId
from pi.gait.trajectory import StepParams, is_stance

# The two tripods. Membership is fixed by the leg layout.
TRIPOD_A: tuple[LegId, ...] = (LegId.L1, LegId.R2, LegId.L3)
TRIPOD_B: tuple[LegId, ...] = (LegId.R1, LegId.L2, LegId.R3)

# Phase offset applied to each leg: group A leads, group B is half a cycle behind.
_GROUP_OFFSET: dict[LegId, float] = {leg: 0.0 for leg in TRIPOD_A}
_GROUP_OFFSET.update({leg: 0.5 for leg in TRIPOD_B})


class TripodScheduler:
    """Advances a global gait phase and reports each leg's phase.

    period_s is the duration of one full swing+stance cycle; shorter = faster
    stepping. The phase only ever moves forward — speed/direction of travel are
    the step vector's job, not the scheduler's.
    """

    def __init__(self, period_s: float = 1.0) -> None:
        self.period_s = max(1e-3, period_s)
        self._phase = 0.0

    @property
    def phase(self) -> float:
        """The global cycle phase, 0..1 (tripod A's phase)."""
        return self._phase

    def reset(self, phase: float = 0.0) -> None:
        self._phase = phase % 1.0

    def set_period(self, period_s: float) -> None:
        """Change stepping speed without disturbing the current phase."""
        self.period_s = max(1e-3, period_s)

    def advance(self, dt_s: float) -> None:
        """Advance the phase by a control-tick's worth of time."""
        self._phase = (self._phase + dt_s / self.period_s) % 1.0

    def leg_phase(self, leg: LegId) -> float:
        """This leg's phase right now (group offset applied)."""
        return (self._phase + _GROUP_OFFSET[leg]) % 1.0

    def phases(self) -> dict[LegId, float]:
        """Every leg's current phase, in canonical leg order."""
        return {leg: self.leg_phase(leg) for leg in LEG_ORDER}

    @staticmethod
    def group_of(leg: LegId) -> str:
        return "A" if leg in TRIPOD_A else "B"


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.gait.tripod`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    print("tripod scheduler self-test")
    print("=" * 64)

    # Groups partition all six legs, three each, no overlap.
    assert set(TRIPOD_A) | set(TRIPOD_B) == set(LEG_ORDER)
    assert set(TRIPOD_A) & set(TRIPOD_B) == set()
    assert len(TRIPOD_A) == len(TRIPOD_B) == 3
    print("two tripods partition all 6 legs, 3 + 3 ............... OK")

    sched = TripodScheduler(period_s=1.0)

    # The two groups are always exactly half a cycle apart.
    for _ in range(50):
        sched.advance(0.017)
        pa = sched.leg_phase(LegId.L1)   # group A
        pb = sched.leg_phase(LegId.R1)   # group B
        diff = (pb - pa) % 1.0
        assert math.isclose(diff, 0.5, abs_tol=1e-9), "groups must be 180° apart"
    print("groups stay 180 deg out of phase ..................... OK")

    # THE stability invariant: with duty 0.5, exactly 3 feet are grounded at
    # every phase — including the exact swap instants.
    duty = StepParams(duty=0.5)
    sched.reset(0.0)
    for i in range(2000):
        ph = i / 2000.0
        sched.reset(ph)
        grounded = sum(1 for leg in LEG_ORDER if is_stance(sched.leg_phase(leg), duty))
        assert grounded == 3, f"phase {ph}: {grounded} feet grounded (want 3)"
    print("exactly 3 feet grounded at all phases ................ OK")

    # When group A is mid-stance, group B is mid-swing.
    sched.reset(0.25)
    assert is_stance(sched.leg_phase(LegId.L1), duty)        # A grounded
    assert not is_stance(sched.leg_phase(LegId.R1), duty)    # B in air
    sched.reset(0.75)
    assert not is_stance(sched.leg_phase(LegId.L1), duty)    # A in air
    assert is_stance(sched.leg_phase(LegId.R1), duty)        # B grounded
    print("groups alternate stance/swing correctly .............. OK")

    # Timing: advancing by exactly one period returns to the start phase.
    sched.reset(0.0)
    for _ in range(100):
        sched.advance(1.0 / 100)   # 100 ticks of a 1 s period
    assert math.isclose(sched.phase, 0.0, abs_tol=1e-9) or \
           math.isclose(sched.phase, 1.0, abs_tol=1e-9)
    print("one period of ticks returns to phase 0 ............... OK")

    # Period controls speed: half the period -> twice the phase per tick.
    fast = TripodScheduler(period_s=0.5)
    fast.advance(0.1)
    assert math.isclose(fast.phase, 0.2, abs_tol=1e-9)  # 0.1 / 0.5
    print("period scales stepping speed ......................... OK")

    # Visualise the two tripods over one cycle.
    print("\ntripod timeline over one cycle (S = stance/planted, . = swing):")
    cols = [i / 20.0 for i in range(20)]
    print("    phase:  0.0" + " " * 16 + "0.5" + " " * 16 + "1.0")
    for label, legs in (("A (L1 R2 L3)", TRIPOD_A), ("B (R1 L2 R3)", TRIPOD_B)):
        row = []
        for c in cols:
            sched.reset(c)
            row.append("S" if is_stance(sched.leg_phase(legs[0]), duty) else ".")
        print(f"  tripod {label}: " + "".join(row))

    print("-" * 64)
    print("All tripod scheduler self-tests passed.")


if __name__ == "__main__":
    _selftest()
