r"""Stop-and-turn obstacle avoidance.

When the obstacle flag is raised, the robot stops moving forward and turns IN
PLACE until the way ahead is clear, then resumes. This controller owns the
details the bare state machine shouldn't:

  * TURN DIRECTION — with only a single front-facing sonar we can't see which
    side is clearer, so we commit to one direction (config.direction) for the
    whole maneuver rather than dithering. If we're still blocked after
    reverse_after_s (we probably turned into a corner / wall), we flip direction
    and try the other way.
  * OVER-TURN / SETTLE — when the flag clears we keep turning a little longer
    (settle_s) so we don't resume the instant the obstacle's edge slips out of
    the beam and immediately clip it again. Only after the settle do we report
    "done" so the state machine can resume walking.

update(obstacle, dt) -> (command, done):
    command : the GaitCommand to execute this tick (turn in place, or stop).
    done    : True once the maneuver is complete and walking can resume.
"""

from __future__ import annotations

from dataclasses import dataclass

from pi.gait.engine import GaitCommand


@dataclass(frozen=True)
class AvoidanceConfig:
    turn_deg_s: float = 45.0       # in-place turn rate during avoidance
    settle_s: float = 0.4          # keep turning this long after the flag clears
    reverse_after_s: float = 3.0   # if still blocked this long, flip turn direction
    direction: int = -1            # -1 = turn right (CW), +1 = turn left (CCW)


class AvoidanceController:
    """Runs one stop-and-turn maneuver, tracking direction and timers."""

    def __init__(self, config: AvoidanceConfig = AvoidanceConfig()) -> None:
        self.config = config
        self._dir = config.direction
        self._blocked_t = 0.0   # time continuously blocked (for reversal)
        self._clear_t = 0.0     # time since the flag cleared (for settle)
        self.active = False

    @property
    def turn_direction(self) -> int:
        return self._dir

    def start(self) -> None:
        """Begin a maneuver: reset timers and the committed turn direction."""
        self.active = True
        self._dir = self.config.direction
        self._blocked_t = 0.0
        self._clear_t = 0.0

    def update(self, obstacle: bool, dt_s: float) -> tuple[GaitCommand, bool]:
        """Advance the maneuver one tick. Returns (command, done)."""
        cfg = self.config
        if obstacle:
            self._clear_t = 0.0
            self._blocked_t += dt_s
            # Stuck against something on this side — try turning the other way.
            if cfg.reverse_after_s > 0.0 and self._blocked_t >= cfg.reverse_after_s:
                self._dir = -self._dir
                self._blocked_t = 0.0
        else:
            self._clear_t += dt_s
            self._blocked_t = 0.0
            if self._clear_t >= cfg.settle_s:
                self.active = False
                return GaitCommand(), True   # clear long enough -> resume

        # Stop forward/strafe; turn in place in the committed direction.
        return GaitCommand(vx=0.0, vy=0.0, omega_deg_s=self._dir * cfg.turn_deg_s), False


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.avoidance`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    print("avoidance controller self-test")
    print("=" * 60)
    dt = 0.02

    # 1) While blocked: stops forward motion, turns in the committed direction.
    av = AvoidanceController(AvoidanceConfig(turn_deg_s=45.0, direction=-1))
    av.start()
    cmd, done = av.update(obstacle=True, dt_s=dt)
    assert not done
    assert cmd.vx == 0.0 and cmd.vy == 0.0
    assert math.isclose(cmd.omega_deg_s, -45.0), "turn right (CW) at turn rate"
    print("blocked -> stop + turn in place ................. OK")

    # 2) Clearing: keeps turning through the settle window, then reports done.
    av = AvoidanceController(AvoidanceConfig(settle_s=0.1))  # 5 ticks @ 50 Hz
    av.start()
    av.update(True, dt)                      # got blocked once
    clear_ticks = 0
    done = False
    for _ in range(20):
        cmd, done = av.update(obstacle=False, dt_s=dt)
        clear_ticks += 1
        if done:
            break
        assert cmd.omega_deg_s != 0.0, "still over-turning during settle"
    expected = round(av.config.settle_s / dt)  # 5
    assert done and clear_ticks == expected, f"done after {expected} clear ticks"
    assert cmd.omega_deg_s == 0.0, "stops turning when done"
    print(f"clear -> over-turn {av.config.settle_s}s then done .......... OK")

    # 3) Settle resets if the obstacle reappears mid-settle (no premature resume).
    av = AvoidanceController(AvoidanceConfig(settle_s=0.2))
    av.start()
    av.update(True, dt)
    av.update(False, dt)                     # start settling
    av.update(False, dt)
    _, done = av.update(True, dt)            # obstacle back -> settle resets
    assert not done
    # now it must settle the FULL duration again before finishing
    ticks = 0
    while True:
        _, done = av.update(False, dt)
        ticks += 1
        if done:
            break
    assert abs(ticks - round(0.2 / dt)) <= 1, "full settle required again"
    print("obstacle reappearing mid-settle resets it ...... OK")

    # 4) Stuck: still blocked after reverse_after_s -> flip turn direction.
    av = AvoidanceController(AvoidanceConfig(reverse_after_s=0.2, direction=-1))
    av.start()
    assert av.turn_direction == -1
    flipped_at = None
    for i in range(40):
        cmd, _ = av.update(obstacle=True, dt_s=dt)
        if av.turn_direction == 1 and flipped_at is None:
            flipped_at = i + 1   # number of blocked ticks before the flip
    assert flipped_at is not None
    # ~reverse_after_s / dt ticks (allow ±1 tick for float accumulation).
    assert abs(flipped_at - round(0.2 / dt)) <= 1, "reverses after timeout"
    print("stuck against a wall -> reverses direction ..... OK")

    # 5) Direction commitment: stays the chosen way while blocked (no dithering).
    av = AvoidanceController(AvoidanceConfig(direction=1, reverse_after_s=0.0))
    av.start()
    for _ in range(50):
        cmd, _ = av.update(True, dt)
        assert cmd.omega_deg_s > 0.0, "commits to left turn, no flip-flop"
    print("commits to one direction while blocked ......... OK")

    print("-" * 60)
    print("All avoidance controller self-tests passed.")


if __name__ == "__main__":
    _selftest()
