r"""Behavior state machine — IDLE / STAND / WALK / AVOID.

The top-level decision layer. It takes what the user wants (a desired velocity,
stand/sit requests) and what the world looks like (the obstacle flag) and decides
both the robot's behavioural STATE and the actual GaitCommand to execute.

    IDLE  ── stand_request ─────────────▶ STAND        (play stand-up)
    STAND ── desired != 0 ──────────────▶ WALK
    STAND ── sit_request ───────────────▶ IDLE         (play sit-down)
    WALK  ── obstacle ──────────────────▶ AVOID
    WALK  ── desired == 0 ──────────────▶ STAND
    AVOID ── obstacle cleared ──────────▶ WALK / STAND
    (any active) ── sit_request ────────▶ IDLE         (play sit-down)

State-dependent command:
    IDLE  -> zero (servos at the sit pose)
    STAND -> zero (hold the stance)
    WALK  -> the user's desired velocity
    AVOID -> stop forward motion and turn in place (the actual turn-direction
             choice and resume logic are fleshed out in the avoidance stage)

The machine is pure/decision-only: transitions that need a motion sequence
(stand-up, sit-down) are signalled via flags so the runner can play them; the
machine does not block.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from pi.control.avoidance import AvoidanceController
from pi.gait.engine import GaitCommand


class State(Enum):
    IDLE = "IDLE"      # sitting/folded, not standing
    STAND = "STAND"    # standing, body up, holding still
    WALK = "WALK"      # walking per the desired velocity
    AVOID = "AVOID"    # obstacle ahead: stop and turn away


@dataclass
class BehaviorInput:
    """Everything the machine needs each tick."""

    desired: GaitCommand = field(default_factory=GaitCommand)  # user's wanted velocity
    obstacle: bool = False                                     # from perception
    stand_request: bool = False                                # "get up"
    sit_request: bool = False                                  # "sit down"


@dataclass
class BehaviorOutput:
    """The machine's decision for this tick."""

    state: State
    command: GaitCommand                 # the command to actually feed the gait
    start_standup: bool = False          # runner should play the stand-up sequence
    start_sitdown: bool = False          # runner should play the sit-down sequence


class BehaviorStateMachine:
    """Tracks the behavioural state and maps (intent + perception) -> command."""

    def __init__(self, avoidance: AvoidanceController | None = None) -> None:
        self.state = State.IDLE
        # The AVOID state delegates the turn maneuver to this controller.
        self.avoidance = avoidance or AvoidanceController()

    def reset(self) -> None:
        self.state = State.IDLE
        self.avoidance.active = False

    def update(self, inp: BehaviorInput, dt_s: float = 0.0) -> BehaviorOutput:
        # Sit-down request from any active state takes priority -> IDLE.
        if inp.sit_request and self.state in (State.STAND, State.WALK, State.AVOID):
            self.state = State.IDLE
            return BehaviorOutput(State.IDLE, GaitCommand(), start_sitdown=True)

        if self.state is State.IDLE:
            if inp.stand_request:
                self.state = State.STAND
                return BehaviorOutput(State.STAND, GaitCommand(), start_standup=True)
            return BehaviorOutput(State.IDLE, GaitCommand())

        if self.state is State.STAND:
            if not inp.desired.is_zero():
                self.state = State.WALK
                return BehaviorOutput(State.WALK, inp.desired)
            return BehaviorOutput(State.STAND, GaitCommand())

        if self.state is State.WALK:
            if inp.obstacle:
                # Begin a stop-and-turn maneuver.
                self.state = State.AVOID
                self.avoidance.start()
                cmd, _ = self.avoidance.update(inp.obstacle, dt_s)
                return BehaviorOutput(State.AVOID, cmd)
            if inp.desired.is_zero():
                self.state = State.STAND
                return BehaviorOutput(State.STAND, GaitCommand())
            return BehaviorOutput(State.WALK, inp.desired)

        # AVOID: let the controller drive the turn and decide when it's done.
        cmd, done = self.avoidance.update(inp.obstacle, dt_s)
        if done:
            # Maneuver complete: resume walking if still wanted, else stand.
            if inp.desired.is_zero():
                self.state = State.STAND
                return BehaviorOutput(State.STAND, GaitCommand())
            self.state = State.WALK
            return BehaviorOutput(State.WALK, inp.desired)
        return BehaviorOutput(State.AVOID, cmd)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.state_machine`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    from pi.control.avoidance import AvoidanceConfig, AvoidanceController

    print("behavior state machine self-test")
    print("=" * 60)

    fwd = GaitCommand(vx=60.0)
    stop = GaitCommand()
    dt = 0.02

    # Most transition tests use a zero-settle avoidance so "clear" resumes the
    # next tick — isolating the state logic from the avoidance timing (which is
    # covered by avoidance.py's own tests and the settle test below).
    def make_sm(settle: float = 0.0) -> BehaviorStateMachine:
        return BehaviorStateMachine(AvoidanceController(AvoidanceConfig(settle_s=settle)))

    # IDLE ignores movement/obstacle until told to stand.
    sm = make_sm()
    out = sm.update(BehaviorInput(desired=fwd, obstacle=True), dt)
    assert sm.state is State.IDLE and out.command.is_zero()
    print("IDLE ignores move/obstacle ...................... OK")

    # IDLE -> STAND on stand_request, signalling stand-up.
    out = sm.update(BehaviorInput(stand_request=True), dt)
    assert sm.state is State.STAND and out.start_standup
    print("IDLE -> STAND triggers stand-up ................. OK")

    # STAND -> WALK on a move command; command passes through.
    out = sm.update(BehaviorInput(desired=fwd), dt)
    assert sm.state is State.WALK and out.command == fwd
    print("STAND -> WALK on move command .................. OK")

    # WALK -> AVOID on obstacle; command becomes stop + turn.
    out = sm.update(BehaviorInput(desired=fwd, obstacle=True), dt)
    assert sm.state is State.AVOID
    assert out.command.vx == 0.0 and out.command.omega_deg_s != 0.0
    print("WALK -> AVOID: stop and turn in place .......... OK")

    # AVOID holds while the obstacle is present.
    out = sm.update(BehaviorInput(desired=fwd, obstacle=True), dt)
    assert sm.state is State.AVOID and out.command.vx == 0.0
    print("AVOID persists while obstacle present .......... OK")

    # AVOID -> WALK when cleared (zero settle) and the user still wants to move.
    out = sm.update(BehaviorInput(desired=fwd, obstacle=False), dt)
    assert sm.state is State.WALK and out.command == fwd
    print("AVOID -> WALK on clear (resume) ................ OK")

    # WALK -> STAND when the user stops.
    out = sm.update(BehaviorInput(desired=stop), dt)
    assert sm.state is State.STAND and out.command.is_zero()
    print("WALK -> STAND on stop .......................... OK")

    # Over-turn: with a real settle, AVOID keeps turning for settle_s after the
    # obstacle clears, THEN resumes — no premature resume.
    sm = make_sm(settle=0.1)  # 5 ticks @ 50 Hz
    sm.state = State.WALK
    sm.update(BehaviorInput(desired=fwd, obstacle=True), dt)   # enter AVOID
    still_avoiding = 0
    for _ in range(20):
        out = sm.update(BehaviorInput(desired=fwd, obstacle=False), dt)
        if sm.state is State.AVOID:
            still_avoiding += 1
        else:
            break
    assert sm.state is State.WALK and still_avoiding >= 4, "over-turns then resumes"
    print(f"AVOID over-turns ~{still_avoiding} ticks after clear, resumes  OK")

    # Sit-down from any active state -> IDLE, signalling sit-down.
    for start in (State.STAND, State.WALK, State.AVOID):
        sm = make_sm()
        sm.state = start
        out = sm.update(BehaviorInput(sit_request=True), dt)
        assert sm.state is State.IDLE and out.start_sitdown
    print("sit_request from STAND/WALK/AVOID -> IDLE ...... OK")

    # End-to-end scenario walk-through (zero-settle so AVOID clears in one tick).
    sm = make_sm()
    script = [
        (BehaviorInput(stand_request=True), State.STAND),
        (BehaviorInput(desired=fwd), State.WALK),
        (BehaviorInput(desired=fwd), State.WALK),
        (BehaviorInput(desired=fwd, obstacle=True), State.AVOID),
        (BehaviorInput(desired=fwd, obstacle=True), State.AVOID),
        (BehaviorInput(desired=fwd, obstacle=False), State.WALK),
        (BehaviorInput(desired=stop), State.STAND),
        (BehaviorInput(sit_request=True), State.IDLE),
    ]
    for inp, expected in script:
        sm.update(inp, dt)
        assert sm.state is expected, f"expected {expected}, got {sm.state}"
    print("full IDLE->STAND->WALK->AVOID->WALK->STAND->IDLE  OK")

    print("-" * 60)
    print("All behavior state machine self-tests passed.")


if __name__ == "__main__":
    _selftest()
