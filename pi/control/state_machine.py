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

    def __init__(self, avoid_turn_deg_s: float = 40.0) -> None:
        self.state = State.IDLE
        self.avoid_turn_deg_s = avoid_turn_deg_s

    def reset(self) -> None:
        self.state = State.IDLE

    def _avoid_command(self, inp: BehaviorInput) -> GaitCommand:
        """Stop forward/strafe and turn in place. (Direction selection and the
        resume logic are refined in the avoidance stage.)"""
        return GaitCommand(vx=0.0, vy=0.0, omega_deg_s=self.avoid_turn_deg_s)

    def update(self, inp: BehaviorInput) -> BehaviorOutput:
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
                self.state = State.AVOID
                return BehaviorOutput(State.AVOID, self._avoid_command(inp))
            if inp.desired.is_zero():
                self.state = State.STAND
                return BehaviorOutput(State.STAND, GaitCommand())
            return BehaviorOutput(State.WALK, inp.desired)

        # AVOID
        if not inp.obstacle:
            # Cleared: resume walking if the user still wants to move, else stand.
            if inp.desired.is_zero():
                self.state = State.STAND
                return BehaviorOutput(State.STAND, GaitCommand())
            self.state = State.WALK
            return BehaviorOutput(State.WALK, inp.desired)
        return BehaviorOutput(State.AVOID, self._avoid_command(inp))


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.state_machine`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    print("behavior state machine self-test")
    print("=" * 60)

    fwd = GaitCommand(vx=60.0)
    stop = GaitCommand()

    # IDLE ignores movement/obstacle until told to stand.
    sm = BehaviorStateMachine()
    out = sm.update(BehaviorInput(desired=fwd, obstacle=True))
    assert sm.state is State.IDLE and out.command.is_zero()
    print("IDLE ignores move/obstacle ...................... OK")

    # IDLE -> STAND on stand_request, signalling stand-up.
    out = sm.update(BehaviorInput(stand_request=True))
    assert sm.state is State.STAND and out.start_standup
    print("IDLE -> STAND triggers stand-up ................. OK")

    # STAND -> WALK on a move command; command passes through.
    out = sm.update(BehaviorInput(desired=fwd))
    assert sm.state is State.WALK and out.command == fwd
    print("STAND -> WALK on move command .................. OK")

    # WALK -> AVOID on obstacle; command becomes stop + turn.
    out = sm.update(BehaviorInput(desired=fwd, obstacle=True))
    assert sm.state is State.AVOID
    assert out.command.vx == 0.0 and out.command.omega_deg_s != 0.0
    print("WALK -> AVOID: stop and turn in place .......... OK")

    # AVOID holds while the obstacle is present.
    out = sm.update(BehaviorInput(desired=fwd, obstacle=True))
    assert sm.state is State.AVOID and out.command.vx == 0.0
    print("AVOID persists while obstacle present .......... OK")

    # AVOID -> WALK when cleared and the user still wants to move.
    out = sm.update(BehaviorInput(desired=fwd, obstacle=False))
    assert sm.state is State.WALK and out.command == fwd
    print("AVOID -> WALK on clear (resume) ................ OK")

    # WALK -> STAND when the user stops.
    out = sm.update(BehaviorInput(desired=stop))
    assert sm.state is State.STAND and out.command.is_zero()
    print("WALK -> STAND on stop .......................... OK")

    # AVOID -> STAND when cleared but the user is no longer moving.
    sm.state = State.AVOID
    out = sm.update(BehaviorInput(desired=stop, obstacle=False))
    assert sm.state is State.STAND
    print("AVOID -> STAND on clear w/o move intent ........ OK")

    # Sit-down from any active state -> IDLE, signalling sit-down.
    for start in (State.STAND, State.WALK, State.AVOID):
        sm.state = start
        out = sm.update(BehaviorInput(sit_request=True))
        assert sm.state is State.IDLE and out.start_sitdown
    print("sit_request from STAND/WALK/AVOID -> IDLE ...... OK")

    # End-to-end scenario walk-through.
    sm = BehaviorStateMachine()
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
        sm.update(inp)
        assert sm.state is expected, f"expected {expected}, got {sm.state}"
    print("full IDLE->STAND->WALK->AVOID->WALK->STAND->IDLE  OK")

    print("-" * 60)
    print("All behavior state machine self-tests passed.")


if __name__ == "__main__":
    _selftest()
