r"""HexaPod entry point — wires the whole stack together and runs it.

Pipeline each control tick:

    teleop intent ─┐
                   ▼
    state machine ──▶ command ──▶ velocity commander (ramp) ──▶ control loop
        ▲                                                          │
        │  obstacle flag                                  gait ▶ contact filter
        │                                                  ▶ leveling pose ▶ IK
   obstacle detector ◀── telemetry ◀───────────────────── serial ◀────────┘
        leveling ◀───────┘

Run it:
    python -m pi.main                 # auto-loads ./hexapod.yaml (mock if no port)
    python -m pi.main --config x.yaml
    python -m pi.main --port COM5     # talk to a real Arduino
    python -m pi.main --mock          # force the offline fake Arduino

Teleop keys: w/s forward/back · a/d strafe · q/e turn · space stop
             u stand up · j sit down · o (mock) toggle a fake obstacle · x quit
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass, field
from typing import Optional

from pi.comms.serial_link import FakeArduino, SerialLink, open_mock, open_serial
from pi.comms.servo_map import SERVO_MAX_US, SERVO_MIN_US
from pi.config import LEG_ORDER
from pi.control.avoidance import AvoidanceConfig, AvoidanceController
from pi.control.commander import VelocityCommander, VelocityLimits
from pi.control.contact import ContactAdapter, ContactConfig, contacts_from_telemetry
from pi.control.leveling import BodyLeveler, LevelingConfig
from pi.control.loop import ControlLoop
from pi.control.standup import (
    boot_to_sit_frames,
    play,
    sit_down_frames,
    stance_pulses_at_height,
    stand_up_frames,
)
from pi.control.state_machine import (
    BehaviorInput,
    BehaviorStateMachine,
    State,
)
from pi.gait.engine import GaitCommand, GaitEngine
from pi.gait.trajectory import StepParams
from pi.kinematics.body import BodyPose
from pi.perception.obstacle import ObstacleConfig, ObstacleDetector


# ════════════════════════════════════════════════════════════════════════════
# Settings (defaults + YAML overlay)
# ════════════════════════════════════════════════════════════════════════════
@dataclass
class Settings:
    serial_port: Optional[str] = None     # None / "mock" => offline fake Arduino
    baud: int = 115200
    gait_period_s: float = 1.0
    step_params: StepParams = field(default_factory=StepParams)
    max_step_mm: float = 70.0
    velocity: VelocityLimits = field(default_factory=VelocityLimits)
    leveling: LevelingConfig = field(default_factory=LevelingConfig)
    contact: ContactConfig = field(default_factory=ContactConfig)
    obstacle: ObstacleConfig = field(default_factory=ObstacleConfig)
    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)
    sit_height_mm: float = 35.0
    standup_duration_s: float = 1.5
    mock_distance_mm: int = 1500


def load_settings(path: Optional[str] = None) -> Settings:
    """Load settings from YAML, overlaying any provided keys on the defaults.

    Missing file or missing PyYAML -> defaults (with a note). Only keys present
    in the file override; everything else keeps its default.
    """
    s = Settings()
    if path is None:
        default_path = os.path.join(os.getcwd(), "hexapod.yaml")
        path = default_path if os.path.exists(default_path) else None
    if path is None:
        return s
    try:
        import yaml  # lazy: not needed for the mock/test path
    except ImportError:
        print("[config] PyYAML not installed; using defaults.")
        return s
    if not os.path.exists(path):
        print(f"[config] {path} not found; using defaults.")
        return s

    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    def sub(name: str) -> dict:
        d = data.get(name, {})
        return d if isinstance(d, dict) else {}

    ser, g, v = sub("serial"), sub("gait"), sub("velocity")
    lv, ct, ob = sub("leveling"), sub("contact"), sub("obstacle")
    av, su = sub("avoidance"), sub("standup")

    port = ser.get("port", s.serial_port)
    s.serial_port = None if port in (None, "mock", "null") else str(port)
    s.baud = int(ser.get("baud", s.baud))
    s.gait_period_s = float(g.get("period_s", s.gait_period_s))
    s.step_params = StepParams(
        step_height_mm=float(g.get("step_height_mm", s.step_params.step_height_mm)),
        duty=float(g.get("duty", s.step_params.duty)),
    )
    s.max_step_mm = float(g.get("max_step_mm", s.max_step_mm))
    s.velocity = VelocityLimits(
        max_vx=float(v.get("max_vx", s.velocity.max_vx)),
        max_vy=float(v.get("max_vy", s.velocity.max_vy)),
        max_omega_deg_s=float(v.get("max_omega_deg_s", s.velocity.max_omega_deg_s)),
        accel_lin=float(v.get("accel_lin", s.velocity.accel_lin)),
        accel_omega=float(v.get("accel_omega", s.velocity.accel_omega)),
    )
    s.leveling = LevelingConfig(
        kp=float(lv.get("kp", s.leveling.kp)),
        max_tilt_deg=float(lv.get("max_tilt_deg", s.leveling.max_tilt_deg)),
        sign=int(lv.get("sign", s.leveling.sign)),
        enabled=bool(lv.get("enabled", s.leveling.enabled)),
    )
    s.contact = ContactConfig(
        probe_speed_mm_s=float(ct.get("probe_speed_mm_s", s.contact.probe_speed_mm_s)),
        recover_speed_mm_s=float(ct.get("recover_speed_mm_s", s.contact.recover_speed_mm_s)),
        max_extend_mm=float(ct.get("max_extend_mm", s.contact.max_extend_mm)),
        enabled=bool(ct.get("enabled", s.contact.enabled)),
    )
    s.obstacle = ObstacleConfig(
        threshold_mm=float(ob.get("threshold_mm", s.obstacle.threshold_mm)),
        clear_margin_mm=float(ob.get("clear_margin_mm", s.obstacle.clear_margin_mm)),
        window=int(ob.get("window", s.obstacle.window)),
    )
    s.avoidance = AvoidanceConfig(
        turn_deg_s=float(av.get("turn_deg_s", s.avoidance.turn_deg_s)),
        settle_s=float(av.get("settle_s", s.avoidance.settle_s)),
        reverse_after_s=float(av.get("reverse_after_s", s.avoidance.reverse_after_s)),
        direction=int(av.get("direction", s.avoidance.direction)),
    )
    s.sit_height_mm = float(su.get("sit_height_mm", s.sit_height_mm))
    s.standup_duration_s = float(su.get("duration_s", s.standup_duration_s))
    return s


# ════════════════════════════════════════════════════════════════════════════
# Cross-platform non-blocking keyboard
# ════════════════════════════════════════════════════════════════════════════
class KeyPoller:
    """Context manager that returns single keypresses without blocking/Enter."""

    def __enter__(self) -> "KeyPoller":
        self._win = os.name == "nt"
        if self._win:
            import msvcrt
            self._msvcrt = msvcrt
        else:
            import select
            import sys
            import termios
            import tty
            self._select, self._sys, self._termios = select, sys, termios
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc: object) -> None:
        if not self._win:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)

    def poll(self) -> Optional[str]:
        if self._win:
            if self._msvcrt.kbhit():
                return self._msvcrt.getwch()
            return None
        dr, _, _ = self._select.select([self._sys.stdin], [], [], 0)
        return self._sys.stdin.read(1) if dr else None


# ════════════════════════════════════════════════════════════════════════════
# Application — owns every component and runs the integrated tick
# ════════════════════════════════════════════════════════════════════════════
class Application:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings

        # Serial link: real port or in-memory fake Arduino.
        self.fake: Optional[FakeArduino] = None
        if settings.serial_port is None:
            self.fake = FakeArduino(distance_mm=settings.mock_distance_mm)
            self.link: SerialLink = open_mock(self.fake)
        else:
            self.link = open_serial(settings.serial_port, settings.baud)

        # Motion + behaviour components.
        self.engine = GaitEngine(
            period_s=settings.gait_period_s,
            step_params=settings.step_params,
            max_step_mm=settings.max_step_mm,
        )
        self.loop = ControlLoop(self.link, engine=self.engine)
        self.commander = VelocityCommander(settings.velocity)
        self.sm = BehaviorStateMachine(AvoidanceController(settings.avoidance))
        self.leveler = BodyLeveler(settings.leveling)
        self.contact = ContactAdapter(settings.contact, settings.step_params)
        self.obstacle = ObstacleDetector(settings.obstacle)

        # Fold terrain feedback into the loop's pipeline.
        self.loop.set_target_filter(self._contact_filter)

        # Pre-compute the sit pose held while IDLE.
        self.sit_pulses = stance_pulses_at_height(settings.sit_height_mm)

        # Live state.
        self.latest = None
        self.obstacle_flag = False
        self._fwd = self._strafe = self._turn = 0.0
        self._stand_req = self._sit_req = False
        self._quit = False
        self.sequence_pace = True   # tests set False to skip the 1.5 s sleeps
        self.tick_count = 0
        self._last_log = 0.0

    # ── pipeline pieces ──
    def _contact_filter(self, targets, dt):
        return self.contact.update(
            targets, contacts_from_telemetry(self.latest),
            self.engine.scheduler.phases(), dt,
        )

    def _raw_desired(self) -> GaitCommand:
        v = self.settings.velocity
        return GaitCommand(
            vx=self._fwd * v.max_vx,
            vy=self._strafe * v.max_vy,
            omega_deg_s=self._turn * v.max_omega_deg_s,
        )

    def startup(self) -> None:
        """Glide from the firmware's centred boot pose to the sit pose."""
        play(self.link, boot_to_sit_frames(sit_height_mm=self.settings.sit_height_mm),
             pace=self.sequence_pace)

    def shutdown(self) -> None:
        """Sit down on the way out (if we were standing)."""
        if self.sm.state is not State.IDLE:
            play(self.link, sit_down_frames(sit_height_mm=self.settings.sit_height_mm),
                 pace=self.sequence_pace)

    def step(self, dt: float):
        """One integrated control tick."""
        binp = BehaviorInput(
            desired=self._raw_desired(),
            obstacle=self.obstacle_flag,
            stand_request=self._stand_req,
            sit_request=self._sit_req,
        )
        out = self.sm.update(binp, dt)
        self._stand_req = self._sit_req = False   # one-shot requests

        # Transition motion sequences (blocking on purpose).
        if out.start_standup:
            play(self.link, stand_up_frames(
                sit_height_mm=self.settings.sit_height_mm,
                duration_s=self.settings.standup_duration_s), pace=self.sequence_pace)
            self.commander.stop()                 # start standing from rest
        if out.start_sitdown:
            play(self.link, sit_down_frames(
                sit_height_mm=self.settings.sit_height_mm,
                duration_s=self.settings.standup_duration_s), pace=self.sequence_pace)

        self.tick_count += 1

        # IDLE: hold the sit pose (the gait's neutral is full height, which would
        # otherwise stand the robot back up).
        if self.sm.state is State.IDLE:
            self.link.send_servos(self.sit_pulses)
            self._drain_telemetry()
            return out

        # Active states: ramp the command, apply leveling pose, run the pipeline.
        self.commander.set_target(out.command.vx, out.command.vy, out.command.omega_deg_s)
        self.loop.set_command(self.commander.update(dt))
        self.loop.set_pose(self.leveler.update(self.latest, BodyPose()))
        res = self.loop.tick(dt)

        self.latest = res.telemetry
        self.obstacle_flag = self.obstacle.update(res.telemetry)
        return out

    def _drain_telemetry(self) -> None:
        got = self.link.poll()
        if got:
            self.latest = got[-1]
            self.obstacle_flag = self.obstacle.update(self.latest)

    # ── teleop key handling ──
    def apply_key(self, key: str) -> None:
        k = key.lower()
        if k == "w":
            self._fwd = 1.0
        elif k == "s":
            self._fwd = -1.0
        elif k == "a":
            self._strafe = 1.0
        elif k == "d":
            self._strafe = -1.0
        elif k == "q":
            self._turn = 1.0
        elif k == "e":
            self._turn = -1.0
        elif k == " ":
            self._fwd = self._strafe = self._turn = 0.0
        elif k == "u":
            self._stand_req = True
        elif k == "j":
            self._sit_req = True
        elif k == "o" and self.fake is not None:
            # Toggle a simulated obstacle (mock mode only).
            self.fake.distance_mm = 150 if (self.fake.distance_mm or 0) > 400 else 1500
        elif k in ("x", "\x1b", "\x03"):
            self._quit = True

    # ── logging ──
    def status_line(self) -> str:
        c = self.commander.current
        d = self.latest.distance_mm if self.latest else None
        roll = self.latest.roll_deg if self.latest else 0.0
        pitch = self.latest.pitch_deg if self.latest else 0.0
        contacts = "".join("1" if (self.latest and self.latest.contact(l)) else "0"
                           for l in LEG_ORDER)
        return (f"[{self.sm.state.value:5}] "
                f"v=({c.vx:+5.0f},{c.vy:+5.0f}) w={c.omega_deg_s:+4.0f}  "
                f"dist={'--' if d is None else f'{d:4d}'}mm "
                f"rp=({roll:+4.1f},{pitch:+4.1f}) feet={contacts} "
                f"obs={'Y' if self.obstacle_flag else 'n'}")

    # ── interactive loop ──
    def run(self) -> None:
        print(__doc__.split("Run it:")[0].strip())
        print("\nTeleop: w/s a/d q/e move · space stop · u stand · j sit · "
              + ("o fake-obstacle · " if self.fake else "") + "x quit\n")
        target_dt = self.loop.target_dt
        max_dt = target_dt * 5
        with KeyPoller() as kp:
            self.startup()
            last = time.perf_counter()
            while not self._quit:
                now = time.perf_counter()
                dt = min(now - last, max_dt)
                last = now
                key = kp.poll()
                if key:
                    self.apply_key(key)
                self.step(dt)
                if now - self._last_log >= 0.5:
                    self._last_log = now
                    print("  " + self.status_line(), end="\r", flush=True)
                slack = target_dt - (time.perf_counter() - now)
                if slack > 0:
                    time.sleep(slack)
        print("\nshutting down...")
        self.shutdown()
        self.link.close()


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="HexaPod controller")
    ap.add_argument("--config", help="path to YAML config (default ./hexapod.yaml)")
    ap.add_argument("--port", help="serial port (e.g. COM5, /dev/ttyACM0)")
    ap.add_argument("--mock", action="store_true", help="force the offline fake Arduino")
    args = ap.parse_args(argv)

    settings = load_settings(args.config)
    if args.mock:
        settings.serial_port = None
    elif args.port:
        settings.serial_port = args.port

    where = "mock Arduino" if settings.serial_port is None else settings.serial_port
    print(f"HexaPod starting (link: {where})")
    app = Application(settings)
    try:
        app.run()
    except KeyboardInterrupt:
        print("\ninterrupted")
        app.shutdown()
        app.link.close()
    return 0


# ════════════════════════════════════════════════════════════════════════════
# Headless self-test: `python -m pi.main --selftest`  (no keyboard, runs the
# full integrated stack against the mock Arduino).
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    print("main integration self-test (headless, mock Arduino)")
    print("=" * 64)

    app = Application(load_settings(None))   # defaults, mock link
    app.sequence_pace = False                # don't sleep through sequences
    dt = app.loop.target_dt

    app.startup()
    assert app.sm.state is State.IDLE
    print("startup glides to sit, state IDLE ............... OK")

    def run(n: int) -> None:
        for _ in range(n):
            out = app.step(dt)
            # servos always commanded in the safe range
            last = app.fake.last_pulses
            if last is not None:
                assert all(SERVO_MIN_US <= p <= SERVO_MAX_US for p in last)

    # Stand up.
    app.apply_key("u")
    run(1)
    assert app.sm.state is State.STAND
    print("'u' -> stand up -> STAND ........................ OK")

    # Walk forward.
    app.apply_key("w")
    run(40)
    assert app.sm.state is State.WALK
    assert app.commander.current.vx > 0, "ramped up to forward speed"
    print("'w' -> WALK, command ramps up ................... OK")

    # Simulated obstacle -> detector flags -> AVOID, turning in place.
    app.fake.distance_mm = 150
    run(20)
    assert app.obstacle_flag and app.sm.state is State.AVOID
    assert app.commander.current.vx >= 0  # forward ramped down
    print("obstacle -> AVOID, stop and turn ................ OK")

    # Clear the obstacle -> resume walking.
    app.fake.distance_mm = 1500
    run(40)
    assert app.sm.state is State.WALK
    print("obstacle cleared -> resume WALK ................. OK")

    # Stop, then sit down -> IDLE holds the sit pose.
    app.apply_key(" ")
    run(40)
    assert app.sm.state is State.STAND
    app.apply_key("j")
    run(2)
    assert app.sm.state is State.IDLE
    before = app.fake.command_count
    run(5)
    # IDLE keeps sending the sit pose (not standing back up).
    assert app.fake.last_pulses == app.sit_pulses
    assert app.fake.command_count > before
    print("space->STAND, 'j'->sit->IDLE holds sit pose .... OK")

    # YAML loader overlays values onto defaults.
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), "hexapod_test.yaml")
    with open(tmp, "w") as f:
        f.write("gait:\n  period_s: 0.6\nobstacle:\n  threshold_mm: 300\n")
    s = load_settings(tmp)
    assert s.gait_period_s == 0.6 and s.obstacle.threshold_mm == 300
    assert s.max_step_mm == 70.0  # untouched keys keep defaults
    os.remove(tmp)
    print("YAML overlay overrides only given keys .......... OK")

    print(f"\nfinal status: {app.status_line()}")
    print("-" * 64)
    print("All main integration self-tests passed.")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
    else:
        raise SystemExit(main())
