r"""Main control loop — the fixed-rate tick that actually walks the robot.

Each tick runs the full Pi-side pipeline:

    gait engine  ->  foot targets (body frame)
       |
    body kinematics + IK (solve_body)  ->  18 joint angles
       |
    servo map (calibration)  ->  18 microsecond pulses
       |
    serial link  ->  Arduino   (and poll telemetry back)

This stage is OPEN-LOOP: telemetry is read and stored but not yet used to change
behaviour (leveling/contact/avoidance come in later stages). Because the serial
link has a mock mode, the whole loop runs on a laptop with no hardware — that's
how the self-test below "walks" the robot in simulation.

Timing: the loop measures the ACTUAL elapsed time each tick (perf_counter) and
feeds that real dt to the gait engine, so the gait phase stays correct even if
the OS scheduler jitters the loop period. dt is clamped so a one-off stall (GC,
scheduling hiccup) can't jump the gait a huge amount.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from pi.comms.serial_link import SerialLink, open_mock
from pi.comms.servo_map import DEFAULT_CALIBRATION, CalibrationTable, angles_to_pulses
from pi.config import ROBOT, RobotConfig
from pi.comms.protocol import Telemetry
from pi.gait.engine import GaitCommand, GaitEngine
from pi.kinematics.body import BodyPose, solve_body


@dataclass
class TickResult:
    """What one control tick produced — handy for logging and tests."""

    pulses: list[int]
    telemetry: Telemetry | None


class ControlLoop:
    """Drives one robot. Feed it a SerialLink (real or mock)."""

    def __init__(
        self,
        link: SerialLink,
        config: RobotConfig = ROBOT,
        engine: GaitEngine | None = None,
        calibration: CalibrationTable = DEFAULT_CALIBRATION,
        pose: BodyPose = BodyPose(),
    ) -> None:
        self.link = link
        self.config = config
        self.engine = engine or GaitEngine(config=config)
        self.calibration = calibration
        self.pose = pose                      # body height/leveling (identity for now)
        self.command = GaitCommand()          # current velocity command
        self.tick_count = 0
        self.target_dt = 1.0 / config.control_hz
        self._running = False

    # ── command / pose setters (teleop, leveling, state machine use these) ──
    def set_command(self, command: GaitCommand) -> None:
        self.command = command

    def set_pose(self, pose: BodyPose) -> None:
        self.pose = pose

    # ── one control step ──
    def tick(self, dt_s: float) -> TickResult:
        """Run the full pipeline once for an elapsed time of ``dt_s`` seconds."""
        targets = self.engine.update(self.command, dt_s)          # gait
        angles = solve_body(targets, self.pose, self.config)      # body IK
        pulses = angles_to_pulses(angles, self.calibration)       # calibration
        self.link.send_servos(pulses)                             # -> Arduino
        received = self.link.poll()                               # telemetry <-
        self.tick_count += 1
        telem = received[-1] if received else self.link.latest()
        return TickResult(pulses=pulses, telemetry=telem)

    # ── fixed-rate runner ──
    def run(self, duration_s: float | None = None) -> None:
        """Run the loop at the configured rate until ``duration_s`` elapses (or
        forever if None / until stop())."""
        self._running = True
        max_dt = self.target_dt * 5.0   # clamp: never advance the gait more than 5 ticks
        start = time.perf_counter()
        last = start
        while self._running:
            now = time.perf_counter()
            dt = now - last
            last = now
            if dt > max_dt:
                dt = max_dt
            self.tick(dt)

            if duration_s is not None and (time.perf_counter() - start) >= duration_s:
                break
            # Sleep the remainder of the tick to hold the target rate.
            slack = self.target_dt - (time.perf_counter() - now)
            if slack > 0:
                time.sleep(slack)
        self._running = False

    def stop(self) -> None:
        self._running = False


def build_mock_loop(**engine_kwargs: object) -> ControlLoop:
    """A ControlLoop wired to an in-memory fake Arduino — for laptop demos/tests."""
    from pi.comms.serial_link import FakeArduino
    link = open_mock(FakeArduino(distance_mm=1500))
    engine = GaitEngine(**engine_kwargs) if engine_kwargs else None  # type: ignore[arg-type]
    return ControlLoop(link, engine=engine)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.loop`  (walks the robot against a fake Arduino)
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    from pi.comms.serial_link import FakeArduino, open_mock
    from pi.comms.servo_map import SERVO_MAX_US, SERVO_MIN_US

    print("control loop self-test (open-loop, mock Arduino)")
    print("=" * 64)

    fake = FakeArduino(distance_mm=1234, roll_deg=1.5,
                       contacts=[True, False, True, False, True, False])
    loop = ControlLoop(open_mock(fake))
    dt = loop.target_dt

    # Walk forward for one full gait cycle worth of ticks.
    loop.set_command(GaitCommand(vx=60.0))
    seen = []
    for _ in range(50):  # 1 s at 50 Hz
        res = loop.tick(dt)
        assert len(res.pulses) == 18
        assert all(SERVO_MIN_US <= p <= SERVO_MAX_US for p in res.pulses)
        seen.append(res.pulses)
    assert loop.tick_count == 50
    print("50 ticks: 18 in-range pulses each, no errors ......... OK")

    # Telemetry from the fake Arduino comes back through the loop.
    res = loop.tick(dt)
    assert res.telemetry is not None
    assert res.telemetry.distance_mm == 1234
    assert res.telemetry.contacts[0] and res.telemetry.contacts[2]
    print(f"telemetry round-trips (dist={res.telemetry.distance_mm}mm, "
          f"roll={res.telemetry.roll_deg}) ........ OK")

    # The legs actually move: pulses differ across the cycle.
    assert seen[5] != seen[25], "servo pulses should change while walking"
    print("servo pulses change over the gait cycle .............. OK")

    # Fake Arduino really received every command.
    assert fake.command_count >= 50
    print(f"fake Arduino received {fake.command_count} servo commands ......... OK")

    # Stop: pulses settle to the neutral stance (and stay put).
    loop.set_command(GaitCommand())
    a = loop.tick(dt).pulses
    b = loop.tick(dt).pulses
    assert a == b, "stopped robot must hold a steady pose"
    print("stop command -> steady neutral pose .................. OK")

    # Timed run() at the real rate for a short spell (smoke test the runner).
    loop2 = build_mock_loop()
    loop2.set_command(GaitCommand(vx=40.0, omega_deg_s=15.0))
    t0 = time.perf_counter()
    loop2.run(duration_s=0.2)
    elapsed = time.perf_counter() - t0
    assert loop2.tick_count > 0
    assert 0.15 <= elapsed <= 0.6, f"run() timing off: {elapsed:.3f}s"
    print(f"run() for 0.2s did {loop2.tick_count} ticks in {elapsed:.3f}s ...... OK")

    print("-" * 64)
    print("All control loop self-tests passed.")


if __name__ == "__main__":
    _selftest()
