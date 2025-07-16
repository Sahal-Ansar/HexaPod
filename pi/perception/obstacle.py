r"""Ultrasonic obstacle detection — filter the distance, flag close obstacles.

The raw HC-SR04 distance (from telemetry) is noisy: occasional spurious spikes
and 'no echo' returns. Acting on a single raw reading would make the robot
twitch. So this:

  1. Substitutes a 'far' value for no-echo (no echo = nothing close = open space).
  2. Moving-averages the last N readings to smooth noise and reject lone spikes.
  3. Raises an obstacle flag when the filtered distance drops below a threshold,
     with HYSTERESIS: once flagged, it only clears when the distance rises well
     ABOVE the threshold (threshold + clear_margin). Without hysteresis the flag
     would chatter on/off when an obstacle sits right at the threshold.

Output is a simple boolean the state machine reads to trigger avoidance.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from pi.comms.protocol import Telemetry


@dataclass(frozen=True)
class ObstacleConfig:
    threshold_mm: float = 250.0          # obstacle if filtered distance below this
    clear_margin_mm: float = 80.0        # must rise this far above threshold to clear
    window: int = 5                      # moving-average length
    no_echo_distance_mm: float = 4000.0  # treat 'no echo' as this far (open space)
    enabled: bool = True


class ObstacleDetector:
    """Filtered distance + a debounced (hysteretic) 'obstacle ahead' flag."""

    def __init__(self, config: ObstacleConfig = ObstacleConfig()) -> None:
        self.config = config
        self._buf: deque[float] = deque(maxlen=max(1, config.window))
        self._flag = False
        self._filtered: float | None = None

    @property
    def distance(self) -> float:
        """Current filtered distance (mm). 'Far' until any reading arrives."""
        return self._filtered if self._filtered is not None else self.config.no_echo_distance_mm

    @property
    def detected(self) -> bool:
        return self._flag

    def reset(self) -> None:
        self._buf.clear()
        self._flag = False
        self._filtered = None

    def feed(self, distance_mm: int | None) -> bool:
        """Add one raw distance reading (None = no echo) and return the flag."""
        if not self.config.enabled:
            self._flag = False
            return False

        cfg = self.config
        d = cfg.no_echo_distance_mm if distance_mm is None else float(distance_mm)
        self._buf.append(d)
        self._filtered = sum(self._buf) / len(self._buf)

        # Hysteresis: different set/clear thresholds prevent boundary chatter.
        if self._flag:
            if self._filtered > cfg.threshold_mm + cfg.clear_margin_mm:
                self._flag = False
        else:
            if self._filtered < cfg.threshold_mm:
                self._flag = True
        return self._flag

    def update(self, telemetry: Telemetry | None) -> bool:
        """Feed from a telemetry packet. None (no telemetry) holds the flag."""
        if telemetry is None:
            return self._flag
        return self.feed(telemetry.distance_mm)


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.perception.obstacle`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    print("obstacle detector self-test")
    print("=" * 60)

    cfg = ObstacleConfig(threshold_mm=250.0, clear_margin_mm=80.0, window=5)

    # 1) Steady far readings: never flagged, filtered tracks the distance.
    det = ObstacleDetector(cfg)
    for _ in range(10):
        det.feed(1000)
    assert not det.detected and abs(det.distance - 1000) < 1e-6
    print("steady far -> not detected ...................... OK")

    # 2) Approaching obstacle trips the flag once the average crosses threshold.
    det = ObstacleDetector(cfg)
    for _ in range(5):
        det.feed(240)            # below 250
    assert det.detected
    print("close obstacle -> detected ...................... OK")

    # 3) Hysteresis: just above the threshold does NOT clear; well above does.
    for _ in range(5):
        det.feed(260)            # above 250 but below 250+80=330
    assert det.detected, "must stay flagged in the hysteresis band"
    for _ in range(5):
        det.feed(400)            # above 330 -> clears
    assert not det.detected
    print("hysteresis: holds in band, clears well above .... OK")

    # 4) Lone spike rejection: one bad near reading among far ones doesn't trip.
    det = ObstacleDetector(cfg)
    for _ in range(4):
        det.feed(1000)
    det.feed(0)                  # single spurious 'something touching' spike
    assert not det.detected, "averaging must reject a lone spike"
    print("single spurious near reading rejected ........... OK")

    # 5) No-echo is treated as open space (far), never an obstacle.
    det = ObstacleDetector(cfg)
    for _ in range(5):
        det.feed(None)
    assert not det.detected
    assert abs(det.distance - cfg.no_echo_distance_mm) < 1e-6
    print("no-echo -> open space, not detected ............. OK")

    # 6) Telemetry wrapper + dropout holds the flag.
    det = ObstacleDetector(cfg)
    t_near = Telemetry(distance_mm=200, roll_deg=0, pitch_deg=0, contacts=(False,) * 6)
    for _ in range(5):
        det.update(t_near)
    assert det.detected
    assert det.update(None) is True, "no telemetry holds last flag"
    print("telemetry wrapper + dropout holds flag .......... OK")

    # 7) Disabled never flags.
    det = ObstacleDetector(ObstacleConfig(enabled=False, threshold_mm=250))
    for _ in range(5):
        det.feed(50)
    assert not det.detected
    print("disabled detector never flags ................... OK")

    # 8) A realistic approach sweep: flag rises once and stays (no chatter).
    det = ObstacleDetector(cfg)
    transitions = 0
    prev = False
    for d in [1000, 900, 700, 500, 400, 320, 260, 230, 210, 200, 195, 190]:
        det.feed(d)
        if det.detected != prev:
            transitions += 1
            prev = det.detected
    assert transitions == 1 and det.detected, "should latch once, no flicker"
    print("monotone approach latches once, no flicker ...... OK")

    print("-" * 60)
    print("All obstacle detector self-tests passed.")


if __name__ == "__main__":
    _selftest()
