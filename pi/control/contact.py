r"""Foot-contact feedback — refine stance using the limit switches.

The gait commands a swing->stance transition at a fixed phase, assuming flat
ground. Real terrain isn't flat: a foot may touch down EARLY (ground is higher
than expected) or LATE / not at all (a dip or hole). The per-leg limit switches
report the truth, and this module adapts each leg's vertical reach so it keeps
finding the ground — distributed terrain adaptation.

Per leg we keep a vertical offset added to the commanded foot target:

  * STANCE but NO contact  -> the foot is hanging over a dip. Reach DOWN a little
    each tick (offset goes negative) until the switch closes — "probe for ground".
  * STANCE with contact     -> ground found; HOLD the offset (stop probing).
  * SWING                   -> relax the offset back toward 0 so the next stance
    starts fresh. Contact during swing is flagged as an EARLY touchdown (a high
    spot) so higher layers/telemetry can see it.

The offset is bounded (max_extend) so a permanently-missing switch can't drive a
leg to full extension. On flat ground with correct contact it's a no-op — it
only ever moves a foot that disagrees with the gait's flat-ground assumption.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from pi.config import LEG_ORDER, LegId
from pi.gait.trajectory import StepParams, is_stance
from pi.math_utils import Vec3


@dataclass(frozen=True)
class ContactConfig:
    probe_speed_mm_s: float = 60.0     # downward search speed when no contact in stance
    recover_speed_mm_s: float = 120.0  # how fast the offset relaxes during swing
    max_extend_mm: float = 30.0        # most a leg will reach below nominal
    enabled: bool = True


# Per-leg event labels (handy for logging / telemetry / the state machine).
EV_PLANTED = "planted"          # stance, in contact (nominal)
EV_PROBING = "probing"          # stance, no contact -> reaching down
EV_EARLY = "early_touchdown"    # swing, but already in contact (high ground)
EV_SWING = "swing"              # swing, no contact (nominal)


class ContactAdapter:
    """Holds a per-leg vertical offset that tracks the real ground."""

    def __init__(self, config: ContactConfig = ContactConfig(),
                 step_params: StepParams = StepParams()) -> None:
        self.config = config
        self.step_params = step_params
        self._offset: dict[LegId, float] = {leg: 0.0 for leg in LEG_ORDER}
        self.events: dict[LegId, str] = {leg: EV_SWING for leg in LEG_ORDER}

    @property
    def offsets(self) -> dict[LegId, float]:
        return dict(self._offset)

    def reset(self) -> None:
        for leg in LEG_ORDER:
            self._offset[leg] = 0.0
            self.events[leg] = EV_SWING

    def _relax(self, value: float, rate: float) -> float:
        """Move ``value`` toward 0 by at most ``rate``."""
        if value > 0.0:
            return max(0.0, value - rate)
        return min(0.0, value + rate)

    def update(
        self,
        targets: Mapping[LegId, Vec3],
        contacts: Mapping[LegId, bool] | None,
        phases: Mapping[LegId, float],
        dt_s: float,
    ) -> dict[LegId, Vec3]:
        """Return foot targets adjusted by the per-leg ground offset.

        ``contacts`` None (no telemetry) relaxes all offsets toward 0.
        """
        if not self.config.enabled:
            return dict(targets)

        cfg = self.config
        probe = cfg.probe_speed_mm_s * dt_s
        recover = cfg.recover_speed_mm_s * dt_s

        adjusted: dict[LegId, Vec3] = {}
        for leg in LEG_ORDER:
            stance = is_stance(phases[leg], self.step_params)
            contact = bool(contacts[leg]) if contacts is not None else None

            if contact is None:
                # No sensor info: ease back to nominal.
                self._offset[leg] = self._relax(self._offset[leg], recover)
                self.events[leg] = EV_SWING if not stance else EV_PLANTED
            elif stance:
                if contact:
                    self.events[leg] = EV_PLANTED            # ground found: hold
                else:
                    # Reach down to find the ground (offset more negative).
                    self._offset[leg] = max(-cfg.max_extend_mm, self._offset[leg] - probe)
                    self.events[leg] = EV_PROBING
            else:  # swing
                self._offset[leg] = self._relax(self._offset[leg], recover)
                self.events[leg] = EV_EARLY if contact else EV_SWING

            base = targets[leg]
            adjusted[leg] = Vec3(base.x, base.y, base.z + self._offset[leg])
        return adjusted


def contacts_from_telemetry(telemetry) -> dict[LegId, bool] | None:
    """Convenience: Telemetry -> {leg: contact} (or None if no telemetry)."""
    if telemetry is None:
        return None
    return {leg: telemetry.contact(leg) for leg in LEG_ORDER}


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.control.contact`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    import math

    from pi.kinematics.body import default_stance_body

    print("contact adapter self-test")
    print("=" * 60)

    params = StepParams(duty=0.5)
    stance = default_stance_body()
    dt = 0.02

    def all_phase(p: float) -> dict[LegId, float]:
        return {leg: p for leg in LEG_ORDER}

    # 1) Flat ground (stance legs in contact, swing legs not) -> no adaptation.
    ad = ContactAdapter(step_params=params)
    phases = {LegId.L1: 0.25, LegId.R1: 0.75, LegId.L2: 0.75, LegId.R2: 0.25,
              LegId.L3: 0.25, LegId.R3: 0.75}
    contacts = {leg: is_stance(phases[leg], params) for leg in LEG_ORDER}  # perfect
    for _ in range(50):
        out = ad.update(stance, contacts, phases, dt)
    for leg in LEG_ORDER:
        assert math.isclose(ad.offsets[leg], 0.0, abs_tol=1e-9)
        assert out[leg].distance_to(stance[leg]) < 1e-9
    print("flat ground (correct contact) -> no-op .......... OK")

    # 2) Hole under a stance leg: no contact while planted -> probe down to limit.
    ad = ContactAdapter(ContactConfig(max_extend_mm=30.0), step_params=params)
    phases = all_phase(0.25)                      # all legs nominally in stance
    contacts = {leg: True for leg in LEG_ORDER}
    contacts[LegId.L2] = False                    # L2 over a dip
    for _ in range(100):
        out = ad.update(stance, contacts, phases, dt)
    assert math.isclose(ad.offsets[LegId.L2], -30.0, abs_tol=1e-6), "should reach to limit"
    assert ad.events[LegId.L2] == EV_PROBING
    assert math.isclose(out[LegId.L2].z, stance[LegId.L2].z - 30.0, abs_tol=1e-6)
    # other legs untouched
    assert math.isclose(ad.offsets[LegId.L1], 0.0)
    print("dip under a planted foot -> probes down to limit  OK")

    # 3) Ground found partway: probing stops and holds.
    ad = ContactAdapter(ContactConfig(probe_speed_mm_s=60.0), step_params=params)
    phases = all_phase(0.25)
    contacts = {leg: True for leg in LEG_ORDER}
    contacts[LegId.L2] = False
    for _ in range(10):                            # probe a bit
        ad.update(stance, contacts, phases, dt)
    depth = ad.offsets[LegId.L2]
    assert depth < 0.0
    contacts[LegId.L2] = True                       # foot reaches the ground
    for _ in range(10):
        ad.update(stance, contacts, phases, dt)
    assert math.isclose(ad.offsets[LegId.L2], depth), "offset holds once planted"
    assert ad.events[LegId.L2] == EV_PLANTED
    print("ground found -> stop probing, hold depth ........ OK")

    # 4) Early touchdown: contact during swing is detected and offset relaxes.
    ad = ContactAdapter(step_params=params)
    phases = all_phase(0.75)                        # all legs in swing
    ad._offset[LegId.R1] = -10.0                    # pretend a leftover offset
    contacts = {leg: False for leg in LEG_ORDER}
    contacts[LegId.R1] = True                       # hits a high spot mid-swing
    ad.update(stance, contacts, phases, dt)
    assert ad.events[LegId.R1] == EV_EARLY
    assert ad.offsets[LegId.R1] > -10.0             # relaxing toward 0
    print("early touchdown flagged, offset relaxes ......... OK")

    # 5) Disabled / no telemetry behave safely.
    ad = ContactAdapter(ContactConfig(enabled=False), step_params=params)
    out = ad.update(stance, {leg: False for leg in LEG_ORDER}, all_phase(0.25), dt)
    assert all(out[leg].distance_to(stance[leg]) < 1e-9 for leg in LEG_ORDER)
    ad = ContactAdapter(step_params=params)
    ad._offset[LegId.L1] = -20.0
    ad.update(stance, None, all_phase(0.25), dt)    # telemetry dropped
    assert ad.offsets[LegId.L1] > -20.0             # relaxes toward 0
    print("disabled = no-op; missing telemetry relaxes ..... OK")

    # 6) Adjusted targets still solve through IK within servo range.
    from pi.comms.servo_map import SERVO_MAX_US, SERVO_MIN_US, angles_to_pulses
    from pi.kinematics.body import BodyPose, solve_body
    ad = ContactAdapter(step_params=params)
    phases = all_phase(0.25)
    contacts = {leg: True for leg in LEG_ORDER}
    contacts[LegId.L3] = False
    for _ in range(40):
        out = ad.update(stance, contacts, phases, dt)
        pulses = angles_to_pulses(solve_body(out, BodyPose()))
        assert all(SERVO_MIN_US <= p <= SERVO_MAX_US for p in pulses)
    print("adapted targets solve through IK in range ....... OK")

    print("-" * 60)
    print("All contact adapter self-tests passed.")


if __name__ == "__main__":
    _selftest()
