"""Serial link to the Arduino — send servo packets, receive telemetry.

The link is split into two layers so the same framing/parsing code runs on real
hardware and in offline tests:

  * SerialLink     — protocol layer: encodes servo commands, parses incoming
                     telemetry. Knows nothing about *how* bytes move.
  * Transport      — byte layer: just write(bytes) / read_available() / close().
                     SerialTransport wraps pyserial; LoopbackTransport fakes an
                     Arduino entirely in memory for laptop testing.

Because the loopback re-encodes telemetry to bytes and SerialLink re-decodes it,
the mock path exercises the *real* CRC/framing in both directions — a bug in the
wire format fails the offline test, not just on the bench.

pyserial is imported lazily (only when a real port is opened) so this module and
the whole test suite import fine on a machine with no serial hardware.
"""

from __future__ import annotations

from typing import Callable, Optional, Protocol, Sequence, runtime_checkable

from pi.comms.protocol import (
    MSG_SERVO,
    MSG_TELEMETRY,
    NUM_LEGS,
    FrameParser,
    Telemetry,
    decode_servo_command,
    decode_telemetry,
    encode_servo_command,
    encode_telemetry,
)

# Default baud. The traffic is light (~3 kB/s at 50 Hz), so 115200 is ample and
# rock-solid over USB; bump higher (e.g. 500000) on the Mega if you want margin.
DEFAULT_BAUD = 115200


# ════════════════════════════════════════════════════════════════════════════
# Transport interface
# ════════════════════════════════════════════════════════════════════════════
@runtime_checkable
class Transport(Protocol):
    """A non-blocking byte pipe to the Arduino."""

    def write(self, data: bytes) -> None: ...
    def read_available(self) -> bytes: ...
    def close(self) -> None: ...


class SerialTransport:
    """Real USB-serial transport backed by pyserial (imported lazily)."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = 0.0) -> None:
        import serial  # lazy: only needed for a real port
        # timeout=0 → fully non-blocking reads; we poll in_waiting ourselves.
        self._ser = serial.Serial(port, baudrate=baud, timeout=timeout)

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def read_available(self) -> bytes:
        n = self._ser.in_waiting
        return self._ser.read(n) if n else b""

    def close(self) -> None:
        self._ser.close()


# ════════════════════════════════════════════════════════════════════════════
# In-memory Arduino simulation for offline testing
# ════════════════════════════════════════════════════════════════════════════
ArduinoFn = Callable[[list[int]], Optional[Telemetry]]


class FakeArduino:
    """A stand-in Arduino: every servo command it receives triggers one
    telemetry reply built from its (mutable) sensor attributes.

    Set the attributes between ticks to script scenarios — e.g. drop
    ``distance_mm`` to test obstacle avoidance, or tilt ``roll_deg`` to test
    leveling — without any hardware.
    """

    def __init__(
        self,
        distance_mm: int | None = 1000,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        contacts: Sequence[bool] = (False,) * NUM_LEGS,
    ) -> None:
        self.distance_mm = distance_mm
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.contacts = tuple(contacts)
        self.command_count = 0
        self.last_pulses: list[int] | None = None

    def __call__(self, pulses: list[int]) -> Telemetry:
        self.command_count += 1
        self.last_pulses = list(pulses)
        return Telemetry(
            distance_mm=self.distance_mm,
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            contacts=self.contacts,  # type: ignore[arg-type]
        )


class LoopbackTransport:
    """Fake transport that runs a FakeArduino (or any ArduinoFn) in memory.

    On write(), it parses the servo command(s) the Pi sent and asks the Arduino
    callback for a telemetry reply, which it encodes to bytes and queues for the
    Pi to read — mimicking the request/reply cadence closely enough for tests.
    (The real firmware streams telemetry on its own clock; see module note.)
    """

    def __init__(self, arduino: Optional[ArduinoFn] = None) -> None:
        self._arduino: ArduinoFn = arduino or FakeArduino()
        self._cmd_parser = FrameParser()
        self._inbox = bytearray()

    def write(self, data: bytes) -> None:
        for msg_type, payload in self._cmd_parser.feed(data):
            if msg_type != MSG_SERVO:
                continue
            pulses = decode_servo_command(payload)
            reply = self._arduino(pulses)
            if reply is not None:
                self._inbox += encode_telemetry(
                    reply.distance_mm, reply.roll_deg, reply.pitch_deg, reply.contacts
                )

    def read_available(self) -> bytes:
        data = bytes(self._inbox)
        self._inbox.clear()
        return data

    def close(self) -> None:
        pass


# ════════════════════════════════════════════════════════════════════════════
# Protocol-layer link
# ════════════════════════════════════════════════════════════════════════════
class SerialLink:
    """Send servo pulses, receive Telemetry. Transport-agnostic."""

    def __init__(self, transport: Transport) -> None:
        self._t = transport
        self._parser = FrameParser()
        self._latest: Telemetry | None = None

    def send_servos(self, pulses_us: Sequence[int]) -> None:
        """Encode and transmit the 18 servo pulse targets."""
        self._t.write(encode_servo_command(pulses_us))

    def poll(self) -> list[Telemetry]:
        """Read whatever bytes are available, parse, and return any complete
        telemetry packets (non-blocking). Updates ``latest()``."""
        out: list[Telemetry] = []
        for msg_type, payload in self._parser.feed(self._t.read_available()):
            if msg_type == MSG_TELEMETRY:
                self._latest = decode_telemetry(payload)
                out.append(self._latest)
        return out

    def latest(self) -> Telemetry | None:
        """Most recently received telemetry, or None if nothing yet."""
        return self._latest

    def close(self) -> None:
        self._t.close()

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


# ── Factories ───────────────────────────────────────────────────────────────
def open_serial(port: str, baud: int = DEFAULT_BAUD, timeout: float = 0.0) -> SerialLink:
    """Open a real serial link to the Arduino on ``port`` (e.g. 'COM5',
    '/dev/ttyACM0'). Requires pyserial."""
    return SerialLink(SerialTransport(port, baud, timeout))


def open_mock(arduino: Optional[ArduinoFn] = None) -> SerialLink:
    """Open an in-memory link to a fake Arduino — no hardware needed."""
    return SerialLink(LoopbackTransport(arduino))


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.comms.serial_link`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    from pi.comms.servo_map import angles_to_pulses
    from pi.config import LegId
    from pi.kinematics.body import BodyPose, default_stance_body, solve_body

    print("serial_link self-test")
    print("=" * 56)

    fake = FakeArduino(distance_mm=500, roll_deg=2.5, pitch_deg=-1.0,
                       contacts=[True, False, False, True, False, True])
    link = open_mock(fake)

    # Send a real (kinematically derived) servo command, read the reply.
    pulses = angles_to_pulses(solve_body(default_stance_body(), BodyPose()))
    link.send_servos(pulses)
    telem = link.poll()
    assert len(telem) == 1
    t = telem[0]
    assert t.distance_mm == 500
    assert abs(t.roll_deg - 2.5) < 1e-9 and abs(t.pitch_deg + 1.0) < 1e-9
    assert t.contact(LegId.L1) and t.contact(LegId.R3) and not t.contact(LegId.L2)
    assert link.latest() is t
    assert fake.command_count == 1 and fake.last_pulses == pulses
    print("send -> fake Arduino -> telemetry round-trip ... OK")

    # Telemetry tracks mutable sensor state across ticks.
    fake.distance_mm = 120
    link.send_servos(pulses)
    assert link.poll()[0].distance_mm == 120
    print("scripted sensor change between ticks .......... OK")

    # No-echo distance survives the full encode/decode round-trip.
    fake.distance_mm = None
    link.send_servos(pulses)
    assert link.poll()[0].distance_mm is None
    print("no-echo distance sentinel preserved ........... OK")

    # Context-manager use closes cleanly.
    with open_mock(FakeArduino()) as l2:
        l2.send_servos(pulses)
        assert len(l2.poll()) == 1
    print("context manager open/close .................... OK")

    # Transport conforms to the protocol.
    assert isinstance(LoopbackTransport(), Transport)
    print("LoopbackTransport satisfies Transport ......... OK")

    print("-" * 56)
    print("All serial_link self-tests passed.")


if __name__ == "__main__":
    _selftest()
