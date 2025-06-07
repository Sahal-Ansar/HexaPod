r"""Pi ↔ Arduino serial protocol: framing, checksum, and (de)serialisation.

This is the *wire contract* between the Raspberry Pi (brain) and the Arduino
Mega (motor controller). The Arduino firmware must implement byte-for-byte the
same frame layout and CRC, so this module is the single, documented source of
truth — the C code mirrors it.

────────────────────────────────────────────────────────────────────────────
FRAME LAYOUT (both directions)
────────────────────────────────────────────────────────────────────────────
    ┌──────┬──────┬──────┬─────┬───────────────┬──────┐
    │ 0xAA │ 0x55 │ TYPE │ LEN │  PAYLOAD ...   │ CRC8 │
    └──────┴──────┴──────┴─────┴───────────────┴──────┘
       sync bytes    1B    1B     LEN bytes        1B

  * 0xAA 0x55  : two sync bytes so the receiver can lock onto frame starts and
                 resynchronise after a glitch (one sync byte gives too many
                 false positives in binary data).
  * TYPE       : message type (0x01 servo command, 0x02 telemetry).
  * LEN        : number of PAYLOAD bytes (lets the parser know the frame size
                 and makes the protocol extensible).
  * PAYLOAD    : message-specific, little-endian (Arduino AVR is little-endian,
                 so no byte-swapping on either side).
  * CRC8       : CRC-8 (poly 0x07, init 0x00) over TYPE, LEN, and PAYLOAD — NOT
                 the sync bytes. Catches the bit/byte errors a plain XOR/sum
                 checksum would miss, while staying a tiny table-free loop the
                 Arduino can run cheaply.

────────────────────────────────────────────────────────────────────────────
MESSAGES
────────────────────────────────────────────────────────────────────────────
SERVO COMMAND  (Pi → Arduino, TYPE=0x01, LEN=36)
    18 × uint16 LE = servo pulse widths in MICROSECONDS, one per servo, in
    global servo-index order (index = leg*3 + joint; see servo_index()).
    The Pi has already applied all per-servo calibration, so the Arduino just
    writes each value to its PWM channel:  board = index // 9, channel = index % 9.
    Microseconds (≈500–2500) keep the protocol hardware-agnostic and readable.

TELEMETRY      (Arduino → Pi, TYPE=0x02, LEN=7)
    uint16 LE  distance_mm   front HC-SR04 distance, mm; 0xFFFF = no echo.
    int16  LE  roll_cdeg     body roll  in centidegrees (deg × 100).
    int16  LE  pitch_cdeg    body pitch in centidegrees.
    uint8      contacts      bit i (0..5) = leg i foot limit switch closed.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterable, Sequence

from pi.config import Joint, LegId

# ── Frame constants ─────────────────────────────────────────────────────────
SYNC0 = 0xAA
SYNC1 = 0x55

MSG_SERVO = 0x01      # Pi → Arduino
MSG_TELEMETRY = 0x02  # Arduino → Pi

# ── Servo addressing (shared with the firmware) ─────────────────────────────
NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG  # 18
SERVOS_PER_BOARD = 9                    # 9 of the PCA9685's 16 channels used
NUM_BOARDS = 2

SERVO_PAYLOAD_LEN = NUM_SERVOS * 2  # 18 × uint16 = 36
TELEMETRY_PAYLOAD_LEN = 7

# Sentinel distance meaning "no echo received / out of range".
DISTANCE_NO_ECHO = 0xFFFF


def servo_index(leg: LegId, joint: Joint) -> int:
    """Global 0..17 index of a servo in the wire packet (leg-major order)."""
    return int(leg) * JOINTS_PER_LEG + int(joint)


def board_of(index: int) -> int:
    """Which PCA9685 board drives the servo at this global index (0 or 1)."""
    return index // SERVOS_PER_BOARD


def channel_of(index: int) -> int:
    """Which channel (0..8) on that board drives the servo at this index."""
    return index % SERVOS_PER_BOARD


# ════════════════════════════════════════════════════════════════════════════
# CRC-8 (poly 0x07, init 0x00) — identical algorithm runs on the Arduino.
# ════════════════════════════════════════════════════════════════════════════
def crc8(data: Iterable[int]) -> int:
    crc = 0
    for byte in data:
        crc ^= byte & 0xFF
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _build_frame(msg_type: int, payload: bytes) -> bytes:
    """Wrap a payload in sync bytes, type, length, and CRC."""
    if len(payload) > 255:
        raise ValueError("payload too long for a 1-byte length field")
    head = bytes([msg_type, len(payload)]) + payload  # CRC covers this slice
    return bytes([SYNC0, SYNC1]) + head + bytes([crc8(head)])


# ════════════════════════════════════════════════════════════════════════════
# Servo command (Pi → Arduino)
# ════════════════════════════════════════════════════════════════════════════
def encode_servo_command(pulses_us: Sequence[int]) -> bytes:
    """Build a servo-command frame from 18 microsecond pulse widths.

    Calibration/clamping is the servo-map's job; this is a pure serialiser and
    only enforces the uint16 wire range so a bad value fails loudly here rather
    than silently wrapping on the AVR.
    """
    if len(pulses_us) != NUM_SERVOS:
        raise ValueError(f"expected {NUM_SERVOS} pulses, got {len(pulses_us)}")
    out = []
    for i, p in enumerate(pulses_us):
        v = int(round(p))
        if not 0 <= v <= 0xFFFF:
            raise ValueError(f"servo {i} pulse {p}us outside uint16 range")
        out.append(v)
    return _build_frame(MSG_SERVO, struct.pack("<18H", *out))


def decode_servo_command(payload: bytes) -> list[int]:
    """Decode a servo-command payload back into 18 microsecond values. Mainly
    used by the loopback mock and by tests (the firmware does this in C)."""
    if len(payload) != SERVO_PAYLOAD_LEN:
        raise ValueError(f"servo payload must be {SERVO_PAYLOAD_LEN} bytes")
    return list(struct.unpack("<18H", payload))


# ════════════════════════════════════════════════════════════════════════════
# Telemetry (Arduino → Pi)
# ════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class Telemetry:
    """One decoded telemetry packet from the Arduino."""

    distance_mm: int | None  # None when no echo (out of range / nothing seen)
    roll_deg: float
    pitch_deg: float
    contacts: tuple[bool, bool, bool, bool, bool, bool]

    def contact(self, leg: LegId) -> bool:
        """Foot-contact state for one leg."""
        return self.contacts[int(leg)]


def _clamp_int(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


def encode_telemetry(
    distance_mm: int | None,
    roll_deg: float,
    pitch_deg: float,
    contacts: Sequence[bool],
) -> bytes:
    """Build a telemetry frame. Used by the Arduino-side mock for offline tests
    (the real firmware emits the identical bytes in C)."""
    if len(contacts) != NUM_LEGS:
        raise ValueError(f"expected {NUM_LEGS} contact flags")
    dist = DISTANCE_NO_ECHO if distance_mm is None else _clamp_int(int(distance_mm), 0, 0xFFFE)
    roll = _clamp_int(int(round(roll_deg * 100)), -32768, 32767)
    pitch = _clamp_int(int(round(pitch_deg * 100)), -32768, 32767)
    cbyte = 0
    for i, c in enumerate(contacts):
        if c:
            cbyte |= 1 << i
    return _build_frame(MSG_TELEMETRY, struct.pack("<HhhB", dist, roll, pitch, cbyte))


def decode_telemetry(payload: bytes) -> Telemetry:
    """Decode a telemetry payload into a Telemetry record."""
    if len(payload) != TELEMETRY_PAYLOAD_LEN:
        raise ValueError(f"telemetry payload must be {TELEMETRY_PAYLOAD_LEN} bytes")
    dist, roll_cdeg, pitch_cdeg, cbyte = struct.unpack("<HhhB", payload)
    contacts = tuple(bool(cbyte & (1 << i)) for i in range(NUM_LEGS))
    return Telemetry(
        distance_mm=None if dist == DISTANCE_NO_ECHO else dist,
        roll_deg=roll_cdeg / 100.0,
        pitch_deg=pitch_cdeg / 100.0,
        contacts=contacts,  # type: ignore[arg-type]
    )


# ════════════════════════════════════════════════════════════════════════════
# Streaming frame parser — tolerates chunked reads, garbage, and bad CRCs.
# Real serial delivers bytes in arbitrary-sized chunks, so the receiver can't
# assume a read() returns exactly one frame. This buffers across feeds and
# resynchronises on the sync bytes.
# ════════════════════════════════════════════════════════════════════════════
class FrameParser:
    """Incremental parser. ``feed(bytes)`` returns the list of complete, CRC-
    valid (type, payload) frames found so far; partial frames stay buffered."""

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf = bytearray()
        self._max = max_buffer

    def _find_sync(self, start: int) -> int:
        buf = self._buf
        for j in range(start, len(buf) - 1):
            if buf[j] == SYNC0 and buf[j + 1] == SYNC1:
                return j
        return -1

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        buf = self._buf
        frames: list[tuple[int, bytes]] = []
        i = 0
        while True:
            start = self._find_sync(i)
            if start < 0:
                # No sync pair found; keep the final byte (it may be a stray
                # SYNC0 whose SYNC1 hasn't arrived yet) and discard the rest.
                i = max(i, len(buf) - 1)
                break
            if start + 4 > len(buf):
                i = start  # header (type+len) not fully here yet — wait
                break
            length = buf[start + 3]
            frame_len = 2 + 2 + length + 1  # sync2 + type + len + payload + crc
            if start + frame_len > len(buf):
                i = start  # payload/crc not fully here yet — wait
                break
            msg_type = buf[start + 2]
            payload = bytes(buf[start + 4:start + 4 + length])
            crc_recv = buf[start + 4 + length]
            crc_calc = crc8(buf[start + 2:start + 4 + length])  # type, len, payload
            if crc_calc == crc_recv:
                frames.append((msg_type, payload))
                i = start + frame_len
            else:
                i = start + 1  # corrupt frame; step past this sync and resync
        del buf[:i]
        if len(buf) > self._max:  # runaway-buffer guard (never seen a valid frame)
            del buf[:-1]
        return frames


# ════════════════════════════════════════════════════════════════════════════
# Self-test: `python -m pi.comms.protocol`
# ════════════════════════════════════════════════════════════════════════════
def _selftest() -> None:
    print("protocol self-test")
    print("=" * 56)

    # CRC determinism + sensitivity to a single-bit change.
    assert crc8(b"\x01\x02\x03") == crc8(b"\x01\x02\x03")
    assert crc8(b"\x01\x02\x03") != crc8(b"\x01\x02\x02")
    print("crc8 deterministic + change-sensitive .... OK")

    # Servo command round-trip through a frame + parser.
    pulses = [1500 + 10 * i for i in range(NUM_SERVOS)]
    frame = encode_servo_command(pulses)
    parser = FrameParser()
    got = parser.feed(frame)
    assert len(got) == 1 and got[0][0] == MSG_SERVO
    assert decode_servo_command(got[0][1]) == pulses
    print("servo command encode->parse->decode ...... OK")

    # Telemetry round-trip, incl. no-echo distance, negative tilt, contact bits.
    tframe = encode_telemetry(
        distance_mm=None, roll_deg=-3.21, pitch_deg=12.34,
        contacts=[True, False, True, False, False, True],
    )
    t = decode_telemetry(FrameParser().feed(tframe)[0][1])
    assert t.distance_mm is None
    assert abs(t.roll_deg + 3.21) < 1e-9 and abs(t.pitch_deg - 12.34) < 1e-9
    assert t.contacts == (True, False, True, False, False, True)
    assert t.contact(LegId.L1) and not t.contact(LegId.L2) and t.contact(LegId.R3)
    print("telemetry encode->parse->decode .......... OK")

    # Two frames concatenated in one read both come out.
    two = encode_servo_command([1500] * NUM_SERVOS) + tframe
    out = FrameParser().feed(two)
    assert len(out) == 2 and out[0][0] == MSG_SERVO and out[1][0] == MSG_TELEMETRY
    print("two concatenated frames in one read ...... OK")

    # Byte-by-byte (chunked) delivery still yields exactly one frame.
    p = FrameParser()
    collected: list = []
    for b in frame:
        collected += p.feed(bytes([b]))
    assert len(collected) == 1 and decode_servo_command(collected[0][1]) == pulses
    print("byte-by-byte chunked delivery ............ OK")

    # Leading garbage then a good frame: parser resynchronises.
    p = FrameParser()
    out = p.feed(b"\x00\xAA\x12\x99garbage" + frame)
    assert len(out) == 1 and out[0][0] == MSG_SERVO
    print("resync past leading garbage .............. OK")

    # Corrupted CRC is rejected, but a following good frame is still parsed.
    bad = bytearray(frame)
    bad[6] ^= 0xFF  # flip a payload byte; CRC no longer matches
    p = FrameParser()
    out = p.feed(bytes(bad) + tframe)
    types = [t for t, _ in out]
    assert MSG_SERVO not in types and MSG_TELEMETRY in types
    print("corrupt frame dropped, next frame kept ... OK")

    # Addressing sanity: index → board/channel.
    assert servo_index(LegId.L1, Joint.COXA) == 0
    assert servo_index(LegId.R3, Joint.TIBIA) == 17
    assert board_of(0) == 0 and board_of(8) == 0 and board_of(9) == 1 and board_of(17) == 1
    assert channel_of(9) == 0 and channel_of(17) == 8
    print("servo addressing (index/board/channel) ... OK")

    print("-" * 56)
    print("All protocol self-tests passed.")


if __name__ == "__main__":
    _selftest()
