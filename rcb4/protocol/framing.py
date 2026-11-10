"""Packet framing and commands shared by the Kondo RCB-4 family.

The RCB-4mini and the ARM-H7 (whose firmware grew out of the RCB-4) frame
every packet the same way and share the ack, version and batch servo move
commands. This module holds exactly that: it builds and parses bytes, opens no
port and knows no radians or servo units.

Packet form
-----------
Every packet, in both directions, is::

    [len] [body ...] [checksum]

where ``len`` counts the whole packet (so ``len = len(body) + 2``) and
``checksum = (len + sum(body)) & 0xFF``. The board's replies carry it too
(measured on an RCB-4mini, ``references.md``), so :func:`unframe` refuses a
reply whose checksum does not match (:class:`ChecksumError`).

Every byte layout here is pinned by ``tests/rcb4_tests/test_protocol_framing.py``
against what :class:`rcb4.rcb4interface.RCB4Interface` sends and against
replies recorded from an RCB-4mini, so that one copy of the wire protocol
serves every host.
"""

from __future__ import annotations

import struct
from typing import Iterable
from typing import Sequence

# --------------------------------------------------------------------------
# Opcodes
# --------------------------------------------------------------------------

#: Firmware version string.
OP_VERSION = 0xFD
#: Ack round-trip (liveness check).
OP_ACK = 0xFE

#: Move N servos, one shared move duration.
OP_MULTI_SERVO_SINGLE_VELOCITY = 0x10
#: Move N servos, per-servo move duration.
OP_MULTI_SERVO_MULTI_VELOCITIES = 0x11

#: Servo ids are addressed as a bitmap this many bytes wide (ids 0..39).
SERVO_ID_BITMAP_BYTES = 5

#: Largest packet the board accepts (the length field is a single byte).
MAX_PACKET_LENGTH = 0xFF


class ProtocolError(RuntimeError):
    """A packet could not be built or parsed. Never silently repaired."""


class ChecksumError(ProtocolError):
    """A received packet's trailing byte is not its checksum.

    The packet arrived whole (its length byte matched), but its contents
    were corrupted on the way, so nothing in it can be trusted.
    """


# --------------------------------------------------------------------------
# Framing
# --------------------------------------------------------------------------


def checksum(packet: Iterable[int]) -> int:
    """Checksum of a packet's bytes: the low byte of their sum.

    Parameters
    ----------
    packet : iterable of int
        The length byte followed by the body (i.e. the packet without its
        trailing checksum byte).

    Returns
    -------
    int
        The checksum byte.
    """
    total = 0
    for byte in packet:
        total += int(byte) & 0xFF
    return total & 0xFF


def frame(body: bytes) -> bytes:
    """Wrap a packet body in its length byte and checksum.

    Parameters
    ----------
    body : bytes
        Opcode plus arguments, with neither length nor checksum.

    Returns
    -------
    bytes
        The complete packet ready for the wire.

    Raises
    ------
    ProtocolError
        If the packet would exceed the single-byte length field.
    """
    length = len(body) + 2
    if length > MAX_PACKET_LENGTH:
        raise ProtocolError(
            f"packet of {length} bytes exceeds the board's single-byte length field"
            )
    head = bytes([length]) + bytes(body)
    return head + bytes([checksum(head)])


def unframe(packet: bytes) -> bytes:
    """Strip a packet's length byte and trailing checksum, checking both.

    Returns
    -------
    bytes
        The packet contents between the length byte and the checksum.

    Raises
    ------
    ProtocolError
        If the leading length byte does not match the packet's actual length
        (a framing failure: the stream is out of step).
    ChecksumError
        If the trailing byte is not ``(len + sum(body)) & 0xFF``. The board
        checksums its replies (measured on an RCB-4mini, see
        ``references.md``), so a mismatch means a corrupted reply, never a
        board that leaves the byte out. It is a :class:`ProtocolError`, so a
        caller that only catches that still refuses the packet.
    """
    if len(packet) < 2:
        raise ProtocolError(f"packet too short to be framed: {packet!r}")
    if packet[0] != len(packet):
        raise ProtocolError(
            f"length byte {packet[0]} does not match the {len(packet)} bytes received"
            )
    expected = checksum(packet[:-1])
    if packet[-1] != expected:
        got, packet_hex = packet[-1], bytes(packet).hex()
        raise ChecksumError(
            f"checksum byte {got:#04x} is not {expected:#04x} in {packet_hex}")
    return bytes(packet[1:-1])


# --------------------------------------------------------------------------
# Packet builders (bodies -- the transport adds length + checksum)
# --------------------------------------------------------------------------


def build_version() -> bytes:
    """Body of the firmware-version request."""
    return bytes([OP_VERSION])


def build_ack() -> bytes:
    """Body of the ack round-trip. ``0x06`` is echoed back by the board."""
    return bytes([OP_ACK, 0x06])


def encode_id_bitmap(ids: Sequence[int],
                     num_bytes: int = SERVO_ID_BITMAP_BYTES) -> bytes:
    """Encode servo ids as a little-endian bitmap.

    The board addresses servos positionally: bit *i* selects servo id *i*. This
    is why the flat id -- not the ``(uart, id)`` pair -- is the identity on this
    board; the ICS channel a servo hangs off is configuration metadata, not an
    address.
    """
    bits = 0
    for servo_id in ids:
        if not 0 <= int(servo_id) < num_bytes * 8:
            raise ProtocolError(
                f"servo id {servo_id} outside the {num_bytes * 8}-bit address bitmap"
                )
        bits |= 1 << int(servo_id)
    return bytes((bits >> (8 * i)) & 0xFF for i in range(num_bytes))


def clamp_velocity(velocity: float) -> int:
    """Clamp a move-duration byte into the board's 1..255 range."""
    return max(1, min(255, int(round(velocity))))


def build_multi_servo(ids: Sequence[int], pulses: Sequence[int],
                      velocity) -> bytes:
    """Body of the batch servo move: **one** packet for every servo.

    The board's sync-write: an id bitmap followed by one 16-bit pulse per
    selected servo, in ascending id order.

    ``velocity`` is a *move duration*, not a speed -- the board interpolates to
    the target over ``velocity`` output cycles (15 ms each on an RCB-4mini
    with the frame field at ``01``, see ``references.md``) and each new packet
    restarts that interpolation. A scalar uses the compact single-velocity
    opcode; a per-servo sequence uses the interleaved one.

    Parameters
    ----------
    ids : sequence of int
        Servo ids. Need not be sorted; they are sorted here together with
        ``pulses`` because the board reads the pulses in bitmap order.
    pulses : sequence of int
        Target pulse per id (``0..0xFFFF``; the servo-on/off sentinels ride in
        this field too -- that is the actuator's business, not this layer's).
    velocity : float or sequence of float
        Shared or per-servo move duration.

    Returns
    -------
    bytes
        The command body.
    """
    ids = [int(i) for i in ids]
    if len(ids) != len(pulses):
        raise ProtocolError(
            f"ids and pulses differ in length: {len(ids)} vs {len(pulses)}"
            )
    if not ids:
        raise ProtocolError("a servo command with no servos is not a command")
    if len(set(ids)) != len(ids):
        raise ProtocolError(f"duplicate servo ids in one command: {ids}")

    per_servo = not isinstance(velocity, (int, float))
    if per_servo and len(velocity) != len(ids):
        raise ProtocolError(
            f"velocity has {len(velocity)} entries for {len(ids)} servos"
            )

    order = sorted(range(len(ids)), key=lambda i: ids[i])
    sorted_ids = [ids[i] for i in order]
    sorted_pulses = [max(0, min(0xFFFF, int(round(pulses[i])))) for i in order]

    body = bytearray()
    if per_servo:
        body.append(OP_MULTI_SERVO_MULTI_VELOCITIES)
        body += encode_id_bitmap(sorted_ids)
        for index, pulse in zip(order, sorted_pulses):
            body.append(clamp_velocity(velocity[index]))
            body += struct.pack("<H", pulse)
    else:
        body.append(OP_MULTI_SERVO_SINGLE_VELOCITY)
        body += encode_id_bitmap(sorted_ids)
        body.append(clamp_velocity(velocity))
        for pulse in sorted_pulses:
            body += struct.pack("<H", pulse)
    return bytes(body)
