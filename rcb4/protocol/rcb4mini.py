"""Wire protocol of the Kondo RCB-4mini control board.

Like :mod:`rcb4.protocol.framing`, this module only builds and parses packets
and holds the board's memory layout. It opens no port and knows no radians.

What is shared with the ARM-H7, and what is not
-----------------------------------------------
The ARM-H7 firmware grew out of the RCB-4, so the two boards share the packet
framing (length byte + checksum), the ack and version requests and the batch
servo move (``0x10``/``0x11``). Those live in :mod:`rcb4.protocol.framing`
and are re-exported here. ``tests/rcb4_tests/test_protocol_*.py`` pin every
request against what :class:`rcb4.rcb4interface.RCB4Interface` sends, so the
two cannot drift apart, and every reply parser against bytes recorded from an
RCB-4mini.

Everything about memory differs. The RCB-4mini has a 16-bit RAM address space
reached with the ``MOVE`` command (``0x00``), no strided read, and a fixed
layout -- there is no symbol map, because there is only one firmware build
(:data:`FIRMWARE_VERSION_REPLY`).

RAM layout used here
--------------------
* ``$0000`` -- system config word. Bits 6-7 select the COM (host) baud rate,
  bits 13-14 the ICS (servo) baud rate.
* ``$0044`` -- ICS assignment table, one uint16 per ICS number: ``$FFFF`` means
  unassigned, anything else is the RAM address of that number's servo record.
  The board talks only to assigned numbers. It is loaded from the HeartToHeart
  project in ROM at power-up, so a board without a project talks to no servo.
* ``$0090`` -- servo records, :data:`SERVO_RECORD_SIZE` bytes per ICS number.

Provenance: ``rcb4/protocol/references.md``.
"""

from __future__ import annotations

import struct
from typing import Sequence

from rcb4.protocol.framing import build_ack  # noqa: F401  (re-exported)
from rcb4.protocol.framing import build_multi_servo  # noqa: F401  (re-exported)
from rcb4.protocol.framing import build_version  # noqa: F401  (re-exported)
from rcb4.protocol.framing import ChecksumError  # noqa: F401  (re-exported)
from rcb4.protocol.framing import frame  # noqa: F401  (re-exported)
from rcb4.protocol.framing import ProtocolError
from rcb4.protocol.framing import unframe  # noqa: F401  (re-exported)

# --------------------------------------------------------------------------
# Opcodes
# --------------------------------------------------------------------------

#: Memory move. The sub-command selects the direction.
OP_MOVE = 0x00
#: ``MOVE`` sub-command: RAM -> COM, i.e. a read.
SUB_RAM_TO_COM = 0x20
#: ``MOVE`` sub-command: COM -> RAM, i.e. a write.
SUB_COM_TO_RAM = 0x02

#: Change a servo parameter (stretch or speed) of selected ICS numbers; the
#: board passes it on to the servos. The command reference prints "11h" in
#: this command's table, the code of the batch move it copies its layout from;
#: rcb4 sends 0x12 (references.md).
OP_SERVO_PARAM = 0x12
#: ``OP_SERVO_PARAM`` sub-command: stretch (the servo's holding gain).
SUB_STRETCH = 0x01
#: Stretch values the command takes.
STRETCH_MIN = 1
STRETCH_MAX = 127
#: Bytes of the ICS number mask in ``OP_SERVO_PARAM``: bit n = ICS number n.
SERVO_MASK_BYTES = 5

#: Byte the board puts after the echoed opcode to acknowledge a write or ack.
ACK_BYTE = 0x06
#: Second byte of a refusal. The board sometimes answers a request with a
#: two-byte body ``[x, 0x15]`` instead of carrying it out; ``x`` was seen as
#: 0x01, 0x02 and 0x0b and its meaning is not known. The same request a
#: moment later is answered normally (measured, see references.md).
NAK_BYTE = 0x15

#: The whole reply to the version request (opcode echo included), from the
#: only firmware build there is. ``rcb4`` compares against the same bytes.
FIRMWARE_VERSION_REPLY = b"\xfdCB-4 V1.0      090715          \xc7"

#: Largest single RAM read the packet can carry: the reply holds the data plus
#: three bytes in a packet whose length field is one byte.
MAX_READ_BYTES = 252
#: Largest read :func:`plan_record_reads` plans. Reads above it are not safe on
#: this board even though they fit a packet: ``$01FC``+242 repeated alone was
#: refused every second time, and every board hang whose last request was
#: logged ended on a read of 162 bytes or more (the board then answered
#: nothing until power-cycled). Reads of up to 126 drew no refusal and no hang,
#: and are what rcb4 and kxreus read (measured, see references.md).
READ_CHUNK_BYTES = 126
#: Largest single RAM write: the request carries seven bytes besides the data.
MAX_WRITE_BYTES = 0xFF - 7

# --------------------------------------------------------------------------
# RAM layout
# --------------------------------------------------------------------------

CONFIG_ADDRESS = 0x0000
#: Analog input 0, uint16. kxreus reads the supply voltage from it.
BATTERY_ADC_ADDRESS = 0x0022
#: Volts per count of :data:`BATTERY_ADC_ADDRESS`: kxreus's
#: ``:battery-voltage`` is ``count * 25 / 1024``.
BATTERY_VOLTS_PER_COUNT = 25.0 / 1024.0
ICS_ASSIGN_ADDRESS = 0x0044
ICS_UNASSIGNED = 0xFFFF
SERVO_RECORD_ADDRESS = 0x0090
SERVO_RECORD_SIZE = 20
#: ICS numbers the board has a record for.
ICS_SLOT_COUNT = 35

#: Servo record members this module decodes: (offset, struct format).
#: ``error_angle`` (+2) is deliberately absent: rcb4 reads it, but nothing pins
#: its signedness or meaning on this board.
SERVO_SLOTS: dict[str, tuple[int, str]] = {
    "current_angle": (4, "<H"),
    "ref_angle": (6, "<H"),
}

#: Config bit: run the program in ROM (EEPROM). With it set, the board's own
#: program runs alongside the host and can command the same servos.
CONFIG_RUN_ROM = 0x0002
#: Config bit: vector jumps (enter ROM motions on an event) are enabled.
CONFIG_VECTOR_JUMP = 0x0008
#: Config bit: the board sends a command out of COM when a servo's
#: interpolation ends ([21] firmware reference p.4). Bytes nobody asked for
#: would be read as the reply to the next request.
CONFIG_SERVO_RESPONSE = 0x0004
#: Config output-cycle field (bits 4-5): how often the board talks to the
#: servos, 00 = 10 ms, 01 = 15 ms, 10 = 20 ms, 11 = 25 ms ([21] p.4). One
#: move-byte unit is one cycle.
FRAME_MASK = 0x0030
#: The output cycle of an RCB-4mini without a project: 01, 15 ms (config
#: 0x8313 at power-up, measured, references.md).
FLAT_FRAME = 0x0010

COM_BITRATE_MASK = 0x00C0
COM_BITRATE_SHIFT = 6
#: Config value of the COM baud-rate field per host bit rate. Only the two that
#: were switched to and answered on a board are listed; 625000 is not.
COM_BITRATE_CODES = {115200: 0, 1250000: 2}

ICS_BITRATE_MASK = 0x6000
ICS_BITRATE_SHIFT = 13
#: Config value of the ICS baud-rate field per servo bit rate. KRS servos set
#: to 1.25 Mbit/s answered with 2 and not with 0 or 1 (3 also answered; what it
#: means is not known, so it is not used). 0 puts 115200 bit/s on the ICS line
#: (scope measurement) and KRS servos set to 115200 answer with it.
ICS_BITRATE_CODES = {115200: 0, 1250000: 2}


# --------------------------------------------------------------------------
# Packet builders (bodies -- the transport adds length + checksum)
# --------------------------------------------------------------------------


def build_ram_read(address: int, size: int) -> bytes:
    """Body of a read of ``size`` bytes of RAM at ``address``.

    Raises
    ------
    ProtocolError
        If the read does not fit one reply.
    """
    if not 0 < size <= MAX_READ_BYTES:
        raise ProtocolError(
            f"RAM read of {size} bytes; one reply carries 1..{MAX_READ_BYTES}")
    if not 0 <= address <= 0xFFFF:
        raise ProtocolError(f"RAM address {address:#x} is not 16-bit")
    return bytes([OP_MOVE, SUB_RAM_TO_COM, 0x00, 0x00, 0x00,
                  address & 0xFF, address >> 8, size])


def build_stretch(stretch: dict[int, int]) -> bytes:
    """Body that sets the stretch of each ICS number in ``stretch``.

    Parameters
    ----------
    stretch : dict
        ICS number -> stretch (:data:`STRETCH_MIN`..:data:`STRETCH_MAX`).

    Raises
    ------
    ProtocolError
        If a number or a value is out of range, or ``stretch`` is empty.
    """
    if not stretch:
        raise ProtocolError("a stretch command for no servo")
    mask = 0
    for slot, value in stretch.items():
        if not 0 <= slot < ICS_SLOT_COUNT:
            raise ProtocolError(f"ICS number {slot} outside the board's 0..{ICS_SLOT_COUNT - 1}")
        if not STRETCH_MIN <= value <= STRETCH_MAX:
            raise ProtocolError(f"stretch {value} for ICS {slot} outside {STRETCH_MIN}..{STRETCH_MAX}")
        mask |= 1 << slot
    # The values follow in ICS number order, one per set bit.
    return (bytes([OP_SERVO_PARAM]) + mask.to_bytes(SERVO_MASK_BYTES, "little")
            + bytes([SUB_STRETCH]) + bytes(stretch[s] for s in sorted(stretch)))


def build_ram_write(address: int, data: bytes) -> bytes:
    """Body of a write of ``data`` to RAM at ``address``."""
    if not 0 < len(data) <= MAX_WRITE_BYTES:
        raise ProtocolError(
            f"RAM write of {len(data)} bytes; one packet carries 1..{MAX_WRITE_BYTES}")
    if not 0 <= address <= 0xFFFF:
        raise ProtocolError(f"RAM address {address:#x} is not 16-bit")
    return bytes([OP_MOVE, SUB_COM_TO_RAM, address & 0xFF, address >> 8,
                  0x00]) + bytes(data)


# --------------------------------------------------------------------------
# Reply parsers
# --------------------------------------------------------------------------


def is_nak(body: bytes) -> bool:
    """Whether ``body`` is the board refusing a request (see :data:`NAK_BYTE`)."""
    return len(body) == 2 and body[1] == NAK_BYTE


def is_ack(body: bytes) -> bool:
    """Whether ``body`` is the reply to :func:`build_ack`."""
    return bytes(body) == bytes([0xFE, ACK_BYTE])


def is_write_ack(body: bytes) -> bool:
    """Whether ``body`` acknowledges a :func:`build_ram_write`."""
    return bytes(body) == bytes([OP_MOVE, ACK_BYTE])


def is_stretch_ack(body: bytes) -> bool:
    """Whether ``body`` acknowledges a :func:`build_stretch`."""
    return bytes(body) == bytes([OP_SERVO_PARAM, ACK_BYTE])


def parse_ram_read(body: bytes, size: int) -> bytes:
    """Data of a reply to ``build_ram_read(address, size)``.

    Raises
    ------
    ProtocolError
        If the reply is not a read reply of that size. A short reply is not
        padded: it means the stream is out of step.
    """
    body = bytes(body)
    if len(body) != size + 1 or body[0] != OP_MOVE:
        raise ProtocolError(
            f"expected a {size}-byte RAM read reply, got {body!r}")
    return body[1:]


# --------------------------------------------------------------------------
# Servo records
# --------------------------------------------------------------------------


def servo_record_address(slot: int) -> int:
    """RAM address of the servo record of ICS number ``slot``."""
    if not 0 <= slot < ICS_SLOT_COUNT:
        raise ProtocolError(
            f"ICS number {slot} outside the board's 0..{ICS_SLOT_COUNT - 1}")
    return SERVO_RECORD_ADDRESS + SERVO_RECORD_SIZE * slot


# ICS number = 2 x ID + port, port 0 for SIO1-4 and 1 for SIO5-8 (references.md,
# "Kondo RCB-4mini").
ICS_PORT_COUNT = 2
ICS_PORT_NAMES = ("SIO1-4", "SIO5-8")


def max_ics_id() -> int:
    """Highest servo ID the board has an ICS number for (port 0 only)."""
    return (ICS_SLOT_COUNT - 1) // ICS_PORT_COUNT


def ics_numbers_of(servo_id: int) -> list[int]:
    """ICS numbers a servo with ICS ID ``servo_id`` may answer on, one per port
    the board has a record for."""
    return [ICS_PORT_COUNT * servo_id + port for port in range(ICS_PORT_COUNT)
            if ICS_PORT_COUNT * servo_id + port < ICS_SLOT_COUNT]


def port_name(slot: int) -> str:
    """Connector group of ICS number ``slot``."""
    return ICS_PORT_NAMES[slot % ICS_PORT_COUNT]


def plan_record_reads(slots: Sequence[int], slot_names: Sequence[str],
                      max_bytes: int = READ_CHUNK_BYTES
                      ) -> list[tuple[int, int]]:
    """Group the records of ``slots`` into as few RAM reads as fit.

    Each read covers whole consecutive records, trimmed to the span of the
    requested members, so ids that sit close together share one round trip.

    Parameters
    ----------
    slots
        ICS numbers to read.
    slot_names
        Members of :data:`SERVO_SLOTS` to read.
    max_bytes
        Largest read to plan; see :data:`READ_CHUNK_BYTES`.

    Returns
    -------
    list of (int, int)
        ``(address, size)`` per read, in ascending address order.
    """
    if not 0 < max_bytes <= MAX_READ_BYTES:
        raise ProtocolError(
            f"reads of up to {max_bytes} bytes; one reply carries 1..{MAX_READ_BYTES}")
    if not slot_names:
        raise ProtocolError("no servo record member requested")
    for name in slot_names:
        if name not in SERVO_SLOTS:
            raise ProtocolError(
                f"unknown servo record member {name!r}; known: {sorted(SERVO_SLOTS)}")
    first = min(SERVO_SLOTS[name][0] for name in slot_names)
    last = max(SERVO_SLOTS[name][0] + struct.calcsize(SERVO_SLOTS[name][1])
               for name in slot_names)
    reads: list[tuple[int, int]] = []
    group: list[int] = []
    for slot in sorted({int(s) for s in slots}):
        servo_record_address(slot)
        if group and (slot - group[0]) * SERVO_RECORD_SIZE + last - first > max_bytes:
            reads.append(_span(group, first, last))
            group = []
        group.append(slot)
    if group:
        reads.append(_span(group, first, last))
    return reads


def _span(group: Sequence[int], first: int, last: int) -> tuple[int, int]:
    start = servo_record_address(group[0]) + first
    end = servo_record_address(group[-1]) + last
    return start, end - start


def decode_servo_records(address: int, data: bytes, slots: Sequence[int],
                         slot_names: Sequence[str]) -> dict[int, dict[str, int]]:
    """Pick each requested member of each ICS number out of a RAM read.

    Parameters
    ----------
    address, data
        Where the read started and what it returned.
    slots
        ICS numbers to decode; those outside the read are skipped.
    slot_names
        Members of :data:`SERVO_SLOTS` to decode.

    Returns
    -------
    dict
        ICS number -> member name -> raw value.
    """
    out: dict[int, dict[str, int]] = {}
    for slot in slots:
        values: dict[str, int] = {}
        for name in slot_names:
            offset, fmt = SERVO_SLOTS[name]
            begin = servo_record_address(slot) + offset - address
            end = begin + struct.calcsize(fmt)
            if begin < 0 or end > len(data):
                break
            values[name] = struct.unpack(fmt, data[begin:end])[0]
        else:
            out[int(slot)] = values
    return out


# --------------------------------------------------------------------------
# Config word
# --------------------------------------------------------------------------


def com_bitrate_of(config: int) -> int:
    """Host bit rate selected by ``config``.

    Raises
    ------
    ProtocolError
        If the field holds a value this module has no measured rate for.
    """
    code = (config & COM_BITRATE_MASK) >> COM_BITRATE_SHIFT
    for bitrate, value in COM_BITRATE_CODES.items():
        if value == code:
            return bitrate
    raise ProtocolError(
        f"config {config:#06x} selects COM baud-rate code {code}, which has no known "
        + "bit rate")


def with_com_bitrate(config: int, bitrate: int) -> int:
    """``config`` with its COM baud-rate field set for ``bitrate``."""
    if bitrate not in COM_BITRATE_CODES:
        raise ProtocolError(
            f"COM bit rate {bitrate} has no known config value; known: {sorted(COM_BITRATE_CODES)}")
    return ((config & ~COM_BITRATE_MASK)
            | (COM_BITRATE_CODES[bitrate] << COM_BITRATE_SHIFT)) & 0xFFFF


def with_ics_bitrate(config: int, bitrate: int) -> int:
    """``config`` with its ICS baud-rate field set for ``bitrate``."""
    if bitrate not in ICS_BITRATE_CODES:
        raise ProtocolError(
            f"ICS bit rate {bitrate} has no known config value; known: {sorted(ICS_BITRATE_CODES)}")
    return ((config & ~ICS_BITRATE_MASK)
            | (ICS_BITRATE_CODES[bitrate] << ICS_BITRATE_SHIFT)) & 0xFFFF


def flat_config(config: int) -> int:
    """``config`` with every setting a project changes put back to flat.

    The ROM program and vector jumps stopped (kxreus clears the same two bits
    around every servo command it sends, so that the board's own program does
    not command the servos at the same time), the interpolation-end message
    off, and the output cycle at :data:`FLAT_FRAME`. The COM and ICS rates are
    left to :func:`with_com_bitrate` and :func:`with_ics_bitrate`.
    """
    cleared = config & ~(CONFIG_RUN_ROM | CONFIG_VECTOR_JUMP
                         | CONFIG_SERVO_RESPONSE | FRAME_MASK)
    return (cleared | FLAT_FRAME) & 0xFFFF
