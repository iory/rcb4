"""Kondo ICS 3.5 / 3.6 serial servo protocol, without any I/O.

This is the protocol a host speaks *directly to a servo* -- a KRS servo on a
Dual USB Adapter in ICS mode, or on any half-duplex ICS line -- not the
RCB-4 board protocol of :mod:`rcb4.protocol.framing`. It has no length byte
and no checksum. What the host can check instead is

* the **loopback**: the line is shared, so the host first receives every byte
  it sent, unchanged; and
* the **reply header** ``R_CMD``: the command byte with its MSB cleared (the
  ID commands excepted), followed, for parameter commands, by the same
  sub-command.

:func:`parse_reply` checks both and raises :class:`IcsReplyError` on any
mismatch, so a reply from the wrong servo, a reply at the wrong baud rate or a
corrupted reply is never taken for data.

Packet form (byte numbers as in the manual, TX = sent, RX = received)::

    position    TX  CMD POS_H POS_L          RX  loopback(3) R_CMD TCH_H TCH_L
    read        TX  CMD SC                   RX  loopback(2) R_CMD SC DATA...
    write       TX  CMD SC DATA...           RX  loopback(n) R_CMD SC [DATA]
    ID          TX  CMD SC SC SC             RX  loopback(4) R_CMD

Every 7-bit field keeps its MSB at 0; only the command bytes (and the ID
replies) have it set. Provenance: ``rcb4/protocol/references.md``, "Kondo ICS
3.5 / 3.6".
"""

from __future__ import annotations

from typing import Sequence

from rcb4.protocol.framing import ProtocolError

# --------------------------------------------------------------------------
# Command header: main command in bits 5-7, servo ID in bits 0-4
# --------------------------------------------------------------------------

#: Move to a position, and reply with the current one.
CMD_POSITION = 0x80
#: Read a parameter (the sub-command says which).
CMD_READ = 0xA0
#: Write a parameter (the sub-command says which).
CMD_WRITE = 0xC0
#: Read or write the servo's ID.
CMD_ID = 0xE0

#: Bits 5-7 of a command byte.
CMD_MASK = 0xE0
#: Bits 0-4 of a command byte.
ID_MASK = 0x1F
#: Highest servo ID.
MAX_ID = 31
#: A servo's reply clears the MSB of the command byte it answers, so it cannot
#: be taken for a host command (not for the ID commands).
REPLY_MASK = 0x7F

# --------------------------------------------------------------------------
# Sub-commands (SC)
# --------------------------------------------------------------------------

#: The whole EEPROM, :data:`EEPROM_BYTES` 4-bit values.
SC_EEPROM = 0x00
#: Stretch (holding gain), 1..127.
SC_STRETCH = 0x01
#: Speed, 1..127.
SC_SPEED = 0x02
#: Read: current (0..63 one way, 64..127 the other). Write: current limit.
SC_CURRENT = 0x03
#: Read: temperature (smaller is hotter). Write: temperature limit.
SC_TEMPERATURE = 0x04
#: Read only, ICS 3.6: the current position, without commanding one.
SC_POSITION = 0x05

#: Sub-commands that read or write a single 7-bit value.
SINGLE_VALUE_SUBCOMMANDS = (SC_STRETCH, SC_SPEED, SC_CURRENT, SC_TEMPERATURE)

#: The EEPROM is read and written as 64 bytes, each holding 4 bits.
EEPROM_BYTES = 64

#: EEPROM bytes (0-based) that must be written back exactly as read: the
#: ``0x5A`` header (bytes 1-2 in the manual) and the factory calibration of
#: the individual servo (25-26, 33-50, 55-56: 変更禁止). Writing another
#: servo's values there loses this servo's calibration.
EEPROM_PROTECTED = tuple(
    i - 1 for i in [1, 2, 25, 26] + list(range(33, 51)) + [55, 56])


def merge_eeprom(current: Sequence[int], wanted: Sequence[int],
                 keep: Sequence[int] = ()) -> list:
    """``wanted`` with every :data:`EEPROM_PROTECTED` byte, and each byte in
    ``keep``, taken from ``current`` (the servo's own EEPROM, as read).

    Parameters
    ----------
    current
        The EEPROM read from the servo this will be written to.
    wanted
        The values to write.
    keep
        More 0-based bytes to leave as they are, e.g. the ID (56, 57) or the
        baud rate (26, 27).
    """
    current, wanted = list(current), list(wanted)
    for name, values in (("current", current), ("wanted", wanted)):
        if len(values) != EEPROM_BYTES:
            raise ProtocolError(
                f"{name} EEPROM has {len(values)} values; it is {EEPROM_BYTES}")
    for index in set(EEPROM_PROTECTED) | set(keep):
        wanted[index] = current[index]
    return wanted


#: ID command sub-commands, sent three times.
SC_ID_READ = 0x00
SC_ID_WRITE = 0x01
#: Command byte of an ID read: ID bits all set, so any servo answers.
ID_READ_COMMAND = CMD_ID | ID_MASK

# --------------------------------------------------------------------------
# Positions
# --------------------------------------------------------------------------

#: A position is 14 bits, sent as two 7-bit halves.
POSITION_MAX = 0x3FFF
#: Position 0 releases the servo (free) instead of moving it.
POSITION_FREE = 0
#: Neutral of a 270 deg servo, whose range is 3500..11500.
POSITION_NEUTRAL = 7500


class IcsReplyError(ProtocolError):
    """A servo's reply is not the answer to the command that was sent.

    The loopback differs from what was sent, the reply header names another
    command, servo or sub-command, or the reply has the wrong length.
    """


# --------------------------------------------------------------------------
# Encoding
# --------------------------------------------------------------------------


def _check_id(servo_id: int) -> int:
    if not 0 <= servo_id <= MAX_ID:
        raise ProtocolError(f"servo ID {servo_id} outside 0..{MAX_ID}")
    return servo_id


def _check_7bit(name: str, value: int, low: int = 0, high: int = 0x7F) -> int:
    if not low <= value <= high:
        raise ProtocolError(f"{name} {value} outside {low}..{high}")
    return value


def encode_position(pulse: int) -> bytes:
    """``POS_H POS_L``: the 14-bit ``pulse`` split into two 7-bit halves."""
    pulse = int(pulse)
    if not 0 <= pulse <= POSITION_MAX:
        raise ProtocolError(f"position {pulse} outside 0..{POSITION_MAX}")
    return bytes([(pulse >> 7) & 0x7F, pulse & 0x7F])


def decode_position(high: int, low: int) -> int:
    """The 14-bit position carried by ``TCH_H TCH_L``."""
    return ((high & 0x7F) << 7) | (low & 0x7F)


def build_position(servo_id: int, pulse: int) -> bytes:
    """Move servo ``servo_id`` to ``pulse`` (:data:`POSITION_FREE` releases it)."""
    return bytes([CMD_POSITION | _check_id(servo_id)]) + encode_position(pulse)


def build_read(servo_id: int, subcommand: int) -> bytes:
    """Read the parameter ``subcommand`` (``SC_*``) of servo ``servo_id``."""
    if subcommand not in (SC_EEPROM, SC_POSITION) + SINGLE_VALUE_SUBCOMMANDS:
        raise ProtocolError(f"no readable parameter {subcommand:#04x}")
    return bytes([CMD_READ | _check_id(servo_id), subcommand])


def build_write(servo_id: int, subcommand: int, value: int) -> bytes:
    """Write one value: stretch, speed, current limit or temperature limit."""
    if subcommand not in SINGLE_VALUE_SUBCOMMANDS:
        raise ProtocolError(
            f"sub-command {subcommand:#04x} does not write a single value")
    return bytes([CMD_WRITE | _check_id(servo_id), subcommand,
                  _check_7bit("value", int(value), 1)])


def build_eeprom_write(servo_id: int, eeprom: Sequence[int]) -> bytes:
    """Write the whole EEPROM: :data:`EEPROM_BYTES` values of 0..15.

    Several fields must be written back exactly as read (factory calibration,
    the ``0x5A`` header); start from a :func:`build_read` of
    :data:`SC_EEPROM`, never from scratch.
    """
    eeprom = [int(v) for v in eeprom]
    if len(eeprom) != EEPROM_BYTES:
        raise ProtocolError(
            f"EEPROM write of {len(eeprom)} values; it is {EEPROM_BYTES}")
    for index, value in enumerate(eeprom):
        _check_7bit(f"EEPROM byte {index + 1}", value, 0, 0x0F)
    return bytes([CMD_WRITE | _check_id(servo_id), SC_EEPROM] + eeprom)


def build_id_read() -> bytes:
    """Ask the one servo on the line for its ID. Never on a shared line."""
    return bytes([ID_READ_COMMAND, SC_ID_READ, SC_ID_READ, SC_ID_READ])


def build_id_write(servo_id: int) -> bytes:
    """Set the one servo on the line to ``servo_id``. Never on a shared line:
    every servo that hears it takes the ID."""
    return bytes([CMD_ID | _check_id(servo_id),
                  SC_ID_WRITE, SC_ID_WRITE, SC_ID_WRITE])


# --------------------------------------------------------------------------
# Replies
# --------------------------------------------------------------------------


def reply_data_length(command: bytes) -> int:
    """Bytes the servo sends after the loopback of ``command``, ``R_CMD``
    included."""
    command = bytes(command)
    if not command:
        raise ProtocolError("empty ICS command")
    kind = command[0] & CMD_MASK
    if kind == CMD_POSITION:
        return 3                                     # R_CMD TCH_H TCH_L
    if kind == CMD_ID:
        return 1                                     # R_CMD
    if len(command) < 2:
        raise ProtocolError(f"ICS command {command.hex()} has no sub-command")
    subcommand = command[1]
    if kind == CMD_READ:
        if subcommand == SC_EEPROM:
            return 2 + EEPROM_BYTES                  # R_CMD SC EEPROM...
        if subcommand == SC_POSITION:
            return 4                                 # R_CMD SC TCH_H TCH_L
        return 3                                     # R_CMD SC value
    if kind == CMD_WRITE:
        if subcommand == SC_EEPROM:
            return 2                                 # R_CMD SC
        return 3                                     # R_CMD SC value
    raise ProtocolError(f"{command[0]:#04x} is not an ICS command byte")


def reply_length(command: bytes) -> int:
    """Bytes the host receives for ``command``: its loopback, then the reply."""
    return len(bytes(command)) + reply_data_length(command)


def parse_reply(command: bytes, received: bytes) -> bytes:
    """Check what came back for ``command`` and return the reply's data.

    Parameters
    ----------
    command
        The bytes sent.
    received
        Everything received for it: the loopback, then the servo's reply.

    Returns
    -------
    bytes
        What follows the reply header: ``TCH_H TCH_L`` for a position, the
        value(s) for a read, the value written back for a single-value
        write, nothing for an EEPROM write. An ID command's reply *is* its
        header; use :func:`parse_id_reply`.

    Raises
    ------
    IcsReplyError
        If ``received`` has the wrong length, its loopback differs from
        ``command``, or the reply header names another command, servo or
        sub-command.
    """
    command, received = bytes(command), bytes(received)
    expected = reply_length(command)
    if len(received) != expected:
        got, sent = len(received), command.hex()
        raise IcsReplyError(
            f"{got} bytes received for {sent}; expected {expected} (loopback + reply)")
    loopback, reply = received[:len(command)], received[len(command):]
    if loopback != command:
        raise IcsReplyError(
            f"loopback {loopback.hex()} is not the command {command.hex()}")
    kind, servo_id = command[0] & CMD_MASK, command[0] & ID_MASK
    if kind == CMD_ID:
        if reply[0] & CMD_MASK != CMD_ID:
            raise IcsReplyError(f"reply {reply.hex()} is not an ID reply")
        if command[1] == SC_ID_WRITE and reply[0] != command[0]:
            raise IcsReplyError(
                f"ID write {command.hex()} answered with {reply.hex()}")
        return b""
    header = command[0] & REPLY_MASK
    if reply[0] != header and not (
            # ICS 2.0 compatibility: a position reply from ID 0 at 115200
            # bit/s keeps the MSB. Accepted for ID 0 at any rate, since this
            # layer does not know the rate; it is still the command's own
            # byte, so nothing else is taken for it.
            kind == CMD_POSITION and servo_id == 0
            and reply[0] == command[0]):
        got, cmd = reply[0], command[0]
        raise IcsReplyError(
            f"reply header {got:#04x} is not {header:#04x} (answer to {cmd:#04x})")
    if kind == CMD_POSITION:
        return reply[1:]
    if reply[1] != command[1]:
        got, sent = reply[1], command[1]
        raise IcsReplyError(f"reply sub-command {got:#04x} is not {sent:#04x}")
    data = reply[2:]
    if kind == CMD_WRITE and command[1] != SC_EEPROM and data != command[2:]:
        raise IcsReplyError(
            f"write of {command[2:].hex()} answered with {data.hex()}")
    return data


def parse_position_reply(command: bytes, received: bytes) -> int:
    """The current position in the reply to a position command, or to a
    read of :data:`SC_POSITION`."""
    data = parse_reply(command, received)
    if len(data) != 2:
        raise IcsReplyError(f"{command.hex()} carries no position")
    return decode_position(data[0], data[1])


def parse_id_reply(command: bytes, received: bytes) -> int:
    """The servo ID in the reply to :func:`build_id_read` or
    :func:`build_id_write`."""
    parse_reply(command, received)
    return bytes(received)[-1] & ID_MASK
