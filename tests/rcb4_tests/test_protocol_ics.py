"""Byte-level tests for the Kondo ICS 3.5 / 3.6 servo protocol.

The expected bytes are the worked examples of the Kondo ICS3.5/3.6 software
manual (command reference, Ver 2.6): pages 15, 17, 20 and 22. See
``rcb4/protocol/references.md``, "Kondo ICS 3.5 / 3.6".
"""

import pytest

from rcb4.protocol import ics


def rx(command, reply_hex):
    """What the host receives: the loopback of ``command``, then the reply."""
    return bytes(command) + bytes.fromhex(reply_hex)


# --------------------------------------------------------------------------
# The manual's examples
# --------------------------------------------------------------------------


def test_position_7500_to_id_1():
    """p.15: ID 1 to 7500 is 81 3A 4C; the reply is 01 3A 4C."""
    command = ics.build_position(1, 7500)
    assert command.hex() == "813a4c"
    assert ics.reply_length(command) == 6
    assert ics.parse_position_reply(command, rx(command, "013a4c")) == 7500


def test_stretch_read_of_id_1():
    """p.17: A1 01, answered 21 01 1E (stretch 30)."""
    command = ics.build_read(1, ics.SC_STRETCH)
    assert command.hex() == "a101"
    assert ics.reply_length(command) == 5
    assert ics.parse_reply(command, rx(command, "21011e")) == bytes([30])


def test_speed_write_of_100_to_id_10():
    """p.20: CA 02 64, answered 4A 02 64."""
    command = ics.build_write(10, ics.SC_SPEED, 100)
    assert command.hex() == "ca0264"
    assert ics.reply_length(command) == 6
    assert ics.parse_reply(command, rx(command, "4a0264")) == bytes([100])


def test_id_read_and_write():
    """p.22: FF 00 00 00 reads the ID; F4 01 01 01 sets 20, answered F4."""
    read = ics.build_id_read()
    assert read.hex() == "ff000000"
    assert ics.parse_id_reply(read, rx(read, "f4")) == 20
    write = ics.build_id_write(20)
    assert write.hex() == "f4010101"
    assert ics.parse_id_reply(write, rx(write, "f4")) == 20


# --------------------------------------------------------------------------
# Lengths and encodings
# --------------------------------------------------------------------------


def test_reply_lengths_of_every_command():
    eeprom = [0] * ics.EEPROM_BYTES
    assert ics.reply_length(ics.build_read(3, ics.SC_EEPROM)) == 2 + 66
    assert ics.reply_length(ics.build_read(3, ics.SC_POSITION)) == 2 + 4
    assert ics.reply_length(ics.build_read(3, ics.SC_TEMPERATURE)) == 2 + 3
    assert ics.reply_length(ics.build_eeprom_write(3, eeprom)) == 66 + 2
    assert ics.reply_length(ics.build_write(3, ics.SC_STRETCH, 60)) == 3 + 3


def test_position_halves_round_trip():
    for pulse in (0, 3500, 7500, 11500, ics.POSITION_MAX):
        high, low = ics.encode_position(pulse)
        assert high < 0x80 and low < 0x80
        assert ics.decode_position(high, low) == pulse


def test_angle_read_returns_both_halves():
    command = ics.build_read(2, ics.SC_POSITION)
    assert command.hex() == "a205"
    assert ics.parse_position_reply(command, rx(command, "22053a4c")) == 7500


@pytest.mark.parametrize("build", [
    lambda: ics.build_position(32, 7500),
    lambda: ics.build_position(1, ics.POSITION_MAX + 1),
    lambda: ics.build_read(1, 0x06),
    lambda: ics.build_write(1, ics.SC_EEPROM, 1),
    lambda: ics.build_write(1, ics.SC_SPEED, 128),
    lambda: ics.build_eeprom_write(1, [0] * 63),
    lambda: ics.build_eeprom_write(1, [0x10] + [0] * 63),
])
def test_out_of_range_commands_are_refused(build):
    with pytest.raises(ics.ProtocolError):
        build()


# --------------------------------------------------------------------------
# Replies that are not the answer
# --------------------------------------------------------------------------


@pytest.mark.parametrize("received", [
    bytes.fromhex("813a4c013a"),          # one byte short
    bytes.fromhex("813a4d013a4c"),        # loopback differs from the command
    bytes.fromhex("813a4c023a4c"),        # another servo's reply
    bytes.fromhex("813a4c213a4c"),        # a read reply, not a position reply
])
def test_bad_position_replies_raise(received):
    command = ics.build_position(1, 7500)
    with pytest.raises(ics.IcsReplyError):
        ics.parse_reply(command, received)


def test_wrong_subcommand_raises():
    command = ics.build_read(1, ics.SC_STRETCH)
    with pytest.raises(ics.IcsReplyError):
        ics.parse_reply(command, rx(command, "21021e"))


def test_write_answered_with_another_value_raises():
    command = ics.build_write(10, ics.SC_SPEED, 100)
    with pytest.raises(ics.IcsReplyError):
        ics.parse_reply(command, rx(command, "4a0263"))


def test_id_write_answered_with_another_id_raises():
    command = ics.build_id_write(20)
    with pytest.raises(ics.IcsReplyError):
        ics.parse_id_reply(command, rx(command, "f5"))


def test_id_0_position_reply_may_keep_its_msb():
    """p.14-15: at 115.2 kbit/s a position reply from ID 0 keeps the MSB."""
    command = ics.build_position(0, 7500)
    assert ics.parse_position_reply(command, rx(command, "803a4c")) == 7500
    assert ics.parse_position_reply(command, rx(command, "003a4c")) == 7500


def test_reply_error_is_a_protocol_error():
    assert issubclass(ics.IcsReplyError, ics.ProtocolError)
