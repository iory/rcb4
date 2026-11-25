"""The RCB-4mini ICS relay (0x0E) and the servo EEPROM settings built on it.

The relay bytes and the EEPROM were **recorded from an RCB-4mini** with a KRS
servo at ICS 3 (ID 1 on SIO5-8), 2026-11-26: the relay of an EEPROM read
was acknowledged ``04 0e 06 18`` and left the reply below in RAM, and so was
the 66-byte EEPROM write (``41 00``). See ``rcb4/protocol/references.md``.
"""

import pytest

from rcb4.protocol import ics
from rcb4.protocol import rcb4mini as protocol
from rcb4.protocol.framing import ProtocolError
from rcb4.rcb4interface import RCB4Interface

#: The EEPROM of the recorded servo, as its reply carried it.
EEPROM = list(bytes.fromhex(
    "050a0b04070f00000002010e0f0a0006020c0e0c000d0a0c00000000000a010e"
    + "010209080a0009090304090d0f0909090d0a000100000e060001030c07080f0e"))


# --------------------------------------------------------------------------
# Protocol
# --------------------------------------------------------------------------


def test_relay_request_as_recorded():
    body = protocol.build_ics_relay(1, 2, 0x0400, 0x0410)
    assert protocol.frame(body).hex() == "090e01020004100432"
    assert protocol.is_ics_relay_ack(protocol.unframe(bytes.fromhex("040e0618")))


@pytest.mark.parametrize("port, count, tx, rx", [
    (2, 2, 0x0400, 0x0410), (0, 0, 0x0400, 0x0410),
    (0, protocol.ICS_RELAY_MAX_BYTES + 1, 0x0400, 0x0410), (0, 2, 0x0490, 0x0410),
])
def test_relay_request_out_of_range(port, count, tx, rx):
    with pytest.raises(ProtocolError):
        protocol.build_ics_relay(port, count, tx, rx)


def test_recorded_eeprom_decodes():
    settings = ics.eeprom_settings(EEPROM)
    assert settings["stretch"] == 180
    assert settings["speed"] == 127
    assert settings["dead_band"] == 2
    assert settings["id"] == 1
    assert settings["baud"] == ics.EEPROM_BAUD_CODES[1250000]
    assert settings["flags"] == 0x06
    assert settings["free"] and not settings["rotation"] and not settings["reverse"]


@pytest.mark.parametrize("name, value, flags", [
    ("rotation", True, 0x16), ("reverse", True, 0x07), ("pwm_inhibit", True, 0x0E),
])
def test_flags_set_one_bit(name, value, flags):
    eeprom = ics.with_eeprom_value(EEPROM, name, value)
    assert ics.eeprom_value(eeprom, "flags") == flags
    assert ics.with_eeprom_value(eeprom, name, False) == EEPROM


def test_fields_round_trip():
    eeprom = ics.with_eeprom_value(EEPROM, "pulse_max", 11000)
    eeprom = ics.with_eeprom_value(eeprom, "user_offset", -5)
    assert ics.eeprom_value(eeprom, "pulse_max") == 11000
    assert ics.eeprom_value(eeprom, "user_offset") == -5
    assert eeprom[52:54] == [0x0F, 0x0B]


@pytest.mark.parametrize("name, value", [
    ("free", False), ("current_limit", 64), ("stretch", 101), ("nothing", 1),
])
def test_bad_writes_are_refused(name, value):
    with pytest.raises(ProtocolError):
        ics.with_eeprom_value(EEPROM, name, value)


def test_flags_keep_free_and_the_fixed_bit():
    eeprom = ics.with_eeprom_value(EEPROM, "flags", 0x00)
    assert ics.eeprom_value(eeprom, "flags") == 0x06


# --------------------------------------------------------------------------
# RCB4Interface over a board that relays to one servo
# --------------------------------------------------------------------------


class RelayBoard(RCB4Interface):
    """A board with RAM and one servo at ICS ``servo``, answering the relay
    as the recorded board did."""

    def __init__(self, servo=3, eeprom=EEPROM):
        super().__init__()
        self.ram = bytearray(range(256)) * 5
        self.servo = servo
        self.eeprom = list(eeprom)
        self.relays = []
        self.freed = []

    def search_servo_ids(self):
        return [self.servo]

    def free(self, servo_ids=None):
        self.freed.extend(servo_ids)

    def _relay(self, port, count, tx, rx):
        command = bytes(self.ram[tx:tx + count])
        self.relays.append(command)
        ics_id = command[0] & ics.ID_MASK
        assert 2 * ics_id + port == self.servo
        if command[0] & ics.CMD_MASK == ics.CMD_READ:
            reply = bytes([0x20 | ics_id, 0]) + bytes(self.eeprom)
        else:
            self.eeprom = list(command[2:])
            reply = bytes([0x40 | ics_id, 0])
        self.ram[rx:rx + len(reply)] = reply

    def serial_write(self, byte_list, timeout=10, close_on_timeout=True):
        body = protocol.unframe(bytes(byte_list))
        if body[:2] == bytes([protocol.OP_MOVE, protocol.SUB_RAM_TO_COM]):
            address, size = body[5] | body[6] << 8, body[7]
            return bytes([protocol.OP_MOVE]) + bytes(self.ram[address:address + size])
        if body[:2] == bytes([protocol.OP_MOVE, protocol.SUB_COM_TO_RAM]):
            address, data = body[2] | body[3] << 8, body[5:]
            self.ram[address:address + len(data)] = data
            return bytes([protocol.OP_MOVE, protocol.ACK_BYTE])
        if body[0] == protocol.OP_ICS_RELAY:
            self._relay(body[1], body[2], body[3] | body[4] << 8, body[5] | body[6] << 8)
            return bytes([protocol.OP_ICS_RELAY, protocol.ACK_BYTE])
        raise AssertionError(f"unexpected request {body.hex()}")


def test_settings_read_and_ram_restored():
    board = RelayBoard()
    before = bytes(board.ram)
    assert board.servo_settings(3)["dead_band"] == 2
    assert board.relays == [bytes([0xA1, 0x00])]
    assert bytes(board.ram) == before


def test_rotation_frees_the_servo_and_writes_one_bit():
    board = RelayBoard()
    settings = board.set_servo_settings(3, rotation=True)
    assert settings["rotation"] and settings["flags"] == 0x16
    assert board.freed == [3]
    assert [c[:2] for c in board.relays] == [bytes([0xA1, 0]), bytes([0xA1, 0]),
                                             bytes([0xC1, 0]), bytes([0xA1, 0])]
    assert len(board.relays[2]) == 66
    assert ics.with_eeprom_value(EEPROM, "rotation", True) == board.eeprom


def test_unchanged_settings_write_nothing():
    board = RelayBoard()
    board.set_servo_settings(3, dead_band=2)
    assert all(c[0] == 0xA1 for c in board.relays)
    assert board.freed == []


def test_id_baud_and_calibration_are_kept():
    board = RelayBoard()
    wanted = [0] * ics.EEPROM_BYTES
    wanted = ics.with_eeprom_value(wanted, "dead_band", 4)
    written = board.write_servo_eeprom(3, wanted)
    for index in list(ics.EEPROM_PROTECTED) + [26, 27, 56, 57]:
        assert written[index] == EEPROM[index]
    assert ics.eeprom_value(written, "dead_band") == 4


@pytest.mark.parametrize("call", [
    lambda b: b.set_servo_settings(3, id=2),
    lambda b: b.set_servo_settings(3, slave=True),
    lambda b: b.ics_relay(2, bytes([0xA1, 0x00])),       # no servo at ICS 2
    lambda b: b.ics_relay(3, bytes([0xA2, 0x00])),       # ID 2 is not ICS 3
    lambda b: b.ics_relay(3, bytes([0xFF, 0, 0, 0])),    # ID command
])
def test_refused_before_anything_is_sent(call):
    board = RelayBoard()
    with pytest.raises(ValueError):
        call(board)
    assert board.relays == []
