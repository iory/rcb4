"""Byte-level tests for the Kondo RCB-4mini packet protocol.

Request vectors were **generated from ``rcb4``** at commit
``dfa222528e0f21a9ecd1495b7398ae7eb12e25bc`` (github.com/iory/rcb4), by
capturing what ``RCB4Interface`` passes to ``serial_write``::

    itf = RCB4Interface()
    itf.serial_write = capture
    itf.servo_on_states_dict = {i: True for i in (2, 5, 8, 10, 18, 30)}
    itf.get_ack(); itf.get_version()
    itf.write_move_ram_to_com_command(0x0000, 2)
    itf.write_move_com_to_ram_command(0x0000, [0x93, 0xC3])
    itf.servo_angle_vector([2, 5, 8, 10, 18, 30],
                           [7500, 8000, 7000, 32767, 32768, 11500], velocity=6)

Reply vectors were **recorded from an RCB-4mini** (2026-10-13) at 1.25 Mbit/s,
with ICS numbers 2, 5, 8, 10, 18 and 30 assigned in RAM.

If one of these fails, the driver has diverged from the board.
"""

import struct

import pytest

from rcb4.protocol import rcb4mini as protocol


def framed(body):
    return protocol.frame(body)


# --------------------------------------------------------------------------
# Requests, against rcb4
# --------------------------------------------------------------------------


def test_ack_matches_rcb4():
    assert framed(protocol.build_ack()).hex() == "04fe0608"


def test_version_request_matches_rcb4():
    assert framed(protocol.build_version()).hex() == "03fd00"


@pytest.mark.parametrize("address, size, expected", [
    (0x0000, 2, "0a00200000000000022c"),
    (0x0044, 70, "0a0020000000440046b4"),
    (0x0092, 126, "0a002000000092007e3a"),
])
def test_ram_read_matches_rcb4(address, size, expected):
    assert framed(protocol.build_ram_read(address, size)).hex() == expected


@pytest.mark.parametrize("address, data, expected", [
    (0x0000, bytes([0x93, 0xC3]), "09000200000093c361"),
    (0x0048, bytes([0xB8, 0x00]), "090002480000b8000b"),
])
def test_ram_write_matches_rcb4(address, data, expected):
    assert framed(protocol.build_ram_write(address, data)).hex() == expected


def test_multi_servo_matches_rcb4():
    body = protocol.build_multi_servo(
        [2, 5, 8, 10, 18, 30], [7500, 8000, 7000, 32767, 32768, 11500], 6)
    assert framed(body).hex() == "15102405044000064c1d401f581bff7f0080ec2ce9"


def test_multi_servo_per_servo_velocity_matches_rcb4():
    body = protocol.build_multi_servo([5, 2], [8000, 7500], [10, 20])
    assert framed(body).hex() == "0e112400000000144c1d0a401f29"


def test_ram_read_size_is_bounded_by_the_length_byte():
    protocol.build_ram_read(0x0090, protocol.MAX_READ_BYTES)
    with pytest.raises(protocol.ProtocolError):
        protocol.build_ram_read(0x0090, protocol.MAX_READ_BYTES + 1)
    with pytest.raises(protocol.ProtocolError):
        protocol.build_ram_read(0x0090, 0)


def test_ram_write_size_is_bounded_by_the_length_byte():
    assert len(framed(protocol.build_ram_write(0, bytes(protocol.MAX_WRITE_BYTES)))) == 0xFF
    with pytest.raises(protocol.ProtocolError):
        protocol.build_ram_write(0, bytes(protocol.MAX_WRITE_BYTES + 1))


# --------------------------------------------------------------------------
# Replies, recorded from the board
# --------------------------------------------------------------------------


def unframed(hex_packet):
    body, checksum_ok = protocol.unframe(bytes.fromhex(hex_packet))
    assert checksum_ok
    return body


def test_ack_reply():
    assert protocol.is_ack(unframed("04fe0608"))


def test_version_reply_is_the_rcb4mini_firmware():
    body = unframed(
        "23fd43422d342056312e3020202020202030393037313520202020202020202020c708")
    assert body == protocol.FIRMWARE_VERSION_REPLY


def test_config_read_reply():
    data = protocol.parse_ram_read(unframed("05009bc363"), 2)
    config = struct.unpack("<H", data)[0]
    assert config == 0xC39B
    assert protocol.com_bitrate_of(config) == 1250000


def test_assign_table_reply():
    data = protocol.parse_ram_read(unframed(
        "4900ffffffffb800fffffffff400ffffffff3001ffff5801ffffffffffffffff"
        + "fffffffffffff801ffffffffffffffffffffffffffffffffffffffffffffe802"
        + "ffffffffffffffff28"), 70)
    table = struct.unpack("<35H", data)
    assigned = {i: table[i] for i in range(35) if table[i] != protocol.ICS_UNASSIGNED}
    assert assigned == {i: protocol.servo_record_address(i)
                        for i in (2, 5, 8, 10, 18, 30)}


def test_servo_record_reply():
    address = protocol.servo_record_address(2)
    data = protocol.parse_ram_read(
        unframed("170000010000c227008000c227000000ffff01ffff0168"), 20)
    decoded = protocol.decode_servo_records(
        address, data, [2], ["current_angle", "ref_angle"])
    assert decoded == {2: {"current_angle": 0x27C2, "ref_angle": 0x8000}}


def test_short_read_reply_is_refused():
    with pytest.raises(protocol.ProtocolError):
        protocol.parse_ram_read(bytes([0x00, 0x01]), 2)


def test_write_ack():
    assert protocol.is_write_ack(bytes([0x00, 0x06]))
    assert not protocol.is_write_ack(bytes([0x00, 0x15]))


# --------------------------------------------------------------------------
# Layout helpers
# --------------------------------------------------------------------------


def test_record_reads_group_nearby_ids():
    ids = [30, 2, 18, 10, 5, 8]
    reads = protocol.plan_record_reads(ids, ["current_angle"])
    # 2..8 span 6 records (122 bytes); adding 10 would need 162, over the
    # 126-byte reads this board answers reliably, so 10, 18 and 30 read alone.
    assert reads == [(protocol.servo_record_address(2) + 4, 6 * 20 + 2),
                     (protocol.servo_record_address(10) + 4, 2),
                     (protocol.servo_record_address(18) + 4, 2),
                     (protocol.servo_record_address(30) + 4, 2)]
    assert max(size for _, size in reads) <= protocol.READ_CHUNK_BYTES
    covered = set()
    for address, size in reads:
        covered |= set(protocol.decode_servo_records(
            address, bytes(size), ids, ["current_angle"]))
    assert covered == set(ids)


def test_record_reads_can_use_the_whole_packet():
    reads = protocol.plan_record_reads([2, 5, 8, 10, 18, 30], ["current_angle"],
                                       max_bytes=protocol.MAX_READ_BYTES)
    assert [size for _, size in reads] == [8 * 20 + 2, 12 * 20 + 2]
    with pytest.raises(protocol.ProtocolError):
        protocol.plan_record_reads([2], ["current_angle"],
                                   max_bytes=protocol.MAX_READ_BYTES + 1)


def test_flat_config_stops_the_program_and_keeps_the_rates():
    # 0x831b: the config an RCB-4mini without a project powered up with.
    assert protocol.flat_config(0x831B) == 0x8311
    assert protocol.flat_config(0x8311) == 0x8311
    # Interpolation-end message on and a 25 ms cycle: both put back.
    assert protocol.flat_config(0xC2B5) == 0xC291


def test_record_reads_cover_both_members():
    (address, size), = protocol.plan_record_reads([3], ["current_angle", "ref_angle"])
    assert address == protocol.servo_record_address(3) + 4
    assert size == 4


def test_unknown_member_is_refused():
    with pytest.raises(protocol.ProtocolError):
        protocol.plan_record_reads([3], ["error_angle"])


def test_ics_number_out_of_range_is_refused():
    with pytest.raises(protocol.ProtocolError):
        protocol.servo_record_address(protocol.ICS_SLOT_COUNT)


def test_config_bitrate_fields():
    assert protocol.with_com_bitrate(0xC313, 1250000) == 0xC393
    assert protocol.with_com_bitrate(0xC393, 115200) == 0xC313
    assert protocol.with_ics_bitrate(0x8313, 1250000) == 0xC313
    with pytest.raises(protocol.ProtocolError):
        protocol.with_com_bitrate(0x8313, 625000)
    with pytest.raises(protocol.ProtocolError):
        protocol.com_bitrate_of(0x8353)


# Generated from rcb4 (github.com/iory/rcb4, e5cd18d) with
# RCB4Interface.set_servo_parameters_command(values, servo_ids=ids)[1].
@pytest.mark.parametrize("stretch, expected", [
    ({15: 40, 14: 60}, "0b1200c000000001" "3c2842"),
    ({34: 30}, "0a12000000000401" "1e3f"),
])
def test_stretch_matches_rcb4(stretch, expected):
    assert framed(protocol.build_stretch(stretch)).hex() == expected


@pytest.mark.parametrize("stretch", [{}, {35: 40}, {2: 0}, {2: 128}])
def test_stretch_out_of_range_is_refused(stretch):
    with pytest.raises(protocol.ProtocolError):
        protocol.build_stretch(stretch)
