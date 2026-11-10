"""Byte-level tests for the packet framing shared by the RCB-4 family.

The expected bytes were generated from ``rcb4`` at commit
``dc1ccf559ea90e0334a652711f3d2fec9aaab6a3`` by capturing what
``ARMH7Interface.servo_angle_vector`` passed to ``serial_write``.
"""


import pytest

from rcb4.protocol import framing as protocol


def framed(body):
    """Helper: the complete on-wire packet for a body."""
    return protocol.frame(body)


# --------------------------------------------------------------------------
# Servo command packets
# --------------------------------------------------------------------------


def test_multi_servo_single_velocity_matches_rcb4():
    """The batch move packet is byte-identical to rcb4's.

    Covers all three things that could silently differ: the 5-byte id bitmap
    (ids 0, 1, 5, 23, 30 land in bytes 0, 2 and 3), little-endian pulses, and
    the servo on/off sentinels riding in the pulse field.
    """
    body = protocol.build_multi_servo(
        [0, 1, 5, 23, 30], [7500, 8000, 7000, 32767, 32768], velocity=127)
    assert framed(body).hex() == "131023008040007f4c1d401f581bff7f0080be"


def test_multi_servo_sorts_by_id_like_rcb4():
    """Pulses must be reordered to match the bitmap's ascending id order.

    The board reads pulses positionally, so accepting the caller's order would
    deliver joint A's angle to joint B -- silently, and only for callers whose
    joint list happens not to be sorted.
    """
    body = protocol.build_multi_servo([5, 0, 1], [7000, 7500, 8000], velocity=1)
    assert framed(body).hex() == "0f102300000000014c1d401f581b7e"


def test_multi_servo_per_velocity_matches_rcb4():
    """The interleaved velocity/pulse form, used for per-joint move times."""
    body = protocol.build_multi_servo(
        [0, 1, 5], [7500, 8000, 7000], velocity=[10, 20, 255])
    assert framed(body).hex() == "111123000000000a4c1d14401fff581b9d"


def test_per_servo_velocity_follows_the_id_sort():
    """Velocities are reordered with their pulses, not left behind."""
    unsorted_body = protocol.build_multi_servo(
        [5, 0, 1], [7000, 7500, 8000], velocity=[255, 10, 20])
    sorted_body = protocol.build_multi_servo(
        [0, 1, 5], [7500, 8000, 7000], velocity=[10, 20, 255])
    assert unsorted_body == sorted_body


def test_velocity_is_clamped_into_the_boards_range():
    """0 and 300 are not valid move-duration bytes; 1 and 255 are."""
    assert protocol.clamp_velocity(0) == 1
    assert protocol.clamp_velocity(0.4) == 1
    assert protocol.clamp_velocity(300) == 255
    assert protocol.clamp_velocity(127.8) == 128


def test_duplicate_or_empty_servo_commands_are_refused():
    """A duplicate id collapses in the bitmap and would shift every later pulse."""
    with pytest.raises(protocol.ProtocolError):
        protocol.build_multi_servo([1, 1], [7500, 7600], velocity=127)
    with pytest.raises(protocol.ProtocolError):
        protocol.build_multi_servo([], [], velocity=127)
    with pytest.raises(protocol.ProtocolError):
        protocol.build_multi_servo([1, 2], [7500], velocity=127)


def test_servo_id_outside_the_bitmap_is_refused():
    """Id 40 has no bit in a 5-byte bitmap; encoding it would address nothing."""
    with pytest.raises(protocol.ProtocolError):
        protocol.build_multi_servo([40], [7500], velocity=127)


# --------------------------------------------------------------------------
# Memory access packets
# --------------------------------------------------------------------------


def test_version_and_ack_match_rcb4():
    assert framed(protocol.build_version()).hex() == "03fd00"
    assert framed(protocol.build_ack()).hex() == "04fe0608"


def test_frame_round_trips():
    body = bytes([0x10, 0x01, 0x02])
    packet = protocol.frame(body)
    assert packet[0] == len(packet)
    decoded, checksum_ok = protocol.unframe(packet)
    assert decoded == body
    assert checksum_ok is True


def test_unframe_rejects_a_length_mismatch():
    """A length byte that disagrees with the bytes received is a framing
    failure, not an assumption -- so it raises rather than being counted."""
    with pytest.raises(protocol.ProtocolError):
        protocol.unframe(bytes([0x09, 0x01, 0x02]))


def test_unframe_reports_a_bad_checksum_without_raising():
    """The board's replies have never been verified to carry a checksum, so a
    mismatch is reported for counting rather than treated as fatal."""
    packet = bytearray(protocol.frame(bytes([0x10, 0x01])))
    packet[-1] ^= 0xFF
    body, checksum_ok = protocol.unframe(bytes(packet))
    assert body == bytes([0x10, 0x01])
    assert checksum_ok is False


def test_oversized_packet_is_refused():
    with pytest.raises(protocol.ProtocolError):
        protocol.frame(bytes(300))
