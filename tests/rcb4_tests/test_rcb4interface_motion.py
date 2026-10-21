import unittest
import unittest.mock

import numpy as np
import serial

from rcb4.rcb4interface import flat_frame
from rcb4.rcb4interface import RamAddr
from rcb4.rcb4interface import RCB4Interface
from rcb4.rcb4interface import ServoOnOffValues


def record(ics_number):
    return RamAddr.ServoRamAddress.value + 20 * ics_number


class FakeBoard(RCB4Interface):
    """RCB4Interface on a RAM bytearray, recording the commands it sends."""

    def __init__(self, servo_ids, reply=b"\x12\x06"):
        super().__init__()
        self.ram = bytearray(0x490)
        self._config_data = flat_frame
        self.sent = []
        self.reply = reply
        self.servo_ids = servo_ids
        self.search_servo_ids()

    def _angle_vector(self, slot="current"):
        av = np.zeros(36)
        av[self.servo_ids] = 7500 if slot == "current" else ServoOnOffValues.OFF.value
        return av

    def write_move_ram_to_com_command(self, src_addr, src_data_size):
        return list(self.ram[src_addr:src_addr + src_data_size])

    def serial_write(self, byte_list, timeout=10):
        self.sent.append(bytes(byte_list))
        return self.reply


class TestTime(unittest.TestCase):

    def test_a_move_lasts_speed_output_cycles(self):
        board = FakeBoard([2, 5])
        self.assertAlmostEqual(board.frame_time(), 0.015)
        self.assertEqual(board.time_to_velocity(1.5), 100)
        self.assertEqual(board.time_to_velocity(0), 1)
        with self.assertRaises(ValueError):
            board.time_to_velocity(255 * 0.015 + 0.01)

    def test_time_sets_the_speed_byte(self):
        board = FakeBoard([2, 5])
        board.hold()
        board.angle_vector([10, 20], [2, 5], time=1.5)
        # size, command, 5-byte mask, speed, positions..., checksum
        self.assertEqual(board.sent[-1][7], 100)


class TestFreeServos(unittest.TestCase):

    def test_a_free_servo_is_not_moved_and_is_named(self):
        board = FakeBoard([2, 5])
        board.hold([2])
        board.sent.clear()
        with unittest.mock.patch("builtins.print") as printed:
            board.angle_vector([10, 20], [2, 5])
        printed.assert_called_once()
        self.assertIn("[5]", printed.call_args[0][0])
        # Only ICS 2 is in the command's mask.
        self.assertEqual(board.sent[-1][2:7], bytes([0x04, 0, 0, 0, 0]))


class TestInterpolation(unittest.TestCase):

    def test_moving_servos_are_the_ones_with_a_speed(self):
        board = FakeBoard([2, 5, 30])
        board.ram[record(5) + 8] = 100
        board.ram[record(30) + 8] = 3
        self.assertEqual(board.interpolating_servo_ids(), [5, 30])
        self.assertFalse(board.wait_interpolation(timeout=0.02))
        board.ram[record(5) + 8] = 0
        board.ram[record(30) + 8] = 0
        self.assertTrue(board.wait_interpolation())

    def test_print_servos(self):
        board = FakeBoard([2, 5])
        board.ram[record(2) + 4:record(2) + 8] = (7500).to_bytes(2, "little") + (
            ServoOnOffValues.OFF.value).to_bytes(2, "little")
        board.ram[record(5) + 4:record(5) + 8] = (8389).to_bytes(2, "little") * 2
        with unittest.mock.patch("builtins.print") as printed:
            board.print_servos()
        rows = [c[0][0] for c in printed.call_args_list]
        self.assertEqual(rows[1].split(), ["2", "1", "1-4", "0.0", "-", "off", "no"])
        self.assertEqual(rows[2].split(), ["5", "2", "5-8", "30.0", "30.0", "on", "no"])


class TestStretch(unittest.TestCase):

    def test_one_value_for_every_servo(self):
        board = FakeBoard([2, 5])
        self.assertTrue(board.send_stretch(40))
        self.assertEqual(board.sent[-1].hex(), "0b12240000000001282892")

    def test_one_value_per_servo(self):
        board = FakeBoard([2, 5])
        self.assertTrue(board.send_stretch([30, 40], [2, 5]))
        self.assertEqual(board.sent[-1][8:10], bytes([30, 40]))

    def test_no_acknowledgement_raises(self):
        board = FakeBoard([2, 5], reply=b"\x12\x15")
        with self.assertRaises(serial.SerialException):
            board.send_stretch(40)


if __name__ == "__main__":
    unittest.main()
