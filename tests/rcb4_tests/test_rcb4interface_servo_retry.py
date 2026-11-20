"""Servo commands survive a refused or lost reply instead of closing the port."""

import unittest

from rcb4.rcb4interface import ReplyTimeout
from rcb4.rcb4interface import request_retries

from .test_rcb4interface_connect import FlakyBoard


class ServoBoard(FlakyBoard):
    """FlakyBoard that acknowledges servo commands as ``[command, 0x06]``."""

    def __init__(self, rate, failures=()):
        super().__init__(rate, failures)
        self.servo_on_states_dict = {2: True, 4: True}

    def serial_write(self, byte_list, timeout=10, close_on_timeout=True):
        reply = super().serial_write(byte_list, timeout, close_on_timeout)
        if list(reply) == [0x00, 0x06]:
            return bytes([byte_list[1], 0x06])
        return reply


class TestServoCommandRetries(unittest.TestCase):

    def test_a_lost_reply_is_sent_again_and_the_port_stays_open(self):
        board = ServoBoard(625000, ["lost", "nak"])
        self.assertEqual(board.servo_angle_vector([2, 4], [7500, 7500]), [2, 4])
        self.assertEqual(len(board.requests), 3)
        self.assertTrue(board.is_opened())

    def test_free_after_a_lost_reply(self):
        board = ServoBoard(625000, ["lost"])
        board.servo_sorted_ids = [2, 4]
        self.assertEqual(board.free(), [2, 4])
        self.assertTrue(board.is_opened())

    def test_gives_up_after_the_retries(self):
        board = ServoBoard(625000, ["lost"] * (request_retries + 1))
        with self.assertRaises(ReplyTimeout):
            board.servo_angle_vector([2], [7500])
        self.assertEqual(len(board.requests), request_retries + 1)


if __name__ == "__main__":
    unittest.main()
