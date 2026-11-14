"""Connecting survives a board whose COM link drops or corrupts replies.

One RCB-4mini lost or refused 459 of 1024 ROM reads at 1.25 Mbit/s
(2026-11-14); its ack after a baud-rate switch was lost and the connect gave
up with "No RCB-4 board answered."
"""

import unittest

from rcb4.protocol.framing import ChecksumError
from rcb4.rcb4interface import com_baudrate_codes
from rcb4.rcb4interface import com_baudrate_shift
from rcb4.rcb4interface import ConfigData
from rcb4.rcb4interface import RCB4Interface
from rcb4.rcb4interface import ReplyTimeout
from rcb4.rcb4interface import request_retries
from rcb4.rcb4interface import SubMoveCmd

ack_request = [0x04, 0xFE, 0x06, 0x08]
ack_body = [0xFE, 0x06]
version_body = list(b"\xfdCB-4 V1.0      090715          \xc7")


class FakePort:
    """Stands in for an open serial port; only its baud rate matters."""

    def __init__(self, baudrate):
        self.baudrate = baudrate

    def close(self):
        pass


class FlakyBoard(RCB4Interface):
    """A board at COM rate ``rate`` whose replies fail as ``failures`` says.

    ``failures`` is a list consumed one entry per request: ``None`` answers,
    ``"lost"`` raises ReplyTimeout, ``"corrupt"`` raises ChecksumError and
    ``"nak"`` refuses. A config write switches ``rate`` even when its
    acknowledgement then fails, as the board does.
    """

    def __init__(self, rate, failures=(), config=0x0000):
        super().__init__()
        self.rate = rate
        self.config = config
        self.failures = list(failures)
        self.serial = FakePort(rate)
        self._serial_is_ftdi = False
        self.requests = []

    def _open_serial(self, port, baudrate, timeout):
        return FakePort(baudrate)

    def serial_write(self, byte_list, timeout=10, close_on_timeout=True):
        self.requests.append(list(byte_list))
        if self.serial.baudrate != self.rate:
            if close_on_timeout:
                self.close()
            raise ReplyTimeout("Timeout: No data received.")
        byte_list = list(byte_list)
        failure = self.failures.pop(0) if self.failures else None
        move = [0x00, SubMoveCmd.ComToRam.value]
        if failure != "nak" and byte_list[1:3] == move and byte_list[3:5] == [0, 0]:
            # COM -> RAM write to the config word: the board switches.
            self.config = byte_list[6] | byte_list[7] << 8
            code = (self.config & ConfigData.Baudrate.value) >> com_baudrate_shift
            self.rate = {v: k for k, v in com_baudrate_codes.items()}[code]
        if failure == "lost":
            if close_on_timeout:
                self.close()
            raise ReplyTimeout("Timeout: No data received.")
        if failure == "corrupt":
            raise ChecksumError("corrupted")
        # Bodies as serial_read returns them, after unframe.
        if failure == "nak":
            return bytes([byte_list[1], 0x15])
        if byte_list == ack_request:
            return bytes(ack_body)
        if byte_list[1] == 0xFD:
            return bytes(version_body)
        if byte_list[1:3] == [0x00, SubMoveCmd.RamToCom.value]:
            # RAM -> COM read of the config word.
            return bytes([0x00, self.config & 0xFF, self.config >> 8])
        return bytes([0x00, 0x06])


class TestConnectRetries(unittest.TestCase):

    def test_a_lost_ack_is_sent_again(self):
        board = FlakyBoard(115200, ["lost", "corrupt", "nak"])
        self.assertTrue(board.check_ack())
        self.assertEqual(len(board.requests), 4)

    def test_no_board_gives_up_after_the_retries(self):
        board = FlakyBoard(115200)
        board.serial = FakePort(1250000)
        with self.assertRaises(ReplyTimeout):
            board.check_ack()
        self.assertEqual(len(board.requests), request_retries + 1)

    def test_a_refused_version_request_is_sent_again(self):
        board = FlakyBoard(115200, ["nak"])
        board.check_firmware_version()

    def test_switch_survives_a_lost_acknowledgement(self):
        board = FlakyBoard(115200, ["lost"])
        board._config_data = 0x0000
        self.assertTrue(board.switch_com_baudrate("/dev/null", 625000))
        self.assertEqual(board.rate, 625000)
        self.assertEqual(board.serial.baudrate, 625000)

    def test_switch_survives_a_lost_ack_at_the_new_rate(self):
        # Config write answered, then the first ack at the new rate lost.
        board = FlakyBoard(115200, [None, "lost"])
        board._config_data = 0x0000
        self.assertTrue(board.switch_com_baudrate("/dev/null", 1250000))
        self.assertEqual(board.serial.baudrate, 1250000)

    def test_a_refused_switch_is_sent_again_at_the_old_rate(self):
        board = FlakyBoard(115200, ["nak"])
        board._config_data = 0x0000
        self.assertTrue(board.switch_com_baudrate("/dev/null", 625000))
        self.assertEqual(board.rate, 625000)

    def test_a_board_that_answers_at_neither_rate_fails(self):
        board = FlakyBoard(115200, ["lost"] * 100)
        board._config_data = 0x0000
        self.assertFalse(board.switch_com_baudrate("/dev/null", 625000))


if __name__ == "__main__":
    unittest.main()
