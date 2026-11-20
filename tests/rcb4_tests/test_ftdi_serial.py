"""open_ftdi_serial sets the FTDI latency timer."""

import unittest
from unittest import mock

from rcb4 import ftdi_serial


class FakeFtdiPort:

    def __init__(self):
        self.udev = None

    def open(self):
        self.udev = mock.Mock()


@unittest.skipUnless(ftdi_serial.PYFTDI_AVAILABLE, "pyftdi not installed")
class TestLatencyTimer(unittest.TestCase):

    def _open(self, **kwargs):
        port = FakeFtdiPort()
        with mock.patch.object(ftdi_serial, "serial_for_url", return_value=port):
            return ftdi_serial.open_ftdi_serial("ftdi://x/1", 625000, 0.01, **kwargs)

    def test_default_is_one_ms(self):
        self._open().udev.set_latency_timer.assert_called_once_with(1)

    def test_none_leaves_the_chip_setting(self):
        self._open(latency_timer_ms=None).udev.set_latency_timer.assert_not_called()


if __name__ == "__main__":
    unittest.main()
