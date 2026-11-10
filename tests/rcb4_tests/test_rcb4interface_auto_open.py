"""auto_open skips an adapter that answers but is not an RCB-4mini."""

import unittest

from rcb4.rcb4interface import FirmwareVersionError
from rcb4.rcb4interface import RCB4Interface


class TwoAdapters(RCB4Interface):
    """The first adapter is in ICS mode (loopback), the second has a board."""

    def __init__(self):
        super().__init__()
        self.tried = []

    def _candidate_ports(self):
        return ["/dev/ics-mode", "/dev/rcb4"]

    def open(self, port, baudrate, **kwargs):
        self.tried.append((port, baudrate))
        if port == "/dev/ics-mode":
            raise FirmwareVersionError("loopback, not an RCB-4mini")
        return True


class TestAutoOpen(unittest.TestCase):

    def test_an_ics_mode_adapter_is_skipped(self):
        ri = TwoAdapters()
        self.assertTrue(ri.auto_open())
        self.assertEqual(ri.tried[-1][0], "/dev/rcb4")
        # One answer that is not an RCB-4mini is enough: no other rate tried.
        self.assertEqual([p for p, _ in ri.tried].count("/dev/ics-mode"), 1)

    def test_firmware_version_error_is_a_runtime_error(self):
        self.assertTrue(issubclass(FirmwareVersionError, RuntimeError))


if __name__ == "__main__":
    unittest.main()
