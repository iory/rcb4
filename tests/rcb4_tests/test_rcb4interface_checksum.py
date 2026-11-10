"""RCB4Interface.serial_read checks the checksum of every reply."""

import unittest

from rcb4.protocol import framing
from rcb4.rcb4interface import RCB4Interface


class ScriptedPort:
    """A port that answers every write with one fixed reply."""

    def __init__(self, reply):
        self.reply = bytes(reply)
        self.pending = b""

    @property
    def in_waiting(self):
        return len(self.pending)

    def flushInput(self):
        self.pending = b""

    def write(self, data):
        self.pending = self.reply

    def read(self, size=1):
        chunk, self.pending = self.pending[:size], self.pending[size:]
        return chunk

    def close(self):
        pass


def interface_answering(reply):
    ri = RCB4Interface()
    ri.serial = ScriptedPort(reply)
    # Polled like a pyftdi port: no file descriptor for select().
    ri._serial_is_ftdi = True
    return ri


class TestReplyChecksum(unittest.TestCase):

    def test_a_good_reply_returns_its_body(self):
        ri = interface_answering(framing.frame(bytes([0xFE, 0x06])))
        self.assertEqual(ri.serial_write([0x04, 0xFE, 0x06, 0x08], timeout=1.0),
                         bytes([0xFE, 0x06]))
        self.assertTrue(ri.check_ack(timeout=1.0))

    def test_a_corrupted_reply_raises_checksum_error(self):
        packet = bytearray(framing.frame(bytes([0xFE, 0x06])))
        packet[2] ^= 0x40            # body corrupted, checksum byte intact
        ri = interface_answering(packet)
        with self.assertRaises(framing.ChecksumError):
            ri.check_ack(timeout=1.0)

    def test_the_recorded_version_reply_passes(self):
        """The version reply recorded from an RCB-4mini, framed as it arrives."""
        body = b"\xfdCB-4 V1.0      090715          \xc7"
        ri = interface_answering(framing.frame(body))
        self.assertEqual(ri.get_version(), body)


if __name__ == "__main__":
    unittest.main()
