"""rcb4.protocol must build exactly the bytes RCB4Interface sends.

RCB4Interface still assembles its packets itself. Until it is moved onto
rcb4.protocol, this test is what keeps the two from drifting: it captures
what the interface would write to the port and compares it with the
protocol module's packet for the same request, with no hardware attached.
"""

import unittest

from rcb4.protocol import rcb4mini as protocol
from rcb4.rcb4interface import RCB4Interface


class CapturingInterface(RCB4Interface):
    """RCB4Interface whose serial writes are recorded, not sent."""

    def __init__(self):
        super().__init__()
        self.sent = []

    def serial_write(self, byte_list, timeout=10):
        packet = bytes(byte_list)
        self.sent.append(packet)
        # The reply the board gives to a command it accepted: its opcode and ACK.
        return [packet[1], 0x06]


class TestProtocolMatchesInterface(unittest.TestCase):

    def test_ack(self):
        itf = CapturingInterface()
        itf.get_ack()
        self.assertEqual(itf.sent[-1], protocol.frame(protocol.build_ack()))

    def test_version(self):
        itf = CapturingInterface()
        itf.get_version()
        self.assertEqual(itf.sent[-1], protocol.frame(protocol.build_version()))

    def test_ram_read(self):
        for address, size in [(0x0000, 2), (0x0044, 70), (0x0092, 126), (0x1234, 1)]:
            with self.subTest(address=hex(address), size=size):
                _, byte_list = RCB4Interface.move_ram_to_com_command(address, size)
                self.assertEqual(bytes(byte_list),
                                 protocol.frame(protocol.build_ram_read(address, size)))

    def test_ram_write(self):
        for address, data in [(0x0000, [0x93, 0xC3]), (0x0048, [0xB8, 0x00]),
                              (0x0090, [0x01]), (0x0100, list(range(20)))]:
            with self.subTest(address=hex(address), size=len(data)):
                _, byte_list = RCB4Interface.move_com_to_ram_command(address, data)
                self.assertEqual(bytes(byte_list),
                                 protocol.frame(protocol.build_ram_write(address, bytes(data))))

    def test_multi_servo_single_velocity(self):
        ids = [2, 5, 8, 10, 18, 30]
        pulses = [7500, 8000, 7000, 32767, 32768, 11500]
        itf = CapturingInterface()
        itf.servo_on_states_dict = dict.fromkeys(ids, True)
        itf.servo_angle_vector(ids, pulses, velocity=6)
        self.assertEqual(itf.sent[-1],
                         protocol.frame(protocol.build_multi_servo(ids, pulses, 6)))

    def test_multi_servo_unsorted_ids(self):
        ids = [18, 2, 30, 5]
        pulses = [7600, 7400, 8000, 7000]
        itf = CapturingInterface()
        itf.servo_on_states_dict = dict.fromkeys(ids, True)
        itf.servo_angle_vector(ids, pulses, velocity=127)
        self.assertEqual(itf.sent[-1],
                         protocol.frame(protocol.build_multi_servo(ids, pulses, 127)))

    def test_multi_servo_per_servo_velocity(self):
        ids = [2, 5, 8]
        pulses = [7500, 8000, 7000]
        velocities = [10, 20, 30]
        itf = CapturingInterface()
        itf.servo_on_states_dict = dict.fromkeys(ids, True)
        itf.servo_angle_vector(ids, pulses, velocity=velocities)
        self.assertEqual(itf.sent[-1],
                         protocol.frame(protocol.build_multi_servo(ids, pulses, velocities)))

    def test_stretch(self):
        for stretch in [{2: 127}, {2: 40, 5: 60, 30: 1}]:
            with self.subTest(stretch=stretch):
                ids = list(stretch)
                _, byte_list = RCB4Interface().set_servo_parameters_command(
                    [stretch[i] for i in ids], ids)
                self.assertEqual(bytes(byte_list),
                                 protocol.frame(protocol.build_stretch(stretch)))


if __name__ == "__main__":
    unittest.main()
