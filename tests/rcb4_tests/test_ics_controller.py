"""ICSServoController against a simulated servo on a half-duplex line.

The servo below is written from the ICS3.5/3.6 manual, not from
rcb4.protocol.ics, so the controller and its protocol module are checked
against an independent reading of the reference.
"""

import unittest

from rcb4.ics import ICSServoController
from rcb4.ics import KeyListener
from rcb4.protocol import ics as protocol


class SimulatedServo:
    """A port with one ICS servo on it. Every write is looped back first."""

    def __init__(self, servo_id=3):
        self.servo_id = servo_id
        self.position = 7500
        self.values = {1: 60, 2: 127, 3: 5, 4: 70}
        self.eeprom = [5, 10] + [1] * 62
        self.pending = b""
        self.is_open = True
        self.baudrate = 1250000
        self.timeout = 0.1
        self.corrupt_reply = False

    def reset_input_buffer(self):
        self.pending = b""

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def _answer(self, tx):
        head, sid = tx[0] & 0xE0, tx[0] & 0x1F
        if head == 0xE0:
            if tx[0] == 0xFF:
                return bytes([0xE0 | self.servo_id])
            self.servo_id = sid
            return bytes([tx[0]])
        if sid != self.servo_id:
            return b""
        if head == 0x80:
            pulse = (tx[1] << 7) | tx[2]
            if pulse:
                self.position = pulse
            return bytes([sid, self.position >> 7, self.position & 0x7F])
        if head == 0xA0:
            if tx[1] == 0:
                return bytes([0x20 | sid, 0]) + bytes(self.eeprom)
            if tx[1] == 5:
                return bytes([0x20 | sid, 5, self.position >> 7,
                              self.position & 0x7F])
            return bytes([0x20 | sid, tx[1], self.values[tx[1]]])
        if head == 0xC0:
            if tx[1] == 0:
                self.eeprom = list(tx[2:])
                return bytes([0x40 | sid, 0])
            self.values[tx[1]] = tx[2]
            return bytes([0x40 | sid, tx[1], tx[2]])
        return b""

    def write(self, data):
        reply = self._answer(bytes(data))
        if self.corrupt_reply and reply:
            reply = bytes([reply[0] ^ 0x01]) + reply[1:]
        self.pending = bytes(data) + reply

    def read(self, size=1):
        chunk, self.pending = self.pending[:size], self.pending[size:]
        return chunk


def controller_with(servo):
    controller = ICSServoController()
    controller.ics = servo
    return controller


class TestICSServoController(unittest.TestCase):

    def setUp(self):
        self.servo = SimulatedServo(servo_id=3)
        self.controller = controller_with(self.servo)

    def test_id_and_parameters(self):
        self.assertEqual(self.controller.get_servo_id(), 3)
        self.assertEqual(self.controller.get_stretch(), 60)
        self.assertEqual(self.controller.get_speed(), 127)
        self.assertEqual(self.controller.set_speed(100), 100)
        self.assertEqual(self.controller.get_speed(), 100)
        self.assertEqual(self.controller.set_stretch(30), 30)
        self.assertEqual(self.servo.values[1], 30)

    def test_angles(self):
        self.assertEqual(self.controller.set_angle(8000), 8000)
        self.assertEqual(self.controller.read_angle(), 8000)
        self.assertEqual(self.controller.set_free(True), 8000)

    def test_eeprom_round_trip(self):
        params, _ = self.controller.read_param()
        self.assertEqual(params, self.servo.eeprom)
        params[4] = 2
        self.controller.set_param(params)
        self.assertEqual(self.servo.eeprom[4], 2)

    def test_set_servo_id(self):
        self.assertEqual(self.controller.set_servo_id(7), 7)
        self.assertEqual(self.controller.get_servo_id(), 7)

    def test_a_corrupted_reply_is_refused(self):
        self.controller.get_servo_id()
        self.servo.corrupt_reply = True
        with self.assertRaises(protocol.IcsReplyError):
            self.controller.get_stretch(servo_id=3)

    def test_no_reply_is_an_os_error(self):
        with self.assertRaises(OSError):
            self.controller.get_stretch(servo_id=4)

    def test_default_eeprom_keeps_calibration_baud_and_id(self):
        calibration = [9] * protocol.EEPROM_BYTES
        self.servo.eeprom = list(calibration)
        self.controller.set_default_eeprom_param(servo_id=3)
        written = self.servo.eeprom
        for index in list(protocol.EEPROM_PROTECTED) + [26, 27, 56, 57]:
            self.assertEqual(written[index], 9, index)
        self.assertEqual(written[4:6], [7, 15])         # speed 127, a default

    def test_unfree_holds_where_the_servo_is_without_an_eeprom_write(self):
        self.servo.position = 6000
        eeprom = list(self.servo.eeprom)
        writes = []
        original = self.servo.write
        self.servo.write = lambda data: (writes.append(bytes(data)),
                                         original(data))
        self.assertEqual(self.controller.set_free(False, servo_id=3), 6000)
        self.assertEqual(self.servo.eeprom, eeprom)
        self.assertEqual([w[0] for w in writes], [0x83, 0x83])
        self.assertEqual(protocol.decode_position(*writes[-1][1:]), 6000)

    def test_rotation_mode_leaves_the_servo_free(self):
        self.servo.position = 6000
        self.controller.set_rotation(True, servo_id=3)
        self.assertTrue(self.controller.is_continuous_rotation_mode)
        self.assertEqual(self.servo.eeprom[14] & 1, 1)
        self.controller.set_rotation(False, servo_id=3)
        self.assertEqual(self.servo.eeprom[14] & 1, 0)

    def test_move_and_wait_returns_the_reached_position(self):
        self.assertEqual(self.controller.move_and_wait(8200, servo_id=3), 8200)


class TestKeyListener(unittest.TestCase):

    def test_keys_are_handed_over_in_order(self):
        listener = KeyListener()
        for key in ("a", "b", "c"):
            listener.keys.put(key)
        self.assertEqual([listener.get_key() for _ in range(4)],
                         ["a", "b", "c", None])


if __name__ == "__main__":
    unittest.main()
