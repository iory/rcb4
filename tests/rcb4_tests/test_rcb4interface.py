import time
import unittest

import numpy as np
from numpy import testing

from rcb4.rcb4interface import RCB4Interface


@unittest.skip("Disable all tests in this class.")
class TestRCB4Interface(unittest.TestCase):
    interface = None

    @classmethod
    def setUpClass(cls):
        cls.interface = RCB4Interface()
        if cls.interface.auto_open() is not True:
            raise unittest.SkipTest(
                f"auto_open failed, skipping all tests in {cls.__class__.__name__}."
            )

    def test_reference_angle_vector(self):
        self.interface.reference_angle_vector([])

    def test_sequentialized_servo_ids(self):
        self.interface.sequentialized_servo_ids([])

        testing.assert_array_equal(
            self.interface.sequentialized_servo_ids([32, 34]), [0, 1]
        )
        testing.assert_array_equal(
            self.interface.sequentialized_servo_ids([34, 32]), [1, 0]
        )

    def test_angle_vector_to_servo_angle_vector(self):
        sv = self.interface.angle_vector_to_servo_angle_vector([0, 0])
        testing.assert_array_almost_equal(sv, [7500, 7500])

        sv = self.interface.angle_vector_to_servo_angle_vector([30, 60], [32, 34])
        testing.assert_array_almost_equal(sv, [8388.888874, 9277.777748])

        sv = self.interface.angle_vector_to_servo_angle_vector([30, 60], [34, 32])
        testing.assert_array_almost_equal(sv, [8388.888874, 9277.777748])

    def test_angle_vector(self):
        self.interface.hold()
        self.interface.neutral()
        time.sleep(4.0)
        reference = [60, 30]
        self.interface.angle_vector(reference, velocity=127)
        time.sleep(4.0)
        testing.assert_array_almost_equal(
            self.interface.angle_vector(), reference, decimal=0
        )
        self.interface.neutral()
        time.sleep(4.0)
        self.interface.free()

        self.interface.angle_vector([], [])

    def test_servo_angle_vector(self):
        self.interface.hold()
        self.interface.neutral()
        reference = [
            8000,
        ]
        target_servo_ids = [
            32,
        ]

        self.interface.servo_angle_vector(target_servo_ids, reference, velocity=1)
        time.sleep(1.0)
        testing.assert_array_almost_equal(
            self.interface.reference_angle_vector(target_servo_ids), reference
        )
        time.sleep(4.0)
        if np.any(
            np.abs(self.interface.angle_vector(servo_ids=target_servo_ids) - 16) > 2.0
        ):
            raise AssertionError("Servo angles are beyond the acceptable range.")

        time.sleep(1.0)
        self.interface.neutral()
        time.sleep(4.0)
        self.interface.free()
