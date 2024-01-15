import time
import unittest

import numpy as np
from numpy import testing

from rcb4.armh7interface import ARMH7Interface


class TestRobotModel(unittest.TestCase):

    interface = None

    @classmethod
    def setUpClass(cls):
        cls.interface = ARMH7Interface()
        cls.interface.auto_open()

    def test_sequentialized_servo_ids(self):
        testing.assert_array_equal(
            self.interface.sequentialized_servo_ids([32, 34]),
            [0, 1])
        testing.assert_array_equal(
            self.interface.sequentialized_servo_ids([34, 32]),
            [1, 0])

    def test_angle_vector_to_servo_angle_vector(self):
        sv = self.interface.angle_vector_to_servo_angle_vector(
            [0, 0])
        testing.assert_array_almost_equal(
            sv, [7500, 7500])

        sv = self.interface.angle_vector_to_servo_angle_vector(
            [30, 60], [32, 34])
        testing.assert_array_almost_equal(
            sv, [8400, 9300])

        sv = self.interface.angle_vector_to_servo_angle_vector(
            [30, 60], [34, 32])
        testing.assert_array_almost_equal(
            sv, [8400, 9300])

    def test_angle_vector(self):
        self.interface.hold()
        self.interface.neutral()
        reference = [60, 30]
        self.interface.angle_vector(
            reference,
            velocity=127)
        time.sleep(4.0)
        testing.assert_array_almost_equal(
            self.interface.angle_vector(), reference,
            decimal=0)
        self.interface.neutral()
        time.sleep(4.0)
        self.interface.free()

    def test_servo_angle_vector(self):
        self.interface.hold()
        self.interface.neutral()
        reference = [8000, 8000]

        self.interface.servo_angle_vector(
            [32, 34],
            reference,
            velocity=1)
        time.sleep(1.0)
        testing.assert_array_almost_equal(
            self.interface.reference_angle_vector(),
            reference)
        time.sleep(4.0)
        if np.any(np.abs(self.interface.angle_vector() - 16) > 2.0):
            self.assertRaises()

        # multi velocities
        reference = [7500, 7500]
        self.interface.servo_angle_vector(
            [32, 34],
            reference,
            velocity=[10, 1])
        time.sleep(1.0)
        testing.assert_array_almost_equal(
            self.interface.reference_angle_vector(),
            reference)
        time.sleep(4.0)
        if np.any(np.abs(self.interface.angle_vector() - 0) > 2.0):
            self.assertRaises()

        self.interface.neutral()
        time.sleep(4.0)
        self.interface.free()
