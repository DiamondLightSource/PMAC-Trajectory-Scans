from PmacCoordinateSystem import PmacCoordinateSystem
import unittest
from pkg_resources import require
require("mock")
from mock import ANY, patch


class InitTest(unittest.TestCase):

    def test_default_attributes_set(self):
        cs_num = 1
        self.pmacCS = PmacCoordinateSystem(cs_num)

        self.assertEqual(self.pmacCS.cs_number, cs_num)
        self.assertEqual(self.pmacCS.motor_map, {})
        self.assertEqual(self.pmacCS.axis_map, {})
        self.assertEqual(self.pmacCS.max_velocities, {'x': 0, 'y': 0, 'z': 0,
                                                      'u': 0, 'v': 0, 'w': 0,
                                                      'a': 0, 'b': 0, 'c': 0})


class AddMotorAssignmentTest(unittest.TestCase):

    def setUp(self):
        self.PmacCS = PmacCoordinateSystem(1)

    def test_given_valid_args_then_set(self):
        motor = 1
        axis = "X"
        scaling = 100

        self.PmacCS.add_motor_assignment(motor, axis, scaling)

        self.assertEqual((axis, scaling), self.PmacCS.motor_map[str(motor)])
        self.assertEqual((motor, scaling), self.PmacCS.axis_map[str(axis)])


class SetMaxVelocitiesTest(unittest.TestCase):

    def setUp(self):
        self.PmacCS = PmacCoordinateSystem(1)

    def test_given_velocities_list_then_set(self):
        velocities = ["10", "20", "30", "40", "50", "60", "70", "80", "90"]
        expected_vel_dict = {'x': velocities[0], 'y': velocities[1], 'z': velocities[2],
                             'u': velocities[3], 'v': velocities[4], 'w': velocities[5],
                             'a': velocities[6], 'b': velocities[7], 'c': velocities[8]}

        self.PmacCS.set_max_velocities(velocities)

        self.assertEqual(expected_vel_dict, self.PmacCS.max_velocities)
