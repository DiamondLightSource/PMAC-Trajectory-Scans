from TrajectoryScanGenerator import TrajectoryScanGenerator
import unittest
from pkg_resources import require
require("mock")
from mock import ANY, patch


class ConvertPointsToPmacFloat(unittest.TestCase):

    def setUp(self):
        self.PointGen = TrajectoryScanGenerator()

    @patch('TrajectoryScanGenerator.TrajectoryScanGenerator.double_to_pmac_float')
    def test_given_points_call_convert_function(self, converter_mock):
        points = {'time': [10, 10, 10, 10],
                  'x': [0.0, 0.099861063292, 0.198723793760, 0.295599839129]}

        self.PointGen.convert_points_to_pmac_float(points)

        self.assertEqual(4, converter_mock.call_count)
        call_list = [call[0][0] for call in converter_mock.call_args_list]
        self.assertEqual(call_list, points['x'])


class DoubleToPmacFloatTest(unittest.TestCase):

    def test_given_zero_then_return_zero(self):

        value = '$0'
        pmac_float = TrajectoryScanGenerator.double_to_pmac_float(0)

        self.assertEqual(pmac_float, value)

    def test_given_positive_then_convert(self):

        value = '$500000000803'
        pmac_float = TrajectoryScanGenerator.double_to_pmac_float(10)

        self.assertEqual(pmac_float, value)

    def test_given_less_than_1_decimal_then_convert(self):

        value = '$4bac6e59b7fe'
        pmac_float = TrajectoryScanGenerator.double_to_pmac_float(0.295599839124)

        self.assertEqual(pmac_float, value)

    def test_given_more_than_1_decimal_then_convert(self):

        value = '$52eb1b910800'
        pmac_float = TrajectoryScanGenerator.double_to_pmac_float(1.2955998341)

        self.assertEqual(pmac_float, value)

    def test_given_negative_then_convert(self):

        value = '$ffaffffffff803'
        pmac_float = TrajectoryScanGenerator.double_to_pmac_float(-10)

        self.assertEqual(pmac_float, value)


class SetPointSpecifiersTest(unittest.TestCase):

    def setUp(self):
        self.PointGen = TrajectoryScanGenerator()

    def test_given_valid_vel_mode_then_set(self):
        vel_mode = 1
        time = "$10"
        expected_new_time = "$10000010"

        new_time = self.PointGen.set_point_vel_mode(time, vel_mode)

        self.assertEqual(expected_new_time, new_time)

    def test_given_invalid_vel_mode_then_error(self):
        vel_mode = 3
        time = "$10"
        expected_error = "Velocity mode must be 0, 1 or 2"

        with self.assertRaises(ValueError) as error:
            self.PointGen.set_point_vel_mode(time, vel_mode)

        self.assertEqual(expected_error, error.exception.message)

    def test_given_valid_subroutine_then_set(self):
        subroutine = 10
        time = "$10"
        expected_new_time = "$a000010"

        new_time = self.PointGen.set_point_subroutine(time, subroutine)

        self.assertEqual(expected_new_time, new_time)

    def test_given_invalid_subroutine_then_error(self):
        subroutine = 3
        time = "$10"
        expected_error = "Subroutine must be in range 10 - 16"

        with self.assertRaises(ValueError) as error:
            self.PointGen.set_point_subroutine(time, subroutine)

        self.assertEqual(expected_error, error.exception.message)
