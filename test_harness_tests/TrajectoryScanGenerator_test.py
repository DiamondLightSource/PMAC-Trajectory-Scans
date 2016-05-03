from test_harness.TrajectoryScanGenerator import TrajectoryScanGenerator
import unittest

from pkg_resources import require
require("mock")
from mock import ANY, patch, MagicMock


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


class ConvertPointsToPmacFloat(unittest.TestCase):

    def setUp(self):
        self.ScanGen = TrajectoryScanGenerator()

    @patch('test_harness.TrajectoryScanGenerator.TrajectoryScanGenerator.double_to_pmac_float')
    def test_given_points_call_convert_function(self, converter_mock):
        points = {'time': [10, 10, 10, 10],
                  'x': [0.0, 0.099861063292, 0.198723793760, 0.295599839129]}

        self.ScanGen.convert_points_to_pmac_float(points)

        self.assertEqual(4, converter_mock.call_count)
        call_list = [call[0][0] for call in converter_mock.call_args_list]
        self.assertEqual(call_list, points['x'])


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
        subroutine = 18
        time = "$10"
        expected_error = "Subroutine must be in range 1 - 16"

        with self.assertRaises(ValueError) as error:
            self.PointGen.set_point_subroutine(time, subroutine)

        self.assertEqual(expected_error, error.exception.message)


class GetBufferOfPointsTest(unittest.TestCase):

    def setUp(self):
        self.PointGen = TrajectoryScanGenerator()
        self.PointGen.point_set = {'y': [1.0, 0.99, 0.98, 0.95, 0.92],
                                   'x': [0.0, 0.10, 0.19, 0.29, 0.38],
                                   'time': [{'time_val': 500, 'subroutine': 0, 'vel_mode': 0}]*5}

    def test_given_no_overflow_then_return_points_grab(self):
        points, _ = self.PointGen.grab_buffer_of_points(0, 4)
        expected_points = {'y': [1.0, 0.99, 0.98, 0.95],
                           'x': [0.0, 0.10, 0.19, 0.29],
                           'time': [{'time_val': 500, 'subroutine': 0, 'vel_mode': 0}]*4}

        self.assertEqual(expected_points, points)

    def test_given_overflow_then_return_points_grab(self):
        points, _ = self.PointGen.grab_buffer_of_points(3, 4)
        expected_points = {'y': [0.95, 0.92, 1.0, 0.99],
                           'x': [0.29, 0.38, 0.0, 0.10],
                           'time': [{'time_val': 500, 'subroutine': 0, 'vel_mode': 0}]*4}

        self.assertEqual(expected_points, points)

    def test_generate_buffer_points(self):
        expected_points = {'y': [0.95, 0.92, 1.0, 0.99, 0.98,
                                 0.95, 0.92, 1.0, 0.99, 0.98, 0.95, 0.92],
                           'x': [0.29, 0.38, 0.0, 0.10, 0.19,
                                 0.29, 0.38, 0.0, 0.10, 0.19, 0.29, 0.38],
                           'time': [{'time_val': 500, 'subroutine': 0, 'vel_mode': 0}]*12}

        points, _ = self.PointGen.generate_buffer_of_points(3, 12)

        self.assertEqual(expected_points, points)


class CheckMaxVelocityTest(unittest.TestCase):

    def setUp(self):
        self.ScanGen = TrajectoryScanGenerator()
        self.PmacCS = MagicMock()
        self.PmacCS.max_velocities = {'x': 1}

    def test_given_valid_points_then_no_error(self):
        self.ScanGen.point_set = {'time': [{'time_val': 10, 'subroutine': 0, 'vel_mode': 0},
                                           {'time_val': 10, 'subroutine': 0, 'vel_mode': 0},
                                           {'time_val': 10, 'subroutine': 0, 'vel_mode': 0}],
                                  'x': [1, 2, 3]}

        response = self.ScanGen.check_max_velocity_of_points(self.PmacCS)

        self.assertTrue(response)

    def test_given_invalid_points_then_error(self):
        self.ScanGen.point_set = {'time': [{'time_val': 1, 'subroutine': 0, 'vel_mode': 0},
                                           {'time_val': 1, 'subroutine': 0, 'vel_mode': 0},
                                           {'time_val': 1, 'subroutine': 0, 'vel_mode': 0}],
                                  'x': [1, 2, 3]}

        expected_error_message = "Points set will exceed maximum velocity, 1, for axis x: 4.0"

        with self.assertRaises(ValueError) as error:
            self.ScanGen.check_max_velocity_of_points(self.PmacCS)

        self.assertEqual(expected_error_message, error.exception.message)


class FormatPointsTest(unittest.TestCase):

    def setUp(self):
        self.ScanGen = TrajectoryScanGenerator()
        self.ScanGen.point_set = {'y': [1.0, 0.99],
                                  'x': [0.0, 0.1],
                                  'time': [{'time_val': 500, 'subroutine': 10, 'vel_mode': 0},
                                           {'time_val': 500, 'subroutine': 0, 'vel_mode': 1}]}

    @patch('test_harness.TrajectoryScanGenerator.TrajectoryScanGenerator.double_to_pmac_float',
           side_effect=['$0', '$1', '$2', '$3'])
    @patch('test_harness.TrajectoryScanGenerator.TrajectoryScanGenerator.set_point_subroutine',
           return_value='$a0001f4')
    @patch('test_harness.TrajectoryScanGenerator.TrajectoryScanGenerator.set_point_vel_mode',
           return_value='$100001f4')
    def test_given_point_set_then_correct_calls_made(self, set_vel_mock, set_sub_mock, d_to_pf_mock):
        expected_point_set = {'time': ['$a0001f4', '$100001f4'],
                              'x': ['$2', '$3'], 'y': ['$0', '$1'], 'u': [],
                              'v': [], 'w': [], 'a': [], 'b': [], 'c': [], 'z': []}

        self.ScanGen.format_point_set()

        call_list = [call[0][0] for call in d_to_pf_mock.call_args_list]
        self.assertEqual([1.0, 0.99, 0.0, 0.1], call_list)
        set_sub_mock.assert_called_once_with('$1f4', 10)
        set_vel_mock.assert_called_once_with('$1f4', 1)
        self.assertEqual(expected_point_set, self.ScanGen.point_set)


class ScanGeneratorTest(unittest.TestCase):

    def setUp(self):
        self.ScanGen = TrajectoryScanGenerator()

    def test_linear_points(self):
        self.ScanGen.generate_linear_points(100, 1, 10)

        expected_points = {'time': [{'subroutine': 0, 'time_val': 100, 'vel_mode': 0}]*10,
                           'a': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]}

        self.assertEqual(expected_points, self.ScanGen.point_set)

    def test_snake_scan(self):
        self.maxDiff = None
        trajectory = {'move_time': 100, 'width': 3, 'length': 3, 'step': 10, 'direction': 0}
        self.ScanGen.generate_snake_scan(trajectory)

        expected_points = {'time': [{'subroutine': 0, 'time_val': 100, 'vel_mode': 0},
                                    {'subroutine': 1, 'time_val': 100, 'vel_mode': 0},
                                    {'subroutine': 0, 'time_val': 100, 'vel_mode': 1},
                                    {'subroutine': 0, 'time_val': 100, 'vel_mode': 2},
                                    {'subroutine': 2, 'time_val': 100, 'vel_mode': 0},
                                    {'subroutine': 0, 'time_val': 100, 'vel_mode': 1},
                                    {'subroutine': 0, 'time_val': 100, 'vel_mode': 2},
                                    {'subroutine': 1, 'time_val': 100, 'vel_mode': 0},
                                    {'subroutine': 0, 'time_val': 100, 'vel_mode': 1}],
                           'x': [0, 10, 20, 20, 10, 0, 0, 10, 20],
                           'y': [0, 0, 0, 10, 10, 10, 20, 20, 20]}

        self.assertEqual(expected_points, self.ScanGen.point_set)

    def test_circle_scan(self):
        self.maxDiff = None
        self.ScanGen.generate_circle_points(100, 10)

        expected_points = {'x': [0.0, -0.23395555690000003, -0.8263518223,
                                 -1.5, -1.9396926208, -1.9396926208, -1.5,
                                 -0.8263518223, -0.23395555690000003, 0.0],
                           'y': [0.0, 0.6427876097, 0.984807753, 0.8660254038,
                                 0.3420201433, -0.3420201433, -0.8660254038,
                                 -0.984807753, -0.6427876097, 0.0],
                           'time': [{'subroutine': 0, 'time_val': 100, 'vel_mode': 0}]*10}

        self.assertEqual(expected_points, self.ScanGen.point_set)

    def test_generate_sine_points(self):
        points = self.ScanGen._generate_sine_points(6)

        expected_points = [0.0, 0.9510565163, 0.5877852523,
                           -0.5877852523, -0.9510565163, -0.0]

        self.assertEqual(expected_points, points)

    @patch('test_harness.TrajectoryScanGenerator.TrajectoryScanGenerator._generate_sine_points',
           return_value=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    def test_generate_sine_scan_one_axis(self, gen_sine_points_mock):
        num_points = 6
        self.ScanGen.generate_sine_points_one_axis(100, num_points)

        expected_points = {'x': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'time': [{'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1}]}

        gen_sine_points_mock.assert_called_once_with(num_points)
        self.assertEqual(expected_points, self.ScanGen.point_set)

    @patch('test_harness.TrajectoryScanGenerator.TrajectoryScanGenerator._generate_sine_points',
           return_value=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    def test_generate_sine_scan_all_axis(self, gen_sine_points_mock):
        num_points = 6
        self.ScanGen.generate_sine_points_all_axes(100, num_points)

        expected_points = {'x': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'y': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'z': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'u': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'v': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'w': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'a': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'b': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'c': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                           'time': [{'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1},
                                    {'time_val': 100, 'subroutine': 0, 'vel_mode': 1}]}

        gen_sine_points_mock.assert_called_once_with(num_points)
        self.assertEqual(expected_points, self.ScanGen.point_set)
