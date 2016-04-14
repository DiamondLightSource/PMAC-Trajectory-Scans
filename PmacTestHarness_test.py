from PmacTestHarness import PmacTestHarness
from PmacCoordinateSystem import PmacCoordinateSystem
import unittest
from pkg_resources import require
require("mock")
from mock import ANY, patch


class TesterPmacTestHarness(PmacTestHarness):

    def __init__(self):

        self.status = "1"
        self.status = "0"
        self.total_points = 0
        self.current_index = 0
        self.current_buffer = 0
        self.buffer_length = "50"
        self.buffer_address_a = "30000"
        self.buffer_address_b = "30226"
        self.addresses = {}
        self.coordinate_system = PmacCoordinateSystem(1)

        self.prev_buffer_write = 1


class InitTest(unittest.TestCase):

    @patch('PmacTestHarness.PmacTestHarness.read_variable')
    def test_default_attributes_set(self, read_variable_mock):
        self.pmac = PmacTestHarness("test")

        self.assertEqual(read_variable_mock.call_args_list[0][0][0], "P4001")
        self.assertEqual(read_variable_mock.call_args_list[1][0][0], "P4015")
        self.assertEqual(read_variable_mock.call_args_list[2][0][0], "P4004")
        self.assertEqual(read_variable_mock.call_args_list[3][0][0], "P4008")
        self.assertEqual(read_variable_mock.call_args_list[4][0][0], "P4009")


@patch('PmacTestHarness_test.TesterPmacTestHarness.read_variable', return_value="0")
class UpdateStatusVariablesTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_status_updated(self, read_variable_mock):
        read_variable_mock.return_value = "1"

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.status, 1)
        self.assertEqual(read_variable_mock.call_args_list[0][0][0], "P4001")

    def test_error_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.total_points, 0)
        self.assertEqual(read_variable_mock.call_args_list[1][0][0], "P4015")

    def test_total_points_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.total_points, 0)
        self.assertEqual(read_variable_mock.call_args_list[2][0][0], "P4005")

    def test_current_index_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.current_index, 0)
        self.assertEqual(read_variable_mock.call_args_list[3][0][0], "P4006")

    def test_current_buffer_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.current_buffer, 0)
        self.assertEqual(read_variable_mock.call_args_list[4][0][0], "P4007")


class UpdateAddressesTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_given_buffer_A_then_update(self):
        expected_address = {'a': '3015e',
                            'b': '30190',
                            'c': '301c2',
                            'time': '30000',
                            'u': '300c8',
                            'v': '300fa',
                            'w': '3012c',
                            'x': '30032',
                            'y': '30064',
                            'z': '30096'}

        self.pmac.update_address_dict(self.pmac.buffer_address_a)

        self.assertEqual(expected_address, self.pmac.addresses)

    def test_given_buffer_B_then_update(self):
        expected_address = {'a': '30384',
                            'b': '303b6',
                            'c': '303e8',
                            'time': '30226',
                            'u': '302ee',
                            'v': '30320',
                            'w': '30352',
                            'x': '30258',
                            'y': '3028a',
                            'z': '302bc'}

        self.pmac.update_address_dict(self.pmac.buffer_address_b)

        self.assertEqual(expected_address, self.pmac.addresses)


class UpdateVelocitiesTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_variable',
           side_effect=['10', '20', '30', '40', '50', '60', '70', '80', '90'])
    @patch('PmacCoordinateSystem.PmacCoordinateSystem.set_max_velocities')
    def test_update_velocities(self, set_max_vel_mock, _):
        expected_call = ['10', '20', '30', '40', '50', '60', '70', '80', '90']

        self.pmac.read_cs_max_velocities()

        set_max_vel_mock.assert_called_once_with(expected_call)


@patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
class CommandsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacCoordinateSystem.PmacCoordinateSystem.add_motor_assignment')
    def test_assign_motors_command(self, add_motor_mock, send_command_mock):
        axis_map = [(1, "X", 100), (3, "Y", 25)]

        self.pmac.assign_motors(axis_map)

        call_list = [call[0] for call in add_motor_mock.call_args_list]
        self.assertIn(axis_map[0], call_list)
        self.assertIn(axis_map[1], call_list)
        send_command_mock.assert_called_once_with("&1 #1->100X #3->25Y")

    def test_home_motors_command(self, send_command_mock):
        self.pmac.coordinate_system.add_motor_assignment(1, "X", 1)
        self.pmac.coordinate_system.add_motor_assignment(2, "Y", 1)
        self.pmac.home_motors()

        send_command_mock.assert_called_once_with(
            "#1HMZ#2HMZ")

    def test_run_motion_program_command(self, send_command_mock):
        self.pmac.coordinate_system.add_motor_assignment(1, "X", 1)
        self.pmac.coordinate_system.add_motor_assignment(2, "Y", 1)

        self.pmac.run_motion_program(1)

        send_command_mock.assert_called_once_with(
            "#1J/#2J/&1B1R")

    def test_abort_command(self, send_command_mock):

        self.pmac.force_abort()

        send_command_mock.assert_called_once_with("A")

    def test_read_motor_position_command(self, send_command_mock):

        self.pmac.read_motor_position(1)

        send_command_mock.assert_called_once_with("#1P")

    def test_read_motor_velocity_command(self, send_command_mock):

        self.pmac.read_motor_velocity(1)

        send_command_mock.assert_called_once_with("#1V")


class SetCurrentCoordinatesTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_motor_position',
           side_effect=["10", "20"])
    @patch('PmacTestHarness_test.TesterPmacTestHarness.set_variable')
    def test_set_initial_coordinates_makes_correct_calls(self, set_variable_mock, _):
        self.pmac.coordinate_system.motor_map = {"1": ("X", 50), "2": ("Y", 20)}

        self.pmac.set_initial_coordinates()

        call_list = [call[0] for call in set_variable_mock.call_args_list]
        self.assertIn(("P4111", "500"), call_list)
        self.assertIn(("P4112", "400"), call_list)


# class CheckProgramExistsTest(unittest.TestCase):
#
#     def setUp(self):
#         self.pmac = TesterPmacTestHarness()
#
#     def test_check_program_exists(self):
#
#         exists = self.pmac.check_program_exists()
#
#         self.assertTrue(exists)


class SetAxesTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.set_variable')
    def test_axes_set(self, set_variable_mock):
        self.pmac.set_axes(510)

        set_variable_mock.assert_called_once_with("P4003", "510")


class SetAbortTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.set_variable')
    def test_abort_set(self, set_variable_mock):
        self.pmac.set_abort()

        set_variable_mock.assert_called_once_with("P4002", "1")


@patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
class ReadWriteTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_read_address_given_valid_args_return_value(self, send_command_mock):
        send_command_mock.return_value = ("1\r", True)
        value = self.pmac.read_address("X", "30000")

        send_command_mock.assert_called_once_with("RX $30000")
        self.assertEqual(value, "1")

    def test_read_address_given_invalid_args_then_raise_error(self, send_command_mock):
        send_command_mock.return_value = ("1\r", False)

        with self.assertRaises(IOError) as error:
            self.pmac.read_address("U", "1000000")

        send_command_mock.assert_called_once_with("RU $1000000")
        self.assertEqual(error.exception.message, "Read failed")

    def test_write_address_given_valid_args_then_success(self, send_command_mock):
        send_command_mock.return_value = ("1\r", True)
        success = self.pmac.write_to_address("X", "30000", "100")

        send_command_mock.assert_called_once_with("WX $30000 100")
        self.assertTrue(success)

    def test_write_address_given_invalid_args_then_raise_error(self, send_command_mock):
        send_command_mock.return_value = ("1\r", False)

        with self.assertRaises(IOError) as error:
            self.pmac.write_to_address("U", "1000000", "")

        send_command_mock.assert_called_once_with("WU $1000000 ")
        self.assertEqual(error.exception.message, "Write failed")

    def test_read_variable_given_valid_args_return_value(self, send_command_mock):
        send_command_mock.return_value = ("1\r", True)
        value = self.pmac.read_variable("P4001")

        send_command_mock.assert_called_once_with("P4001")
        self.assertEqual(value, "1")

    def test_read_variable_given_invalid_args_then_raise_error(self, send_command_mock):
        send_command_mock.return_value = ("1\r", False)

        with self.assertRaises(IOError) as error:
            self.pmac.read_variable("P10000")

        send_command_mock.assert_called_once_with("P10000")
        self.assertEqual(error.exception.message, "Read failed")

    def test_set_variable_given_valid_args_success(self, send_command_mock):
        send_command_mock.return_value = ("1\r", True)
        success = self.pmac.set_variable("P4002", "1")

        send_command_mock.assert_called_once_with("P4002=1")
        self.assertTrue(success)

    def test_set_variable_given_invalid_args_then_raise_error(self, send_command_mock):
        send_command_mock.return_value = ("1\r", False)

        with self.assertRaises(IOError) as error:
            self.pmac.set_variable("P10000", "1")

        send_command_mock.assert_called_once_with("P10000=1")
        self.assertEqual(error.exception.message, "Write failed")


class ResetBuffersTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()
        self.pmac.buffer_length = 5

    @patch('PmacTestHarness_test.TesterPmacTestHarness.fill_current_buffer')
    @patch('PmacTestHarness_test.TesterPmacTestHarness.fill_idle_buffer')
    def test_buffers_set_to_zero(self, idle_mock, current_mock):
        self.pmac.reset_buffers()
        expected_call = {'time': ['$0', '$0', '$0', '$0', '$0'],
                         'x': ['$0', '$0', '$0', '$0', '$0'],
                         'y': ['$0', '$0', '$0', '$0', '$0'],
                         'z': ['$0', '$0', '$0', '$0', '$0'],
                         'u': ['$0', '$0', '$0', '$0', '$0'],
                         'v': ['$0', '$0', '$0', '$0', '$0'],
                         'w': ['$0', '$0', '$0', '$0', '$0'],
                         'a': ['$0', '$0', '$0', '$0', '$0'],
                         'b': ['$0', '$0', '$0', '$0', '$0'],
                         'c': ['$0', '$0', '$0', '$0', '$0']}

        current_mock.assert_called_once_with(expected_call)
        idle_mock.assert_called_once_with(expected_call)


class ConstructWriteCommandTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_given_points_then_construct_message(self):
        command_details = {'points': ['$f']*98,
                           'mode': 'L', 'address': '30386'}
        expected_response = {'points': ['$f']*16,
                             'command': 'WL$30386,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,$f,'
                                        '$f', 'num_sent': 82}

        response = self.pmac._construct_write_command_and_remove_used_points(command_details)

        self.assertEqual(expected_response, response)


@patch('PmacTestHarness_test.TesterPmacTestHarness._fill_buffer')
@patch('PmacTestHarness_test.TesterPmacTestHarness.update_address_dict')
class FillIdleBufferTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()
        self.points = {'time': [10, 10, 10, 10],
                       'x': [10, 10, 10, 10]}

    def test_given_a_then_update_b_and_fill_buffer(self, update_mock, fill_mock):
        self.pmac.current_buffer = 0

        self.pmac.fill_idle_buffer(self.points)

        update_mock.assert_called_once_with(self.pmac.buffer_address_b)
        fill_mock.assert_called_once_with(self.points)

    def test_given_b_then_update_a_and_fill_buffer(self, update_mock, fill_mock):
        self.pmac.current_buffer = 1

        self.pmac.fill_idle_buffer(self.points)

        update_mock.assert_called_once_with(self.pmac.buffer_address_a)
        fill_mock.assert_called_once_with(self.points)


@patch('PmacTestHarness_test.TesterPmacTestHarness._fill_buffer')
@patch('PmacTestHarness_test.TesterPmacTestHarness.update_address_dict')
class FillCurrentBufferTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()
        self.points = {'time': [10, 10, 10, 10],
                       'x': [10, 10, 10, 10]}

    def test_given_a_then_update_a_and_fill_buffer(self, update_mock, fill_mock):
        self.pmac.current_buffer = 0

        self.pmac.fill_current_buffer(self.points)

        update_mock.assert_called_once_with(self.pmac.buffer_address_a)
        fill_mock.assert_called_once_with(self.points)

    def test_given_b_then_update_b_and_fill_buffer(self, update_mock, fill_mock):
        self.pmac.current_buffer = 1

        self.pmac.fill_current_buffer(self.points)

        update_mock.assert_called_once_with(self.pmac.buffer_address_b)
        fill_mock.assert_called_once_with(self.points)


class FillBufferTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_given_points_longer_than_buffer_length_then_error(self):
        points = {'time': [10]*51, 'x': [10]*51}
        expected_error_message = \
            "Point set cannot be longer than PMAC buffer length"

        with self.assertRaises(ValueError) as error:
            self.pmac._fill_buffer(points)

        self.assertEqual(expected_error_message, error.exception.message)

    def test_given_different_length_axes_then_error(self):
        points = {'time': [10]*50, 'x': [10]*51}
        expected_error_message = \
            "Point set must have equal points in all axes"

        with self.assertRaises(ValueError) as error:
            self.pmac._fill_buffer(points)

        self.assertEqual(expected_error_message, error.exception.message)

    @patch('PmacTestHarness_test.TesterPmacTestHarness.'
           '_construct_write_command_and_remove_used_points')
    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
    def test_given_valid_points_then_fill_buffer(self, send_mock, construct_mock):
        self.pmac.addresses = {'time': '30000', 'x': '30032'}
        points = {'time': ['10']*50, 'x': ['20']*50}
        time_construct_call = {'mode': 'L', 'address': '30000', 'points': points['time']}
        x_construct_call = {'mode': 'L', 'address': '30032', 'points': points['x']}
        time_cmd = 'WL$30000,10,10,10,10,10,10,10,10,10,10,10,10,10,10,' \
                   '10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,' \
                   '10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10'
        x_cmd = 'WL$30032,20,20,20,20,20,20,20,20,20,20,20,20,20,20,' \
                '20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,' \
                '20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20'
        construct_mock.side_effect = [{'command': time_cmd, 'points': [], 'num_sent': 0},
                                      {'command': x_cmd, 'points': [], 'num_sent': 0}]

        self.pmac._fill_buffer(points)

        construct_calls = [call[0][0] for call in construct_mock.call_args_list]
        send_calls = [call[0][0] for call in send_mock.call_args_list]

        self.assertIn(x_construct_call, construct_calls)
        self.assertIn(time_construct_call, construct_calls)
        self.assertIn(x_cmd, send_calls)
        self.assertIn(time_cmd, send_calls)


@patch('PmacTestHarness_test.TesterPmacTestHarness.read_address')
class ReadPointsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_given_buffer_A_then_read(self, read_address_mock):
        read_address_mock.side_effect = [100, 100, 100, 200, 200, 200,
                                         300, 300, 300, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0]
        pmac_buffer = self.pmac.read_points(3, buffer_num=0, num_axes=2)

        points = [100, 100, 100,
                  200, 200, 200,
                  300, 300, 300]

        self.assertEqual(pmac_buffer[:9], points)

    def test_given_buffer_B_then_read(self, read_address_mock):
        read_address_mock.side_effect = [100, 100, 100, 200, 200, 200,
                                         300, 300, 300, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0]
        pmac_buffer = self.pmac.read_points(3, buffer_num=1, num_axes=2)

        points = [100, 100, 100,
                  200, 200, 200,
                  300, 300, 300]

        self.assertEqual(pmac_buffer[:9], points)


@patch('PmacTestHarness_test.TesterPmacTestHarness.set_variable')
class SetBufferFillTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    def test_current_given_A_active_then_A_buffer_set(self, set_variable_mock):
        self.pmac.current_buffer = 0
        self.pmac.set_current_buffer_fill(50)

        set_variable_mock.assert_called_once_with("P4011", "50")

    def test_current_given_B_active_then_B_buffer_set(self, set_variable_mock):
        self.pmac.current_buffer = 1
        self.pmac.set_current_buffer_fill(50)

        set_variable_mock.assert_called_once_with("P4012", "50")

    def test_idle_given_A_active_then_B_buffer_set(self, set_variable_mock):
        self.pmac.current_buffer = 0
        self.pmac.set_idle_buffer_fill(50)

        set_variable_mock.assert_called_once_with("P4012", "50")

    def test_current_given_B_active_then_A_buffer_set(self, set_variable_mock):
        self.pmac.current_buffer = 1
        self.pmac.set_idle_buffer_fill(50)

        set_variable_mock.assert_called_once_with("P4011", "50")


class DecHexConverterTest(unittest.TestCase):

    def test_add_hex(self):

        sum_ = PmacTestHarness.add_hex("A", "E")

        self.assertEqual(sum_, "18")

    def test_add_dechex(self):

        sum_ = PmacTestHarness.add_dechex("9", 1)

        self.assertEqual(sum_, "a")

        sum_ = PmacTestHarness.add_dechex("9", 6)

        self.assertEqual(sum_, "f")

        sum_ = PmacTestHarness.add_dechex("9", 7)

        self.assertEqual(sum_, "10")

    def test_inc_hex(self):

        hex_ = "0"
        series = []

        for i in range(0, 27):
            series.append(hex_)
            hex_ = PmacTestHarness.inc_hex(hex_)

        self.assertEqual(series, ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
                                  "a", "b", "c", "d", "e", "f", "10", "11", "12",
                                  "13", "14", "15", "16", "17", "18", "19", "1a"])
