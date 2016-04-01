from PmacTestHarness import PmacTestHarness
import unittest
from pkg_resources import require
require("mock")
from mock import ANY, patch


class TesterPmacTestHarness(PmacTestHarness):

    def __init__(self):

        self.status = "1"
        self.total_points = 0
        self.current_index = 0
        self.current_buffer = 0
        self.buffer_length = "50"
        self.buffer_address_a = "30000"
        self.buffer_address_b = "30226"
        self.addresses = {}

        self.prev_buffer_write = 1


class InitTest(unittest.TestCase):

    @patch('PmacTestHarness.PmacTestHarness.read_variable')
    def test_default_attributes_set(self, read_variable_mock):
        self.pmac = PmacTestHarness("test")

        self.assertEqual(read_variable_mock.call_args_list[0][0][0], "P4001")
        self.assertEqual(read_variable_mock.call_args_list[1][0][0], "P4004")
        self.assertEqual(read_variable_mock.call_args_list[2][0][0], "P4008")
        self.assertEqual(read_variable_mock.call_args_list[3][0][0], "P4009")


class UpdateStatusVariablesTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_variable', return_value="1")
    def test_status_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.status, 1)
        self.assertEqual(read_variable_mock.call_args_list[0][0][0], "P4001")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_variable', return_value="0")
    def test_total_points_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.total_points, 0)
        self.assertEqual(read_variable_mock.call_args_list[1][0][0], "P4005")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_variable', return_value="0")
    def test_current_index_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.current_index, 0)
        self.assertEqual(read_variable_mock.call_args_list[2][0][0], "P4006")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_variable', return_value="0")
    def test_current_buffer_updated(self, read_variable_mock):

        self.pmac.update_status_variables()
        self.assertEqual(self.pmac.current_buffer, 0)
        self.assertEqual(read_variable_mock.call_args_list[3][0][0], "P4007")


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


class CommandsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
    def test_assign_motors_command(self, send_command_mock):

        self.pmac.assign_motors()

        send_command_mock.assert_called_once_with(
            "&1 #1->100X #2->100Y #3->Z #4->U #5->V #6->W #7->A #8->B")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
    def test_home_motors_command(self, send_command_mock):

        self.pmac.home_motors()

        send_command_mock.assert_called_once_with(
            "#1HMZ #2HMZ #3HMZ #4HMZ #5HMZ #6HMZ #7HMZ #8HMZ #9HMZ")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
    def test_run_motion_program_command(self, send_command_mock):

        self.pmac.run_motion_program(1)

        send_command_mock.assert_called_once_with(
            "#1J/ #2J/ #3J/ #4J/ #5J/ #6J/ #7J/ #8J/ &1 B1 R")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand')
    def test_abort_command(self, send_command_mock):

        self.pmac.force_abort()

        send_command_mock.assert_called_once_with("A")


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
    def test_axes_set(self, set_variable_mock):
        self.pmac.set_abort()

        set_variable_mock.assert_called_once_with("P4002", "1")


class ReadAddressTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("1\r", True))
    def test_given_valid_args_return_value(self, send_command_mock):
        value = self.pmac.read_address("X", "30000")

        send_command_mock.assert_called_once_with("RX $30000")
        self.assertEqual(value, "1")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("\r", False))
    def test_given_invalid_args_then_raise_error(self, send_command_mock):

        with self.assertRaises(IOError) as error:
            self.pmac.read_address("U", "1000000")

        send_command_mock.assert_called_once_with("RU $1000000")
        self.assertEqual(error.exception.message, "Read failed")


class WriteToAddressTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("1\r", True))
    def test_given_valid_args_then_success(self, send_command_mock):
        success = self.pmac.write_to_address("X", "30000", "100")

        send_command_mock.assert_called_once_with("WX $30000 100")
        self.assertTrue(success)

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("\r", False))
    def test_given_invalid_args_then_raise_error(self, send_command_mock):
        with self.assertRaises(IOError) as error:
            self.pmac.write_to_address("U", "1000000", "")

        send_command_mock.assert_called_once_with("WU $1000000 ")
        self.assertEqual(error.exception.message, "Write failed")


class ReadVariableTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("1\r", True))
    def test_given_valid_args_return_value(self, send_command_mock):
        value = self.pmac.read_variable("P4001")

        send_command_mock.assert_called_once_with("P4001")
        self.assertEqual(value, "1")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("\r", False))
    def test_given_invalid_args_then_raise_error(self, send_command_mock):

        with self.assertRaises(IOError) as error:
            self.pmac.read_variable("P10000")

        send_command_mock.assert_called_once_with("P10000")
        self.assertEqual(error.exception.message, "Read failed")


class SetVariableTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("1\r", True))
    def test_given_valid_args_success(self, send_command_mock):
        success = self.pmac.set_variable("P4002", "1")

        send_command_mock.assert_called_once_with("P4002=1")
        self.assertTrue(success)

    @patch('PmacTestHarness_test.TesterPmacTestHarness.sendCommand', return_value=("\r", False))
    def test_given_invalid_args_then_raise_error(self, send_command_mock):

        with self.assertRaises(IOError) as error:
            self.pmac.set_variable("P10000", "1")

        send_command_mock.assert_called_once_with("P10000=1")
        self.assertEqual(error.exception.message, "Write failed")


class ResetBuffersTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()
        self.num_points = int(self.pmac.buffer_length)*10*2

    @patch('PmacTestHarness_test.TesterPmacTestHarness.write_to_address')
    def test_buffers_set_to_zero(self, write_mock):
        self.pmac.reset_buffers()

        self.assertEqual(write_mock.call_args_list[0][0], ("L", "30000", "0"))
        self.assertEqual(write_mock.call_args_list[-1][0], ("L", ANY, "0"))
        self.assertEqual(write_mock.call_count, self.num_points)


class ConvertPointsToPmacFloat(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.double_to_pmac_float')
    def test_given_points_call_convert_function(self, converter_mock):
        points = {'time': [10, 10, 10, 10],
                  'x': [0.0, 0.099861063292, 0.198723793760, 0.295599839129]}

        self.pmac.convert_points_to_pmac_float(points)

        self.assertEqual(4, converter_mock.call_count)
        call_list = [call[0][0] for call in converter_mock.call_args_list]
        self.assertEqual(call_list, points['x'])


class SendPointsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()
        self.points = {'time': [100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
                       'x': [200, 200, 200, 200, 200, 200, 200, 200, 200, 200],
                       'y': [300, 300, 300, 300, 300, 300, 300, 300, 300, 300]}

    @patch('PmacTestHarness_test.TesterPmacTestHarness.write_to_address')
    def test_given_current_True_then_points_sent_to_A(self, write_mock):

        self.pmac.send_points(self.points, current=True)
        root_address = self.pmac.buffer_address_a

        call_list = [call[0] for call in write_mock.call_args_list]
        self.assertEqual(write_mock.call_count, 30)
        self.assertIn(("L", root_address, "100"), call_list)
        self.assertIn(
            ("L", self.pmac.add_dechex(root_address, int(self.pmac.buffer_length)), "200"),call_list)
        self.assertIn(
            ("L", self.pmac.add_dechex(root_address, 2*int(self.pmac.buffer_length)), "300"), call_list)

    @patch('PmacTestHarness_test.TesterPmacTestHarness.write_to_address')
    def test_given_current_False_points_sent_to_B(self, write_mock):

        self.pmac.send_points(self.points, current=False)
        root_address = self.pmac.buffer_address_b

        call_list = [call[0] for call in write_mock.call_args_list]
        self.assertEqual(write_mock.call_count, 30)
        self.assertIn(("L", root_address, "100"), call_list)
        self.assertIn(
            ("L", self.pmac.add_dechex(root_address, int(self.pmac.buffer_length)), "200"), call_list)
        self.assertIn(
            ("L", self.pmac.add_dechex(root_address, 2*int(self.pmac.buffer_length)), "300"), call_list)


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


class ReadPointsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_address',
           side_effect=[100, 100, 100, 200, 200, 200, 300, 300, 300,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0])
    def test_read_points(self, _):
        pmac_buffer = self.pmac.read_points(3, num_axes=2)

        points = [100, 100, 100,
                  200, 200, 200,
                  300, 300, 300]

        self.assertEqual(pmac_buffer[:9], points)


class SetBufferFill(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.set_variable')
    def test_given_current_True_then_A_buffer_set(self, set_variable_mock):
        self.pmac.set_buffer_fill(50, current=True)

        set_variable_mock.assert_called_once_with("P4011", "50")

    @patch('PmacTestHarness_test.TesterPmacTestHarness.set_variable')
    def test_given_current_True_then_A_buffer_set(self, set_variable_mock):
        self.pmac.set_buffer_fill(50, current=False)

        set_variable_mock.assert_called_once_with("P4012", "50")


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


class DoubleToPmacFloatTest(unittest.TestCase):

    def test_given_positive_then_convert(self):

        value = '$500000000803'
        pmac_float = PmacTestHarness.double_to_pmac_float(10)

        self.assertEqual(pmac_float, value)

    def test_given_less_than_1_decimal_then_convert(self):

        value = '$4bac6e59b7fe'
        pmac_float = PmacTestHarness.double_to_pmac_float(0.295599839124)

        self.assertEqual(pmac_float, value)

    def test_given_more_than_1_decimal_then_convert(self):

        value = '$52eb1b910800'
        pmac_float = PmacTestHarness.double_to_pmac_float(1.2955998341)

        self.assertEqual(pmac_float, value)

    def test_given_negative_then_convert(self):

        value = '$ffaffffffff803'
        pmac_float = PmacTestHarness.double_to_pmac_float(-10)

        self.assertEqual(pmac_float, value)
