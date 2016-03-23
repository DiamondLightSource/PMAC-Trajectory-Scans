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


class SendPointsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.write_to_address')
    def test_given_current_True_then_points_sent_to_A(self, write_mock):
        points = [[100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
                  [200, 200, 200, 200, 200, 200, 200, 200, 200, 200],
                  [300, 300, 300, 300, 300, 300, 300, 300, 300, 300]]

        self.pmac.send_points(points, current=True)

        self.assertEqual(write_mock.call_args_list[0][0], ("Y", "30000", "100"))
        self.assertEqual(write_mock.call_args_list[10][0], ("L", ANY, "200"))
        self.assertEqual(write_mock.call_args_list[20][0], ("L", ANY, "300"))
        self.assertEqual(write_mock.call_count, 30)

    @patch('PmacTestHarness_test.TesterPmacTestHarness.write_to_address')
    def test_given_current_False_points_sent_to_B(self, write_mock):
        points = [[100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
                  [200, 200, 200, 200, 200, 200, 200, 200, 200, 200],
                  [300, 300, 300, 300, 300, 300, 300, 300, 300, 300]]

        self.pmac.send_points(points, current=False)

        self.assertEqual(write_mock.call_args_list[0][0], ("Y", "30226", "100"))
        self.assertEqual(write_mock.call_args_list[10][0], ("L", ANY, "200"))
        self.assertEqual(write_mock.call_args_list[20][0], ("L", ANY, "300"))
        self.assertEqual(write_mock.call_count, 30)


class ReadPointsTest(unittest.TestCase):

    def setUp(self):
        self.pmac = TesterPmacTestHarness()

    @patch('PmacTestHarness_test.TesterPmacTestHarness.read_address',
           side_effect=[100, 100, 100, 200, 200, 200, 300, 300, 300,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0])
    def test_read_points(self, _):
        pmac_buffer = self.pmac.read_points(3)

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
