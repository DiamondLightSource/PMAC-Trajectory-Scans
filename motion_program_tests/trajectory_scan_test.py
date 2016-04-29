from test_harness.PmacTestHarness import PmacTestHarness
from test_harness.TrajectoryScanGenerator import TrajectoryScanGenerator as ScanGen
import unittest
import time
import numpy

# PMAC_IP = "172.23.243.169"
PMAC_IP = "172.23.253.15"
PROG_NUM = 1
CS_NUM = 1


class InitialisationTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_cs_motors([(1, "X", 100), (2, "Y", 100)], CS_NUM)

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_valid_axes_then_set_axis_values(self):
        self.pmac.set_axes(['A', 'B', 'C', 'U', 'V', 'W', 'X', 'Y', 'Z'])
        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        time.sleep(1)

        self.assertEqual(self.pmac.read_variable("M4040"), "511")  # AxesParser
        self.assertEqual(self.pmac.read_variable("M4041"), "1")    # *_Axis Values
        self.assertEqual(self.pmac.read_variable("M4042"), "1")
        self.assertEqual(self.pmac.read_variable("M4043"), "1")
        self.assertEqual(self.pmac.read_variable("M4044"), "1")
        self.assertEqual(self.pmac.read_variable("M4045"), "1")
        self.assertEqual(self.pmac.read_variable("M4046"), "1")
        self.assertEqual(self.pmac.read_variable("M4047"), "1")
        self.assertEqual(self.pmac.read_variable("M4048"), "1")
        self.assertEqual(self.pmac.read_variable("M4049"), "1")

    def test_given_axes_too_high_then_error_status_and_abort(self):
        self.pmac.set_variable(self.pmac.P_variables['axes'], "550")
        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['status']), "3")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['abort']), "0")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['error']), "1")

    def test_given_axes_too_low_then_error_status_and_abort(self):
        self.pmac.set_axes([])
        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['status']), "3")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['abort']), "0")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['error']), "1")


class FillBuffersTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.current_buffer = 0

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_set_current_buffer_fill(self):
        self.pmac.set_current_buffer_fill(50)

        self.assertEqual(self.pmac.read_variable(
            self.pmac.P_variables['buffer_fill_A']), "50")

    def test_set_idle_buffer_fill(self):
        self.pmac.set_idle_buffer_fill(50)

        self.assertEqual(self.pmac.read_variable(
            self.pmac.P_variables['buffer_fill_B']), "50")


class AbortTests(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_cs_motors([(1, "X", 1), (2, "Y", 1)], CS_NUM)
        self.pmac.home_cs_motors(CS_NUM)
        self.pmac.set_axes(['X', 'Y'])

        self.ScanGen = ScanGen()

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_running_and_abort_command_then_abort(self):
        points = self.ScanGen.convert_points_to_pmac_float(
            {'time': ['$FA0', '$FA0', '$3E8', '$3E8', '$3E8', '$3E8'],
             'x': [1, 2, 3, 4, 5, 6]})
        self.pmac.fill_current_buffer(points)
        self.pmac.set_current_buffer_fill(50)
        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        time.sleep(1)
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['status']), "1")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['abort']), "0")

        self.pmac.set_abort()

        time.sleep(2)
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['status']), "2")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['abort']), "1")

    def test_given_time_0_then_status_3_and_error_2(self):
        points = self.ScanGen.convert_points_to_pmac_float(
            {'time': ['$0', '$0', '$0'], 'x': [1, 2, 3]})
        self.pmac.fill_current_buffer(points)
        self.pmac.set_current_buffer_fill(3)
        self.pmac.run_motion_program(PROG_NUM, CS_NUM)
        time.sleep(1)

        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['status']), "3")
        self.assertEqual(self.pmac.read_variable(self.pmac.P_variables['error']), "2")


class TrajectoryScanTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_cs_motors([(1, "X", 1), (2, "Y", 1)], CS_NUM)
        self.pmac.home_cs_motors(CS_NUM)
        self.pmac.set_axes(['X', 'Y'])
        self.pmac.reset_buffers()
        self.move_time = 40

        self.ScanGen = ScanGen()
        self.ScanGen.generate_circle_points(self.move_time, 3600)
        self.ScanGen.format_point_set()

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_single_point_then_move(self):

        buffer_fill = 1
        self.ScanGen.point_set = {'x': [10.0],
                                  'time': [{'time_val': 4000, 'subroutine': 0, 'vel_mode': 0}]}
        self.ScanGen.format_point_set()

        self.pmac.fill_current_buffer(self.ScanGen.point_set)
        self.pmac.set_current_buffer_fill(buffer_fill)
        self.pmac.set_idle_buffer_fill(0)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)
        scan_time = (self.move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 3)

        self.assertEqual("2", self.pmac.read_variable(self.pmac.P_variables['status']))
        self.assertEqual("0", self.pmac.read_variable(self.pmac.P_variables['abort']))

    def test_given_one_partial_buffer_then_complete(self):

        buffer_fill = 25
        current_points, _ = self.ScanGen.grab_buffer_of_points(0, buffer_fill)

        self.pmac.fill_current_buffer(current_points)
        self.pmac.set_current_buffer_fill(buffer_fill)
        self.pmac.set_idle_buffer_fill(0)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)
        scan_time = (self.move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 2)

        self.assertEqual("2", self.pmac.read_variable(self.pmac.P_variables['status']))
        self.assertEqual("0", self.pmac.read_variable(
            self.pmac.P_variables['buffer_address_B']))
        self.assertEqual(str(buffer_fill), self.pmac.read_variable(
            self.pmac.P_variables['total_points']))

    def test_given_one_full_buffer_then_complete(self):

        buffer_fill = self.pmac.buffer_length
        current_points, _ = self.ScanGen.grab_buffer_of_points(0, buffer_fill)

        self.pmac.fill_current_buffer(current_points)
        self.pmac.set_current_buffer_fill(buffer_fill)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)
        scan_time = (self.move_time/4*buffer_fill)/1000
        print("Scan Time: " + str(scan_time) + "s")
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable(self.pmac.P_variables['status']))
        self.assertEqual("0", self.pmac.read_variable(self.pmac.P_variables['abort']))
        self.assertEqual(str(buffer_fill), self.pmac.read_variable(
            self.pmac.P_variables['total_points']))

    def test_given_second_partial_buffer_then_complete(self):

        buffer_fill_a = int(self.pmac.buffer_length)
        buffer_fill_b = 25

        a_points, _ = self.ScanGen.grab_buffer_of_points(0, buffer_fill_a)
        b_points, _ = self.ScanGen.grab_buffer_of_points(0, buffer_fill_b)

        self.pmac.fill_current_buffer(a_points)
        self.pmac.set_current_buffer_fill(buffer_fill_a)

        self.pmac.fill_idle_buffer(b_points)
        self.pmac.set_idle_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        scan_time = (self.move_time/4*buffer_fill_a + self.move_time/4*buffer_fill_b)/1000
        print("Scan Time: " + str(scan_time) + "s")
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable(self.pmac.P_variables['status']))
        self.assertEqual("0", self.pmac.read_variable(self.pmac.P_variables['abort']))
        self.assertEqual(str(buffer_fill_a + buffer_fill_b),
                         self.pmac.read_variable(self.pmac.P_variables['total_points']))

    def test_given_two_full_buffers_then_complete(self):

        buffer_fill_a = int(self.pmac.buffer_length)
        buffer_fill_b = int(self.pmac.buffer_length)

        a_points, _ = self.ScanGen.grab_buffer_of_points(0, buffer_fill_a)
        b_points, _ = self.ScanGen.grab_buffer_of_points(0, buffer_fill_b)

        self.pmac.fill_current_buffer(a_points)
        self.pmac.set_current_buffer_fill(buffer_fill_a)

        self.pmac.fill_idle_buffer(b_points)
        self.pmac.set_idle_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        scan_time = (self.move_time/4*buffer_fill_a + self.move_time/4*buffer_fill_b)/1000
        print("Scan Time: " + str(scan_time) + "s")
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable(self.pmac.P_variables['status']))
        self.assertEqual("0", self.pmac.read_variable(self.pmac.P_variables['abort']))
        self.assertEqual(str(buffer_fill_a + buffer_fill_b),
                         self.pmac.read_variable(self.pmac.P_variables['total_points']))

    def test_given_five_full_buffers_then_complete(self):

        buffer_fill_a = int(self.pmac.buffer_length)
        buffer_fill_b = int(self.pmac.buffer_length)

        a_points, end = self.ScanGen.grab_buffer_of_points(0, buffer_fill_a)
        b_points, end = self.ScanGen.grab_buffer_of_points(0, buffer_fill_b)

        self.pmac.fill_current_buffer(a_points)
        self.pmac.set_current_buffer_fill(buffer_fill_a)

        self.pmac.fill_idle_buffer(b_points)
        self.pmac.set_idle_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)

        num_buffers = 1
        self.pmac.prev_buffer_write = 0
        while num_buffers < 5:

            if self.pmac.prev_buffer_write == 1 and int(self.pmac.current_buffer) == 1:
                a_points, end = self.ScanGen.grab_buffer_of_points(0, buffer_fill_a)
                self.pmac.fill_idle_buffer(a_points)
                self.pmac.set_idle_buffer_fill(buffer_fill_a)
                self.pmac.prev_buffer_write = 0
                num_buffers += 1
                print("Filled buffer A")
            elif self.pmac.prev_buffer_write == 0 and int(self.pmac.current_buffer) == 0:
                b_points, end = self.ScanGen.grab_buffer_of_points(0, buffer_fill_b)
                self.pmac.fill_idle_buffer(b_points)
                self.pmac.set_idle_buffer_fill(buffer_fill_b)
                self.pmac.prev_buffer_write = 1
                num_buffers += 1
                print("Filled buffer B")

            time.sleep(self.move_time/4/1000)

            self.pmac.update_status_variables()

        scan_time = (self.move_time/4*buffer_fill_a + self.move_time/4*buffer_fill_b)/1000
        print("Remaining Scan Time: " + str(scan_time) + "s")
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable(self.pmac.P_variables['status']))
        self.assertEqual("0", self.pmac.read_variable(self.pmac.P_variables['abort']))
        self.assertEqual(str(3*buffer_fill_a + 2*buffer_fill_b),
                         self.pmac.read_variable(self.pmac.P_variables['total_points']))


class TrajectoryScanPositionTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_cs_motors([(1, "X", 100), (2, "Y", 100)], CS_NUM)
        self.pmac.home_cs_motors(CS_NUM)
        self.pmac.set_axes(['X', 'Y'])
        self.pmac.reset_buffers()

        self.variables = {'prev_x': "P4107",
                          'curr_x': "P4117",
                          'next_x': "M4007"}

        self.ScanGen = ScanGen()

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_simple_point_set_then_positions_reached(self):
        move_time = 4000
        point_set = {'x': [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                     'time': [{'time_val': move_time, 'subroutine': 0, 'vel_mode': 0}]*6}
        expected_positions = [0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0]

        self.ScanGen.point_set = point_set
        self.ScanGen.format_point_set()

        self.pmac.fill_current_buffer(self.ScanGen.point_set)
        self.pmac.set_current_buffer_fill(6)
        self.pmac.set_idle_buffer_fill(0)
        self.pmac.set_cs_initial_coordinates(CS_NUM)

        self.pmac.run_motion_program(PROG_NUM, CS_NUM)
        act_pos = [self.pmac.read_motor_position(1)]
        x_pos = [(self.pmac.read_variable(self.variables['prev_x']),
                  self.pmac.read_variable(self.variables['curr_x']),
                  self.pmac.read_variable(self.variables['next_x']))]

        time.sleep(1)
        for _ in range(0, len(point_set['x'])):
            x_pos.append((self.pmac.read_variable(self.variables['prev_x']),
                          self.pmac.read_variable(self.variables['curr_x']),
                          self.pmac.read_variable(self.variables['next_x'])))
            act_pos.append(self.pmac.read_motor_position(1))
            time.sleep(1)

        print(act_pos)
        print(x_pos)

        for i, position in enumerate(act_pos):
            rounded_position = numpy.round(float(position)/100, 0)*100
            self.assertEqual(expected_positions[i], rounded_position)
