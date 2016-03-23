from PmacTestHarness import PmacTestHarness
import trajectory_scan_driver as driver
import unittest
import time

PMAC_IP = "172.23.243.169"
PROG_NUM = 1


class InitialisationTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_motors()

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_valid_axes_then_set_axis_values(self):
        self.pmac.set_axes(511)
        self.pmac.run_motion_program(PROG_NUM)

        self.assertEqual(self.pmac.read_variable("P4003"), "0")
        self.assertEqual(self.pmac.read_variable("P4101"), "1")
        self.assertEqual(self.pmac.read_variable("P4102"), "1")
        self.assertEqual(self.pmac.read_variable("P4103"), "1")
        self.assertEqual(self.pmac.read_variable("P4104"), "1")
        self.assertEqual(self.pmac.read_variable("P4105"), "1")
        self.assertEqual(self.pmac.read_variable("P4106"), "1")
        self.assertEqual(self.pmac.read_variable("P4107"), "1")
        self.assertEqual(self.pmac.read_variable("P4108"), "1")
        self.assertEqual(self.pmac.read_variable("P4109"), "1")

    def test_given_axes_too_high_then_error_status_and_abort(self):
        self.pmac.set_axes(550)
        self.pmac.run_motion_program(PROG_NUM)

        self.assertEqual(self.pmac.read_variable("P4001"), "3")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")

    def test_given_axes_too_low_then_error_status_and_abort(self):
        self.pmac.set_axes(0)
        self.pmac.run_motion_program(PROG_NUM)

        self.assertEqual(self.pmac.read_variable("P4001"), "3")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")


class FillBuffersTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_send_points_then_buffers_filled(self):
        self.pmac.send_points(driver.generate_lin_points(5, 100), current=True)

        pmac_buffer = self.pmac.read_points(5)

        self.assertEqual(pmac_buffer[:15], ['100', '100', '100', '100', '100',
                                            '1', '2', '3', '4', '5',
                                            '1', '2', '3', '4', '5'])

    def test_set_buffer_fill_current(self):
        self.pmac.set_buffer_fill(50, current=True)

        self.assertEqual(self.pmac.read_variable("P4011"), "50")

    def test_set_buffer_fill_not_current(self):
        self.pmac.set_buffer_fill(50, current=False)

        self.assertEqual(self.pmac.read_variable("P4012"), "50")


class AbortTests(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_motors()
        self.pmac.home_motors()
        self.pmac.set_axes(256)

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_running_and_abort_command_then_abort(self):
        self.pmac.send_points(driver.generate_lin_points(50, 500), current=True)
        self.pmac.set_buffer_fill(50, current=True)
        self.pmac.run_motion_program(PROG_NUM)

        time.sleep(0.1)
        self.assertEqual(self.pmac.read_variable("P4001"), "1")
        self.assertEqual(self.pmac.read_variable("P4002"), "0")

        self.pmac.set_abort()

        time.sleep(0.5)
        self.assertEqual(self.pmac.read_variable("P4001"), "2")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")

    def test_given_time_0_then_abort_and_status_3(self):
        self.pmac.send_points([[0, 0, 0], [1, 2, 3]], current=True)
        self.pmac.set_buffer_fill(3, current=True)
        self.pmac.run_motion_program(PROG_NUM)
        time.sleep(0.1)

        self.assertEqual(self.pmac.read_variable("P4001"), "3")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")


class TrajectoryScanTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_motors()
        self.pmac.home_motors()
        self.pmac.set_axes(384)

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_one_partial_buffer_then_complete_and_abort(self):

        buffer_fill = 25
        move_time = 400

        line_points = driver.generate_lin_points(buffer_fill, move_time)

        self.pmac.send_points(line_points, current=True)
        self.pmac.set_buffer_fill(buffer_fill, current=True)

        self.pmac.run_motion_program(PROG_NUM)
        scan_time = (move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 1)

        self.assertEqual(self.pmac.read_variable("P4001"), "2")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")
        self.assertEqual(self.pmac.read_variable("P4005"), str(buffer_fill))

    def test_given_one_full_buffer_then_complete(self):

        buffer_length = self.pmac.buffer_length
        buffer_fill = int(buffer_length)
        move_time = 400

        line_points = driver.generate_lin_points(buffer_fill, move_time)
        self.pmac.send_points(line_points, current=True)
        self.pmac.set_buffer_fill(buffer_fill, current=True)

        self.pmac.run_motion_program(PROG_NUM)
        scan_time = (move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 1)

        self.assertEqual(self.pmac.read_variable("P4001"), "2")
        self.assertEqual(self.pmac.read_variable("P4002"), "0")
        self.assertEqual(self.pmac.read_variable("P4005"), str(buffer_fill))

    def test_given_second_partial_buffer_then_complete_and_abort(self):

        buffer_length = self.pmac.buffer_length
        buffer_fill_a = int(buffer_length)
        buffer_fill_b = 25
        move_time = 400

        line_points = driver.generate_lin_points(buffer_fill_a, move_time)
        self.pmac.send_points(line_points, current=True)
        self.pmac.set_buffer_fill(buffer_fill_a, current=True)

        line_points = driver.generate_lin_points(buffer_fill_b, move_time)
        self.pmac.send_points(line_points)
        self.pmac.set_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM)

        scan_time = (move_time/4*buffer_fill_a + move_time/4*buffer_fill_b)/1000
        time.sleep(scan_time + 1)

        self.assertEqual(self.pmac.read_variable("P4001"), "2")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")
        self.assertEqual(self.pmac.read_variable("P4005"),
                         str(buffer_fill_a + buffer_fill_b))

    def test_given_two_full_buffers_then_complete(self):

        buffer_length = self.pmac.buffer_length
        buffer_fill_a = int(buffer_length)
        buffer_fill_b = int(buffer_length)
        move_time = 400

        line_points = driver.generate_lin_points(buffer_fill_a, move_time)
        self.pmac.send_points(line_points, current=True)
        self.pmac.set_buffer_fill(buffer_fill_a, current=True)

        line_points = driver.generate_lin_points(buffer_fill_b, move_time)
        self.pmac.send_points(line_points)
        self.pmac.set_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM)

        scan_time = (move_time/4*buffer_fill_a + move_time/4*buffer_fill_b)/1000
        time.sleep(scan_time + 1)

        self.assertEqual(self.pmac.read_variable("P4001"), "2")
        self.assertEqual(self.pmac.read_variable("P4002"), "0")
        self.assertEqual(self.pmac.read_variable("P4005"), "100")

    def test_given_five_full_buffers_then_complete(self):

        buffer_length = self.pmac.buffer_length
        buffer_fill_a = int(buffer_length)
        buffer_fill_b = int(buffer_length)
        move_time = 400

        line_points = driver.generate_lin_points(buffer_fill_a, move_time)
        self.pmac.send_points(line_points, current=True)
        self.pmac.set_buffer_fill(buffer_fill_a, current=True)

        line_points = driver.generate_lin_points(buffer_fill_b, move_time)
        self.pmac.send_points(line_points)
        self.pmac.set_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM)

        num_buffers = 2
        while num_buffers < 5:

            if self.pmac.prev_buffer_write == 1 and int(self.pmac.current_buffer) == 1:
                self.pmac.send_points(line_points)
                self.pmac.set_buffer_fill(buffer_fill_a)
                self.pmac.prev_buffer_write = 0
                num_buffers += 1
            elif self.pmac.prev_buffer_write == 0 and int(self.pmac.current_buffer) == 0:
                self.pmac.send_points(line_points)
                self.pmac.set_buffer_fill(buffer_fill_b)
                self.pmac.prev_buffer_write = 1
                num_buffers += 1

            time.sleep(move_time/4/1000)

            self.pmac.update_status_variables()

        scan_time = (move_time/4*buffer_fill_a + move_time/4*buffer_fill_b)/1000
        time.sleep(scan_time + 1)

        self.assertEqual(self.pmac.read_variable("P4001"), "2")
        self.assertEqual(self.pmac.read_variable("P4002"), "0")
        self.assertEqual(self.pmac.read_variable("P4005"), "250")
