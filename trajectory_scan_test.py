from PmacTestHarness import PmacTestHarness
import trajectory_scan_driver as driver
import unittest
import time

# PMAC_IP = "172.23.243.169"
PMAC_IP = "172.23.253.15"
PROG_NUM = 1


class InitialisationTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_motors(["100X", "100Y"])

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_valid_axes_then_set_axis_values(self):
        self.pmac.set_axes(511)
        self.pmac.run_motion_program(PROG_NUM)

        self.assertEqual(self.pmac.read_variable("M4040"), "511")
        self.assertEqual(self.pmac.read_variable("M4041"), "1")
        self.assertEqual(self.pmac.read_variable("M4042"), "1")
        self.assertEqual(self.pmac.read_variable("M4043"), "1")
        self.assertEqual(self.pmac.read_variable("M4044"), "1")
        self.assertEqual(self.pmac.read_variable("M4045"), "1")
        self.assertEqual(self.pmac.read_variable("M4046"), "1")
        self.assertEqual(self.pmac.read_variable("M4047"), "1")
        self.assertEqual(self.pmac.read_variable("M4048"), "1")
        self.assertEqual(self.pmac.read_variable("M4049"), "1")

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

    def test_set_buffer_fill_current(self):
        self.pmac.set_buffer_fill(50, current=True)

        self.assertEqual(self.pmac.read_variable("P4011"), "50")

    def test_set_buffer_fill_not_current(self):
        self.pmac.set_buffer_fill(50, current=False)

        self.assertEqual(self.pmac.read_variable("P4012"), "50")


class AbortTests(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_motors(["100X", "100Y"])
        self.pmac.home_motors()
        self.pmac.set_axes(256)

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_running_and_abort_command_then_abort(self):
        points = self.pmac.convert_points_to_pmac_float({'time': ['$190', '$190', '$190'], 'x': [1, 2, 3]})
        self.pmac.fill_current_buffer(points)
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
        points = self.pmac.convert_points_to_pmac_float({'time': ['$0', '$0', '$0'], 'x': [1, 2, 3]})
        self.pmac.fill_current_buffer(points)
        self.pmac.set_buffer_fill(3, current=True)
        self.pmac.run_motion_program(PROG_NUM)
        time.sleep(0.1)

        self.assertEqual(self.pmac.read_variable("P4001"), "3")
        self.assertEqual(self.pmac.read_variable("P4002"), "1")


class TrajectoryScanTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.assign_motors(["100X", "100Y"])
        self.pmac.home_motors()
        self.pmac.set_axes(384)
        # self.pmac.reset_buffers()
        self.move_time = 250

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_given_single_point_then_move(self):

        buffer_fill = 1

        circle_points = driver.generate_circle_points(self.move_time)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        current_points, _ = driver.grab_buffer_of_points(0, buffer_fill, circle_points)

        self.pmac.fill_current_buffer(current_points)
        self.pmac.set_buffer_fill(buffer_fill, current=True)

        self.pmac.run_motion_program(PROG_NUM)
        scan_time = (self.move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable("P4001"))
        self.assertEqual("0", self.pmac.read_variable("P4002"))
        self.assertEqual(str(buffer_fill), self.pmac.read_variable("P4005"))

    def test_given_one_partial_buffer_then_complete(self):

        buffer_fill = 25

        circle_points = driver.generate_circle_points(self.move_time)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        current_points, _ = driver.grab_buffer_of_points(0, buffer_fill, circle_points)

        self.pmac.fill_current_buffer(current_points)
        self.pmac.set_buffer_fill(buffer_fill, current=True)

        self.pmac.run_motion_program(PROG_NUM)
        scan_time = (self.move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 2)

        self.assertEqual("2", self.pmac.read_variable("P4001"))
        self.assertEqual("0", self.pmac.read_variable("P4002"))
        self.assertEqual(str(buffer_fill), self.pmac.read_variable("P4005"))

    def test_given_one_full_buffer_then_complete(self):

        buffer_fill = int(self.pmac.buffer_length)

        circle_points = driver.generate_circle_points(self.move_time)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        current_points, _ = driver.grab_buffer_of_points(0, buffer_fill, circle_points)

        self.pmac.fill_current_buffer(current_points)
        self.pmac.set_buffer_fill(buffer_fill, current=True)

        self.pmac.run_motion_program(PROG_NUM)
        scan_time = (self.move_time/4*buffer_fill)/1000
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable("P4001"))
        self.assertEqual("0", self.pmac.read_variable("P4002"))
        self.assertEqual(str(buffer_fill), self.pmac.read_variable("P4005"))

    def test_given_second_partial_buffer_then_complete(self):

        buffer_fill_a = int(self.pmac.buffer_length)
        buffer_fill_b = 25

        circle_points = driver.generate_circle_points(self.move_time)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        a_points, _ = driver.grab_buffer_of_points(0, buffer_fill_a, circle_points)
        b_points, _ = driver.grab_buffer_of_points(buffer_fill_a, buffer_fill_b, circle_points)

        self.pmac.fill_current_buffer(a_points)
        self.pmac.set_buffer_fill(buffer_fill_a, current=True)

        self.pmac.fill_idle_buffer(b_points)
        self.pmac.set_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM)

        scan_time = (self.move_time/4*buffer_fill_a + self.move_time/4*buffer_fill_b)/1000
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable("P4001"))
        self.assertEqual("0", self.pmac.read_variable("P4002"))
        self.assertEqual(str(buffer_fill_a + buffer_fill_b),
                         self.pmac.read_variable("P4005"))

    def test_given_two_full_buffers_then_complete(self):

        buffer_fill_a = int(self.pmac.buffer_length)
        buffer_fill_b = int(self.pmac.buffer_length)

        circle_points = driver.generate_circle_points(self.move_time)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        a_points, _ = driver.grab_buffer_of_points(0, buffer_fill_a, circle_points)
        b_points, _ = driver.grab_buffer_of_points(buffer_fill_a, buffer_fill_b, circle_points)

        self.pmac.fill_current_buffer(a_points)
        self.pmac.set_buffer_fill(buffer_fill_a, current=True)

        self.pmac.fill_idle_buffer(b_points)
        self.pmac.set_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM)

        scan_time = (self.move_time/4*buffer_fill_a + self.move_time/4*buffer_fill_b)/1000
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable("P4001"))
        self.assertEqual("0", self.pmac.read_variable("P4002"))
        self.assertEqual(str(buffer_fill_a + buffer_fill_b), self.pmac.read_variable("P4005"))

    def test_given_five_full_buffers_then_complete(self):

        buffer_fill_a = int(self.pmac.buffer_length)
        buffer_fill_b = buffer_fill_a

        circle_points = driver.generate_circle_points(self.move_time)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        a_points, end = driver.grab_buffer_of_points(0, buffer_fill_a, circle_points)
        b_points, end = driver.grab_buffer_of_points(end, buffer_fill_b, circle_points)

        self.pmac.fill_current_buffer(a_points)
        self.pmac.set_buffer_fill(buffer_fill_a, current=True)

        self.pmac.fill_idle_buffer(b_points)
        self.pmac.set_buffer_fill(buffer_fill_b)

        self.pmac.run_motion_program(PROG_NUM)

        num_buffers = 1
        self.pmac.prev_buffer_write = 0
        while num_buffers < 5:

            if self.pmac.prev_buffer_write == 1 and int(self.pmac.current_buffer) == 1:
                a_points, end = driver.grab_buffer_of_points(end, buffer_fill_a, circle_points)
                self.pmac.fill_idle_buffer(a_points)
                self.pmac.set_buffer_fill(buffer_fill_a)
                self.pmac.prev_buffer_write = 0
                num_buffers += 1
                print("Filled buffer A")
            elif self.pmac.prev_buffer_write == 0 and int(self.pmac.current_buffer) == 0:
                b_points, end = driver.grab_buffer_of_points(end, buffer_fill_b, circle_points)
                self.pmac.fill_idle_buffer(b_points)
                self.pmac.set_buffer_fill(buffer_fill_b)
                self.pmac.prev_buffer_write = 1
                num_buffers += 1
                print("Filled buffer B")

            time.sleep(self.move_time/4/1000)

            self.pmac.update_status_variables()

        scan_time = (self.move_time/4*buffer_fill_a + self.move_time/4*buffer_fill_b)/1000
        time.sleep(scan_time + 1)

        self.assertEqual("2", self.pmac.read_variable("P4001"))
        self.assertEqual("0", self.pmac.read_variable("P4002"))
        self.assertEqual(str(3*buffer_fill_a + 2*buffer_fill_b),
                         self.pmac.read_variable("P4005"))
