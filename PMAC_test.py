from PmacTestHarness import PmacTestHarness
import trajectory_scan_driver as driver
import unittest
import time

# PMAC_IP = "172.23.243.169"
PMAC_IP = "172.23.253.15"
PROG_NUM = 1


def length_str_in_list(string_list):
    length = 0
    for string in string_list:
        length += len(string)

    return length


class WriteTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.home_motors()
        self.pmac.set_axes(384)

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_buffer_fill_time_one_axis(self):

        buffer_fill = int(self.pmac.buffer_length)

        sine_points = driver.generate_sine_points_one_axis(400, 1000)
        sine_points = self.pmac.convert_points_to_pmac_float(sine_points)

        length = 0
        for axis in sine_points.itervalues():
            length += length_str_in_list(axis)
        print("Total message length: " + str(length))

        start_time = time.time()
        self.pmac.fill_current_buffer(sine_points)
        self.pmac.set_buffer_fill(buffer_fill)
        print("Time to fill buffers: " + str(time.time() - start_time))
        print("Messages sent: " + str(self.pmac.sendCommand.called))

    def test_buffer_fill_time_two_axes(self):

        buffer_fill = int(self.pmac.buffer_length)

        circle_points = driver.generate_circle_points(400, 3600)
        circle_points = self.pmac.convert_points_to_pmac_float(circle_points)
        buffer_points, _ = driver.grab_buffer_of_points(0, buffer_fill, circle_points)

        length = 0
        for axis in buffer_points.itervalues():
            length += length_str_in_list(axis)
        print("Total message length: " + str(length))

        start_time = time.time()
        self.pmac.fill_current_buffer(buffer_points)
        self.pmac.set_buffer_fill(buffer_fill)
        print("Time to fill buffers: " + str(time.time() - start_time))
        print("Messages sent: " + str(self.pmac.sendCommand.called))

    def test_buffer_fill_time_nine_axes(self):

        buffer_fill = int(self.pmac.buffer_length)

        sine_points = driver.generate_sine_points_all_axes(400, 1000)
        sine_points = self.pmac.convert_points_to_pmac_float(sine_points)

        length = 0
        for axis in sine_points.itervalues():
            length += length_str_in_list(axis)
        print("Total message length: " + str(length))

        start_time = time.time()
        self.pmac.fill_current_buffer(sine_points)
        self.pmac.set_buffer_fill(buffer_fill)
        print("Time to fill buffers: " + str(time.time() - start_time))
        print("Messages sent: " + str(self.pmac.sendCommand.called))

    def test_read_max_velocities(self):
        self.pmac.update_max_velocities()

        print("Max Axis Velocities:")
        for axis, velocity in self.pmac.max_velocities.iteritems():
            print(axis + ": " + velocity + "cts/ms")

    def test_float_parsing(self):
        self.pmac.sendCommand("M4500->L:$30000,0,48")

        value = 10
        pmac_float = self.pmac.double_to_pmac_float(value)
        print(str(value) + ' : ' + str(pmac_float))
        self.pmac.write_to_address("L", "30000", pmac_float)
        self.assertAlmostEqual(value, float(self.pmac.read_variable("M4500")), places=5)

