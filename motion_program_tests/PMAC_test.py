from test_harness.PmacTestHarness import PmacTestHarness
from test_harness.TrajectoryScanGenerator import TrajectoryScanGenerator as ScanGen
import unittest
import time
import random

# PMAC_IP = "172.23.243.169"
PMAC_IP = "172.23.253.15"
PROG_NUM = 1
CS_NUM = 1


def length_str_in_list(string_list):
    length = 0
    for string in string_list:
        length += len(string)

    return length


class WriteTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness(PMAC_IP)
        self.pmac.home_cs_motors(CS_NUM)
        self.pmac.set_axes(['X', 'Y'])

    def tearDown(self):
        self.pmac.force_abort()
        self.pmac.disconnect()

    def test_buffer_fill_time_one_axis(self):

        scan = ScanGen()
        scan.generate_sine_points_one_axis(400, self.pmac.buffer_length)
        scan.format_point_set()

        length = 0
        for axis in scan.point_set.itervalues():
            length += length_str_in_list(axis)
        print("Total message length: " + str(length))

        start_time = time.time()
        self.pmac.fill_current_buffer(scan.point_set)
        self.pmac.set_current_buffer_fill(self.pmac.buffer_length)
        print("Time to fill buffers: " + str(time.time() - start_time))
        print("Messages sent: " + str(self.pmac.sendCommand.called))

    def test_buffer_fill_time_two_axes(self):

        scan = ScanGen()
        scan.generate_sine_points_one_axis(400, self.pmac.buffer_length)
        scan.format_point_set()
        scan.point_set['y'] = scan.point_set['x'][:]

        length = 0
        for axis in scan.point_set.itervalues():
            length += length_str_in_list(axis)
        print("Total message length: " + str(length))

        start_time = time.time()
        self.pmac.fill_current_buffer(scan.point_set)
        self.pmac.set_current_buffer_fill(self.pmac.buffer_length)
        print("Time to fill buffers: " + str(time.time() - start_time))
        print("Messages sent: " + str(self.pmac.sendCommand.called))

    def test_buffer_fill_time_nine_axes(self):

        scan = ScanGen()
        scan.generate_sine_points_all_axes(400, self.pmac.buffer_length)
        scan.format_point_set()

        length = 0
        for axis in scan.point_set.itervalues():
            length += length_str_in_list(axis)
        print("Total message length: " + str(length))

        start_time = time.time()
        self.pmac.fill_current_buffer(scan.point_set)
        self.pmac.set_current_buffer_fill(self.pmac.buffer_length)
        print("Time to fill buffers: " + str(time.time() - start_time))
        print("Messages sent: " + str(self.pmac.sendCommand.called))

    def test_read_max_velocities(self):
        self.pmac.assign_cs_motors([(1, "X", 50)], CS_NUM)
        self.pmac.read_cs_max_velocities(CS_NUM)

        print("Max Axis Velocities:")
        velocities = self.pmac.coordinate_system['1'].max_velocities
        for axis, velocity in velocities.iteritems():
            print(axis + ": " + str(velocity) + "cts/ms")

    def test_float_parsing(self):
        self.pmac.sendCommand("M4500->L:$30000,0,48")

        for _ in range(1000):
            number = random.uniform(-10000000.0, 10000000)
            pmac_float = ScanGen.double_to_pmac_float(number)

            self.pmac.write_to_address("L", "30000", pmac_float)
            read_back = float(self.pmac.read_variable("M4500"))
            # print(number, read_back)
            self.assertAlmostEqual(number, read_back, places=10-len(str(int(number)))-1)
