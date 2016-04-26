import time
from test_harness.PmacTestHarness import PmacTestHarness
from test_harness.TrajectoryScanGenerator import TrajectoryScanGenerator as ScanGen

from pkg_resources import require
require('numpy')
import numpy

# IP_ADDRESS = "172.23.243.169"
IP_ADDRESS = "172.23.253.15"


def make_status_message(pmac, start_time, sleep_time):

    time.sleep(sleep_time)

    pmac.update_status_variables()

    if pmac.current_buffer == 0:
        status_message = "Buffer: A"
    else:
        status_message = "Buffer: B"

    scan_time = str(numpy.round(time.time() - start_time, 2))

    status_message += (" - Status: " + str(pmac.status) +
                       " - Error: " + str(pmac.error) +
                       " - Index: " + str(pmac.current_index) +
                       " - Total Points: " + str(pmac.total_points) +
                       " - VelMode: " + pmac.read_variable("M4011") +
                       " - Trigger State: " + pmac.read_variable("M32") +
                       " - Scan Time: " + scan_time)

    return status_message


def snake_trajectory_scan():

    pmac = PmacTestHarness(IP_ADDRESS)

    pmac.force_abort()
    pmac.assign_motors([(1, "X", 1), (2, "Y", 1)])
    pmac.home_motors()
    pmac.reset_buffers()
    pmac.set_axes(384)

    width = 20
    length = 20
    trajectory = {'move_time': 2000,
                  'width': width,
                  'length': length,
                  'step': 1,
                  'direction': 0}

    snake_scan = ScanGen()
    snake_scan.generate_snake_scan_w_vel(trajectory)
    for axis in snake_scan.point_set.iteritems():
        print(axis)

    pmac.read_cs_max_velocities()
    snake_scan.check_max_velocity_of_points(pmac.coordinate_system)

    snake_scan.format_point_set()
    for axis in snake_scan.point_set.iteritems():
        print(axis)

    buffer_fill = width * length
    pmac.fill_current_buffer(snake_scan.point_set)
    pmac.set_current_buffer_fill(buffer_fill)

    start_time = time.time()
    pmac.run_motion_program(1)

    time.sleep(3)

    pmac.update_status_variables()
    print("Status: " + str(pmac.status))

    while int(pmac.status) == 1:
        status_message = make_status_message(pmac, start_time, 0.5)
        print(status_message)


def circle_trajectory_scan():

    pmac = PmacTestHarness(IP_ADDRESS)

    pmac.force_abort()
    pmac.assign_motors([(1, "X", 100), (2, "Y", 100)])
    pmac.home_motors()
    pmac.reset_buffers()
    pmac.set_axes(384)

    circle_scan = ScanGen()
    circle_scan.generate_circle_points(400, 3600)
    print(circle_scan.point_set)
    print(len(circle_scan.point_set['time']))
    print(len(circle_scan.point_set['x']))
    print(len(circle_scan.point_set['y']))
    circle_scan.format_point_set()

    buffer_length = pmac.buffer_length
    current_start = 0
    current_points, end = circle_scan.grab_buffer_of_points(current_start, buffer_length)
    current_start = end + 1

    pmac.fill_current_buffer(current_points)
    pmac.set_current_buffer_fill(buffer_length)
    pmac.prev_buffer_write = 0

    start_time = time.time()
    pmac.run_motion_program(1)
    time.sleep(1)

    pmac.update_status_variables()
    print("Status: " + str(pmac.status))

    while int(pmac.status) == 1:

        if pmac.prev_buffer_write == 1 and int(pmac.current_buffer) == 1:
            current_points, end = circle_scan.grab_buffer_of_points(current_start, buffer_length)
            current_start = end + 1

            pmac.fill_idle_buffer(current_points)
            pmac.set_idle_buffer_fill(buffer_length)
            pmac.prev_buffer_write = 0

        elif pmac.prev_buffer_write == 0 and int(pmac.current_buffer) == 0:
            current_points, end = circle_scan.grab_buffer_of_points(current_start, buffer_length)
            current_start = end + 1

            pmac.fill_idle_buffer(current_points)
            pmac.set_idle_buffer_fill(buffer_length)
            pmac.prev_buffer_write = 1

        status_message = make_status_message(pmac, start_time, 0.5)
        print(status_message)


def main():

    # trajectory_scan()
    snake_trajectory_scan()
    # circle_trajectory_scan()

if __name__ == "__main__":
    main()
