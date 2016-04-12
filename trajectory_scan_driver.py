import time
from PmacTestHarness import PmacTestHarness
from scanpointgenerator import NestedGenerator, LineGenerator

from pkg_resources import require
require('numpy')
import numpy

# IP_ADDRESS = "172.23.243.169"
IP_ADDRESS = "172.23.253.15"


def check_max_velocity_of_points(points, max_velocities):

    for axis, axis_points in points.itervalues():
        for point_num in range(1, len(axis)):
            next_point = axis_points[point_num]
            prev_point = axis_points[point_num - 1]
            move_time = points['time'][point_num]
            max_velocity = max_velocities[axis]

            if (next_point - prev_point)/move_time > max_velocity:
                raise ValueError(
                    "Points set will exceed maximum velocity for motor {}".format(axis))


def generate_lin_points(num_points, move_time):

    points = {'time': [], 'x': [], 'y': []}

    for j in range(1, num_points+1, 1):
        points['time'].append(hex(move_time)[2:])

    for j in range(1, num_points+1, 1):
        points['x'].append(j)
        points['y'].append(j)

    return points


def generate_snake_scan(move_time, reverse=False):

    points = {'time': [], 'x': [], 'y': []}

    for i in range(0, 50):
        points['time'].append(move_time)

    if reverse:
        raise NotImplementedError("Reverse not implemented")
    else:
        xs = LineGenerator("x", "mm", 0, 10, 5)
        ys = LineGenerator("y", "mm", 0, 10, 5)
        gen = NestedGenerator(ys, xs, snake=True)

    for point in gen.iterator():
        points['x'].append(int(point.lower['x']))
        points['x'].append(int(point.upper['x']))
    for point in gen.iterator():
        points['y'].append(int(point.lower['y']))
        points['y'].append(int(point.upper['y']))

    return points


def generate_snake_scan_w_vel(trajectory):

    move_time = trajectory['move_time']
    width = trajectory['width']
    length = trajectory['length']
    direction = trajectory['direction']

    points = {'time': [], 'x': [], 'y': []}
    trigger = 0

    for i in range(0, width*length):
        time_point = "$" + hex(move_time)[2:]

        if (i+1) % width == 0 and i > 0:
            time_point = PmacTestHarness.set_point_vel_mode(time_point, 1)
        elif i % width == 0 and i > 0:
            time_point = PmacTestHarness.set_point_vel_mode(time_point, 2)

        if (i+1) % width == width/2:
            if trigger == 0:
                time_point = PmacTestHarness.set_point_subroutine(time_point, "A")
                trigger = 1
            else:
                time_point = PmacTestHarness.set_point_subroutine(time_point, "B")
                trigger = 0

        points['time'].append(time_point)

    if direction == 0:
        xs = LineGenerator("x", "mm", 0, 10, width)
        ys = LineGenerator("y", "mm", 0, 10, length)
        gen = NestedGenerator(ys, xs, snake=True)
    else:
        raise NotImplementedError("Reverse not implemented")

    for point in gen.iterator():
        points['x'].append(PmacTestHarness.double_to_pmac_float(int(point.positions['x'])))
    for point in gen.iterator():
        points['y'].append(PmacTestHarness.double_to_pmac_float(int(point.positions['y'])))

    return points


def generate_circle_points(move_time, num_points):

    time_points = ["$" + PmacTestHarness.add_hex(hex(move_time)[2:], "20000000")]*num_points
    x_points = []
    y_points = []

    for angle in numpy.linspace(0.0, 360.0, num_points):
        x_points.append(numpy.sin(angle))
        y_points.append(numpy.cos(angle))

    points = {'time': time_points,
              'x': x_points,
              'y': y_points}

    return points


def generate_sine_points_one_axis(move_time, num_points):

    time_points = ['$' + hex(move_time)[2:]]*num_points
    x_points = []

    for angle in numpy.linspace(0.0, 360.0, num_points):
        x_points.append(numpy.sin(angle))

    points = {'time': time_points,
              'x': x_points}

    return points


def generate_sine_points_all_axes(move_time, num_points):

    time_points = ["$" + PmacTestHarness.add_hex(hex(move_time)[2:], "20000000")]*num_points

    points = []

    for angle in numpy.linspace(0.0, 360.0, num_points):
        points.append(numpy.sin(angle))

    points = {'time': time_points,
              'x': points,
              'y': points,
              'z': points,
              'u': points,
              'v': points,
              'w': points,
              'a': points,
              'b': points,
              'c': points}

    return points


def grab_buffer_of_points(start, length, points):

    end = start + length
    num_points = len(points['time'])
    points_grab = {'time': [], 'x': [], 'y': []}

    if end < num_points:
        points_grab['time'] = points['time'][start:end]
        points_grab['x'] = points['x'][start:end]
        points_grab['y'] = points['y'][start:end]
    else:
        new_end = end - num_points
        points_grab['time'] = points['time'][start:num_points] + points['time'][:new_end]
        points_grab['x'] = points['x'][start:num_points] + points['x'][:new_end]
        points_grab['y'] = points['y'][start:num_points] + points['y'][:new_end]
        end = new_end

    return points_grab, end


def trajectory_scan():

    pmac = PmacTestHarness(IP_ADDRESS)

    pmac.assign_motors()
    pmac.home_motors()
    pmac.reset_buffers()
    pmac.set_axes(384)

    line_points = generate_lin_points(50, 1003)
    snake_points = generate_snake_scan(1003)
    print(line_points)
    print(snake_points)

    buffer_fill_a = 50
    buffer_fill_b = 50
    pmac.send_points(line_points, current=True)
    pmac.send_points(snake_points)
    pmac.set_buffer_fill(buffer_fill_a, current=True)
    pmac.set_buffer_fill(buffer_fill_b)

    pmac.run_motion_program(1)

    time.sleep(3)

    pmac.update_status_variables()
    print("Status: " + str(pmac.status))

    a_points = snake_points
    b_points = line_points

    while int(pmac.status) == 1:

        if pmac.prev_buffer_write == 1 and int(pmac.current_buffer) == 1:
            if a_points == line_points:
                a_points = snake_points
            else:
                a_points = line_points
            pmac.send_points(a_points)
            pmac.set_buffer_fill(buffer_fill_a)
            pmac.prev_buffer_write = 0
        elif pmac.prev_buffer_write == 0 and int(pmac.current_buffer) == 0:
            if b_points == line_points:
                b_points = snake_points
            else:
                b_points = line_points
            pmac.send_points(b_points)
            pmac.set_buffer_fill(buffer_fill_b)
            pmac.prev_buffer_write = 1

        time.sleep(0.25)

        if 1 > 2:
            pmac.setVar("P4007", 1)  # End Program

        pmac.update_status_variables()

        if pmac.current_buffer == 0:
            print("Status: " + str(pmac.status) + " - Buffer: A" + " - Index: " +
                  str(pmac.current_index) + " - Total Points: " + str(pmac.total_points))
        else:
            print("Status: " + str(pmac.status) + " - Buffer: B" + " - Index: " +
                  str(pmac.current_index) + " - Total Points: " + str(pmac.total_points))


def snake_scan():

    pmac = PmacTestHarness(IP_ADDRESS)

    pmac.force_abort()
    pmac.assign_motors(["5X", "5Y"])
    pmac.home_motors()
    pmac.reset_buffers()
    pmac.set_axes(384)

    width = 10
    length = 10
    trajectory = {'move_time': 4000,
                  'width': width,
                  'length': length,
                  'direction': 0}
    snake_points = generate_snake_scan_w_vel(trajectory)
    for axis in snake_points.iteritems():
        print(axis)

    buffer_fill = width * length
    pmac.fill_current_buffer(snake_points)
    pmac.set_buffer_fill(buffer_fill, current=True)

    pmac.run_motion_program(1)

    time.sleep(3)

    pmac.update_status_variables()
    print("Status: " + str(pmac.status))

    while int(pmac.status) == 1:

        time.sleep(1)

        pmac.update_status_variables()
        print(pmac.read_variable("M4011"))
        print(pmac.read_variable("M4016"))

        if pmac.current_buffer == 0:
            print("Status: " + str(pmac.status) + " - Error: " + str(pmac.error) + " - Buffer: A" + " - Index: " +
                  str(pmac.current_index) + " - Total Points: " + str(pmac.total_points))
        else:
            print("Status: " + str(pmac.status) + " - Error: " + str(pmac.error) + " - Buffer: B" + " - Index: " +
                  str(pmac.current_index) + " - Total Points: " + str(pmac.total_points))


def circle_scan():

    pmac = PmacTestHarness(IP_ADDRESS)

    pmac.force_abort()
    pmac.assign_motors(["100X", "100Y"])
    pmac.home_motors()
    # pmac.reset_buffers()
    pmac.set_axes(384)

    circle_points = generate_circle_points(500, 3600)
    print(circle_points)
    print(len(circle_points['time']))
    print(len(circle_points['x']))
    print(len(circle_points['y']))
    circle_points = pmac.convert_points_to_pmac_float(circle_points)

    buffer_length = 1000
    current_start = 0
    current_points, end = grab_buffer_of_points(current_start, buffer_length, circle_points)
    current_start = end + 1

    pmac.fill_current_buffer(current_points)
    pmac.set_buffer_fill(buffer_length, current=True)
    pmac.prev_buffer_write = 0

    start_time = time.time()
    pmac.run_motion_program(1)
    time.sleep(1)

    pmac.update_status_variables()
    print("Status: " + str(pmac.status))

    while int(pmac.status) == 1:

        if pmac.prev_buffer_write == 1 and int(pmac.current_buffer) == 1:
            current_points, end = grab_buffer_of_points(current_start, 1000, circle_points)
            current_start = end + 1

            pmac.fill_idle_buffer(current_points)
            pmac.set_buffer_fill(buffer_length)
            pmac.prev_buffer_write = 0

        elif pmac.prev_buffer_write == 0 and int(pmac.current_buffer) == 0:
            current_points, end = grab_buffer_of_points(current_start, 1000, circle_points)
            current_start = end + 1

            pmac.fill_idle_buffer(current_points)
            pmac.set_buffer_fill(buffer_length)
            pmac.prev_buffer_write = 1

        time.sleep(0.1)

        if 1 > 2:
            pmac.setVar("P4007", 1)  # End Program

        pmac.update_status_variables()
        scan_time = str(numpy.round(time.time() - start_time, 2))

        if pmac.current_buffer == 0:
            print("Status: " + str(pmac.status) + " - Buffer: A" + " - Index: " +
                  str(pmac.current_index) + " - Total Points: " + str(pmac.total_points) +
                  " - Scan Time: " + scan_time)
        else:
            print("Status: " + str(pmac.status) + " - Buffer: B" + " - Index: " +
                  str(pmac.current_index) + " - Total Points: " + str(pmac.total_points) +
                  " - Scan Time: " + scan_time)


def main():

    # trajectory_scan()
    snake_scan()
    # circle_scan()

if __name__ == "__main__":
    main()
