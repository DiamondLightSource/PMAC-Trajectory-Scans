import time
from PMAC_test_harness import PmacTestHarness
from scanpointgenerator import NestedGenerator, LineGenerator


def generate_lin_points(num_points):

    time_points = []
    x_points = []
    y_points = []

    for j in range(1, num_points+1, 1):
        time_points.append(250)

    for i in range(1, 9, 1):
        for j in range(1, num_points+1, 1):
            x_points.append(j)
            y_points.append(j)

    return time_points, x_points, y_points


def generate_snake_scan(reverse=False):

    time_points = []
    x_points = []
    y_points = []

    for i in range(0, 50):
        time_points.append(250)

    if reverse:
        xs = LineGenerator("x", "mm", 0, 10, 5)
        ys = LineGenerator("y", "mm", 0, 10, 5)
        gen = NestedGenerator(ys, xs, snake=True)
    else:
        xs = LineGenerator("x", "mm", 0, 10, 5)
        ys = LineGenerator("y", "mm", 0, 10, 5)
        gen = NestedGenerator(ys, xs, snake=True)

    for point in gen.iterator():
        x_points.append(int(point.lower['x']))
        x_points.append(int(point.upper['x']))
    for point in gen.iterator():
        y_points.append(int(point.lower['y']))
        y_points.append(int(point.upper['y']))

    return time_points, x_points, y_points


def trajectory_scan():

    pmac = PmacTestHarness("172.23.243.169")

    pmac.assign_motors()
    pmac.home_motors()
    pmac.reset_buffers()
    pmac.set_axes(384)

    line_points = generate_lin_points(50)
    snake_points = generate_snake_scan()
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

        time.sleep(0.5)

        if 1 > 2:
            pmac.setVar("P4007", 1)  # End Program

        pmac.update_status_variables()

        print("Status: " + str(pmac.status) + " - Buffer: " + str(pmac.current_buffer) + " - Index: " +
              str(pmac.current_index) + " - Total Points: " + str(pmac.total_points))


def main():

    trajectory_scan()
    # points = generate_lin_points(25)
    # send_points(points)


if __name__ == "__main__":
    main()