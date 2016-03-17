import time
import socket
from dls_pmacremote import PmacEthernetInterface
from scanpointgenerator import NestedGenerator, LineGenerator


class PmacTestHarness(PmacEthernetInterface):

    def __init__(self):
        super(PmacTestHarness, self).__init__(parent=None, verbose=False, numAxes=None, timeout=3.0)

        self.setConnectionParams(host="172.23.243.169", port=1025)
        self.connect()

        self.status = self.read_variable("P4001")
        self.total_points = 0
        self.current_index = 0
        self.current_buffer = 0
        self.buffer_length = self.read_variable("P4004")
        self.buffer_address_a = str(hex(int(self.read_variable("P4008")))[2:])
        self.buffer_address_b = str(hex(int(self.read_variable("P4009")))[2:])

        self.prev_buffer_write = 1

    def update_status_variables(self):

        self.status = int(self.read_variable("P4001"))
        self.total_points = int(self.read_variable("P4005"))
        self.current_index = int(self.read_variable("P4006"))
        self.current_buffer = int(self.read_variable("P4007"))

    def assign_motors(self):

        self.sendCommand("&1 #1->X #2->Y #3->Z #4->U #5->V #6->W #7->A #8->B")

    def home_motors(self):

        self.sendCommand("#1hmz#2hmz#3hmz#4hmz#5hmz#6hmz#7hmz#8hmz#9hmz")

    def run_motion_program(self):

        self.sendCommand("#1J/ #2J/ #3J/ #4J/ #5J/ #6J/ #7J/ #8J/ &1 B1 R")

    def set_axes(self, axes):

        self.set_variable("P4003", axes)

    def set_buffer_fill(self, fill_level, current=False):

        buffer_toggle = int(current)

        if self.current_buffer == buffer_toggle:
            self.set_variable("P4012", fill_level)
        else:
            self.set_variable("P4011", fill_level)

    def read_address(self, mode, address):

        value, success = self.sendCommand("R" + mode + " $" + address)
        if success:
            return value.split('\r')[0]
        else:
            return "Read failed"

    def write_to_address(self, mode, address, value):

        response, success = self.sendCommand("W" + mode + " $" + address + " " + value)

        if success:
            return success
        else:
            return response, success

    def read_variable(self, variable):

        value, success = self.sendCommand(variable)
        if success:
            return value.split('\r')[0]
        else:
            return "Read failed"

    def set_variable(self, variable, value):

        response, success = self.sendCommand(str(variable) + "=" + str(value))

        if success:
            return success
        else:
            return response, success

    def send_points(self, points, current=False):

        buffer_toggle = int(current)

        if self.current_buffer == buffer_toggle:
            start = self.buffer_address_b
        else:
            start = self.buffer_address_a

        for i, subset in enumerate(points):
            current_address = self.add_dechex(start, int(self.buffer_length)*i)
            # print(current_address)
            for point in subset:
                self.write_to_address("L", current_address, str(point))
                current_address = self.inc_hex(current_address)

        print("Points sent to " + start)

    def reset_buffers(self):

        current_address = self.buffer_address_a

        for i in range(0, int(self.buffer_length)*11*2):
            self.write_to_address("L", current_address, "0")
            current_address = self.inc_hex(current_address)

    @staticmethod
    def add_dechex(hexdec, dec):

        return hex(int(hexdec, base=16) + dec)[2:]

    @staticmethod
    def inc_hex(hexdec):

        return PmacTestHarness.add_dechex(hexdec, 1)


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

    pmac = PmacTestHarness()

    pmac.assign_motors()
    pmac.home_motors()
    pmac.reset_buffers()
    pmac.set_axes("384")

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

    pmac.run_motion_program()

    print("Buffer length: " + pmac.buffer_length)
    print("Buffer A: " + pmac.buffer_address_a)
    print("Buffer B: " + pmac.buffer_address_b)

    time.sleep(5)

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
