import time
import dls_pmacremote
from scanpointgenerator import NestedGenerator, LineGenerator

pmac = dls_pmacremote.PmacEthernetInterface()
pmac.setConnectionParams(host="172.23.243.169", port=1025)
pmac.connect()
pmac.isModelGeobrick()
pmac.getNumberOfAxes()


def read_address(mode, address):

    value, success = pmac.sendCommand("R" + mode + " $" + address)
    if success:
        return value.split('\r')[0]
    else:
        return "Read failed"


def write_to_address(mode, address, value):

    response, success = pmac.sendCommand("W" + mode + " $" + address + " " + value)

    if success:
        return success
    else:
        return response, success


def read_variable(variable):

    value, success = pmac.sendCommand(variable)
    if success:
        return value.split('\r')[0]
    else:
        return "Read failed"


def add_dechex(hexdec, dec):

    return hex(int(hexdec, base=16) + dec)[2:]


def inc_hex(hexdec):

    return add_dechex(hexdec, 1)


def generate_points(num_points, buffer_length):

    points = []

    for j in range(1, num_points+1, 1):
        points.append(100)

    for i in range(1, 9, 1):
        for j in range(1, num_points+1, 1):
            points.append(j)

    for j in range(0, num_points+1, 1):
        points.append(0)

    return points


def generate_snake_scan():

    points = []

    for i in range(0, 50):
        points.append(500)

    xs = LineGenerator("x", "mm", 0, 10, 5)
    ys = LineGenerator("y", "mm", 0, 10, 5)
    gen = NestedGenerator(ys, xs, snake=True)

    for point in gen.iterator():
        print(point.lower)
        print(point.upper)
        points.append(int(point.lower['x']))
        points.append(int(point.upper['x']))
    for point in gen.iterator():
        points.append(int(point.lower['y']))
        points.append(int(point.upper['y']))

    return points


def send_points(points, start):

    for point in points:
        write_to_address("L", start, str(point))
        current_address = inc_hex(current_address)


def reset_buffers(buffer_length):

    current_address = "30000"

    for i in range(0, int(buffer_length)*11*2):
        write_to_address("L", current_address, "0")
        current_address = inc_hex(current_address)


def trajectory_scan():

    pmac.sendCommand("&1 #1->X #2->Y #3->Z #4->U #5->V #6->W #7->A #8->B")
    pmac.sendCommand("#1hmz#2hmz#3hmz#4hmz#5hmz#6hmz#7hmz#8hmz#9hmz")

    buffer_length = read_variable("P4004")
    buffer_a_address = read_variable("P4008")
    buffer_b_address = read_variable("P4009")

    reset_buffers(buffer_length)

    axes = "384"
    pmac.setVar("P4003", axes)

    # points = generate_points(20, int(buffer_length))
    points = generate_snake_scan()
    buffer_fill_a = 50
    send_points(points[:10], "30000")
    pmac.setVar("P4011", buffer_fill_a)
    pmac.sendCommand("#1J/ #2J/ #3J/ #4J/ #5J/ #6J/ #7J/ #8J/ &1 B1 R")

    print("Buffer fill: " + str(buffer_fill_a))
    print("Buffer length: " + buffer_length)
    print("Buffer A: " + buffer_a_address)
    print("Buffer B: " + buffer_b_address)
    print(points)

    time.sleep(2)

    status = read_variable("P4001")
    print("Status: " + status)

    while int(status) == 1:

        time.sleep(1)

        if 1 > 2:
            pmac.setVar("P4002", 1)  # End Program

        status = read_variable("P4001")
        total_points = read_variable("P4005")
        current_buffer_index = read_variable("P4006")
        current_buffer = read_variable("P4007")

        print("Status: " + status + " - Total Points: " + total_points + " - Index: " +
              current_buffer_index + " - Buffer: " + current_buffer)


def main():

    trajectory_scan()
    # points = generate_points(25)
    # send_points(points)


if __name__ == "__main__":
    main()