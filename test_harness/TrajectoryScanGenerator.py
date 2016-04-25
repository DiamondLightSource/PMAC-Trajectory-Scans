from PmacTestHarness import PmacTestHarness
from scanpointgenerator import NestedGenerator, LineGenerator

from pkg_resources import require
require('numpy')
import numpy


class TrajectoryScanGenerator(object):
    """
    A class to generate and manipulate point sets for the PmacTestHarness to perform
    trajectory scans

    """

    def __init__(self):

        self.point_set = {'time': []}

    def generate_snake_scan_w_vel(self, trajectory):
        """
        Generate a snake trajectory scan with dynamic velocity for turnarounds and
        trigger setting

        Args:
            trajectory(dict): A dictionary containing the move_time, width & length of
            the snake_scan area and direction (not implemented yet)

        """

        move_time = trajectory['move_time']
        width = trajectory['width']
        length = trajectory['length']
        direction = trajectory['direction']

        self.point_set = {'time': [], 'x': [], 'y': []}
        trigger = 0

        for i in range(0, width*length):

            if (i+1) % width == 0 and i > 0:
                vel_mode = 1
            elif i % width == 0 and i > 0:
                vel_mode = 2
            else:
                vel_mode = 0

            if (i+1) % width == width/2:
                if trigger == 0:
                    subroutine = 1
                    trigger = 1
                else:
                    subroutine = 2
                    trigger = 0
            else:
                subroutine = 0

            self.point_set['time'].append({'time_val': move_time, 'vel_mode': vel_mode, 'subroutine': subroutine})

        if direction == 0:
            xs = LineGenerator("x", "mm", 0, 10, width)
            ys = LineGenerator("y", "mm", 0, 10, length)
            gen = NestedGenerator(ys, xs, snake=True)
        else:
            raise NotImplementedError("Reverse not implemented")

        for point in gen.iterator():
            self.point_set['x'].append(point.positions['x'])
        for point in gen.iterator():
            self.point_set['y'].append(point.positions['y'])

    def generate_circle_points(self, move_time, num_points):
        """
        Generate a circular trajectory scan

        Args:
            move_time(int): Move time between points
            num_points(int): Number of points to split circle into

        """

        time_points = []
        x_points = []
        y_points = []

        for i in range(0, num_points):

            time_points.append({'time_val': move_time, 'vel_mode': 0, 'subroutine': 0})

        for angle in numpy.linspace(0.0, 2.0*numpy.pi, num_points):
            x_points.append(round(numpy.sin(angle), 10))
            y_points.append(round(numpy.cos(angle), 10) - 1.0)

        self.point_set = {'time': time_points,
                          'x': x_points,
                          'y': y_points}

    def generate_sine_points_one_axis(self, move_time, num_points):
        """
        Generate a point set with sine points for the X-Axis only

        Args:
            move_time(int): Move time between points
            num_points(int): Number of points to split one cycle (0-360) of sine wave into

        """

        time_points = [{'time_val': move_time, 'subroutine': 0, 'vel_mode': 1}]*num_points
        x_points = self._generate_sine_points(num_points)

        self.point_set = {'time': time_points,
                          'x': x_points}

    def generate_sine_points_all_axes(self, move_time, num_points):
        """
        Generate a point set with sine points for all axes

        Args:
            move_time(int): Move time between points
            num_points(int): Number of points to split one cycle (0-360) of sine wave into

        """

        time_points = [{'time_val': move_time, 'subroutine': 0, 'vel_mode': 1}]*num_points

        points = self._generate_sine_points(num_points)

        self.point_set = {'time': time_points,
                          'x': points[:],
                          'y': points[:],
                          'z': points[:],
                          'u': points[:],
                          'v': points[:],
                          'w': points[:],
                          'a': points[:],
                          'b': points[:],
                          'c': points[:]}

    @staticmethod
    def _generate_sine_points(num_points):
        """
        Generate points of a sine wave

        Args:
            num_points(int): Number of points to split one cycle (0-360) of sine wave into

        Returns:
            list: Points of sine wave

        """

        points = []
        for angle in numpy.linspace(0.0, 2.0*numpy.pi, num_points):
            points.append(round(numpy.sin(angle), 10))
        return points

    def check_max_velocity_of_points(self, cs):
        """
        Check that the current point set won't exceed the max velocity of the axes on
        the given coordinate system

        Args:
            cs(PmacCoordinateSystem.PmacCoordinateSystem): CS instance with
            max_velocities stored

        Returns:
            bool: True if points are OK

        Raises:
            Value Error: "Points set will exceed maximum velocity for motor `motor_num`"

        """

        for axis, axis_points in self.point_set.iteritems():

            if axis_points and axis != 'time':

                for point_num in range(1, len(axis_points)):
                    next_point = float(axis_points[point_num])
                    prev_point = float(axis_points[point_num - 1])
                    # Divide by 4 to convert time values from 1/4s of a ms to ms
                    move_time = float(self.point_set['time'][point_num]['time_val']) / 4

                    velocity = (next_point - prev_point)/move_time
                    if velocity > cs.max_velocities[axis]:
                        raise ValueError(
                            "Points set will exceed maximum velocity, {max_vel}, for axis {axis}: {vel}"
                            "".format(axis=axis, max_vel=cs.max_velocities[axis], vel=velocity))

        return True

    def format_point_set(self):

        formatted_points = {'time': [],
                            'x': [], 'y': [], 'z': [],
                            'u': [], 'v': [], 'w': [],
                            'a': [], 'b': [], 'c': []}

        for axis, axis_points in self.point_set.iteritems():
            if axis == 'time':
                for point in axis_points:
                    time_point = "$" + hex(point['time_val'])[2:]

                    if point['subroutine'] > 0:
                        time_point = self.set_point_subroutine(time_point, point['subroutine'])

                    if point['vel_mode'] > 0:
                        time_point = self.set_point_vel_mode(time_point, point['vel_mode'])

                    formatted_points['time'].append(time_point)
            else:
                for point in axis_points:
                    formatted_points[axis].append(self.double_to_pmac_float(point))

        self.point_set = formatted_points

    def grab_buffer_of_points(self, start, length):

        end = start + length
        num_points = len(self.point_set['time'])
        points_grab = {'time': [], 'x': [], 'y': []}

        if end < num_points:
            points_grab['time'] = self.point_set['time'][start:end]
            points_grab['x'] = self.point_set['x'][start:end]
            points_grab['y'] = self.point_set['y'][start:end]
        else:
            new_end = end - num_points
            points_grab['time'] = self.point_set['time'][start:num_points] + \
                self.point_set['time'][:new_end]
            points_grab['x'] = self.point_set['x'][start:num_points] + \
                self.point_set['x'][:new_end]
            points_grab['y'] = self.point_set['y'][start:num_points] + \
                self.point_set['y'][:new_end]
            end = new_end

        return points_grab, end

    def convert_points_to_pmac_float(self, points):
        """
        Convert the position coordinates of `points` to pmac float type.

        Args:
            points(dict): Set of points to convert

        Returns:
            dict: Set of points with position coordinated converted to pmac float
        """

        pmac_points = {'time': points['time'],
                       'x': [], 'y': [], 'z': [],
                       'u': [], 'v': [], 'w': [],
                       'a': [], 'b': [], 'c': []}

        for axis, axis_points in points.iteritems():
            if axis != 'time':
                for point in axis_points:
                    pmac_points[axis].append(self.double_to_pmac_float(point))

        return pmac_points

    @staticmethod
    def double_to_pmac_float(value):
        """
        Convert `value` to the custom PMAC hex format for a float

        Args:
            value(float):

        Returns:
            str: PMAC hex format of `value`
        """

        if value == value*10:
            return '$0'

        negative = False
        if value < 0.0:
            value *= -1.0
            negative = True

        exp_value = value
        exponent = 0
        # Normalise value between 1 and 2
        while exp_value >= 2.0:
            exp_value /= 2.0
            exponent += 1
        while exp_value < 1.0:
            exp_value *= 2.0
            exponent -= 1
        # Offset exponent to provide +-2048 range
        exponent += 0x800

        # Bit shift mantissa to maximum precision (in powers of 2)
        mantissa_value = value
        max_mantissa = 34359738368.0
        while mantissa_value < max_mantissa:
            mantissa_value *= 2.0
        mantissa_value /= 2.0

        # To make value negative, subtract value from max
        if negative:
            int_value = 0xFFFFFFFFFFF - int(mantissa_value)
        else:
            int_value = int(mantissa_value)

        # Bit shift mantissa to correct location, then add exponent to end
        pmac_float = int_value << 12
        pmac_float += exponent

        # Convert decimal representation to string of hex representation.
        return "$" + str(hex(pmac_float))[2:]

    @staticmethod
    def set_point_vel_mode(coord, velocity_mode):
        """
        Add velocity mode to time coordinate

        Args:
            coord(str): Time coordinate in hex
            velocity_mode(int): Velocity mode to set; 0, 1 or 2

        Returns:
            Updated time coordinate

        """

        if velocity_mode in [0, 1, 2]:
            velocity_specifier = "{vel_mode}0000000".format(vel_mode=velocity_mode)
            new_coord = "$" + PmacTestHarness.add_hex(coord[1:], velocity_specifier)
        else:
            raise ValueError("Velocity mode must be 0, 1 or 2")

        return new_coord

    @staticmethod
    def set_point_subroutine(coord, subroutine):
        """
        Add subroutine call to time coordinate

        Args:
            coord(str): Time coordinate in hex
            subroutine(int): Subroutine to set; 10-16

        Returns:
            Updated time coordinate

        """

        if subroutine in range(10, 16):
            velocity_specifier = "{vel_mode}000000".format(vel_mode=hex(subroutine)[2:])
            new_coord = "$" + PmacTestHarness.add_hex(coord[1:], velocity_specifier)
        else:
            raise ValueError("Subroutine must be in range 10 - 16")

        return new_coord
