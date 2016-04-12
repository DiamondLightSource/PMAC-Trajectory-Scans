from dls_pmacremote import PmacEthernetInterface


class PmacTestHarness(PmacEthernetInterface):
    """
    A pmac controller that can interface with and control the `trajectory_scan` motion program
    as the EPICS driver does.
    """

    def __init__(self, ip_address):
        """
        Set up the connection to the given pmac and retrieve the required variables
        for any functions.

        Args:
            ip_address(str): The IP address of the pmac to connect to

        """
        super(PmacTestHarness, self).__init__(
            parent=None, verbose=False, numAxes=None, timeout=3.0)

        self.setConnectionParams(host=ip_address, port=1025)
        self.connect()

        self.status = self.read_variable("P4001")  # Change to int() ?
        self.error = self.read_variable("P4015")
        self.total_points = 0
        self.current_index = 0
        self.current_buffer = 0
        self.buffer_length = self.read_variable("P4004")
        self.buffer_address_a = str(hex(int(self.read_variable("P4008")))[2:])
        self.buffer_address_b = str(hex(int(self.read_variable("P4009")))[2:])
        self.addresses = {}

        self.prev_buffer_write = 1

        self.max_velocities = {'x': 0, 'y': 0, 'z': 0,
                               'u': 0, 'v': 0, 'w': 0,
                               'a': 0, 'b': 0, 'c': 0}

    def update_status_variables(self):
        """
        Update status, total points scanned, current index and current buffer specifier from the pmac

        """

        self.status = int(self.read_variable("P4001"))
        self.error = int(self.read_variable("P4015"))
        self.total_points = int(self.read_variable("P4005"))
        self.current_index = int(self.read_variable("P4006"))
        self.current_buffer = int(self.read_variable("P4007"))

    def update_address_dict(self, root_address):
        """
        Update the addresses of each sub-buffer based on the current half buffer

        Args:
            root_address(str): Root address of current half-buffer

        """

        self.addresses = {'time': root_address,
                          'x': self.add_dechex(root_address, int(self.buffer_length)),
                          'y': self.add_dechex(root_address, 2*int(self.buffer_length)),
                          'z': self.add_dechex(root_address, 3*int(self.buffer_length)),
                          'u': self.add_dechex(root_address, 4*int(self.buffer_length)),
                          'v': self.add_dechex(root_address, 5*int(self.buffer_length)),
                          'w': self.add_dechex(root_address, 6*int(self.buffer_length)),
                          'a': self.add_dechex(root_address, 7*int(self.buffer_length)),
                          'b': self.add_dechex(root_address, 8*int(self.buffer_length)),
                          'c': self.add_dechex(root_address, 9*int(self.buffer_length))}

    def update_max_velocities(self):
        """
        Read the maximum allowed velocities from variables ix16 on the PMAC

        """

        velocities = []
        for i in range(1, 10):
            velocities.append(self.read_variable("i{axis}16".format(axis=i)))

        self.max_velocities = {'x': velocities[0], 'y': velocities[1], 'z': velocities[2],
                               'u': velocities[3], 'v': velocities[4], 'w': velocities[5],
                               'a': velocities[6], 'b': velocities[7], 'c': velocities[8]}

    def assign_motors(self, axis_map):
        """
        Send command to assign motors to the required axes

        Args:
            axis_map(list(str)): List of axes to assign to motor, with scaling.
            Index corresponds to motor number. e.g. ["100X", "", "25Y"] => &1 #1->100X #3->25Y

        """

        command = "&1"

        for axis_num, axis in enumerate(axis_map):
            if axis:
                command += " #{axis_num}->{axis}".format(axis_num=axis_num + 1, axis=axis)

        self.sendCommand(command)

    def home_motors(self):
        """
        Send command to home motors

        """

        self.sendCommand("#1HMZ #2HMZ #3HMZ #4HMZ #5HMZ #6HMZ #7HMZ #8HMZ #9HMZ")

    def run_motion_program(self, program_num):
        """
        Send command to run motion program

        Args:
            program_num(int): Number of motion program to run

        """

        self.sendCommand("#1J/ #2J/ #3J/ #4J/ #5J/ #6J/ #7J/ #8J/ &1 B{num} R".format(
                num=str(program_num)))

    def force_abort(self):
        """
        Send on-line abort command to pmac

        """

        self.sendCommand("A")

    def set_abort(self):
        """
        Set abort variable

        """

        self.set_variable("P4002", "1")

    def set_axes(self, axes):
        """
        Send number of require axes

        Args:
            axes(int): Int between 1 and 511 that will be split into 9 bits specifying the
            required motors
            e.g. X, Y and Z = 256 + 128 + 64 = 448; X, Y and U = 256 + 128 + 32 = 416
        """

        self.set_variable("P4003", str(axes))

    def read_address(self, mode, address):
        """
        Read the value at a memory location specified by address in a given read mode
        e.g. X, Y, L, D

        Args:
            mode(str): The read mode to interpret the data at the given address
            address(str): The hex address of the memory location to access

        Returns:
            str: Value stored at given address
        Raises:
            IOError: Read failed

        """

        value, success = self.sendCommand("R" + mode + " $" + address)
        if success:
            return value.split('\r')[0]
        else:
            raise IOError("Read failed")

    def write_to_address(self, mode, address, value):
        """
        Write a value into a memory location specified by address in the given mode
        e.g. X, Y, L, D

        Args:
            mode(str): The mode to write the data into the given address with
            address(str): The hex address of the memory location to access
            value(str): The value to write

        Returns:
            bool: Success specifier
        Raises:
            IOError: Write failed

        """

        response, success = self.sendCommand("W" + mode + " $" + address + " " + value)

        if success:
            return response, success
        else:
            raise IOError("Write failed")

    def read_variable(self, variable):
        """
        Read a given variable

        Args:
            variable(str): The variable to read e.g. P4001, M4000

        Returns:
            str: Value of variable

        Raises:
            IOError: Read failed

        """

        value, success = self.sendCommand(variable)
        if success:
            return value.split('\r')[0]
        else:
            raise IOError("Read failed")

    def set_variable(self, variable, value):
        """
        Set a given variable

        Args:
            variable(str): The variable to set e.g. P4001, M4000
            value(str): Value to set variable to

        Returns:
            bool: Success specifier
        Raises:
            IOError: Write failed

        """

        response, success = self.sendCommand(variable + "=" + value)

        if success:
            return response, success
        else:
            raise IOError("Write failed")

    def reset_buffers(self):
        """
        Reset all memory in buffers to 0

        Raises:
            IOError: Write failed

        """

        zeroes = ['$0']*int(self.buffer_length)
        reset_points = {'time': zeroes[:],
                        'x': zeroes[:], 'y': zeroes[:], 'z': zeroes[:],
                        'u': zeroes[:], 'v': zeroes[:], 'w': zeroes[:],
                        'a': zeroes[:], 'b': zeroes[:], 'c': zeroes[:]}

        self.fill_current_buffer(reset_points)
        self.fill_idle_buffer(reset_points)

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

    def send_points(self, points, current=False):
        """
        Send points to fill a buffer. If current is False, the currently unused buffer will be filled.
        If current is True, the current buffer will be filled (e.g. for filling both buffers before
        program is run).

        Args:
            points(list(list(int))): List of lists of the time and coordinates for each move
            current(int): A 1 or 0 to specify which buffer to fill

        Raises:
            IOError: Write failed

        """

        # Toggle to invert buffer choice logic
        buffer_toggle = int(current)

        if self.current_buffer == buffer_toggle:
            self.update_address_dict(self.buffer_address_b)
        else:
            self.update_address_dict(self.buffer_address_a)

        axis_num = 0
        for axis, axis_points in points.iteritems():
            current_address = self.addresses[axis]
            for point in axis_points:
                self.write_to_address("L", current_address, str(point))
                current_address = self.inc_hex(current_address)
            axis_num += 1

        print("Points sent to " + self.addresses['time'])

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
            subroutine(str): Subroutine to set; A, B, C, D, E or F

        Returns:
            Updated time coordinate

        """

        if subroutine in ["A", "B", "C", "D", "E", "F"]:
            velocity_specifier = "{vel_mode}000000".format(vel_mode=subroutine)
            new_coord = "$" + PmacTestHarness.add_hex(coord[1:], velocity_specifier)
        else:
            raise ValueError("Subroutine must be A, B, C, D, E or F")

        return new_coord

    @staticmethod
    def _construct_write_command_and_remove_used_points(command_details):
        """
        Construct a command to send as many points as possible from the given set.
        Length must be less than 255 characters

        Args:
            command_details(dict): The write mode, address and point set to send

        Returns:
            dict: The resulting command, the remaining point set and the number of points sent
        """

        command = 'W' + command_details['mode'] + '$' + command_details['address']

        point_num = 0
        axis_points = command_details['points']
        while len(axis_points) > 0 and len(command + ',' + axis_points[0]) <= 255:
            command += ',' + axis_points.pop(0)
            point_num += 1

        response = {'command': command, 'points': axis_points, 'num_sent': point_num}
        return response

    def fill_idle_buffer(self, points):
        """
        Update the address dictionary to fill idle buffer and then call _fill_buffer()

        Args:
            points(dict): Point set to fill with

        """

        if self.current_buffer == 0:
            self.update_address_dict(self.buffer_address_b)
        else:
            self.update_address_dict(self.buffer_address_a)

        self._fill_buffer(points)

    def fill_current_buffer(self, points):
        """
        Update the address dictionary to fill current buffer and then call _fill_buffer()

        Args:
            points(dict): Point set to fill with

        """

        if self.current_buffer == 0:
            self.update_address_dict(self.buffer_address_a)
        else:
            self.update_address_dict(self.buffer_address_b)

        self._fill_buffer(points)

    def _fill_buffer(self, points):
        """
        Fill buffer specified by `self.addresses` with `points`

        Args:
            points(dict): Point set to fill with

        """

        num_points = len(points['time'])
        if num_points > int(self.buffer_length):
            raise ValueError("Point set cannot be longer than PMAC buffer length")
        for axis in points.itervalues():
            if axis and len(axis) != num_points:
                raise ValueError("Point set must have equal points in all axes")

        for axis_num, axis_points in points.iteritems():
            address = self.addresses[axis_num]
            while len(axis_points) > 0:
                command_details = {'mode': 'L', 'address': address, 'points': axis_points}
                response = self._construct_write_command_and_remove_used_points(command_details)

                self.sendCommand(response['command'])

                address = self.add_dechex(address, response['num_sent'])
                axis_points = response['points']

    def set_buffer_fill(self, fill_level, current=False):
        """
        Set the buffer fill level of a buffer. If current is False, the currently unused buffer will
        be set. If current is True, the current buffer will be set.

        Args:
            fill_level(int): Number of coordinate sets in buffer
            current(int): A 1 or 0 to specify which buffer to set

        """

        buffer_toggle = int(current)

        if self.current_buffer == buffer_toggle:
            self.set_variable("P4012", str(fill_level))
        else:
            self.set_variable("P4011", str(fill_level))

    def read_points(self, num_points, buffer_num=0, num_axes=1):
        """
        Read points store in pmac memory buffer

        Args:
            num_points(int): Number of sets of points to read
            buffer_num(int): Specifier for buffer A (0) or B (1)
            num_axes(int): Number of axes to read

        Returns:
            list(str): Points stored in pmac memory

        Raises:
            IOError: Read failed
        """

        if buffer_num == 0:
            start = "30000"
        else:
            start = self.add_dechex("30000", int(self.buffer_length)*10)

        pmac_buffer = []
        current_address = start
        for _ in range(0, num_points):
            pmac_buffer.append(self.read_address("L", current_address))
            current_address = self.inc_hex(current_address)
        for i in range(1, num_axes + 1):
            current_address = self.add_dechex(start, int(self.buffer_length)*i)
            # print(current_address)
            for _ in range(0, num_points):
                pmac_buffer.append(self.read_address("L", current_address))
                current_address = self.inc_hex(current_address)

        return pmac_buffer

    @staticmethod
    def add_hex(hex1, hex2):
        """
        Add two hexadecimal strings

        Args:
            hex1(str): First hexadecimal string to add
            hex2(str): Second hexadecimal string to add

        Returns:
            str: Hexadecimal sum of hex1 and hex 2
        """

        return hex(int(hex1, base=16) + int(hex2, base=16))[2:]

    @staticmethod
    def add_dechex(hexdec, dec):
        """
        Add an int to a hexadecimal string

        Args:
            hexdec(str): Hexadecimal string to add to
            dec(int): Decimal number to add

        Returns:
            str: Hexadecimal sum of hexdec and dec
        """

        return hex(int(hexdec, base=16) + dec)[2:]

    @staticmethod
    def inc_hex(hexdec):
        """
        Increment a hexadecimal string

        Args:
            hexdec(str): Hexadecimal string to increment

        Returns:
            str: Incremented hexadecimal value
        """

        return PmacTestHarness.add_dechex(hexdec, 1)

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
