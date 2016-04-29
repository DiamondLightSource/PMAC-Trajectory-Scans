from dls_pmacremote import PmacEthernetInterface
from PmacCoordinateSystem import PmacCoordinateSystem as PmacCS


class PmacTestHarness(PmacEthernetInterface):
    """
    A pmac controller that can interface with and control the `trajectory_scan` motion
    program as the EPICS driver does.

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

        # Connection initialisation
        self.setConnectionParams(host=ip_address, port=1025)
        self.connect()

        # Variables read directly from PMAC
        self.P_variables = {'status': "P4001",
                            'abort': "P4002",
                            'axes': "P4003",
                            'buffer_length': "P4004",
                            'total_points': "P4005",
                            'current_index': "P4006",
                            'current_buffer': "P4007",
                            'buffer_address_A': "P4008",
                            'buffer_address_B': "P4009",
                            'buffer_fill_A': "P4011",
                            'buffer_fill_B': "P4012",
                            'error': "P4015",
                            'version': "P4020"}

        self.status = int(self.read_variable(self.P_variables['status']))
        self.error = int(self.read_variable(self.P_variables['error']))
        self.total_points = 0
        self.current_index = 0
        self.current_buffer = 0

        # Fixed values
        self.buffer_length = int(
            self.read_variable(self.P_variables['buffer_length']))
        self.buffer_address_A = str(hex(int(
            self.read_variable(self.P_variables['buffer_address_A'])))[2:])
        self.buffer_address_B = str(hex(int(
            self.read_variable(self.P_variables['buffer_address_B'])))[2:])

        # Other PMAC information
        self.addresses = {}  # Addresses for each sub-buffer, set based on current buffer
        self.prev_buffer_write = 1

        # PMAC CS Set Up
        self.coordinate_system = {'1': PmacCS(1)}
        self.read_cs_max_velocities(1)

    def update_status_variables(self):
        """
        Update status, error, total points scanned, current index and current buffer
        specifier from the pmac

        """

        status, error, total_points, current_index, current_buffer = \
            self.read_multiple_variables([self.P_variables['status'], self.P_variables['error'],
                                          self.P_variables['total_points'],
                                          self.P_variables['current_index'],
                                          self.P_variables['current_buffer']])

        self.status = int(status)
        self.error = int(error)
        self.total_points = int(total_points)
        self.current_index = int(current_index)
        self.current_buffer = int(current_buffer)

    def update_address_dict(self, root_address):
        """
        Update the addresses of each sub-buffer based on the current half buffer

        Args:
            root_address(str): Root address of current half-buffer

        """

        self.addresses = {'time': root_address,
                          'a': self.add_dechex(root_address, int(self.buffer_length)),
                          'b': self.add_dechex(root_address, 2*int(self.buffer_length)),
                          'c': self.add_dechex(root_address, 3*int(self.buffer_length)),
                          'u': self.add_dechex(root_address, 4*int(self.buffer_length)),
                          'v': self.add_dechex(root_address, 5*int(self.buffer_length)),
                          'w': self.add_dechex(root_address, 6*int(self.buffer_length)),
                          'x': self.add_dechex(root_address, 7*int(self.buffer_length)),
                          'y': self.add_dechex(root_address, 8*int(self.buffer_length)),
                          'z': self.add_dechex(root_address, 9*int(self.buffer_length))}

    def add_coordinate_system(self, cs_instance, cs_number):
        """
        Add a coordinate system instance

        Args:
            cs_instance(PmacCoordinateSystem): Coordinate system to add
            cs_number(int): Number of coordinate system

        """

        self.coordinate_system[str(cs_number)] = cs_instance

    def read_cs_max_velocities(self, cs_number):
        """
        Read the maximum allowed velocities from variables ix16 on the PMAC for
        given coordinate system

        Args:
            cs_number: Coordinate system to read for

        """

        velocities = []
        for i in range(1, 10):
            velocities.append(self.read_variable("i{axis}16".format(axis=i)))

        self.coordinate_system[str(cs_number)].set_max_velocities(velocities)

    def set_cs_initial_coordinates(self, cs_number):
        """
        Set Current_* values for required axes to be the actual motor positions; these act
        as the start positions for the motion program

        Args:
            cs_number(int): Coordinate system to set coordinates for

        """

        axis_assignments = {'A': 1, 'B': 2, 'C': 3,
                            'U': 4, 'V': 5, 'W': 6,
                            'X': 7, 'Y': 8, 'Z': 9}

        for axis, egu_scaling in self.coordinate_system[str(cs_number)].motor_map.itervalues():
            current_position = str(int(self.read_motor_position(axis_assignments[axis])) * egu_scaling)

            self.set_variable("P411" + str(axis_assignments[axis]), current_position)

    def assign_cs_motors(self, axis_map, cs_number):
        """
        Send command to assign motors to the required axes

        Args:
            axis_map(list(int, str, int)): List of axes to assign to motor, with scaling
            e.g. [(1, "X", 100), (3, "Y", 25)] => &1 #1->100X #3->25Y
            cs_number(int): Coordinate system to assign motors for

        """

        command = "&" + str(cs_number)
        for motor, axis, scaling in axis_map:
            if int(motor) not in range(1, 16):
                raise ValueError("Motor selection invalid")
            if axis.upper() not in ["X", "Y", "Z", "U", "V", "W", "A", "B", "C"]:
                raise ValueError("Axis selection invalid")

            self.coordinate_system[str(cs_number)].add_motor_assignment(motor, axis, scaling)
            command += \
                " #{motor_num}->{scaling}{axis}".format(
                    motor_num=motor, scaling=scaling, axis=axis)

        self.sendCommand(command)

    def home_cs_motors(self, cs_number):
        """
        Send command to home motors in given coordinate system

        Args:
            cs_number(int): Coordinate system to assign motors for

        """

        command = ""
        for motor in self.coordinate_system[str(cs_number)].motor_map.iterkeys():
            command += "#{motor}HMZ".format(motor=motor)

        self.sendCommand(command)

    def run_motion_program(self, program_num, cs_number):
        """
        Send command to run motion program

        Args:
            program_num(int): Number of motion program to run
            cs_number(int): Coordinate system to run motion program in

        """

        command = ""
        for motor in self.coordinate_system[str(cs_number)].motor_map.iterkeys():
            command += "#{motor}J/".format(motor=motor)
        command += "&" + str(self.coordinate_system[str(cs_number)].cs_number)
        command += "B" + str(program_num) + "R"

        self.sendCommand(command)

    def check_program_exists(self, program_number):

        response = self.sendCommand("List Program " + str(program_number))

        # If the program exists, PmacEthernetInterface will return a third
        # entry with the program, if not it will be empty - the PMAC will
        # return an ERR003 for which PEI raises an IOError
        if len(response) == 3:
            return True
        else:
            return False

    def force_abort(self):
        """
        Send on-line abort command to pmac

        """

        self.sendCommand("A")

    def set_abort(self):
        """
        Set abort variable

        """

        self.set_variable(self.P_variables['abort'], "1")

    def set_axes(self, axes):
        """
        Send number of require axes

        Args:
            axes(list(str)): List of required axes

        """
        axis_definitions = {'A': 1, 'B': 2, 'C': 4,
                            'U': 8, 'V': 16, 'W': 32,
                            'X': 64, 'Y': 128, 'Z': 256}

        axes_val = 0
        for axis in axes:
            axes_val += axis_definitions[axis.upper()]

        self.set_variable(self.P_variables['axes'], str(axes_val))

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

    def read_points(self, num_points, buffer_num=0, num_axes=1):
        """
        Read points stored in pmac memory buffer

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

    def read_multiple_variables(self, variables):
        """
        Read a set of variables

        Args:
            variables(list(str)): The list of variables to read

        Returns:
            list(str): Values of variables

        Raises:
            IOError: Read failed

        """

        command = ""
        for variable in variables:
            command += variable

        value, success = self.sendCommand(command)
        if success:
            return tuple(value.split('\r')[:-1])
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
                        'a': zeroes[:], 'b': zeroes[:], 'c': zeroes[:],
                        'u': zeroes[:], 'v': zeroes[:], 'w': zeroes[:],
                        'x': zeroes[:], 'y': zeroes[:], 'z': zeroes[:]}
        self.fill_current_buffer(reset_points)

        reset_points = {'time': zeroes[:],
                        'a': zeroes[:], 'b': zeroes[:], 'c': zeroes[:],
                        'u': zeroes[:], 'v': zeroes[:], 'w': zeroes[:],
                        'x': zeroes[:], 'y': zeroes[:], 'z': zeroes[:]}
        self.fill_idle_buffer(reset_points)

    def read_motor_position(self, motor_num):
        """
        Request current position of give motor

        Args:
            motor_num(int): Motor to read position for

        Returns:
            str: Current position of motor

        """

        position = self.sendCommand("#{motor}P".format(motor=motor_num))[0].split('\r')[0]

        return position

    def read_motor_velocity(self, motor_num):
        """
        Request current velocity of give motor

        Args:
            motor_num(int): Motor to read velocity for

        Returns:
            str: Current velocity of motor

        """

        position = self.sendCommand("#{motor}V".format(motor=motor_num))[0].split('\r')[0]

        return position

    def fill_idle_buffer(self, points):
        """
        Update the address dictionary to fill idle buffer and then call _fill_buffer()

        Args:
            points(dict): Point set to fill with

        """

        if self.current_buffer == 0:
            self.update_address_dict(self.buffer_address_B)
        else:
            self.update_address_dict(self.buffer_address_A)

        self._fill_buffer(points)

    def fill_current_buffer(self, points):
        """
        Update the address dictionary to fill current buffer and then call _fill_buffer()

        Args:
            points(dict): Point set to fill with

        """

        if self.current_buffer == 0:
            self.update_address_dict(self.buffer_address_A)
        else:
            self.update_address_dict(self.buffer_address_B)

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
                response = self._construct_write_command_and_remove_used_points(
                    command_details)

                self.sendCommand(response['command'])

                address = self.add_dechex(address, response['num_sent'])
                axis_points = response['points']

    @staticmethod
    def _construct_write_command_and_remove_used_points(command_details):
        """
        Construct a command to send as many points as possible from the given set.
        Length must be less than 255 characters

        Args:
            command_details(dict): The write mode, address and point set to send

        Returns:
            dict: The resulting command, the remaining point set and the number of
            points sent
        """

        command = 'W' + command_details['mode'] + '$' + command_details['address']

        point_num = 0
        axis_points = command_details['points']
        while len(axis_points) > 0 and len(command + ',' + axis_points[0]) <= 255:
            command += ',' + axis_points.pop(0)
            point_num += 1

        response = {'command': command, 'points': axis_points, 'num_sent': point_num}
        return response

    def set_idle_buffer_fill(self, fill_level):
        """
        Set the buffer fill level of the idle buffer.

        Args:
            fill_level(int): Number of coordinate sets in buffer

        """

        if self.current_buffer == 0:
            self.set_variable(self.P_variables['buffer_fill_B'], str(fill_level))
        else:
            self.set_variable(self.P_variables['buffer_fill_A'], str(fill_level))

    def set_current_buffer_fill(self, fill_level):
        """
        Set the buffer fill level of the current buffer.

        Args:
            fill_level(int): Number of coordinate sets in buffer

        """

        if self.current_buffer == 0:
            self.set_variable(self.P_variables['buffer_fill_A'], str(fill_level))
        else:
            self.set_variable(self.P_variables['buffer_fill_B'], str(fill_level))

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
