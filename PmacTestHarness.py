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
        self.total_points = 0
        self.current_index = 0
        self.current_buffer = 0
        self.buffer_length = self.read_variable("P4004")
        self.buffer_address_a = str(hex(int(self.read_variable("P4008")))[2:])
        self.buffer_address_b = str(hex(int(self.read_variable("P4009")))[2:])

        self.prev_buffer_write = 1

    def update_status_variables(self):
        """
        Update status, total points scanned, current index and current buffer specifier from the pmac

        """

        self.status = int(self.read_variable("P4001"))
        self.total_points = int(self.read_variable("P4005"))
        self.current_index = int(self.read_variable("P4006"))
        self.current_buffer = int(self.read_variable("P4007"))

    def assign_motors(self):
        """
        Send command to assign motors to the required axes

        """

        self.sendCommand("&1 #1->100X #2->100Y #3->Z #4->U #5->V #6->W #7->A #8->B")

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

        self.sendCommand("#1J/ #2J/ #3J/ #4J/ #5J/ #6J/ #7J/ #8J/ &{num} B{num} R".format(
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
            axes(int): Int between 1 and 510 that will be split into 8 bits specifying the
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

        current_address = self.buffer_address_a

        for _ in range(0, int(self.buffer_length)*10*2):
            self.write_to_address("L", current_address, "0")
            current_address = self.inc_hex(current_address)

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
            start = self.buffer_address_b
        else:
            start = self.buffer_address_a

        current_address = start
        for time_point in points[0]:
            self.write_to_address("Y", current_address, str(time_point))
            current_address = self.inc_hex(current_address)

        axis_num = 0
        for axis in points[1:]:
            axis_num += 1
            current_address = self.add_dechex(start, int(self.buffer_length)*axis_num)
            # print(current_address)
            for point in axis:
                self.write_to_address("L", current_address, str(point))
                current_address = self.inc_hex(current_address)

        print("Points sent to " + start)

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
