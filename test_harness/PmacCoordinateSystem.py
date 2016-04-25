

class PmacCoordinateSystem(object):
    """
    A helper class for PmacTestHarness to store information specific to a PMAC
    coordinate system

    """

    def __init__(self, cs_num):

        self.cs_number = cs_num

        self.motor_map = {}
        self.axis_map = {}

        self.max_velocities = {'x': 0, 'y': 0, 'z': 0,
                               'u': 0, 'v': 0, 'w': 0,
                               'a': 0, 'b': 0, 'c': 0}

    def add_motor_assignment(self, motor, axis, scaling):
        """
        Add motor assignment to CS motor and axis maps

        Args:
            motor(int): Motor number e.g. 1, 2, 3, ...
            axis(str): Axis specifier e.g. X, Y, Z, U, V, ...
            scaling(int): Scale factor between axis and motor definitions

        """

        self.motor_map[str(motor)] = (axis, scaling)
        self.axis_map[str(axis)] = (motor, scaling)

    def set_max_velocities(self, velocities):
        """
        Set the maximum velocities in EGUs for the axes in the coordinate system

        Args:
            velocities(list(int)): Max velocities in cts, read from PMAC

        """

        # Assign motor max_velocities to corresponding axes
        for motor_num, (axis, scaling) in self.motor_map.iteritems():
            self.max_velocities[axis.lower()] = float(velocities[int(motor_num) - 1]) / scaling
