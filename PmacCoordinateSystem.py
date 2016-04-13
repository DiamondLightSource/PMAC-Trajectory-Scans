

class PmacCoordinateSystem(object):

    def __init__(self, cs_num):

        self.cs_number = cs_num

        self.axis_map = {}

        self.max_velocities = {'x': 0, 'y': 0, 'z': 0,
                               'u': 0, 'v': 0, 'w': 0,
                               'a': 0, 'b': 0, 'c': 0}

    def add_motor_assignment(self, motor, axis, scaling):

        self.axis_map[str(motor)] = (axis, scaling)

    def set_max_velocities(self, velocities):

        self.max_velocities = {'x': velocities[0], 'y': velocities[1], 'z': velocities[2],
                               'u': velocities[3], 'v': velocities[4], 'w': velocities[5],
                               'a': velocities[6], 'b': velocities[7], 'c': velocities[8]}
