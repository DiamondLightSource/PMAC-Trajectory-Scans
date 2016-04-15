PMAC Test Harness
=================

PmacTestHarness is a class that can connect to a PMAC over ethernet, read a write variables and addresses, and run the trajectory_scan motion program. Many of these functions are specific to the variable definitions in the motion program. It inherits from PmacEthernetInterface in dls_pmacremote.py, which means it also has some generic commands like jogging motors and reading Pmac hardware details.

PmacCoordinateSystem is a helper class that holds information for a single coordinate system on the PMAC, such as its axis-motor definitions and maximum velocities of the motors. This simplifies the generation of point sets by calculating how the coordinates will convert to the actual motion of the motors.

TrajectoryScanGenerator is a class to generate point sets that can be used to command the motion program. The point sets are generated in a generic, readable, format to start with and then formatted into something that can be understood by the motion program. These point sets can then be written to the PMAC by PmacTestHarness.

.. module:: PmacTestHarness

.. autoclass:: PmacTestHarness
    :members: