.. _motion_program_tests:

====================
Motion Program Tests
====================

These test scripts require a functional PMAC with the trajectory_scan motion program downloaded. They are system_tests in that nothing is mocked out and it will carry out processes just as they will need to work in the final implementation.

Trajectory Scan Test
--------------------

This test script has some simple tests to check that the PMAC and motion program can be interfaced with correctly. It checks that the the PmacTestHarness can connect to the PMAC, assign motors to a coordinate system, read and set variables and perform simple scans.

Trajectory Scan Driver
----------------------

This script contains some test scans and interacts with the PMAC like EPICS will. The current examples generate a set of points (e.g. circle or sine wave) fills one of the buffers and runs the trajectory scan. It will then run alongside the PMAC, reading the current buffer and filling the idle buffer when the PMAC swaps over. It tracks the status variables from the PMAC, such as total number of points scanned and position, and ends the program if an error occurs. These scans can run indefinitely.

PMAC Test
---------

These are tests that do not depend on the motion program but the hardware and network connection to the brick. It tests things such as how long it takes to fill a buffer of points for different numbers of axes, the maximum velocities of the motors and it also has a check to verify the PMAC-Float conversion function. This writes PMAC-Float to an address, assigns an M-Variable to that address, which the PMAC will perform it's conversion back to a decimal-base for, and then reads the variable and compares it to what was supposed to be sent.
