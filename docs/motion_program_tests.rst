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

Creating A Scan
~~~~~~~~~~~~~~~

If in doubt, or even if not, I would recommend just copying a current scan and changing it to suit your needs; they are all quite similar. The following will give an idea of how to write one from scratch.

The test scans have three main stages:
    * Initialising the Pmac: Create PmacTestHarness instance, assign motors, reset buffers, set axes
    * Generating points: Use ScanGen() to create and format point set, send half-buffer of points, set *both* buffer fills (one will be zero), set initial coordinates
    * Running the scan: Run motion program, a while(status == 1) loop with a check for buffer swap that will fill a buffer and/or a print out of Pmac variables (e.g. TotalPoints)

Things that *must* be done are as follows:
    * Instantiate PmacTestHarness with correct IP address
    * Assign motors to axes in a coordinate system
    * Set the Axes value (motors won't move unless activated via the Axes value)
    * Generate and format points
        * If your point set is longer than the half-buffer, grab a buffer of points
    * Fill current half-buffer
    * Set buffer fill (motion program while loop depends on this value)
    * Set previous buffer write (if you are using more than one half-buffer)
    * Run motion program

If you do these, the program should, at least, run and move motors. It will still work if Pmac writing steps (e.g. assigning motors) are done, correctly, elsewhere.

I also recommend the following, but they may not be necessary in certain circumstances:
    * Force abort (allows program to be restarted while running)
    * Reset the buffers (so you know there aren't old points stored; it is easier to tell if, for example, you set the buffer fill higher than your number of points if it moves to zero than if it moves to wrong coordinates)
    * Set other buffer fill (the Pmac should know if it is empty, it may try to use bad points if still set from a previous run)
    * If using more than one half-buffer, only fill the first before running the motion program. If you only want two half-buffers, it will still work if you fill both and then run, but the idea is that buffers are filled while the program runs. It will be quicker to start and is just as simple.

You may want to add a new point set type; this should be done in the TrajectoryScanGenerator class; the only change to the driver should be the generate_*_points call (and maybe a grab_buffer_of_points call, if the point set size changes). Point sets should be formatted as a dictionary with keys for the axes (e.g. 'x', 'y') corresponding to a list of points. The 'time' value should itself be a dictionary with 'subroutine', 'vel_mode' and 'time_val' entries. The format_point_set can then convert the point set, in place, to motion program coordinates. If your point set is longer than the buffer length you can use the grab_buffer_of_points function to get points and keep track of the current position in the point set (see circle_scan).

PMAC Test
---------

These are tests that do not utilise the motion program, but depends on the hardware of, and network connection to, the PMAC. It tests things such as how long it takes to fill a buffer of points for different numbers of axes and the maximum velocities of the motors. It also has a check to verify the PMAC-Float conversion function; this writes a PMAC float to an address, assigns an M-Variable to that address, which the PMAC will perform its conversion back to a decimal-base for, then reads the variable and compares it to what was supposed to be sent.
