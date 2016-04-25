.. _trajectory_scan:

==============================
Trajectory Scan Motion Program
==============================

PMAC Requirements
-----------------

 * P-Variables - 4000..4150
 * Q-Variables - 1..18
 * M-Variables - 4000..4050
 * User memory of 20 times the required buffer length

.. _program_design:
Program Design
--------------

The motion program operates using two buffers, one of which can be scanned through by the PMAC while the other can be filled with points by EPICS. The idea is that a scan of any length can be sent from the data acquisition layer to the EPICS layer and can then be run continuously as if there is no limit to the PMAC memory.

The program keeps 3 points per axis (plus time) accessible at any time; Prev\_* and Current\_* values are stored in P-Variables and Next\_* values are stored in an M variable. The M variable is used to iterate through the user memory addresses using pointers (_Adr values) to the M Variable definitions. Before each increment, the 3-point-buffers are shifted Current -> Prev and then Next -> Current. This allows the PMAC to calculate the required trajectory for each Current\_* point

Dynamic Velocity Calculation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The motion program implements a dynamic velocity calculation to allow more generic trajectory scans to be demanded from higher level software. By default (0) the velocity will be calculated using the three currently stored coordinates, i.e. the previous->next vector:

       Prev Curr Next
         |    |    |
    X    X    X    X    X
         |--------->

If VelMode is set to 1 it will calculate the previous->current vector:

       Prev Curr Next
         |    |    |
    X    X    X    X    X
         |---->

If VelMode is set to 2 it will calculate the current->next vector:

       Prev Curr Next
         |    |    |
    X    X    X    X    X
              |---->

This allows the user to set exit and entrance velocity vectors from an area of interest and the PMAC will interpolate the smoothest curve, given the time allowed time for the move, between the points. The user then doesn't have to worry about adding points to ensure smooth turnarounds.

Quirks of PMAC
~~~~~~~~~~~~~~

The program has a structure that may be a little confusing in places. This is, in part, due to the quirks of PMAC (see lookahead and double jump-back). This section will explain some sections of code that have had to be written in a certain way to overcome these factors.

The buffer indexing runs from 0 to N-1, but appears in EPICS as 1 to N. This is because the move at the move command the PMAC lookahead sees the CurrentIndex increment line and sets the variable. This was left as is because it makes more sense to run from 1 to N, so the offset would have had to be introduced elsewhere anyway.

Their was a problem with the buffer swapping where the swap would occur before the final increment to store the last two values of the previous buffer. This appeared to be caused by a combination of two consecutive EndWhile statements without a move and the lookahead of the PMAC. This is why the buffer address swap is at the start of the outer while loop, with a check to stop it running on the first loop.

.. _epics_api:
Epics API
---------

There are various P-Variables in the motion program that are relevant to EPICS, these are:

Read Variables
~~~~~~~~~~~~~~
    * Status (P4001) - The current state of the motion program i.e. 0: Initialised, 1: Active, 2: Idle, 3: Error
    * BufferLength (P4004) - The length of a single buffer e.g. len(AX)
    * TotalPoints (P4005) - The total number of points that the PMAC has scanned through
    * CurrentIndex (P4006) - The current point in the buffer
    * CurrentBuffer (P4007) - The specifier for the current half-buffer i.e. 0: Buffer A, 1: Buffer B
    * BufferAdr_A/BufferAdr_B (P4008/9) - The starting address in the PMAC user memory for buffer A/B
    * Error (P40015) - Error code corresponding to Status = 3; 0: No error, 1: Invalid axes value, 2: Move time of 0, 3: Following error/ Run-time error

Write Variables
~~~~~~~~~~~~~~~
    * Abort (P4002) - A trigger to abort the scan (will move to current target point)
    * Axes (P4003) - A bit mask to specify which axes are to be used in the scan. X = 256, Y = 128, ..., C = 1 e.g. for axes X, Y, U and V the value would be 256 + 128 + 32 + 16 = 432
    * BufferFill_A/BufferFill_B (P4011/12) - The number of points written into buffer A/B

Buffer Filling
~~~~~~~~~~~~~~

EPICS must write the position coordinates as 48-bit PMAC floats (with a write L command). These are a custom delta tau format and must be written in hex. PmacTestHarness has a converter for this. The time coordinates, user and velocity mode values must be written into a single address (also with a write L) in the following format:

    4 4 4 4 4 4 4 4 3 3 3 3 3 3 3 3   3 3 2 2   2 2 2 2   2 2 2 2 1 1 1 1 1 1 1 1 1 1
     7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2   1 0 9 8   7 6 5 4   3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
     _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _   _ _ _ _   _ _ _ _   _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
     <------------Unused----------->     User    VelMode   <---------------------Time--------------------->

Time will then be read from the Y memory and User & VelMode will be read from the appropriate bits in the X memory. Time is the integer number of 1/4s of a milliseconds for the move (this must be written in hex), VelMode is 0, 1 or 2 as described in :ref:`program_design` and User is the number of the subroutine that should be run at the point.

