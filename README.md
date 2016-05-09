[![Build Status](https://api.travis-ci.org/dls-controls/PMAC-Trajectory-Scans.svg)](https://travis-ci.org/dls-controls/PMAC-Trajectory-Scans)
[![Coverage Status](https://coveralls.io/repos/github/dls-controls/PMAC-Trajectory-Scans/badge.svg?branch=master)](https://coveralls.io/github/dls-controls/PMAC-Trajectory-Scans?branch=master)
[![Code Health](https://landscape.io/github/dls-controls/PMAC-Trajectory-Scans/master/landscape.svg?style=flat)](https://landscape.io/github/dls-controls/PMAC-Trajectory-Scans/master)
[![ReadTheDocs](https://readthedocs.org/projects/pmac-trajectory-scans/badge/?version=latest)](http://pmac-trajectory-scans.readthedocs.org)

# PMAC-Trajectory-Scans

To run the scripts and tests you must add the root of PMAC-Trajectory-Scans to your python path (export PYTHONPATH=/path/to/PMAC-Trajectory-Scans:$PYTHONPATH) and add scanpointgenerator to the the root of the module. Tests can be run using `nosetests -v test_harness_tests` from the root. motion_program_tests should be checked before running as they require a PMAC and have a hard-coded IP address. The only function for some of the tests is to print out a result, so these should use the -s flag.