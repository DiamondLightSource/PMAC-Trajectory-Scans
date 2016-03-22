from PmacTestHarness import PmacTestHarness
import unittest


class InitialisationTest(unittest.TestCase):

    def setUp(self):
        self.pmac = PmacTestHarness("172.23.243.169")
        self.pmac.assign_motors()

    def test_given_valid_axes_then_set_axis_values(self):
        self.pmac.set_axes(511)
        self.pmac.run_motion_program(1)

        self.assertEqual(self.pmac.read_variable("P4003"), "0")
        self.assertEqual(self.pmac.read_variable("P4101"), "1")
        self.assertEqual(self.pmac.read_variable("P4102"), "1")
        self.assertEqual(self.pmac.read_variable("P4103"), "1")
        self.assertEqual(self.pmac.read_variable("P4104"), "1")
        self.assertEqual(self.pmac.read_variable("P4105"), "1")
        self.assertEqual(self.pmac.read_variable("P4106"), "1")
        self.assertEqual(self.pmac.read_variable("P4107"), "1")
        self.assertEqual(self.pmac.read_variable("P4108"), "1")
        self.assertEqual(self.pmac.read_variable("P4109"), "1")
