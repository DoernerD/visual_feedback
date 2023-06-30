#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)
"""
Unit test module for simple path planner
"""

import unittest

import numpy as np

from src.simple_path_planner import SimplePathPlanner

class TestSimplePathPlanner(unittest.TestCase):
    """
    Unit tests for the simple path planner
    """

    @classmethod
    def setUpClass(self):
        self.simple_path_planner =  SimplePathPlanner("SimplePathPlanner")

    # Test Cases
    #region testRequirePlan
    def test_require_plan_no_plan(self):
        """
        Test case when no plan existing
        """
        self.simple_path_planner.path_calculated = False

        self.assertTrue(self.simple_path_planner.check_require_plan())

    def test_require_plan_goal_moved(self):
        """
        Test case plan when goal moved
        """
        self.simple_path_planner.path_calculated = True

        self.simple_path_planner.target_base = np.array([0, 0])
        self.simple_path_planner.target_base[0] = 1.
        self.simple_path_planner.target_base[1] = 1. 

        self.simple_path_planner.path["x"] = np.array([0, 0])
        self.simple_path_planner.path["y"] = np.array([0, 0])

        self.simple_path_planner.path["x"][-1] = 2. 
        self.simple_path_planner.path["y"][-1] = 2. 

        self.assertTrue(self.simple_path_planner.check_require_plan())

    def test_require_plan_goal_rotated(self):
        """
        Test planning wehn goal rotated
        """
        self.simple_path_planner.path_calculated = True

        self.simple_path_planner.target_base = np.array([0, 0])
        self.simple_path_planner.target_base[0] = 1.
        self.simple_path_planner.target_base[1] = 1. 

        self.simple_path_planner.path["x"] = np.array([0, 0])
        self.simple_path_planner.path["y"] = np.array([0, 0])

        self.simple_path_planner.path["x"][-1] = 1. 
        self.simple_path_planner.path["y"][-1] = 1.

        self.simple_path_planner.goal_base = np.zeros(3)
        self.simple_path_planner.goal_base[2] = 0

        self.simple_path_planner.control_points["theta"] = np.zeros(3)
        self.simple_path_planner.control_points["theta"][-1] = 1

        self.assertTrue(self.simple_path_planner.check_require_plan())

    def test_require_plan_false(self):
        """
        Test when plan already exists. Returns False
        """
        self.simple_path_planner.path_calculated = True

        self.simple_path_planner.target_base = np.array([0, 0])
        self.simple_path_planner.target_base[0] = 1.
        self.simple_path_planner.target_base[1] = 1.

        self.simple_path_planner.path["x"] = np.array([0, 0])
        self.simple_path_planner.path["y"] = np.array([0, 0])

        self.simple_path_planner.path["x"][-1] = 1.
        self.simple_path_planner.path["y"][-1] = 1.

        self.simple_path_planner.goal_base = np.zeros(3)
        self.simple_path_planner.goal_base[2] = 0

        self.simple_path_planner.control_points["theta"] = np.zeros(3)
        self.simple_path_planner.control_points["theta"][-1] = 0

        self.assertFalse(self.simple_path_planner.check_require_plan())
    #endregion

    #region testInFeasibleRegion
    # Needs to be expanded once that funcation is fixed.
    def test_in_feasible_region_true(self):
        """
        Test if we're in feasable region
        """
        self.assertTrue(self.simple_path_planner.check_in_feasible_region())
    #endregion

    def test_calculate_path_segment(self):
        """
        Test calculation of third control point
        This test is a bit weird as I calculate the results a priori to compare against them. 
        Everything is in /sam/base_link frame
        """
        self.simple_path_planner.goal_base = np.array([3., 10., 0.2915 + np.pi/2])
        self.simple_path_planner.target_base = np.array([2., 10., 0.2915 + np.pi/2])

        self.simple_path_planner.calculate_path_segments()

        self.assertAlmostEqual(self.simple_path_planner.control_points["x"].all(),
                               np.array([0., 2 + 0.3, 10.]).all(), msg="x calculation failed")
        self.assertAlmostEqual(self.simple_path_planner.control_points["y"].all(),
                               np.array([0., 0, 10.]).all(), msg="y calculation failed")
        self.assertAlmostEqual(self.simple_path_planner.control_points["theta"].all(),
                               np.array([0., 0, 0.2915 + np.pi/2]).all(), msg="theta calculation failed")

    def test_calculate_bezier_curve(self):
        """
        Test the calculation of the bezier curve
        """
        self.simple_path_planner.control_points["x"] = np.array([0, 4, 4])
        self.simple_path_planner.control_points["y"] = np.array([0, 0, 5])

        self.simple_path_planner.calculate_bezier_curve()

        self.assertEqual(self.simple_path_planner.path["x"].all(),
                         np.array([0., 3., 4.]).all(), msg="x calculation wrong")
        self.assertEqual(self.simple_path_planner.path["y"].all(),
                         np.array([0., 1.25, 5.]).all(), msg="y calculation wrong")

if __name__ == "__main__":
    unittest.main()
    