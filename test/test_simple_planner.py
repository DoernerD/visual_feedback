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
        self.simple_path_planner =  SimplePathPlanner()

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

        self.simple_path_planner.target_map = np.array([0, 0])
        self.simple_path_planner.target_map[0] = 1.
        self.simple_path_planner.target_map[1] = 1.

        self.simple_path_planner.path_map["x"] = np.array([0, 0])
        self.simple_path_planner.path_map["y"] = np.array([0, 0])

        self.simple_path_planner.path_map["x"][-1] = 2.
        self.simple_path_planner.path_map["y"][-1] = 2.

        self.assertTrue(self.simple_path_planner.check_require_plan())

    def test_require_plan_goal_rotated(self):
        """
        Test planning when goal rotated
        """
        self.simple_path_planner.path_calculated = True

        self.simple_path_planner.target_map = np.array([0, 0])
        self.simple_path_planner.target_map[0] = 1.
        self.simple_path_planner.target_map[1] = 1.

        self.simple_path_planner.path_map["x"] = np.array([0, 0])
        self.simple_path_planner.path_map["y"] = np.array([0, 0])

        self.simple_path_planner.path_map["x"][-1] = 1.
        self.simple_path_planner.path_map["y"][-1] = 1.

        self.simple_path_planner.goal_map = np.zeros(3)
        self.simple_path_planner.goal_map[2] = 0

        self.simple_path_planner.control_points_map["theta"] = np.zeros(3)
        self.simple_path_planner.control_points_map["theta"][-1] = 1

        self.assertTrue(self.simple_path_planner.check_require_plan())

    def test_require_plan_false(self):
        """
        Test when plan already exists. Returns False
        """
        self.simple_path_planner.path_calculated = True

        self.simple_path_planner.target_map = np.array([0, 0])
        self.simple_path_planner.target_map[0] = 1.
        self.simple_path_planner.target_map[1] = 1.

        self.simple_path_planner.path_map["x"] = np.array([0, 0])
        self.simple_path_planner.path_map["y"] = np.array([0, 0])

        self.simple_path_planner.path_map["x"][-1] = 1.
        self.simple_path_planner.path_map["y"][-1] = 1.

        self.simple_path_planner.goal_map = np.zeros(3)
        self.simple_path_planner.goal_map[2] = 0

        self.simple_path_planner.control_points_map["theta"] = np.zeros(3)
        self.simple_path_planner.control_points_map["theta"][-1] = 0

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

        self.assertAlmostEqual(self.simple_path_planner.control_points_base["x"].all(),
                               np.array([0., 2 + 0.3, 10.]).all(), msg="x calculation failed")
        self.assertAlmostEqual(self.simple_path_planner.control_points_base["y"].all(),
                               np.array([0., 0, 10.]).all(), msg="y calculation failed")
        self.assertAlmostEqual(self.simple_path_planner.control_points_base["theta"].all(),
                               np.array([0., 0, 0.2915 + np.pi/2]).all(), msg="theta calculation failed")

    def test_calculate_bezier_curve(self):
        """
        Test the calculation of the bezier curve
        """
        self.simple_path_planner.control_points_base["x"] = np.array([0, 4, 4])
        self.simple_path_planner.control_points_base["y"] = np.array([0, 0, 5])

        self.simple_path_planner.calculate_bezier_curve()

        self.assertEqual(self.simple_path_planner.path_base["x"].all(),
                         np.array([0., 3., 4.]).all(), msg="x calculation wrong")
        self.assertEqual(self.simple_path_planner.path_base["y"].all(),
                         np.array([0., 1.25, 5.]).all(), msg="y calculation wrong")
        
    def test_calculate_orientation_axes(self):
        """
        Test the calculation of the orientation of the axes
        """
        theta = np.pi/2
        axis_length = 1.

        x_axis_prime, y_axis_prime = self.simple_path_planner.calculate_orientation_axes(theta, axis_length)

        self.assertAlmostEqual(x_axis_prime[0], 0, msg="x axis x computation")
        self.assertAlmostEqual(x_axis_prime[1], 1, msg="x axis y computation")
        self.assertAlmostEqual(y_axis_prime[0], -1. , msg="y axis x computation")
        self.assertAlmostEqual(y_axis_prime[1], 0., msg="y axis y computation")

    def test_print_states(self):
        """
        Test the verbose function
        The test is a bit hacky, as there's no return statement for the print
        function. That's why there's the flag and should something fail, it'll catch the 
        exception.
        """
        self.simple_path_planner.start_map = np.linspace(0,2,3)
        self.simple_path_planner.goal_map = np.linspace(3,5,3)

        print_flag = False

        e = ''

        try:
            self.simple_path_planner.print_states()
            print_flag = True
        except Exception as e:
            pass

        self.assertTrue(print_flag, msg = e)


    def test_plot_path(self):
        """
        Test the plot path function
        """
        self.simple_path_planner.start_map = np.linspace(0,2,3)
        self.simple_path_planner.goal_map = np.linspace(3,5,3)
        self.simple_path_planner.target_map = np.linspace(6,8,2)

        self.simple_path_planner.control_points_map["x"] = np.linspace(0,2,3)
        self.simple_path_planner.control_points_map["y"] = np.linspace(0,2,3)

        print_flag = False

        error = ''

        try:
            self.simple_path_planner.plot_path()
            print_flag = True
        except Exception as error:
            print(error)
            pass

        self.assertTrue(print_flag, msg = "Plot not successful")


    def test_plot_position(self):
        """
        Test the plot position function
        """
        self.simple_path_planner.start_map = np.linspace(0,2,3)
        self.simple_path_planner.goal_map = np.linspace(3,5,3)
        
        print_flag = False

        error = ''

        try:
            self.simple_path_planner.plot_position()
            print_flag = True
        except Exception as error:
            print(error)
            pass

        self.assertTrue(print_flag, msg = "Plot not successful")


    def test_plot_tf_sam_base(self):
        """
        Test the plot position function
        """
        self.simple_path_planner.start_map = np.linspace(0,2,3)
        self.simple_path_planner.goal_map = np.linspace(3,5,3)
        self.simple_path_planner.target_map = np.linspace(6,8,2)
        self.simple_path_planner.goal_base = np.linspace(6,8,3)
        self.simple_path_planner.target_base = np.linspace(6,8,3)
        

        self.simple_path_planner.control_points_map["x"] = np.linspace(0,2,3)
        self.simple_path_planner.control_points_map["y"] = np.linspace(0,2,3)

        print_flag = False

        error = ''

        try:
            self.simple_path_planner.plot_tf_sam_base()
            print_flag = True
        except Exception as error:
            print(error)
            pass

        self.assertTrue(print_flag, msg = "Plot not successful")

if __name__ == "__main__":
    unittest.main()
    