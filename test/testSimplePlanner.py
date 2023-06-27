#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)

import unittest

import numpy as np

from src.simplePathPlanner import SimplePathPlanner

class TestSimplePathPlanner(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.simplePathPlanner =  SimplePathPlanner("SimplePathPlanner")

    # Test Cases
    '''
    Test cases for the individual functions
    '''
    #region testRequirePlan
    def testRequirePlanNoPlan(self):
        self.simplePathPlanner.calculatedPath = False

        self.assertTrue(self.simplePathPlanner.requirePlan())

    def testRequirePlanGoalMoved(self):
        self.simplePathPlanner.calculatedPath = True
        
        self.simplePathPlanner.targetBase = np.array([0, 0])
        self.simplePathPlanner.targetBase[0] = 1.
        self.simplePathPlanner.targetBase[1] = 1. 
        
        self.simplePathPlanner.path["x"] = np.array([0, 0])
        self.simplePathPlanner.path["y"] = np.array([0, 0])
        
        self.simplePathPlanner.path["x"][-1] = 2. 
        self.simplePathPlanner.path["y"][-1] = 2. 

        self.assertTrue(self.simplePathPlanner.requirePlan())

    def testRequirePlanGoalRotated(self):
        self.simplePathPlanner.calculatedPath = True
        
        self.simplePathPlanner.targetBase = np.array([0, 0])
        self.simplePathPlanner.targetBase[0] = 1.
        self.simplePathPlanner.targetBase[1] = 1. 
        
        self.simplePathPlanner.path["x"] = np.array([0, 0])
        self.simplePathPlanner.path["y"] = np.array([0, 0])
        
        self.simplePathPlanner.path["x"][-1] = 1. 
        self.simplePathPlanner.path["y"][-1] = 1.
        
        self.simplePathPlanner.goalBase = np.zeros(3)
        self.simplePathPlanner.goalBase[2] = 0
        
        self.simplePathPlanner.controlPoints["theta"] = np.zeros(3)
        self.simplePathPlanner.controlPoints["theta"][-1] = 1

        self.assertTrue(self.simplePathPlanner.requirePlan())

    def testRequirePlanFalse(self):
        self.simplePathPlanner.calculatedPath = True
        
        self.simplePathPlanner.targetBase = np.array([0, 0])
        self.simplePathPlanner.targetBase[0] = 1.
        self.simplePathPlanner.targetBase[1] = 1. 
        
        self.simplePathPlanner.path["x"] = np.array([0, 0])
        self.simplePathPlanner.path["y"] = np.array([0, 0])
        
        self.simplePathPlanner.path["x"][-1] = 1. 
        self.simplePathPlanner.path["y"][-1] = 1.
        
        self.simplePathPlanner.goalBase = np.zeros(3)
        self.simplePathPlanner.goalBase[2] = 0
        
        self.simplePathPlanner.controlPoints["theta"] = np.zeros(3)
        self.simplePathPlanner.controlPoints["theta"][-1] = 0

        self.assertFalse(self.simplePathPlanner.requirePlan())
    #endregion
    
    #region testInFeasibleRegion
    # Needs to be expanded once that funcation is fixed.
    def testInFeasibleRegionTrue(self):
        self.assertTrue(self.simplePathPlanner.inFeasibleRegion())
    #endregion

    def testCalculatePathSegment(self):
        # This test is a bit weird as I calculate the results a priori to compare against them. 
        # Everything is in /sam/base_link frame
        self.simplePathPlanner.goalBase = np.array([3., 10., 0.2915 + np.pi/2])
        self.simplePathPlanner.targetBase = np.array([2., 10., 0.2915 + np.pi/2])

        self.simplePathPlanner.calculatePathSegments()

        self.assertAlmostEqual(self.simplePathPlanner.controlPoints["x"].all(), np.array([0., 2 + 0.3, 10.]).all(), msg="x calculation failed")
        self.assertAlmostEqual(self.simplePathPlanner.controlPoints["y"].all(), np.array([0., 0, 10.]).all(), msg="y calculation failed")
        self.assertAlmostEqual(self.simplePathPlanner.controlPoints["theta"].all(), np.array([0., 0, 0.2915 + np.pi/2]).all(), msg="theta calculation failed")

if __name__ == "__main__":
    unittest.main()