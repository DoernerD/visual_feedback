#!/usr/bin/python3
# Copyright 2023 David Doerner (ddorner@kth.se)

from __future__ import division, print_function

import numpy as np
import matplotlib.pyplot as plt
import os 

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import tf

class SimplePathPlanner(object):
    def __init__(self, name):

        # Init
        # map frame
        self.start = np.array([0., 0., np.deg2rad(0)])
        self.goal = np.array([0., 0., np.deg2rad(90)])
        self.target = np.array([0., 0., 0.])

        # sam base_link frame
        self.goalBase = np.array([0., 0., 0.])
        self.targetBase = np.array([0., 0., 0.])

        self.receivedGoal = False

        self.eps = 1.
        
        self.lenControlPoints = 3

        self.controlPoints = {}
        self.controlPoints["x"] = np.zeros(self.lenControlPoints)
        self.controlPoints["y"] = np.zeros(self.lenControlPoints)
        self.controlPoints["theta"] = np.zeros(self.lenControlPoints)

        self.controlPointsMap = {}
        self.controlPointsMap["x"] = np.zeros(self.lenControlPoints)
        self.controlPointsMap["y"] = np.zeros(self.lenControlPoints)
        self.controlPointsMap["theta"] = np.zeros(self.lenControlPoints)

        self.path = {}
        self.path["x"] = []
        self.path["y"] = []

        self.pathMap = {}
        self.pathMap["x"] = np.zeros(self.lenControlPoints)
        self.pathMap["y"] = np.zeros(self.lenControlPoints)

        self.calculatedPath = False
        self.currentIndex = 0

        self.currentWaypoint = [0., 0.]

        # TF tree listener
        # TODO: Put that into the corresponding ROS node
        # self.listener = tf.TransformListener()
        # self.base_frame = 'sam/base_link/estimated'

    #region functions
    def requirePlan(self):
        # Check if we need a motion plan:
        # Yes:  - no plan
        #       - Docking Station Estimate changed, e.g. new external waypoint. 
        #           -> give some good lee-way, otherwise there might be a lot of jumping...
        #       - TODO: Too far from current waypoint (How to measure?)
        if not self.calculatedPath:
            print("No plan yet")
            return True
        
        xDiff = self.targetBase[0] - self.path["x"][-1]
        yDiff = self.targetBase[1] - self.path["y"][-1]
        distanceCurrentGoalDSEstimate = np.linalg.norm([xDiff, yDiff])
                
        if distanceCurrentGoalDSEstimate > 0.5:
            print("Goal moved")
            return True
        
        thetaDiff = self.goalBase[2] - self.controlPoints["theta"][-1]
        if abs(thetaDiff) > 0.175/2: # 5deg change
            print("Goal rotated")
            return True
        
        return False


    def inFeasibleRegion(self):
        return True
        if self.goalBase[0] < 0:
            rospy.logwarn("Not facing docking station, xDS: {}".format(self.goalBase[0]))
            return False

        if abs(self.goalBase[2]) < np.pi/2:
            rospy.logwarn("Wrong angle, phiDS: {}".format(abs(self.goalBase[2])))
            return False

        return True

        
    def calculatePathSegments(self):
        # Calculate Bezier Control Point (second array entry)
        phiPrime = self.goalBase[2] - np.pi/2   # 90deg offset for the rotation. sign doesn't matter here, bc. tan is periodic with pi.
        xPrime = self.targetBase[1] * np.tan(phiPrime)
        self.controlPoints["x"]= np.array([0, self.targetBase[0] + xPrime, self.targetBase[0]])
        self.controlPoints["y"] = np.array([0, 0,  self.targetBase[1]])
        self.controlPoints["theta"] = np.array([0., 0., self.goalBase[2]])
        
        # Transform into map frame
        # TODO: This is in the ROS node again
        # for pt in range(len(self.controlPoints["x"])):
        #     controlpointPose = self.waypoint2pose([self.controlPoints["x"][pt], self.controlPoints["y"][pt]])
        #     controlpointMap = self.tf2Map(controlpointPose)
        #     self.controlPointsMap["x"][pt], self.controlPointsMap["y"][pt] = controlpointMap.pose.position.x, controlpointMap.pose.position.y


    def calculateBezierCurve(self):
        # Definition Bezier curve
        P = lambda t: (1 - t)**2 * np.array([self.controlPoints["x"][0],self.controlPoints["y"][0]]) \
                    + 2 * t * (1 - t) * np.array([self.controlPoints["x"][1],self.controlPoints["y"][1]]) \
                    + t**2 * np.array([self.controlPoints["x"][2],self.controlPoints["y"][2]])
        
        # Calculation Bezier curve in sam/base_link
        points = np.array([P(t) for t in np.linspace(0, 1, 3)])
        self.path["x"], self.path["y"] = points[:,0], points[:,1]        

        # Transform into map frame
        # TODO: This is in the ROS node again
        # for pt in range(len(self.path["x"])):
        #     waypointPose = self.waypoint2pose([self.path["x"][pt], self.path["y"][pt]])
        #     waypointMap = self.tf2Map(waypointPose)
        #     self.pathMap["x"][pt], self.pathMap["y"][pt] = waypointMap.pose.position.x, waypointMap.pose.position.y


    def getCurrentWaypoint(self):
        # Here we have the logic on how to populate self.currentWaypoint.
        distanceToWaypoint = self.calculateDistanceToWaypoint()
        if distanceToWaypoint < self.eps:
            if self.currentIndex < len(self.path["x"])-1:
                self.currentIndex += 1
        self.currentWaypoint = [self.path["x"][self.currentIndex], self.path["y"][self.currentIndex]]


    def calculateDistanceToWaypoint(self):
        xDiff = self.currentWaypoint[0] - self.start[0]
        yDiff = self.currentWaypoint[1] - self.start[1]
        distance = np.linalg.norm([xDiff, yDiff])
        return distance    
    
    # TODO: Put that in the ROS Node
    # def waypoint2pose(self, waypoint):
    #     poseWaypoint = PoseWithCovarianceStamped()
    #     poseWaypoint.header.frame_id = self.base_frame
    #     poseWaypoint.header.stamp = rospy.Time(0)
    #     poseWaypoint.pose.pose.position.x = waypoint[0]
    #     poseWaypoint.pose.pose.position.y = waypoint[1]
    #     poseWaypoint.pose.pose.position.z = 0
    #     poseWaypoint.pose.pose.orientation.w = 0
    #     poseWaypoint.pose.pose.orientation.x = 0
    #     poseWaypoint.pose.pose.orientation.y = 0
    #     poseWaypoint.pose.pose.orientation.z = 0

    #     return poseWaypoint

    #region Plotting
    def plotPath(self):
        fig, ax = plt.subplots()
        plt.gca().set_aspect('equal')

        # Path Planning
        ax.plot(self.goal[0], self.goal[1], 'o', color='red', label='DS')
        ax.plot(self.controlPointsMap["x"], self.controlPointsMap["y"], label='path')
        ax.plot(self.controlPointsMap["x"][0] ,self.controlPointsMap["y"][0], 's', color='C2', markersize=10, label='start')
        ax.plot(self.controlPointsMap["x"][1] ,self.controlPointsMap["y"][1], 'o', color='C2', markersize=10)
        ax.plot(self.controlPointsMap["x"][-1] ,self.controlPointsMap["y"][-1], '*', color='C2', markersize=10, label='end')
        ax.plot(self.pathMap["x"], self.pathMap["y"], 'x')


        # # Coordinate visualization
        # originDS = [self.goal[0], self.goal[1]]
        # xAxisDS, yAxisDS = self.calculateOrientationAxes(self.goal[2], 0.5)
        
        # originSAM = [self.start[0], self.start[1]]
        # xAxisSAM, yAxisSAM = self.calculateOrientationAxes(self.start[2], 0.5)

        # # DS Axes
        # plt.arrow(*originDS, *xAxisDS, width=0.01, color='r')
        # plt.arrow(*originDS, *yAxisDS, width=0.01, color='g')
        # plt.arrow(*self.target, *xAxisDS, width=0.01, color='r')
        # plt.arrow(*self.target, *yAxisDS, width=0.01, color='g')

        # # SAM Coordinate axes
        
        # plt.arrow(*originSAM, *xAxisSAM, width=0.01, color='r')
        # plt.arrow(*originSAM, *yAxisSAM, width=0.01, color='g')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('map')
        ax.legend()

        home_dir = os.path.expanduser('~')  
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/bezierPathMap.png', dpi=300)


    def plotPosition(self):
        fig, ax = plt.subplots()
        plt.gca().set_aspect('equal')

        # Path Planning
        ax.plot(self.goal[0], self.goal[1], 'o', color='red', label='DS')
        ax.plot(self.start[0], self.start[1], 'o', color='b', label='SAM')

        # Coordinate visualization
        originDS = [self.goal[0], self.goal[1]]
        xAxisDS, yAxisDS = self.calculateOrientationAxes(self.goal[2], 0.5)
        
        originSAM = [self.start[0], self.start[1]]
        xAxisSAM, yAxisSAM = self.calculateOrientationAxes(self.start[2], 0.5)

        # plt.arrow(*originDS, *xAxisDS, width=0.01, color='r')
        # plt.arrow(*originDS, *yAxisDS, width=0.01, color='g')
        # plt.arrow(*originSAM, *xAxisSAM, width=0.01, color='r')
        # plt.arrow(*originSAM, *yAxisSAM, width=0.01, color='g')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('simple path')
        ax.legend()

        home_dir = os.path.expanduser('~')  
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/position.png', dpi=300)


    def plotTFSamBase(self):
        
        fig, ax = plt.subplots()
        plt.gca().set_aspect('equal')

        # Path segments
        ax.plot(self.controlPoints["x"], self.controlPoints["y"], label='path')
        ax.plot(self.controlPoints["x"][0] ,self.controlPoints["y"][0], 's', color='C2', markersize=10, label='start')
        ax.plot(self.controlPoints["x"][1] ,self.controlPoints["y"][1], 'o', color='C2', markersize=10)
        ax.plot(self.controlPoints["x"][-1] ,self.controlPoints["y"][-1], '*', color='C2', markersize=10, label='end')
        ax.plot(self.path["x"], self.path["y"], 'x')

        # Targets
        ax.plot(self.goalBase[0], self.goalBase[1], 'o', color='red', label='DS')
        ax.plot(self.targetBase[0], self.targetBase[1], 'o', color='green', label='target')
        ax.plot(0, 0, 'o', color='b', label='SAM')

        # Coordinate visualization
        originDS = [self.goalBase[0], self.goalBase[1]]
        xAxisDS, yAxisDS = self.calculateOrientationAxes(self.goalBase[2], 0.5)

        originTargetBase = [self.targetBase[0], self.targetBase[1]]
        
        originSAM = [0, 0]
        xAxisSAM, yAxisSAM = self.calculateOrientationAxes(0, 0.5)
        
        # plt.arrow(*originDS, *xAxisDS, width=0.01, color='r')
        # plt.arrow(*originDS, *yAxisDS, width=0.01, color='g')
        # plt.arrow(*originTargetBase, *xAxisDS, width=0.01, color='r')
        # plt.arrow(*originTargetBase, *yAxisDS, width=0.01, color='g')

        # plt.arrow(*originTargetBase, *xAxisSAM, width=0.01, color='r', alpha=0.5)
        # plt.arrow(*originTargetBase, *yAxisSAM, width=0.01, color='g', alpha=0.5)
        # plt.arrow(*originSAM, *xAxisSAM, width=0.01, color='r')
        # plt.arrow(*originSAM, *yAxisSAM, width=0.01, color='g')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('sam/base_link')
        ax.legend()

        home_dir = os.path.expanduser('~')  
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/bezierPathSAMFrame.png', dpi=300)


    def calculateOrientationAxes(self, theta, axisLen):
        xAxisDS = [axisLen, 0]
        yAxisDS = [0, axisLen]
        rotation = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
            ])
        
        xAxisDSPrime = rotation.dot(xAxisDS)
        yAxisDSPrime = rotation.dot(yAxisDS)

        return xAxisDSPrime, yAxisDSPrime
    #endregion

    def printStates(self):
        print("SAM Pose (map): x = {}, y = {}, phiSAM = {}".format(self.start[0], self.start[1], self.start[2]))
        print("DS Pose (map): x = {}, y = {}, phiDS = {}".format(self.goal[0], self.goal[1], self.goal[2]))

    #endregion

    #region Outward-facing stuff
    # The callbacks set the corresponding variables here
    # Then you have the corresponding functions that return whatever and the ROS node can convert 
    # it into the proper frame etc.

    #endregion