#!/usr/bin/python
# Copyright 2023 David Doerner (ddorner@kth.se)

# Bezier implementation from
# https://towardsdatascience.com/b%C3%A9zier-curve-bfffdadea212

from __future__ import division, print_function

import numpy as np
import matplotlib.pyplot as plt
import os 

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import tf


class SimplePathPlanner(object):
    def __init__(self, name):

        # External Parameter
        self.loop_freq = rospy.get_param("~loop_freq", 20)


        # Init
        self.start = np.array([0., 0., np.deg2rad(0)])
        self.goal = np.array([0., 0., np.deg2rad(90)])

        self.eps = 1.
        
        self.lenControlPoints = 3

        self.controlPoints = {}
        self.controlPoints["x"] = np.zeros(self.lenControlPoints)
        self.controlPoints["y"] = np.zeros(self.lenControlPoints)
        self.controlPoints["theta"] = np.zeros(self.lenControlPoints)

        self.path = {}
        self.path["x"] = []
        self.path["y"] = []

        self.calculatedPath = False
        self.currentIndex = 0

        self.currentWaypoint = [0., 0.]


        # Topics
        dockingStationEstimateTopic = rospy.get_param("~dockingStationEstimationTopic")
        stateEstimateTopic = rospy.get_param("~stateEstimationTopic")
        waypointTopic = rospy.get_param("~waypointTopic")

        # Subscribers
        rospy.Subscriber(dockingStationEstimateTopic, PoseWithCovarianceStamped, self.dockingStationEstimateCallback)
        rospy.Subscriber(stateEstimateTopic, PoseWithCovarianceStamped, self.stateEstimateCallback)

        # Publisher
        self.waypointPub = rospy.Publisher(waypointTopic, PoseWithCovarianceStamped, queue_size=1)

        rate = rospy.Rate(self.loop_freq) 

        # Run
        while not rospy.is_shutdown():
            # 1. Check if you need to calculate a plan
            planNeeded = self.requirePlan()
            # 1.1 Yes: Calculate Plan
            if planNeeded:
                print("Planning")
                self.calculatePathSegments()
                self.calculateBezierCurve()
                self.calculatedPath = True

                # Plot Path -> saves to file
                self.plotPath()
            # 1.2 No: continue

            # 2. Get current waypoint
            self.getCurrentWaypoint()

            # 3. Publish current waypoint
            self.publishCurrentWaypoint()

            # 4. Sleep
            rate.sleep()


    #region Callbacks
    def dockingStationEstimateCallback(self,DSEstim):
        DSPose = self.getEulerFromQuaternion(DSEstim.pose)
        self.goal[0] = DSPose[0]
        self.goal[1] = DSPose[1]
        self.goal[2] = DSPose[5]


    def stateEstimateCallback(self,estim):
        SAMPose = self.getEulerFromQuaternion(estim.pose)
        self.start[0] = SAMPose[0]
        self.start[1] = SAMPose[1]
        self.start[2] = SAMPose[5]

    def getEulerFromQuaternion(self, pose):
        # Pose is in ENU. The callback extracts the position and Euler angles.
        #
        # Pose in ENU 
        xENU = pose.pose.position.x
        yENU = pose.pose.position.y
        zENU = pose.pose.position.z

        eta0ENU = pose.pose.orientation.w
        eta1ENU = pose.pose.orientation.x
        eta2ENU = pose.pose.orientation.y
        eta3ENU = pose.pose.orientation.z
        
        rpyENU = euler_from_quaternion([eta1ENU,eta2ENU,eta3ENU,eta0ENU])
        
        rollENU = rpyENU[0]
        pitchENU = rpyENU[1]
        yawENU = rpyENU[2]

        states = np.array([xENU, yENU, zENU, rollENU, pitchENU, yawENU])

        return states
    #endregion

    #region functions
    def requirePlan(self):
        # Check if we need a motion plan:
        # Yes:  - no plan
        #       - Docking Station Estimate changed, e.g. new external waypoint. 
        #           -> give some good lee-way, otherwise there might be a lot of jumping...
        #       - TODO: Too far from current waypoint (How to measure?)
        if not self.calculatedPath:
            return True
        
        xDiff = self.goal[0] - self.path["x"][-1]
        yDiff = self.goal[1] - self.path["y"][-1]
        distanceCurrentGoalDSEstimate = np.linalg.norm([xDiff, yDiff])
                
        if distanceCurrentGoalDSEstimate > 1.:
            # print("Distance Goal to DS: {}".format(distanceCurrentGoalDSEstimate))
            # print("xDiff: {}, xDS Estimate: {}, xGoal: {}".format(xDiff, self.goal[0], self.path["x"][-1]))
            # print("yDiff: {}, yDS Estimate: {}, yGoal: {}".format(yDiff, self.goal[1], self.path["y"][-1]))
            return True
        
        return False


    def calculatePathSegments(self):
        self.controlPoints["x"]= np.array([self.start[0], self.goal[0], self.goal[0]])
        self.controlPoints["y"] = np.array([self.start[1], self.start[1], self.goal[1] - 1])

        yCalc = self.start[1] + (self.goal[0] - self.start[0]) * np.tan(self.start[2])

        self.controlPoints["y"][1] = yCalc


    def calculateBezierCurve(self):
        # Definition Bezier curve
        P = lambda t: (1 - t)**2 * np.array([self.controlPoints["x"][0],self.controlPoints["y"][0]]) \
                    + 2 * t * (1 - t) * np.array([self.controlPoints["x"][1],self.controlPoints["y"][1]]) \
                    + t**2 * np.array([self.controlPoints["x"][2],self.controlPoints["y"][2]])
        
        # Calculation Bezier curve
        points = np.array([P(t) for t in np.linspace(0, 1, 3)])
        self.path["x"], self.path["y"] = points[:,0], points[:,1]


    def getCurrentWaypoint(self):
        # Here we have the logic on how to populate self.currentWaypoint.
        distanceToWaypoint = self.calculateDistanceToWaypoint()
        if distanceToWaypoint < self.eps:
            if self.currentIndex < len(self.path["x"])-1:
                self.currentIndex += 1
        self.currentWaypoint = [self.path["x"][self.currentIndex], self.path["y"][self.currentIndex]]
        # print("Current Index: ", self.currentIndex)


    def calculateDistanceToWaypoint(self):
        xDiff = self.currentWaypoint[0] - self.start[0]
        yDiff = self.currentWaypoint[1] - self.start[1]
        distance = np.linalg.norm([xDiff, yDiff])
        return distance


    def publishCurrentWaypoint(self):        
        try:
            publishWaypoint = PoseWithCovarianceStamped()
            publishWaypoint.header.frame_id = 'map'
            publishWaypoint.header.stamp = rospy.Time(0)
            publishWaypoint.pose.pose.position.x = self.currentWaypoint[0]
            publishWaypoint.pose.pose.position.y = self.currentWaypoint[1]
            publishWaypoint.pose.pose.position.z = 0
            publishWaypoint.pose.pose.orientation.w = 0
            publishWaypoint.pose.pose.orientation.x = 0
            publishWaypoint.pose.pose.orientation.y = 0
            publishWaypoint.pose.pose.orientation.z = 0

            self.waypointPub.publish(publishWaypoint)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("map frame not available yet")
            pass


    def plotPath(self):
        fig, ax = plt.subplots()

        ax.plot(self.goal[0], self.goal[1], 'o', color='red', label='DS')
        ax.plot(self.controlPoints["x"], self.controlPoints["y"], label='path')
        ax.plot(self.controlPoints["x"][0] ,self.controlPoints["y"][0], 's', color='C2', markersize=10, label='start')
        ax.plot(self.controlPoints["x"][1] ,self.controlPoints["y"][1], 'o', color='C2', markersize=10)
        ax.plot(self.controlPoints["x"][-1] ,self.controlPoints["y"][-1], '*', color='C2', markersize=10, label='end')
        ax.plot(self.path["x"], self.path["y"], 'x')

        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('simple path')
        ax.legend()

        home_dir = os.path.expanduser('~')  
        plt.savefig(home_dir + '/catkin_ws/src/visual_feedback/fig/bezierPath.png', dpi=300)
    #endregion

if __name__ == "__main__":
    rospy.init_node("SimplePathPlanner")
    pathPlanner = SimplePathPlanner(rospy.get_name())

    