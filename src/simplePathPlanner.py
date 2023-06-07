#!/usr/bin/python
# Copyright 2023 David Doerner (ddorner@kth.se)

# Bezier implementation from
# https://towardsdatascience.com/b%C3%A9zier-curve-bfffdadea212

from __future__ import division, print_function

import numpy as np
import matplotlib.pyplot as plt
import os 

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import tf


class SimplePathPlanner(object):
    def __init__(self, name):

        # External Parameter
        self.loop_freq = rospy.get_param("~loop_freq", 20)
        rate = rospy.Rate(self.loop_freq) 

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

        # Topics
        dockingStationEstimateTopic = rospy.get_param("~dockingStationEstimationTopic")
        stateEstimateTopic = rospy.get_param("~stateEstimationTopic")
        waypointTopic = rospy.get_param("~waypointTopic")

        # Subscribers
        rospy.Subscriber(dockingStationEstimateTopic, PoseWithCovarianceStamped, self.dockingStationEstimateCallback)
        rospy.Subscriber(stateEstimateTopic, PoseWithCovarianceStamped, self.stateEstimateCallback)

        # Publisher
        self.waypointPub = rospy.Publisher(waypointTopic, PoseWithCovarianceStamped, queue_size=1)

        # TF tree listener
        self.listener = tf.TransformListener()
        self.base_frame = 'sam/base_link'

        # Run
        while not rospy.is_shutdown():
            if self.receivedGoal:
                # 1. Check if you need to calculate a plan
                planNeeded = self.requirePlan()
                # 1.1 Yes: Calculate Plan
                if planNeeded:
                    print("Planning")
                    if self.inFeasibleRegion():
                        self.calculatePathSegments()
                        self.calculateBezierCurve()
                        self.calculatedPath = True

                        # Plot Path -> saves to file
                        self.plotPath()
                        self.plotTFSamBase()
                    else:
                        self.plotPosition()
                # 1.2 No: continue

                if self.calculatedPath:
                    # 2. Get current waypoint
                    self.getCurrentWaypoint()

                    # 3. Publish current waypoint
                    self.publishCurrentWaypoint()
            else:
                print("No goal available")

            # 4. Sleep
            rate.sleep()


    #region Callbacks
    def dockingStationEstimateCallback(self,DSEstim):
        # map frame
        DSPose = self.getEulerFromQuaternion(DSEstim.pose)
        self.goal[0] = DSPose[0]
        self.goal[1] = DSPose[1]
        self.goal[2] = DSPose[5]

        xAxisDS, yAxisDS = self.calculateOrientationAxes(self.goal[2], 1)
        self.target = np.array([self.goal[0] + xAxisDS[0], self.goal[1] + xAxisDS[1]])

        targetMap = PoseWithCovarianceStamped()
        targetMap.header.frame_id = 'map'
        targetMap.header.stamp = rospy.Time(0)
        targetMap.pose.pose.position.x = self.target[0]
        targetMap.pose.pose.position.y = self.target[1]
        targetMap.pose.pose.position.z = 0.
        targetMap.pose.pose.orientation.x = DSEstim.pose.pose.orientation.x
        targetMap.pose.pose.orientation.y = DSEstim.pose.pose.orientation.y
        targetMap.pose.pose.orientation.z = DSEstim.pose.pose.orientation.z
        targetMap.pose.pose.orientation.w = DSEstim.pose.pose.orientation.w
                     
        self.goalBase = self.tf2SamBase(DSEstim)
        self.targetBase = self.tf2SamBase(targetMap)
        
        self.receivedGoal = True

    def tf2SamBase(self,poseArg):
        poseBase = np.array([0., 0., 0.])

        # Transform into sam/baselink
        tmpPose = PoseStamped()
        tmpPose.header.frame_id = 'map'
        tmpPose.header.stamp = rospy.Time(0)
        tmpPose.pose.position.x = poseArg.pose.pose.position.x
        tmpPose.pose.position.y = poseArg.pose.pose.position.y
        tmpPose.pose.position.z = poseArg.pose.pose.position.z
        tmpPose.pose.orientation.x = poseArg.pose.pose.orientation.x
        tmpPose.pose.orientation.y = poseArg.pose.pose.orientation.y
        tmpPose.pose.orientation.z = poseArg.pose.pose.orientation.z
        tmpPose.pose.orientation.w = poseArg.pose.pose.orientation.w

        try:
            goalSamBaseLink = self.listener.transformPose(self.base_frame, tmpPose)

            poseBase[0] = goalSamBaseLink.pose.position.x
            poseBase[1] = goalSamBaseLink.pose.position.y

            rpy = euler_from_quaternion([goalSamBaseLink.pose.orientation.x,
                                         goalSamBaseLink.pose.orientation.y,
                                         goalSamBaseLink.pose.orientation.z,
                                         goalSamBaseLink.pose.orientation.w])
            poseBase[2] = rpy[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform to base frame not available yet")
            pass

        return poseBase


    def tf2Map(self, poseBase):
        poseMap = PoseStamped()

        # Transform into map
        tmpPose = PoseStamped()
        tmpPose.header.frame_id = poseBase.header.frame_id
        tmpPose.header.stamp = rospy.Time(0)
        tmpPose.pose.position.x = poseBase.pose.pose.position.x
        tmpPose.pose.position.y = poseBase.pose.pose.position.y
        tmpPose.pose.position.z = poseBase.pose.pose.position.z
        tmpPose.pose.orientation.x = poseBase.pose.pose.orientation.x
        tmpPose.pose.orientation.y = poseBase.pose.pose.orientation.y
        tmpPose.pose.orientation.z = poseBase.pose.pose.orientation.z
        tmpPose.pose.orientation.w = poseBase.pose.pose.orientation.w

        try:
            poseMap = self.listener.transformPose('map', tmpPose)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform to base frame not available yet")
            pass

        return poseMap


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
        for pt in range(len(self.controlPoints["x"])):
            controlpointPose = self.waypoint2pose([self.controlPoints["x"][pt], self.controlPoints["y"][pt]])
            controlpointMap = self.tf2Map(controlpointPose)
            self.controlPointsMap["x"][pt], self.controlPointsMap["y"][pt] = controlpointMap.pose.position.x, controlpointMap.pose.position.y


    def calculateBezierCurve(self):
        # Definition Bezier curve
        P = lambda t: (1 - t)**2 * np.array([self.controlPoints["x"][0],self.controlPoints["y"][0]]) \
                    + 2 * t * (1 - t) * np.array([self.controlPoints["x"][1],self.controlPoints["y"][1]]) \
                    + t**2 * np.array([self.controlPoints["x"][2],self.controlPoints["y"][2]])
        
        # Calculation Bezier curve in sam/base_link
        points = np.array([P(t) for t in np.linspace(0, 1, 3)])
        self.path["x"], self.path["y"] = points[:,0], points[:,1]        

        # Transform into map frame
        for pt in range(len(self.path["x"])):
            waypointPose = self.waypoint2pose([self.path["x"][pt], self.path["y"][pt]])
            waypointMap = self.tf2Map(waypointPose)
            self.pathMap["x"][pt], self.pathMap["y"][pt] = waypointMap.pose.position.x, waypointMap.pose.position.y


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


    def publishCurrentWaypoint(self):        
        try:
            currentWaypointBase = self.waypoint2pose(self.currentWaypoint)
            tmpWaypoint = self.tf2Map(currentWaypointBase)

            # We always publish PoseWithCovarianceStamped, but tf.listener only can transform PoseStamped. 
            publishWaypoint = PoseWithCovarianceStamped()
            publishWaypoint.header.frame_id = 'map'
            publishWaypoint.header.stamp = rospy.Time(0)
            publishWaypoint.pose.pose.position.x = tmpWaypoint.pose.position.x
            publishWaypoint.pose.pose.position.y = tmpWaypoint.pose.position.y
            publishWaypoint.pose.pose.position.z = tmpWaypoint.pose.position.z
            publishWaypoint.pose.pose.orientation.x = tmpWaypoint.pose.orientation.x
            publishWaypoint.pose.pose.orientation.y = tmpWaypoint.pose.orientation.y
            publishWaypoint.pose.pose.orientation.z = tmpWaypoint.pose.orientation.z
            publishWaypoint.pose.pose.orientation.w = tmpWaypoint.pose.orientation.w

            self.waypointPub.publish(publishWaypoint)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("map frame not available yet")
            pass

    
    def waypoint2pose(self, waypoint):
        poseWaypoint = PoseWithCovarianceStamped()
        poseWaypoint.header.frame_id = self.base_frame
        poseWaypoint.header.stamp = rospy.Time(0)
        poseWaypoint.pose.pose.position.x = waypoint[0]
        poseWaypoint.pose.pose.position.y = waypoint[1]
        poseWaypoint.pose.pose.position.z = 0
        poseWaypoint.pose.pose.orientation.w = 0
        poseWaypoint.pose.pose.orientation.x = 0
        poseWaypoint.pose.pose.orientation.y = 0
        poseWaypoint.pose.pose.orientation.z = 0

        return poseWaypoint

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


        # Coordinate visualization
        originDS = [self.goal[0], self.goal[1]]
        xAxisDS, yAxisDS = self.calculateOrientationAxes(self.goal[2], 0.5)
        
        originSAM = [self.start[0], self.start[1]]
        xAxisSAM, yAxisSAM = self.calculateOrientationAxes(self.start[2], 0.5)

        # DS Axes
        plt.arrow(*originDS, *xAxisDS, width=0.01, color='r')
        plt.arrow(*originDS, *yAxisDS, width=0.01, color='g')
        plt.arrow(*self.target, *xAxisDS, width=0.01, color='r')
        plt.arrow(*self.target, *yAxisDS, width=0.01, color='g')

        # SAM Coordinate axes
        
        plt.arrow(*originSAM, *xAxisSAM, width=0.01, color='r')
        plt.arrow(*originSAM, *yAxisSAM, width=0.01, color='g')

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

        plt.arrow(*originDS, *xAxisDS, width=0.01, color='r')
        plt.arrow(*originDS, *yAxisDS, width=0.01, color='g')
        plt.arrow(*originSAM, *xAxisSAM, width=0.01, color='r')
        plt.arrow(*originSAM, *yAxisSAM, width=0.01, color='g')

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
        
        plt.arrow(*originDS, *xAxisDS, width=0.01, color='r')
        plt.arrow(*originDS, *yAxisDS, width=0.01, color='g')
        plt.arrow(*originTargetBase, *xAxisDS, width=0.01, color='r')
        plt.arrow(*originTargetBase, *yAxisDS, width=0.01, color='g')

        plt.arrow(*originTargetBase, *xAxisSAM, width=0.01, color='r', alpha=0.5)
        plt.arrow(*originTargetBase, *yAxisSAM, width=0.01, color='g', alpha=0.5)
        plt.arrow(*originSAM, *xAxisSAM, width=0.01, color='r')
        plt.arrow(*originSAM, *yAxisSAM, width=0.01, color='g')

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
    #endregion

if __name__ == "__main__":
    rospy.init_node("SimplePathPlanner")
    pathPlanner = SimplePathPlanner(rospy.get_name())

    