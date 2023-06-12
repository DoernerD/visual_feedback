#!/usr/bin/python
# Copyright 2022 David Doerner (ddorner@kth.se)

from __future__ import division, print_function
import sys

import numpy as np
import math

import rospy
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Float64
from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import ThrusterAngles, PercentStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PointStamped


from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import tf


class WaypointFollowingController(object):

    # Main loop
    def __init__(self, name):

        # Launch File Parameters
        self.loop_freq = rospy.get_param("~loop_freq", 20)
        verbose = rospy.get_param("~verbose", True)

        # Init
        self.current_x = np.array([0., 0., 0.1, 1., 0., 0.])
        self.current_y = np.array([0., 0., 0.1, 1., 0., 0.])
        self.stateEstim= np.array([0., 0., 0.1, 1., 0., 0.])
        self.refX = 0.
        self.refY = 0.
        self.refZ = 0.
        self.refRoll = 0.
        self.refPitch = 0.
        self.refYaw = 0.
        self.ref = np.array([self.refX, self.refY, self.refZ, self.refRoll, self.refPitch, self.refYaw])

        # Desired depth and pitch for the experiments (limited to 2D plane)
        # In simulation depth is negative (ENU), in reality, it's positive (NED)
        self.depth_desired = 1. 
        self.pitch_desired = 0.

        self.err = np.array([0., 0., 0., 0., 0., 0.])
        self.errPrev = np.array([0., 0., 0., 0., 0., 0.])
        self.integral = np.array([0., 0., 0., 0., 0., 0.])

        self.distanceErr = 0.
        self.distanceErrPrev = 0.
        self.distanceErrInt = 0.

        self.headingAngle = 0.
        self.headingAngleInt = 0.

        # Neutral actuator inputs
        self.vbsNeutral = 50.
        self.lcgNeutral = 70.
        self.thrusterNeutral = 0
        self.vecHorizontalNeutral = 0.
        self.vecVerticalNeutral = 0.
        self.current_depth = 0.

        self.uNeutral = np.array([self.thrusterNeutral, 
                                  self.vecHorizontalNeutral, 
                                  self.vecVerticalNeutral, 
                                  self.vbsNeutral, 
                                  self.lcgNeutral])

        self.uLimited = self.uNeutral.copy()

        self.antiWindupDifference = np.array([0., 0., 0., 0., 0.])
        self.antiWindupDifferenceInt = np.array([0., 0., 0., 0., 0.])

        self.limitOutputCounter = 0

        # Topics for feedback and actuators
        # state_feedback_topic = rospy.get_param("~state_feedback_topic", "/sam/dr/odom")
        vbs_topic = rospy.get_param("~vbs_topic", "/sam/core/vbs_cmd")
        lcg_topic = rospy.get_param("~lcg_topic", "/sam/core/lcg_cmd")
        rpm1_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster1_cmd")
        rpm2_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster2_cmd")
        thrust_vector_cmd_topic = rospy.get_param("~thrust_vector_cmd_topic", "/sam/core/thrust_vector_cmd")

        ref_pose_topic = rospy.get_param("~ref_pose_topic")
        state_estimate_topic = rospy.get_param("~state_estimation_topic")

        # Subscribers to state feedback, setpoints and enable flags
        # rospy.Subscriber(state_feedback_topic, Odometry, self.feedbackCallback)
        rospy.Subscriber(ref_pose_topic, PoseWithCovarianceStamped, self.waypointCallback)
        rospy.Subscriber(state_estimate_topic, PoseWithCovarianceStamped, self.estimCallback)

        # Publisher to actuators
        self.rpm1Pub = rospy.Publisher(rpm1_topic, ThrusterRPM, queue_size=1)
        self.rpm2Pub = rospy.Publisher(rpm2_topic, ThrusterRPM, queue_size=1)
        self.vecPub = rospy.Publisher(thrust_vector_cmd_topic, ThrusterAngles, queue_size=10)
        self.vbsPub = rospy.Publisher(vbs_topic, PercentStamped, queue_size=10)
        self.lcgPub = rospy.Publisher(lcg_topic, PercentStamped, queue_size=10)
         
        # TF tree listener        
        self.listener = tf.TransformListener()
        self.base_frame = 'sam/base_link/estimated'

        rate = rospy.Rate(self.loop_freq) 

        # Run
        while not rospy.is_shutdown():

            u = self.computeControlAction()

            self.limitControlAction(u)

            self.computeAntiWindup(u)

            # self.publishControlAction(self.uLimited)
            self.publishControlAction(self.uNeutral)

            if verbose:
                self.printStates(u)

            rate.sleep()


    #region Call-backs
    def feedbackCallback(self, odom_fb):
        self.current_x = self.getEulerFromQuaternion(odom_fb.pose)

    def estimCallback(self, estim):
        # [self.current_x,self.velocities] = self.getStateFeedback(odom_fb)
        self.stateEstim = self.getEulerFromQuaternion(estim.pose)
        
    def waypointCallback(self,estimFB):
        # Get way point in map frame
        self.ref = np.zeros([6])

        # Transform waypoint map --> base frame
        goal_point = PointStamped()
        goal_point.header.frame_id = 'map'
        goal_point.header.stamp = rospy.Time(0)
        goal_point.point.x = estimFB.pose.pose.position.x
        goal_point.point.y = estimFB.pose.pose.position.y
        goal_point.point.z = estimFB.pose.pose.position.z

        try:
            goal_point_local = self.listener.transformPoint(
                self.base_frame, goal_point)
                        
            self.ref[0] = goal_point_local.point.x
            self.ref[1] = goal_point_local.point.y
            self.ref[2] = goal_point_local.point.z
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[WPF]: Transform to base frame not available yet")
            pass

               
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

    

    #region Publisher
    def publishControlAction(self, u):

        thruster1 = ThrusterRPM()
        thruster2 = ThrusterRPM()
        vec = ThrusterAngles()
        vbs = PercentStamped()
        lcg = PercentStamped()

        thruster1.rpm = int(u[0])
        thruster2.rpm = int(u[0])
        vec.thruster_horizontal_radians = u[1] 
        vec.thruster_vertical_radians = u[2]
        vbs.value = int(u[3])
        lcg.value = int(u[4])

        # Publish to actuators
        self.rpm1Pub.publish(thruster1)
        self.rpm2Pub.publish(thruster2)
        self.vecPub.publish(vec)
        self.vbsPub.publish(vbs)
        self.lcgPub.publish(lcg)

    #endregion

    # Controller
    def computeControlAction(self):
        # Sliding Mode Control for Depth First control
        epsDepth = 0.4 # offset for depth control
        epsPitch = 0.2 #np.deg2rad(5) # offset for pitch control

        # enforcing depth and pitch rather than using the docking station estimate.
        self.ref[2] = self.depth_desired
        self.ref[4] = self.pitch_desired

        while ((np.abs(self.stateEstim[2] - self.ref[2]) <= epsDepth) and (np.abs(self.stateEstim[4] - self.ref[4]) <= epsPitch)):
            u = self.computeConstVelDepthControlAction()
            return u
 
        u = self.computeDepthControlAction()

        return u

    def computeDepthControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([self.thrusterNeutral, 
                        self.vecHorizontalNeutral, 
                        self.vecVerticalNeutral, 
                        self.vbsNeutral, 
                        self.lcgNeutral])

        ## SIM PARAMETERS
        Kp = np.array([40, 5, 5, 40, 60])      # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 0.75, 1.25])       # I control gain
        Kd = np.array([1., 1., 1., 1., 6.])         # D control gain
        Kaw = np.array([1., 1., 1., 0., 1.])        # Anti-Windup Gain

        ## TANK PARAMETERS
        # Kp = np.array([40, 5, 5, 40, 60])      # P control gain
        # Ki = np.array([0., 0., 0., 0., 0.])       # I control gain
        # Kd = np.array([1., 1., 1., 1., 6.])         # D control gain
        # Kaw = np.zeros(5)

        self.errPrev = self.err
        self.err = self.ref - self.stateEstim

        # We need to integrate the anti windup before adding it to the error
        # because multiple actuators affect the same error. That way we can
        # use the correct actuator anti windup for each actuator
        self.calcAntiWindupIntegral()

        self.integral += (self.err) * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        ## SIM CONTROLLER
        # u[3] = -(Kp[3]*self.err[2] + Ki[3]*self.integral[2])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] + self.lcgNeutral - Ki[4]*self.integral[4] - Kaw[4]*self.antiWindupDifferenceInt[4])   # PI control lcg

        ## TANK CONTROLLER
        u[3] = (Kp[3]*self.err[2] + self.vbsNeutral + Ki[3]*self.integral[2] + Kaw[3]*self.antiWindupDifferenceInt[3])   # PI control vbs
        # u[4] = (Kp[4]*self.err[4] + self.lcgNeutral + Kd[4]*self.deriv[4])   # PD control lcg

        return u

    def computeConstVelDepthControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([self.thrusterNeutral, 
                        self.vecHorizontalNeutral, 
                        self.vecVerticalNeutral, 
                        self.vbsNeutral, 
                        self.lcgNeutral])

        # SIM PARAMETERS
        Kp = np.array([40, 5, 5, 100, 50])        # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 2, 1])       # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])         # D control gain
        Kaw = np.array([1., 1., 1., 4., 1.])        # Anti-Windup Gain
        
        # TANK PARAMETERS
        # Kp = np.array([40, 5, 5, 40, 60])      # P control gain
        # # Ki = np.array([0.5, 0.1, 0.1, 0.75, 1.25])       # I control gain
        # Ki = np.array([0., 0., 0., 0., 0.])       # I control gain
        # Kd = np.array([1., 1., 1., 1., 6.])         # D control gain
        # #Kaw = np.array([1., 1., 1., 0., 1.])        # Anti-Windup Gain
        # Kaw = np.zeros(5)

        
        # We need to integrate the anti windup before adding it to the error
        # because multiple actuators affect the same error. That way we can
        # use the correct actuator anti windup for each actuator
        self.calcAntiWindupIntegral()

        self.errPrev = self.err
        self.err = self.ref - self.stateEstim

        # enforcing depth and pitch
        # The current depth and pitch are already in stateEstim due to the callback. So we don't need that here.
        # self.err[2] = self.depth_desired - self.current_depth
        # self.err[4] = self.pitch_desired - self.current_pitch

        # self.integral += (self.err - antiWindupError) * (1/self.loop_freq)
        self.integral += (self.err) * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        self.headingAngle = math.atan2(self.ref[1], self.ref[0])
        self.headingAngleInt += self.headingAngle * (1/self.loop_freq)

        # Calculate distance to reference pose
        # Workaround bc np.sign(0) = 0
        if np.sign(self.ref[0]) == 0:
            distanceSign = 1
        else:
            distanceSign = np.sign(self.ref[0])

        self.distanceErrPrev = self.distanceErr
        self.distanceErr = np.sqrt(self.ref[0]**2 + self.ref[1]**2) * distanceSign
        self.distanceErrInt += self.distanceErr * (1/self.loop_freq)
        self.distanceErrDeriv = (self.distanceErr - self.distanceErrPrev) * self.loop_freq 
  

        # Compute control action u
        # Going forwards and backwards based on the distance to the target
        stopRadius = 0.2
        if self.distanceErr > stopRadius:
            u[0] = 200
            
            # SIM CONTROLLER
            u[1] = -(Kp[1]*self.headingAngle + Ki[1]*self.headingAngleInt - Kaw[1]*self.antiWindupDifferenceInt[1])   # PI control vectoring (horizontal)
            
            # TANK CONTROLLER
            # u[1] = -(Kp[1]*self.headingAngle + Ki[1]*self.headingAngleInt - Kaw[1]*self.antiWindupDifferenceInt[1])   # PI control vectoring (horizontal)

        elif self.distanceErr < -stopRadius:
            u[0] = -200
            self.headingAngleScaled = np.sign(self.headingAngle) * (np.pi - np.abs(self.headingAngle))
            
            # SIM CONTROLLER
            u[1] = -(Kp[1]*self.headingAngleScaled - (Ki[1]*self.headingAngleInt + Kaw[1]*self.antiWindupDifferenceInt[1]))   # PI control vectoring (horizontal)
            
            # TANK CONTROLLER
            # u[1] = -(Kp[1]*self.headingAngleScaled - (Ki[1]*self.headingAngleInt + Kaw[1]*self.antiWindupDifferenceInt[1]))   # PI control vectoring (horizontal)

        else:
            u[0] = 0
        
        # SIM CONTROLLER
        u[2] = (Kp[2]*self.err[2] + Ki[2]*self.integral[2] + Kaw[2]*self.antiWindupDifferenceInt[2])   # PI control vectoring (vertical)
        # u[3] = -(Kp[3]*self.err[2] + Ki[3]*self.integral[2])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] - Ki[4]*self.integral[4] - Kaw[4]*self.antiWindupDifferenceInt[4])   # PI control lcg

        # TANK CONTROLLER
        u[3] = (Kp[3]*self.err[2] + self.vbsNeutral + Ki[3]*self.integral[2] + Kaw[3]*self.antiWindupDifferenceInt[3])   # PI control vbs
        # u[4] = (Kp[4]*self.err[4] + self.lcgNeutral + Kd[4]*self.deriv[4])   # PD control lcg

        return u
    
    def calcAntiWindupIntegral(self):
        self.antiWindupDifferenceInt += (self.antiWindupDifference) * (1/self.loop_freq)

    def limitControlAction(self,u):
        # Enforce hardware limits on actuator control
        self.uLimited = u.copy()     # without .copy(), python only makes a shallow copy of the array.

        # thrust vector limit horizontal
        if self.uLimited[1] > np.deg2rad(7):
            self.uLimited[1] = np.deg2rad(7)
        elif self.uLimited[1] < -np.deg2rad(7):
            self.uLimited[1] = -np.deg2rad(7)

        # thrust vector limit vertical
        if self.uLimited[2] > np.deg2rad(7):
            self.uLimited[2] = np.deg2rad(7)
        elif self.uLimited[2] < -np.deg2rad(7):
            self.uLimited[2] = -np.deg2rad(7)

        # vbs limit
        if self.uLimited[3] > 100:
            self.uLimited[3] = 100
        if self.uLimited[3] < 0:
            self.uLimited[3] = 0

        # lcg limit
        if self.uLimited[4] > 100:
            self.uLimited[4] = 100
        if self.uLimited[4] < 0:
            self.uLimited[4] = 0
   
    def computeAntiWindup(self,u):
        self.antiWindupDifference = self.uLimited - u

    def printStates(self, u):
        # use print("string {}".format(data)) instead. Python will format data accordingly
        # and print it in place of {}. To print multiple variables, use multiple {}.

        np.set_printoptions(suppress=True)
        self.limitOutputCounter += 1
        if self.limitOutputCounter % 20 == 1:
            print("All in ENU:")
            print("[x, y, z, roll, pitch, yaw]")
            print("Current States: {}".format(np.array2string(self.stateEstim, suppress_small = True, precision = 4)))
            print("Reference States: {}".format(np.array2string(self.ref, precision = 4, suppress_small = True, floatmode = 'fixed')))
            print("Distance Error: {:.4f}, Heading Angle: {:.4f}, Depth Error: {:.4f}".format(self.distanceErr, self.headingAngle, self.err[2]))
            print("[thruster, vec (horizontal), vec (vertical), vbs, lcg]")
            print("Control Input raw: {}".format(np.array2string(u, precision = 4, floatmode = 'fixed')))
            print("Control Input: {}".format(np.array2string(self.uLimited, precision = 4, floatmode = 'fixed')))
            print("Anti Windup Int: {}".format(np.array2string(self.antiWindupDifferenceInt, precision = 4, floatmode = 'fixed')))
            print("")


if __name__ == "__main__":
    rospy.init_node("WaypointFollowingController")
    controller = WaypointFollowingController(rospy.get_name())
