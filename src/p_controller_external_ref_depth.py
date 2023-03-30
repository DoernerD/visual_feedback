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


class P_Controller(object):

    # Main loop
    def __init__(self, name):

        self.loop_freq = rospy.get_param("~loop_freq", 11)

        # Init
        self.current_x = np.array([0., 0., 0.1, 1., 0., 0.])
        self.current_y = np.array([0., 0., 0.1, 1., 0., 0.])
        self.stateEstim= np.array([0., 0., 0.1, 1., 0., 0.])
        self.current_pitch = 0.
        self.current_depth = 0.
        self.refX = 0.
        self.refY = 0.
        self.refZ = 0.
        self.refRoll = 0.
        self.refPitch = 0.
        self.refYaw = 0.
        self.ref = np.array([self.refX, self.refY, self.refZ, self.refRoll, self.refPitch, self.refYaw])

        # Desired depth and pitch for the experiments (limited to 2D plane)
        self.depth_desired = 0.    # in NED, bc. the dr/depth is in NED, ie. it's positive
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
        self.vbsNeutral = 50
        self.lcgNeutral = 50
        self.thrusterNeutral = 0
        self.vecHorizontalNeutral = 0
        self.vecVerticalNeutral = 0

        self.uLimited = np.array([self.thrusterNeutral, 
                                  self.vecHorizontalNeutral, 
                                  self.vecVerticalNeutral, 
                                  self.vbsNeutral, 
                                  self.lcgNeutral])
        self.antiWindupDifference = np.array([0., 0., 0., 0., 0.])
        self.antiWindupDifferenceInt = np.array([0., 0., 0., 0., 0.])

        # Topics for feedback and actuators
        # state_feedback_topic = rospy.get_param("~state_feedback_topic", "/sam/dr/odom")
        depth_topic = rospy.get_param("~depth_topic", "/sam/dr/depth")      # in NED
        pitch_topic = rospy.get_param("~pitch_topic", "/sam/dr/pitch")

        vbs_topic = rospy.get_param("~vbs_topic", "/sam/core/vbs_cmd")
        lcg_topic = rospy.get_param("~lcg_topic", "/sam/core/lcg_cmd")
        rpm1_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster1_cmd")
        rpm2_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster2_cmd")
        thrust_vector_cmd_topic = rospy.get_param("~thrust_vector_cmd_topic", "/sam/core/thrust_vector_cmd")


        control_error_topic = rospy.get_param("~control_error_topic", "/sam/ctrl/control_error")

        ref_pose_topic = rospy.get_param("~ref_pose_topic")
        state_estimate_topic = rospy.get_param("~state_estimation_topic")

        # Subscribers to state feedback, setpoints and enable flags
        # rospy.Subscriber(state_feedback_topic, Odometry, self.feedbackCallback)
        rospy.Subscriber(depth_topic, Float64, self.depthCallback)
        rospy.Subscriber(pitch_topic, Float64, self.pitchCallback)
        rospy.Subscriber(ref_pose_topic, PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber(state_estimate_topic, PoseWithCovarianceStamped, self.estimCallback)

        # Publisher to actuators
        self.rpm1Pub = rospy.Publisher(rpm1_topic, ThrusterRPM, queue_size=10)
        self.rpm2Pub = rospy.Publisher(rpm2_topic, ThrusterRPM, queue_size=10)
        self.vecPub = rospy.Publisher(thrust_vector_cmd_topic, ThrusterAngles, queue_size=10)
        self.vbsPub = rospy.Publisher(vbs_topic, PercentStamped, queue_size=10)
        self.lcgPub = rospy.Publisher(lcg_topic, PercentStamped, queue_size=10)
 
        self.control_error_pub = rospy.Publisher(control_error_topic, Float64, queue_size=10)
        

        # TF tree listener        
        self.listener = tf.TransformListener()
        self.base_frame = 'sam/base_link/estimated'

        rate = rospy.Rate(self.loop_freq) 

        # Run
        while not rospy.is_shutdown():

            u = self.computeControlAction()

            self.limitControlAction(u)

            self.computeAntiWindup(u)

            self.publishControlAction(self.uLimited)

            rate.sleep()


    #region Call-backs
    def feedbackCallback(self, odom_fb):
        # [self.current_x,self.velocities] = self.getStateFeedback(odom_fb)
        self.current_x = self.getEulerFromQuaternion(odom_fb.pose)

    def depthCallback(self, depth):
        # [self.current_x,self.velocities] = self.getStateFeedback(odom_fb)
        # print(depth.data)
        self.current_depth = depth.data

    def pitchCallback(self, pitch):
        # [self.current_x,self.velocities] = self.getStateFeedback(odom_fb)
        self.current_pitch = pitch.data
        # print(pitch)

    def estimCallback(self, estim):
        # [self.current_x,self.velocities] = self.getStateFeedback(odom_fb)
        self.stateEstim = self.getEulerFromQuaternion(estim.pose)
        self.stateEstim[2] = self.current_depth
        self.stateEstim[4] = self.current_pitch

    def poseCallback(self,estimFB):
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
            rospy.logwarn("Transform to base frame not available yet")
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

    #region ENU-NED transform
    # def refPoseCb(self, pose):
    #     # Pose is in ENU. The callback extracts the position and Euler angles and subsequently transforms it in NED for internal use.
    #     #
    #     # Pose in ENU 
    #     xENU = pose.pose.position.x
    #     yENU = pose.pose.position.y
    #     zENU = pose.pose.position.z

    #     eta0ENU = pose.pose.orientation.w
    #     eta1ENU = pose.pose.orientation.x
    #     eta2ENU = pose.pose.orientation.y
    #     eta3ENU = pose.pose.orientation.z
        
    #     rpyENU = euler_from_quaternion([eta0ENU,eta1ENU,eta2ENU,eta3ENU],'sxyz')
    #     rollENU = rpyENU[0]
    #     pitchENU = rpyENU[1]
    #     yawENU = rpyENU[2]
        
    #     # Transform ENU -> NED
    #     # Swapping axes:
    #     #   xENU -> yNED
    #     #   yENU -> xNED
    #     #   zENU -> -zNED
    #     self.refX = yENU
    #     self.refY = xENU
    #     self.refZ = -zENU

    #     # Changing angles:
    #     #   roll and pitch remain
    #     #   yawNED = -yawENU + 90deg (because axis flipped (-) and we rotate by 90deg to keep the other angles the same)
    #     rollNED = rollENU
    #     pitchNED = pitchENU
    #     yawNED = np.pi/2-yawENU

    #     self.refRoll = rollNED
    #     self.refPitch = pitchNED
    #     self.refYaw = yawNED

    #     self.ref = np.array([self.refX, self.refY, self.refZ, self.refRoll, self.refPitch, self.refYaw])

    # def getStateFeedback(self, odom_msg):
    #     # Note: The callbacks are necessary, but we can define them ourselves.
    #     # # Converting from ENU to NED, e.g. see https://www.programmersought.com/article/83764943652/ or # https://robotics.stackexchange.com/questions/19669/rotating-ned-to-enu
    #     # that is why we switch y and x, rotate the yaw by 90 degrees and have the opposite sign on z.
    #     x =  odom_msg.pose.pose.position.y
    #     y =  odom_msg.pose.pose.position.x
    #     z = -odom_msg.pose.pose.position.z
    #     eta0 = odom_msg.pose.pose.orientation.w
    #     eta1 = odom_msg.pose.pose.orientation.x
    #     eta2 = odom_msg.pose.pose.orientation.y
    #     eta3 = odom_msg.pose.pose.orientation.z

    #     rpy   = euler_from_quaternion([eta0,eta1,eta2,eta3],'sxyz')
    #     roll  = rpy[0]
    #     pitch = rpy[1]
    #     yaw   = np.pi/2-rpy[2]

    #     # print("RPY (ENU): ", rpy)

    #     # Velocities
    #     u =  odom_msg.twist.twist.linear.y
    #     v =  odom_msg.twist.twist.linear.x
    #     w = -odom_msg.twist.twist.linear.z        
    #     p =  odom_msg.twist.twist.angular.x
    #     q =  odom_msg.twist.twist.angular.y
    #     r = -odom_msg.twist.twist.angular.z

    #     state  = np.array([x,y,z,roll,pitch,yaw]) 
    #     velocity = np.array([u,v,w,p,q,r])

    #     return [state, velocity]
    #endregion

    #region Publisher
    def publishControlAction(self, u):

        thruster1 = ThrusterRPM()
        thruster2 = ThrusterRPM()
        vec = ThrusterAngles()
        vbs = PercentStamped()
        lcg = PercentStamped()

        thruster1.rpm = u[0]
        thruster2.rpm = u[0]
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
        
        # Publish control inputs and error for visualization
        self.control_error_pub.publish(self.err[5])   
    #endregion

    # Controller
    def computeControlAction(self):
        # Sliding Mode Control for Depth First control
        epsDepth = 0.4 # offset for depth control
        epsPitch = 0.05 # offset for pitch control

        # enforcing depth and pitch rather than using the docking station estimate.
        self.ref[2] = self.depth_desired
        self.ref[4] = self.pitch_desired

        while ((np.abs(self.current_depth - self.ref[2]) <= epsDepth) and (np.abs(self.current_pitch - self.ref[4]) <= epsPitch)):
            u = self.computeConstVelDepthControlAction()
            return u
 
        u = self.computeDepthControlAction()

        return u

    def computeDepthControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([0., 0., 0., 0., 50.])

        Kp = np.array([40, 5, 5, 100, 500])      # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 2, 10])       # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])         # D control gain
        Kaw = np.array([1., 1., 1., 4., 1.])        # Anti-Windup Gain

        self.ref[2] = self.depth_desired
        self.ref[4] = self.pitch_desired

        self.errPrev = self.err
        self.err = self.ref - self.stateEstim

        # We need to integrate the anti windup before adding it to the error
        # because multiple actuators affect the same error. That way we can
        # use the correct actuator anti windup for each actuator
        self.calcAntiWindupIntegral()

        # enforcing depth and pitch
        self.err[2] = self.depth_desired - self.current_depth
        self.err[4] = self.pitch_desired - self.current_pitch

        self.integral += (self.err) * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        # u[3] = (Kp[3]*self.err[2] + Ki[3]*self.integral[2])   # PI control vbs
        u[3] = (Kp[3]*self.err[2] + Ki[3]*self.integral[2] + Kaw[3]*self.antiWindupDifferenceInt[3])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] - Ki[4]*self.integral[4] - Kaw[4]*self.antiWindupDifferenceInt[4])   # PI control lcg

        return u

    def computeConstVelDepthControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([0., 0., 0., 0., 50.])

        Kp = np.array([40, 5, 5, 100, 500])        # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 2, 10])       # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])         # D control gain
        Kaw = np.array([1., 1., 1., 4., 1.])        # Anti-Windup Gain
        
        # We need to integrate the anti windup before adding it to the error
        # because multiple actuators affect the same error. That way we can
        # use the correct actuator anti windup for each actuator
        self.calcAntiWindupIntegral()

        self.errPrev = self.err
        self.err = self.ref - self.stateEstim

        # enforcing depth and pitch
        self.err[2] = self.depth_desired - self.current_depth
        self.err[4] = self.pitch_desired - self.current_pitch

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
        if self.distanceErr > 1:
            u[0] = 200
            u[1] = -(Kp[1]*self.headingAngle + Ki[1]*self.headingAngleInt - Kaw[1]*self.antiWindupDifferenceInt[1])   # PI control vectoring (horizontal)

        elif self.distanceErr < -1:
            u[0] = -200
            self.headingAngleScaled = np.sign(self.headingAngle) * (np.pi - np.abs(self.headingAngle))
            u[1] = -(Kp[1]*self.headingAngleScaled - (Ki[1]*self.headingAngleInt + Kaw[1]*self.antiWindupDifferenceInt[1]))   # PI control vectoring (horizontal)

        else:
            u[0] = 0
        u[2] = (Kp[2]*self.err[2] + Ki[2]*self.integral[2] + Kaw[2]*self.antiWindupDifferenceInt[2])   # PI control vectoring (vertical)
        u[3] = (Kp[3]*self.err[2] + Ki[3]*self.integral[2] + Kaw[3]*self.antiWindupDifferenceInt[3])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] - Ki[4]*self.integral[4] - Kaw[4]*self.antiWindupDifferenceInt[4])   # PI control lcg

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

        print("All in ENU:")
        print("[x, y, z, roll, pitch, yaw]")
        self.printNumpyArray(self.stateEstim,"Current States (map): %.4f %.4f %.4f %.4f %.4f %.4f\n")
        self.printNumpyArray(self.ref,"Reference States (SAM): %.4f %.4f %.4f %.4f %.4f %.4f\n")
        # self.printNumpyArray(self.err,"Control Error: %.4f %.4f %.4f %.4f %.4f %.4f\n")
        sys.stdout.write("Distance Error: %.4f, Heading Angle: %.4f, Depth Error: %.4f\n" 
                         % (self.distanceErr, self.headingAngle, self.err[2]))    
        print("[thruster, vec (horizontal), vec (vertical), vbs, lcg]")
        sys.stdout.write("Control Input raw: %.4f %.4f %.4f %.4f %.4f\n"  
                         % (u[0], u[1], u[2], u[3], u[4]))
        sys.stdout.write("Control Input: %.4f %.4f %.4f %.4f %.4f\n"  
                         % (self.uLimited[0], self.uLimited[1], self.uLimited[2], self.uLimited[3], self.uLimited[4]))
        sys.stdout.write("Anti Windup Int: %.4f %.4f %.4f %.4f %.4f\n"  
                         % (self.antiWindupDifferenceInt[0], self.antiWindupDifferenceInt[1], 
                            self.antiWindupDifferenceInt[2], self.antiWindupDifferenceInt[3], 
                            self.antiWindupDifferenceInt[4]))
        print("")

        # return uLimited

    def printNumpyArray(self, array, string):

        sys.stdout.write(string % (array[0], array[1], array[2], array[3], array[4], array[5]))

    def computeAntiWindup(self,u):
        self.antiWindupDifference = self.uLimited - u
        

if __name__ == "__main__":
    rospy.init_node("p_controller")
    controller = P_Controller(rospy.get_name())
