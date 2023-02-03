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
import tf2_ros


class P_Controller(object):

    # Main loop
    def __init__(self, name):

        self.loop_freq = rospy.get_param("~loop_freq", 11)

        # Topics for feedback and actuators
        state_feedback_topic = rospy.get_param("~state_feedback_topic", "/sam/dr/odom")
        vbs_topic = rospy.get_param("~vbs_topic", "/sam/core/vbs_cmd")
        lcg_topic = rospy.get_param("~lcg_topic", "/sam/core/lcg_cmd")
        rpm1_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster1_cmd")
        rpm2_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster2_cmd")
        thrust_vector_cmd_topic = rospy.get_param("~thrust_vector_cmd_topic", "/sam/core/thrust_vector_cmd")
        samPoseTopic = rospy.get_param("~samPoseTopic", "/sam/pose")


        control_error_topic = rospy.get_param("~control_error_topic", "/sam/ctrl/control_error")
        control_input_topic = rospy.get_param("~control_input_topic", "/sam/ctrl/control_input")

        # ref_pose_topic = rospy.get_param("~ref_pose_topic", "/dockingStation/pose")
        ref_pose_topic = rospy.get_param("~ref_pose_topic")

        # Subscribers to state feedback, setpoints and enable flags
        rospy.Subscriber(state_feedback_topic, Odometry, self.feedbackCallback)
        # rospy.Subscriber(ref_pose_topic, PoseStamped, self.poseCallback)
        # rospy.Subscriber(ref_pose_topic, PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber(ref_pose_topic, Pose, self.poseCallback)

        # Publisher to actuators
        self.rpm1Pub = rospy.Publisher(rpm1_topic, ThrusterRPM, queue_size=10)
        self.rpm2Pub = rospy.Publisher(rpm2_topic, ThrusterRPM, queue_size=10)
        self.vecPub = rospy.Publisher(thrust_vector_cmd_topic, ThrusterAngles, queue_size=10)
        self.vbsPub = rospy.Publisher(vbs_topic, PercentStamped, queue_size=10)
        self.lcgPub = rospy.Publisher(lcg_topic, PercentStamped, queue_size=10)
 
        self.posePub = rospy.Publisher(samPoseTopic, PoseStamped, queue_size=10)

        self.control_error_pub = rospy.Publisher(control_error_topic, Float64, queue_size=10)
        # self.u_pub = rospy.Publisher(control_input_topic, Float64, queue_size=10)

        rate = rospy.Rate(self.loop_freq) 

        # Init
        self.current_x = np.array([0., 0., 0.1, 1., 0., 0.])
        self.current_y = np.array([0., 0., 0.1, 1., 0., 0.])
        self.refX = 0.
        self.refY = 0.
        self.refZ = 0.
        self.refRoll = 0.
        self.refPitch = 0.
        self.refYaw = 0.
        self.ref = np.array([self.refX, self.refY, self.refZ, self.refRoll, self.refPitch, self.refYaw])

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

        # Run
        while not rospy.is_shutdown():

            u = self.computeControlAction()

            uLimited = self.limitControlAction(u)

            self.publishPose()
            self.publishControlAction(uLimited)

            rate.sleep()


    #region Call-backs
    def feedbackCallback(self, odom_fb):
        # [self.current_x,self.velocities] = self.getStateFeedback(odom_fb)
        self.current_x = self.getEulerFromQuaternion(odom_fb.pose,'xyzs')

    def poseCallback(self,estimFB):
        # poseSXYZ = self.getEulerFromQuaternion(estimFB.pose,'skyz')
        # poseXYZS = self.getEulerFromQuaternion(estimFB.pose,'xyzs')

        # sys.stdout.write('\n\r')
        # # sys.stdout.write('Number %d %d' % (poseSXYZ[0], poseSXYZ[1]))
        # sys.stdout.write("Current States: %.4f %.4f %.4f %.4f %.4f %.4f\n" % (self.current_x[0], self.current_x[1], self.current_x[2], self.current_x[3], self.current_x[4], self.current_x[5]))
        # sys.stdout.write("poseSXYZ: %.4f %.4f %.4f %.4f %.4f %.4f\n" % (poseSXYZ[0], poseSXYZ[1], poseSXYZ[2], poseSXYZ[3], poseSXYZ[4], poseSXYZ[5]))
        # sys.stdout.write("poseXYZS: %.4f %.4f %.4f %.4f %.4f %.4f\n" % (poseXYZS[0], poseXYZS[1], poseXYZS[2], poseXYZS[3], poseXYZS[4], poseXYZS[5]))
        # # sys.stdout.flush()
        # # sys.stdout.write("poseCB")

        self.ref = self.getEulerFromQuaternion(estimFB,'xyzs')
        # self.ref = np.zeros([6])

        # # Transform goal map --> base frame
        # goal_point = PointStamped()
        # goal_point.header.frame_id = self.map
        # goal_point.header.stamp = rospy.Time(0)
        # goal_point.point.x = estimFB.pose.position.x
        # goal_point.point.y = estimFB.pose.position.y
        # goal_point.point.z = estimFB.pose.position.z
        # try:
        #     goal_point_local = self.listener.transformPoint(
        #         self.base_frame, goal_point)
            
        #     self.ref[0] = goal_point_local.point.x
        #     self.ref[1] = goal_point_local.point.y
        #     self.ref[2] = goal_point_local.point.z

            
        #     # #Compute throttle error
        #     # # throttle_level = min(self.max_throttle, np.linalg.norm(
        #     # #     np.array([goal_point_local.point.x + goal_point_local.point.y])))
        #     # # Nacho: no real need to adjust the throttle 
        #     # throttle_level = self.max_throttle
        #     # # Compute thrust error
        #     # alpha = math.atan2(goal_point_local.point.y,
        #     #                 goal_point_local.point.x)
        #     # sign = np.copysign(1, alpha)
        #     # yaw_setpoint = sign * min(self.max_thrust, abs(alpha))

        #     # # Command velocities
        #     # self.motion_command(throttle_level, yaw_setpoint, 0.)

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     rospy.logwarn("Transform to base frame not available yet")
        # pass
    
    def getEulerFromQuaternion(self, pose, order):
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
        
        # for some reason the odom pose uses a different order for the euler-quaternion transformation. The else statement should be the default.
        if order is 'xyzs':
            rpyENU = euler_from_quaternion([eta1ENU,eta2ENU,eta3ENU,eta0ENU])
        else:
            rpyENU = euler_from_quaternion([eta0ENU,eta1ENU,eta2ENU,eta3ENU],'sxyz')


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
    def publishPose(self):
        # Pose in NED
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "sam_pose"

        pose.pose.position.x = self.current_x[0]
        pose.pose.position.y = self.current_x[1]
        pose.pose.position.z = self.current_x[2]

        quaternion = quaternion_from_euler(self.current_x[3],self.current_x[4],self.current_x[5])
        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]
        pose.pose.orientation.w = quaternion[0]

        self.posePub.publish(pose)

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
        # self.u_pub.publish(u)   
        self.control_error_pub.publish(self.err[5])   
    #endregion

    # Controller
    def computeControlAction(self):
        # Sliding Mode Control for Depth First control
        epsDepth = 0.2 # offset for depth control
        epsPitch = 0.05 # offset for pitch control

        # enforcing depth and pitch rather than using the docking station estimate.
        # self.ref[2] = -1.5
        # self.ref[4] = 0.

        ## THIS DOESN'T WORK! 
        # There needs to be a different way to implement breaking when approaching the docking
        # station. The idea is to reverse the RPM to get some breaking force.
        # if self.velocities[0] != 0:
        #     if np.abs(self.distanceErr) < 0.5:
        #         u = np.array([0., 0., 0., 0., 50.])

        #         u[0] = -np.sign(self.distanceErr)*500
        #         return u

        # Not sure if the condition works correctly...
        while ((np.abs(self.current_x[2] - self.ref[2]) >= epsDepth) and (np.abs(self.current_x[4] - self.ref[4]) >= epsPitch)):
            u = self.computeDepthControlAction()
            return u
 
        # u = self.computePIDControlAction()
        u = self.computeConstVelDepthControlAction()

        return u

    def computeDepthControlAction(self):
        # print("Depth Control")

        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([0., 0., 0., 0., 50.])

        Kp = np.array([40, 10, 10, 150, 1000])        # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 3, 10])       # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])

        # self.ref[2] = -1.5
        # self.ref[4] = 0.

        self.errPrev = self.err
        self.err = self.ref - self.current_x
        # self.err = -self.ref
        self.integral += self.err * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        u[3] = -(Kp[3]*self.err[2] + Ki[3]*self.integral[2])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] + Ki[4]*self.integral[4])   # PI control lcg

        return u

    def computeConstVelDepthControlAction(self):
        # print("Vel Control")

        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([0., 0., 0., 0., 50.])

        Kp = np.array([40, 10, 5, 150, 500])        # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 3, 10])       # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])

        #Compute throttle error
        # # throttle_level = min(self.max_throttle, np.linalg.norm(
        # #     np.array([goal_point_local.point.x + goal_point_local.point.y])))
        # # Nacho: no real need to adjust the throttle 
        # throttle_level = self.max_throttle
        # # Compute thrust error
        # alpha = math.atan2(goal_point_local.point.y,
        #                 goal_point_local.point.x)
        # sign = np.copysign(1, alpha)
        # yaw_setpoint = sign * min(self.max_thrust, abs(alpha))

        # # Command velocities
        # self.motion_command(throttle_level, yaw_setpoint, 0.)


        self.errPrev = self.err
        self.err = self.ref - self.current_x
        # self.err = self.ref

        # enforcing depth and pitch
        # self.err[2] = -1.5 - self.current_x[2]
        # self.err[4] = 0. - self.current_x[4]

        self.integral += self.err * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        self.headingAngle = math.atan2(self.err[1], self.err[0])
        self.headingAngleInt += 0.#self.headingAngleInt * (1/self.loop_freq)

        # Calculate distance to reference pose
        self.distanceErrPrev = self.distanceErr
        self.distanceErr = np.sqrt(self.err[0]**2 + self.err[1]**2)# * np.sign(self.headingAngle)
        self.distanceErrInt += self.distanceErr * (1/self.loop_freq)
        self.distanceErrDeriv = (self.distanceErr - self.distanceErrPrev) * self.loop_freq   

        

        # Going forwards and backwards based on th distance to the target
        if self.distanceErr > 1:
            u[0] = 500
        elif self.distanceErr < -1:
            u[0] = -500
        else:
            u[0] = 0
        u[1] = -(Kp[1]*self.err[5] + Ki[1]*self.integral[5])   # PI control vectoring (horizontal)
        # u[2] = -(Kp[2]*self.err[2] + Ki[2]*self.integral[2])   # PI control vectoring (vertical)
        u[2] = -(Kp[2]*self.headingAngle + Ki[2]*self.headingAngleInt)   # PI control vectoring (vertical)
        u[3] = -(Kp[3]*self.err[2] + Ki[3]*self.integral[2])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] + Ki[4]*self.integral[4])   # PI control lcg

        return u



    def computePIDControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]

        # Gains for distance based error
        Kp = np.array([10, 10, 10, 150, 1000])        # P control gain
        Ki = np.array([1, 0.1, 0.1, 3, 10])                # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])

        u = np.array([0., 0., 0., 0., 50.])

        # Error calculation
        self.errPrev = self.err.copy()
        self.err = self.ref - self.current_x
        self.integral += self.err * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        # Calculate distance to reference pose
        self.distanceErrPrev = self.distanceErr
        self.distanceErr = np.sqrt(self.err[0]**2 + self.err[1]**2) * np.sign(math.atan2(self.err[1], self.err[0]))
        self.distanceErrInt += self.distanceErr * (1/self.loop_freq)
        self.distanceErrDeriv = (self.distanceErr - self.distanceErrPrev) * self.loop_freq        

        # Controller
        u[0] = (Kp[0]*self.distanceErr \
            + Ki[0]*self.distanceErrInt \
            + Kd[0]*self.distanceErrDeriv)    # PID control thrusters    
        u[1] = (Kp[1]*self.err[5] + Ki[1]*self.integral[5])   # PI control vectoring (horizontal)
        u[2] = (Kp[2]*self.err[2] + Ki[2]*self.integral[2])   # PI control vectoring (vertical)
        u[3] = -(Kp[3]*self.err[2] + Ki[3]*self.integral[2])   # PI control vbs
        u[4] = -(Kp[4]*self.err[4] + Ki[4]*self.integral[4])   # PI control lcg

        return u

    def limitControlAction(self,u):
        # Enforce hardware limits on actuator control
        uLimited = u.copy()     # without .copy(), python only makes a shallow copy of the array.

        # vbs limit
        if uLimited[3] > 100:
            uLimited[3] = 100
        if uLimited[3] < 0:
            uLimited[3] = 0

        # lcg limit
        if uLimited[4] > 100:
            uLimited[4] = 100
        if uLimited[4] < 0:
            uLimited[4] = 0

        # print("All in ENU:")
        # print("[x, y, z, roll, pitch, yaw]")
        # self.printNumpyArray(self.current_x,"Current States: %.4f %.4f %.4f %.4f %.4f %.4f\n")
        # self.printNumpyArray(self.ref,"Reference States: %.4f %.4f %.4f %.4f %.4f %.4f\n")
        # self.printNumpyArray(self.err,"Control Error: %.4f %.4f %.4f %.4f %.4f %.4f\n")
        # sys.stdout.write("Distance Error: %.4f, Heading Angle: %.4f\n" % (self.distanceErr, self.headingAngle))    
        # print("[thruster, vec (horizontal), vec (vertical), vbs, lcg]")
        # print("Control Input raw:", np.around(u, decimals=3))
        # print("Control Input lim:", np.around(uLimited, decimals=3))
        # print("")



        return uLimited

    def printNumpyArray(self, array, string):
        sys.stdout.write(string  % (array[0], array[1], array[2], array[3], array[4], array[5]))
        

if __name__ == "__main__":
    rospy.init_node("p_controller")
    controller = P_Controller(rospy.get_name())
