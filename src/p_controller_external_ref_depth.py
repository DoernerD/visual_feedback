#!/usr/bin/python
# Copyright 2022 David Doerner (ddorner@kth.se)

from __future__ import division, print_function

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg

from std_msgs.msg import Float64
from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import ThrusterAngles, PercentStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


from tf.transformations import euler_from_quaternion, quaternion_from_euler


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

        ref_pose_topic = rospy.get_param("~ref_pose_topic", "/dockingStation/pose")

        # Subscribers to state feedback, setpoints and enable flags
        rospy.Subscriber(state_feedback_topic, Odometry, self.feedback_cb)
        rospy.Subscriber(ref_pose_topic, PoseStamped, self.refPoseCb)

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

        # Neutral actuator inputs
        self.vbsNeutral = 50
        self.lcgNeutral = 0
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
    def feedback_cb(self, odom_fb):
        [self.current_x,self.current_y] = self.getStateFeedback(odom_fb)

    def refPoseCb(self, pose):
        # Pose in ENU 
        self.refX = pose.pose.position.y
        self.refY = pose.pose.position.x
        self.refZ = -pose.pose.position.z

        eta0 = pose.pose.orientation.w
        eta1 = pose.pose.orientation.x
        eta2 = pose.pose.orientation.y
        eta3 = pose.pose.orientation.z

        rpy = euler_from_quaternion([eta1,eta2,eta3,eta0])
        roll = rpy[0]
        pitch = rpy[1]
        yaw = (np.pi - rpy[2])

        self.refRoll = roll
        self.refPitch = pitch
        self.refYaw = yaw

        self.ref = np.array([self.refX, self.refY, self.refZ, self.refRoll, self.refPitch, self.refYaw])

    def getStateFeedback(self, odom_msg):
        # Note: The callbacks are necessary, but we can define them ourselves.
        # # Converting from ENU to NED, e.g. see https://www.programmersought.com/article/83764943652/ or https://robotics.stackexchange.com/questions/19669/rotating-ned-to-enu
        # that is why we switch y and x, rotate the yaw by 90 degrees and have the opposite sign on z.
        x =  odom_msg.pose.pose.position.y
        y =  odom_msg.pose.pose.position.x
        z = -odom_msg.pose.pose.position.z
        eta0 = odom_msg.pose.pose.orientation.w
        eta1 = odom_msg.pose.pose.orientation.x
        eta2 = odom_msg.pose.pose.orientation.y
        eta3 = odom_msg.pose.pose.orientation.z

        rpy   = euler_from_quaternion([eta1,eta2,eta3,eta0])
        roll  = rpy[0]
        pitch = rpy[1]
        yaw   = (np.pi/2-rpy[2])

        # Velocities
        u =  odom_msg.twist.twist.linear.y
        v =  odom_msg.twist.twist.linear.x
        w = -odom_msg.twist.twist.linear.z        
        p =  odom_msg.twist.twist.angular.x
        q =  odom_msg.twist.twist.angular.y
        r = -odom_msg.twist.twist.angular.z

        current_state  = np.array([x,y,z,roll,pitch,yaw]) 
        current_output = np.array([x,z,pitch,u,w,q])

        return [current_state, current_output]
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
        vbs.value = u[3]
        lcg.value = u[4]

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
        # u = np.array([0., 0., 0., 0., 50.])
        epsDepth = 0.2 # offset for depth control
        epsPitch = 0.05 # offset for pitch control

        while ((np.abs(self.current_x[2] - self.ref[2]) >= epsDepth) and \
            (np.abs(self.current_x[4] - self.ref[4]) >= epsPitch)):

            u = self.computeDepthControlAction()
            return u
            
        u = self.computePIDControlAction()
        return u

    def computeDepthControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]  
        u = np.array([0., 0., 0., 0., 50.])

        Kp = np.array([40, 10, 10, 100, 1000])        # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 5, 10])       # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])
        u = np.array([0., 0., 0., 0., 0.])

        self.errPrev = self.err
        self.err = self.ref - self.current_x
        self.integral += self.err * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        u[3] = Kp[3]*self.err[2] + Ki[3]*self.integral[2]   # PI control vbs
        u[4] = Kp[4]*self.err[4] + Ki[4]*self.integral[4]   # PI control lcg

        return u


    def computePIDControlAction(self):
        # u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        # x = [x, y, z, roll, pitch, yaw]

        Kp = np.array([10, 10, 10, 100, 1000])        # P control gain
        Ki = np.array([0.5, 0.1, 0.1, 0.5, 10])                # I control gain
        Kd = np.array([1., 1., 1., 1., 1.])
        u = np.array([0., 0., 0., 0., 0.])

        self.errPrev = self.err
        self.err = self.ref - self.current_x
        self.integral += self.err * (1/self.loop_freq)
        self.deriv = (self.err - self.errPrev) * (self.loop_freq)

        u[0] = (Kp[0]*self.err[0] + Ki[0]*self.integral[0] \
            + Kp[0]*self.err[1] + Ki[0]*self.integral[1] \
            + Kd[0]*self.deriv[0] + Kd[0]*self.deriv[1])    # PID control thrusters
        # u[0] = (Kp[0]*self.err[0] \
        #     + Ki[0]*self.integral[0] \
        #     + Kd[0]*self.deriv[0])    # PID control thrusters
        u[1] = Kp[1]*self.err[5] + Ki[1]*self.integral[5]   # PI control vectoring (horizontal)
        u[2] = self.vecVerticalNeutral #Kp[2]*self.err[2] + Ki[2]*self.integral[2]   # PI control vectoring (vertical)
        u[3] = Kp[3]*self.err[2] + Ki[3]*self.integral[2]   # PI control vbs
        u[4] = Kp[4]*self.err[4] + Ki[4]*self.integral[4]   # PI control lcg

        return u

    def limitControlAction(self,u):
        # Enforce hardware limits on actuator control
        uLimited = u

        # np.clip(uLimited[0], -2000, 2000)
        # np.clip(uLimited[1], -0.15, 0.15)
        # np.clip(uLimited[2], 0, 100, out = uLimited[2])

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

        # np.clip(uLimited[3], 0, 100)

        print("[x, y, z, roll, pitch, yaw]")
        print("Current States: ", np.around(self.current_x, decimals=3))
        print("Reference States: ", np.around(self.ref, decimals=3))
        print("Control Error:", np.around(self.err, decimals=3))
        print("[thruster, vec (horizontal), vec (vertical), vbs, lcg]")
        print("Control Input:", np.around(uLimited, decimals=3))
        print("")

        return uLimited

if __name__ == "__main__":
    rospy.init_node("p_controller")
    controller = P_Controller(rospy.get_name())
