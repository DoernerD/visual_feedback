#!/usr/bin/python3
# Copyright 2022 David Doerner (ddorner@kth.se)

from __future__ import division, print_function

import curses
import math
import numpy as np

import rospy

from tf.transformations import euler_from_quaternion
import tf

from sam_msgs.msg import ThrusterAngles, PercentStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from smarc_msgs.msg import ThrusterRPM


class WaypointFollowingController(object):
    """
    Waypoint following controller with depth first control.
    """

    # Main loop
    def __init__(self, name):

        # Launch File Parameters
        self.loop_freq = rospy.get_param("~loop_freq", 20)
        verbose = rospy.get_param("~verbose", True)

        # Init
        self.state_estimated = np.array([0., 0., 0.1, 1., 0., 0.])
        self.x_ref = 0.
        self.y_ref = 0.
        self.z_ref = 0.
        self.roll_ref = 0.
        self.pitch_ref = 0.
        self.yaw_ref = 0.
        self.ref = np.array([self.x_ref, self.y_ref, self.z_ref, self.roll_ref, self.pitch_ref, self.yaw_ref])

        self.velocity = np.array([0., 0., 0., 0., 0., 0.])
        self.vel_x_ref = 0.
        self.vel_y_ref = 0.
        self.vel_z_ref = 0.
        self.vel_roll_ref = 0.
        self.vel_pitch_ref = 0.
        self.vel_yaw_ref = 0.
        self.vel_ref = np.array([self.vel_x_ref, self.vel_y_ref, self.vel_z_ref, self.vel_roll_ref, self.vel_pitch_ref, self.vel_yaw_ref])

        # Control Gains
        self.Kp = np.array([1000, 5, 5, 40, 60])      # P control gain
        self.Ki = np.array([10., 0.1, 0.1, 0.1, 0.1])    # I control gain
        self.Kd = np.array([1., 1., 1., 1., 6.])    # D control gain
        self.Kaw = np.array([1., 1., 1., 1., 6.])   # Anti windup gain

        self.eps_depth = 0.4 # offset for depth control
        self.eps_pitch = 0.2 # offset for pitch control

        self.error = np.array([0., 0., 0., 0., 0., 0.])
        self.error_prev = np.array([0., 0., 0., 0., 0., 0.])
        self.integral = np.array([0., 0., 0., 0., 0., 0.])

        self.distance_error = 0.

        self.heading_angle_error = 0.
        self.heading_angle_error_integral = 0.
        self.heading_angle_error_prev = 0.
        self.heading_angle_error_deriv = 0.

        self.error_velocity = 0.
        self.error_velocity_integral = 0.
        self.error_velocity_prev = 0.
        self.error_velocity_deriv = 0.

        self.stop_radius = 0.2
        self.rpm_limit = 250.   # hard rpm limit for safety in the tank

        # Neutral actuator inputs
        self.thruster_neutral = 0
        self.horizontal_thrust_vector_neutral = 0.
        self.vertical_thrust_vector_neutral = 0.
        self.vbs_neutral = 52.
        self.lcg_neutral = 35.

        self.u_neutral = np.array([self.thruster_neutral,
                                  self.horizontal_thrust_vector_neutral,
                                  self.vertical_thrust_vector_neutral,
                                  self.vbs_neutral,
                                  self.lcg_neutral])

        self.anti_windup_diff = np.array([0., 0., 0., 0., 0.])
        self.anti_windup_diff_integral = np.array([0., 0., 0., 0., 0.])

        self.limit_output_cnt = 0

        # Topics for feedback and actuators
        vbs_topic = rospy.get_param("~vbs_topic", "/sam/core/vbs_cmd")
        lcg_topic = rospy.get_param("~lcg_topic", "/sam/core/lcg_cmd")
        rpm1_topic = rospy.get_param("~rpm_topic_1", "/sam/core/thruster1_cmd")
        rpm2_topic = rospy.get_param("~rpm_topic_2", "/sam/core/thruster2_cmd")
        thrust_vector_cmd_topic = rospy.get_param("~thrust_vector_cmd_topic", "/sam/core/thrust_vector_cmd")

        ref_pose_topic = rospy.get_param("~ref_pose_topic")
        state_estimate_topic = rospy.get_param("~state_estimation_topic")

        # Subscribers to state feedback, setpoints and enable flags
        rospy.Subscriber(ref_pose_topic, Odometry, self.waypoint_callback, queue_size=1)
        rospy.Subscriber(state_estimate_topic, Odometry, self.estimation_callback, queue_size=1)

        # Publisher to actuators
        self.rpm1_pub = rospy.Publisher(rpm1_topic, ThrusterRPM, queue_size=1)
        self.rpm2_pub = rospy.Publisher(rpm2_topic, ThrusterRPM, queue_size=1)
        self.thrust_vector_pub = rospy.Publisher(thrust_vector_cmd_topic, ThrusterAngles, queue_size=10)
        self.vbs_pub = rospy.Publisher(vbs_topic, PercentStamped, queue_size=10)
        self.lcg_pub = rospy.Publisher(lcg_topic, PercentStamped, queue_size=10)

        # TF tree listener        
        self.listener = tf.TransformListener()
        self.base_frame = 'sam/base_link'

        rate = rospy.Rate(self.loop_freq)

        rospy.logwarn("[WPF]: Controller Initialized")

        if verbose:
            self.console = curses.initscr()  # initialize is our playground

        # Run
        while not rospy.is_shutdown():

            u = self.compute_control_action()

            u_limited = self.limit_control_action(u)

            self.compute_anti_windup(u, u_limited)

            self.publish_control_action(u_limited)

            if verbose:
                self.print_states(u, u_limited)

            rate.sleep()

        if verbose:
            curses.endwin()  # return control back to the console


    #region Call-backs
    def estimation_callback(self, estim):
        """
        Get the current state of the vehicle from the state estimation node.
        """
        self.state_estimated = self.get_euler_from_quaternion(estim.pose)
        self.velocity = [estim.twist.twist.linear.x,
                         estim.twist.twist.linear.y,
                         estim.twist.twist.linear.z,
                         estim.twist.twist.angular.x,
                         estim.twist.twist.angular.y,
                         estim.twist.twist.angular.z]


    def waypoint_callback(self,waypoint_ref):
        """
        Get the current waypoint from the planner node and convert it to the base frame.
        """
        waypoint_euler = self.get_euler_from_quaternion(waypoint_ref.pose)

        # Transform waypoint map --> base frame
        goal_point = PointStamped()
        goal_point.header.frame_id = 'sam/odom'
        goal_point.header.stamp = rospy.Time(0)
        goal_point.point.x = waypoint_euler[0]
        goal_point.point.y = waypoint_euler[1]
        goal_point.point.z = waypoint_euler[2]

        try:
            goal_point_local = self.listener.transformPoint(
                self.base_frame, goal_point)
                        
            # Pose references
            self.ref[0] = goal_point_local.point.x
            self.ref[1] = goal_point_local.point.y

            # We don't transform the depth and the pitch
            # since we compare them to sensor data and 
            # therefore need absolute values.
            self.ref[2] = waypoint_euler[2]
            self.ref[4] = waypoint_euler[4]

            # Velocity references
            self.vel_ref[0] = waypoint_ref.twist.twist.linear.x

        except Exception as exception_msg:
            rospy.logwarn("[WPF]: Can't transform WP: {}".format(exception_msg))


    def get_euler_from_quaternion(self, pose):
        """
        Extracts the position and Euler angles.
        """
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        quat = [pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w]

        rpy = euler_from_quaternion(quat)

        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        states = np.array([x, y, z, roll, pitch, yaw])

        return states
    #endregion


    #region Publisher
    def publish_control_action(self, u):
        """
        Publish the control action to the actuators.
        """

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
        self.rpm1_pub.publish(thruster1)
        self.rpm2_pub.publish(thruster2)
        self.thrust_vector_pub.publish(vec)
        self.vbs_pub.publish(vbs)
        self.lcg_pub.publish(lcg)

    #endregion


    # Controller
    def compute_control_action(self):
        """
        Sliding Mode Control for Depth First control.
        The control structure is as follows:
            The error is used for pitch and depth control.
            The heading angle is used for horizontal control and calculated separately.
            The distance to the target is used for forward and backward control.
        u = [thruster, vec (horizontal), vec (vertical), vbs, lcg]
        x = [x, y, z, roll, pitch, yaw]
        """
        u = np.array([self.thruster_neutral,
                        self.horizontal_thrust_vector_neutral,
                        self.vertical_thrust_vector_neutral,
                        self.vbs_neutral,
                        self.lcg_neutral])

        self.error_prev = self.error
        self.error = self.ref - self.state_estimated

        # Depth is always negative, which is why we change the signs on the
        # depth error. Then we can keep the remainder of the control structure
        self.error[2] = -self.error[2]

        # Anti windup integral is calculated separately, because
        # dim(e) != dim(u).
        self.calculate_anti_windup_integral()

        self.integral += self.error * (1/self.loop_freq)
        self.deriv = (self.error - self.error_prev) * self.loop_freq

        # When on navigation plane
        if ((np.abs(self.state_estimated[2] - self.ref[2]) <= self.eps_depth)\
                and (np.abs(self.state_estimated[4] - self.ref[4]) <= self.eps_pitch)):

            # Compute heading angle
            self.heading_angle = math.atan2(self.ref[1], self.ref[0])
            self.heading_angle_error_prev = self.heading_angle_error
            self.heading_angle_error = 0 - self.heading_angle
            self.heading_angle_error_integral += self.heading_angle_error * (1/self.loop_freq)
            self.heading_angle_error_deriv = (self.heading_angle_error - self.heading_angle_error_prev) * self.loop_freq

            # Calculate distance to reference pose
            # Workaround bc np.sign(0) = 0
            if np.sign(self.ref[0]) == 0:
                distance_sign = 1
            else:
                distance_sign = np.sign(self.ref[0])

            self.distance_error = np.sqrt(self.ref[0]**2 + self.ref[1]**2) * distance_sign

            # FIXME: Check the u[1] calculation. Seems sketchy with the flipping and esp. with the integral when you change signs. Not good!
            if self.distance_error > self.stop_radius:
                u[0] = self.calculate_velocity_control_action(self.vel_ref[0])

                u[1] = self.Kp[1]*self.heading_angle + self.Ki[1]*(self.heading_angle_error_integral - self.anti_windup_diff_integral[1]) + self.Kd[1]*self.heading_angle_error_deriv   # PID control vectoring (horizontal)
                u[1] = -u[1] # FIXME: This is a hack to get the sign right. This is due to the conversion from ENU to NED for the thruster commands

            elif self.distance_error < -self.stop_radius:
                u[0] = self.calculate_velocity_control_action(-0.5*self.vel_ref[0])

                self.heading_angle_scaled = np.sign(self.heading_angle) * (np.pi - np.abs(self.heading_angle))

                u[1] = -(self.Kp[1]*self.heading_angle_scaled + (self.Ki[1]*(self.heading_angle_error_integral - self.anti_windup_diff_integral[1])) + self.Kd[1]*self.heading_angle_error_deriv)   # PID control vectoring (horizontal)
                u[1] = -u[1] # FIXME: This is a hack to get the sign right. This is due to the conversion from ENU to NED for the thruster commands

            else:
                u[0] = self.calculate_velocity_control_action(0)

        u[3] = self.Kp[3]*self.error[2] + self.vbs_neutral + self.Ki[3]*(self.integral[2] - self.anti_windup_diff_integral[3]) + self.Kd[3]*self.deriv[2]   # PID control vbs
        u[4] = self.Kp[4]*self.error[4] + self.lcg_neutral + self.Ki[4]*(self.integral[4] - self.anti_windup_diff_integral[4]) + self.Kd[4]*self.deriv[4]   # PID control lcg

        return u


    def calculate_anti_windup_integral(self):
        """
        Calculate the anti windup integral
        """
        self.anti_windup_diff_integral += (self.anti_windup_diff) * (1/self.loop_freq)


    def calculate_velocity_control_action(self, velocity_desired):
        """
        Returns RPM based on the desired velocity in x direction.
        """
        u = 0

        self.error_velocity_prev = self.error_velocity
        self.error_velocity = velocity_desired - self.velocity[0]
        self.error_velocity_integral += self.error_velocity * (1/self.loop_freq)
        self.error_velocity_deriv = (self.error_velocity - self.error_velocity_prev) * self.loop_freq

        u = self.Kp[0]*self.error_velocity + self.Ki[0]*(self.error_velocity_integral - self.anti_windup_diff_integral[0]) + self.Kd[0]*self.error_velocity_deriv

        return u


    def limit_control_action(self,u):
        """
        Take hardware limitations into account and limit actuator outputs.
        """
        # Enforce hardware limits on actuator control
        u_limited = u.copy()     # without .copy(), python only makes a shallow copy of the array.

        # rpm limit
        if u_limited[0] > self.rpm_limit:
            u_limited[0] = self.rpm_limit
        elif u_limited[0] < -self.rpm_limit:
            u_limited[0] = -self.rpm_limit

        # thrust vector limit horizontal
        if u_limited[1] > np.deg2rad(7):
            u_limited[1] = np.deg2rad(7)
        elif u_limited[1] < -np.deg2rad(7):
            u_limited[1] = -np.deg2rad(7)

        # thrust vector limit vertical
        if u_limited[2] > np.deg2rad(7):
            u_limited[2] = np.deg2rad(7)
        elif u_limited[2] < -np.deg2rad(7):
            u_limited[2] = -np.deg2rad(7)

        # vbs limit
        if u_limited[3] > 100:
            u_limited[3] = 100
        if u_limited[3] < 0:
            u_limited[3] = 0

        # lcg limit
        if u_limited[4] > 100:
            u_limited[4] = 100
        if u_limited[4] < 0:
            u_limited[4] = 0

        return u_limited


    def compute_anti_windup(self, u, u_limited):
        """
        Compute anti wind up difference.
        """
        self.anti_windup_diff = self.Kaw * (u_limited - u)


    def print_states(self, u, u_limited):
        """
        Print function for system states
        use print("string {}".format(data)) instead. Python will format data accordingly
        and print it in place of {}. To print multiple variables, use multiple {}.
        Using the curses package, we can overwrite the console output. Is a bit weird when we have
        other cli outputs, too, but makes looking at the control stuff a lot easier.
        """
        # FIXME: Get the useful stuff for the print, right now they might not make sense.
        np.set_printoptions(suppress=True)
        self.limit_output_cnt += 1
        if self.limit_output_cnt % 20 == 1:
            self.console.addstr(0,0, "All in ENU: [x, y, z, roll, pitch, yaw]")
            self.console.addstr(1,0, (""))
            self.console.addstr(2,0, "Current States: {}".format(np.array2string(self.state_estimated, precision = 2, suppress_small = True, floatmode = 'fixed')))
            self.console.addstr(3,0, "Reference States: {}".format(np.array2string(self.ref, precision = 2, suppress_small = True, floatmode = 'fixed')))
            self.console.addstr(4,0, "Distance Error: {:.4f}, Heading Angle: {:.2f}, Depth Error: {:.2f}, velocity error: {:.2f}".format(self.distance_error, self.heading_angle, self.error[2], self.error_velocity))
            self.console.addstr(5,0, (""))
            self.console.addstr(6,0, "                   [RPM,  hor,  ver,  vbs,  lcg]")
            self.console.addstr(7,0, "Control Input raw: {}".format(np.array2string(u, precision = 2, floatmode = 'fixed')))
            self.console.addstr(8,0, "Control Input lim: {}".format(np.array2string(u_limited, precision = 2, floatmode = 'fixed')))
            self.console.addstr(9,0, "Anti Windup Int: {}".format(np.array2string(self.anti_windup_diff_integral, precision = 2, floatmode = 'fixed')))
            self.console.addstr(10,0, "")
            self.console.refresh()
            # print("All in ENU: [x, y, z, roll, pitch, yaw]")
            # print("")
            # print("Current States: {}".format(np.array2string(self.state_estimated, precision = 2, suppress_small = True, floatmode = 'fixed')))
            # print("Reference States: {}".format(np.array2string(self.ref, precision = 2, suppress_small = True, floatmode = 'fixed')))
            # print("Distance Error: {:.4f}, Heading Angle: {:.2f}, Depth Error: {:.2f}".format(self.distance_error, self.heading_angle, self.error[2]))
            # print("")
            # print("[thruster, vec (horizontal), vec (vertical), vbs, lcg]")
            # print("Control Input raw: {}".format(np.array2string(u, precision = 2, floatmode = 'fixed')))
            # print("Control Input lim: {}".format(np.array2string(u_limited, precision = 2, floatmode = 'fixed')))
            # print("Anti Windup Int: {}".format(np.array2string(self.anti_windup_diff_integral, precision = 2, floatmode = 'fixed')))
            # print("-----")


if __name__ == "__main__":
    rospy.init_node("WaypointFollowingController")
    controller = WaypointFollowingController(rospy.get_name())
