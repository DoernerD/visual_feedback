#!/usr/bin/python3
"""
ROS Node for simple path planner for underwater proxops

Copyright 2023 David Doerner (ddorner@kth.se)
"""
from __future__ import division, print_function

import numpy as np

from simple_path_planner import SimplePathPlanner

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import tf

class PlannerNode(object):
    """
    Planner Node Class, with all the interfaces to the planner of choice.
    """

    def __init__(self):
        # External Parameter
        self.loop_freq = rospy.get_param("~loop_freq", 20)
        rate = rospy.Rate(self.loop_freq)
        verbose = rospy.get_param("~verbose", True)

        # Init planner
        self.planner = SimplePathPlanner()

        # Topics
        docking_station_pose_topic = rospy.get_param("~docking_station_pose_topic")
        state_estimate_topic = rospy.get_param("~state_estimation_topic")
        waypoint_topic = rospy.get_param("~waypoint_topic")

        # Subscribers
        rospy.Subscriber(docking_station_pose_topic, Odometry,
                         self.docking_station_pose_cb, queue_size=1)
        rospy.Subscriber(state_estimate_topic, Odometry,
                         self.state_estimat_cb, queue_size=1)

        # Publisher
        self.waypoint_pub = rospy.Publisher(waypoint_topic, PoseWithCovarianceStamped, queue_size=1)

        # TF tree listener
        self.listener = tf.TransformListener()
        self.base_frame = 'sam/base_link'   # TODO: put that into the launch file.
        self.planning_frame = 'sam/odom'    # The frame we get sam and the docking station in. 


        # Run
        while not rospy.is_shutdown():
            # print("Receiving goal?")
            if self.planner.received_goal:
                # 1. Check if you need to calculate a plan
                # print("Checking for plan")
                plan_needed = self.planner.check_require_plan()
                # 1.1 Yes: Calculate Plan
                if plan_needed:
                    print("Planning")
                    if self.planner.check_in_feasible_region():
                        self.planner.calculate_path_segments()
                        self.planner.calculate_bezier_curve()

                        self.transform_results()

                        self.planner.path_calculated = True

                        # Plot Path -> saves to file
                    #     self.planner.plot_path()
                    #     self.planner.plot_tf_sam_base()
                    # else:
                    #     self.planner.plot_position()
                # 1.2 No: continue

                if self.planner.path_calculated:
                    # 2. Get current waypoint
                    current_position_map = self.planner.start_map[0:2]
                    self.planner.get_current_waypoint(current_position_map)

                    # 3. Publish current waypoint
                    self.publish_current_waypoint()
            # else:
                # print("No goal available")

            if verbose:
                self.planner.print_states()

            # 4. Sleep
            rate.sleep()


    #region Callbacks
    def docking_station_pose_cb(self, docking_station_pose_msg):
        """
        Callback to extract the docking station pose and calculate the 
        corresponding target for docking.
        """
        # sam/odom frame, but with the orientation of the camera frame
        # To make things easier, we add a rotation to it, s.t. it's the same
        # orientation as sam/base_link, e.g. ENU
        docking_station_pose = self.get_pose_from_mgs(docking_station_pose_msg.pose)

        t_ds_pose = docking_station_pose[0:3]
        quat_ds_pose = quaternion_from_euler(*docking_station_pose[3:6])
        R_ds_pose = quaternion_matrix(quat_ds_pose)
        
        quat_rot_x = quaternion_from_euler(np.pi/2, 0., 0.)
        R_rot_x = quaternion_matrix(quat_rot_x)

        quat_rot_z = quaternion_from_euler(0., 0., np.pi/2)
        R_rot_z = quaternion_matrix(quat_rot_z)

        R_ds_prime = np.matmul(np.matmul(R_ds_pose, R_rot_x), R_rot_z)

        quat_ds_prime = quaternion_from_matrix(R_ds_prime)
        rpy_ds_prime = euler_from_quaternion(quat_ds_prime)


        self.planner.goal_map[0] = docking_station_pose[0]
        self.planner.goal_map[1] = docking_station_pose[1]
        self.planner.goal_map[2] = rpy_ds_prime[2]

        x_axis_docking_station, _ = self.planner.calculate_orientation_axes(self.planner.goal_map[2], -1)
        self.planner.target_map = np.array([self.planner.goal_map[0] + x_axis_docking_station[0],
                                        self.planner.goal_map[1] + x_axis_docking_station[1]])

        target_map = PoseWithCovarianceStamped()
        target_map.header.frame_id = 'sam/odom'
        target_map.header.stamp = rospy.Time(0)
        target_map.pose.pose.position.x = self.planner.target_map[0]
        target_map.pose.pose.position.y = self.planner.target_map[1]
        target_map.pose.pose.position.z = 0.
        target_map.pose.pose.orientation.x = docking_station_pose_msg.pose.pose.orientation.x
        target_map.pose.pose.orientation.y = docking_station_pose_msg.pose.pose.orientation.y
        target_map.pose.pose.orientation.z = docking_station_pose_msg.pose.pose.orientation.z
        target_map.pose.pose.orientation.w = docking_station_pose_msg.pose.pose.orientation.w

        self.planner.goal_base = self.transform_to_base(docking_station_pose_msg)
        self.planner.target_base = self.transform_to_base(target_map)

        self.planner.received_goal = True


    def transform_to_base(self, pose_arg):
        """
        Transform a pose into base frame.
        """
        pose_base = np.array([0., 0., 0.])

        # Transform into sam/baselink
        tmp_pose = PoseStamped()
        tmp_pose.header.frame_id = 'sam/odom'
        tmp_pose.header.stamp = rospy.Time(0)
        tmp_pose.pose.position.x = pose_arg.pose.pose.position.x
        tmp_pose.pose.position.y = pose_arg.pose.pose.position.y
        tmp_pose.pose.position.z = pose_arg.pose.pose.position.z
        tmp_pose.pose.orientation.x = pose_arg.pose.pose.orientation.x
        tmp_pose.pose.orientation.y = pose_arg.pose.pose.orientation.y
        tmp_pose.pose.orientation.z = pose_arg.pose.pose.orientation.z
        tmp_pose.pose.orientation.w = pose_arg.pose.pose.orientation.w

        try:
            goal_sam_base = self.listener.transformPose(self.base_frame, tmp_pose)

            pose_base[0] = goal_sam_base.pose.position.x
            pose_base[1] = goal_sam_base.pose.position.y

            rpy = euler_from_quaternion([goal_sam_base.pose.orientation.x,
                                         goal_sam_base.pose.orientation.y,
                                         goal_sam_base.pose.orientation.z,
                                         goal_sam_base.pose.orientation.w])
            pose_base[2] = rpy[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[SPP]: Transform to base frame not available yet")

        return pose_base


    def state_estimat_cb(self,estim):
        """
        Get SAM's pose from the DR into the planner
        """
        pose_sam = self.get_pose_from_mgs(estim.pose)
        self.planner.start_map[0] = pose_sam[0]
        self.planner.start_map[1] = pose_sam[1]
        self.planner.start_map[2] = pose_sam[5]


    def get_pose_from_mgs(self, pose):
        """
        Pose is in ENU. The callback extracts the position and Euler angles.
        """
        # Pose in ENU
        x_enu = pose.pose.position.x
        y_enu = pose.pose.position.y
        z_enu = pose.pose.position.z

        eta0_enu = pose.pose.orientation.w
        eta1_enu = pose.pose.orientation.x
        eta2_enu = pose.pose.orientation.y
        eta3_enu = pose.pose.orientation.z

        rpy_enu = euler_from_quaternion([eta1_enu,eta2_enu,eta3_enu,eta0_enu])

        roll_enu = rpy_enu[0]
        pitch_enu = rpy_enu[1]
        yaw_enu = rpy_enu[2]

        states = np.array([x_enu, y_enu, z_enu, roll_enu, pitch_enu, yaw_enu])

        return states
    #endregion

    def transform_results(self):
        """
        Function to transform the control and waypoints into the map frame 
        for plotting.
        """
        for pt in range(len(self.planner.control_points_base["x"])):
            control_point_pose = self.transform_waypoint_to_pose([self.planner.control_points_base["x"][pt],
                                                                  self.planner.control_points_base["y"][pt]],
                                                                  self.base_frame)
            control_point_map = self.transform_to_map(control_point_pose)
            self.planner.control_points_map["x"][pt] = control_point_map.pose.position.x
            self.planner.control_points_map["y"][pt] = control_point_map.pose.position.y

        # "storing" the latest orientation of the docking station to check if it changes
        # TODO: Find a better solution to this.
        self.planner.control_points_map["theta"][-1] = self.planner.goal_map[2]

        for pt in range(len(self.planner.path_base["x"])):
            waypoint_pose = self.transform_waypoint_to_pose([self.planner.path_base["x"][pt],
                                                             self.planner.path_base["y"][pt]],
                                                             self.base_frame)
            waypoint_map = self.transform_to_map(waypoint_pose)
            self.planner.path_map["x"][pt] = waypoint_map.pose.position.x
            self.planner.path_map["y"][pt] = waypoint_map.pose.position.y


    def publish_current_waypoint(self):
        """
        Transform the current waypoint into pose and publish it
        """
        current_waypoint_map = self.transform_waypoint_to_pose(self.planner.current_waypoint_map, self.planning_frame)

        self.waypoint_pub.publish(current_waypoint_map)


    def transform_waypoint_to_pose(self, waypoint, frame_id):
        """
        Transform the waypoint into a pose
        """
        waypoint_pose = PoseWithCovarianceStamped()
        waypoint_pose.header.frame_id = frame_id
        waypoint_pose.header.stamp = rospy.Time.now()
        waypoint_pose.pose.pose.position.x = waypoint[0]
        waypoint_pose.pose.pose.position.y = waypoint[1]
        waypoint_pose.pose.pose.position.z = 0
        waypoint_pose.pose.pose.orientation.w = 0
        waypoint_pose.pose.pose.orientation.x = 0
        waypoint_pose.pose.pose.orientation.y = 0
        waypoint_pose.pose.pose.orientation.z = 0

        return waypoint_pose


    def transform_to_map(self, pose_arg):
        """
        Transform a pose into map frame
        """
        pose_map = PoseStamped()

        # Transform into map
        tmp_pose = PoseStamped()
        tmp_pose.header.frame_id = pose_arg.header.frame_id
        tmp_pose.header.stamp = rospy.Time(0)
        tmp_pose.pose.position.x = pose_arg.pose.pose.position.x
        tmp_pose.pose.position.y = pose_arg.pose.pose.position.y
        tmp_pose.pose.position.z = pose_arg.pose.pose.position.z
        tmp_pose.pose.orientation.x = pose_arg.pose.pose.orientation.x
        tmp_pose.pose.orientation.y = pose_arg.pose.pose.orientation.y
        tmp_pose.pose.orientation.z = pose_arg.pose.pose.orientation.z
        tmp_pose.pose.orientation.w = pose_arg.pose.pose.orientation.w

        try:
            pose_map = self.listener.transformPose('sam/odom', tmp_pose)

        except Exception as exception_msg:
            rospy.logwarn("[SPP]: Transform to base frame failed: {}".format(exception_msg))

        return pose_map


if __name__ == "__main__":
    rospy.init_node("SimplePathPlanner")
    try:
        PlannerNode()
    except rospy.ROSInterruptException:
        rospy.logerr("[Planner]: Couldn't lauch planner node")
