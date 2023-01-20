#!/usr/bin/python
# Copyright 2022 David Doerner (ddorner@kth.se)

from __future__ import division, print_function

import numpy as np

import rospy

from std_msgs.msg import Float64
from sam_msgs.msg import PercentStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class DockingStation(object):

    def __init__(self, name):

        # Topics
        dockingStationPoseTopic = rospy.get_param("~dockingStationPoseTopic", "/dockingStation/pose")
        
        # Publisher docking station pose
        self.posePub = rospy.Publisher(dockingStationPoseTopic, PoseStamped, queue_size=10)

        self.loop_freq = rospy.get_param("~loop_freq", 11)
        rate = rospy.Rate(self.loop_freq) 

        # Run
        while not rospy.is_shutdown():

            self.publishPose()

            rate.sleep()

    def publishPose(self):
        # Pose in ENU
        pose = PoseStamped()

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "docking_station"

        pose.pose.position.x = 0.0
        pose.pose.position.y = 10.0  #20.0
        pose.pose.position.z = -1.5


        roll = 0 
        pitch = 0
        yaw = 0 

        quaternion = quaternion_from_euler(roll, pitch, yaw, 'sxyz')

        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]
        pose.pose.orientation.w = quaternion[0]

        self.posePub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("dockingStation")
    dockingStation = DockingStation(rospy.get_name())