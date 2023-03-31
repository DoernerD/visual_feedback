#!/usr/bin/python
"""
Simple node to publish the an arbitrary TF as a waypoint expressed on the
map frame.
"""

from __future__ import division, print_function

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == "__main__":
    rospy.init_node("tf_to_waypoint_publisher", anonymous=True)

    waypoint_pub = rospy.Publisher("/waypoint_topic",
                               PoseWithCovarianceStamped, queue_size=10)

    # We're gonna use tf2 since it's more stable than tf.
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    br = tf.TransformBroadcaster()

    # Frames are kept constant.
    waypoint = PoseWithCovarianceStamped()
    waypoint.header.frame_id = "map"

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            map_to_ds = tf_buffer.lookup_transform(
                'map','docking_station_planning', rospy.Time(0))

            # Apparently pose_msg uses point and tfm_msg uses vector...
            waypoint.pose.pose.position.x = map_to_ds.transform.translation.x
            waypoint.pose.pose.position.y = map_to_ds.transform.translation.y
            waypoint.pose.pose.position.z = map_to_ds.transform.translation.z
            # Add 90 degree rotation about Z axis to comply with waypoint axis.
            euler = tf.transformations.euler_from_quaternion([map_to_ds.transform.rotation.x,
                                                              map_to_ds.transform.rotation.y,
                                                              map_to_ds.transform.rotation.z,
                                                              map_to_ds.transform.rotation.w])
            quat = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] + 1.5707)
            waypoint.pose.pose.orientation.x = quat[0]
            waypoint.pose.pose.orientation.y = quat[1]
            waypoint.pose.pose.orientation.z = quat[2]
            waypoint.pose.pose.orientation.w = quat[3]
            waypoint.header.stamp = rospy.Time.now()

        except Exception as e:
            rospy.logerr(e)
            rate.sleep()
            continue

        waypoint_pub.publish(waypoint)
        rate.sleep()

