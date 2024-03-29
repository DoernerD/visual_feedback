#!/usr/bin/python
"""
Simple node to publish the TRUE relative transformation from
sam/base_link to the docking_station/base_link, on sam/base_link's
frame of reference.
"""
# TODO: Got hardcoded values everywhere.

from __future__ import division, print_function
import sys

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == "__main__":
    rospy.init_node("sam_tfm_dock_publisher", anonymous=True)

    # Keeping message type consistent, publishing same
    # pose message as our perception module.
    pose_pub = rospy.Publisher("/docking_station/gt/pose",
                               PoseWithCovarianceStamped, queue_size=10)
    sam_pub = rospy.Publisher("/sam/gt/pose",
                               PoseWithCovarianceStamped, queue_size=10)
    # self.publishers["found_station"] = rospy.Publisher("found_station", Bool, queue_size=1)

    # We're gonna use tf2 since it's more stable than tf.
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    br = tf.TransformBroadcaster()

    # Frames are kept constant.
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map" #"sam/base_link"
    # FIXME: Covariance needed?
    samPose = PoseWithCovarianceStamped()
    samPose.header.frame_id = "sam/base_link/estimated"

    rate = rospy.Rate(20)
    # Run publisher
    while not rospy.is_shutdown():
        try:
            # Get tfm ^S T_{S/DS}.
            # sam_tfm_ds = tf_buffer.lookup_transform(
            #     'docking_station/base_link', 'sam/base_link', rospy.Time())

            sam_tfm_ds = tf_buffer.lookup_transform(
                'map','docking_station/base_link', rospy.Time())
                
            # Apparently pose_msg uses point and tfm_msg uses vector...
            pose.pose.pose.position.x = sam_tfm_ds.transform.translation.x
            pose.pose.pose.position.y = sam_tfm_ds.transform.translation.y
            pose.pose.pose.position.z = sam_tfm_ds.transform.translation.z
            pose.pose.pose.orientation = sam_tfm_ds.transform.rotation
            pose.header.stamp = rospy.Time.now()

        except:
            rospy.logwarn("Can't find transform: sam to docking_station")
            continue

        try:
            # Get tfm ^S T_{S/DS}.
            # sam_tfm_ds = tf_buffer.lookup_transform(
            #     'docking_station/base_link', 'sam/base_link', rospy.Time())

            # This only works in stonefish because we have the ground truth!
            sam_tfm_map = tf_buffer.lookup_transform(
                'map','sam/base_link', rospy.Time())
                
            # Apparently pose_msg uses point and tfm_msg uses vector...
            samPose.pose.pose.position.x = sam_tfm_map.transform.translation.x
            samPose.pose.pose.position.y = sam_tfm_map.transform.translation.y
            samPose.pose.pose.position.z = sam_tfm_map.transform.translation.z
            samPose.pose.pose.orientation = sam_tfm_map.transform.rotation
            samPose.header.stamp = rospy.Time.now()

            

        except:
            rospy.logwarn("Can't find transform: sam to map")
            continue

        br.sendTransform((samPose.pose.pose.position.x, 
                              samPose.pose.pose.position.y, 
                              samPose.pose.pose.position.z),
                             (samPose.pose.pose.orientation.x,
                              samPose.pose.pose.orientation.y,
                              samPose.pose.pose.orientation.z,
                              samPose.pose.pose.orientation.w),
                             rospy.Time.now(),
                             "sam/base_link/estimated",
                             "map")

        # sys.stdout.write("Current States: %.4f %.4f %.4f\n SAM Position: %.4f %.4f %.4f\n---\n" % (
        #     pose.pose.pose.position.x,
        #     pose.pose.pose.position.y, 
        #     pose.pose.pose.position.z,
        #     samPose.pose.pose.position.x,
        #     samPose.pose.pose.position.y, 
        #     samPose.pose.pose.position.z))
        
        pose_pub.publish(pose)
        sam_pub.publish(samPose)

        rate.sleep()
