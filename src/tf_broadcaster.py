#!/usr/bin/python3  
import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

def handle_pose(msg, frameName):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     frameName,
                     "map")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    frameName = rospy.get_param('~frameName')
    frameTopic = rospy.get_param('~frameTopic')
    rospy.Subscriber(frameTopic,
                     PoseWithCovarianceStamped,
                     handle_pose,
                     frameName)
    rospy.spin()