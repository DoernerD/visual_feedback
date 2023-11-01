#!/usr/bin/python3  
import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class PoseBroadcaster(object):
    def __init__(self,name):

        self.DSpose = PoseWithCovarianceStamped()
        
        frameName = rospy.get_param('~frameName')
        frameTopic = rospy.get_param('~frameTopic')

        estimTopic = "docking_station/feature_model/estimated_pose"
        self.estimPup = rospy.Publisher(estimTopic, PoseWithCovarianceStamped, queue_size=1)

        rospy.Subscriber(frameTopic,
                        Odometry,
                        self.handle_pose,
                        frameName)
        

        # Run
        while not rospy.is_shutdown():
            self.publishPoses()



    def handle_pose(self, msg, frameName):
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
                        (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
                        rospy.Time.now(),
                        frameName,
                        "sam/odom")
        
        self.DSpose = PoseWithCovarianceStamped()
        self.DSpose.header.frame_id = 'sam/odom'
        self.DSpose.header.stamp = rospy.Time(0)
        self.DSpose.pose.pose.position.x = msg.pose.pose.position.x
        self.DSpose.pose.pose.position.y = msg.pose.pose.position.x
        self.DSpose.pose.pose.position.z = msg.pose.pose.position.x
        self.DSpose.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.DSpose.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.DSpose.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.DSpose.pose.pose.orientation.w = msg.pose.pose.orientation.w
         
        
    def publishPoses(self):
        self.estimPup.publish(self.DSpose)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    poseBroadcaster = PoseBroadcaster(rospy.get_name())
