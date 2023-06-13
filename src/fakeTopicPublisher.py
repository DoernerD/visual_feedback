#!/usr/bin/python3  
import rospy

import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import ThrusterAngles, PercentStamped

from sbg_driver.msg import SbgEkfEuler
from sensor_msgs.msg import Imu, FluidPressure

class PoseBroadcaster(object):
    def __init__(self,name):

        # Init
        # self.DSpose = PoseWithCovarianceStamped()
        
        # Topics
        frameName = rospy.get_param('~frameName')
        frameTopic = rospy.get_param('~frameTopic')

        # estimTopic = "docking_station/feature_model/estimated_pose"
        
        pressureTopic = "/sam/core/depth20_pressure"
        thrustVecTopic = "/sam/core/thrust_vector_cmd"
        thrustRPMTopic = "/sam/core/thruster1_cmd"
        sbgTopic = "/sam/sbg/ekf_euler"

        # Subscribers
        # For subscribing to a fake docking station position
        # rospy.Subscriber(frameTopic,
        #                 PoseWithCovarianceStamped,
        #                 self.handle_pose,
        #                 frameName)
        
        # Publishers        
        # self.estimPup = rospy.Publisher(estimTopic, PoseWithCovarianceStamped, queue_size=1)
        self.pressurePup = rospy.Publisher(pressureTopic, FluidPressure, queue_size=1)
        self.thrustVecPub = rospy.Publisher(thrustVecTopic, ThrusterAngles, queue_size=1)
        self.thrustRPMPub = rospy.Publisher(thrustRPMTopic, ThrusterRPM, queue_size=1)
        self.sbgTopic = rospy.Publisher(sbgTopic, SbgEkfEuler, queue_size=1)
        
        

        # Run
        while not rospy.is_shutdown():
            self.publishPoses()



    # def handle_pose(self, msg, frameName):
    #     br = tf.TransformBroadcaster()
    #     br.sendTransform((msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z),
    #                     (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w),
    #                     rospy.Time.now(),
    #                     frameName,
    #                     "map")
        
    #     self.DSpose = PoseWithCovarianceStamped()
    #     self.DSpose.header.frame_id = 'map'
    #     self.DSpose.header.stamp = rospy.Time(0)
    #     self.DSpose.pose.pose.position.x = msg.pose.pose.position.x
    #     self.DSpose.pose.pose.position.y = msg.pose.pose.position.y
    #     self.DSpose.pose.pose.position.z = msg.pose.pose.position.z
    #     self.DSpose.pose.pose.orientation.x = msg.pose.pose.orientation.x
    #     self.DSpose.pose.pose.orientation.y = msg.pose.pose.orientation.y
    #     self.DSpose.pose.pose.orientation.z = msg.pose.pose.orientation.z
    #     self.DSpose.pose.pose.orientation.w = msg.pose.pose.orientation.w

        # print("[FTP] DSpose: x = {}, y = {}".format(self.DSpose.pose.pose.position.x, self.DSpose.pose.pose.position.y))
         
        
    def publishPoses(self):
        # Fake values if needed
        pressure = FluidPressure()
        pressure.fluid_pressure = 10000
        thruster1 = ThrusterRPM()
        vec = ThrusterAngles()
        sbgValue = SbgEkfEuler()
        sbgValue.angle.x = 0.
        sbgValue.angle.y = 0.
        sbgValue.angle.z = 0.

        thruster1.rpm = 0
        vec.thruster_horizontal_radians = 0
        vec.thruster_vertical_radians = 0


        # self.estimPup.publish(self.DSpose)
        self.pressurePup.publish(pressure)
        self.thrustRPMPub.publish(thruster1)
        self.thrustVecPub.publish(vec)
        self.sbgTopic.publish(sbgValue)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    poseBroadcaster = PoseBroadcaster(rospy.get_name())
