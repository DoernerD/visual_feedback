#!/usr/bin/python3  
# Copyright David Doerner 2023

# Import Statements
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
import imutils 
from imutils import paths
import message_filters

from sensor_msgs.msg import Image, CameraInfo



# Stitcher Class
class ImageStitcher(object):
    def __init__(self, name):
        # Init

        self.bridge = CvBridge()
        
        self.forwardImage = Image()
        self.portImage = Image()
        self.starboardImage = Image()

        self.images = []

        self.stitchedImage = Image()

        self.stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create()
        self.status = 0

        # Topics

        self.right_image_topic = rospy.get_param("~right_image_topic")
        self.left_image_topic = rospy.get_param("~left_image_topic")
        self.front_image_topic = rospy.get_param("~front_image_topic")

        # Subscribers
        self.forwardSub = message_filters.Subscriber(self.front_image_topic, Image)
        self.starboardSub = message_filters.Subscriber(self.right_image_topic, Image)
        self.portSub = message_filters.Subscriber(self.left_image_topic, Image)

        self.camImagesSub = message_filters.ApproximateTimeSynchronizer([self.forwardSub, self.starboardSub, self.portSub],
                                                                2, slop=1., allow_headerless=False)
        self.camImagesSub.registerCallback(self.stitchCallback)

        # Publishers
        self.stitchedImagePub = rospy.Publisher('stitchedImage', Image, queue_size=10)


        # Run
        # while not rospy.is_shutdown():
        #     if self.forwardImage:
        #         try:
        #             imgForwardColor = self.bridge.imgmsg_to_cv2(self.forwardImage, 'bgr8')
        #             imgPortColor = self.bridge.imgmsg_to_cv2(self.portImage, 'bgr8')
        #             imgStarboardColor = self.bridge.imgmsg_to_cv2(self.starboardImage, 'bgr8')
                
        #             self.images.append(imgForwardColor)
        #             self.images.append(imgPortColor)
        #             self.images.append(imgStarboardColor)
                
        #         except CvBridgeError as e:
        #             print(e)

        #         self.stitchImages()
        #         self.publishImage()

        # pass
        rospy.spin()

    # Callbacks
    def stitchCallback(self, forwardImageMsg, starboardImageMsg, portImageMsg):
        # convert to openCV images
        try:
            imgForwardColor = self.bridge.imgmsg_to_cv2(forwardImageMsg, 'bgr8')
            imgPortColor = self.bridge.imgmsg_to_cv2(starboardImageMsg, 'bgr8')
            imgStarboardColor = self.bridge.imgmsg_to_cv2(portImageMsg, 'bgr8')
        
            self.images.append(imgForwardColor)
            self.images.append(imgPortColor)
            self.images.append(imgStarboardColor)
       

            (self.status, self.stitchedImage) = self.stitcher.stitch(self.images)

            
            
            if self.status == 0:
                print("Publishing")
                self.stitchedImagePub.publish(self.bridge.cv2_to_imgmsg(self.stitchedImage))
            else:
                print("[INFO] image stitching failed ({})".format(self.status))

            self.images = []

        except CvBridgeError as e:
            print(e)
        

    # def camForwardCallback(self,image):
    #     self.forwardImage = image

    # def camPortCallback(self,image):
    #     self.portImage = image

    # def camStarboardCallback(self,image):
    #     self.starboardImage = image

    # # Publishing
    # def publishImage(self):
    #     if self.status == 0:
    #         self.stitchedImagePub.publish(self.bridge.cv2_to_imgmsg(self.stitchedImage))
    #     else:
    #         print("[INFO] image stitching failed ({})".format(self.status))


    # # Functions
    # def stitchImages(self):
    #     print("Stitching Images")
    #     (self.status, self.stitchedImage) = self.stitcher.stitch(self.images)


# ROS Call
if __name__ == '__main__':
    rospy.init_node('imageStitcher')
    imageStitcher = ImageStitcher(rospy.get_name())
