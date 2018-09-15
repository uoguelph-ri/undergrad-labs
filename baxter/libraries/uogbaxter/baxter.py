#!/bin/python2

import math
import copy
import time
import cv2
import matplotlib.pyplot as plt

import rospy
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

import baxter_interface
from baxter_interface import RobotEnable
from baxter_interface import CameraController
from baxter_interface import Gripper
from sensor_msgs.msg import Image, Range
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

bridge = CvBridge()


class ArmVision(object):
    
    def __init__(self, side='right'):
        
        self.topic_name = '/cameras/right_hand_camera/image'
        if side == 'left':
            self.topic_name = '/cameras/left_hand_camera/image'

        self.image_subscriber = rospy.Subscriber(self.topic_name, Image, callback=self.__processImageCallback, queue_size=1)
        self.image = None
        self.annotated_image = None
        
    def __processImageCallback(self, message):
        # convert ROS iamge to OpenCV image
        self.cv2_image = bridge.imgmsg_to_cv2(message)
        self.image = self.cv2_image.copy()
        
        # this is where we do our image processing. 
        # We're just going to draw a red rectangle and the distance we previously got from our distance sensor
        # note, that in OpenCV, colors are represented as (blue, green, red), rather than RGB
        cv2.rectangle(
                      self.cv2_image,
                      pt1=(280,200),
                      pt2=(680,400),
                      color=(0,0,255),
                      thickness=5
                      )
        cv2.putText(
                    self.cv2_image,
                    text='target: {}'.format('unknown'),
                    org=(400,500),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=2,
                    color=(255,255,0)
                    )
        
        self.annotated_image = self.cv2_image

        # convert OpenCV image back to ROS image
        #ros_image = bridge.cv2_to_imgmsg(cv2_image)
        
        # publish our modified image to the display
        #self.__display_publisher.publish(ros_image)