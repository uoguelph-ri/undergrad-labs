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


class Camera(object):
    '''
         IMPORTANT: You can only have two cameras open at a time at standard
         resolutions, due to bandwidth limitations.
         The hand cameras are opened by default at boot-time.

        Parameters:
            camera_name: the name of the camera {right, left, head}
            resolution: image resolution, max (960, 600)
            exposure: camera exposure, 0-100 or auto
            gain: camera gain, 0-79 or auto
    '''

    def __init__(self, camera_name='right',
                 resolution=(960, 600), exposure='auto', gain='auto'):

        if camera_name not in ['right', 'left', 'head']:
            raise LookupError('side must be "right", "left", or "head"')

        self.image = None
        self.annotated_image = None

        self.topic_name = '/cameras/right_hand_camera/image'
        self.camera = CameraController('right_hand_camera')
        if camera_name == 'left':
            self.topic_name = '/cameras/left_hand_camera/image'
            self.camera = CameraController('left_hand_camera')
        elif camera_name == 'head':
            self.topic_name = '/cameras/head_camera/image'
            self.camera = CameraController('head_camera')

        self.camera.resoution = resolution
        if exposure == 'auto':
            self.camera.exposure = CameraController.CONTROL_AUTO
        else:
            self.camera.exposure = exposure

        if gain == 'auto':
            self.camera.gain = CameraController.CONTROL_AUTO
        else:
            self.camera.gain = gain

        self.image_subscriber = rospy.Subscriber(
            self.topic_name, Image, callback=self.__Callback, queue_size=1)

    def __Callback(self, message):
        # convert ROS iamge to OpenCV image
        self.cv2_image = bridge.imgmsg_to_cv2(message)
        self.image = self.cv2_image.copy()

        # this is where we do our image processing.
        # We're just going to draw a red rectangle and some text
        # NOTE: OpenCV stores colors in (blue, green, red) order instead of RGB
        cv2.rectangle(
            self.cv2_image,
            pt1=(280, 200),
            pt2=(680, 400),
            color=(0, 0, 255),
            thickness=5
        )
        cv2.putText(
            self.cv2_image,
            text='target: {}'.format('unknown'),
            org=(400, 500),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=2,
            color=(255, 255, 0)
        )

        self.annotated_image = self.cv2_image

        # convert OpenCV image back to ROS image and show on screen
        # TODO: causing notebook issues
        # ros_image = bridge.cv2_to_imgmsg(cv2_image)
        # self.__display_publisher.publish(ros_image)


class ArmRangeSensor(object):
    def __init__(self, side='right'):

        if side not in ['right', 'left']:
            raise LookupError('side must be "right" or "left"')

        self.distance = None

        self.topic_name = '/robot/range/{}_hand_range/state'.format(side)

        self.__sensor = rospy.Subscriber(
            self.topic_name, Range, callback=self.__Callback, queue_size=1)

    def __Callback(self, msg):
        self.distance = msg.range


class ArmWorldPosition(object):
    def __init__(self, side='right'):

        if side not in ['right', 'left']:
            raise LookupError('side must be "right" or "left"')

        self.endpoint = None

        self.topic_name = '/robot/limb/{}/endpoint_state'.format(side)

        self.__position = rospy.Subscriber(
            self.topic_name, EndpointState, callback=self.__Callback, queue_size=1)

    def __Callback(self, msg):
        self.endpoint = msg


class ArmInverseKinematicsSolver(object):
    def __init__(self, side='right'):

        if side not in ['right', 'left']:
            raise LookupError('side must be "right" or "left"')

        self.topic_name = \
            'ExternalTools/{}/PositionKinematicsNode/IKService'.format(side)

        self.service_ik = rospy.ServiceProxy(self.topic_name, SolvePositionIK)
        self.request_ik = SolvePositionIKRequest()
        self.header_ik = Header(stamp=rospy.Time.now(), frame_id='base')

        self.solution = None

    def solve(self, pose):

        self.pose_stamped = PoseStamped(
            header=self.header_ik,
            pose=pose
        )

        self.request_ik.pose_stamp[:] = []
        self.request_ik.pose_stamp.append(self.pose_stamped)

        try:
            rospy.wait_for_service(self.topic_name, 5.0)
            self.ik_response = self.service_ik(self.request_ik)
            if (self.ik_response.isValid[0]):
                self.solution = dict(
                    zip(self.ik_response.joints[0].name, self.ik_response.joints[0].position))
            else:
                self.solution = None
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))

        return self.solution
