import rclpy
from rclpy.callback_groups import CallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import copy
import threading
import pathlib
import shutil
import time


# 硬件只支持D系列 Realsense 相机，
# L系列硬件太老需要微调SDK版本到2.4.2，默认ROS2 Humble不支持。
# ROS2 Humble下使用，只支持RGB流采集
class CameraSubscriber:
    def __init__(self, camera_name):
        # 录制相关
        self.if_record_start = False
        self.if_record_start_lock = threading.Lock()
        self.writer = None
        # 相机 rgb 数据采集
        self.camera_name = camera_name
        self.img = None
        self.img_lock = threading.Lock()
        self.rgb_subscriber = None

    def create_rgb_subscriber(self, node:rclpy.node.Node, camera_topic:str,
                              qos_profile: int, callback_group:rclpy.callback_groups.CallbackGroup):
        self.rgb_subscriber = node.create_subscription(Image, camera_topic, self.img_callback, qos_profile, callback_group=callback_group)

    def get_img(self):
        self.img_lock.acquire()
        img = copy.deepcopy(self.img)
        self.img_lock.release()
        return img

    def img_callback(self, msg):
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.img_lock.acquire()
        self.img = copy.deepcopy(img)
        self.img_lock.release()
