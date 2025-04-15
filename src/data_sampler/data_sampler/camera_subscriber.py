import rclpy
from rclpy.callback_groups import CallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
import struct
import cv2
import copy
import threading
import pathlib
import shutil
import time
import numpy as np

# 硬件只支持D系列 Realsense 相机，
# L系列硬件太老需要微调SDK版本到2.4.2，默认ROS2 Humble不支持。
# ROS2 Humble下使用，只支持RGB流采集
class CameraSubscriber:
    def __init__(self, camera_name: str = "camera", camera_id: int = 0, if_record: bool = False, sample_frequency: float = 30, prefix: str = "videos"):
        # 录制相关
        self.epoch = 0
        self.if_record = if_record
        self.if_record_start = False
        self.if_record_start_lock = threading.Lock()
        self.writer = None
        self.record_path = ""
        self.prefix = prefix
        self.sample_frequency = sample_frequency
        # 相机 rgb 数据采集
        self.camera_name = camera_name
        self.camera_id = camera_id
        # self.imgs = []
        # self.timestamps = []
        self.img = None
        self.img_lock = threading.Lock()
        self.rgb_subscriber = None
        # 深度图像数据
        self.depth_img = None
        self.depth_img_lock = threading.Lock()
        self.depth_subscriber = None
        # 点云数据
        self.pointcloud = None
        self.pointcloud_lock = threading.Lock()
        self.pointcloud_subscriber = None

    def create_rgb_subscriber(self, node:rclpy.node.Node, camera_topic:str,
                              qos_profile: int, callback_group:rclpy.callback_groups.CallbackGroup):
        if self.if_record:
            self.rgb_subscriber = node.create_subscription(Image, camera_topic, self.img_callback_with_recoder, qos_profile, callback_group=callback_group)
        else:
            self.rgb_subscriber = node.create_subscription(Image, camera_topic, self.img_callback, qos_profile, callback_group=callback_group)

    def create_depth_subscriber(self, node: rclpy.node.Node, depth_topic: str,
                              qos_profile: int, callback_group: rclpy.callback_groups.CallbackGroup):
        """创建深度图像订阅者"""
        self.depth_subscriber = node.create_subscription(
            Image, depth_topic, self.depth_callback, qos_profile, callback_group=callback_group)

    def create_pointcloud_subscriber(self, node: rclpy.node.Node, pointcloud_topic: str,
                                   qos_profile: int, callback_group: rclpy.callback_groups.CallbackGroup):
        """创建点云订阅者"""
        self.pointcloud_subscriber = node.create_subscription(
            PointCloud2, pointcloud_topic, self.pointcloud_callback, qos_profile, callback_group=callback_group)

    def get_img(self):
        self.img_lock.acquire()
        img = copy.deepcopy(self.img)
        self.img_lock.release()
        return img

    def get_depth_img(self):
        """获取深度图像"""
        self.depth_img_lock.acquire()
        depth_img = copy.deepcopy(self.depth_img)
        self.depth_img_lock.release()
        return depth_img



    def get_pointcloud(self):
        """获取点云数据，并返回包含颜色信息的6维数据 (x, y, z, r, g, b)"""
        self.pointcloud_lock.acquire()
        msg = copy.deepcopy(self.pointcloud)
        self.pointcloud_lock.release()
        if msg is None:
            return None

        # 转换为numpy数组
        pc_points = []
        for point in point_cloud2.read_points(msg, 
                                            field_names=("x", "y", "z", "rgb"),
                                            skip_nans=True):
            # 提取 x, y, z 坐标
            x, y, z, rgb = point
            
            # 解码 RGB 颜色值
            # 将浮点数解包为整数
            rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (rgb_int >> 16) & 0x0000ff  # 提取红色分量
            g = (rgb_int >> 8) & 0x0000ff   # 提取绿色分量
            b = rgb_int & 0x0000ff          # 提取蓝色分量
            
            # 归一化颜色值到 [0, 1] 范围（可选）
            r, g, b = r / 255.0, g / 255.0, b / 255.0
            
            # 添加到点云数据中
            pc_points.append([x, y, z, r, g, b])
        
        # 返回6维点云数据
        return np.array(pc_points)

        # self.pointcloud_lock.acquire()
        # pointcloud = copy.deepcopy(self.pointcloud)
        # print(f"frame___pointcloud: {pointcloud}")
        # self.pointcloud_lock.release()
        # return pointcloud

    def img_callback(self, msg):
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.img_lock.acquire()
        self.img = copy.deepcopy(img)
        self.img_lock.release()

    def img_callback_with_recoder(self, msg):
        img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.img_lock.acquire()
        self.img = copy.deepcopy(img)
        self.img_lock.release()
        self.if_record_start_lock.acquire()
        if self.if_record_start:
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'avc1')  # 注意：OpenCV可能不直接支持X264
                self.writer = cv2.VideoWriter(self.record_path, fourcc, self.sample_frequency, (img.shape[1], img.shape[0]))
            self.writer.write(img)
        self.if_record_start_lock.release()

    def depth_callback(self, msg):
        """深度图像回调函数"""
        depth_img = CvBridge().imgmsg_to_cv2(msg)
        self.depth_img_lock.acquire()
        self.depth_img = copy.deepcopy(depth_img)
        self.depth_img_lock.release()

    def pointcloud_callback(self, msg):
        """点云数据回调函数"""
        self.pointcloud_lock.acquire()
        self.pointcloud = copy.deepcopy(msg)
        self.pointcloud_lock.release()

    def start_record(self, record_path_prefix: str):
        if self.writer is not None:
            print("Recorder has been existed.")
            return False
        if not self.if_record:
            print("Not record mode.")
            return False
        # 处理文件路径
        if not self.prefix == "":
            record_path = pathlib.Path(record_path_prefix).joinpath(self.prefix)
        else:
            record_path = pathlib.Path(record_path_prefix)
        epoch = 0
        if record_path.exists():
            epoch = len(list(record_path.glob("*")))
        # 多相机录制时，该目录会被之前的相机先创建，但该目录没有自己的文件，这种情况下epoch 会-1
            record_path_check = record_path.joinpath(str(epoch - 1), f"{self.camera_id}.mp4")
            if record_path_check.parent.exists() and (not record_path_check.exists()):
                epoch -= 1

        record_path = record_path.joinpath(str(epoch), f"{self.camera_id}.mp4")
        record_path.parent.mkdir(parents=True, exist_ok=True)
        self.record_path = str(record_path)
        print(self.record_path)
        self.epoch = epoch + 1
        # 开启视频写入
        self.if_record_start_lock.acquire()
        self.if_record_start = True
        self.if_record_start_lock.release()
        return True

    def delete_record_data(self, not_exist_ok: bool = False):
        if not self.if_record:
            print("Not record mode.")
            return False
        if self.epoch < 1: # self.epoch >= 1 时一定使用过start_record
            print("No record data.")
            return False
        # 停止视频写入
        self.stop_record()
        # 删除数据
        record_path = pathlib.Path(self.record_path)
        if record_path.parent.exists() and record_path.parent.is_dir():
            shutil.rmtree(record_path.parent)
            print(f"Folder {record_path.parent} has been removed.")
            return True
        else:
            if not_exist_ok:
                return True
            print(f"Folder {record_path.parent} does not exist or is not a directory.")
            return False

    def stop_record(self):
        self.if_record_start_lock.acquire()
        self.if_record_start = False
        self.if_record_start_lock.release()
        self.destroy_recorder()

    def destroy_recorder(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None