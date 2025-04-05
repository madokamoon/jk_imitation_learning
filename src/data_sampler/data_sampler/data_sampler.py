import pathlib

import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.action import ActionServer
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from jk_robot_msgs.action import StrAction

import json
import time
import cv2
import threading
import copy
import numpy as np
from scipy.spatial.transform import Rotation
import time, datetime
import os
import yaml

from data_sampler.robot_client import RobotClient, Mode
from data_sampler.xbox_controller import XBOXController
from data_sampler.camera_subscriber import CameraSubscriber


class DataSampler(Node):
    def __init__(self, node_name:str):
        super().__init__(node_name)
        # 读取配置文件
        file_yaml = 'config/config_data_sampler.yaml'
        rf = open(file=file_yaml, mode='r', encoding='utf-8')
        crf = rf.read()
        rf.close()  # 关闭文件
        yaml_data = yaml.load(stream=crf, Loader=yaml.FullLoader)
        # 采样设置
        self.sample_frequency = yaml_data["sample_frequency"]
        self.img_show = yaml_data["image_show"]
        self.go_on_path = yaml_data["go_on_path"]
        self.record_prefix_path = yaml_data["record_prefix_path"]
        self.elapsed_time_show = yaml_data["elapsed_time_show"]
        self.robot_init_pos = np.array(yaml_data["robot_init_pos"])
        self.robot_end_pos = np.array(yaml_data["robot_end_pos"])
        self.end_delay = yaml_data["end_delay"]
        # 设置回调组
        self.sensors_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.actions_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.hardware_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        # 动作服务初始化
        self.action_server = ActionServer(
            self,
            StrAction,
            'robot_policy_action',
            self.robot_policy_callback)
        self.get_logger().info("Policy Action 初始化完成")
        # 机器人客户端初始化
        robot_client_config = yaml_data["robot_client"]
        self.robot_client = RobotClient(**robot_client_config)
        self.robot_client.create_robot_client(self, "jk_robot_server", self.hardware_callback_group)
        self.robot_client.create_gripper_client(self, "robotiq_2f85_server", self.hardware_callback_group)
        self.get_logger().info("机器人客户端 初始化完成")
        # 相机初始化
        self.camera_subscirbers = []
        cameras_subscriber_config: dict
        cameras_subscriber_config = yaml_data["cameras_subscriber"]
        for k, v in cameras_subscriber_config.items():
            v.update({"camera_name": k})
            self.camera_subscirbers.append(CameraSubscriber(**v))
            self.camera_subscirbers[-1].create_rgb_subscriber(self, "/" + k + "/color/image_raw", 5,self.sensors_callback_group)
            self.get_logger().info(f"""{k} 初始化完成""")
        # XBOX初始化
        xbox_controller_config = yaml_data["xbox_controller"]
        self.xbox_controller = XBOXController(**xbox_controller_config)
        self.xbox_controller.create_joy_subscriber(self, "/joy", 5, self.hardware_callback_group)
        self.xbox_controller.create_control_action_client(self, 'robot_policy_action', self.actions_callback_group)
        self.get_logger().info("XBOX 初始化完成")

    def robot_policy_callback(self, goal_handle) -> StrAction.Result:
        # 初始化运行周期
        dt = 1 / self.sample_frequency
        last_command = None
        # 初始化录制路径（相机、机器人状态都适配）
        middle_path = "sample"
        now_time = datetime.datetime.now()
        str_time = now_time.strftime("%Y-%m-%d-%H-%M-%S")
        record_path = pathlib.Path(self.record_prefix_path)
        if self.go_on_path == "":
            record_path = record_path.joinpath(middle_path, str_time)
            if record_path.exists():
                self.get_logger().error("给定路径已存在： %s" % str(record_path))
                goal_handle.failed()
                return StrAction.Result(result=goal_handle.request.goal)
            self.go_on_path = str_time
        else:
            record_path = record_path.joinpath(middle_path, self.go_on_path)
        # 机器人操作
        print("机器人复原...")
        self.robot_client.set_init_pos()
        self.robot_client.set_mode(Mode.ENDVEL.value)
        time.sleep(0.05)
        self.robot_client.set_switch_flag4(1)
        time.sleep(0.05)
        self.robot_client.open_gripper()
        print("完成")
        # 初始化机器人状态数据录制
        self.robot_client.start_sample(str(record_path))
        # 初始化相机录制
        for camera_subscirber in self.camera_subscirbers:
            camera_subscirber.start_record(str(record_path))
        while 1:
            data = {}
            start_time = time.time()
            # 获取状态（机器人状态、图片）、按钮信息
            pressed_buttons_id, control_command = self.xbox_controller.get_all_infos()
            # X键退出动作服务并删除录制数据
            if self.xbox_controller.X_ID in pressed_buttons_id:
                print("终止动作，删除已录制数据...")
                for camera_subscirber in self.camera_subscirbers:
                    camera_subscirber.delete_record_data(not_exist_ok=True)
                self.robot_client.delete_sampled_datas()
                print("完成")
                # 机器人暂停
                self.robot_client.set_switch_flag4(0)
                break
            # 上传键执行预定义操作并录制，最后上传录制
            if self.xbox_controller.UPLOAD_ID in pressed_buttons_id:
                # 到达指定位置并录制
                data = {}
                data["timestamp"] = time.time()
                now_pos = np.ones(7)
                now_pos[0:6] = copy.deepcopy(self.robot_client.get_end_pos())
                now_pos[3:6] *= np.pi / 180
                data["robot_eef_pose"] = now_pos.tolist()
                data["end_vel_command"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                # 机械臂、夹爪到指定位置set_pos,当前状态获取，下一个状态是当前的动作，在指定误差内进入数据上传阶段
                self.robot_client.set_switch_flag4(0)
                self.robot_client.set_mode(Mode.ENDPOS.value)
                time.sleep(0.025)
                self.robot_client.set_switch_flag4(1)
                self.robot_client.set_pos(self.robot_end_pos)
                self.robot_client.close_gripper()
                target_pos = np.array(self.robot_end_pos)
                target_pos[3:6] *= np.pi / 180
                while 1:
                    start_time = time.time()
                    now_pos[0:6] = copy.deepcopy(self.robot_client.get_end_pos())
                    now_pos[3:6] *= np.pi / 180
                    data["action"] = now_pos.tolist()
                    self.robot_client.put_data(data)
                    d_pos = np.linalg.norm(now_pos[0:6] - np.array(target_pos))
                    print("d_pos: {0}, target: {1}".format(d_pos, 0.003))
                    if self.img_show:
                        for camera_subscirber in self.camera_subscirbers:
                            img = camera_subscirber.get_img()
                            if not (img is None):
                                cv2.imshow(str(camera_subscirber.camera_id), img)
                        cv2.waitKey(1)
                    if d_pos < 0.003:
                        break
                    else:
                        data["timestamp"] = time.time()
                        data["robot_eef_pose"] = now_pos.tolist()
                        # 等 1/sample_frequency s
                        while (time.time() - start_time) < dt:
                            pass

                self.robot_client.open_gripper()
                now_pos[6] = 0
                for i in range(self.end_delay):
                    start_time = time.time()
                    # 等 1/sample_frequency s
                    while (time.time() - start_time) < dt:
                        pass
                    data["timestamp"] = time.time()
                    data["robot_eef_pose"] = now_pos.tolist()
                    data["action"] = now_pos.tolist()
                    self.robot_client.put_data(data)

                # 数据操作
                print("上传采样...")
                for camera_subscirber in self.camera_subscirbers:
                    camera_subscirber.stop_record()
                self.robot_client.stop_sample()
                print("完成")
                break
            data["timestamp"] = time.time()
            end_pos = copy.deepcopy(self.robot_client.get_end_pos())
            end_pos[3:6] *= np.pi / 180  # 单位：° -> rad
            action = np.zeros(7)
            if not (control_command is None):
                # self.robot_client.set_end_vel(control_command[0:6].tolist())
                self.robot_client.set_end_vel(control_command[0:6])
                latent_time = time.time() - start_time
                self.robot_client.close_gripper() if control_command[6] == 1 \
                    else self.robot_client.open_gripper()
                control_command[0:3] /= 100  # 单位：cm/s -> m/s
                control_command[3:6] *= np.pi / 180  # 单位：°/s -> rad/s
                action[0:6] = control_command[0:6] * (dt - latent_time) + end_pos
                if last_command is not None:
                    action[0:6] += last_command[0:6] * latent_time
                last_command = copy.deepcopy(control_command)
                action[6] = control_command[6]

                # print("now: {}".format(end_pos.tolist()))
                # print("command_rec: {0}".format(control_command.tolist()))
                # print("command_after: {0}".format(action.tolist()))


                # robot_eef_pose-7 action-7 end_vel_command-6
                data["robot_eef_pose"] = end_pos.tolist() + [action[6]]  
                data["action"] = action.tolist()
                data["end_vel_command"] = control_command[0:6].tolist()

                self.robot_client.put_data(data)

            # 处理图片显示
            if self.img_show:
                for camera_subscirber in self.camera_subscirbers:
                    img = camera_subscirber.get_img()
                    if not (img is None):
                        cv2.imshow(str(camera_subscirber.camera_id), img)
                cv2.waitKey(1)

            # 处理用时显示
            if self.elapsed_time_show:
                print("用时: {0}s".format(time.time() - start_time))
            # 控制周期
            while (time.time() - start_time) < dt:
                pass
            # self.robot_client.if_thread_close = False
        goal_handle.succeed()
        return StrAction.Result(result=goal_handle.request.goal)

    def destroy_node(self):
        for camera_subscirber in self.camera_subscirbers:
            camera_subscirber.destroy_recorder()
        super().destroy_node()

def main():
    rclpy.init()

    robot_policy = DataSampler("robot_policy_node")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot_policy)

    executor.spin()

    robot_policy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

