import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from jk_robot_msgs.action import StrAction

import threading
import numpy as np
import copy
from math import *
import time

class XBOXController:
    def __init__(self, sample_frequency:float = 1):
        self.sample_frequency = sample_frequency
        self.control_action_client = None
        self.control_action_future: rclpy.task.Future = None
        # buttons
        self.if_press = np.zeros(12)
        self.A_ID = 0 # Human模式 控制夹爪
        self.B_ID = 1 # 切模式 Human、DP、Restart
        self.X_ID = 2 # RobotPolicyAction执行时退出动作，否则退出程序
        self.Y_ID = 3 # 请求RobotPolicyAction服务
        self.LB_BUTTON_ID = 4  # 按下进入旋转遥控模式
        self.UPLOAD_ID = 11
        # axes 请将人与机械臂Y轴保持同向进行操作
        self.LEFT_HORIZONAL_ID = 0 # 左1.0 右-1.0 左右 | Y轴旋转
        self.LEFT_VERTICAL_ID = 1 # 上1.0 下-1.0 前后 | -X轴旋转
        self.RIGHT_HORIZONAL_ID = 3 # 左1.0 右-1.0 Z轴旋转
        self.RIGHT_VERTICAL_ID = 4 # 上1.0 下-1.0 上下
        self.RB_BUTTON_ID = 5 # 未定义
        self.MAX_END_IINEAR_VEL = 10.0 # cm / s
        self.MAX_END_ANGLE_VEL = 10.0 # ° / s
        # 数据
        self.button_lock = threading.Lock()
        self.multi_press_finish_id = None
        self.control_command = None

    def create_joy_subscriber(self, node:rclpy.node.Node, joy_topic:str, qos_profile: int, callback_group:CallbackGroup) -> None:
        node.create_subscription(Joy, joy_topic, self.joy_callback, qos_profile, callback_group=callback_group)

    def create_control_action_client(self, node:rclpy.node.Node, action_name: str, callback_group:CallbackGroup) -> None:
        self.control_action_client = ActionClient(node, StrAction, action_name, callback_group=callback_group)

    def get_pressed_buttons_id(self) -> list:
        pressed_buttons_id = []
        self.button_lock.acquire()
        multi_press_finish_id = self.multi_press_finish_id
        self.button_lock.release()
        if multi_press_finish_id is not None:
            for press_finish_id in multi_press_finish_id:
                pressed_buttons_id.append(press_finish_id)
        return pressed_buttons_id

    def get_all_infos(self) -> tuple:
        pressed_buttons_id = []
        self.button_lock.acquire()
        multi_press_finish_id = self.multi_press_finish_id
        control_command = self.control_command
        self.button_lock.release()
        if multi_press_finish_id is not None:
            for press_finish_id in multi_press_finish_id:
                pressed_buttons_id.append(press_finish_id)
        return pressed_buttons_id, control_command

    def joy_callback(self, msg: Joy):
        # 处理buttons
        buttons = np.array(msg.buttons.tolist())
        multi_press = np.where(buttons == 1)
        multi_un_press = np.where(buttons == 0)
        last_if_press = copy.deepcopy(self.if_press)
        self.if_press[multi_press] = 1
        self.if_press[multi_un_press] = 0
        multi_press_finish_id = np.where(abs(self.if_press - 1) * last_if_press == 1)
        # 处理axes
        control_command = self.axes_handle(msg)
        # policy action请求
        if self.Y_ID in multi_press_finish_id:
            if self.control_action_client is None:
                print("Init action first.")
            elif self.control_action_future is None or self.control_action_future.done():
                print("等待动作服务端接收...")
                self.control_action_client: ActionClient
                future: rclpy.task.Future
                future = self.control_action_client.send_goal_async(StrAction.Goal(goal="diffusion_policy")) # 可以在这里加反馈回调
                future.add_done_callback(self.goal_response_callback)
                while not future.done():
                    time.sleep(0.01)

        self.button_lock.acquire()
        self.multi_press_finish_id = multi_press_finish_id
        self.control_command = control_command
        self.button_lock.release()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
            return

        print('Goal accepted :)')

        self.control_action_future = goal_handle.get_result_async()
        self.control_action_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result: StrAction.Result
        result = future.result().result
        print('Action has completed: {0}'.format(result.result))


    def axes_handle(self, msg: Joy) -> np.ndarray:
        buttons = np.array(msg.buttons.tolist())
        if buttons[self.LB_BUTTON_ID] == 1:
            rot_x = 0.0 if fabs(msg.axes[self.LEFT_VERTICAL_ID]) < 0.001 else msg.axes[self.LEFT_VERTICAL_ID]
            rot_y = 0.0 if fabs(msg.axes[self.LEFT_HORIZONAL_ID]) < 0.001 else msg.axes[self.LEFT_HORIZONAL_ID]
            vel_x = 0.0
            vel_y = 0.0
        else:
            vel_x = 0.0 if fabs(msg.axes[self.LEFT_HORIZONAL_ID]) < 0.001 else -msg.axes[self.LEFT_HORIZONAL_ID]
            vel_y = 0.0 if fabs(msg.axes[self.LEFT_VERTICAL_ID]) < 0.001 else msg.axes[self.LEFT_VERTICAL_ID]
            rot_x = 0.0
            rot_y = 0.0

        vel_z = 0.0 if fabs(msg.axes[self.RIGHT_VERTICAL_ID]) < 0.001 else msg.axes[self.RIGHT_VERTICAL_ID]
        rot_z = 0.0 if fabs(msg.axes[self.RIGHT_HORIZONAL_ID]) < 0.001 else -msg.axes[self.RIGHT_HORIZONAL_ID]

        control_command = np.array([
            vel_x * self.MAX_END_IINEAR_VEL,
            vel_y * self.MAX_END_IINEAR_VEL,
            vel_z * self.MAX_END_IINEAR_VEL,
            rot_x * self.MAX_END_ANGLE_VEL,
            rot_y * self.MAX_END_ANGLE_VEL,
            rot_z * self.MAX_END_ANGLE_VEL,
            buttons[self.A_ID]
        ])
        return control_command
