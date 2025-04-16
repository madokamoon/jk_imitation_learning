import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from jk_robot_msgs.srv import StrFuncCommand


import json
import numpy as np
from typing import Union
import time
import copy
from enum import Enum


class Mode(Enum):
    ENDPOS  = 6
    ENDVEL  = 5

class RobotClient:
    def __init__(self, robot_init_pos: list, pos_target_d: float = 0.003):
        # 机器人初始化参数
        assert len(robot_init_pos) == 6
        self.robot_init_pos = np.array(robot_init_pos)
        self.pos_target_d = pos_target_d

        # 机械臂客户端初始化参数
        self.jk_robot_client = None

    def create_robot_client(self, node: rclpy.node.Node, server_name: str,
                            callback_group: rclpy.callback_groups.CallbackGroup):
        self.jk_robot_client = node.create_client(StrFuncCommand, server_name, callback_group=callback_group)

    # 指令相关
    def send_command(self, client: Client, func_name: str, args: str, func_return: dict):
        if client is None:
            print("client is None")
            return False

        request = StrFuncCommand.Request()
        request.func_name = func_name
        request.args = args
        while not client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                print('Interrupted while waiting for service. Exiting.')
                return False
            print('service not available, waiting again...')
        result = client.call(request)
        func_return.update(json.loads(result.func_return))
        return True

    def send_command2jk(self, func_name: str, args: str, func_return: dict) -> bool:
        return self.send_command(self.jk_robot_client, func_name, args, func_return)

    def set_init_pos(self) -> bool:
        self.set_switch_flag4(0)
        self.set_mode(Mode.ENDPOS.value)
        time.sleep(0.05)
        self.set_pos(self.robot_init_pos)
        self.set_switch_flag4(1)
        now_end_pos = copy.deepcopy(self.get_end_pos())
        robot_init_pos = copy.deepcopy(self.robot_init_pos)
        robot_init_pos[3:6] *= np.pi / 180
        now_end_pos[3:6] *= np.pi / 180
        d_pos = np.linalg.norm(now_end_pos - robot_init_pos)
        while d_pos > self.pos_target_d:
            now_end_pos = copy.deepcopy(self.get_end_pos())
            now_end_pos[3:6] *= np.pi / 180
            d_pos = np.linalg.norm(now_end_pos - robot_init_pos)
            print("d_pos: {0}, target: {1}".format(d_pos, self.pos_target_d), end='\r')
        return True

    def get_end_pos(self) -> Union[np.ndarray, None]:
        func_name = "getEndPos"
        args_to_send = {}
        func_return = {}
        success = self.send_command2jk(func_name, json.dumps(args_to_send), func_return)
        if not (success and func_return["success"]):
            return None

        return np.array(func_return['data'])

    def set_enable(self, enable: int) -> bool:
        func_name = "setEnable"
        args_to_send = {
            "enable": enable
        }
        func_return = {}
        success = self.send_command2jk(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

    def set_switch_flag4(self, flag: int) -> bool:
        func_name = "setSwitchFlag4"
        args_to_send = {
            "flag": flag
        }
        func_return = {}
        success = self.send_command2jk(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

    def set_mode(self, mode: int) -> bool:
        """
        作用：
            设置机械臂运动模式
        Args:
            mode: 机械臂运动模式，5为末端速度，6为末端位姿
        Returns:
            成功
        """
        func_name = "setMode"
        args_to_send = {
            "mode": mode
        }
        func_return = {}
        success = self.send_command2jk(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

    def set_pos(self, pos: np.ndarray) -> bool:
        func_name = "setPos"
        args_to_send = {
            "Pos": pos.tolist()
        }
        func_return = {}
        success = self.send_command2jk(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

    def set_end_vel(self, vel: np.ndarray) -> bool:
        func_name = "setEndVel"
        args_to_send = {
            "vel": vel.tolist()
        }
        func_return = {}
        success = self.send_command2jk(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

