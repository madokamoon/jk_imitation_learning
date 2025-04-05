import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from jk_robot_msgs.srv import StrFuncCommand

import json
import numpy as np
from typing import Union
import time
import pathlib
import threading
import copy
from enum import Enum

from .replay_buffer import ReplayBuffer

class Mode(Enum):
    ENDPOS  = 6
    ENDVEL  = 5

class RobotClient:
    def __init__(self, robot_init_pos: list, pos_target_d: float = 0.003, saved_mode: str = "json", prefix: str = "robot_states"):
        # 机器人初始化参数
        assert len(robot_init_pos) == 6
        self.robot_init_pos = np.array(robot_init_pos)
        self.pos_target_d = pos_target_d

        # 夹爪初始化参数
        self.gripper_client = None

        # 机械臂客户端初始化参数
        self.jk_robot_client = None

        # 数据采样
        assert saved_mode == "json" or saved_mode == "zarr"
        self.data_id = 0
        self.epoch = 0
        self.datas = {}
        self.saved_mode = saved_mode
        self.prefix = prefix
        self.record_path = ""
        self.replay_buffer = None

    def create_robot_client(self, node: rclpy.node.Node, server_name: str,
                            callback_group: rclpy.callback_groups.CallbackGroup):
        self.jk_robot_client = node.create_client(StrFuncCommand, server_name, callback_group=callback_group)

    def create_gripper_client(self, node: rclpy.node.Node, server_name: str,
                              callback_group: rclpy.callback_groups.CallbackGroup):
        self.gripper_client = node.create_client(StrFuncCommand, server_name, callback_group=callback_group)

    def start_sample(self, record_path_prefix: str):
        if self.saved_mode == "json":
            # 处理文件路径
            if not self.prefix == "":
                record_path = pathlib.Path(record_path_prefix).joinpath(self.prefix)
            else:
                record_path = pathlib.Path(record_path_prefix)
            epoch = 0
            if record_path.exists():
                epoch = len(list(record_path.glob("*")))
            record_path = record_path.joinpath(f"{str(epoch)}.json")
            record_path.parent.mkdir(parents=True, exist_ok=True)
            self.record_path = str(record_path)
            print(self.record_path)
        elif self.saved_mode == "zarr":
            if self.replay_buffer is None:
                if not self.prefix == "":
                    record_path = pathlib.Path(record_path_prefix).joinpath(self.prefix)
                else:
                    record_path = pathlib.Path(record_path_prefix)
                zarr_path = pathlib.Path(str(record_path), 'replay_buffer.zarr')
                self.replay_buffer = ReplayBuffer.create_from_path(
                            zarr_path=zarr_path, mode='a')
                for k, v in self.replay_buffer.items():
                    self.datas[k] = np.array(v).tolist()
            print("第 {0} 次运行".format(self.replay_buffer.n_episodes))
        return True

    def stop_sample(self):
        if self.saved_mode == "json":
            with open(self.record_path, 'w') as json_file:
                json.dump(self.datas, json_file, indent=4)
            self.datas.clear()
            self.data_id = 0
        elif self.saved_mode == "zarr":
            for k, v in self.datas.items():
                self.datas[k] = np.array(self.datas[k])
            self.replay_buffer.add_episode(self.datas, compressors='disk')
            self.datas.clear()
            self.epoch += 1

    def put_data(self, data: dict):
        if self.saved_mode == "json":
            self.datas[str(self.data_id)] = data
            self.data_id += 1
        elif self.saved_mode == "zarr":
            for k, v in data.items():
                if k not in self.datas.keys():
                    self.datas[k] = []
                self.datas[k].append(v)

    def delete_sampled_datas(self):
        self.datas.clear()
        self.data_id = 0

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

    def send_command2gripper(self, func_name: str, args: str, func_return: dict) -> bool:
        return self.send_command(self.gripper_client, func_name, args, func_return)

    def open_gripper(self, blocking: bool = False) -> bool:
        func_name = "open_gripper"
        args_to_send = {
            "blocking": blocking
        }
        func_return = {}
        success = self.send_command2gripper(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

    def close_gripper(self, blocking: bool = False) -> bool:
        func_name = "close_gripper"
        args_to_send = {
            "blocking": blocking
        }
        func_return = {}
        success = self.send_command2gripper(func_name, json.dumps(args_to_send), func_return)
        return success and func_return["success"]

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
            print("d_pos: {0}, target: {1}".format(d_pos, self.pos_target_d))
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

