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
import h5py
from .replay_buffer import ReplayBuffer
import zarr

class Mode(Enum):
    ENDPOS  = 6
    ENDVEL  = 5

class RobotClient:
    def __init__(self, robot_init_pos: list, pos_target_d: float = 0.003, saved_mode: str = "json", prefix: str = "robot_states", camera_names: list = ["camera0", "camera1", "camera2"]):
        # 机器人初始化参数
        assert len(robot_init_pos) == 6
        self.robot_init_pos = np.array(robot_init_pos)
        self.pos_target_d = pos_target_d

        # 夹爪初始化参数
        self.gripper_client = None

        # 机械臂客户端初始化参数
        self.jk_robot_client = None

        # 数据采样
        assert saved_mode == "json"  or saved_mode == "hdf5" or saved_mode == "dp3"
        self.data_id = 0
        self.epoch = 0
        self.datas = {}
        self.camera_names = camera_names
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
        elif self.saved_mode == "dp3":
            if not self.prefix == "":
                record_path = pathlib.Path(record_path_prefix).joinpath(self.prefix)
            else:
                record_path = pathlib.Path(record_path_prefix)
            epoch = 0
            if record_path.exists():
                epoch = len(list(record_path.glob("*")))
            record_path = record_path.joinpath(f"episode_{str(epoch)}.zarr")  
            record_path.parent.mkdir(parents=True, exist_ok=True)
            self.record_path = str(record_path)

            # 初始化数据收集字典
            self.datas = {
                'img': [],  # RGB图像
                'state': [],  # 机器人状态
                'point_cloud': [],  # 点云数据
                'depth': [],  # 深度图像
                'action': [],  # 动作
                'episode_ends': []  # 轨迹结束标记
            }

        elif self.saved_mode == "hdf5":
            # 处理文件路径
            if not self.prefix == "":
                record_path = pathlib.Path(record_path_prefix).joinpath(self.prefix)
            else:
                record_path = pathlib.Path(record_path_prefix)
            epoch = 0
            if record_path.exists():
                epoch = len(list(record_path.glob("*")))
            record_path = record_path.joinpath(f"episode_{str(epoch)}.hdf5")  
            record_path.parent.mkdir(parents=True, exist_ok=True)
            self.record_path = str(record_path)
            # 初始化数据字典
            self.datas = {
                '/observations/qpos': [],
                '/observations/qvel': [],
                '/action': [],
            }
            for cam_name in self.camera_names:
                self.datas[f'/observations/images/{cam_name}'] = []

            print(self.record_path)
        return True

    def stop_sample(self):
        if self.saved_mode == "json":
            with open(self.record_path, 'w') as json_file:
                json.dump(self.datas, json_file, indent=4)
            self.datas.clear()
            self.data_id = 0
        elif self.saved_mode == "dp3":
            # 创建zarr根组和子组
            zarr_root = zarr.group(self.record_path)
            zarr_data = zarr_root.create_group('data')
            zarr_meta = zarr_root.create_group('meta')

            # 将列表数据转换为numpy数组
            img_arrays = np.array(self.datas['img'])
            if img_arrays.shape[1] == 3:  # 确保通道在最后
                img_arrays = np.transpose(img_arrays, (0,2,3,1))
            state_arrays = np.array(self.datas['state'])
            point_cloud_arrays = np.array(self.datas['point_cloud'])
            depth_arrays = np.array(self.datas['depth'])
            action_arrays = np.array(self.datas['action'])
            episode_ends_arrays = np.array(self.datas['episode_ends'])

            # 设置压缩器
            compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)

            # 设置分块大小
            img_chunk_size = (100, img_arrays.shape[1], img_arrays.shape[2], img_arrays.shape[3])
            state_chunk_size = (100, state_arrays.shape[1])
            point_cloud_chunk_size = (100, point_cloud_arrays.shape[1], point_cloud_arrays.shape[2])
            depth_chunk_size = (100, depth_arrays.shape[1], depth_arrays.shape[2])
            action_chunk_size = (100, action_arrays.shape[1])

            # 创建数据集
            zarr_data.create_dataset('img', data=img_arrays, chunks=img_chunk_size, 
                                    dtype='uint8', compressor=compressor)
            zarr_data.create_dataset('state', data=state_arrays, chunks=state_chunk_size, 
                                    dtype='float32', compressor=compressor)
            zarr_data.create_dataset('point_cloud', data=point_cloud_arrays, 
                                    chunks=point_cloud_chunk_size, dtype='float32', compressor=compressor)
            zarr_data.create_dataset('depth', data=depth_arrays, chunks=depth_chunk_size, 
                                    dtype='float32', compressor=compressor)
            zarr_data.create_dataset('action', data=action_arrays, chunks=action_chunk_size, 
                                    dtype='float32', compressor=compressor)
            zarr_meta.create_dataset('episode_ends', data=episode_ends_arrays, dtype='int64', 
                                    compressor=compressor)

            # 打印数据形状信息
            print(f'img shape: {img_arrays.shape}, range: [{np.min(img_arrays)}, {np.max(img_arrays)}]')
            print(f'point_cloud shape: {point_cloud_arrays.shape}, range: [{np.min(point_cloud_arrays)}, {np.max(point_cloud_arrays)}]')
            print(f'depth shape: {depth_arrays.shape}, range: [{np.min(depth_arrays)}, {np.max(depth_arrays)}]')
            print(f'state shape: {state_arrays.shape}, range: [{np.min(state_arrays)}, {np.max(state_arrays)}]')
            print(f'action shape: {action_arrays.shape}, range: [{np.min(action_arrays)}, {np.max(action_arrays)}]')
            print(f'Saved zarr file to {self.record_path}')

            # 清理数据
            self.datas.clear()
            self.epoch += 1

        elif self.saved_mode == "hdf5":
            # 根据存入数据大小设置读取缓存 ACT中图像大小为(480, 640, 3) 设置为2MB
            max_timesteps = len(self.datas['/action'])
            with h5py.File(self.record_path, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
                # 设置全局参数
                root.attrs['sim'] = False
                obs = root.create_group('observations')
                image = obs.create_group('images')
                for cam_name in self.camera_names:
                    _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',chunks=(1, 480, 640, 3), )
                qpos = obs.create_dataset('qpos', (max_timesteps, 7))
                qvel = obs.create_dataset('qvel', (max_timesteps, 7))
                action = root.create_dataset('action', (max_timesteps, 7))
                # 存入数据
                for name, array in self.datas.items():
                    root[name][...] = array
            self.datas.clear()



    def put_data(self, data: dict):
        if self.saved_mode == "json":
            self.datas[str(self.data_id)] = copy.deepcopy(data)
            self.data_id += 1

        elif self.saved_mode == "hdf5":
            self.datas['/observations/qpos'].append(data["robot_eef_pose"])
            end_vel_command_expend_to_seven = data['end_vel_command'] + [data['action'][6]] # 爪子速度定义为命令吗，act中爪子不是二变量
            self.datas['/observations/qvel'].append(end_vel_command_expend_to_seven)
            self.datas['/action'].append(data['action'])
            for cam_name in self.camera_names:
                self.datas[f'/observations/images/{cam_name}'].append(data[cam_name])
        elif self.saved_mode == "dp3":
            # 存储RGB图像
            self.datas['img'].append(data["cam_eye"+"color"])
            # 存储机器人状态
            self.datas['state'].append(data["robot_eef_pose"])
            # 存储点云数据
            self.datas['point_cloud'].append(data["cam_eye"+"point"])
            # 存储深度图像
            self.datas['depth'].append(data["cam_eye"+"depth"])
            # 存储动作
            self.datas['action'].append(data["action"])
            # 存储轨迹结束标记 (如果没有则默认为0)
            self.datas['episode_ends'].append(data.get("episode_ends", 0))


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

