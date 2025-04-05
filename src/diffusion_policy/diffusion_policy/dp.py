import sys
sys.path.append(f'/home/starsky/anaconda3/envs/robodiff-ros2/lib/python3.10/site-packages')
import time
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import torch
import dill
import hydra
import yaml
import pathlib
import skvideo.io
from omegaconf import OmegaConf
import rclpy
import scipy.spatial.transform as st
# from diffusion_policy.real_world.real_env import RealEnv

from diffusion_policy.real_world.real_inference_util import (
    get_real_obs_resolution,
    get_real_obs_dict)
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.cv2_util import get_image_transform

OmegaConf.register_new_resolver("eval", eval, replace=True)

class DiffusionPolicy:
    def __init__(self, yaml_data):
        self.ckpt_path = yaml_data["ckpt_path"]
        self.output_dir = yaml_data["output_dir"]
        device_str = yaml_data["device"]
        self.device = torch.device(device_str)
        self.control_frequency = yaml_data["control_frequency"]
        self.steps_per_inference = yaml_data["steps_per_inference"]
        self.num_inference_steps = yaml_data["num_inference_steps"]
        self.control_frequency = yaml_data["control_frequency"]
        self.dt = 1 / self.control_frequency
        self.steps_per_inference = yaml_data["steps_per_inference"]
        self.obs_res = None
        self.n_obs_steps = None
        self.last_cameras_imgs = []
        self.last_robot_states = []
        # payload
        self.payload = None
        # cfg
        self.cfg = None
        # workspace
        self.workspace = None
        # policy
        self.policy = None

    def diffusion_policy_init(self):
        # payload
        self.payload = torch.load(open(self.ckpt_path, 'rb'), pickle_module=dill)
        # cfg
        self.cfg = self.payload['cfg']
        # workspace
        cls = hydra.utils.get_class(self.cfg._target_)
        self.workspace = cls(self.cfg)
        self.workspace: BaseWorkspace
        self.workspace.load_payload(self.payload, exclude_keys=None, include_keys=None)
        print("工作空间初始化完成")
        # policy
        self.policy: BaseImagePolicy
        self.policy = self.workspace.model
        if self.cfg.training.use_ema:
            self.policy = self.workspace.ema_model
        self.policy.eval().to(self.device)
        print("策略模型加载完成")
        # set inference params
        self.policy.num_inference_steps = self.num_inference_steps  # DDIM inference iterations
        self.policy.n_action_steps = self.policy.horizon - self.policy.n_obs_steps + 1

        # setup experiment
        self.obs_res = get_real_obs_resolution(self.cfg.task.shape_meta)
        self.n_obs_steps = self.cfg.n_obs_steps
        print("参数初始化完成")

    def get_actions(self, cameras_img: dict, robot_state: dict):
        obs_align_timestamps = np.zeros(self.n_obs_steps)
        cameras_imgs_obs = dict()
        robot_states_obs = dict()
        self.last_cameras_imgs.append(cameras_img)
        self.last_robot_states.append(robot_state)
        n_obs_steps_gap = self.n_obs_steps - len(self.last_cameras_imgs)
        if n_obs_steps_gap < 0:
            self.last_cameras_imgs.pop(0)
            self.last_robot_states.pop(0)
            for key, attr in self.cfg.task.shape_meta.obs.items():
                type = attr.get('type', 'low_dim')
                shape = attr.get('shape')
                if type == 'rgb':
                    cameras_imgs_obs[key] = self.last_cameras_imgs[0][key]
                    for i in range(self.n_obs_steps - 1):
                        cameras_imgs_obs[key] = np.concatenate((cameras_imgs_obs[key], self.last_cameras_imgs[i + 1][key]), axis=0)
                elif type == 'low_dim':
                    robot_states_obs[key] = self.last_robot_states[0][key]
                    for i in range(self.n_obs_steps - 1):
                        robot_states_obs[key] = np.concatenate((robot_states_obs[key], self.last_robot_states[i + 1][key]), axis=0)
        else: # n_obs_steps_gap >= 0 使用最开始的状态的复制
            for k, v in self.last_cameras_imgs[0].items():
                cameras_imgs_obs[k] = np.repeat(v, n_obs_steps_gap + 1, axis=0)
                for i in range(len(self.last_cameras_imgs) - 1):
                    cameras_imgs_obs[k] = np.concatenate((cameras_imgs_obs[k], self.last_cameras_imgs[i + 1][k]), axis=0)
            for k, v in self.last_robot_states[0].items():
                robot_states_obs[k] = np.repeat(v, n_obs_steps_gap + 1, axis=0)
                for i in range(len(self.last_robot_states) - 1):
                    robot_states_obs[k] = np.concatenate((robot_states_obs[k], self.last_robot_states[i + 1][k]), axis=0)

        obs_data = dict(cameras_imgs_obs)
        obs_data.update(robot_states_obs)
        obs_data['timestamp'] = obs_align_timestamps
        with torch.no_grad():
            self.policy.reset()
            obs_dict_np = get_real_obs_dict(
                env_obs=obs_data, shape_meta=self.cfg.task.shape_meta)
            obs_dict = dict_apply(obs_dict_np,
                lambda x: torch.from_numpy(x).unsqueeze(0).to(self.device))
            result = self.policy.predict_action(obs_dict)
            action = result['action'][0].detach().to('cpu').numpy()
        return action

