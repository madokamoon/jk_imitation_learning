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

# @click.command()
# @click.option('--input', '-i', required=True, help='Path to checkpoint')
# @click.option('--output', '-o', required=True, help='Directory to save recording')
# @click.option('--robot_ip', '-ri', required=True, help="UR5's IP address e.g. 192.168.0.204")
# @click.option('--vis_camera_idx', default=0, type=int, help="Which RealSense camera to visualize.")
# @click.option('--init_joints', '-j', is_flag=True, default=False, help="Whether to initialize robot joint configuration in the beginning.")
# @click.option('--steps_per_inference', '-si', default=6, type=int, help="Action horizon for inference.")
# @click.option('--max_duration', '-md', default=60, help='Max duration for each epoch in seconds.')
# @click.option('--frequency', '-f', default=10, type=float, help="Control frequency in Hz.")
# @click.option('--command_latency', '-cl', default=0.01, type=float, help="Latency between receiving SapceMouse command to executing on Robot in Sec.")
def main():
    # 加载参数
    file_yaml = 'config/dp_eval.yaml'
    rf = open(file=file_yaml, mode='r', encoding='utf-8')
    crf = rf.read()
    rf.close()  # 关闭文件
    yaml_data = yaml.load(stream=crf, Loader=yaml.FullLoader)
    ckpt_path = yaml_data["ckpt_path"]
    output_dir = yaml_data["output_dir"]
    device = yaml_data["device"]
    control_frequency = yaml_data["control_frequency"]
    steps_per_inference = yaml_data["steps_per_inference"]

    # load checkpoint
    payload = torch.load(open(ckpt_path, 'rb'), pickle_module=dill)
    cfg = payload['cfg']
    print(cfg._target_)
    # target_class = cfg._target_
    # default_module_path = "install.diffusion_policy.lib.python3.10.site-packages"
    # if not target_class.startswith(default_module_path):
    #     target_class = f"{default_module_path}.{target_class}"


    cls = hydra.utils.get_class(cfg._target_)
    workspace = cls(cfg)
    workspace: BaseWorkspace
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    # hacks for method-specific setup.
    delta_action = False
    if 'diffusion' in cfg.name:
    # diffusion model
        policy: BaseImagePolicy
        policy = workspace.model
        if cfg.training.use_ema:
            policy = workspace.ema_model

        device = torch.device(device)
        policy.eval().to(device)

        # set inference params
        policy.num_inference_steps = 16  # DDIM inference iterations
        policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1

    # setup experiment
    dt = 1 / control_frequency
    obs_res = get_real_obs_resolution(cfg.task.shape_meta)
    n_obs_steps = cfg.n_obs_steps
    print("n_obs_steps: ", n_obs_steps)
    print("steps_per_inference:", steps_per_inference)
    while 1:
        pass
    # n_obs_steps = cfg.n_obs_steps
    # print("n_obs_steps: ", n_obs_steps)
    # print("steps_per_inference:", steps_per_inference)


if __name__ == '__main__':
    main()