# jk_imitation-learning
敬科jk5机械臂 模仿学习

# 环境配置
## 全局

```shell
ubuntu 22.04
python 3.10
ros2 humble
cuda 12.2
```
## 安装ACT
```shell
conda create -n jk python=3.10
conda activate jk
# 加入act
cd imitation_learning/src
ros2 pkg create act_plus_plus --build-type ament_python
cd act_plus_plus
git clone https://github.com/MarkFzp/act-plus-plus.git
rm -r act_plus_plus
mv act-plus-plus act_plus_plus

# 安装ACT依赖包
cd act_plus_plus/detr && pip install -e . && cd ..
pip install torchvision
pip install torch
pip install pyquaternion
pip install pyyaml
pip install rospkg
pip install pexpect
pip install mujoco==2.3.7
pip install dm_control==1.0.14
pip install opencv-python
pip install matplotlib
pip install einops
pip install packaging
pip install h5py
pip install ipython

# 安装以下内容以运行act-plus中的 Diffusion Policy 但是安装后 numpy 版本冲突 暂不安装
# git clone https://githubfast.com/ARISE-Initiative/robomimic --recurse-submodules
# git checkout r2d2
# pip install -e .
```
**测试**
```shell
python record_sim_episodes.py --task_name sim_transfer_cube_scripted --dataset_dir data/test --num_episodes 1 --onscreen_render
```

