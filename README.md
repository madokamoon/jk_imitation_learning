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

## 安装librealsense
安装包
[v2.54.2.tar.gz](https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.54.2.tar.gz)
安装说明
[librealsense/doc/installation.md at master · IntelRealSense/librealsense · GitHub](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

**测试**
```shell
realsense-viewer
```
## 安装realsense-ros-wrapper

下载4.54.1版本ros2 wrapper：[https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/4.54.1.tar.gz](https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/4.54.1.tar.gz)
```shell
#创建一个 ROS2 工作空间
mkdir -p ~/code/realsence_warpper/src
cd ~/code/realsence_warpper/src/  #解压到此
cd ~/code/realsence_warpper
#安装依赖项
sudo apt-get install python3-rosdep -y
sudo rosdep init 
rosdep update 
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
pip install empy 
colcon build
# 手动 source
. install/local_setup.bash
# 自动 加入到 ~/.bashrc
source ~/code/realsence_warpper/install/local_setup.bash
```
**测试**
```
ros2 run realsense2_camera realsense2_camera_node
```


# 使用