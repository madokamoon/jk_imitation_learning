# jk_imitation-learning

敬科jk5机械臂 模仿学习
[GitHub - madokamoon/jk\_imitation\_learning](https://github.com/madokamoon/jk_imitation_learning)

# 环境配置
## 全局环境
```shell
ubuntu 22.04
python 3.10
ros2 humble
cuda 12.2
```
## ACT环境
参考：[GitHub - MarkFzp/act-plus-plus:](https://github.com/MarkFzp/act-plus-plus.git)
```shell
conda create -n jk python=3.10
conda activate jk
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
## 安装 librealsense 2.54.2

安装包：[librealsense/archive/refs/tags/v2.54.2.tar.gz](https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.54.2.tar.gz)
安装说明，跳过内核修补：[IntelRealSense/librealsense · GitHub](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

**测试**
```shell
realsense-viewer
```

## 安装 realsense-ros 4.54.1

安装包：[realsense-ros/archive/refs/tags/4.54.1.tar.gz](https://github.com/IntelRealSense/realsense-ros/archive/refs/tags/4.54.1.tar.gz)

```shell
#创建一个 ROS2 工作空间
mkdir -p ~/realsence_warpper/src
cd ~/realsence_warpper/src/  #解压到此
cd ~/realsence_warpper
# 安装依赖项
sudo apt-get install python3-rosdep -y
sudo rosdep init 
rosdep update 
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
colcon build
# 加入环境
source ~/code/realsence_warpper/install/local_setup.bash
```
**测试**
```
ros2 launch realsense2_camera rs_launch.py 
```

## 安装 json-3.11.3

安装包： https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz

```bash
wget https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
tar -xf v3.11.3.tar.gz 
cd json-3.11.3
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j12
sudo make install
```
 
## 安装其他包

```shell
# 系统python环境中
pip3 install lark
pip3 install==3.3.4
pip3 install numpy==1.24.0
pip3 install minimalmodbus
pip3 install pyserial
```

# 数据采集

```shell
#系统python环境编译
conda deactivate
rm -rf build/ install/ log/
colcon build
source install/setup.bash

#一键启动脚本
# ros2 launch teleop_twist_joy teleop-launch.py
bash scripts/startrobot.sh start/stop
bash scripts/startcamera.sh start/stop

# 数据采集
# 默认配置文件：config_data_sampler_default.yaml
ros2 run data_sampler data_sampler 

# 数据可视化 参数为epoch
# 默认配置文件：config_visualize_epoch.yaml
python scripts/visualize_epoch.py 0

```

操作方式：
- Y 开始收集
- B 执行预定义动作
- upload 结束收集
- 左摇杆 水平移动
- 右摇杆 上下移动与旋转

## 训练

```bash
python src/act_plus_plus/act_plus_plus/imitate_episodes.py --task_name demo --ckpt_dir training --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 --lr 1e-5 --seed 0 --num_steps 2000

```


