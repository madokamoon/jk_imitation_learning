import json
import yaml
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pathlib
import matplotlib
import argparse
from PIL import Image
import numpy as np
import time
from collections import deque

root_path = pathlib.Path(__file__).parent.parent

# 添加命令行参数解析
parser = argparse.ArgumentParser()
parser.add_argument("subfolder", type=str, help="epoch子文件夹名称")
args = parser.parse_args()

# 先读取配置文件
config_file = pathlib.Path(root_path, 'config/config_visualize_epoch.yaml')

# 读取 config.yaml
with open(config_file, 'r') as f:
    config = yaml.safe_load(f)

# 从配置文件获取需要显示的维度、数据路径和图像
show_data = config.get('show_data')
datapath = config.get('datapath')
show_image = config.get('show_image')

# 正确顺序：先定义data_folder，再定义json_file
data_folder = pathlib.Path(root_path, datapath, args.subfolder)
json_file = pathlib.Path(data_folder, 'state.json')

# 从配置文件读取维度索引，而不是使用硬编码值
show_dims = [
    show_data.get('robot_state'), 
    show_data.get('robot_action'), 
    show_data.get('robot_vel_command') 
]

print("使用的维度索引:")
print(f"robot_state: {show_dims[0]}")
print(f"robot_action: {show_dims[1]}")
print(f"robot_vel_command: {show_dims[2]}")

print("开始加载数据...")
start_time = time.time()

# 读取数据
with open(json_file, 'r') as f:
    data = json.load(f)

# 按编号排序
keys = sorted(data.keys(), key=lambda x: int(x))

# 提取多维数据
def extract(data, key, dims):
    arr = []
    for k in keys:
        if key in data[k] and len(data[k][key]) > 0:
            arr.append([data[k][key][d] for d in dims])
        else:
            arr.append([None for _ in dims])
    return arr

# 使用正确的字段名称
robot_state = extract(data, 'robot_state', show_dims[0])
robot_action = extract(data, 'robot_action', show_dims[1])
robot_vel_command = extract(data, 'robot_vel_command', show_dims[2])

# 提取夹爪信息
grasp_state = []
grasp_action = []
for k in keys:
    if 'grasp_state' in data[k] and len(data[k]['grasp_state']) > 0:
        grasp_state.append(data[k]['grasp_state'][0])
    else:
        grasp_state.append(None)
    
    if 'grasp_action' in data[k] and len(data[k]['grasp_action']) > 0:
        grasp_action.append(data[k]['grasp_action'][0])
    else:
        grasp_action.append(None)

# 检查图像文件夹是否存在
camera_folders = []
for camera in show_image[:3]:  # 最多取前三个摄像头
    camera_folder = data_folder / camera
    if camera_folder.exists():
        camera_folders.append(camera_folder)
    else:
        print(f"警告：摄像头文件夹 {camera_folder} 不存在")
        camera_folders.append(None)

# 填充到3个摄像头
while len(camera_folders) < 3:
    camera_folders.append(None)

# 修改为4行2列的图像布局
fig, axs = plt.subplots(4, 2, figsize=(20, 15))
titles_left = ['robot_state', 'robot_action', 'robot_vel_command', 'grasp_info']
datas = [robot_state, robot_action, robot_vel_command]
lines = []

# 定义颜色列表，足够多即可
colors = list(matplotlib.colormaps['tab10'].colors) + list(matplotlib.colormaps['tab20'].colors)

# 绘制左侧四个子图（数据图表）
for i in range(4):
    axs[i, 0].set_title(titles_left[i])
    axs[i, 0].set_xlim(0, len(keys))
    
    if i < 3:  # 前三个子图（机械臂数据）
        # 计算所有维度的全局最小最大值，忽略None
        flat = [v for arr in datas[i] for v in arr if v is not None]
        minv = min(flat) if flat else -1
        maxv = max(flat) if flat else 1
        axs[i, 0].set_ylim(minv - 0.01, maxv + 0.01)
        # 为每个维度画一条线，颜色按维度编号统一
        line_list = []
        for j, d in enumerate(show_dims[i]):
            color = colors[d % len(colors)]
            line, = axs[i, 0].plot([], [], lw=2, label=f'dim_{d}', color=color)
            line_list.append(line)
        axs[i, 0].legend()
        lines.append(line_list)
    else:  # 第四个子图（夹爪信息）
        # 计算夹爪信息的全局最小最大值
        grasp_flat = [v for v in grasp_state if v is not None] + [v for v in grasp_action if v is not None]
        grasp_minv = min(grasp_flat) if grasp_flat else -1
        grasp_maxv = max(grasp_flat) if grasp_flat else 1
        axs[i, 0].set_ylim(grasp_minv - 0.01, grasp_maxv + 0.01)
        # 添加夹爪状态和动作的线
        line_state, = axs[i, 0].plot([], [], lw=2, label='grasp_state', color=colors[0])
        line_action, = axs[i, 0].plot([], [], lw=2, label='grasp_action', color=colors[1])
        grasp_lines = [line_state, line_action]
        axs[i, 0].legend()
        lines.append(grasp_lines)

# 配置右侧前三个子图（摄像头图像）
for i in range(3):
    if i < len(show_image):
        axs[i, 1].set_title(show_image[i])
    axs[i, 1].axis('off')  # 关闭坐标轴

# 配置右侧第四个子图（信息显示）
axs[3, 1].axis('off')  # 关闭坐标轴
info_text = axs[3, 1].text(0.5, 0.5, '', ha='center', va='center', fontsize=12)

# 图像对象列表（用于更新）
img_objects = [None, None, None]

time_diffs = deque(maxlen=4)
frame_rates = deque(maxlen=4)
last_timestamp = None

# 在全局变量区域添加新的变量（在time_diffs定义之前）
first_timestamp = None
max_frame_rate = 0
min_frame_rate = float('inf')
total_frame_rate = 0
frame_count = 0
max_frame_rate_idx = 0  # 记录最大帧率对应的帧编号
min_frame_rate_idx = 0  # 记录最小帧率对应的帧编号

def update(frame):
    if frame < len(keys):
        key = keys[frame]
        timestamp = data[key].get("timestamp", None)
        
        # 更新左侧前三个图（机械臂数据）
        for i in range(3):
            for j, d in enumerate(show_dims[i]):
                y = [arr[j] if len(arr) > j and arr[j] is not None else float('nan') for arr in datas[i][:frame]]
                lines[i][j].set_data(range(frame), y)
        
        # 更新左侧第四个图（夹爪信息）
        state_y = [v if v is not None else float('nan') for v in grasp_state[:frame]]
        action_y = [v if v is not None else float('nan') for v in grasp_action[:frame]]
        lines[3][0].set_data(range(frame), state_y)
        lines[3][1].set_data(range(frame), action_y)
        
        # 更新右侧前三个图（摄像头图像）
        for i in range(3):
            if camera_folders[i] is not None:
                img_path = camera_folders[i] / f"{key}.png"
                if img_path.exists():
                    # 清除之前的图像
                    # if img_objects[i] is not None:
                    #     img_objects[i].remove()
                    
                    # 读取并显示新图像
                    img = plt.imread(img_path)
                    img_objects[i] = axs[i, 1].imshow(img)
                    axs[i, 1].set_title(f"{show_image[i]} - Frame {key}")
        
        # 更新右侧第四个图（信息显示）
        global last_timestamp, first_timestamp, max_frame_rate, min_frame_rate, total_frame_rate, frame_count, max_frame_rate_idx, min_frame_rate_idx
        
        # 初始化第一帧时间戳
        if first_timestamp is None and timestamp is not None:
            first_timestamp = timestamp
        
        if timestamp is not None and last_timestamp is not None:
            time_diff = timestamp - last_timestamp
            frame_rate = 1.0 / time_diff if time_diff > 0 else 0
            
            # 更新统计信息
            if frame_rate > 0:
                if frame_rate > max_frame_rate:
                    max_frame_rate = frame_rate
                    max_frame_rate_idx = frame
                if frame_rate < min_frame_rate:
                    min_frame_rate = frame_rate
                    min_frame_rate_idx = frame
                total_frame_rate += frame_rate
                frame_count += 1
            
            time_diffs.append(round(time_diff, 4))
            frame_rates.append(round(frame_rate, 2))
        
        # 构建显示字符串
        current_time = timestamp - first_timestamp if first_timestamp is not None else 0
        avg_frame_rate = total_frame_rate / frame_count if frame_count > 0 else 0
        
        info_str = f"Frame: {key}\nTimestamp: {timestamp}\n"
        info_str += f"Current Time: {current_time:.2f}s\n"
        info_str += f"Max FPS: {max_frame_rate:.2f} (Frame {max_frame_rate_idx})\n"
        info_str += f"Min FPS: {min_frame_rate:.2f} (Frame {min_frame_rate_idx})\n"
        info_str += f"Avg FPS: {avg_frame_rate:.2f} (Total {frame_count} frames)\n\n"
        
        # 使用enumerate获取实际的帧序号，存储在time_diffs和frame_rates中的最近4个帧的统计信息
        frame_history = list(range(max(0, frame-len(time_diffs)), frame))
        for frame_idx, (td, fr) in zip(frame_history, zip(time_diffs, frame_rates)):
            info_str += f"{frame_idx:5d} | {td:7.4f} | {fr:7.2f}\n"
        
        info_text.set_text(info_str)
        last_timestamp = timestamp
        
        print(f"Frame {frame}: key = {key}, timestamp = {timestamp}")
    
    return [l for sub in lines for l in sub] + [obj for obj in img_objects if obj is not None] + [info_text]

ani = FuncAnimation(fig, update, frames=len(keys)+1, interval=100, blit=True, repeat=False)
plt.tight_layout()
plt.show()