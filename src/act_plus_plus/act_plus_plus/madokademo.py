import h5py
import os
import numpy as np

# 定义文件夹路径
data_dir = "/home/haoyue/code/jk_imitation_learning/src/act_plus_plus/act_plus_plus/data/sim_transfer_cube_scripted"

# 获取所有 HDF5 文件路径
file_paths = [os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.endswith(".hdf5")]

# Step 1: 找到所有文件中的最小 T
min_T = float("inf")
for file_path in file_paths:
    with h5py.File(file_path, "r") as f:
        T = len(f["action"])  # 假设 action 的长度代表 T
        min_T = min(min_T, T)

print(f"最小时间步长 (T): {min_T}")

# Step 2: 处理每个文件
for file_path in file_paths:
    print(f"处理文件: {file_path}")
    with h5py.File(file_path, "r+") as f:
        # 截断数据到最小 T
        for key in ["observations/qpos", "observations/qvel", "action"]:
            if key in f:
                truncated_data = f[key][:min_T]  # 截取前 min_T 个时间步
                del f[key]  # 删除原始数据集
                f.create_dataset(key, data=truncated_data)  # 创建新的截断数据集

        # 修改图像数据
        if "observations/images" in f:
            images_group = f["observations/images"]
            cam_names = list(images_group.keys())  # 获取当前相机名称列表

            if len(cam_names) == 2:
                # 获取两个相机的图像数据
                cam1_data = images_group[cam_names[0]][:min_T]
                cam2_data = images_group[cam_names[1]][:min_T]

                # 删除原始相机数据
                del images_group[cam_names[0]]
                del images_group[cam_names[1]]

                # 创建新的相机组并写入数据
                images_group.create_dataset("top", data=cam1_data)
                images_group.create_dataset("left_wrist", data=cam2_data)
                images_group.create_dataset("right_wrist", data=cam2_data)  # 复制第二个相机作为第三个相机

print("处理完成！")