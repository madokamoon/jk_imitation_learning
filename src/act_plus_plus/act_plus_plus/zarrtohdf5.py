import zarr
import imagecodecs
import matplotlib.pyplot as plt

root = zarr.open('/home/haoyue/code/jk_imitation_learning/data/redblock.zarr', mode='r')
data = root['data']
meta = root['meta']

# print("meta",meta)
# 假设有 'episode_ends' 这个键，可以这样读取

episode_ends = meta['episode_ends'][:]
index = 0

actions = data['action'][0:episode_ends[index]]  # (N, 7)
eef_pose = data['robot_eef_pose'][0:episode_ends[index]]  # (N, 6)


# 绘制 actions
plt.figure(figsize=(12, 6))
for i in range(actions.shape[1]):
    plt.plot(actions[:, i], label=f'action_{i}')
plt.title('Actions over Time')
plt.xlabel('Time Step')
plt.ylabel('Action Value')
plt.legend()
plt.tight_layout()
plt.savefig("Action.png")

# 绘制 eef_pose
plt.figure(figsize=(12, 6))
for i in range(eef_pose.shape[1]):
    plt.plot(eef_pose[:, i], label=f'eef_pose_{i}')
plt.title('EEF Pose over Time')
plt.xlabel('Time Step')
plt.ylabel('EEF Pose Value')
plt.legend()
plt.tight_layout()
plt.savefig("EEF.png")