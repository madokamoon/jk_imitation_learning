#!/bin/bash

# 获取所有主序列号（排除 Asic），每个前面加 "_"
serials=($(rs-enumerate-devices | grep "Serial Number" | grep -v "Asic" | awk '{print "_"$NF}'))

if [ ${#serials[@]} -eq 0 ]; then
    echo "❌ 没有检测到任何 RealSense 摄像头。"
    exit 1
fi

echo "✅ 检测到 ${#serials[@]} 个摄像头："
for i in "${!serials[@]}"; do
    echo "  camera$i : ${serials[$i]}"
done

# 启动每个摄像头对应的 ROS 节点
for i in "${!serials[@]}"; do
    echo "🚀 启动 camera$i..."
    ros2 launch realsense2_camera rs_launch.py \
        camera_name:=camera$i \
        serial_no:=${serials[$i]} \
        enable_depth:= false \
        rgb_camera.profile:=640x480x30 &

    sleep 1  # 给每个节点一点启动时间
done

echo "🎉 总计 ${#serials[@]} 个摄像头节点已启动！"
