#!/bin/bash

# 对照表
declare -A serial_to_name=(
    ["203522250889"]="cam_eye"
    ["f1370488"]="cam_right"
    ["939622074891"]="cam_left"
)

if [ "$1" == "start" ]; then
    serials=($(rs-enumerate-devices | grep "Serial Number" | grep -v "Asic" | awk '{print $NF}'))

    if [ ${#serials[@]} -eq 0 ]; then
        echo "❌ 没有检测到任何 RealSense 摄像头。"
        exit 1
    fi

    echo "✅ 检测到 ${#serials[@]} 个摄像头："
    for i in "${!serials[@]}"; do
        echo "  camera$i : ${serials[$i]}"
    done

    # 启动每个摄像头对应的 ROS 节点
    started_cameras=()
    for i in "${!serials[@]}"; do
        serial=${serials[$i]}
        camera_name="camera$i"  # 默认名称
        if [[ -n "${serial_to_name[$serial]}" ]]; then
            camera_name="${serial_to_name[$serial]}"  
        fi

        echo "🚀 启动 $camera_name (序列号: $serial)..."
        ros2 launch realsense2_camera rs_launch.py \
            camera_name:=$camera_name \
            serial_no:="_$serial" \
            enable_depth:=false \
            pointcloud.enable:=false \
            align_depth.enable:=false \
            depth_module.profile:=640x480x30 \
            rgb_camera.profile:=640x480x30 &

        sleep 1  # 给每个节点一点启动时间

        # 记录已启动的摄像头信息
        started_cameras+=("$serial:$camera_name")
    done

    echo "🎉 总计 ${#serials[@]} 个摄像头节点已启动！"
    for cam_info in "${started_cameras[@]}"; do
        echo "  已启动摄像头节点：$cam_info"
    done

elif [ "$1" == "stop" ]; then
    # 停止所有服务
    pkill -f realsense2_camera
    echo "所有摄像头已停止"
else
    echo "Usage: $0 {start|stop}"
fi