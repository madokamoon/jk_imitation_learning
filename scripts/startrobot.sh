#!/bin/bash

if [ "$1" == "start" ]; then
    # 启动所有服务
    source install/setup.bash
    ros2 run jk_robot jk_robot_server &
    echo "机械臂启动"
    sleep 2
    sudo chmod 777 /dev/ttyUSB0
    sleep 2
    # ros2 launch teleop_twist_joy teleop-launch.py &
    # echo "手柄启动"
elif [ "$1" == "stop" ]; then
    # 停止所有服务
    pkill -f "jk_robot_server"
    pkill -f "robotiq_2F85_server"
    pkill -f "joy_node"
    pkill -f "teleop_node"
    echo "所有服务已停止"
else
    echo "Usage: $0 {start|stop}"
fi