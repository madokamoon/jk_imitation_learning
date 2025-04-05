/*
** Name: JKRobotClient.cpp
** Description:
**     机器人 ROS2 CPP版客户端
**
** Author: ZeFei Wang
** Email: starsky23311@gmail.com
** License: MIT License
*/

#include "jk_robot_server/JKRobotClient.h"

JKRobotClient::JKRobotClient(){

}

JKRobotClient::~JKRobotClient() {
}

/**
 * 对客户端初始化，默认配置，如需指定队列长度，联系开发者
 * @param node
 * @param service_name
 * @param callback_group
 * @return
 */
bool JKRobotClient::createJKRobotClient(rclcpp::Node::SharedPtr node, string service_name, rclcpp::CallbackGroup::SharedPtr callback_group) {
    node_ = node;
    jk_robot_client_ = node_->create_client<jk_robot_msgs::srv::StrFuncCommand>(service_name, rmw_qos_profile_services_default, callback_group);
    return true;
}

/**
 * 用于ROS2回调函数中的发送指令函数（直接用在主函数会卡住）
 * @param func_name
 * @param args
 * @param func_return
 * @return
 */
bool JKRobotClient::sendCommandCB(rclcpp::Node::SharedPtr node, string func_name, string args, json &func_return) {
    auto request = std::make_shared<jk_robot_msgs::srv::StrFuncCommand::Request>();
    request->func_name = func_name;
    request->args = args;

    while (!jk_robot_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = jk_robot_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        func_return = json::parse(result.get()->func_return);
        if (!func_return["success"])
            return false;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("jk_robot_client"), "Failed to call service");
        return false;
    }
    return true;
}

/**
 * 用于ROS2主函数中的发送指令函数（需要手动spinsome，否则会卡住）
 * @param func_name
 * @param args
 * @param func_return
 * @return
 */
bool JKRobotClient::sendCommand(string func_name, string args, json &func_return) {
    auto request = std::make_shared<jk_robot_msgs::srv::StrFuncCommand::Request>();
    request->func_name = func_name;
    request->args = args;

    while (!jk_robot_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = jk_robot_client_->async_send_request(request);

    if (result.wait_for(std::chrono::microseconds(500)) == std::future_status::timeout)
    {
        func_return = json::parse(result.get()->func_return);
        if (!func_return["success"])
            return false;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("jk_robot_client"), "Failed to call service");
        return false;
    }
    return true;
}

/**
 * 使能机械臂，参考时延 6ms，使能需要在执行该代码后等待
 * @param enable 1为使能，0为失能
 * @return
 */
bool JKRobotClient::setEnable(int enable) {
    args_to_send_["enable"] = enable;
    bool success = sendCommandCB(node_, "setEnable", args_to_send_.dump(), j_return_);
    args_to_send_.clear();
    return success;
}

/**
 * 设置机械臂运动标志位，参考时延 2ms
 * @param flag 1为运动，0为暂停
 * @return
 */
bool JKRobotClient::setSwitchFlag4(int flag) {
    args_to_send_["flag"] = flag;
    bool success = sendCommandCB(node_, "setSwitchFlag4", args_to_send_.dump(), j_return_);
    args_to_send_.clear();
    return success;
}

/**
 * 设置机械臂运动模式，参考时延 2ms
* @param mode ENDVEL为末端速度，ENDPOS为末端位姿
 * @return
 */
bool JKRobotClient::setMode(Mode mode) {
    args_to_send_["mode"] = mode;
    bool success = sendCommandCB(node_, "setMode", args_to_send_.dump(), j_return_);
    args_to_send_.clear();
    return success;
}

/**
 * 获取末端位姿，参考时延 2ms
 * @param pos 获取的位姿会赋值给pos，6位
 * @return
 */
bool JKRobotClient::getEndPos(vector<double> &pos) {
    bool success = sendCommandCB(node_, "getEndPos", args_to_send_.dump(), j_return_);
    pos = j_return_["data"].get<vector<double>>();
    return success;
}

/**
 * 末端位姿模式的机械臂运作，参考时延 7ms
 * @param pos 设置的末端位姿，6位
 * @return
 */
bool JKRobotClient::setPos(const vector<double> pos) {
    args_to_send_["Pos"] = pos;
    bool success = sendCommandCB(node_, "setPos", args_to_send_.dump(), j_return_);
    args_to_send_.clear();
    return success;
}

/**
 * 末端速度模式的机械臂运作，参考时延 4ms
 * @param vel 设置的末端速度，6位
 * @return
 */
bool JKRobotClient::setEndVel(const vector<double> vel) {
    args_to_send_["vel"] = vel; //单位为厘米，度
    bool success = sendCommandCB(node_, "setEndVel", args_to_send_.dump(), j_return_);
    args_to_send_.clear();
    return success;
}
