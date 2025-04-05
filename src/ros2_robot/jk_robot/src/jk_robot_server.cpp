#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "jk_robot_msgs/srv/str_func_command.hpp"
#include "std_msgs/msg/string.hpp"

#include "jk_robot_server/RobotStateUnifiedInterfaceWrapper.h"
#include "jk_robot_server/TCPClient.h"
#include "jk_robot_server/TCPServer.h"

#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <filesystem>
#include <std_msgs/msg/detail/header__traits.hpp>


using namespace std::chrono_literals;
using namespace std;
namespace fs = std::filesystem;

class JKRobotServer : public rclcpp::Node {
  public:
  JKRobotServer() : Node("jk_robot_server_node"){
    // 初始化 ROS2 服务
    service_ = this->create_service<jk_robot_msgs::srv::StrFuncCommand>("jk_robot_server",
    std::bind(&JKRobotServer::handleCommand, this, std::placeholders::_1, std::placeholders::_2));

    // 读取配置
    string config_path = std::string("config/config_jk_robot.yaml");
    YAML::Node config = YAML::LoadFile(config_path);

    string TCP_IP = config["ip"].as<string>();
    int TCP_PORT = config["port"].as<int>();

    // 与JK机械臂建立TCP连接
    bool tcp_flag = tcp_client_jkrobot_.setup(TCP_IP.c_str(), TCP_PORT);
    if (tcp_flag == false)
    {
      cout << "Connect failed. Error: No route to host" << endl;
      exit(1);
    }
    else {
      cout << "Tcp "<< TCP_IP << ":" << TCP_PORT << " Connected." << endl;
    }
    robot_.getTcpPointer(&tcp_client_jkrobot_);
  }

  ~JKRobotServer() {
    // this->undeclare_parameter("jk_robot_server_ids");
  }
  private:
  rclcpp::Service<jk_robot_msgs::srv::StrFuncCommand>::SharedPtr service_;

  TCPClient tcp_client_jkrobot_;
  RobotStateUnifiedInterfaceWrapper robot_;

  void handleCommand(const jk_robot_msgs::srv::StrFuncCommand::Request::SharedPtr request,
                     jk_robot_msgs::srv::StrFuncCommand::Response::SharedPtr response) {
    const string name = request->func_name;
    const string args = request->args;
    response->func_return = this->robot_.handleCommand(name, args);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  JKRobotServer::SharedPtr jk_robot_server = std::make_shared<JKRobotServer>();

  rclcpp::spin(jk_robot_server);
  rclcpp::shutdown();
}

