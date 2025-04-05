//
// Created by starsky on 25-3-6.
//

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "jk_robot_msgs/srv/str_func_command.hpp"

#include "robotiq/robotiq_gripper_interface.h"

#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <filesystem>
#include "nlohmann/json.hpp"
#include <assert.h>

using namespace std::chrono_literals;
using namespace std;
namespace fs = std::filesystem;
using json = nlohmann::json;

class Robotiq2F85Server : public rclcpp::Node{
public:
  Robotiq2F85Server() : Node("robotiq_2f85_server_node"){
    // pUnifiedInterfaceFuncMap_["connect"] = std::bind(&Robotiq2F85Server::connect, this, std::placeholders::_1);
    // pUnifiedInterfaceFuncMap_["reset"] = std::bind(&Robotiq2F85Server::reset, this, std::placeholders::_1);
    // pUnifiedInterfaceFuncMap_["activate"] = std::bind(&Robotiq2F85Server::activate, this, std::placeholders::_1);
    // pUnifiedInterfaceFuncMap_["is_activated"] = std::bind(&Robotiq2F85Server::is_activated, this, std::placeholders::_1);
    pUnifiedInterfaceFuncMap_["close_gripper"] = std::bind(&Robotiq2F85Server::close_gripper, this, std::placeholders::_1);
    pUnifiedInterfaceFuncMap_["open_gripper"] = std::bind(&Robotiq2F85Server::open_gripper, this, std::placeholders::_1);
    // pUnifiedInterfaceFuncMap_["set_gripper_position"] = std::bind(&Robotiq2F85Server::set_gripper_position, this, std::placeholders::_1);
    // pUnifiedInterfaceFuncMap_["set_timeout"] = std::bind(&Robotiq2F85Server::set_timeout, this, std::placeholders::_1);


    service_ = this->create_service<jk_robot_msgs::srv::StrFuncCommand>("robotiq_2f85_server",
      std::bind(&Robotiq2F85Server::handleCommand, this, std::placeholders::_1, std::placeholders::_2));


    string config_path = std::string("config/config_robotiq_2f85.yaml");
    YAML::Node config = YAML::LoadFile(config_path);

    string RS485_PORT = config["port"].as<string>();
    int RS485_BAUD = config["baud"].as<int>();
    assert(gripper_.connect(RS485_PORT, RS485_BAUD));
    assert(gripper_.reset());
    assert(gripper_.activate());
    cout << "夹爪初始化完毕。" << endl;
  }
  ~Robotiq2F85Server(){}

  void handleCommand(const jk_robot_msgs::srv::StrFuncCommand::Request::SharedPtr request,
                     jk_robot_msgs::srv::StrFuncCommand::Response::SharedPtr response) {
    const string func_name = request->func_name;
    const string args = request->args;
    // 判断映射是否存在该函数名
    if (pUnifiedInterfaceFuncMap_.count(func_name) == 1) {
      cout << "handle command: " << func_name << endl;
      response->func_return =  pUnifiedInterfaceFuncMap_[func_name](args);
      return;
    }

    cout << "Unknown function: " << func_name << endl;
    json j_return;
    j_return["success"] = false;
    j_return["data"] = nullptr;
    response->func_return = j_return.dump();
  }

  string close_gripper(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("blocking")) {
      cout << "Error: close_gripper() has property blocking." << endl;
      j_return["success"] = false;
      j_return["data"] = nullptr;
      return j_return;
    }
    if (!j["blocking"].is_boolean()) {
      cout << "Error: The type of the mode is boolean." << endl;
      j_return["success"] = false;
      j_return["data"] = nullptr;
      return j_return;
    }

    if (this->gripper_.close_gripper(j["blocking"])) {
      j_return["success"] = true;
    }
    else {
      j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();
  }

  string open_gripper(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("blocking")) {
      cout << "Error: close_gripper() has property blocking." << endl;
      j_return["success"] = false;
      j_return["data"] = nullptr;
      return j_return;
    }
    if (!j["blocking"].is_boolean()) {
      cout << "Error: The type of the mode is boolean." << endl;
      j_return["success"] = false;
      j_return["data"] = nullptr;
      return j_return;
    }

    if (this->gripper_.open_gripper(j["blocking"])) {
      j_return["success"] = true;
    }
    else {
      j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();
  }

private:
  robotiq::RobotiqGripperInterface gripper_;
  rclcpp::Service<jk_robot_msgs::srv::StrFuncCommand>::SharedPtr service_;
  unordered_map<string,std::function<string(string)>> pUnifiedInterfaceFuncMap_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Robotiq2F85Server::SharedPtr robotiq_2f85_server = std::make_shared<Robotiq2F85Server>();

  RCLCPP_INFO(rclcpp::get_logger("robotiq_2f85_server"), "Ready.");

  rclcpp::spin(robotiq_2f85_server);
  rclcpp::shutdown();
}

