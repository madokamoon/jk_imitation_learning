#include "rclcpp/rclcpp.hpp"
#include "jk_robot_msgs/srv/str_func_command.hpp"

#include <memory>
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std;

#ifndef JKROBOTCLIENT_H
#define JKROBOTCLIENT_H

class JKRobotClient{
    public:
    typedef enum Mode {
        ENDVEL = 5,
        ENDPOS = 6,
    };

    JKRobotClient();
    ~JKRobotClient();

    bool createJKRobotClient(rclcpp::Node::SharedPtr node, string service_name, rclcpp::CallbackGroup::SharedPtr callback_group = nullptr);
    bool sendCommandCB(rclcpp::Node::SharedPtr node, string func_name, string args, json &func_return);
    bool sendCommand(string func_name, string args, json &func_return);

    bool setEnable(int enable);
    bool setSwitchFlag4(int flag);
    bool setMode(Mode mode) ;
    bool getEndPos(vector<double> &pos);
    bool setPos(const vector<double> pos);
    bool setEndVel(const vector<double> vel);
  
    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<jk_robot_msgs::srv::StrFuncCommand>::SharedPtr jk_robot_client_;

    json args_to_send_;
    json j_return_;
};



#endif //JKROBOTCLIENT_H
