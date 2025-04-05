/*
** Name: jk_robot_client_test.cpp
** Description:
**     机器人 ROS2 CPP版客户端测试
**
** Author: ZeFei Wang
** Email: starsky23311@gmail.com
** License: MIT License
*/

#include "rclcpp/rclcpp.hpp"
#include "jk_robot_msgs/srv/str_func_command.hpp"
#include "jk_robot_server/JKRobotClient.h"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <time.h>
#include <sys/timeb.h>
#include "nlohmann/json.hpp"
using json = nlohmann::json;

using namespace std::chrono_literals;
using namespace std;

class JKRobotClientTest{
    public:
    JKRobotClientTest(string node_name){
        _node = rclcpp::Node::make_shared(node_name);
        _client.createJKRobotClient(_node, "jk_robot_server");
        cout << "Created JKRobotClient" << endl;
    }

    void test_setEnable() {
        ftime(&start_time);
        bool success = _client.setEnable(0);
        ftime(&end_time);
        double elapsed = (end_time.time - start_time.time)*1000 + (end_time.millitm - start_time.millitm);
        cout << "Elapsed time = " << elapsed << " ms" << endl;
        cout << "Success: " << success << endl;
    }

    void test_setSwitchFlag4() {
        ftime(&start_time);
        bool success = _client.setSwitchFlag4(1);
        ftime(&end_time);
        double elapsed = (end_time.time - start_time.time)*1000 + (end_time.millitm - start_time.millitm);
        cout << "Elapsed time = " << elapsed << " ms" << endl;
        cout << "Success: " << success << endl;
    }

    void test_setMode() {
        ftime(&start_time);
        bool success = _client.setMode(JKRobotClient::Mode::ENDPOS);
        ftime(&end_time);
        double elapsed = (end_time.time - start_time.time)*1000 + (end_time.millitm - start_time.millitm);
        cout << "Elapsed time = " << elapsed << " ms" << endl;
        cout << "Success: " << success << endl;
    }

    void test_getEndPos() {
        ftime(&start_time);
        vector<double> pos;
        bool success = _client.getEndPos(pos);
        ftime(&end_time);
        if(success)
            cout << pos.data() << endl;
        double elapsed = (end_time.time - start_time.time)*1000 + (end_time.millitm - start_time.millitm);
        cout << "Elapsed time = " << elapsed << " ms" << endl;
    }

    void test_setPos() {
        //设置模式
        bool success = _client.setMode(JKRobotClient::Mode::ENDPOS);
        rclcpp::sleep_for(std::chrono::microseconds(200));
        //设置运行
        success = _client.setSwitchFlag4(1);
        rclcpp::sleep_for(std::chrono::microseconds(200));
        //设置末端位姿
        vector<double> pos = vector<double>{0.093, 0.336, 0.341, -172.385, -1.685, 122.126};
        ftime(&start_time);
        success = _client.setPos(pos);
        ftime(&end_time);
        double elapsed = (end_time.time - start_time.time)*1000 + (end_time.millitm - start_time.millitm);
        cout << "Elapsed time = " << elapsed << " ms" << endl;
    }

    void test_setEndVel() {
        //设置模式
        bool success = _client.setMode(JKRobotClient::Mode::ENDVEL);
        rclcpp::sleep_for(std::chrono::microseconds(200));
        //设置运行
        success = _client.setSwitchFlag4(1);
        rclcpp::sleep_for(std::chrono::microseconds(200));
        //运作
        vector<double> vel = vector<double>{0.0, 0.0, 0.1, 0.0, 0.0, 0.0}; //单位为厘米，度
        ftime(&start_time);
        _client.setEndVel(vel);
        ftime(&end_time);
        double elapsed = (end_time.time - start_time.time)*1000 + (end_time.millitm - start_time.millitm);
        cout << "Elapsed time = " << elapsed << " ms" << endl;
    }

    private:
    rclcpp::Node::SharedPtr _node;
    JKRobotClient _client;

    struct timeb start_time, end_time;
};




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(500);
    auto jk_robot_client_test = std::make_shared<JKRobotClientTest>("jk_robot_client_test_node");

    // // setEnable
    // jk_robot_client_test->test_setEnable();

    // // setSwitchFlag4
    // jk_robot_client_test->test_setSwitchFlag4();

    // // setMode
    // jk_robot_client_test->test_setMode();

    // // getEndPos
    // while(rclcpp::ok()) {
    //     jk_robot_client_test->test_getEndPos();
    // }

    // // setPos
    // jk_robot_client_test->test_setPos();

    // setEndVel
    jk_robot_client_test->test_setEndVel();

    cout<<"Finished"<<endl;
    rclcpp::shutdown();
    return 0;
}
