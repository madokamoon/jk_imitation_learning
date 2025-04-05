//
// Created by starsky on 25-2-22.
//

#include "jk_robot_server/RobotStateUnifiedInterfaceWrapper.h"

RobotStateUnifiedInterfaceWrapper::RobotStateUnifiedInterfaceWrapper() {
    pUnifiedInterfaceFuncMap["getEndPos"] = std::bind(&RobotStateUnifiedInterfaceWrapper::getEndPos, this, std::placeholders::_1);

    pUnifiedInterfaceFuncMap["setEnable"] = std::bind(&RobotStateUnifiedInterfaceWrapper::setEnable, this, std::placeholders::_1);
    pUnifiedInterfaceFuncMap["setSwitchFlag4"] = std::bind(&RobotStateUnifiedInterfaceWrapper::setSwitchFlag4, this, std::placeholders::_1);
    pUnifiedInterfaceFuncMap["setMode"] = std::bind(&RobotStateUnifiedInterfaceWrapper::setMode, this, std::placeholders::_1);
    pUnifiedInterfaceFuncMap["setPos"] = std::bind(&RobotStateUnifiedInterfaceWrapper::setPos, this, std::placeholders::_1);
    pUnifiedInterfaceFuncMap["setEndVel"] = std::bind(&RobotStateUnifiedInterfaceWrapper::setEndVel, this, std::placeholders::_1);

}

RobotStateUnifiedInterfaceWrapper::~RobotStateUnifiedInterfaceWrapper() {
    // pUnifiedInterfaceFuncMap.clear();
    cout << "kill" << endl;
}

string RobotStateUnifiedInterfaceWrapper::handleCommand(const string &func_name, const string &args) {
    json j_return;
    // 判断映射是否存在该函数名
    if (pUnifiedInterfaceFuncMap.count(func_name) == 1) {
        cout << "handle command: " << func_name << endl;
        return pUnifiedInterfaceFuncMap[func_name](args);
    }

    cout << "Unknown function: " << func_name << endl;
    j_return["success"] = false;
    j_return["data"] = nullptr;
    return j_return.dump();
}

string RobotStateUnifiedInterfaceWrapper::getEndPos(const string &args) {
    json j_return;
    Vector6d endPos;
    if (this->robotState::getEndPos(endPos)) {
        j_return["success"] = true;
        j_return["data"] = vector<double>(&endPos(0, 0), endPos.data() + endPos.cols()*endPos.rows());
    }
    else {
        j_return["success"] = false;
        j_return["data"] = nullptr;
    }
    return j_return.dump();
}

string RobotStateUnifiedInterfaceWrapper::setEnable(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("enable")) {
        cout << "Error: setEnable() has property enable." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }
    if (!j["enable"].is_number_integer()) {
        cout << "Error: The type of the enable is integer." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }

    if (this->robotState::setEnable(j["enable"])) {
        j_return["success"] = true;
    }
    else {
        j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();
}

string RobotStateUnifiedInterfaceWrapper::setSwitchFlag4(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("flag")) {
        cout << "Error: setSwitchFlag4() has property enable." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }
    if (!j["flag"].is_number_integer()) {
        cout << "Error: The type of the flag is integer." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }

    if (this->robotState::setSwitchFlag4(j["flag"])) {
        j_return["success"] = true;
    }
    else {
        j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();
}

string RobotStateUnifiedInterfaceWrapper::setMode(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("mode")) {
        cout << "Error: setMode() has property mode." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }
    if (!j["mode"].is_number_integer()) {
        cout << "Error: The type of the mode is integer." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }

    if (this->robotState::setMode(j["mode"])) {
        j_return["success"] = true;
    }
    else {
        j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();
}

string RobotStateUnifiedInterfaceWrapper::setPos(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("Pos")) {
        cout << "Error: setPos() has property Pos." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }
    if (!j["Pos"].is_array()) {
        cout << "Error: The type of the Pos is array." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }

    Vector6d endPos(j["Pos"].get<vector<double>>().data());
    if (this->robotState::setPos(endPos)) {
        j_return["success"] = true;
    }
    else {
        j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();

}

string RobotStateUnifiedInterfaceWrapper::setEndVel(const string &args) {
    json j = json::parse(args);
    json j_return;
    if (!j.contains("vel")) {
        cout << "Error: setEndVel() has property vel." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }
    if (!j["vel"].is_array()) {
        cout << "Error: The type of the vel is array." << endl;
        j_return["success"] = false;
        j_return["data"] = nullptr;
        return j_return;
    }

    Vector6d endVel(j["vel"].get<vector<double>>().data());
    if (this->robotState::setEndVel(endVel)) {
        j_return["success"] = true;
    }
    else {
        j_return["success"] = false;
    }
    j_return["data"] = nullptr;
    return j_return.dump();

}

