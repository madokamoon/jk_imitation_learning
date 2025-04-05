//
// Created by starsky on 25-2-22.
//
#include "robotState.h"
#include <unordered_map>
#include <functional>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

#ifndef ROBOTSTATEUNIFIEDINTERFACEWRAPPER_H
#define ROBOTSTATEUNIFIEDINTERFACEWRAPPER_H



class RobotStateUnifiedInterfaceWrapper : public robotState{
public:
    RobotStateUnifiedInterfaceWrapper();
    ~RobotStateUnifiedInterfaceWrapper();

    string handleCommand(const string &func_name, const string &args);
    string getEndPos(const string &args);
    string setEnable(const string &args);
    string setSwitchFlag4(const string &args);
    string setMode(const string &args);
    string setPos(const string &args);
    string setEndVel(const string &args);

private:
    // robotState m_robot;
    // typedef bool (*pUnifiedInterfaceFunc)(string);
    unordered_map<string,std::function<string(string)>> pUnifiedInterfaceFuncMap;
};



#endif //ROBOTSTATEUNIFIEDINTERFACEWRAPPER_H
