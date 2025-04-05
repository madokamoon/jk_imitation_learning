/**
 * @file   robotState.cpp
 * @brief
 *
 * <long description>
 *
 * @author Liu Chenlu
 * @date   2019-05-21
 */
#include "jk_robot_server/robotState.h"
#include <string>
#include <vector>
#include <chrono>

#define endr "\n\r"
robotState::robotState() {
  coeffJv << 1, 1, 1, 1, 1, 1;
}
using namespace chrono;

int SplitString(const string &s, vector<string> &v, const string &c) {
  string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  v.clear();
  while (string::npos != pos2) {
    v.push_back(s.substr(pos1, pos2 - pos1));
    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length())
    v.push_back(s.substr(pos1));
  return v.size();
}

int robotState::updateRobotState() {
  getJoints(curJoint);
  getJointsVel(curJointVel);
  getEndPos(cEnd);
  return 1;
}

int robotState::printRobotState() {
  cout << "End pos: " << cEnd.transpose() << endr
       << "Joint pos: " << curJoint.transpose() << endr;
  return 0;
}

int robotState::getTcpPointer(TCPClient *_tcp) {
  tcp = _tcp;
  return 1;
}

/**************** general method *********************/
int robotState::getOneParameter(const char *cmdStr, void *value,
                                DATA_TYPE type) {
  vector<string> splitStrs;

  tcp->Send(cmdStr);
  string str = tcp->receive(1000);
  int n = SplitString(str, splitStrs, " ");
  if (n != 2)
    return -1;
  if (strcasecmp(splitStrs[0].c_str(), cmdStr) != 0)
    return 0;
  switch (type) {
    case BOOL_TYPE: {
      int *p = (int *)value;
      if (splitStrs[1] == "0") {
	*p = 0;
      } else if (splitStrs[1] == "1") {
	*p = 1;
      } else
	return 0;
      break;
    }
    case INT_TYPE: {
      int *p = (int *)value;
      *p = atoi(splitStrs[1].c_str());
      break;
    }
    case DOUBLE_TYPE: {
      double *p = (double *)value;
      *p = atof(splitStrs[1].c_str());
      break;
    }
    default:
      return 0;
      break;
  }
  return 1;
}

int robotState::setOneParameter(const char *cmdStr, void *value,
                                DATA_TYPE type) {
  vector<string> splitStrs;
  char cmd[256] = "";

  switch (type) {
    case BOOL_TYPE: {
      int *p = (int *)value;
      sprintf(cmd, "%s %d", cmdStr, *p);
      break;
    }
    case INT_TYPE: {
      int *p = (int *)value;
      sprintf(cmd, "%s %d", cmdStr, *p);
      cout << cmd << endl;
      break;
    }
    case DOUBLE_TYPE: {
      double *p = (double *)value;
      sprintf(cmd, "%s %lf", cmdStr, *p);
      break;
    }
    default:
      return 0;
      break;
  }

  tcp->Send(cmd);

  string str = tcp->receive(1000);

  int n = SplitString(str, splitStrs, " ");
  if (n != 2)
    return -1;
  if (strcasecmp(splitStrs[0].c_str(), cmdStr) != 0)
    return 0;
  if (strcasecmp(splitStrs[1].c_str(), "OK") != 0)
    return 0;

  return 1;
}

int robotState::getMultiParameters(const char *cmdStr, void *value, int dim,
                                   DATA_TYPE type) {
  vector<string> splitStrs;
  if (dim <= 0) return 0;

  tcp->Send(cmdStr);
  string str = tcp->receive(1000);

  //cout << "recStr: "<<str<<endr;

  int n = SplitString(str, splitStrs, " ");
  if (n != dim + 1)
    return -1;
  if (strcasecmp(splitStrs[0].c_str(), cmdStr) != 0)
    return 0;
  switch (type) {
    case BOOL_TYPE: {
      int *p = (int *)value;
      for (int i = 0; i < dim; i++) {
	if (splitStrs[i + 1] == "0") {
	  p[i] = 0;
	} else if (splitStrs[i + 1] == "1") {
	  p[i] = 1;
	} else
	  return 0;
      }
      break;
    }
    case INT_TYPE: {
      int *p = (int *)value;
      for (int i = 0; i < dim; i++)
	p[i] = atoi(splitStrs[i + 1].c_str());
      break;
    }
    case DOUBLE_TYPE: {
      double *p = (double *)value;
      for (int i = 0; i < dim; i++)
	p[i] = atof(splitStrs[i + 1].c_str());
      break;
    }

    default:
      return 0;
      break;
  }
  return 1;
}

int robotState::setMultiParameters(const char *cmdStr, void *value,
																	 int dim, DATA_TYPE type) {
  vector<string> splitStrs;
  string cmd = "";
  char str[256] = "";

  if (dim <= 0) return 0;

  switch (type) {
    case BOOL_TYPE: {
      int *p = (int *)value;
      cmd = cmdStr;
      for (int i = 0; i < dim; i++) {
	sprintf(str, " %d", p[i]);
	cmd += str;
      }
      break;
    }
    case INT_TYPE: {
      int *p = (int *)value;
      cmd = cmdStr;
      for (int i = 0; i < dim; i++) {
	sprintf(str, " %d", p[i]);
	cmd += str;
      }
      break;
    }
    case DOUBLE_TYPE: {
      double *p = (double *)value;
      cmd = cmdStr;
      for (int i = 0; i < dim; i++) {
	sprintf(str, " %lf", p[i]);
	cmd += str;
      }
      break;
    }
    default:
      return 0;
      break;
  }
  tcp->Send(cmd);

  string str2 = tcp->receive(1000);

  int n = SplitString(str2, splitStrs, " ");
  if (n != 2)
    return -1;
  if (strcasecmp(splitStrs[0].c_str(), cmdStr) != 0)
    return 0;
  if (strcasecmp(splitStrs[1].c_str(), "OK") != 0)
    return 0;

  return 1;
}
/******************* get status *********************/
int robotState::getEnableState(int* Enable) {
  if (getOneParameter("GETEnable", Enable, INT_TYPE) == 0)
    return 0;
  else
    cout << "GETEnable : OK" << endr;
  return 1;
}

int robotState::getJoints(Vector6d& joints) {
  double value[6];
  if (getMultiParameters("GETJ", value, 6, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 6; i++) {
      joints[i] = value[i];
    }
  }
  // cout << "GETJ: OK" << joints.transpose() << endr;
  return 1;
}

int robotState::getEndVel(Vector6d& vel) {
  double value[6];
  if (getMultiParameters("GETEV", value, 6, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 6; i++) {
      vel[i] = value[i];
    }
  }
  // cout << "GETJ: OK" << joints.transpose() << endr;
  return 1;
}


int robotState::getJointsVel(Vector6d &joints) {
  double value[6];
  if (getMultiParameters("GETJV", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  else {
    for (int i =0; i< 6; i++) joints[i] = value[i];
  }
  //  cout << "GETJV: OK" << joints.transpose() << endr;
  return 1;
}

int robotState::getEndPos(Vector6d &pos) {
  double value[6];
  if (getMultiParameters("GETP", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  else {
    for (int i =0; i< 6; i++) pos[i] = value[i];
  }
  //  cout << "GETP: OK" << pos.transpose() << endr;
  return 1;
}

int robotState::getMode(int* mode) {
  if (getOneParameter("GETM", mode, INT_TYPE) == 0)
    return 0;
  else
    cout << "GETM : OK" << endr;
  return 1;
}

int robotState::getPvr(double* ratio) {
  if (getOneParameter("GETPVR", ratio, DOUBLE_TYPE) == 0)
    return 0;
  else
    cout << "GETPVR : OK" << endr;
  return 1;
}

int robotState::getJvr(double* ratio) {
  if (getOneParameter("GETJVR", ratio, DOUBLE_TYPE) == 0)
    return 0;
  else
    cout << "GETJVR : OK" << endr;
  return 1;
}
int robotState::getSwitchFlag4(int* flag) {
  if (getOneParameter("GETSF4", flag, INT_TYPE) == 0)
    return 0;
  return 1;
}
int robotState::getRelMovAndFrameFlag(int* relFlag, int *frameFlag) {
  int value[2];
  if (getMultiParameters("GETFM", value, 2, INT_TYPE) == 0)
    return 0;
  else
  {
    *frameFlag = value[0];
    *relFlag = value[1];
  }
  cout << "GETFM2 : OK" << *relFlag<<*frameFlag << endr;
  return 1;
}

int robotState::getJointMovStatusFlag(int* flag) {
  if (getOneParameter("GETJF", flag, INT_TYPE) == 0)
    return 0;
  return 1;
}
int robotState::getPosMovStatusFlag(int* flag) {
  if (getOneParameter("GETPF", flag, INT_TYPE) == 0)
    return 0;
   // cout << "GETPF : OK"<< *flag << endr;
  return 1;
}

int robotState::getCircleMovStatusFlag(int* flag) {
  if (getOneParameter("GETCF", flag, INT_TYPE) == 0)
    return 0;
  return 1;
}

int robotState::getCartesianImpCtr(int* flag) {
  if (getOneParameter("GETCIC", flag, INT_TYPE) == 0)
    return 0;
  return 1;
}

int robotState::getForceTouchFlag(int* flag)
{
  if (getOneParameter("GETFTF", flag, INT_TYPE) == 0)
    return 0;
    // cout << "GETFTF : OK"<< *flag << endr;
  return 1;
}
int robotState::getForceTouchThres(Vector6d &thres) {
  double value[6];
  if (getMultiParameters("GETFORCE", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  else {
    for (int i =0; i< 6; i++)
      thres[i] = value[i];
  }
  cout << "GETFORCE: OK" << thres.transpose() << endr;
  return 1;
}
int robotState::getCarForceFeedFroward(Vector6d& ForceForward) {
  double value[6];
  if (getMultiParameters("GETCFFF", value, 6, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 6; i++) {
      ForceForward[i] = value[i];
    }
  }
  cout << "GETCFFF: OK" << ForceForward.transpose() << endr;
  return 1;
}
// 获取笛卡尔空间刚度
int robotState::getKX(Vector6d& KX) {
  double value[6];
  if (getMultiParameters("GETKX", value, 6, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 6; i++) {
      KX[i] = value[i];
    }
  }
  cout << "GETKX: OK" << KX.transpose() << endr;
  return 1;
}
//  设置笛卡尔空间zuni
int robotState::getKV(Vector6d& KV) {
  double value[6];
  if (getMultiParameters("GETKV", value, 6, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 6; i++) {
      KV[i] = value[i];
    }
  }
  cout << "GETKV: OK" << KV.transpose() << endr;
  return 1;
}

int robotState::getP7E(Vector3d& P7E) {
  double value[3];
  if (getMultiParameters("GETP7E", value, 3, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 3; i++) {
      P7E[i] = value[i];
    }
  }
  cout << "GETP7E: OK" << P7E.transpose() << endr;
  return 1;
}

int robotState::getLoadParas(Vector4d& LoadParas) {
  double value[4];
  if (getMultiParameters("GETLoadParas", value, 4, DOUBLE_TYPE) == 0) {
    return 0;
  } else {
    for (int i =0; i< 4; i++) {
      LoadParas[i] = value[i];
    }
  }
  cout << "GETLoadParas: OK" << LoadParas.transpose() << endr;
  return 1;
}
/******************* set status *********************/

int robotState::setMode(int mode)
{
  if (setOneParameter("SETM", &mode, INT_TYPE) == 0)
    return 0;
  // else
  //   cout << "SETM : OK" << endr;
  return 1;
}
int robotState::setPvr(double ratio) {
  if (setOneParameter("SETPVR", &ratio, DOUBLE_TYPE) == 0)
    return 0;
  // else
  //   cout << "SETPVR : OK" << endr;
  return 1;
}
int robotState::setEnable(int enable) {
  if (setOneParameter("SETENABLE", &enable, INT_TYPE) == 0) return 0;
  else
    cout << "SETENABLE : OK" << endr;
  return 1;
}

int robotState::setSafetyIndex(double ratio) {
  if (setOneParameter("SETSI", &ratio, DOUBLE_TYPE) == 0) return 0;
  // else
  //   cout << "SETSI : OK" << endr;
  return 1;
}

int robotState::setJvr(double ratio) {
  if (setOneParameter("SETJVR", &ratio, DOUBLE_TYPE) == 0)
    return 0;
  else
    // cout << "SETJVR : OK" << endr;
  return 1;
}

int robotState::setJvs(double ratio) {
  if (setOneParameter("SETJVS", &ratio, DOUBLE_TYPE) == 0)
    return 0;
  // else
  //   cout << "SETJVS : OK" << endr;
  return 1;
}

int robotState::setSwitchFlag4(int flag)
{
  if (setOneParameter("SETSF4", &flag, INT_TYPE) == 0)
    return 0;
  // else
  //   cout << "SETSF4 : OK " << flag << endr;
  return 1;
}

int robotState::setRelMovAndFrameFlag(int relFlag, int frameFlag) {
  int value[2];
  value[0] = frameFlag;
  value[1] = relFlag;

  if (setMultiParameters("SETFM", value, 2, INT_TYPE) == 0)
    return 0;
  // cout << "SETFM : OK" << relFlag<< frameFlag << endr;
  return 1;
}
int robotState::setCartesianImpCtr(int flag) {
  if (setOneParameter("SETCIC", &flag, INT_TYPE) == 0)
    return 0;
  else
    // cout << "SETCIC : OK" << endr;
  return 1;
}

int robotState::setForcelimitProtect(int flag) {
  if (setOneParameter("SETFCRMF", &flag, INT_TYPE) == 0) {
       cout<< "failed"<< endl;
       return 0;
}
  // cout << "SETFCRMF : OK" << endr;
  return 1;
}

int robotState::setP7E(Vector3d &P7E) {
  double value[3];
  for (int i =0; i< 3; i++) value[i] = P7E[i];
  if (setMultiParameters("SETP7E", value, 3, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETP7E : OK" << P7E.transpose() << endr;
  return 1;
}

int robotState::setLoadParas(Vector4d &LoadParas) {
  double value[4];
  for (int i =0; i< 4; i++) value[i] = LoadParas[i];
  if (setMultiParameters("SETLP", value, 4, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETLP : OK" << LoadParas.transpose() << endr;
  return 1;
}

/********************  Virtual wall **************************/
int robotState::setVirtualWallLimit(Vector6d &VirWallLimits) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = VirWallLimits[i];

  if (setMultiParameters("SETVWL", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETVWL : OK" << VirWallLimits.transpose() << endr;
  return 1;
}
int robotState::setVirtualWallAngle(double angle) {
  if (setOneParameter("SETVWA", &angle, DOUBLE_TYPE) == 0)
    return 0;
  else
    cout << "SETVWA : OK" << endr;
  return 1;
}
int robotState::setVirtualWallEnable(int flag) {
  if (setOneParameter("SETVWE", &flag, INT_TYPE) == 0)
    return 0;
  else
    cout << "SETVWE : OK" << endr;
  return 1;
}

/********************  set pos and joints and torque**************************/
int robotState::setJoints(const Vector6d &joints) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = joints[i];
  if (setMultiParameters("SETJ", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  // cout << "SETJ : OK" << joints.transpose() << endr;
  return 1;
}
int robotState::setJointsVel(const Vector6d &joints) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = joints[i];

  if (setMultiParameters("SETJV", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  // cout << "SETJV : OK" << joints.transpose() << endr;
  return 1;
}

int robotState::setTotalVel(double m_vel) {
  if (setPvr(m_vel) == 0) return 0;
  if (setJvr(m_vel) == 0) return 0;
  if (setJvs (m_vel) == 0) return 0;
  return 1;
}

int robotState::setPos(const Vector6d &pos) {
  double value[6];
  int rtn = 0;
  // cout << "SETP : rtn" << rtn << endr;
  for (int i =0; i< 6; i++) value[i] = pos[i];
  rtn = setMultiParameters("SETP", value, 6, DOUBLE_TYPE);
  // cout << "SETP : rtn" << rtn << endr;

  if (rtn == 0)
    return 0;
  // cout << "SETP : OK" << pos.transpose() << endr;
  return 1;
}
int robotState::setEndVel(const Vector6d &vel) {
  double value[6];
  int rtn = 0;
  for (int i =0; i< 6; i++) value[i] = vel[i];
  rtn = setMultiParameters("SETEV", value, 6, DOUBLE_TYPE);
  if (rtn == 0)
    return 0;
  return 1;
}
// 设置笛卡尔空间受力超限标志
int robotState::setForceTouchFlag(int flag) {
  if (setOneParameter("SETFTF", &flag, INT_TYPE) == 0)
    return 0;
  // else
  //   cout << "SETFTF : OK" << endr;
  return 1;
}
// set film
int robotState::setForceTouchThres(const Vector6d &thres) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = thres[i];

  if (setMultiParameters("SETFORCE", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETFORCE : OK" << thres.transpose() << endr;
  return 1;
}

int robotState::setCarForceFeedForward(const Vector6d &ForceForward) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = ForceForward[i];
  if (setMultiParameters("SETCFFF", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETCFFF : OK" << ForceForward.transpose() << endr;
  return 1;
}
//设置笛卡尔空间刚度
int robotState::setKX(const Vector6d &kx) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = kx[i];

  if (setMultiParameters("SETKX", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETKX : OK" << kx.transpose() << endr;
  return 1;
}
// 设置笛卡尔空间zuni
int robotState::setKV(const Vector6d &kv) {
  double value[6];
  for (int i =0; i< 6; i++) value[i] = kv[i];

  if (setMultiParameters("SETKV", value, 6, DOUBLE_TYPE) == 0)
    return 0;
  cout << "SETKV : OK" << kv.transpose() << endr;
  return 1;
}

int robotState::setModeOCCM(double MOCCM) {
  if (setOneParameter("SETMOCCM", &MOCCM, DOUBLE_TYPE) == 0) return 0;
  else
    cout << "SETMOCCM : OK" << endr;
  return 1;
}

int robotState::setROCCM(double rOCCM) {
  if (setOneParameter("SETROCCM", &rOCCM, DOUBLE_TYPE) == 0) return 0;
  else
    cout << "SETROCCM : OK" << endr;
  return 1;
}

int robotState::setSelectOPos(double select_O_pos) {
  if (setOneParameter("SETSOP", &select_O_pos, DOUBLE_TYPE) == 0) return 0;
  else
    cout << "SETSOP : OK" << endr;
  return 1;
}

int robotState::setSelectDirection(double select_direction) {
  if (setOneParameter("SETSD", &select_direction, DOUBLE_TYPE) == 0) return 0;
  else
    cout << "SETSD : OK" << endr;
  return 1;
}

//  force reset
int robotState::setForceReset(int flag) {
  if (setOneParameter("SETFR", &flag, INT_TYPE) == 0)
    return 0;
  else
    cout << "SETFR : OK" << endr;
  return 1;
}
/************** new middle level API ***************/



// int robotState::CmoveJoint(const Vector6d joints, double Jvr) {
//   int jointFlag = 1;
//   int getSF4;
  
//   // getSwitchFlag4(&getSF4);
//   // std::cout << "A" << getSF4 << std::endl;
  
//   // if (setSwitchFlag4(0) == 0 ) return 0;
  
//   // getSwitchFlag4(&getSF4);
//   // std::cout << "B"  << getSF4 << std::endl;
  
  
//   getSwitchFlag4(&getSF4);
//   std::cout << "C"  << getSF4 << std::endl;
  
//   if (setSwitchFlag4(1) == 0 ) return 0;
  
//   getSwitchFlag4(&getSF4);
//   std::cout << "D"  << getSF4 << std::endl;
  
//   return 1;
// }

int robotState::CmoveJoint(const Vector6d joints, double Jvr)
{
  int getSF4;
  if (setSwitchFlag4(0) == 0 ) return 0;
  if (setMode(2) == 0 ) return 0;
  if (setTotalVel(Jvr) == 0 ) return 0;
  if (setJoints(joints) == 0 ) return 0;
  usleep(1000);
  if (setSwitchFlag4(1) == 0 ) return 0;
  // 新增 到位判断
  int count = 0;
  int jointFlag = 0;
  while (!jointFlag) {
    getJointMovStatusFlag(&jointFlag);
    std::cout << "count = " << count++  << ",\t" << "jointFlag = " << jointFlag << "\t\r";
    usleep(10);
  }
  std::cout << "CmoveJoint Done" << std::endl;
  return 1;
}

int robotState::CmoveEndPos(const Vector6d pos, double Pvr, int frame, int relative) {
  // frame = 1 means tool frame, relative = 1 means relative movement
  int posFlag = 1;
  if (setSwitchFlag4(0) == 0 ) return 0;  // close running
  if (setMode(6) == 0) return 0;
  if (setTotalVel(Pvr) == 0 ) return 0;
  if (setRelMovAndFrameFlag(relative, frame)  == 0 ) return 0;
  if (setPos(pos) == 0 ) return 0;
  usleep(100);
  if (setSwitchFlag4(1) == 0 ) return 0;  // running
  // 新增 到位判断
  posFlag = 0;
  while (!posFlag) {
    getPosMovStatusFlag(&posFlag);
    usleep(10);
  }
  return 1;
}

int robotState::CmoveEEFXY(double distanceX, double distanceY, double Pvr, int ifSub)  {
  // frame = 1 means tool frame, relative = 1 means relative movement 注意工具系z+朝下，基系z+朝上
  int posFlag = 1;
  if (setSwitchFlag4(0) == 0 ) return 0;  // close running
  if (setMode(6) == 0) return 0;
  if (setTotalVel(Pvr) == 0 ) return 0;
  if (setRelMovAndFrameFlag(0, 1)  == 0 ) return 0;
  getEndPos(currentPos);
  targetPos = currentPos;
  targetPos[0] = targetPos[0] + pow(-1,ifSub)*distanceX;
  targetPos[1] = targetPos[1] - distanceY;
  if (setPos(targetPos) == 0 ) return 0;
  usleep(100);
  if (setSwitchFlag4(1) == 0 ) return 0;  // running
  posFlag = 0;
  while (!posFlag) {
    getPosMovStatusFlag(&posFlag);
    usleep(10);
  }
  if (setSwitchFlag4(0) == 0 ) return 0;
  return 1;
}


int robotState::CmoveUporDown(double distance, double Pvr, int frame, int relative, int mode, int upordown)  {
  // frame = 1 means tool frame, relative = 1 means relative movement 注意工具系z+朝下，基系z+朝上
  // mode = 5 末端速度模式，mode = 6 末端位置模式
  // upordown = 0 工具坐标系Z 负方向，upordown = 1 工具坐标系Z 正方向
  chrono::steady_clock::time_point timep1;
  chrono::steady_clock::time_point timep2;
  chrono::duration<double> timed12;

  int posFlag = 1;
  double velmodetime;
  double deltaTz;
  if (setSwitchFlag4(0) == 0 ) return 0;  // close running
  if (setMode(mode) == 0) return 0;
  if (setTotalVel(Pvr) == 0 ) return 0;
  if (setRelMovAndFrameFlag(relative, frame)  == 0 ) return 0;
  // std::cout << "before getEndPos" <<  std::endl;
  // if (getEndPos(currentPos)) return 0;
  getEndPos(currentPos);
  // std::cout << "before targetPos" <<  std::endl;
  targetPos = currentPos;
  // std::cout << "before pow" <<  std::endl;
  targetPos[2] = targetPos[2] + pow(-1,upordown)*distance;
  // std::cout << "before setPos" <<  std::endl;
  if(mode==6) { // 位置
    if (setPos(targetPos) == 0 ) return 0;
    usleep(100);
    if (setSwitchFlag4(1) == 0 ) return 0;  // running
    // 新增 到位判断
    posFlag = 0;
    while (!posFlag) {
      getPosMovStatusFlag(&posFlag);
    }
    if (setSwitchFlag4(0) == 0 ) return 0;
  }
  else if(mode==5) { // 速度 不到位就一直运行
    endVel << 0.0, 0.0, pow(-1,upordown)*pow(-1,frame)*Pvr, 0.0, 0.0, 0.0;
    if (setEndVel(endVel) == 0) return 0;
    usleep(100);
    if (setSwitchFlag4(1) == 0 ) return 0;  // running
    deltaTz = (targetPos[2] - currentPos[2])*pow(-1,upordown);
    while(deltaTz>0.0001) {
      getEndPos(currentPos);
      deltaTz = (targetPos[2] - currentPos[2])*pow(-1,upordown);
      // cout << "== false ==" << deltaTz << endl;
      // getJointsVel(endVel); // 数据单位按结果看应该是deg/s
      // std::cout << " endVel. " << endVel.transpose() << std::endl;
    }
    endVel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    if (setEndVel(endVel) == 0) return 0;
    if (setSwitchFlag4(0) == 0 ) return 0;
  }

  return 1;
}

