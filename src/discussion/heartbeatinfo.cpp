#include "formation/discussion/heartbeatinfo.h"

using namespace formation;

HeartBeatInfo::HeartBeatInfo()
{

}

HeartBeatInfo::HeartBeatInfo(int robotID, int time, bool timeout)
{
  this->RobotID = robotID;
  this->Time = time;
  this->Timeout = timeout;
}

void HeartBeatInfo::setRobotID(int robotID)
{
  this->RobotID = robotID;
}

int HeartBeatInfo::getRobotID()
{
  return this->RobotID;
}

void HeartBeatInfo::setTime(int time)
{
  this->Time = time;
}

int HeartBeatInfo::getTime()
{
  return this->Time;
}

void HeartBeatInfo::setTimeout(bool timeout)
{
  this->Timeout = timeout;
}

bool HeartBeatInfo::getTimeout()
{
  return this->Timeout;
}
