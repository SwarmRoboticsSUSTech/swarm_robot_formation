#ifndef HEARTBEAT_H
#define HEARTBEAT_H


class HeartBeatInfo
{
public:
  HeartBeatInfo();
  HeartBeatInfo(int robotID, int time, bool timeout);
  void setRobotID(int robotID);
  int getRobotID();
  void setTime(int time);
  int getTime();
  void setTimeout(bool timeout);
  bool getTimeout();

private:
  int RobotID;
  int Time;
  bool Timeout;
};

#endif // HEARTBEAT_H
