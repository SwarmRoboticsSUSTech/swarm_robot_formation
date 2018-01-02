#include "ros/ros.h"
#include "std_msgs/String.h"
#include "formation/ChatContent.h"
#include "formation/reference/leader_reference.h"
#include "formation/discussion/desirepositioninfo.h"
#include "formation/discussion/heartbeatinfo.h"
#include "formation/PositionRequest.h"

using namespace formation;

const std::string SCRAMBLE = "scramble";
const std::string HEARTBEAT = "heartbeat";
const std::string REJECT = "reject";
const int CANNOT_FIND_VACANCY = -1;
const int LATEST_TIME = 10;

std::vector<DesirePositionInfo> g_desirePositions;
std::vector<HeartBeatInfo> g_grabbedPositions;
int g_scramble_position;
int g_desire_position;
uint32_t g_my_id;
int g_scramble_count = 0;
const HeartBeatInfo g_initial_info(-1, 0, false);

ros::Publisher g_chatter_pub;

/*
 * Insert the heartbeat info into grabbed table.
 * If has not enought space, than resize the grabbed table.
*/
void insertToTable(int position, HeartBeatInfo info)
{
  if(g_grabbedPositions.size() <= position)
  {
    g_grabbedPositions.resize(position+1, g_initial_info);
  }
  g_grabbedPositions[position] = info;
}

/*
 * Update the heartbeat time of the grabbed table for each position.
 * Set the robot id with the num -1, if time is out.
*/
void updateTime()
{
  HeartBeatInfo info;
  for(int i = 0; i < g_grabbedPositions.size(); i++)
  {
    info = g_grabbedPositions[i];
    if(info.getRobotID() != -1)
    {
      info.setTime(info.getTime() + 1);
      if(info.getTime() > LATEST_TIME)
      {
        info.setTimeout(true);
        info.setRobotID(-1);
      }
      g_grabbedPositions[i] = info;
    }
  }
}

/*
 * Find a scramble position.Search from position 0.
 * This function is used only when the size of desired positions is 0.
 * */
int getScramblePosition()
{
  int position = 0;   //start form position 0
  for(int i = 0; i < g_grabbedPositions.size(); i++)
  {
    HeartBeatInfo info = g_grabbedPositions[i];
    if(info.getRobotID() != -1)
    {
      position = i + 1;
    }
    else
    {
      break;
    }
  }
  return position;
}

/*
 * Find a scramble position when has vacancy in front.
 * This function is used only when the size of desried position is 1.
 * Search in front self position.
 * */
int findVacancy()
{
  DesirePositionInfo dinfo = *(g_desirePositions.begin());
  int vacancyPosition = dinfo.getPosition() - 1;
  HeartBeatInfo info = g_grabbedPositions[vacancyPosition];

  //Search in front self position
  for(; vacancyPosition >= 0; vacancyPosition --)
  {
    info = g_grabbedPositions[vacancyPosition];
    if(info.getRobotID() != -1)
    {
      break;
    }
  }
  ROS_INFO("dinfo.getPosition() - 1:%d", dinfo.getPosition() - 1);
  ROS_INFO("vacancyPosition:%d", vacancyPosition);
  ROS_INFO("g_scramble_position:%d", g_scramble_position);

  // If "dinfo.getPosition() - 1 == vacancyPosition", there are not vacancy positions in front of self position.
  // "g_scramble_position != -1 && g_scramble_position ==vacancyPosition", we don't scranble the same position last time scrambled.
  if(dinfo.getPosition() - 1 == vacancyPosition ||
     (g_scramble_position != -1 && g_scramble_position ==vacancyPosition))
  {
    return -1;
  }
  else
  {
    // Because the use of "vacancyPosition --" in "for()" circulation.
    return vacancyPosition + 1;
  }
}

void receiver(const formation::ChatContent& receivedMsg)
{
  if(receivedMsg.msg_type == SCRAMBLE)
  {
    //Heared self message
    if(receivedMsg.robot_id == g_my_id && g_scramble_count == 0 && receivedMsg.desire_position == g_scramble_position)
    {
      //Authorized, and update desired position table
      DesirePositionInfo dinfo(receivedMsg.desire_position, 0);
      // Insert desired position into the zore position of the table.
      g_desirePositions.insert(g_desirePositions.begin(), 1, dinfo);
      //write into grabbed position table
      HeartBeatInfo info;
      info.setRobotID(receivedMsg.robot_id);
      info.setTime(0);
      info.setTimeout(false);
      insertToTable(receivedMsg.desire_position, info);
    }
    g_scramble_count = 1;

    //send reject message
    for(int i = 0; i< g_desirePositions.size(); i++)
    {
      DesirePositionInfo dinfo = g_desirePositions[i];
      if(dinfo.getPosition() == receivedMsg.desire_position && receivedMsg.robot_id != g_my_id)
      {
        //REJECT
        formation::ChatContent sendMsg;
        sendMsg.msg_type = REJECT;
        sendMsg.robot_id = g_my_id;
        sendMsg.desire_position = dinfo.getPosition();
        sendMsg.talk_to = receivedMsg.robot_id;
        g_chatter_pub.publish(sendMsg);
      }
    }
  }
  else if(receivedMsg.msg_type == HEARTBEAT)
  {
    //record update table
    HeartBeatInfo info;
    info.setRobotID(receivedMsg.robot_id);
    info.setTime(0);
    info.setTimeout(false);
    insertToTable(receivedMsg.desire_position, info);
  }
  else if(receivedMsg.msg_type == REJECT)
  {
    //write heartbeat info into grabbed table
    HeartBeatInfo info;
    info.setRobotID(receivedMsg.robot_id);
    info.setTime(0);
    info.setTimeout(false);
    insertToTable(receivedMsg.desire_position, info);

    if(receivedMsg.robot_id != g_my_id && receivedMsg.talk_to == g_my_id)
    {
      //update desired position table, delete the reject position
      DesirePositionInfo dinfo;
      for(int i = 0; i < g_desirePositions.size(); i++)
      {
        dinfo = g_desirePositions[i];
        if(dinfo.getPosition() == receivedMsg.desire_position)
        {
          g_desirePositions.erase(g_desirePositions.begin() + i);
        }
      }

      if(g_desirePositions.size() == 0)
      {
        //If the size of desired position is 0, we scramble position again.
        formation::ChatContent sendMsg;
        //search teble
        g_scramble_position = getScramblePosition();
        sendMsg.msg_type = SCRAMBLE;
        sendMsg.robot_id = g_my_id;
        sendMsg.desire_position = g_scramble_position;
        g_chatter_pub.publish(sendMsg);
        g_scramble_count = 0;
      }
      else if(g_desirePositions.size() == 1)
      {
        //Reset the heartbeat info of the only one desired position
        DesirePositionInfo dinfo = g_desirePositions[0];
        HeartBeatInfo info = g_grabbedPositions[dinfo.getPosition()];
        info.setTime(0);
        info.setTimeout(false);
        insertToTable(dinfo.getPosition(), info);
      }

      //Reset the timeout info of the frist desired position,
      //we don't want to timeing the frist desired position.
      if(g_desirePositions.size() > 0)
      {
        DesirePositionInfo dinfo = g_desirePositions[0];
        dinfo.setTime(dinfo.getTime() + 1);
        g_desirePositions[0] = dinfo;
      }
    }
  }
}

/*
 * In this function
 * 1.we timing the heartbeat info,
 * 2.delete the time timeout desired position,
 * 3.scramble vacancy position,
 * */
void timerCallback(const ros::TimerEvent& event)
{
  //update the timing of heartbeat table
  updateTime();

  if(g_desirePositions.size() > 0)
  {
    for(int i = g_desirePositions.size() - 1; i > -1; i--)
    {
      //HEARTBEAT
      formation::ChatContent sendMsg;
      DesirePositionInfo dinfo = g_desirePositions[i];
      sendMsg.msg_type = HEARTBEAT;
      sendMsg.robot_id = g_my_id;
      sendMsg.desire_position = dinfo.getPosition();
      g_chatter_pub.publish(sendMsg);
      //update the time of desire position
      if(i > 0)
      {
        dinfo.setTime(dinfo.getTime() + 1);
        if(dinfo.getTime() > LATEST_TIME)
        {
          g_desirePositions.erase(g_desirePositions.begin() + i);
        }
        else
        {
          g_desirePositions[i] = dinfo;
        }
      }
    }

    //search table, find vacancy position and scramble
    if(g_desirePositions.size() == 1)
    {
      g_scramble_position = findVacancy();
      ROS_INFO("g_scramble_position:%d", g_scramble_position);
      if(g_scramble_position > -1)
      {
        //SCRAMBLE
        formation::ChatContent sendMsg;
        sendMsg.msg_type = SCRAMBLE;
        sendMsg.robot_id = g_my_id;
        sendMsg.desire_position = g_scramble_position;
        ROS_INFO("have vacancy, send scramble!");
        g_chatter_pub.publish(sendMsg);
        g_scramble_count = 0;
      }
    }
  }
}

/*
 * Service function
 * respose the self desired position and leader id
 * */
bool requestPosition(formation::PositionRequest::Request &req,
                     formation::PositionRequest::Response &res)
{
  HeartBeatInfo hinfo = *(g_grabbedPositions.begin());
  int leader_id = hinfo.getRobotID();
  if(g_desirePositions.size() > 0 && leader_id != -1)
  {
    res.position = g_desire_position;
    res.leader_id = leader_id;
    ROS_INFO("leader_id: %d", leader_id);
    ROS_INFO("Service Robot%d Desired Position: %d", g_my_id, g_desire_position);
    return true;
  }
  else
  {
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "none_leader_discussion");
  ros::NodeHandle nh;
  std::string ns = nh.getNamespace();
  g_my_id = atoi(ns.substr(7, ns.size() - 7).c_str());

  g_chatter_pub = nh.advertise<formation::ChatContent>("none_leader_topic", 1000);
  ros::Subscriber chartter_sub = nh.subscribe("none_leader_topic", 1000, &receiver);

  ros::ServiceServer service = nh.advertiseService("position_requestion", requestPosition);

  ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);

  ROS_INFO("Sleep 1 sec for initialization!");
  ros::Duration(1).sleep();
  //SCRAMBLE
  g_scramble_position = getScramblePosition();
  formation::ChatContent sendMsg;
  sendMsg.msg_type = SCRAMBLE;
  sendMsg.robot_id = g_my_id;
  sendMsg.desire_position = g_scramble_position;
  g_chatter_pub.publish(sendMsg);
  g_scramble_count = 0;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if(g_desirePositions.size() > 0)
    {
      DesirePositionInfo dinfo = *(g_desirePositions.begin());
      g_desire_position = dinfo.getPosition();
      ROS_INFO("Robot%d Desired Position: %d", g_my_id, g_desire_position);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
