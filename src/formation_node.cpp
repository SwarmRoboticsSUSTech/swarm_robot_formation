#include "ros/ros.h"
#include "std_msgs/String.h"
#include "leader_reference.h"
#include "formation/PositionRequest.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base/move_base.h>
#include <actionlib/client/simple_action_client.h>
#include <signal.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include <iostream>
#include <string>

bool startFormationFlag = false;
double yawOfPath;
geometry_msgs::PoseStamped globalPlanPose;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* g_ac;
ros::Publisher g_sendplan_pub;
ros::ServiceClient g_clear_map_sc;
bool g_isleader = false;

void startCommandReceived(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("Received goal.");
  if(g_isleader)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header = msg.header;
    goal.target_pose.pose = msg.pose;
//  send goal to reach if this node is the leader
    g_ac->sendGoal(goal);
  }
}

/*
* In this function, we can get the global path of leader.
* So, the pose at position 0 is the begining point of the path.
* And, we can get Yaw through the Quaternion of orientation. The scope is 0~pi and 0~-pi.Positive of X define as zore.
*/
void getGlobalPlanOfLeader(const nav_msgs::Path& gui_path)
{
  ROS_INFO("Receive global plan of leader.");
  startFormationFlag = true;
  int sizeOfPathPose = (int)gui_path.poses.size();
//  ROS_INFO("gui_path.poses.size(): %d", sizeOfPathPose);
  if(sizeOfPathPose >= 40)
  {
    globalPlanPose = gui_path.poses[(gui_path.poses.size()-1)/2];
  }
  else
  {
    globalPlanPose = gui_path.poses[gui_path.poses.size()-1];
  }
//  ROS_INFO("gui_path.poses[end]: %f %f %f, %f %f %f %f", globalPlanPose.pose.position.x, globalPlanPose.pose.position.y, globalPlanPose.pose.position.z,
//           globalPlanPose.pose.orientation.x, globalPlanPose.pose.orientation.y, globalPlanPose.pose.orientation.z, globalPlanPose.pose.orientation.w);

//  ROS_INFO("yawOfPath_odom: %f", tf::getYaw(globalPlanPose.pose.orientation));
}

void getSelfGlobalPlan(const nav_msgs::Path& gui_path)
{
  if(g_isleader)
  {
    g_sendplan_pub.publish(gui_path);
  }
}

void shutdown(int sig)
{
  ros::Duration(1).sleep(); // sleep for  a second
  ROS_INFO("formation node ended!");
  g_ac->~SimpleActionClient();
  ros::shutdown();
}

void clearCostMapTimer(const ros::TimerEvent& event)
{
  std_srvs::Empty::Request req_clearCostmap;
  std_srvs::Empty::Response res_clearCostmap;
  g_clear_map_sc.call(req_clearCostmap, res_clearCostmap);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "formation_node");
  ros::NodeHandle nh;
  tf::TransformListener tfListener(ros::Duration(10));
  ros::Rate loop_rate(40);
  ROS_INFO("in the node!");
  std::string ns = nh.getNamespace();
  ns = ns.substr(1, ns.size() - 1);
  signal(SIGINT, shutdown);
  formation::LeaderRerence leaderRerence(&tfListener, formation::Line, 0.3);
  std::stringstream leader_id_stream;
  int leader_id;
  int position = -1;

  std_srvs::Empty::Request req_clearCostmap;
  std_srvs::Empty::Response res_clearCostmap;
  formation::PositionRequest::Request req_position;
  formation::PositionRequest::Response res_position;

  g_ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(ns + "/move_base", true);
  while(!g_ac->waitForServer(ros::Duration(5)))
  {
    ROS_WARN("Can't connect to move base server, %s!", ns.c_str());
    ros::spinOnce();
  }
  ROS_INFO("Connected to move base server, %s!", ns.c_str());
  ros::Subscriber getGoalSub = nh.subscribe("/goal", 100, &startCommandReceived);

  ros::Subscriber get_selfpaln_sub = nh.subscribe(ns + "/move_base/DWAPlannerROS/global_plan", 100, &getSelfGlobalPlan);
  ros::Subscriber get_leaderpaln_sub = nh.subscribe("/leader_plan", 100, getGlobalPlanOfLeader);
  g_sendplan_pub = nh.advertise<nav_msgs::Path>("/leader_plan", 100);

  g_clear_map_sc = nh.serviceClient<std_srvs::Empty>(ns + "/move_base/clear_costmaps");
  ros::ServiceClient get_position_sc = nh.serviceClient<formation::PositionRequest>(ns + "/position_requestion");

  ros::Timer timer = nh.createTimer(ros::Duration(5), clearCostMapTimer);

  ros::Duration(1).sleep();
  //request the position by argument
  get_position_sc.call(req_position, res_position);
  if(leader_id != res_position.leader_id)
  {
    startFormationFlag = false;
  }
  leader_id = res_position.leader_id;
  leader_id_stream.str("");
  leader_id_stream<<leader_id;
  if(ns == "/robot" + leader_id_stream.str())
  {
    g_isleader = true;
  }
  else
  {
    g_isleader = false;
  }

  ros::Duration(1).sleep();
  ROS_INFO("Start formation now, %s!", ns.c_str());
  while (ros::ok())
  {
    if(!get_position_sc.call(req_position, res_position))
    {
      continue;
    }
    position = res_position.position;
    if(leader_id != res_position.leader_id)
    {
      startFormationFlag = false;
    }
    leader_id = res_position.leader_id;
    leader_id_stream.str("");
    leader_id_stream<<leader_id;
    if(ns == "/robot" + leader_id_stream.str())
    {
      g_isleader = true;
    }
    else
    {
      g_isleader = false;
    }

    if(!g_isleader && startFormationFlag)
    {
      //robot n get the desired position depend on leader.
      geometry_msgs::PoseStamped desiredPosition;
      leaderRerence.getDesiredPosition(desiredPosition, position , leader_id, globalPlanPose);
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header = desiredPosition.header;
      goal.target_pose.pose = desiredPosition.pose;

      ROS_INFO("Send goal");
      g_ac->sendGoal(goal);
      bool finish_within_time = g_ac->waitForResult(ros::Duration(1));

      if(!finish_within_time)
      {
        g_clear_map_sc.call(req_clearCostmap, res_clearCostmap);
        g_ac->cancelGoal();
        ROS_INFO("Time out achieving goal.");
      }
      else
      {
        if(g_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Goal succeeded!");
        }
        else
        {
          ROS_INFO("The base failed for some reason.");
        }
      }
      ROS_INFO("g_ac.getState().getText(): %s", g_ac->getState().toString().c_str());
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  g_ac->~SimpleActionClient();
  return 0;
}
