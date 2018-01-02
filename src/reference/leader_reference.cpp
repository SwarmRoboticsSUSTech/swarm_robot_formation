#include "formation/reference/leader_reference.h"
#include <stdexcept>
#include <math.h>
#include <sstream>

using namespace formation;

LeaderRerence::LeaderRerence(tf::TransformListener* tf, Formation pF, double pDS)
{
  tfListener_ = tf;
  f = pF;
  desiredSpaceing = pDS;
  mNameSpace_ = nh_.getNamespace();
}

bool LeaderRerence::getRobotPose(tf::Stamped<tf::Pose>& global_pose, int leader_id) const
{
  std::string global_frame = "/map";
  std::string leader_base_frame;
  std::stringstream leader_id_stream;
  leader_id_stream<<leader_id;
  leader_base_frame = "/robot" + leader_id_stream.str() + "_base_footprint";
  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = leader_base_frame;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
//    ROS_INFO(" getting robot pose");
    tfListener_->transformPose(global_frame, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::InvalidArgumentException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "InvalidArgumentException: %s\n", ex.what());
    return false;
  }

  double transform_tolerance = 0.1;
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance);
    return false;
  }
//  ROS_INFO("global_pose.frame_id_: %s", global_pose.frame_id_.c_str());
  return true;
}

bool LeaderRerence::tramformToOdom(tf::Stamped<tf::Pose>& orign_pose, geometry_msgs::PoseStamped& output_pose) const
{
  std::string output_frame = mNameSpace_.substr(1, mNameSpace_.length() - 1) + "_odom";
  tf::Stamped<tf::Pose> output_pose_odom;
  output_pose_odom.setIdentity();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
//    ROS_INFO(" getting robot pose");
    tfListener_->transformPose(output_frame, orign_pose, output_pose_odom);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::InvalidArgumentException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "InvalidArgumentException: %s\n", ex.what());
    return false;
  }

  double transform_tolerance = 0.1;
  // check global_pose timeout
  if (current_time.toSec() - output_pose_odom.stamp_.toSec() > transform_tolerance)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), output_pose_odom.stamp_.toSec(), transform_tolerance);
    return false;
  }

  tf::poseStampedTFToMsg(output_pose_odom, output_pose);
  return true;
}

bool LeaderRerence::transformToMap(geometry_msgs::PoseStamped& globalPlanPose, tf::Stamped<tf::Pose>& output_pose_map)
{
  tf::Stamped<tf::Pose> orign_pose;
  tf::poseStampedMsgToTF(globalPlanPose, orign_pose);

  std::string output_frame = "/map";
  output_pose_map.setIdentity();    // Identity is very importance for using to transform the orientation.Without it, we can not transform correctly. Remenber this!!!
//  orign_pose.setIdentity();       // Here, But we can't set orign_pose.setIdentity again!
  output_pose_map.stamp_ = ros::Time();  //Reset the time, so it can transform even this stamp is outtime.
  orign_pose.stamp_ = ros::Time();
  output_pose_map.frame_id_ = output_frame;

  try
  {
    tfListener_->transformPose(output_frame, orign_pose, output_pose_map);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::InvalidArgumentException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "InvalidArgumentException: %s\n", ex.what());
    return false;
  }
//  ROS_INFO("output_pose_map x:%f y:%f z:%f, x:%f y:%f z:%f w:%f", output_pose_map.getOrigin().getX(), output_pose_map.getOrigin().getY(),
//             output_pose_map.getOrigin().getZ(), output_pose_map.getRotation().getX(), output_pose_map.getRotation().getY(), output_pose_map.getRotation().getZ(),
//             output_pose_map.getRotation().getW());

  return true;
}

bool LeaderRerence::getDesiredPosition(geometry_msgs::PoseStamped& outputPosition, int position,
                                       int leader_id, geometry_msgs::PoseStamped globalPlanPose)
{
//  std::string mNameSpace = mNameSpace_.substr(2, mNameSpace_.length() - 2);
  tf::Stamped<tf::Pose> leaderPos;

  tf::Stamped<tf::Pose> poseToMap;
  transformToMap(globalPlanPose, poseToMap);
  double yawOfPath = tf::getYaw(poseToMap.getRotation());
  ROS_INFO("yawOfPath_map: %f", yawOfPath);

//  can't get the pose os leader
  if(!getRobotPose(leaderPos, leader_id))
  {
    ROS_WARN("Can not get DesiredPosition!");
    return false;
  }
  ROS_INFO("leaderPos x:%f y:%f z:%f, x:%f y:%f z:%f w:%f", leaderPos.getOrigin().getX(), leaderPos.getOrigin().getY(),
           leaderPos.getOrigin().getZ(), leaderPos.getRotation().getX(), leaderPos.getRotation().getY(), leaderPos.getRotation().getZ(),
           leaderPos.getRotation().getW());

  tf::Stamped<tf::Pose> desiredPosition(leaderPos);
  desiredPosition.setRotation(poseToMap.getRotation());

  desiredPosition.getOrigin().setY(leaderPos.getOrigin().getY() + position * desiredSpaceing * sin(yawOfPath - M_PI_2));
  desiredPosition.getOrigin().setX(leaderPos.getOrigin().getX() + position * desiredSpaceing * cos(yawOfPath - M_PI_2));

  ROS_INFO("desiredPosition x:%f y:%f z:%f, x:%f y:%f z:%f w:%f", desiredPosition.getOrigin().getX(), desiredPosition.getOrigin().getY(),
           desiredPosition.getOrigin().getZ(), desiredPosition.getRotation().getX(), desiredPosition.getRotation().getY(), desiredPosition.getRotation().getZ(),
           desiredPosition.getRotation().getW());
  tramformToOdom(desiredPosition, outputPosition);
  return true;
}

