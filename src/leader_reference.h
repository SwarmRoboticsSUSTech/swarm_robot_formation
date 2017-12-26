#ifndef LEADER_RERENCE_H_
#define LEADER_RERENCE_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace formation {

/**
 */
enum Formation { Line, Wedge, Column, Diamond};

/**
 */
class  LeaderRerence {
public:
  LeaderRerence(tf::TransformListener* tf, Formation pF, double pDS);
  ~LeaderRerence() {}
  bool getDesiredPosition(geometry_msgs::PoseStamped& desiredPosition, int position,
                          int leader_id, geometry_msgs::PoseStamped globalPlanPose);

protected:
  Formation f;
  double desiredSpaceing;

private:
  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose, int leader_id) const;
  bool tramformToOdom(tf::Stamped<tf::Pose>& orign_pose, geometry_msgs::PoseStamped& output_pose) const;
  bool transformToMap(geometry_msgs::PoseStamped& globalPlanPose, tf::Stamped<tf::Pose>& output_pose_map);
  tf::TransformListener* tfListener_;
  ros::NodeHandle nh_;
  std::string mNameSpace_;
};

} /* namespace formation */
#endif /* LEADER_RERENCE_H_ */
