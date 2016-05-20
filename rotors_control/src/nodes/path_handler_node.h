#ifndef ROTORS_CONTROL_PATH_HANDLER_NODE_H
#define ROTORS_CONTROL_PATH_HANDLER_NODE_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>  // getYaw

#include "rotors_control/common.h" // hasSameYawAndAltitude


// #include <math.h>           

namespace rotors_control {

class PathHandlerNode {
 public:
  PathHandlerNode();
  ~PathHandlerNode();

 private:

  geometry_msgs::PoseStamped last_msg;
  geometry_msgs::PoseStamped last_pos;
  double speed = 1.0;
  double maxSpeed = 1.5;

  std::vector<geometry_msgs::PoseStamped> path;

  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_ground_truth_sub_;

  ros::Publisher mavros_waypoint_publisher;
  ros::Publisher current_waypoint_publisher;

  void ReceiveMessage(const geometry_msgs::PoseStamped& pose_msg);
  void ReceivePath(const nav_msgs::Path& msg);
  void PositionCallback(const geometry_msgs::PoseStamped& pose_msg);

};
}

#endif // ROTORS_CONTROL_PATH_HANDLER_NODE_H
