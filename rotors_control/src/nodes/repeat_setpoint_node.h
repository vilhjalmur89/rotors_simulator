#ifndef ROTORS_CONTROL_REPEAT_SETPOINT_NODE_H
#define ROTORS_CONTROL_REPEAT_SETPOINT_NODE_H

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Path.h>

namespace rotors_control {

class RepeatSetpointNode {
 public:
  RepeatSetpointNode();
  ~RepeatSetpointNode();

 private:

  geometry_msgs::PoseStamped last_msg;

  std::vector<geometry_msgs::PoseStamped> path;

  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_ground_truth_sub_;

  ros::Publisher mavros_waypoint_publisher;

  void ReceiveMessage(const geometry_msgs::PoseStamped& pose_msg);
  void ReceivePath(const nav_msgs::Path& msg);
  void PositionCallback(const geometry_msgs::PoseStamped& pose_msg);

};
}

#endif // ROTORS_CONTROL_REPEAT_SETPOINT_NODE_H
