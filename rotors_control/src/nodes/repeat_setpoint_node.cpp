#include <ros/ros.h>

#include "repeat_setpoint_node.h"

namespace rotors_control {

RepeatSetpointNode::RepeatSetpointNode() {

  ros::NodeHandle nh;

  // cmd_trajectory_sub_ = nh.subscribe("/repeat_setpoint", 1, &RepeatSetpointNode::ReceiveMessage, this);
  cmd_trajectory_sub_ = nh.subscribe("/global_path", 1, &RepeatSetpointNode::ReceivePath, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &RepeatSetpointNode::PositionCallback, this);

  mavros_waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

  last_msg.header.frame_id="/world";
  last_msg.pose.position.x = 0;
  last_msg.pose.position.y = 0;
  last_msg.pose.position.z = 2;

  ros::Rate r(100);
  while(ros::ok())
  {
     r.sleep();
     ros::spinOnce();
     mavros_waypoint_publisher.publish(last_msg);
  }
}

RepeatSetpointNode::~RepeatSetpointNode() { }


void RepeatSetpointNode::ReceiveMessage(
    const geometry_msgs::PoseStamped& pose_msg) {

  // geometry_msgs::PoseStamped pose_msg;
  // pose_msg.header.frame_id="/world";
  // pose_msg.pose.position.x = point_msg.point.x;
  // pose_msg.pose.position.y = point_msg.point.y;
  // pose_msg.pose.position.z = 2;

  last_msg = pose_msg;
}

void RepeatSetpointNode::ReceivePath(
    const nav_msgs::Path& msg) {

  path.clear();
  for (auto p : msg.poses) {
    path.push_back(p);
  }
}

void RepeatSetpointNode::PositionCallback(
    const geometry_msgs::PoseStamped& pose_msg) {

  if (path.size() > 0 && abs(path[0].pose.position.x - pose_msg.pose.position.x) < 2 
                      && abs(path[0].pose.position.y - pose_msg.pose.position.y) < 2) {

    path.erase(path.begin());

    if (path.size() > 0) {
      last_msg = path[0];
    }
  }
}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "repeat_setpoint_node");

  rotors_control::RepeatSetpointNode repeat_setpoint_node;

  ros::spin();

  return 0;
}
