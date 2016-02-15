#include <ros/ros.h>

#include "path_handler_node.h"

namespace rotors_control {

PathHandlerNode::PathHandlerNode() {

  ros::NodeHandle nh;

  // cmd_trajectory_sub_ = nh.subscribe("/path_setpoint", 1, &PathHandlerNode::ReceiveMessage, this);
  cmd_trajectory_sub_ = nh.subscribe("/global_path", 1, &PathHandlerNode::ReceivePath, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PathHandlerNode::PositionCallback, this);

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

PathHandlerNode::~PathHandlerNode() { }


void PathHandlerNode::ReceiveMessage(
    const geometry_msgs::PoseStamped& pose_msg) {

  // geometry_msgs::PoseStamped pose_msg;
  // pose_msg.header.frame_id="/world";
  // pose_msg.pose.position.x = point_msg.point.x;
  // pose_msg.pose.position.y = point_msg.point.y;
  // pose_msg.pose.position.z = 2;

  last_msg = pose_msg;
}

void PathHandlerNode::ReceivePath(
    const nav_msgs::Path& msg) {

  path.clear();
  for (auto p : msg.poses) {
    path.push_back(p);
  }
}

void PathHandlerNode::PositionCallback(
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
  ros::init(argc, argv, "path_handler_node");

  rotors_control::PathHandlerNode path_handler_node;

  ros::spin();

  return 0;
}
