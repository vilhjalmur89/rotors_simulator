#include <ros/ros.h>

#include "path_handler_node.h"

namespace rotors_control {

PathHandlerNode::PathHandlerNode() {

  ros::NodeHandle nh;

  // cmd_trajectory_sub_ = nh.subscribe("/path_setpoint", 1, &PathHandlerNode::ReceiveMessage, this);
  cmd_trajectory_sub_ = nh.subscribe("/global_path", 1, &PathHandlerNode::ReceivePath, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PathHandlerNode::PositionCallback, this);

  mavros_waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  mavros_attitude_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);
  mavros_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

  last_msg.header.frame_id="/world";
  last_msg.pose.position.x = 0;
  last_msg.pose.position.y = 0;
  last_msg.pose.position.z = 2;
  last_pos = last_msg;

  ros::Rate r(10);
  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();

    auto x = last_msg.pose.position.x - last_pos.pose.position.x;
    auto y = last_msg.pose.position.y - last_pos.pose.position.y;
    auto z = last_msg.pose.position.z - last_pos.pose.position.z;
    tf::Vector3 vec(x,y,z);
    // vec.normalize();
    vec *= 2.0;

    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = vec.getX();
    vel.twist.linear.y = vec.getY();
    vel.twist.linear.z = vec.getZ();


    geometry_msgs::PoseStamped increased_distance_pos;
    increased_distance_pos.pose.position.x = last_msg.pose.position.x + vec.getX();
    increased_distance_pos.pose.position.y = last_msg.pose.position.y + vec.getY();
    increased_distance_pos.pose.position.z = last_msg.pose.position.z + vec.getZ();
    mavros_waypoint_publisher.publish(increased_distance_pos);
    

    increased_distance_pos.pose.orientation = last_msg.pose.orientation;
    increased_distance_pos.pose.position.x = 0;
    increased_distance_pos.pose.position.y = 0;
    increased_distance_pos.pose.position.z = 0;
    mavros_attitude_publisher.publish(increased_distance_pos);

    // mavros_velocity_publisher.publish(vel);
  }
}

PathHandlerNode::~PathHandlerNode() { }


void PathHandlerNode::ReceiveMessage(
    const geometry_msgs::PoseStamped& pose_msg) {

  // Not in use
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

  last_pos = pose_msg;
  if (path.size() > 0 && abs(path[0].pose.position.x - pose_msg.pose.position.x) < 1 
                      && abs(path[0].pose.position.y - pose_msg.pose.position.y) < 1
                      && abs(path[0].pose.position.z - pose_msg.pose.position.z) < 1) {

    last_msg = path[0];
    path.erase(path.begin());
  }
}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_handler_node");

  rotors_control::PathHandlerNode path_handler_node;

  ros::spin();

  return 0;
}
