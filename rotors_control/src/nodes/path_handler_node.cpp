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

    double vecLen = vec.length();
    vec.normalize();
    vec *= 0.0 * std::min(1.0, vecLen);
    // printf("dist: %f\n", vec.length());

    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = vec.getX();
    vel.twist.linear.y = vec.getY();
    vel.twist.linear.z = vec.getZ();


    geometry_msgs::PoseStamped increased_distance_pos;
    increased_distance_pos.pose.position.x = last_msg.pose.position.x + vec.getX();
    increased_distance_pos.pose.position.y = last_msg.pose.position.y + vec.getY();
    increased_distance_pos.pose.position.z = last_msg.pose.position.z + vec.getZ();
    // increased_distance_pos.pose.orientation = last_msg.pose.orientation;
    increased_distance_pos.pose.orientation.x = last_msg.pose.orientation.y;
    increased_distance_pos.pose.orientation.y = last_msg.pose.orientation.z;
    increased_distance_pos.pose.orientation.z = last_msg.pose.orientation.w;
    increased_distance_pos.pose.orientation.w = last_msg.pose.orientation.x;
    mavros_waypoint_publisher.publish(increased_distance_pos);
    


    auto q0 = increased_distance_pos.pose.orientation.w;
    auto q1 = increased_distance_pos.pose.orientation.x;
    auto q2 = increased_distance_pos.pose.orientation.y;
    auto q3 = increased_distance_pos.pose.orientation.z;
    // printf("    yaw: %f \n", std::atan2(2. * (q0*q3 + q1*q2), 1. - 2. * (q2*q2 + q3*q3)));
    

    // increased_distance_pos.pose.position.x = 0;
    // increased_distance_pos.pose.position.y = 0;
    // increased_distance_pos.pose.position.z = 0;
    // mavros_attitude_publisher.publish(increased_distance_pos);

    // mavros_velocity_publisher.publish(vel);
  }
}

PathHandlerNode::~PathHandlerNode() { }


void PathHandlerNode::ReceiveMessage(const geometry_msgs::PoseStamped& pose_msg) {

  // Not in use
  last_msg = pose_msg;
}

void PathHandlerNode::ReceivePath(const nav_msgs::Path& msg) {

  path.clear();
  for (auto p : msg.poses) {
    path.push_back(p);
  }
  last_msg = path[0];
}

void PathHandlerNode::PositionCallback(
    const geometry_msgs::PoseStamped& pose_msg) {

  last_pos = pose_msg;
  // Check if we are close enough to current goal to get the next part of the path
  if (path.size() > 0 && abs(last_msg.pose.position.x - pose_msg.pose.position.x) < 1 
                      && abs(last_msg.pose.position.y - pose_msg.pose.position.y) < 1
                      && abs(last_msg.pose.position.z - pose_msg.pose.position.z) < 1) {

    // Pop the first point of the path
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
