#include <ros/ros.h>

#include "path_handler_node.h"

namespace rotors_control {

PathHandlerNode::PathHandlerNode() {

  ros::NodeHandle nh;

  // cmd_trajectory_sub_ = nh.subscribe("/path_setpoint", 1, &PathHandlerNode::ReceiveMessage, this);
  cmd_trajectory_sub_ = nh.subscribe("/global_path", 1, &PathHandlerNode::ReceivePath, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PathHandlerNode::PositionCallback, this);

  mavros_waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);

  last_msg.header.frame_id="/world";
  last_msg.pose.position.x = 0;
  last_msg.pose.position.y = 0;
  last_msg.pose.position.z = 2;

  // -45 deg
  // last_msg.pose.orientation.x = -0.9238795325112867;
  // last_msg.pose.orientation.y = 0.0;
  // last_msg.pose.orientation.z = 0.0;
  // last_msg.pose.orientation.w = -0.3826834323650897;

  // -135 deg
  last_msg.pose.orientation.x = -0.9238795325112867;
  last_msg.pose.orientation.y = 0.0;
  last_msg.pose.orientation.z = 0.0;
  last_msg.pose.orientation.w = 0.3826834323650897;
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
    vec *= 2.0 * std::min(0.5, vecLen);
    // // printf("dist: %f\n", vec.length());

    // Fix the order of the quaternion coordinates 
    auto rot_msg = last_msg;
    rot_msg.pose.position.x = last_msg.pose.position.x + vec.getX();
    rot_msg.pose.position.y = last_msg.pose.position.y + vec.getY();
    rot_msg.pose.position.z = last_msg.pose.position.z + vec.getZ();
    rot_msg.pose.orientation.x = last_msg.pose.orientation.y;
    rot_msg.pose.orientation.y = last_msg.pose.orientation.z;
    rot_msg.pose.orientation.z = last_msg.pose.orientation.w;
    rot_msg.pose.orientation.w = last_msg.pose.orientation.x;

    // Publish setpoint for vizualization
    current_waypoint_publisher.publish(rot_msg);

    // 90 deg fix
    rot_msg.pose.position.x = -(last_msg.pose.position.y);
    rot_msg.pose.position.y = (last_msg.pose.position.x);

    // Publish setpoint to Mavros
    mavros_waypoint_publisher.publish(rot_msg);



    // auto q0 = increased_distance_pos.pose.orientation.w;
    // auto q1 = increased_distance_pos.pose.orientation.x;
    // auto q2 = increased_distance_pos.pose.orientation.y;
    // auto q3 = increased_distance_pos.pose.orientation.z;
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
  if (path.size() > 0) {
    last_msg = path[0];
  }
  else {
    printf("  Received empty path\n");
  }
}

void PathHandlerNode::PositionCallback(
    const geometry_msgs::PoseStamped& pose_msg) {

  last_pos = pose_msg;
    // 90 deg fix
  last_pos.pose.position.x = (pose_msg.pose.position.y);
  last_pos.pose.position.y = -(pose_msg.pose.position.x);
  // Check if we are close enough to current goal to get the next part of the path
  if (path.size() > 0 && abs(last_msg.pose.position.x - last_pos.pose.position.x) < 1 
                      && abs(last_msg.pose.position.y - last_pos.pose.position.y) < 1
                      && abs(last_msg.pose.position.z - last_pos.pose.position.z) < 1) {

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
