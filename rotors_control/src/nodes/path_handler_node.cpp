#include <ros/ros.h>

#include "path_handler_node.h"

namespace rotors_control {

PathHandlerNode::PathHandlerNode() {

  ros::NodeHandle nh;

  // cmd_trajectory_sub_ = nh.subscribe("/path_setpoint", 1, &PathHandlerNode::ReceiveMessage, this);
  cmd_trajectory_sub_ = nh.subscribe("/global_path", 1, &PathHandlerNode::ReceivePath, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PathHandlerNode::PositionCallback, this);
  // cmd_ground_truth_sub_ = nh.subscribe("/iris/ground_truth/pose", 1, &PathHandlerNode::PositionCallback, this);

  mavros_waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);

  last_msg.header.frame_id="/world";
  last_msg.pose.position.x = 1.5;
  last_msg.pose.position.y = 0.5;
  last_msg.pose.position.z = 2.5;

  last_msg.pose.orientation.x = 1.0;
  last_msg.pose.orientation.y = 0.0;
  last_msg.pose.orientation.z = 0.0;
  last_msg.pose.orientation.w = 0.0;
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
    double newLen = vec.length() < 0.5? 0.0 : 1.0;
    vec.normalize();
    vec *= newLen;

    // Fix the order of the quaternion coordinates 
    auto rot_msg = last_msg;
    rot_msg.pose.position.x = last_msg.pose.position.x + vec.getX();
    rot_msg.pose.position.y = last_msg.pose.position.y + vec.getY();
    rot_msg.pose.position.z = last_msg.pose.position.z + vec.getZ();

    // Publish setpoint for vizualization
    current_waypoint_publisher.publish(rot_msg);

    // 90 deg fix
    // rot_msg.pose.position.x = -(last_msg.pose.position.y);    // TODO: why not last_msg?
    // rot_msg.pose.position.y = (last_msg.pose.position.x);

    // Publish setpoint to Mavros
    mavros_waypoint_publisher.publish(rot_msg);
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
  // last_pos.pose.position.x = (pose_msg.pose.position.y);
  // last_pos.pose.position.y = -(pose_msg.pose.position.x);
  // Check if we are close enough to current goal to get the next part of the path
  if (path.size() > 0 && std::abs(last_msg.pose.position.x - last_pos.pose.position.x) < 1 
                      && std::abs(last_msg.pose.position.y - last_pos.pose.position.y) < 1
                      && std::abs(last_msg.pose.position.z - last_pos.pose.position.z) < 1) {

    // double yaw1 = tf::getYaw(last_msg.pose.orientation) + M_PI/4; // 90 deg fix
    double yaw1 = tf::getYaw(last_msg.pose.orientation);
    double yaw2 = tf::getYaw(last_pos.pose.orientation);
    double yawDiff = std::abs(yaw2 - yaw1);
    yawDiff -= std::floor(yawDiff / (2*M_PI)) * (2*M_PI);
    double maxYawDiff = M_PI/4.0;
    // printf("last_msg: %.3f,  last_pos: %.3f,  yawDiff: %.3f \n", yaw1, yaw2, yawDiff);
    if (yawDiff < maxYawDiff || yawDiff  > 2*M_PI - maxYawDiff){
      // if we are also facing forward, then pop the first point of the path
      last_msg = path[0];
      path.erase(path.begin());
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
