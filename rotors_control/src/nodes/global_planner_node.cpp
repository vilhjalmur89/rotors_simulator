/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "global_planner_node.h"

#include "rotors_control/parameters_ros.h"


 #include <tf/transform_listener.h> // getYaw
 #include <nav_msgs/Path.h>
 #include <geometry_msgs/Quaternion.h>



namespace rotors_control {


void getWaypointsFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg, std::vector<WaypointWithTime> & waypoints) {

  for (int i=0; i < msg->points.size(); ++i) {
    mav_msgs::EigenTrajectoryPoint p;
    mav_msgs::eigenTrajectoryPointFromMsg(msg->points[i], &p);
    waypoints.push_back(WaypointWithTime(p.time_from_start_ns / 100, p.position_W[0], p.position_W[1], p.position_W[2], p.getYaw()));
  }
}

GlobalPlannerNode::GlobalPlannerNode() {
  ros::NodeHandle nh;

  cmd_waypoint_sub_ = nh.subscribe(
      "GlobalPlannerTopic", 1,
      &GlobalPlannerNode::MultiDofJointTrajectoryCallback, this);

  cmd_octomap_sub_ = nh.subscribe(
      "/occupied_cells_vis_array", 1,
      &GlobalPlannerNode::OctomapCallback, this);

  cmd_ground_truth_sub_ = nh.subscribe(
      "ground_truth/pose", 1,
      &GlobalPlannerNode::PositionCallback, this);

  cmd_clicked_point_sub_ = nh.subscribe(
      "/clicked_point", 1,
      &GlobalPlannerNode::ClickedPointCallback, this);

  cmd_multi_dof_joint_trajectory_pub_ = 
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  cmd_global_path_pub_ = 
      nh.advertise<nav_msgs::Path>(
      "global_path", 10);
}

GlobalPlannerNode::~GlobalPlannerNode() { }

void GlobalPlannerNode::Publish() {
}

void GlobalPlannerNode::PositionCallback(
    const geometry_msgs::PoseStamped& msg) {

  global_planner.position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  global_planner.yaw = tf::getYaw(msg.pose.orientation);
}

void GlobalPlannerNode::ClickedPointCallback(
    const geometry_msgs::PointStamped& msg) {

  goalCell = WaypointWithTime(0, msg.point.x, msg.point.y, msg.point.z, 0);
  waypoints.resize(0);
  waypoints.push_back(goalCell);
  PlanPathCallback();
}

void GlobalPlannerNode::OctomapCallback(
    const visualization_msgs::MarkerArray& msg) {

  global_planner.occupied.clear();
  bool pathIsBad = false;
  for (auto point : msg.markers[msg.markers.size()-1].points) {
    std::pair<int,int> cell(point.x, point.y);
    global_planner.occupied.insert(cell);
    if (global_planner.pathCells.find(cell) != global_planner.pathCells.end()) {
      pathIsBad = true;
    }
  }
  if (pathIsBad) {
    ROS_INFO("  Path is bad, planning a new path");
    waypoints.resize(0);
    waypoints.push_back(goalCell);
    PlanPathCallback();
  }
}

void GlobalPlannerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

  ROS_INFO("   Current Position: %f, %f", global_planner.position[0], global_planner.position[1]);
  getWaypointsFromMsg(msg, waypoints);
  goalCell = waypoints[0];
  PlanPathCallback();
}

void GlobalPlannerNode::PlanPathCallback() {
  global_planner.getGlobalPath(waypoints);

  ROS_INFO("Start publishing endpoints.");

  trajectory_msgs::MultiDOFJointTrajectory newMsg;
  newMsg.header.stamp = ros::Time::now();
  newMsg.points.resize(waypoints.size());
  newMsg.joint_names.push_back("base_link");

  nav_msgs::Path path;
  path.header.frame_id="/world";



  int64_t time_from_start_ns = 0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &newMsg.points[i]);
    
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id="/world";
    poseMsg.pose.position.x = wp.position[0];
    poseMsg.pose.position.y = wp.position[1];
    poseMsg.pose.position.z = wp.position[2];
    path.poses.push_back(poseMsg);
  }

  cmd_multi_dof_joint_trajectory_pub_.publish(newMsg);
  cmd_global_path_pub_.publish(path);
  printf("\n    Published Full Path \n");

}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");

  rotors_control::GlobalPlannerNode global_planner_node;

  ros::spin();

  return 0;
}
