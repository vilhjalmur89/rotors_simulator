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
#include <math.h> // floor


namespace rotors_control {



GlobalPlannerNode::GlobalPlannerNode() {
  ros::NodeHandle nh;

  cmd_octomap_sub_ = nh.subscribe(
      "/occupied_cells_vis_array", 1,
      &GlobalPlannerNode::OctomapCallback, this);

  cmd_octomap_full_sub_ = nh.subscribe(
      "/octomap_full", 1,
      &GlobalPlannerNode::OctomapFullCallback, this);

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


void GlobalPlannerNode::PositionCallback(
    const geometry_msgs::PoseStamped& msg) {

  global_planner.setPose(msg.pose.position, tf::getYaw(msg.pose.orientation));
}

void GlobalPlannerNode::ClickedPointCallback(
    const geometry_msgs::PointStamped& msg) {

  global_planner.goalPos = Cell(msg.point);
  PlanPathCallback();
}

void GlobalPlannerNode::OctomapFullCallback(
    const octomap_msgs::Octomap& msg) {

  if (global_planner.updateFullOctomap(msg)) {
    ROS_INFO("  Path is bad, planning a new path");
    global_planner.truncatePath();             // Cut off bad part of path
    Cell tmp = global_planner.goalPos;
    global_planner.goalPos = Cell(global_planner.currPos);
    PublishPath();      
    global_planner.goalPos = tmp;              // Publish cut-off path
    PlanPathCallback();                       // Plan a whole new path
  }

}

void GlobalPlannerNode::OctomapCallback(
    const visualization_msgs::MarkerArray& msg) {

  // if (global_planner.updateOctomap(msg)) {
  //   ROS_INFO("  Path is bad, planning a new path");
  //   global_planner.truncatePath();    // Cut off bad part of path
  //   Cell tmp = global_planner.goalPos;
  //   global_planner.goalPos = Cell(global_planner.currPos);
  //   PublishPath();      
  //   global_planner.goalPos = tmp;              // Publish cut-off path
  //   PlanPathCallback();               // Plan a whole new path
  // }
}

void GlobalPlannerNode::PlanPathCallback() {
  ROS_INFO("Start planning path.");
  if (!global_planner.getGlobalPath()) {
    ROS_INFO("Failed to find a path");
    return;
  }
  PublishPath();
}

void GlobalPlannerNode::PublishPath() {
  trajectory_msgs::MultiDOFJointTrajectory newMsg;
  newMsg.header.stamp = ros::Time::now();
  newMsg.joint_names.push_back("base_link");

  nav_msgs::Path path;
  path.header.frame_id="/world";

  int64_t time_from_start_ns = 0;
  for (WaypointWithTime wp : global_planner.waypoints) {

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time);

    trajectory_msgs::MultiDOFJointTrajectoryPoint p;
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &p);
    newMsg.points.push_back(p);
    
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
