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

#ifndef ROTORS_CONTROL_GLOBAL_PLANNER_NODE_H
#define ROTORS_CONTROL_GLOBAL_PLANNER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <set>

#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "rotors_control/common.h"
#include "rotors_control/global_planner.h"

namespace rotors_control {

class GlobalPlannerNode {
 public:
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  GlobalPlanner global_planner;
  WaypointWithTime goalCell;
  geometry_msgs::Point goalPoint;
  std::string namespace_;


  // Subscribers
  ros::Subscriber cmd_waypoint_sub_;
  ros::Subscriber cmd_octomap_sub_;
  ros::Subscriber cmd_octomap_full_sub_;
  ros::Subscriber cmd_ground_truth_sub_;
  ros::Subscriber cmd_clicked_point_sub_;

  // Publishers
  ros::Publisher cmd_multi_dof_joint_trajectory_pub_;
  ros::Publisher cmd_global_path_pub_;

  void PositionCallback(
      const geometry_msgs::PoseStamped& msg);


  void OctomapCallback(
      const visualization_msgs::MarkerArray& msg);

  void OctomapFullCallback(
      const octomap_msgs::Octomap& msg);

  void PlanPathCallback();

  void PublishPath();

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

  void ClickedPointCallback(
      const geometry_msgs::PointStamped& msg);




};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_NODE_H
