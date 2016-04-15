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
#include <math.h>           // abs

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "rotors_control/common.h"
#include "rotors_control/global_planner.h"

namespace rotors_control {

class GlobalPlannerNode {
 public:
  std::vector<Cell> fileGoals;
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  GlobalPlanner global_planner;
  WaypointWithTime goalCell;
  geometry_msgs::Point goalPoint;
  std::string namespace_;
  nav_msgs::Path actualPath;

  int numOctomapMessages = 0;
  int numPositionMessages = 0;



  // Subscribers
  ros::Subscriber cmd_waypoint_sub_;
  ros::Subscriber cmd_octomap_sub_;
  ros::Subscriber cmd_octomap_full_sub_;
  ros::Subscriber cmd_ground_truth_sub_;
  ros::Subscriber cmd_clicked_point_sub_;
  ros::Subscriber laser_sensor_sub_;

  // Publishers
  // ros::Publisher cmd_multi_dof_joint_trajectory_pub_;
  ros::Publisher cmd_global_path_pub_;
  ros::Publisher cmd_actual_path_pub_;
  ros::Publisher cmd_explored_cells_pub_;

  // // lee_controler_publisher
  // ros::Publisher wp_pub;
  // // Mavros publisher
  // ros::Publisher mavros_waypoint_publisher;
  // // path_handler publisher
  // ros::Publisher path_handler_publisher;

  void SetNewGoal(Cell goal);
  void PositionCallback(const geometry_msgs::PoseStamped& msg);
  void ClickedPointCallback(const geometry_msgs::PointStamped& msg);
  void LaserSensorCallback(const sensor_msgs::LaserScan& msg);
  void OctomapCallback(const visualization_msgs::MarkerArray& msg);
  void OctomapFullCallback(const octomap_msgs::Octomap& msg);

  void PlanPath();

  void PublishPath();
  void PublishExploredCells();

};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_NODE_H
