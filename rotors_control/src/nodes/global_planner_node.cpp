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
// #include <mav_msgs/default_topics.h>

#include "global_planner_node.h"

#include "rotors_control/parameters_ros.h"

#include <tf/transform_listener.h> // getYaw createQuaternionMsgFromYaw 
#include <geometry_msgs/Quaternion.h>
#include <math.h> // floor

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using mavros::UAS;

namespace rotors_control {



GlobalPlannerNode::GlobalPlannerNode() {
  ros::NodeHandle nh;

  cmd_octomap_full_sub_ = nh.subscribe("/octomap_full", 1, &GlobalPlannerNode::OctomapFullCallback, this);
  cmd_ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1,&GlobalPlannerNode::PositionCallback, this);
  cmd_clicked_point_sub_ = nh.subscribe("/clicked_point", 1,&GlobalPlannerNode::ClickedPointCallback, this);

  cmd_global_path_pub_ = nh.advertise<nav_msgs::Path>("/global_path", 10);
  cmd_explored_cells_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/explored_cells", 10);
}

GlobalPlannerNode::~GlobalPlannerNode() { }

void GlobalPlannerNode::SetNewGoal(Cell goal) {
  ROS_INFO("========== Set goal : %s ==========", goal.asString().c_str());
  global_planner.setGoal(goal);
  PlanPath();
}

void GlobalPlannerNode::PositionCallback(
    const geometry_msgs::PoseStamped& msg) {

  auto rot_msg = msg;
  double yaw = tf::getYaw(rot_msg.pose.orientation);

  // 90 deg fix
  rot_msg.pose.position.x = (msg.pose.position.y);
  rot_msg.pose.position.y = -(msg.pose.position.x);
  yaw -= M_PI/2;

  global_planner.setPose(rot_msg.pose.position, yaw);    // TODO: call with just pose

  double distToGoal = global_planner.goalPos.manhattanDist(global_planner.currPos.x, global_planner.currPos.y, global_planner.currPos.z);
  if (fileGoals.size() > 0 && (distToGoal < 1.5 || global_planner.goalIsBlocked)) {
    // If there is another goal and we are either at current goal or it is blocked, we set a new goal
    Cell newGoal = fileGoals[0];
    fileGoals.erase(fileGoals.begin());
    ROS_INFO("Reached current goal, %d goals left\n\n", (int) fileGoals.size());
    SetNewGoal(newGoal);
  }
}

void GlobalPlannerNode::ClickedPointCallback(
    const geometry_msgs::PointStamped& msg) {

  SetNewGoal(Cell(msg.point.x, msg.point.y, 2.5));
}

void GlobalPlannerNode::OctomapFullCallback(
    const octomap_msgs::Octomap& msg) {

  if (global_planner.numOctomapMessages++ % 10 > 0) {
    return;     // We get too many of those messages. Only process 1/10 of them
  }

  if (!global_planner.updateFullOctomap(msg)) {
    // Part of the current path is blocked
    ROS_INFO("  Path is bad, planning a new path \n");
    global_planner.truncatePath();             // Cut off bad part of path
    // TODO: Decide whether to truncate path or not
    // Cell tmp = global_planner.goalPos;
    // global_planner.goalPos = Cell(global_planner.currPos);
    // PublishPath();      
    // global_planner.goalPos = tmp;             // Publish cut-off path
    PlanPath();                               // Plan a whole new path
  }
}

void GlobalPlannerNode::PlanPath() {
  ROS_INFO("Start planning path.");
  bool foundPath = global_planner.getGlobalPath();
  PublishExploredCells();
  if (!foundPath) {
    ROS_INFO("Failed to find a path");
    return;
  }
  PublishPath();
}

void GlobalPlannerNode::PublishPath() {
  nav_msgs::Path path;
  path.header.frame_id="/world";

  for (WaypointWithTime wp : global_planner.waypoints) {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id = "/world";
    poseMsg.pose.position.x = wp.position[0];
    poseMsg.pose.position.y = wp.position[1];
    poseMsg.pose.position.z = wp.position[2];
    poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(wp.yaw); 
    poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw( (wp.yaw + 3.1415/2.0));  // 90 deg fix
    path.poses.push_back(poseMsg);
  }
  cmd_global_path_pub_.publish(path);
}

void GlobalPlannerNode::PublishExploredCells() {
  // Publish the cells that were explored in the last search
  visualization_msgs::MarkerArray msg;

  // The first marker deletes the ones from previous search
  int id = 0;
  visualization_msgs::Marker marker;
  marker.id = id;
  marker.action = 3;        // same as visualization_msgs::Marker::DELETEALL
  msg.markers.push_back(marker);
  
  id = 1;
  std::set<Cell>::iterator it;
  for (it = global_planner.seen.begin(); it != global_planner.seen.end(); ++it, ++id) {
    visualization_msgs::Marker marker;
    marker.id = id;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = marker.scale.x;
    marker.scale.z = marker.scale.x;
    marker.pose.position.x = it->x() + 0.5;
    marker.pose.position.y = it->y() + 0.5;
    marker.pose.position.z = it->z() + 0.5;

    // Just a hack to get the (almost) color spectrum depending on height
    // h=1 -> blue    h=3 -> green  h=5 -> red
    double h = (it->z()-1.0) / 5.0;
    marker.color.r = h;
    marker.color.g = 1.0 - 2.0 * std::abs(h - 0.5);
    marker.color.b = 1.0 - h;
    marker.color.a = 1.0;

    msg.markers.push_back(marker);
  }
  cmd_explored_cells_pub_.publish(msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");
  rotors_control::GlobalPlannerNode global_planner_node;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() > 1) {
    ROS_INFO("    ARGS: %s", args.at(1).c_str());
    std::ifstream wp_file(args.at(1).c_str());
    if (wp_file.is_open()) {
      double x, y, z;
      // Only read complete waypoints.
      while (wp_file >> x >> y >> z) {
        global_planner_node.fileGoals.push_back(rotors_control::Cell(x, y, z));
      }
      wp_file.close();
      ROS_INFO("  Read %d waypoints.", global_planner_node.fileGoals.size());
    }
    else {
      ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
      return -1;
    }
  }
  else {
    ROS_INFO("  No goal file given.");
  }

  ros::spin();

  return 0;
}
