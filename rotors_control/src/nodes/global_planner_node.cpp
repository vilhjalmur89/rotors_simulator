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

#include <queue>
#include <map>
#include <algorithm>    // std::reverse
 #include <tf/transform_listener.h> // getYaw



namespace rotors_control {

static const int64_t kNanoSecondsInSecond = 1000000000;

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

double angle(std::pair<int,int> pos, std::pair<int,int> p) {
  int dx = p.first - pos.first;
  int dy = p.second - pos.second;
  if (dy == 0) {
    return M_PI/2 - dx*(M_PI/2);
  }
  return dy*M_PI/2;
}


double between(double start, double end, int steps, int totalSteps) {
  return start + (end - start) * steps / totalSteps;
}

WaypointWithTime getPointBetween(const WaypointWithTime & a, const WaypointWithTime & b, 
                                 int step, int totalSteps, int minTime) {
  double t = minTime;
  double x = between(a.position[0], b.position[0], step, totalSteps);
  double y = between(a.position[1], b.position[1], step, totalSteps);
  double z = between(a.position[2], b.position[2], step, totalSteps);
  double yaw = between(a.yaw, b.yaw, step, totalSteps);
  return WaypointWithTime(t, x, y, z, yaw);
}

double squared(double x) {
  return x * x;
}

double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
  double xDiff = a[0] - b[0];
  double yDiff = a[1] - b[1];
  double zDiff = a[2] - b[2];
  return sqrt(squared(xDiff) + squared(yDiff) + squared(zDiff));
}

void increaseResolution(std::vector<WaypointWithTime> & waypoints, std::vector<WaypointWithTime> & newWaypoints, 
                        double minDist, double minRot, double minTime) {

  newWaypoints.push_back(waypoints[0]);
  for (int i=1; i < waypoints.size(); ++i) {
    double dist = distance(waypoints[i].position, waypoints[i-1].position);
    double diffYaw = std::abs(waypoints[i].yaw - waypoints[i-1].yaw);
    int factor = (int) std::max(dist / minDist, (diffYaw / minRot)) + 1;
    for (int j=1; j <= factor; ++j) {
      newWaypoints.push_back(getPointBetween(waypoints[i-1], waypoints[i], j, factor, minTime));
    }
  }
}

void getWaypointsFromMsg(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg, std::vector<WaypointWithTime> & waypoints) {

  for (int i=0; i < msg->points.size(); ++i) {
    mav_msgs::EigenTrajectoryPoint p;
    mav_msgs::eigenTrajectoryPointFromMsg(msg->points[i], &p);
    waypoints.push_back(WaypointWithTime(p.time_from_start_ns / 100, p.position_W[0], p.position_W[1], p.position_W[2], p.getYaw()));
  }
}

bool findPath(std::pair<int,int> s, std::pair<int,int> t, 
              std::vector< std::pair<int,int> > & path, 
              std::set< std::pair<int,int> > & occupied) {

  ROS_INFO("Trying to find path from %d,%d to %d,%d", s.first, s.second, t.first, t.second);
  std::map< std::pair<int,int>, std::pair<int,int> > parent;
  std::set< std::pair<int,int> > seen;
  std::queue< std::pair<int,int> > q;
  q.push(s);

  while (!q.empty()) {
    std::pair<int,int> u = q.front();
    q.pop();
    if (seen.find(u) != seen.end()) {
      continue;
    }
    seen.insert(u);
    if (u == t) {
      break;
    }

    std::vector< std::pair<int,int> > neighbors;
    neighbors.push_back(std::make_pair(u.first-1, u.second));
    neighbors.push_back(std::make_pair(u.first+1, u.second));
    neighbors.push_back(std::make_pair(u.first, u.second-1));
    neighbors.push_back(std::make_pair(u.first, u.second+1));

    for (auto v : neighbors) {
      if (occupied.find(v) == occupied.end() && seen.find(v) == seen.end()) {
        parent[v] = u;

        q.push(v);
      }
    }
  }

  if (seen.find(t) == seen.end()) {
    ROS_INFO("  Failed to find a path");
    return false;
  }
  std::pair<int, int> walker = t;
  while (walker != s) {
    ROS_INFO("%d,%d", walker.first, walker.second);
    path.push_back(walker);
    walker = parent[walker];
  }
  std::reverse(path.begin(),path.end());

  // for (int i=0; i < path.size(); ++i) {
  //   if (occupied.find(path[i]) != occupied.end()) {
  //     path.resize(i);
  //   }
  // }

  return true;

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

  cmd_multi_dof_joint_trajectory_pub_ = 
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    printf("\n    Planner Constructer \n");
}

GlobalPlannerNode::~GlobalPlannerNode() { }

void GlobalPlannerNode::Publish() {
}

void GlobalPlannerNode::PositionCallback(
    const geometry_msgs::PoseStamped& msg) {

  position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  yaw = tf::getYaw(msg.pose.orientation);
}

void GlobalPlannerNode::OctomapCallback(
    const visualization_msgs::MarkerArray& msg) {

  occupied.clear();
  ROS_INFO("  Num Points: %d", msg.markers[msg.markers.size()-1].points.size());
  for (auto p : msg.markers[msg.markers.size()-1].points) {
    occupied.insert(std::make_pair(p.x, p.y));
  }
}

void GlobalPlannerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

  ROS_INFO("   Current Position: %f, %f", position[0], position[1]);
  std::vector<WaypointWithTime> waypoints;
  waypoints.push_back(WaypointWithTime(0.0, position[0], position[1], position[2], yaw));
  getWaypointsFromMsg(msg, waypoints);

  if (waypoints.size() == 2) {
    std::pair<int,int> start = std::make_pair(waypoints[0].position[0], waypoints[0].position[1]);
    std::pair<int,int> end = std::make_pair(waypoints[1].position[0], waypoints[1].position[1]);
    std::vector< std::pair<int,int> > path;
    findPath(start, end, path, occupied);
    waypoints.resize(0);
    waypoints.push_back(WaypointWithTime(0, position[0], position[1], position[2], yaw));
    std::pair<int,int> pos = std::make_pair(position[0], position[1]);
    for (int i=1; i < path.size(); ++i) {
      auto p = path[i];
      waypoints.push_back(WaypointWithTime(0, p.first+0.5-1, p.second+0.5-1, 2, angle(pos,p)+1.5));
      ROS_INFO("p: %d, %d", p.first, p.second);
      pos = p;
    }
    // waypoints.push_back(WaypointWithTime(0, path[path.size()-1].first+0.5, path[path.size()-1].second+0.5, 0, 0));

  }



  std::vector<WaypointWithTime> newWaypoints;
  increaseResolution(waypoints, newWaypoints, 0.3, 0.05, 0.05 * kNanoSecondsInSecond);
  waypoints = newWaypoints;




  ROS_INFO("Start publishing endpoints.");

  trajectory_msgs::MultiDOFJointTrajectory newMsg;
  newMsg.header.stamp = ros::Time::now();
  newMsg.points.resize(waypoints.size());
  newMsg.joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &newMsg.points[i]);
  }

  cmd_multi_dof_joint_trajectory_pub_.publish(newMsg);
  printf("\n    Published Full Path \n");

}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");

  rotors_control::GlobalPlannerNode global_planner_node;

  ros::spin();

  return 0;
}
