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

#include "rotors_control/global_planner.h"

namespace rotors_control {




double angle(std::pair<int,int> pos, std::pair<int,int> p) {
  int dx = p.first - pos.first;
  int dy = p.second - pos.second;
  if (dy == 0) {
    return M_PI/2 - (dx/abs(dx))*(M_PI/2);
  }
  return (dy/abs(dy))*M_PI/2;
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
  ROS_INFO("0: %f, 1: %f", waypoints[0].yaw, waypoints[1].yaw);
  for (int i=1; i < waypoints.size(); ++i) {
    double dist = distance(waypoints[i].position, waypoints[i-1].position);
    double diffYaw = std::abs(waypoints[i].yaw - waypoints[i-1].yaw);
    int factor = (int) std::max(dist / minDist, (diffYaw / minRot)) + 1;
    for (int j=1; j <= factor; ++j) {
      newWaypoints.push_back(getPointBetween(waypoints[i-1], waypoints[i], j, factor, minTime));
    }
  }
}



bool GlobalPlanner::FindPath(std::pair<int,int> s, std::pair<int,int> t, 
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
  pathCells.clear();
  while (walker != s) {
    path.push_back(walker);
    pathCells.insert(walker);
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

GlobalPlanner::GlobalPlanner()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

GlobalPlanner::~GlobalPlanner() {}

void GlobalPlanner::InitializeParameters() {

}

void GlobalPlanner::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {

}

void GlobalPlanner::getGlobalPath(std::vector<WaypointWithTime> & waypoints) {
  if (waypoints.size() == 1) {
    waypoints.resize(2);
    waypoints[1] = waypoints[0];
    waypoints[0] = WaypointWithTime(0.0, position[0], position[1], position[2], yaw);
    // waypoints.push_back(WaypointWithTime(0.0, position[0], position[1], position[2], yaw));
    std::pair<int,int> start = std::make_pair(floor(waypoints[0].position[0]), floor(waypoints[0].position[1]));
    std::pair<int,int> end = std::make_pair(floor(waypoints[1].position[0]), floor(waypoints[1].position[1]));
    std::vector< std::pair<int,int> > path;
    FindPath(start, end, path, occupied);
    waypoints.resize(0);
    waypoints.push_back(WaypointWithTime(0, position[0], position[1], position[2], yaw));
    std::pair<int,int> pos = std::make_pair(position[0], position[1]);
    for (int i=1; i < path.size(); ++i) {
      auto p = path[i];
      waypoints.push_back(WaypointWithTime(0, p.first+0.5, p.second+0.5, 2, angle(pos,p)));
      pos = p;
    }
    waypoints.push_back(WaypointWithTime(0, path[path.size()-1].first+0.5-1, path[path.size()-1].second+0.5-1, 0, 0));

  }
  else {
    ROS_INFO("Wrong waypoints-size");
  }

  std::vector<WaypointWithTime> newWaypoints;
  increaseResolution(waypoints, newWaypoints, 0.3, 0.1, 0.1 * kNanoSecondsInSecond);
  waypoints.resize(0);
  for (auto x : newWaypoints) {
    waypoints.push_back(x);
  }
  // waypoints = newWaypoints;

}




}
