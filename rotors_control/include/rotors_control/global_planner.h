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

#ifndef ROTORS_CONTROL_GLOBAL_PLANNER_H
#define ROTORS_CONTROL_GLOBAL_PLANNER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <queue>        // std::priority_queue
#include <map>
#include <algorithm>    // std::reverse
#include <math.h>       // abs
#include <tuple>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }
  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }
  WaypointWithTime(double t, Eigen::Vector3d pos, float _yaw)
      : position(pos), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

class Cell {
 public:
  Cell()
      : tpl(0,0,0) {
  }
  Cell(double x, double y)
      : tpl(floor(x), floor(y), 0) {
  }
  Cell(double x, double y, double z)
      : tpl(floor(x), floor(y), floor(z)) {
  }
  Cell(geometry_msgs::Point point)
      : tpl(floor(point.x), floor(point.y), floor(point.z))  {
  }
  Cell(Eigen::Vector3d point)
      : tpl(floor(point[0]), floor(point[1]), floor(point[2]))  {
  }

  bool operator==(const Cell & other) const {
    return this->tpl == other.tpl;
  }

  bool operator<(const Cell & other) const {
    return this->tpl < other.tpl;
  }

  int x() const {
    return std::get<0>(tpl);
  }
  int y() const {
    return std::get<1>(tpl);
  }
  int z() const {
    return std::get<2>(tpl);
  }

  std::tuple<int, int, int> tpl;
};

class GlobalPlanner {
 public:
  std::set<Cell> occupied;
  std::set<Cell> pathCells;
  std::vector<WaypointWithTime> waypoints;
  std::vector<Cell> pathBack;
  geometry_msgs::Point currPos;
  Cell goalPos;
  bool goingBack;
  double yaw;
  double overEstimateFactor = 1.5;
  int minHeight = 1;
  int maxHeight = 10;
  double inf = 1000000000.0;
  int maxIterations = 100000;

  GlobalPlanner();
  ~GlobalPlanner();

  void setPose(geometry_msgs::Point newPos, double newYaw);
  bool getGlobalPath();
  void increaseResolution(double minDist, double minRot, double minTime);
  bool updateOctomap(const visualization_msgs::MarkerArray& msg);
  void truncatePath();
  void getNeighbors(Cell cell, std::vector< std::pair<Cell, double> > & neighbors);
  bool FindPath(std::vector<Cell> & path);

 private:
  static const int64_t kNanoSecondsInSecond = 1000000000;

};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_H
