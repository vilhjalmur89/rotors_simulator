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
#include <unordered_map>
#include <algorithm>    // std::reverse
#include <math.h>       // abs
#include <tuple>
#include <string>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

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

struct HashCell {
    size_t operator()(const Cell &cell ) const
    {
        std::string s = "";
        s += cell.x();
        s += cell.y();
        s += cell.z();
        return std::hash<std::string>()(s);
    }
};

class GlobalPlanner {
 public:
  octomap::OcTree* octree;
  std::set<Cell> occupied;
  std::unordered_map<Cell, double, HashCell> occProb; // TODO: Compare with hashmap
  std::set<Cell> pathCells;
  std::vector<WaypointWithTime> waypoints;
  std::vector<Cell> pathBack;
  geometry_msgs::Point currPos;
  Cell goalPos;
  bool goingBack;
  double yaw;
  double overEstimateFactor = 1.0;
  int minHeight = 1;
  int maxHeight = 12;
  double maxPathProb = -1.0;
  double maxBailProb = 2.0;
  double inf = 1000000000.0;
  int maxIterations = 100000;
  double riskFactor = 100.0;
  double explorePenalty = 0.2;

  GlobalPlanner();
  ~GlobalPlanner();

  void setPose(geometry_msgs::Point newPos, double newYaw);
  bool getGlobalPath();
  void increaseResolution(double minDist, double minRot, double minTime);
  bool updateFullOctomap(const octomap_msgs::Octomap& msg);
  bool updateOctomap(const visualization_msgs::MarkerArray& msg);
  void truncatePath();
  void getNeighbors(Cell cell, std::vector< std::pair<Cell, double> > & neighbors);
  double getRisk(Cell & cell);
  bool FindPath(std::vector<Cell> & path);

 private:
  static const int64_t kNanoSecondsInSecond = 1000000000;

};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_H
