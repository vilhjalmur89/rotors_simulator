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

#include <ros/ros.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h> // getYaw createQuaternionMsgFromYaw 

#include <queue>            // std::priority_queue
#include <unordered_map>  
#include <algorithm>        // std::reverse
#include <math.h>           // abs
#include <tuple>
#include <string>
#include <limits>           // numeric_limits

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

  int x() const {return std::get<0>(tpl);}
  int y() const {return std::get<1>(tpl);}
  int z() const {return std::get<2>(tpl);}

  std::tuple<int, int, int> tpl;
};

inline bool operator==(const Cell& lhs, const Cell& rhs) {return lhs.tpl == rhs.tpl;}
inline bool operator!=(const Cell& lhs, const Cell& rhs) {return !operator==(lhs,rhs);}
inline bool operator< (const Cell& lhs, const Cell& rhs) {return lhs.tpl < rhs.tpl;}
inline bool operator> (const Cell& lhs, const Cell& rhs) {return  operator< (rhs,lhs);}
inline bool operator<=(const Cell& lhs, const Cell& rhs) {return !operator> (lhs,rhs);}
inline bool operator>=(const Cell& lhs, const Cell& rhs) {return !operator< (lhs,rhs);}

typedef std::pair<Cell, double> CellDistancePair;

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

class CompareDist
{
public:
    bool operator()(CellDistancePair n1, CellDistancePair n2) {
        return n1.second > n2.second;
    }
};

class GlobalPlanner {
 public:
  // octomap::OcTree* octree;
  std::vector<double> heightPrior { 1.0, 1.0, 0.8, 0.8, 0.6, 0.5, 0.4,
                                    0.3, 0.2, 0.1, 0.1, 0.1, 0.1};
  std::set<Cell> occupied;
  std::unordered_map<Cell, double, HashCell> occProb;
  std::set<Cell> seen;        // Set of cells that were explored in last search
  std::set<Cell> pathCells;   // Set of cells that are on current path
  std::vector<WaypointWithTime> waypoints;    // TODO: remove and use pathMsg
  nav_msgs::Path pathMsg;
  std::vector<Cell> pathBack;
  geometry_msgs::Point currPos;
  Cell goalPos;
  double yaw;
  bool goingBack = false;
  double overEstimateFactor = 2.0;
  int minHeight = 1;
  int maxHeight = 12;
  double maxPathProb = -1.0;
  double maxBailProb = 1.0;
  double inf = std::numeric_limits<double>::infinity();
  int maxIterations = 100000;
  double riskFactor = 6.0;
  double neighborRiskFlow = 1.0;
  double explorePenalty = 0.1;
  double upPenalty = 5;

  GlobalPlanner();
  ~GlobalPlanner();

  void setPose(geometry_msgs::Point newPos, double newYaw);
  bool updateFullOctomap(const octomap_msgs::Octomap& msg);
  void increaseResolution(double minDist, double minRot, double minTime);
  void truncatePath();
  void getOpenNeighbors(Cell cell, std::vector<CellDistancePair> & neighbors) const;
  double getRisk(Cell & cell);
  geometry_msgs::PoseStamped createPoseMsg(double x, double y, double z, double yaw);
  void pathToWaypoints(std::vector<Cell> & path);
  void goBack();
  bool FindPath(std::vector<Cell> & path);
  bool FindPath(std::vector<Cell> & path, const Cell s, Cell t);
  bool getGlobalPath();


 private:
  static const int64_t kNanoSecondsInSecond = 1000000000;

};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_H
