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

  int x() const {return std::get<0>(tpl);}
  int y() const {return std::get<1>(tpl);}
  int z() const {return std::get<2>(tpl);}

  double manhattanDist(double _x, double _y, double _z) const {
    return std::abs(x()+0.5 - _x) + std::abs(y()+0.5 - _y) + std::abs(z()+0.5 - _z);
  }

  double angle() {
    return atan2(y(), x());
  }

  Cell getNeighborFromYaw(double yaw) {
    // Returns the neighboring cell in the yaw direction
    // E.g. if yaw == PI/4, then it returns Cell(x+1, y+1, z)
    int xDiff = 2*std::cos(yaw);
    int yDiff = 2*std::sin(yaw);
    ROS_INFO("parent: %d %d \n", xDiff, yDiff);
    return Cell(x() + xDiff, y() + yDiff, z());
  }

  std::string asString() const {  
    std::string s = "(" + std::to_string(x()) + "," + std::to_string(y()) + "," + std::to_string(z()) + ")";
    return s;
  }

  std::tuple<int, int, int> tpl;
};

inline bool operator==(const Cell& lhs, const Cell& rhs) {return lhs.tpl == rhs.tpl;}
inline bool operator!=(const Cell& lhs, const Cell& rhs) {return !operator==(lhs,rhs);}
inline bool operator< (const Cell& lhs, const Cell& rhs) {return lhs.tpl < rhs.tpl;}
inline bool operator> (const Cell& lhs, const Cell& rhs) {return  operator< (rhs,lhs);}
inline bool operator<=(const Cell& lhs, const Cell& rhs) {return !operator> (lhs,rhs);}
inline bool operator>=(const Cell& lhs, const Cell& rhs) {return !operator< (lhs,rhs);}

inline Cell operator+(const Cell& lhs, const Cell& rhs) {
  Cell res(lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z());
  return res;
}
inline Cell operator-(const Cell& lhs, const Cell& rhs) {
  Cell res(lhs.x() - rhs.x(), lhs.y() - rhs.y(), lhs.z() - rhs.z());
  return res;
}

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



class Node {
 public:
  Node()
      : cell(), parent() {
  }
  Node(const Cell cell, const Cell parent)
      : cell(cell), parent(parent) {
  }
  Cell cell;
  Cell parent;
};
inline bool operator==(const Node& lhs, const Node& rhs) {
  return lhs.cell == rhs.cell && lhs.parent == rhs.parent;}
inline bool operator< (const Node& lhs, const Node& rhs) {
  return lhs.cell < rhs.cell || (lhs.cell == rhs.cell && lhs.parent < rhs.parent);}
inline bool operator!=(const Node& lhs, const Node& rhs) {return !operator==(lhs,rhs);}
inline bool operator> (const Node& lhs, const Node& rhs) {return  operator< (rhs,lhs);}
inline bool operator<=(const Node& lhs, const Node& rhs) {return !operator> (lhs,rhs);}
inline bool operator>=(const Node& lhs, const Node& rhs) {return !operator< (lhs,rhs);}

typedef std::pair<Node, double> NodeDistancePair;

struct HashNode {
  size_t operator()(const Node &node ) const
  {
    HashCell hash;
    std::string s = "";
    s += hash(node.cell);
    s += " ";
    s += hash(node.parent);
    return std::hash<std::string>()(s);
  }
};  


class CompareDist
{
public:
    bool operator()(const CellDistancePair n1, const CellDistancePair n2) {
      return n1.second > n2.second;
    }
    bool operator()(const NodeDistancePair n1, const NodeDistancePair n2) {
      return n1.second > n2.second;
    }
};


class GlobalPlanner {
 public:
  // octomap::OcTree* octree;
  std::vector<double> heightPrior { 1.0, 1.0, 0.8, 0.6, 0.5, 0.4, 0.3,
                                    0.2, 0.2, 0.1, 0.1, 0.1, 0.1};
  std::set<Cell> occupied;
  std::unordered_map<Cell, double, HashCell> occProb;
  std::set<Cell> seen;        // Set of cells that were explored in last search
  std::set<Cell> pathCells;   // Set of cells that are on current path, and cannot be blocked
  std::vector<WaypointWithTime> waypoints;    // TODO: remove and use pathMsg
  nav_msgs::Path pathMsg;
  std::vector<Cell> pathBack;
  geometry_msgs::Point currPos;
  double currYaw;
  Cell goalPos = Cell(0, 0, 3);
  bool goingBack = false;
  double overEstimateFactor = 2.0;
  int minHeight = 1;
  int maxHeight = 12;
  double maxPathProb = -1.0;
  double maxBailProb = 1.0;
  double inf = std::numeric_limits<double>::infinity();
  int maxIterations = 10000;
  double smoothFactor = 1.0;
  double riskFactor = 10.0;
  double neighborRiskFlow = 1.0;
  double explorePenalty = 0.015;
  double upCost = 3.0;
  double downCost = 1.0;
  bool goalIsBlocked = false;
  int numOctomapMessages = 0;

  GlobalPlanner();
  ~GlobalPlanner();

  void setPose(const geometry_msgs::Point & newPos, double newYaw);
  void setGoal(const Cell & goal);
  bool updateFullOctomap(const octomap_msgs::Octomap & msg);
  void increaseResolution(double minDist, double minRot, double minTime);
  void truncatePath();
  void getOpenNeighbors(const Cell & cell, std::vector<CellDistancePair> & neighbors) const;
  void printPathStats(const std::vector<Cell> & path, const Cell startParent, const Cell start,
                                   const Cell goal, double totalDistance, std::map<Node, double> & distance);

  double getEdgeDist(const Cell & u, const Cell & v);
  double getRisk(const Cell & cell);
  double getTurnSmoothness(const Node & u, const Node & v);
  double getEdgeCost(const Node & u, const Node & v);

  double riskHeuristic(const Cell & u, const Cell & goal);
  double smoothnessHeuristic(const Node & u, const Cell & goal);
  double altitudeHeuristic(const Cell & u, const Cell & goal);
  double getHeuristic(const Node & u, const Cell & goal);
  
  geometry_msgs::PoseStamped createPoseMsg(double x, double y, double z, double yaw);
  void pathToWaypoints(std::vector<Cell> & path);
  void goBack();
  bool FindPath(std::vector<Cell> & path);
  bool FindPath(std::vector<Cell> & path, const Cell & s, Cell t);
  bool FindSmoothPath(std::vector<Cell> & path, const Cell & s, const Cell & t, const Cell & parent);
  bool getGlobalPath();


 private:
  static const int64_t kNanoSecondsInSecond = 1000000000;

};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_H
