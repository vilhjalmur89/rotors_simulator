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

class CompareDist
{
public:
    bool operator()(std::pair<Cell,double> n1,std::pair<Cell,double> n2) {
        return n1.second>n2.second;
    }
};


double angle(Cell pos, Cell p, double lastAng) {
  // Assumption: p is one of the eight neighboring cells of pos
  int dx = p.x() - pos.x();
  int dy = p.y() - pos.y();
  double ang;
  if (dy > 0) {
    ang = M_PI/2 - M_PI/4 * dx;
  }
  else if (dy < 0) {
    ang = 3*M_PI/2 + M_PI/4 * dx;
  }
  else if (dx != 0){
    ang = M_PI/2 - M_PI/2 * dx;     // Left or right
  }
  else {
    ang = lastAng;                  // up
  }
  ang -= int(((ang+M_PI)-lastAng)/(2*M_PI)) * (2*M_PI);
  ang -= int(((ang-M_PI)-lastAng)/(2*M_PI)) * (2*M_PI);
  return ang;
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

double distance2D(const Cell & a, const Cell & b) {
  return sqrt(squared(a.x() - b.x()) + squared(a.y() - b.y()));
}

GlobalPlanner::GlobalPlanner() : goingBack(false) {}
GlobalPlanner::~GlobalPlanner() {}

void GlobalPlanner::setPose(geometry_msgs::Point newPos, double newYaw) {
  currPos = newPos;
  yaw = newYaw;
  Cell currCell = Cell(currPos);
  if (!goingBack && (pathBack.size() == 0 || !(currCell == pathBack[pathBack.size()-1]))) {
    pathBack.push_back(currCell);
  }
}

bool GlobalPlanner::updateFullOctomap(const octomap_msgs::Octomap& msg) {
  // Returns false iff current path has an obstacle
  bool pathIsBad = false;
  occupied.clear();
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(msg);
  octree = dynamic_cast<octomap::OcTree*>(tree);
  if (tree) {
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
      //manipulate node, e.g.:
      Cell cell(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
      double cellProb = it->getValue(); 
      occProb[cell] = cellProb;
      if (cellProb > maxPathProb) {
        occupied.insert(cell);
        if (cellProb > maxBailProb && pathCells.find(cell) != pathCells.end()) {
          pathIsBad = true;
        }
      }
    }
  }
  return pathIsBad;
}

bool GlobalPlanner::updateOctomap(const visualization_msgs::MarkerArray& msg) {
  // Returns false iff current path has an obstacle
  occupied.clear();
  bool pathIsBad = false;
  for (auto point : msg.markers[msg.markers.size()-1].points) {   // TODO: Why the last markers
    Cell cell(floor(point.x), floor(point.y), floor(point.z));
    occupied.insert(cell);
    if (pathCells.find(cell) != pathCells.end()) {
      pathIsBad = true;
    }
  }
  return pathIsBad;
}


void GlobalPlanner::increaseResolution(double minDist, double minRot, double minTime) {

  std::vector<WaypointWithTime> newWaypoints;
  // newWaypoints.push_back(waypoints[0]);
  ROS_INFO("0: %f, 1: %f", waypoints[0].yaw, waypoints[1].yaw);
  for (int i=1; i < waypoints.size(); ++i) {
    double dist = distance(waypoints[i].position, waypoints[i-1].position);
    double diffYaw = std::abs(waypoints[i].yaw - waypoints[i-1].yaw);
    int factor = (int) std::max(dist / minDist, (diffYaw / minRot)) + 1;
    for (int j=1; j <= factor; ++j) {
      newWaypoints.push_back(getPointBetween(waypoints[i-1], waypoints[i], j, factor, minTime));
    }
  }
  waypoints = newWaypoints;
}

void GlobalPlanner::truncatePath() {
  // std::vector<WaypointWithTime> newWaypoints;
  // int currIndex = 0;
  // for (int i=0; i < waypoints.size(); ++i) {
  //   Cell c = Cell(waypoints[i].position);
  //   if (c == Cell(currPos)) {
  //     currIndex = i;
  //     ROS_INFO("currIndex = %d", currIndex);
  //     break;
  //   }
  //   if (occupied.find(c) != occupied.end()) {
  //     return;
  //   }
  // }
  // for (int i=currIndex+1; i < waypoints.size(); ++i) {
  //   Cell c = Cell(waypoints[i].position);
  //   if (occupied.find(c) != occupied.end()) {
  //     return;
  //   }
  //   newWaypoints.push_back(waypoints[i]);
  // }
  // waypoints = newWaypoints;

  pathCells.clear();
  waypoints.resize(0);
}

void GlobalPlanner::getNeighbors(Cell cell, std::vector< std::pair<Cell, double> > & neighbors) {
  // Right angle neighbors
  double x = cell.x();
  double y = cell.y();
  double z = cell.z();
  Cell forw = Cell(x+1, y, z);
  Cell back = Cell(x-1, y, z);
  Cell left = Cell(x, y-1, z);
  Cell righ = Cell(x, y+1, z);
  Cell forwLeft = Cell(x+1, y-1, z);
  Cell forwRigh = Cell(x+1, y+1, z);
  Cell backLeft = Cell(x-1, y-1, z);
  Cell backRigh = Cell(x-1, y+1, z);
  Cell up = Cell(x, y, z+1);
  Cell down = Cell(x, y, z-1);

  bool forwOpen = occupied.find(forw) == occupied.end();
  bool backOpen = occupied.find(back) == occupied.end();
  bool leftOpen = occupied.find(left) == occupied.end();
  bool righOpen = occupied.find(righ) == occupied.end();
  bool forwLeftOpen = occupied.find(forwLeft) == occupied.end();
  bool forwRighOpen = occupied.find(forwRigh) == occupied.end();
  bool backLeftOpen = occupied.find(backLeft) == occupied.end();
  bool backRighOpen = occupied.find(backRigh) == occupied.end();
  bool upOpen = occupied.find(up) == occupied.end();
  bool downOpen = occupied.find(down) == occupied.end();

  if (forwOpen) {
    neighbors.push_back(std::make_pair(forw, 1.0));
    if (leftOpen && forwLeftOpen) {
      neighbors.push_back(std::make_pair(forwLeft, 1.41));
    }
    if (righOpen && forwRighOpen) {
      neighbors.push_back(std::make_pair(forwRigh, 1.41));
    }
  }
  if (backOpen) {
    neighbors.push_back(std::make_pair(back, 1.0));
    if (leftOpen && backLeftOpen) {
      neighbors.push_back(std::make_pair(backLeft, 1.41));
    }
    if (righOpen && backRighOpen) {
      neighbors.push_back(std::make_pair(backRigh, 1.41));
    }
  }
  if (leftOpen) {
    neighbors.push_back(std::make_pair(left, 1.0));
  }
  if (righOpen) {
    neighbors.push_back(std::make_pair(righ, 1.0));
  }

  // Vertical neighbors
  if (z < maxHeight && upOpen) {
    neighbors.push_back(std::make_pair(up, 1.0));
  }
  if (z > minHeight && downOpen) {
    neighbors.push_back(std::make_pair(down, 1.0));
  }
}

bool GlobalPlanner::FindPath(std::vector<Cell> & path) {

  Cell s = Cell(currPos);
  Cell t = Cell(goalPos.x(), goalPos.y(), 2);

  if (occupied.find(s) != occupied.end()) {
    ROS_INFO("Current position is occupied, going back");
    goingBack = true;
    for (int i=pathBack.size()-1; i > 0; i -= 2){
      path.push_back(pathBack[i]);
    }
    return true;
  }

  ROS_INFO("Trying to find path from %d,%d to %d,%d", s.x(), s.y(), t.x(), t.y());
  std::map<Cell, Cell> parent;
  std::map<Cell, double> distance;
  std::set<Cell> seen;
  std::priority_queue< std::pair<Cell,double>, 
                       std::vector< std::pair<Cell,double> >,
                       CompareDist> pq;

  pq.push(std::make_pair(s, 0.0));
  distance[s] = 0.0;
  int numIter = 0;
  while (!pq.empty() && numIter++ < maxIterations) {
    auto cellDistU = pq.top(); pq.pop();
    Cell u = cellDistU.first;
    double d = cellDistU.second;
    if (seen.find(u) != seen.end()) {
      continue;
    }
    seen.insert(u);
    if (u == t) {
      break;
    }
    std::vector< std::pair<Cell, double> > neighbors;
    getNeighbors(u, neighbors);

    for (auto cellDistV : neighbors) {
      Cell v = cellDistV.first;
      double risk = 5;
      // if (occProb.find(v) != occProb.end()) {
      //   risk = occProb[v] + 2.0;
      // }
      double newDist = d + cellDistV.second + risk;
      double oldDist = inf;
      if (distance.find(v) != distance.end()) {
        oldDist = distance[v];
      }
      if (occupied.find(v) == occupied.end() && seen.find(v) == seen.end()
          && newDist < oldDist) {
        parent[v] = u;
        distance[v] = newDist;
        double heuristic = newDist + overEstimateFactor*distance2D(v, t);
        pq.push(std::make_pair(v, heuristic));
      }
    }
  }

  if (seen.find(t) == seen.end()) {
    ROS_INFO("  Failed to find a path");
    return false;
  }
  ROS_INFO("Found path with %d iterations", numIter);

  Cell walker = t;
  pathCells.clear();
  while (!(walker == s)) {
    path.push_back(walker);
    pathCells.insert(walker);
    walker = parent[walker];
  }
  std::reverse(path.begin(),path.end());
  return true;
}  

bool GlobalPlanner::getGlobalPath() {
  std::vector<Cell> path;
  if (!FindPath(path)) {
    return false;
  }

  waypoints.resize(0);
  // Use actual position instead of the center of the cell
  waypoints.push_back(WaypointWithTime(0, currPos.x, currPos.y, currPos.z, yaw));   
  double lastYaw = yaw;
  path.push_back(path[path.size()-1]);
  for (int i=1; i < path.size()-1; ++i) {
    Cell p = path[i];
    double newYaw = angle(p, path[i+1], lastYaw);
    waypoints.push_back(WaypointWithTime(0, p.x()+0.5, p.y()+0.5, p.z()+0.5, newYaw));
    lastYaw = newYaw;
  }
  waypoints.push_back(WaypointWithTime(0, path[path.size()-1].x()+0.5, path[path.size()-1].y()+0.5, 2, 0));

  increaseResolution(0.3, 0.05, 0.1 * kNanoSecondsInSecond);
  return true;

}
}
