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

double angle(Cell pos, Cell p, double lastAng) {
  int dx = p.x() - pos.x();
  int dy = p.y() - pos.y();
  if (dx == 0 && dy == 0) {
    return lastAng;   // Going up or down
  }
  double ang = atan2(dy, dx);
  // Now ang is in the range [-Pi, Pi], but we want ang to be in [lastAng-Pi, lastAng+Pi]
  // This is because the rotor will spin a whole circle when going from 359 deg to 0 deg
  ang -= int(((ang+M_PI)-lastAng)/(2*M_PI)) * (2*M_PI);   // Now ang <= lastAng+Pi
  ang -= int(((ang-M_PI)-lastAng)/(2*M_PI)) * (2*M_PI);   // Now ang >= lastAng-Pi
  return ang;
}

double interpolate(double start, double end, double ratio) {
  return start + (end - start) * ratio;
}

WaypointWithTime interpolateWaypoints(const WaypointWithTime & a, const WaypointWithTime & b, 
                                      int step, int totalSteps, int minTime) {

  double ratio = (double)step / (double)totalSteps;
  double x = interpolate(a.position[0], b.position[0], ratio);
  double y = interpolate(a.position[1], b.position[1], ratio);
  double z = interpolate(a.position[2], b.position[2], ratio);
  double yaw = interpolate(a.yaw, b.yaw, ratio);
  return WaypointWithTime(minTime, x, y, z, yaw);
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
  // Straight-line distance disregarding the z-coordinate
  return sqrt(squared(a.x() - b.x()) + squared(a.y() - b.y()));
}

double diagDistance2D(const Cell & a, const Cell & b) {
  // Minimum distance on a grid where you can move diagonally
  double dx = abs(a.x() - b.x());
  double dy = abs(a.y() - b.y());
  double diagCost = 1.41421356237;
  return (dx + dy) + (diagCost - 2) * std::min(dx, dy);
}

GlobalPlanner::GlobalPlanner()  {}
GlobalPlanner::~GlobalPlanner() {}

void GlobalPlanner::setPose(const geometry_msgs::Point newPos, double newYaw) {
  currPos = newPos;
  yaw = newYaw;
  Cell currCell = Cell(currPos);
  if (!goingBack && (pathBack.size() == 0 || currCell != pathBack[pathBack.size()-1])) {
    // Keep track of where we have been, add current position to pathBack if it is different from last one
    pathBack.push_back(currCell);
  }
}

void GlobalPlanner::setGoal(const Cell goal) {
  goalPos = goal;
  goingBack = false;
  goalIsBlocked = false;
}

bool GlobalPlanner::updateFullOctomap(const octomap_msgs::Octomap & msg) {
  // Returns false iff current path has an obstacle
  bool pathIsBlocked = false;
  occupied.clear();
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(msg);
  octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
  if (tree) {
    for(auto it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it){
      Cell cell(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
      double cellProb = it->getValue(); 
      occProb[cell] = cellProb;
      if (cellProb > maxPathProb) {
        // Cell is too risky to plan a new path through
        occupied.insert(cell);
        if (cellProb > maxBailProb && pathCells.find(cell) != pathCells.end()) {
          // Cell is on path and is risky enough to abort mission
          pathIsBlocked = true;
        }
      }
      if (it.getSize() > 1) {
        // TODO: Need a loop for large leafs
        // ROS_INFO("%d, %d, %d: %d %f", cell.x(), cell.y(), cell.z(), it.getSize(), it->getValue());
      }
    }
  }
  delete octree;
  return !pathIsBlocked;
}

void GlobalPlanner::increaseResolution(double minDist, double minRot, double minTime) {
  // Interpolates the path such that there is at most minDist, minRot and minTime between waypoints
  
  // Remove every other cell to smooth out path
  std::vector<WaypointWithTime> newWaypoints;
  for (int i=0; i < waypoints.size()-1; i+=2) {
    newWaypoints.push_back(waypoints[i]);
  }
  newWaypoints.push_back(waypoints[waypoints.size()-1]);
  waypoints = newWaypoints;

  newWaypoints.resize(0);

  // newWaypoints.push_back(waypoints[0]);
  // ROS_INFO("0: %f, 1: %f", waypoints[0].yaw, waypoints[1].yaw);
  for (int i=1; i < waypoints.size(); ++i) {
    double dist = distance(waypoints[i].position, waypoints[i-1].position);
    double diffYaw = std::abs(waypoints[i].yaw - waypoints[i-1].yaw);
    int numSteps = (int) std::max(dist / minDist, (diffYaw / minRot)) + 1;
    for (int j=1; j <= numSteps; ++j) {
      newWaypoints.push_back(interpolateWaypoints(waypoints[i-1], waypoints[i], j, numSteps, minTime));
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
      // ROS_INFO("currIndex = %d", currIndex);
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

void GlobalPlanner::getOpenNeighbors(Cell cell, std::vector<CellDistancePair> & neighbors) const {
  // Fill neighbors with the 8 horizontal and 2 vertical non-occupied neigbors
  // It's long because it uses the minimum number of 'if's 
  // TODO: Try using weights instead of 'if's
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
    neighbors.push_back(std::make_pair(up, 1.0 + upPenalty));
  }
  if (z > minHeight && downOpen) {
    neighbors.push_back(std::make_pair(down, 1.0));
  }
}

double GlobalPlanner::getRisk(Cell & cell){
  // Computes the risk from the cell, its neighbors and the prior

  double risk = explorePenalty;
  if (occProb.find(cell) != occProb.end()) {
    risk = octomap::probability(occProb[cell]);
  }

  Cell front = Cell(cell.x()+1, cell.y(), cell.z());
  Cell back = Cell(cell.x()-1, cell.y(), cell.z());
  Cell right = Cell(cell.x(), cell.y()+1, cell.z());
  Cell left = Cell(cell.x(), cell.y()-1, cell.z());
  Cell up = Cell(cell.x(), cell.y(), cell.z()+1);
  Cell down = Cell(cell.x(), cell.y(), cell.z()-1);
  std::vector<Cell> cells {front, back, right, left, up, down};

  for (Cell cell : cells) {
    if (occProb.find(cell) != occProb.end()) {
      risk += neighborRiskFlow * octomap::probability(occProb[cell]);
    }
  }

  double prior = heightPrior[floor(cell.z())];
  return risk * prior;
}

 geometry_msgs::PoseStamped GlobalPlanner::createPoseMsg(double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.header.frame_id="/world";
    poseMsg.pose.position.x = x;
    poseMsg.pose.position.y = y;
    poseMsg.pose.position.z = z;
    poseMsg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    poseMsg.pose.orientation.x = poseMsg.pose.orientation.z;
    poseMsg.pose.orientation.z = 0.0;
    return poseMsg;
 }

// TODO: Straight to msg, also fill in pathCells here 
void GlobalPlanner::pathToWaypoints(std::vector<Cell> & path) {
  waypoints.resize(0);
  // Use actual position instead of the center of the cell
  // waypoints.push_back(WaypointWithTime(0, currPos.x, currPos.y, currPos.z, yaw));   
  double lastYaw = yaw;

  path.push_back(path[path.size()-1]); // Needed if every other cell of the path is discarded
  for (int i=1; i < path.size()-1; ++i) {
    Cell p = path[i];
    Cell lastP = path[i-1];
    double newYaw = angle(p, path[i+1], lastYaw);
    // if (newYaw != lastYaw) {   // only publish corner points
      waypoints.push_back(WaypointWithTime(0, p.x()+0.5, p.y()+0.5, p.z()+0.5, newYaw));
    // }
    lastYaw = newYaw;

    pathCells.insert(p);
    if (p.x() != lastP.x() && p.y() != lastP.y()) {
      // For diagonal edges we need the two common neighbors of p and lastP to non occupied
      pathCells.insert(Cell(p.x(), lastP.y(), p.z()));
      pathCells.insert(Cell(lastP.x(), p.y(), p.z()));
    }
  }
  waypoints.push_back(WaypointWithTime(0, path[path.size()-1].x()+0.5, path[path.size()-1].y()+0.5, 2, lastYaw));

  // increaseResolution(2.0, 10, 1000);
  // increaseResolution(0.3, 0.05, 0.1 * kNanoSecondsInSecond);
}

void GlobalPlanner::goBack() {
    goingBack = true;
    std::vector<Cell> path;
    for (int i=pathBack.size()-1; i >= 0; i -= 2){
      // Filter every other cell for a smooth path
      path.push_back(pathBack[i]);
    }
    pathToWaypoints(path);
}

// TODO: Run search backwards to quickly find impossible scenarios
// bool GlobalPlanner::isGoalBlocked() { }


bool GlobalPlanner::FindPath(std::vector<Cell> & path) {
  Cell s = Cell(currPos);
  Cell t = Cell(goalPos.x(), goalPos.y(), 2);
  bool foundPath = FindPath(path, s, t);
  return foundPath;
}

bool GlobalPlanner::FindPath(std::vector<Cell> & path, const Cell s, Cell t) {
  // A* to find a path from currPos to goalPos, true iff it found a path

  
  ROS_INFO("Planning a path from (%d, %d) to (%d, %d)", s.x(), s.y(), t.x(), t.y());

  // Initialize containers
  seen.clear();
  std::map<Cell, Cell> parent;
  std::map<Cell, double> distance;
  std::priority_queue<CellDistancePair, std::vector<CellDistancePair>, CompareDist> pq;                 
  pq.push(std::make_pair(s, 0.0));
  distance[s] = 0.0;
  int numIter = 0;

  // Search until all reachable cells have been found, run out of time or t is found,
  while (!pq.empty() && numIter < maxIterations && seen.find(t) == seen.end()) {
    CellDistancePair cellDistU = pq.top(); pq.pop();
    Cell u = cellDistU.first;
    double d = distance[u];
    if (seen.find(u) != seen.end()) {
      continue;
    }
    seen.insert(u);
    numIter++;
    if (u == t) {
      break;  // Found a path
    }

    std::vector<CellDistancePair> neighbors;
    getOpenNeighbors(u, neighbors);
    for (auto cellDistV : neighbors) {
      Cell v = cellDistV.first;
      double costOfEdge = cellDistV.second;
      double risk = getRisk(v);
      double newDist = d + costOfEdge + riskFactor * risk;
      double oldDist = inf;
      if (distance.find(v) != distance.end()) {
        oldDist = distance[v];
      }
      if (newDist < oldDist) {
        // Found a better path to v, have to add v to the queue 
        parent[v] = u;
        distance[v] = newDist;
        // TODO: try Dynamic Weighting in stead of a constant overEstimateFactor
        double heuristic = diagDistance2D(v, t) + upPenalty * std::max(0, t.z() - v.z());
        double overestimatedHeuristic = newDist + overEstimateFactor*heuristic;
        // double heuristic = newDist + overEstimateFactor*distance2D(v, t);
        pq.push(std::make_pair(v, overestimatedHeuristic));
      }
    }
  }

  if (seen.find(t) == seen.end()) {
    // No path found
    return false;
  }

  // Get the path by walking from t back to s
  Cell walker = t;
  pathCells.clear();
  while (walker != s) {
    path.push_back(walker);
    walker = parent[walker];
  }
  std::reverse(path.begin(),path.end());

  ROS_INFO("Found path with %d iterations, iterations / distance squared: %f", numIter, numIter / squared(path.size()));
  
  return true;
}  

bool GlobalPlanner::getGlobalPath() {
  // true iff a path needs to be published, either a new path or a path back
  // The path is then stored in this.waypoints
  Cell s = Cell(currPos);
  Cell t = Cell(goalPos.x(), goalPos.y(), 2);
  
  if (occupied.find(t) != occupied.end()) {
    // If goal is occupied, no path is published
    ROS_INFO("Goal position is occupied");
    goalIsBlocked = true;
    return false;
  }
  else if (occupied.find(s) != occupied.end()) {
    // If current position is occupied the way back is published
    ROS_INFO("Current position is occupied, going back.");
    goBack();
    return true;
  }
  else {
    // Both current position and goal are free, try to find a path
    std::vector<Cell> path;
    if (!FindPath(path)) {
      ROS_INFO("  Failed to find a path");
      return false;
    }
    pathToWaypoints(path);
    return true;
  }
}



}
