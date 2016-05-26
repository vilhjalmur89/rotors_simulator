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

#include "rotors_control/cell.h"

#include <ctime>  // clock

namespace rotors_control {

Cell::Cell() = default;
Cell::Cell(std::tuple<int, int, int> newTpl)
  : tpl(newTpl) {}
Cell::Cell(double x, double y, double z)
  : tpl(floor(x / scale), floor(y / scale), floor(z / scale)) {}
Cell::Cell(double x, double y) 
  : Cell(x, y, 0.0) {}
Cell::Cell(geometry_msgs::Point point) 
  : Cell(point.x, point.y, point.z) {}
Cell::Cell(Eigen::Vector3d point) 
  : Cell(point[0], point[1], point[2])  {}

int Cell::x() const {return std::get<0>(tpl);}
int Cell::y() const {return std::get<1>(tpl);}
int Cell::z() const {return std::get<2>(tpl);}

double Cell::xPos() const {return scale * x() + scale * 0.5;}
double Cell::yPos() const {return scale * y() + scale * 0.5;}
double Cell::zPos() const {return scale * z() + scale * 0.5;}

double Cell::manhattanDist(double _x, double _y, double _z) const {
  return std::abs(xPos() - _x) + std::abs(yPos() - _y) + std::abs(zPos() - _z);
}

double Cell::distance2D(const Cell & b) const {
  // Straight-line distance disregarding the z-coordinate
  return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos()));
}

double Cell::diagDistance2D(const Cell & b) const {
  // Minimum distance on the XY-grid where you can move diagonally
  double dx = abs(xPos() - b.xPos());
  double dy = abs(yPos() - b.yPos());
  double diagCost = 1.41421356237;
  return (dx + dy) + (diagCost - 2) * std::min(dx, dy);
}

double Cell::angle() const {
  return atan2(y(), x());
}

Cell Cell::getNeighborFromYaw(double yaw) const {
  // Returns the neighboring cell in the yaw direction
  // E.g. if yaw == PI/4, then it returns Cell(x+1, y+1, z)
  int xDiff = 2 * scale * std::cos(yaw);
  int yDiff = 2 * scale * std::sin(yaw);
  // ROS_INFO("parent: %d %d \n", xDiff, yDiff);
  return Cell(xPos() + xDiff, yPos() + yDiff, zPos());
}

std::vector<Cell> Cell::getFlowNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int,int,int>(x() + 1, y(), z())),  
                           Cell(std::tuple<int,int,int>(x() - 1, y(), z())),  
                           Cell(std::tuple<int,int,int>(x(), y() + 1, z())),  
                           Cell(std::tuple<int,int,int>(x(), y() - 1, z())),  
                           Cell(std::tuple<int,int,int>(x(), y(), z() + 1)),  
                           Cell(std::tuple<int,int,int>(x(), y(), z() - 1))
                          };
}

std::vector<Cell> Cell::getDiagonalNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int,int,int>(x() + 1, y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x() + 1, y() - 1, z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y() - 1, z()))
                          };
}

std::string Cell::asString() const {  
  std::string s = "(" + std::to_string(x()) + "," + std::to_string(y()) + "," + std::to_string(z()) + ")";
  return s;
}

} // namespace rotors_control
