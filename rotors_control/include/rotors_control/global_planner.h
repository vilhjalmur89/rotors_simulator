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

#include <queue>
#include <map>
#include <algorithm>    // std::reverse
#include <math.h>       // abs

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

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

class GlobalPlanner {
 public:
  std::set< std::pair<int,int> > occupied;
  std::set< std::pair<int,int> > pathCells;
  Eigen::Vector3d position;
  double yaw;

  GlobalPlanner();
  ~GlobalPlanner();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  void getGlobalPath(std::vector<WaypointWithTime> & waypoints);
  bool FindPath(std::pair<int,int> s, std::pair<int,int> t, 
              std::vector< std::pair<int,int> > & path, 
              std::set< std::pair<int,int> > & occupied);

  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

 private:
  bool initialized_params_;
  bool controller_active_;
  static const int64_t kNanoSecondsInSecond = 1000000000;



  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
};
}

#endif // ROTORS_CONTROL_GLOBAL_PLANNER_H
