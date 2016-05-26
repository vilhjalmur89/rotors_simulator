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

#ifndef INCLUDE_ROTORS_CONTROL_COMMON_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_H_

#include <glog/logging.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>
 

#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMotorSpeedTopic = "command/motor_speed";
static const std::string kDefaultCommandTrajectoryTopic = "command/trajectory";
static const std::string kDefaultCommandRollPitchYawrateThrustTopic = "command/roll_pitch_yawrate_thrust";
static const std::string kDefaultImuTopic = "imu";
static const std::string kDefaultOdometryTopic = "odometry";

struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};

void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                          EigenOdometry* odometry) {
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
                               Eigen::Matrix4Xd* allocation_matrix) {
  CHECK_NOTNULL(allocation_matrix);
  allocation_matrix->resize(4, rotor_configuration.rotors.size());
  unsigned int i = 0;
  for (const Rotor& rotor : rotor_configuration.rotors) {
    // Set first row of allocation matrix.
    (*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set second row of allocation matrix.
    (*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set third row of allocation matrix.
    (*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant
                                 * rotor.rotor_moment_constant;
    // Set forth row of allocation matrix.
    (*allocation_matrix)(3, i) = rotor.rotor_force_constant;
    ++i;
  }
}

void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
  *skew_matrix << 0, -vector.z(), vector.y(),
                  vector.z(), 0, -vector.x(),
                  -vector.y(), vector.x(), 0;
}

void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
  *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}



// GLOBAL PLANNER

template <typename T>
double squared(T x) {
  return x * x;
}

double distance(geometry_msgs::PoseStamped & a, geometry_msgs::PoseStamped & b) {
  double diffX = a.pose.position.x - b.pose.position.x;
  double diffY = a.pose.position.y - b.pose.position.y;
  double diffZ = a.pose.position.z - b.pose.position.z;
  return sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
}

double interpolate(double start, double end, double ratio) {
  return start + (end - start) * ratio;
}

double angleToRange(double angle) {
  // returns the angle in the range [-pi, pi]
  angle += M_PI;
  angle -= (2*M_PI) * std::floor( angle / (2*M_PI) );
  angle -= M_PI;
  return angle;
}

double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
  double xDiff = a[0] - b[0];
  double yDiff = a[1] - b[1];
  double zDiff = a[2] - b[2];
  return sqrt(squared(xDiff) + squared(yDiff) + squared(zDiff));
}

bool hasSameYawAndAltitude(const geometry_msgs::Pose& msg1,
                         const geometry_msgs::Pose& msg2) {

  return msg1.orientation.z == msg2.orientation.z
      && msg1.orientation.w == msg2.orientation.w
      && msg1.position.z == msg2.position.z;
}

double posterior(double p, double prior) {
  // p and prior are independent measurements of the same event
  double isObst = p * prior;
  double isNotObst = (1-p) * (1-prior);
  return isObst / (isObst + isNotObst+0.0001);
}

double pathLength(nav_msgs::Path & path) {
  // p and prior are independent measurements of the same event
  double totalDist = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    totalDist += distance(path.poses[i-1], path.poses[i]);
  }
  return totalDist;
}

double pathKineticEnergy(nav_msgs::Path & path) {
  // p and prior are independent measurements of the same event
  if (path.poses.size() < 3) {
    return 0.0;
  }
  std::vector<double> velX;
  std::vector<double> velY;
  std::vector<double> velZ;
  for (int i=1; i < path.poses.size(); ++i) {
    velX.push_back(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
    velY.push_back(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y);
    velZ.push_back(path.poses[i].pose.position.z - path.poses[i-1].pose.position.z);
  }

  double totalEnergy = 0.0;
  for (int i=1; i < velX.size(); ++i) {
    totalEnergy += std::abs(velX[i]*velX[i] - velX[i-1]*velX[i-1]);
    totalEnergy += std::abs(velY[i]*velY[i] - velY[i-1]*velY[i-1]);
    totalEnergy += std::abs(velZ[i]*velZ[i] - velZ[i-1]*velZ[i-1]);
  }

  return totalEnergy;
}

double pathEnergy(nav_msgs::Path & path, double upPenalty) {
  // p and prior are independent measurements of the same event
  double totalEnergy = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    totalEnergy += distance(path.poses[i-1], path.poses[i]);
    double altitudeIncrease = path.poses[i].pose.position.z - path.poses[i-1].pose.position.z;
    totalEnergy += std::max(0.0, upPenalty * altitudeIncrease);
  }
  return totalEnergy;
}


}

#endif /* INCLUDE_ROTORS_CONTROL_COMMON_H_ */
