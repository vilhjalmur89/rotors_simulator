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

#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <math.h>

bool sim_running = false;
std::string inputFilePath;
ros::Publisher wp_pub;


static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

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

double between(double start, double end, int steps, int totalSteps) {
  return start + (end - start) * steps / totalSteps;
}

void increaseResolution(std::vector<WaypointWithTime> & waypoints, std::vector<WaypointWithTime> & newWaypoints, int factor) {
  newWaypoints.push_back(waypoints[0]);
  for (int i=1; i < waypoints.size(); ++i) {
    for (int j=1; j <= factor; ++j) {
      double t = waypoints[i].waiting_time;
      double x = waypoints[i-1].position[0] + ((waypoints[i].position[0] - waypoints[i-1].position[0]) * j / factor);
      double y = waypoints[i-1].position[1] + ((waypoints[i].position[1] - waypoints[i-1].position[1]) * j / factor);
      double z = waypoints[i-1].position[2] + ((waypoints[i].position[2] - waypoints[i-1].position[2]) * j / factor);
      double yaw = waypoints[i-1].yaw + ((waypoints[i].yaw - waypoints[i-1].yaw) * j / factor);
      newWaypoints.push_back(WaypointWithTime(t, x, y, z, yaw));
    }
  }
}

void increaseResolution(std::vector<WaypointWithTime> & waypoints, std::vector<WaypointWithTime> & newWaypoints, double minDist, double minRot, double minTime) { 
  newWaypoints.push_back(waypoints[0]);
  for (int i=1; i < waypoints.size(); ++i) {
    double diffX = std::abs(waypoints[i].position[0] - waypoints[i-1].position[0]);
    double diffY = std::abs(waypoints[i].position[1] - waypoints[i-1].position[1]);
    double diffZ = std::abs(waypoints[i].position[2] - waypoints[i-1].position[2]);
    double diffYaw = std::abs(waypoints[i].yaw - waypoints[i-1].yaw);
    int factor = (int) std::max( std::max(0.0, diffX / minDist), std::max(diffY / minDist, diffZ / minDist));
    factor = std::max(factor, (int) (diffYaw / minRot)) + 1;
    for (int j=1; j <= factor; ++j) {
      double t = std::max(minTime, (waypoints[i-1].waiting_time + waypoints[i].waiting_time) / (2*factor));
      double x = between(waypoints[i-1].position[0], waypoints[i].position[0], j, factor);
      double y = between(waypoints[i-1].position[1], waypoints[i].position[1], j, factor);
      double z = between(waypoints[i-1].position[2], waypoints[i].position[2], j, factor);
      double yaw = between(waypoints[i-1].yaw, waypoints[i].yaw, j, factor);
      // double yaw = waypoints[i-1].yaw;
      newWaypoints.push_back(WaypointWithTime(t, x, y, z, yaw));
    }
  }
}

void getWaypoints(std::vector<WaypointWithTime> & waypoints) {

  const float DEG_2_RAD = M_PI / 180.0;

  std::ifstream wp_file(inputFilePath.c_str());

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> t >> x >> y >> z >> yaw) {
      waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
    }
    wp_file.close();

    std::vector<WaypointWithTime> newWaypoints;
    // increaseResolution(waypoints, newWaypoints, 1);
    increaseResolution(waypoints, newWaypoints, 0.3, 0.3, 0.1);
    waypoints = newWaypoints;

    ROS_INFO("Read %d waypoints.", (int )waypoints.size());
  }

  else {
    ROS_ERROR_STREAM("Unable to open poses file: " << inputFilePath);
  }
}

void getEndpoints(std::vector<WaypointWithTime> & waypoints) {

  const float DEG_2_RAD = M_PI / 180.0;

  std::ifstream wp_file(inputFilePath.c_str());

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    
    // Start point
    wp_file >> t >> x >> y >> z >> yaw;
    waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
    // End point
    wp_file >> t >> x >> y >> z >> yaw;
    waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));

    wp_file.close();

    ROS_INFO("Read %d End points.", (int )waypoints.size());
  }
}

void sendGlopalPath() {
  std::vector<WaypointWithTime> waypoints;
  getWaypoints(waypoints);

  ROS_INFO("Start publishing endpoints.");

  trajectory_msgs::MultiDOFJointTrajectory msg;
  msg.header.stamp = ros::Time::now();
  msg.points.resize(waypoints.size());
  msg.joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg.points[i]);
    ROS_INFO("  TrajectoryTime: %lu", trajectory_point.time_from_start_ns);
  }

  wp_pub.publish(msg);
  printf("\n    Published \n");

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2 && args.size() != 3) {
    ROS_ERROR("Usage: waypoint_publisher <waypoint_file>"
        "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
    return -1;
  }

  inputFilePath = args.at(1);



  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  // wp_pub =
  //     nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
  //     mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "GlobalPlannerTopic", 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 8s such that everything can settle and the mav flies to the initial position.
  ros::Duration(8).sleep();

  for (int i=0; i < 100; ++i) {
    sendGlopalPath();
    ros::Duration(1).sleep();
  }




  // wp_pub.publish(msg);
  // printf("\n    Published2 \n");



  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
